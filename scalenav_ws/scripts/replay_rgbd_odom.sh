#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$WS/.." && pwd)"
TRAIN_ROOT="$ROOT/train_scalenav"
PYTHON="${PYTHON:-$ROOT/../YOPO-Rally/.venv/bin/python}"
MODEL="${ROUTE_YOPO_MODEL:-$TRAIN_ROOT/saved_route_centerline_w01_train_large_001/YOPO_0/epoch12.pth}"
BAG=""
GOAL=()
RATE="1.0"
LOOP=0
RVIZ=1
DEVICE="${DEVICE:-auto}"

usage() {
  cat <<'EOF'
Usage: replay_rgbd_odom.sh BAG [options]

Replay a three-topic bag through depth projection, ScaleNav graph,
Route-YOPO and RViz. The goal is not recorded, so provide it explicitly.

Options:
  --goal X Y Z       Mission goal in world_enu (recommended)
  --model PATH       Route-YOPO checkpoint
  --device auto|cpu|cuda
  --rate FACTOR      ros2 bag play rate (default: 1.0)
  --loop             Loop the bag
  --no-rviz          Do not start RViz
EOF
}

if [[ $# -eq 0 || "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi
BAG="$1"; shift
while (($#)); do
  case "$1" in
    --goal) [[ $# -ge 4 ]] || { echo "--goal needs X Y Z" >&2; exit 2; }; GOAL=("$2" "$3" "$4"); shift 4 ;;
    --model) [[ $# -ge 2 ]] || { echo "--model needs a path" >&2; exit 2; }; MODEL="$2"; shift 2 ;;
    --device) [[ $# -ge 2 ]] || { echo "--device needs a value" >&2; exit 2; }; DEVICE="$2"; shift 2 ;;
    --rate) [[ $# -ge 2 ]] || { echo "--rate needs a value" >&2; exit 2; }; RATE="$2"; shift 2 ;;
    --loop) LOOP=1; shift ;;
    --no-rviz) RVIZ=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -d "$BAG" || -f "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run $WS/scripts/build.sh first" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "Python not found: $PYTHON" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "Route-YOPO checkpoint not found: $MODEL" >&2; exit 1; }

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export PYTHONPATH="$TRAIN_ROOT:$WS/src/scalenav:$WS/src:${PYTHONPATH:-}"

PIDS=()
stop() {
  trap - EXIT INT TERM
  local pid
  for pid in "${PIDS[@]:-}"; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
  sleep 0.5
  for pid in "${PIDS[@]:-}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap stop EXIT INT TERM
run() { setsid stdbuf -oL -eL "$@" & PIDS+=("$!"); }

run ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py \
  depth_topic:=/camera/depth/image camera_info_topic:=/camera/depth/camera_info \
  pointcloud_topic:=/depth/points free_ray_topic:=/depth/free_rays \
  output_frame:=base_link horizontal_fov_deg:=90.0 vertical_fov_deg:=60.0
run ros2 launch scalenav_graph_ros2 scalenav_graph.launch.py \
  graph_fixed_layer:=true goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
  next_goal_frame:=world_enu visualization_frame:=world_enu odom_twist_frame:=body \
  semantic_cost_weight:=0.0 flight_statistics_file:=/dev/null graph_log_file:=/dev/null
run "$PYTHON" "$WS/src/scalenav/route_yopo_control_ros2.py" \
  --model "$MODEL" --train-root "$TRAIN_ROOT" --device "$DEVICE" \
  --odom-topic /sim/odom --depth-topic /camera/depth/image \
  --path-topic /scalenav/path --graph-topic /scalenav/graph \
  --clearance-topic /scalenav/clearance --world-frame world_enu \
  --odom-twist-frame body
if ((RVIZ)); then run rviz2 -f world_enu -d "$WS/src/config/scalenav_graph.rviz"; fi

if ((${#GOAL[@]} == 3)); then
  run ros2 topic pub --rate 1 /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world_enu}, pose: {position: {x: ${GOAL[0]}, y: ${GOAL[1]}, z: ${GOAL[2]}}, orientation: {w: 1.0}}}"
else
  echo "No --goal supplied; graph and Route-YOPO will wait for /goal_pose." >&2
fi

sleep 1
play=(ros2 bag play "$BAG" --rate "$RATE")
((LOOP)) && play+=(--loop)
run "${play[@]}"
echo "Replaying $BAG (rate=$RATE, loop=$LOOP); RViz shows /scalenav/graph and Route-YOPO candidates."
wait "${PIDS[-1]}"
