#!/usr/bin/env bash
set -Eeo pipefail

PROMPT="tree, blocks, wall"
SEMANTIC=1
SEMANTIC_COST_WEIGHT="2.0"
GRAPH_FIXED_LAYER="${GRAPH_FIXED_LAYER:-true}"
DEVICE="${DEVICE:-cuda}"
RATE="2"
REPLAN_RATE="${ROUTE_YOPO_REPLAN_RATE:-1.0}"
ATTACH=0
LOG_ROOT=""
IGNORE_COLLISION="${IGNORE_COLLISION:-false}"
MINIMUM_TRAJECTORY_HOLD="${MINIMUM_TRAJECTORY_HOLD:-1.0}"
MAXIMUM_SPEED="${ROUTE_YOPO_MAX_SPEED:-6.0}"
ORDERED_BUBBLE_MPC="${ROUTE_YOPO_ORDERED_BUBBLE_MPC:-true}"

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$WS/.." && pwd)"
SRC="$WS/src"
TRAIN_ROOT="$ROOT/train_scalenav"
PYTHON="$ROOT/../YOPO-Rally/.venv/bin/python"
# Original YOPO-Simple followed by route-bubble MPC post-processing.
MODEL="${ROUTE_YOPO_MODEL:-$ROOT/scalenav_ws/src/models/original_yopo_simple/model.pt}"
LOG_ROOT="${LOG_ROOT:-${SCALENAV_LOG_DIR:-$ROOT/log_scalenav}}"

while (($#)); do
  case "$1" in
    --attach) ATTACH=1; shift ;;
    --prompt)
      if (($# < 2)); then echo "--prompt needs a value" >&2; exit 2; fi
      PROMPT="$2"; shift 2
      ;;
    --rate)
      if (($# < 2)); then echo "--rate needs a value" >&2; exit 2; fi
      RATE="$2"; shift 2
      ;;
    --device)
      if (($# < 2)); then echo "--device needs auto, cpu, or cuda" >&2; exit 2; fi
      DEVICE="$2"; shift 2
      ;;
    --maximum-speed|--max-speed)
      if (($# < 2)); then echo "$1 needs a positive value in m/s" >&2; exit 2; fi
      MAXIMUM_SPEED="$2"; shift 2
      ;;
    --no-semantic) SEMANTIC=0; shift ;;
    -h|--help)
      echo "Usage: $0 [--attach] [--device auto|cpu|cuda] [--maximum-speed MPS] [--prompt TEXT] [--rate HZ] [--no-semantic]"
      echo ""
      echo "Default: start simulator, depth adapter, ScaleNav, semantic perception, and Route-YOPO control."
      echo "--attach: attach Route-YOPO control to an existing ROS2/ScaleNav session. Stop the old planner first."
      echo "--maximum-speed: controller/MPC speed limit in m/s (default: 6.0; env: ROUTE_YOPO_MAX_SPEED)."
      echo "Control output: /scalenav/trajectory_point at 50 Hz. This script does not invoke scripts/start.sh."
      exit 0
      ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run $SCRIPT_DIR/build.sh first" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "Python not found: $PYTHON" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "Route-YOPO checkpoint not found: $MODEL" >&2; exit 1; }

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$TRAIN_ROOT:$SRC/scalenav:$SRC:${PYTHONPATH:-}"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$ROOT/../leap-c/external/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROOT/../leap-c:$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"

# A second controller on the same topic is unsafe and makes the recorded run
# impossible to attribute. Require the existing planner stack to be stopped.
if pgrep -f 'online_planner_ros2.py.*--control' >/dev/null 2>&1; then
  echo "scalenav online planner is still running; stop start.sh before starting Route-YOPO" >&2
  exit 1
fi
for _ in {1..5}; do
  control_info="$(ros2 topic info /scalenav/trajectory_point --verbose 2>/dev/null || true)"
  if grep -Eq '^Publisher count: [1-9]' <<<"$control_info"; then
    echo "control topic already has a publisher; stop the existing planner before starting Route-YOPO" >&2
    printf '%s\n' "$control_info" >&2
    exit 1
  fi
  sleep 0.5
done

PIDS=""
LAST_PID=""
run() {
  setsid stdbuf -oL -eL "$@" &
  LAST_PID="$!"
  PIDS="$PIDS $LAST_PID"
}
stop() {
  trap - EXIT INT TERM
  local pid
  for pid in $PIDS; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
  sleep 0.5
  for pid in $PIDS; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap stop EXIT INT TERM

if ((ATTACH == 0)); then
  run ros2 launch scalenav_log scalenav_log.launch.py output_dir:="$LOG_ROOT"
  run ros2 launch airsim_renderer controller_airsim.launch.py \
    ignore_collision:="$IGNORE_COLLISION"
  run ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py
  run ros2 launch scalenav_graph_ros2 scalenav_graph.launch.py \
    graph_fixed_layer:="$GRAPH_FIXED_LAYER" \
    goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
    next_goal_frame:=world_enu visualization_frame:=world_enu \
    odom_twist_frame:=body semantic_heatmap_topic:=/scalenav/text_heatmap_raw \
    flight_statistics_file:=/dev/null graph_log_file:=/dev/null \
    semantic_cost_weight:="$SEMANTIC_COST_WEIGHT"

  if ((SEMANTIC)); then
    run "$PYTHON" "$SRC/scalenav/text_heatmap_ros2.py" \
      --prompt "$PROMPT" --input-topic /camera/color/image \
      --output-topic /scalenav/text_heatmap --device cuda \
      --update-rate "$RATE" --pearl-root "$SRC/global_graph/heatmap_ws/pearl_ws"
  fi
fi

MPC_ARGS=()
if [[ "$ORDERED_BUBBLE_MPC" == "true" || "$ORDERED_BUBBLE_MPC" == "1" ]]; then
  MPC_ARGS+=(--ordered-bubble-mpc)
fi
run "$PYTHON" "$SRC/scalenav/route_yopo_control_ros2.py" \
  --model "$MODEL" --train-root "$TRAIN_ROOT" --device "$DEVICE" \
  --odom-twist-frame body --world-frame world_enu \
  --update-rate "$REPLAN_RATE" \
  --minimum-trajectory-hold "$MINIMUM_TRAJECTORY_HOLD" \
  --maximum-speed "$MAXIMUM_SPEED" "${MPC_ARGS[@]}"

echo "started Route-YOPO control; model=$MODEL attach=$ATTACH replan=${REPLAN_RATE}Hz max_speed=${MAXIMUM_SPEED}m/s mpc=$ORDERED_BUBBLE_MPC control_output=/scalenav/trajectory_point@50Hz"
echo "status=/scalenav/route_yopo/status path=/scalenav/route_yopo/planned_path mpc_path=/scalenav/route_yopo/mpc_path mpc_bubbles=/scalenav/route_yopo/mpc_bubbles"
wait -n $PIDS
