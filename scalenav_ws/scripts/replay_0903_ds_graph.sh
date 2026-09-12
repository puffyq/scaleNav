#!/usr/bin/env bash
set -Eeuo pipefail

# All experiment settings live here so a replay is attributable and repeatable.
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$WS/.." && pwd)"
SRC="$WS/src"
DATA_DIR="$WS/data/0903data/0903data"
BAG="$DATA_DIR/omni_sync_20260902_101624.recovered_sync.bag"
RATE="1.0"
START="0"
DURATION="0"
DEVICE="auto"
RVIZ=0
RUN_YOPO=1
RUN_MPC=1
SEMANTIC=0
PROMPT="building"
SEMANTIC_RATE="2.0"
SEMANTIC_COST_WEIGHT="2.0"
SEMANTIC_DEVICE="cuda"
GRAPH_SAFE_DISTANCE="0.61"
RUN_GCN=1
GCN_MODEL="$ROOT/train_gcn/frontier_gcn_map2_35m.pt"
MODEL_MAX_DEPTH_M="20.0"
SENSOR_MAX_DISTANCE_M="50.0"
CLOUD_STRIDE="1"
FREE_RAY_STRIDE="4"
HORIZONTAL_FOV="90.0"
VERTICAL_FOV="73.7398"
GRAPH_LAYER_Z="1.6"
GRAPH_FIXED_LAYER="false"
PRESERVE_ODOM_Z=1
SCENE_ENDPOINT=""
MODEL="$WS/src/models/original_yopo_simple/model.pt"
OUTPUT_ROOT="$WS/tmp/0903_replay"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --bag PATH                 ROS1 bag (default: 2 m/s bag)
  --rate FACTOR              Playback rate (default: 1.0)
  --start SEC                Start offset (default: 0)
  --duration SEC             Duration; 0 means complete bag (default: 0)
  --device auto|cpu|cuda     YOPO device (default: auto)
  --model-max-depth-m METERS  YOPO input depth clipping (default: 20)
  --sensor-max-distance-m M   q=0 far-plane distance (default: 50)
  --cloud-stride N            Occupied point pixel stride (default: 1)
  --free-ray-stride N         Far-plane ray pixel stride (default: 4)
  --flatten-altitude METERS   Replace real odom Z and use a fixed graph layer
  --output-dir PATH          Output root
  --rviz                     Start RViz
  --no-yopo                  Graph-only replay
  --no-mpc                   Run YOPO without ordered-bubble MPC
  --semantic                 Enable PEARL semantic heatmaps
  --prompt TEXT              PEARL text prompt (default: building)
  --scene-endpoint forest|building  Goal endpoint for the selected scene
  --semantic-rate HZ         PEARL update rate (default: 2)
  --semantic-device DEVICE   PEARL device (default: cuda; falls back to CPU)
  --semantic-cost-weight W   Graph semantic edge cost weight (default: 2)
  --graph-safe-distance M    Graph clearance margin (default: 0.61)
EOF
}

while (($#)); do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --start) START="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --device) DEVICE="$2"; shift 2 ;;
    --model-max-depth-m) MODEL_MAX_DEPTH_M="$2"; shift 2 ;;
    --sensor-max-distance-m) SENSOR_MAX_DISTANCE_M="$2"; shift 2 ;;
    --cloud-stride) CLOUD_STRIDE="$2"; shift 2 ;;
    --free-ray-stride) FREE_RAY_STRIDE="$2"; shift 2 ;;
    --flatten-altitude)
      GRAPH_LAYER_Z="$2"; GRAPH_FIXED_LAYER="true"; PRESERVE_ODOM_Z=0; shift 2 ;;
    --output-dir) OUTPUT_ROOT="$2"; shift 2 ;;
    --rviz) RVIZ=1; shift ;;
    --no-yopo) RUN_YOPO=0; RUN_MPC=0; shift ;;
    --no-mpc) RUN_MPC=0; shift ;;
    --semantic) SEMANTIC=1; shift ;;
    --prompt) PROMPT="$2"; shift 2 ;;
    --scene-endpoint) SCENE_ENDPOINT="$2"; shift 2 ;;
    --semantic-rate) SEMANTIC_RATE="$2"; shift 2 ;;
    --semantic-device) SEMANTIC_DEVICE="$2"; shift 2 ;;
    --semantic-cost-weight) SEMANTIC_COST_WEIGHT="$2"; shift 2 ;;
    --graph-safe-distance) GRAPH_SAFE_DISTANCE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run $WS/scripts/build.sh first" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "model not found: $MODEL" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID
SYSTEM_PYTHON=/usr/bin/python3
YOPO_PYTHON="$ROOT/../YOPO-Rally/.venv/bin/python"
ROSBAGS_SITE="$ROOT/../YOPO-Rally/.venv/lib/python3.10/site-packages"
export PYTHONPATH="$SRC/scalenav:$SRC:$ROSBAGS_SITE:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$ROOT/train_scalenav:$ROOT/train_gcn:${PYTHONPATH:-}"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$ROOT/../leap-c/external/acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROOT/../leap-c:$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH"

RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
RUN_DIR="$OUTPUT_ROOT/run_$RUN_ID"
mkdir -p "$RUN_DIR"
GRAPH_LOG="$RUN_DIR/graph_snapshots.jsonl"
FLIGHT_CSV="$RUN_DIR/flight_statistics.csv"
ODOM_CSV="$RUN_DIR/odom.csv"
COLLECTION="$RUN_DIR/replay_topics.json"
COLLECTOR_STOP="$RUN_DIR/collector.stop"
LOG="$RUN_DIR/replay.log"
exec > >(tee -a "$LOG") 2>&1

PIDS=()
COLLECTOR_PID=""
run() {
  setsid stdbuf -oL -eL "$@" &
  PIDS+=("$!")
}
cleanup() {
  trap - EXIT INT TERM
  local pid
  for pid in "${PIDS[@]:-}"; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

if ((SEMANTIC)); then
  run "$YOPO_PYTHON" "$SRC/scalenav/text_heatmap_ros2.py" \
    --prompt "$PROMPT" --input-topic /camera/color/image \
    --output-topic /scalenav/text_heatmap --device "$SEMANTIC_DEVICE" \
    --update-rate "$SEMANTIC_RATE" \
    --pearl-root "$SRC/global_graph/heatmap_ws/pearl_ws"
fi

run ros2 launch scalenav_graph_ros2 scalenav_graph.launch.py \
  graph_fixed_layer:="$GRAPH_FIXED_LAYER" graph_layer_z:="$GRAPH_LAYER_Z" \
  bubble_astar_safe_distance:="$GRAPH_SAFE_DISTANCE" \
  goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
  next_goal_frame:=world_enu visualization_frame:=world_enu odom_twist_frame:=world \
  semantic_heatmap_topic:=/scalenav/text_heatmap_raw \
  wait_for_initial_semantic:="$([[ "$SEMANTIC" == 1 ]] && echo true || echo false)" \
  semantic_cost_weight:="$([[ "$SEMANTIC" == 1 ]] && echo "$SEMANTIC_COST_WEIGHT" || echo 0.0)" \
  gcn_frontier_column_topic:=/scalenav/gcn_frontier_column \
  gcn_frontier_required:="$([[ "$RUN_GCN" == 1 ]] && echo true || echo false)" \
  flight_statistics_file:="$FLIGHT_CSV" graph_log_file:="$GRAPH_LOG"

if ((RUN_GCN)); then
  run "$SYSTEM_PYTHON" "$SRC/scalenav/gcn_frontier_policy_ros2.py" \
    --model "$GCN_MODEL" --device cuda \
    --odom-topic /sim/odom --graph-topic /scalenav/graph \
    --timing-topic /scalenav/timing --mission-goal-topic /goal_pose \
    --output-topic /scalenav/gcn_frontier_column --marker-topic /scalenav/gcn_selected
fi

if ((RUN_YOPO)); then
  mpc_args=()
  ((RUN_MPC)) && mpc_args+=(--ordered-bubble-mpc)
  run "$YOPO_PYTHON" "$SRC/scalenav/route_yopo_control_ros2.py" \
    --model "$MODEL" --train-root "$ROOT/train_scalenav" --device "$DEVICE" \
    --odom-topic /sim/odom --depth-topic /camera/depth/image \
    --path-topic /scalenav/path --graph-topic /scalenav/graph \
    --bubble-topic /scalenav/bubbles --clearance-topic /scalenav/clearance \
    --world-frame world_enu --odom-twist-frame world --maximum-speed 6.0 \
    --source-horizontal-fov "$HORIZONTAL_FOV" --source-vertical-fov "$VERTICAL_FOV" \
    "${mpc_args[@]}"
fi
if ((RVIZ)); then
  run rviz2 -f world_enu -d "$WS/src/config/scalenav_graph.rviz"
fi
run "$SYSTEM_PYTHON" "$SRC/scalenav/collect_replay_graph_ros2.py" "$COLLECTION" \
  --stop-file "$COLLECTOR_STOP"
COLLECTOR_PID="${PIDS[-1]}"

sleep 2
echo "replay run=$RUN_DIR"
echo "DS RGB=1728x1728 calibrated; depth=512x512 calibrated; output FOV=${HORIZONTAL_FOV}x${VERTICAL_FOV} deg"
echo "depth: d=1/(q/255*0.07812003+0.0166666667)-10; q=0 is ${SENSOR_MAX_DISTANCE_M}m far-plane"
echo "graph: safe_distance=${GRAPH_SAFE_DISTANCE}m semantic=$SEMANTIC prompt='$PROMPT'"
if [[ -z "$SCENE_ENDPOINT" ]]; then
  case "${PROMPT,,}" in
    tree|trees|forest|woods) SCENE_ENDPOINT="forest" ;;
    building|buildings) SCENE_ENDPOINT="building" ;;
  esac
fi
echo "scene endpoint: ${SCENE_ENDPOINT:-global final odometry}"
if ((PRESERVE_ODOM_Z)); then
  echo "odometry: preserve recorded Z; normalize horizontal origin only"
else
  echo "odometry: flatten Z to ${GRAPH_LAYER_Z}m (explicit override)"
fi

set +e
altitude_args=(--preserve-odom-z)
if ((!PRESERVE_ODOM_Z)); then altitude_args=(--fixed-altitude "$GRAPH_LAYER_Z"); fi
goal_args=(--goal-from-final-odom)
if [[ -n "$SCENE_ENDPOINT" ]]; then
  goal_args=(--goal-from-scene-endpoint --scene-endpoint "$SCENE_ENDPOINT")
fi
"$SYSTEM_PYTHON" "$SRC/scalenav/replay_ros1_ds_bag_ros2.py" "$BAG" \
  --rate "$RATE" --start "$START" --duration "$DURATION" \
  --horizontal-fov "$HORIZONTAL_FOV" --vertical-fov "$VERTICAL_FOV" \
  --model-max-depth-m "$MODEL_MAX_DEPTH_M" \
  --sensor-max-distance-m "$SENSOR_MAX_DISTANCE_M" \
  --cloud-stride "$CLOUD_STRIDE" --free-ray-stride "$FREE_RAY_STRIDE" \
  "${altitude_args[@]}" "${goal_args[@]}" --odom-csv "$ODOM_CSV" \
  --preview-dir "$RUN_DIR"
REPLAY_STATUS=$?
set -e

touch "$COLLECTOR_STOP"
for _ in {1..100}; do
  [[ -f "$COLLECTION" ]] && break
  sleep 0.1
done
if [[ -f "$COLLECTION" ]]; then
  PYTHONPATH="$SRC" "$SYSTEM_PYTHON" "$SRC/scalenav/make_real_bag_graph_report.py" "$COLLECTION" \
    --graph-log "$GRAPH_LOG" --odom-csv "$ODOM_CSV" --output-dir "$RUN_DIR" \
    --runtime-log "$LOG" \
    --playback-rate "$RATE" \
    --graph-safe-distance "$GRAPH_SAFE_DISTANCE" \
    --title "0903 real DS replay: RGB, PEARL and planning pipeline" \
    --prompt "$PROMPT" \
    --depth-note "recorded inverse depth d=1/(q/255*0.07812003+0.0166666667)-10 m; q=0 is ${SENSOR_MAX_DISTANCE_M} m far-plane"
  ln -sfn "run_$RUN_ID" "$OUTPUT_ROOT/latest"
  echo "planning snapshot: $RUN_DIR/planning_snapshot.png"
  echo "HTML report: $RUN_DIR/index.html"
else
  echo "collector did not produce $COLLECTION" >&2
  exit 1
fi
exit "$REPLAY_STATUS"
