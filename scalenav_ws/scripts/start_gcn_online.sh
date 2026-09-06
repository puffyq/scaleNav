#!/usr/bin/env bash
set -Eeo pipefail

# Independent online GCN experiment. Existing start.sh/start_route_yopo.sh are
# unchanged. GCN replaces only the five-direction frontier choice; ScaleNav
# still runs its original A*, local-goal logic and YOPO execution pipeline.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$WS/.." && pwd)"
PYTHON="$ROOT/../YOPO-Rally/.venv/bin/python"
SRC="$WS/src"
LOG_ROOT="${SCALENAV_LOG_DIR:-$ROOT/log_scalenav_gcn}"
DEVICE="${DEVICE:-cuda}"
MAX_SPEED="${GCN_MAX_SPEED:-6.0}"
GRAPH_SAFE_DISTANCE="${GCN_SAFE_DISTANCE:-0.61}"
GCN_MODEL="${GCN_MODEL:-$ROOT/train_gcn/frontier_gcn_map2_35m.pt}"
GCN_SEMANTIC="${GCN_SEMANTIC:-1}"
SEMANTIC_PROMPT="${PROMPT:-blocks, wall}"
SEMANTIC_RATE="${GCN_SEMANTIC_RATE:-2}"
SEMANTIC_COST_WEIGHT="${SEMANTIC_COST_WEIGHT:-2.0}"
SEMANTIC_ROUTE_INFLUENCE_M="${SEMANTIC_ROUTE_INFLUENCE_M:-5.0}"
SEMANTIC_POINT_INFLUENCE_M="${SEMANTIC_POINT_INFLUENCE_M:-5.0}"
GRAPH_FIXED_LAYER="${GRAPH_FIXED_LAYER:-true}"
FIXED_ALTITUDE="${FIXED_ALTITUDE:-true}"

case "${1:-}" in
  -h|--help)
    echo "Usage: DEVICE=cuda GCN_MAX_SPEED=6 GCN_SAFE_DISTANCE=0.61 $0"
    echo "Publishes GCN frontier columns and runs the original ScaleNav/YOPO pipeline."
    exit 0
    ;;
  "") ;;
  *) echo "unknown option: $1 (use --help)" >&2; exit 2 ;;
esac

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run scalenav_ws/scripts/build.sh first" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "Python not found: $PYTHON" >&2; exit 1; }
[[ -f "$GCN_MODEL" ]] || { echo "GCN model missing: $GCN_MODEL" >&2; exit 1; }
[[ "$GCN_SEMANTIC" == "0" || "$GCN_SEMANTIC" == "1" ]] || {
  echo "GCN_SEMANTIC must be 0 or 1" >&2
  exit 2
}
[[ "$GRAPH_FIXED_LAYER" == "true" || "$GRAPH_FIXED_LAYER" == "false" ]] || {
  echo "GRAPH_FIXED_LAYER must be true or false" >&2
  exit 2
}
[[ "$FIXED_ALTITUDE" == "true" || "$FIXED_ALTITUDE" == "false" ]] || {
  echo "FIXED_ALTITUDE must be true or false" >&2
  exit 2
}
if pgrep -f 'online_planner_ros2.py|route_yopo_control_ros2.py' >/dev/null 2>&1; then
  echo "an existing online controller is running; stop start.sh/start_route_yopo.sh first" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PYTHONPATH="$SRC/scalenav:$SRC:$ROOT/train_gcn:${PYTHONPATH:-}"

PIDS=()
run() { setsid stdbuf -oL -eL "$@" & PIDS+=("$!"); }
stop() { trap - EXIT INT TERM; for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done; }
trap stop EXIT INT TERM

run ros2 launch scalenav_log scalenav_log.launch.py \
  output_dir:="$LOG_ROOT" gcn_frontier_column_topic:=/scalenav/gcn_frontier_column
run ros2 launch airsim_renderer controller_airsim.launch.py maximum_linear_speed:="$MAX_SPEED" ignore_collision:=false
run ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py
if [[ "$GCN_SEMANTIC" == "1" ]]; then
  run "$PYTHON" "$SRC/scalenav/text_heatmap_ros2.py" \
    --prompt "$SEMANTIC_PROMPT" --input-topic /camera/color/image \
    --output-topic /scalenav/text_heatmap --device "$DEVICE" \
    --update-rate "$SEMANTIC_RATE" --pearl-root "$SRC/global_graph/heatmap_ws/pearl_ws"
fi
run ros2 launch scalenav_graph_ros2 scalenav_graph.launch.py \
  graph_fixed_layer:="$GRAPH_FIXED_LAYER" \
  goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
  next_goal_frame:=world_enu visualization_frame:=world_enu \
  odom_twist_frame:=body wait_for_initial_semantic:=$( [[ "$GCN_SEMANTIC" == "1" ]] && echo true || echo false ) \
  bubble_astar_safe_distance:="$GRAPH_SAFE_DISTANCE" \
  semantic_points_enabled:=$( [[ "$GCN_SEMANTIC" == "1" ]] && echo true || echo false ) \
  semantic_cost_weight:="$SEMANTIC_COST_WEIGHT" \
  semantic_route_influence_m:="$SEMANTIC_ROUTE_INFLUENCE_M" \
  semantic_point_influence_m:="$SEMANTIC_POINT_INFLUENCE_M" \
  gcn_frontier_column_topic:=/scalenav/gcn_frontier_column \
  gcn_frontier_required:=true \
  flight_statistics_file:=/dev/null graph_log_file:=/dev/null

run "$PYTHON" "$SRC/scalenav/gcn_frontier_policy_ros2.py" \
  --model "$GCN_MODEL" --device "$DEVICE" \
  --mission-goal-topic /goal_pose --output-topic /scalenav/gcn_frontier_column
altitude_args=()
if [[ "$FIXED_ALTITUDE" == "true" ]]; then
  altitude_args+=(--fixed-altitude)
fi
run "$PYTHON" "$SRC/scalenav/online_planner_ros2.py" \
  --model "$SRC/models/original_yopo_simple/model.pt" --device "$DEVICE" \
  --config-file "$SRC/config/config.yaml" --control --original-goal-input \
  --goal-topic /scalenav/local_goal --mission-goal-topic /goal_pose \
  --world-frame world_enu --odom-twist-frame body "${altitude_args[@]}" \
  --plan-from-reference --disable-event-log \
  --maximum-trajectory-speed-mps "$MAX_SPEED" \
  --model-image-width 160 --model-image-height 96 --model-vertical-num 3

echo "started online GCN frontier selector (required); model=$GCN_MODEL column=/scalenav/gcn_frontier_column local_goal=/scalenav/local_goal device=$DEVICE pearl=$GCN_SEMANTIC prompt=$SEMANTIC_PROMPT safe_distance=${GRAPH_SAFE_DISTANCE}m graph_fixed_layer=$GRAPH_FIXED_LAYER fixed_altitude=$FIXED_ALTITUDE"
wait -n "${PIDS[@]}"
