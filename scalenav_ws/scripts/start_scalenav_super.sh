#!/usr/bin/env bash
set -Eeuo pipefail

# ScaleNav semantic/topological route layer with SUPER execution.
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd -- "$WS/.." && pwd)"
ROOT_DIR="$PROJECT_ROOT/bc/third_party/compare"
PYTHON="$WS/../../YOPO-Rally/.venv/bin/python"
LOG_ROOT="${SCALENAV_LOG_DIR:-$WS/../log_scalenav}"
PROMPT="${PROMPT:-tree, blocks, wall}"
SEMANTIC_COST_WEIGHT="${SEMANTIC_COST_WEIGHT:-2.0}"
SEMANTIC_ROUTE_INFLUENCE_M="${SEMANTIC_ROUTE_INFLUENCE_M:-8.0}"
SEMANTIC_POINT_INFLUENCE_M="${SEMANTIC_POINT_INFLUENCE_M:-8.0}"
GRAPH_FIXED_LAYER="${GRAPH_FIXED_LAYER:-true}"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$ROOT_DIR/EGO-Planner/install/setup.bash"
source "$ROOT_DIR/SUPER/install/setup.bash"
source "$ROOT_DIR/baseline_adapters/install/setup.bash"
set -u
export PYTHONPATH="$WS/src/scalenav:$WS/src:${PYTHONPATH:-}"

PIDS=""
run() {
  setsid stdbuf -oL -eL "$@" &
  PIDS="$PIDS $!"
}
stop() {
  trap - EXIT INT TERM
  for pid in $PIDS; do
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 0.5
  for pid in $PIDS; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap stop EXIT INT TERM

run ros2 launch baseline_adapters super_map2.launch.py \
  output_dir:="$LOG_ROOT" goal_topic:=/goal_pose
run ros2 launch scalenav_graph_ros2 scalenav_graph.launch.py \
  graph_fixed_layer:="$GRAPH_FIXED_LAYER" \
  goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
  next_goal_frame:=world_enu visualization_frame:=world_enu \
  odom_twist_frame:=body semantic_heatmap_topic:=/scalenav/text_heatmap_raw \
  flight_statistics_file:=/dev/null graph_log_file:=/dev/null \
  semantic_cost_weight:="$SEMANTIC_COST_WEIGHT" \
  semantic_route_influence_m:="$SEMANTIC_ROUTE_INFLUENCE_M" \
  semantic_point_influence_m:="$SEMANTIC_POINT_INFLUENCE_M" \
  wait_for_initial_semantic:=true
run "$PYTHON" "$WS/src/scalenav/text_heatmap_ros2.py" \
  --prompt "$PROMPT" --input-topic /camera/color/image \
  --output-topic /scalenav/text_heatmap --device cuda --update-rate 2 \
  --pearl-root "$WS/src/global_graph/heatmap_ws/pearl_ws"
run ros2 run baseline_adapters scalenav_goal_bridge --ros-args \
  -p input_topic:=/scalenav/local_goal \
  -p output_topic:=/move_base_simple/goal \
  -p min_interval_s:=0.75 -p min_change_m:=0.75 -p frame_id:=world_enu \
  -p fixed_z:=1.6 -p max_z_error_m:=0.75

echo "started ScaleNav+SUPER; mission_goal=/goal_pose local_goal=/scalenav/local_goal executor_goal=/move_base_simple/goal prompt=$PROMPT"
wait -n $PIDS
