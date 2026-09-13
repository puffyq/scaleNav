#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PROJECT_ROOT="$(cd -- "$WS/.." && pwd)"

# Run a fixed number of valid flights by default; stop with Ctrl-C if needed.
# Set TRIAL_COUNT=0 for the previous unbounded mode. A command-line
# --count value passed after the defaults still takes precedence.
TRIAL_COUNT="${TRIAL_COUNT:-10}"
MISSION_TIMEOUT_SECONDS="${MISSION_TIMEOUT_SECONDS:-0}"
SEMANTIC_INFLUENCE="${SEMANTIC_INFLUENCE:-1}"
SEMANTIC_DEPTH_MAX_M="${SEMANTIC_DEPTH_MAX_M:-35.0}"
SEMANTIC_VIRTUAL_DEPTH_M="${SEMANTIC_VIRTUAL_DEPTH_M:-35.0}"
SEMANTIC_VIRTUAL_FRONTIERS_ENABLED="${SEMANTIC_VIRTUAL_FRONTIERS_ENABLED:-false}"
STACK="${STACK:-scalenav}"
GCN_MODEL="${GCN_2D_MODEL:-$PROJECT_ROOT/train_gcn/frontier_gcn_map2_35m.pt}"
START_Z="${START_Z_2D:-1.6}"
GOAL_Z="${GOAL_Z_2D:-1.6}"
PROMPT="${PROMPT_2D:-blocks, walls, box}"
GOAL_X="${GOAL_X:--800.0}"
GOAL_Y="${GOAL_Y:-800.0}"
GRAPH_FIXED_LAYER=true
FIXED_ALTITUDE=true
PERSIST_ROUTE_LAYER="${PERSIST_ROUTE_LAYER:-false}"
if [[ -z "$PERSIST_ROUTE_LAYER" ]]; then
  if [[ "${LOCAL_SLIDING_GRAPH:-false}" == "true" ]]; then
    PERSIST_ROUTE_LAYER=false
  else
    PERSIST_ROUTE_LAYER=true
  fi
fi
case "$PERSIST_ROUTE_LAYER" in
  true) export LOCAL_SLIDING_GRAPH=false ;;
  false) export LOCAL_SLIDING_GRAPH=true ;;
  *) echo "PERSIST_ROUTE_LAYER must be true or false" >&2; exit 2 ;;
esac
export MAP_HISTORY_RADIUS_M="${MAP_HISTORY_RADIUS_M:-40.0}"
export LOCAL_SLIDING_GRAPH_RADIUS_M="${LOCAL_SLIDING_GRAPH_RADIUS_M:-$MAP_HISTORY_RADIUS_M}"

if [[ "$SEMANTIC_INFLUENCE" != "0" && "$SEMANTIC_INFLUENCE" != "1" ]]; then
  echo "SEMANTIC_INFLUENCE must be 0 or 1" >&2
  exit 2
fi
if [[ "$SEMANTIC_VIRTUAL_FRONTIERS_ENABLED" != "true" &&
      "$SEMANTIC_VIRTUAL_FRONTIERS_ENABLED" != "false" ]]; then
  echo "SEMANTIC_VIRTUAL_FRONTIERS_ENABLED must be true or false" >&2
  exit 2
fi

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "ROS2 Humble not found: /opt/ros/humble/setup.bash" >&2
  exit 1
}
[[ -f "$WS/install/setup.bash" ]] || {
  echo "ScaleNav is not built: $WS/install/setup.bash is missing" >&2
  exit 1
}
if [[ "$STACK" == "gcn" && ! -f "$GCN_MODEL" ]]; then
  echo "2D Map2 GCN model is missing: $GCN_MODEL" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export GCN_MODEL GRAPH_FIXED_LAYER FIXED_ALTITUDE
export SEMANTIC_DEPTH_MAX_M SEMANTIC_VIRTUAL_DEPTH_M
export SEMANTIC_VIRTUAL_FRONTIERS_ENABLED

RUN_ARGS=(--count "$TRIAL_COUNT")
RUN_ARGS+=(--stack "$STACK")
RUN_ARGS+=(--timeout "$MISSION_TIMEOUT_SECONDS")
RUN_ARGS+=(--goal-x "$GOAL_X" --goal-y "$GOAL_Y")
RUN_ARGS+=(--start-z "$START_Z" --goal-z "$GOAL_Z")
RUN_ARGS+=(--semantic-depth-max-m "$SEMANTIC_DEPTH_MAX_M")
RUN_ARGS+=(--semantic-virtual-depth-m "$SEMANTIC_VIRTUAL_DEPTH_M")
if [[ "$SEMANTIC_VIRTUAL_FRONTIERS_ENABLED" == "true" ]]; then
  RUN_ARGS+=(--semantic-virtual-frontiers)
fi
RUN_ARGS+=(--prompt "$PROMPT")
if [[ "$SEMANTIC_INFLUENCE" == "0" ]]; then
  RUN_ARGS+=(--no-semantic)
fi

if [[ "$STACK" == "gcn" ]]; then
  echo "2D Map2 GCN model: $GCN_MODEL"
fi
echo "2D mission: stack=$STACK start=(0,0,$START_Z) goal=($GOAL_X,$GOAL_Y,$GOAL_Z) timeout=${MISSION_TIMEOUT_SECONDS}s semantic_depth_max=${SEMANTIC_DEPTH_MAX_M}m semantic_virtual_depth=${SEMANTIC_VIRTUAL_DEPTH_M}m semantic_virtual_frontiers=${SEMANTIC_VIRTUAL_FRONTIERS_ENABLED} graph_fixed_layer=true fixed_altitude=true persist_route_layer=$PERSIST_ROUTE_LAYER map_history_radius=$MAP_HISTORY_RADIUS_M local_sliding_graph=$LOCAL_SLIDING_GRAPH"

exec /usr/bin/python3 "$SCRIPT_DIR/run_repeated_test.py" "${RUN_ARGS[@]}" "$@"
