#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PROJECT_ROOT="$(cd -- "$WS/.." && pwd)"

# Default true-3D experiment configuration. Environment variables can replace
# these defaults; command-line arguments take final precedence.
STACK="${STACK:-gcn}"
TRIAL_COUNT="${TRIAL_COUNT:-10}"
TIMEOUT_S="${TIMEOUT_S:-120}"
START_X="${START_X:-0.0}"
START_Y="${START_Y:-0.0}"
START_Z="${START_Z_3D:-3.5}"
GOAL_X="${GOAL_X:-0.0}"
GOAL_Y="${GOAL_Y:-140.0}"
GOAL_Z="${GOAL_Z_3D:-3.5}"
PROMPT="${PROMPT_3D:-line}"
case "${PROMPT,,}" in
  *tree*)
    GCN_MODEL="${GCN_3D_MODEL:-$PROJECT_ROOT/train_gcn/frontier_gcn_map4_3d_trees.pt}"
    ;;
  *)
    GCN_MODEL="${GCN_3D_MODEL:-$PROJECT_ROOT/train_gcn/frontier_gcn_map4_3d_line.pt}"
    ;;
esac
# 3-D uses the same semantic front end as 2-D. The prompt selects the PEARL
# text query and the checkpoint selects the matching 3-D frontier policy.
SEMANTIC_INFLUENCE="1"
SEMANTIC_COST_WEIGHT="${SEMANTIC_COST_WEIGHT:-2.0}"
SEMANTIC_ROUTE_INFLUENCE_M="${SEMANTIC_ROUTE_INFLUENCE_M:-5.0}"
SEMANTIC_POINT_INFLUENCE_M="${SEMANTIC_POINT_INFLUENCE_M:-5.0}"
COOLDOWN_S="${COOLDOWN_S:-3.0}"
PROMPT_TAG="${PROMPT//[^[:alnum:]]/_}"
SCALENAV_LOG_DIR="${SCALENAV_LOG_DIR:-$PROJECT_ROOT/log_scalenav_3d_${PROMPT_TAG}}"

# Both switches are required: the first preserves 3D graph coordinates and
# the second allows YOPO to execute vertical primitives.
GRAPH_FIXED_LAYER=false
FIXED_ALTITUDE=false

export STACK GRAPH_FIXED_LAYER FIXED_ALTITUDE SEMANTIC_INFLUENCE GCN_MODEL PROMPT START_Z GOAL_Z SCALENAV_LOG_DIR

if [[ "$SEMANTIC_INFLUENCE" != "0" && "$SEMANTIC_INFLUENCE" != "1" ]]; then
  echo "SEMANTIC_INFLUENCE must be 0 or 1" >&2
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
  echo "3D Map4 GCN model is missing: $GCN_MODEL" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

if [[ "$STACK" == "gcn" ]]; then
  echo "3D Map4 GCN model: $GCN_MODEL"
fi
echo "3D mission: start=($START_X,$START_Y,$START_Z) goal=($GOAL_X,$GOAL_Y,$GOAL_Z) prompt=$PROMPT graph_fixed_layer=false fixed_altitude=false"
echo "3D log_root=$SCALENAV_LOG_DIR"

RUN_ARGS=(
  --stack "$STACK"
  --count "$TRIAL_COUNT"
  --timeout "$TIMEOUT_S"
  --cooldown "$COOLDOWN_S"
  --start-x "$START_X"
  --start-y "$START_Y"
  --start-z "$START_Z"
  --goal-x "$GOAL_X"
  --goal-y "$GOAL_Y"
  --goal-z "$GOAL_Z"
  --prompt "$PROMPT"
  --semantic-cost-weight "$SEMANTIC_COST_WEIGHT"
  --semantic-route-influence-m "$SEMANTIC_ROUTE_INFLUENCE_M"
  --semantic-point-influence-m "$SEMANTIC_POINT_INFLUENCE_M"
  --log-root "$SCALENAV_LOG_DIR"
)
if [[ "$SEMANTIC_INFLUENCE" == "0" ]]; then
  RUN_ARGS+=(--no-semantic)
fi

exec /usr/bin/python3 "$SCRIPT_DIR/run_repeated_test.py" "${RUN_ARGS[@]}" "$@"
