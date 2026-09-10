#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PROJECT_ROOT="$(cd -- "$WS/.." && pwd)"

# Run a fixed number of valid flights by default; stop with Ctrl-C if needed.
# Set TRIAL_COUNT=0 for the previous unbounded mode. A command-line
# --count value passed after the defaults still takes precedence.
TRIAL_COUNT="${TRIAL_COUNT:-10}"
SEMANTIC_INFLUENCE="${SEMANTIC_INFLUENCE:-1}"
# This entry point exercises the online GCN stack by default. Set STACK=scalenav
# to run the original non-GCN planner for comparison.
STACK="${STACK:-gcn}"
GCN_MODEL="${GCN_2D_MODEL:-$PROJECT_ROOT/train_gcn/frontier_gcn_map2_35m.pt}"
START_Z="${START_Z_2D:-1.6}"
GOAL_Z="${GOAL_Z_2D:-1.6}"
PROMPT="${PROMPT_2D:-blocks, walls, box}"
GRAPH_FIXED_LAYER=true
FIXED_ALTITUDE=true

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
  echo "2D Map2 GCN model is missing: $GCN_MODEL" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export GCN_MODEL GRAPH_FIXED_LAYER FIXED_ALTITUDE

RUN_ARGS=(--count "$TRIAL_COUNT")
RUN_ARGS+=(--stack "$STACK")
RUN_ARGS+=(--start-z "$START_Z" --goal-z "$GOAL_Z")
RUN_ARGS+=(--prompt "$PROMPT")
if [[ "$SEMANTIC_INFLUENCE" == "0" ]]; then
  RUN_ARGS+=(--no-semantic)
fi

if [[ "$STACK" == "gcn" ]]; then
  echo "2D Map2 GCN model: $GCN_MODEL"
fi
echo "2D mission: start=(0,0,$START_Z) goal=(0,140,$GOAL_Z) graph_fixed_layer=true fixed_altitude=true"

exec /usr/bin/python3 "$SCRIPT_DIR/run_repeated_test.py" "${RUN_ARGS[@]}" "$@"
