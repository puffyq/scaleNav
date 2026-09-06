#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Default true-3D experiment configuration. Environment variables can replace
# these defaults; command-line arguments take final precedence.
STACK="${STACK:-scalenav}"
TRIAL_COUNT="${TRIAL_COUNT:-10}"
TIMEOUT_S="${TIMEOUT_S:-120}"
START_X="${START_X:-0.0}"
START_Y="${START_Y:-0.0}"
START_Z="${START_Z:-1.6}"
GOAL_X="${GOAL_X:-0.0}"
GOAL_Y="${GOAL_Y:-140.0}"
GOAL_Z="${GOAL_Z:-8.0}"
PROMPT="${PROMPT:-blocks, wall}"
SEMANTIC_INFLUENCE="${SEMANTIC_INFLUENCE:-1}"
SEMANTIC_COST_WEIGHT="${SEMANTIC_COST_WEIGHT:-2.0}"
SEMANTIC_ROUTE_INFLUENCE_M="${SEMANTIC_ROUTE_INFLUENCE_M:-5.0}"
SEMANTIC_POINT_INFLUENCE_M="${SEMANTIC_POINT_INFLUENCE_M:-5.0}"
COOLDOWN_S="${COOLDOWN_S:-3.0}"

# Both switches are required: the first preserves 3D graph coordinates and
# the second allows YOPO to execute vertical primitives.
GRAPH_FIXED_LAYER=false
FIXED_ALTITUDE=false

export STACK GRAPH_FIXED_LAYER FIXED_ALTITUDE SEMANTIC_INFLUENCE

exec "$SCRIPT_DIR/run_0_140_continuous.sh" \
  --count "$TRIAL_COUNT" \
  --timeout "$TIMEOUT_S" \
  --cooldown "$COOLDOWN_S" \
  --start-x "$START_X" \
  --start-y "$START_Y" \
  --start-z "$START_Z" \
  --goal-x "$GOAL_X" \
  --goal-y "$GOAL_Y" \
  --goal-z "$GOAL_Z" \
  --prompt "$PROMPT" \
  --semantic-cost-weight "$SEMANTIC_COST_WEIGHT" \
  --semantic-route-influence-m "$SEMANTIC_ROUTE_INFLUENCE_M" \
  --semantic-point-influence-m "$SEMANTIC_POINT_INFLUENCE_M" \
  "$@"
