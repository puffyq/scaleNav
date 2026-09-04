#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

# Default batch size. Change this value to adjust runs without passing --count.
TEST_COUNT=10

# Semantic influence switch: 1=enabled, 0=disabled.
SEMANTIC_INFLUENCE="${SEMANTIC_INFLUENCE:-0}"

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

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

RUN_ARGS=(--count "$TEST_COUNT")
if [[ "$SEMANTIC_INFLUENCE" == "0" ]]; then
  RUN_ARGS+=(--no-semantic)
fi

exec /usr/bin/python3 "$SCRIPT_DIR/run_repeated_test.py" "${RUN_ARGS[@]}" "$@"
