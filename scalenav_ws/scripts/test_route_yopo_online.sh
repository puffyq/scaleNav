#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROOT="$(cd -- "$WS/.." && pwd)"

# Route-YOPO online-test configuration.
MODEL="$ROOT/scalenav_ws/src/models/original_yopo_simple/model.pt"
TEST_COUNT=1
MISSION_TIMEOUT_SECONDS=90
STARTUP_TIMEOUT_SECONDS=60
COOLDOWN_SECONDS=3
START_X=0.0
START_Y=0.0
START_Z=1.6
GOAL_X=0.0
GOAL_Y=140.0
GOAL_Z=1.6
START_TOLERANCE_M=0.5
POSITION_TOLERANCE_M=0.5
SPEED_TOLERANCE_MPS=0.3
MINIMUM_FLIGHT_ALTITUDE_M=0.5
AIRSIM_HOST=127.0.0.1
AIRSIM_PORT=41451
AIRSIM_TIMEOUT_SECONDS=5
RESET_SETTLE_SECONDS=2
LOG_ROOT="$ROOT/log_scalenav"
RESULTS_ROOT="$WS/src/aut_test/results/route_yopo"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "ROS2 Humble not found: /opt/ros/humble/setup.bash" >&2
  exit 1
}
[[ -f "$WS/install/setup.bash" ]] || {
  echo "ScaleNav is not built: $WS/install/setup.bash" >&2
  exit 1
}
[[ -f "$MODEL" ]] || {
  echo "Route-YOPO checkpoint not found: $MODEL" >&2
  exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export ROUTE_YOPO_MODEL="$MODEL"
export ROUTE_YOPO_MAX_SPEED="${ROUTE_YOPO_MAX_SPEED:-6.0}"
export DEVICE=cuda
export IGNORE_COLLISION=false
export GRAPH_FIXED_LAYER=true

exec /usr/bin/python3 "$WS/src/aut_test/run_repeated_test.py" \
  --stack route_yopo \
  --count "$TEST_COUNT" \
  --timeout "$MISSION_TIMEOUT_SECONDS" \
  --startup-timeout "$STARTUP_TIMEOUT_SECONDS" \
  --cooldown "$COOLDOWN_SECONDS" \
  --start-x "$START_X" \
  --start-y "$START_Y" \
  --start-z "$START_Z" \
  --goal-x "$GOAL_X" \
  --goal-y "$GOAL_Y" \
  --goal-z "$GOAL_Z" \
  --start-tolerance "$START_TOLERANCE_M" \
  --position-tolerance "$POSITION_TOLERANCE_M" \
  --speed-tolerance "$SPEED_TOLERANCE_MPS" \
  --minimum-flight-altitude "$MINIMUM_FLIGHT_ALTITUDE_M" \
  --airsim-host "$AIRSIM_HOST" \
  --airsim-port "$AIRSIM_PORT" \
  --airsim-timeout "$AIRSIM_TIMEOUT_SECONDS" \
  --reset-settle "$RESET_SETTLE_SECONDS" \
  --log-root "$LOG_ROOT" \
  --results-root "$RESULTS_ROOT" \
  --no-semantic \
  "$@"
