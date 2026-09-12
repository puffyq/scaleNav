#!/usr/bin/env bash
set -Eeuo pipefail

# Route-YOPO + ordered bubble MPC one-way test.
# Edit the values in this section before running; command-line overrides are
# intentionally not used so each test configuration is recorded in one place.

# Planner and model
MODEL="/mnt/code/lab/yopo/OpenSeek/scalenav_ws/src/models/original_yopo_simple/model.pt"
MAXIMUM_SPEED_MPS="6.0"
ORDERED_BUBBLE_MPC="true"
SEMANTIC="false"
REPLAN_RATE_HZ="1.0"
MINIMUM_TRAJECTORY_HOLD_SECONDS="1.0"
PROMPT="tree, blocks, wall"

# Number of one-way missions. Set to 0 to continue until Ctrl-C.
TEST_COUNT=1
MISSION_TIMEOUT_SECONDS=90
STARTUP_TIMEOUT_SECONDS=60
COOLDOWN_SECONDS=3

# Mission start and goal (world_enu, metres)
START_X=0.0
START_Y=0.0
START_Z=1.6
GOAL_X=0.0
GOAL_Y=140.0
GOAL_Z=1.6

# Mission acceptance and safety checks
START_TOLERANCE_M=0.5
POSITION_TOLERANCE_M=0.5
SPEED_TOLERANCE_MPS=0.3
MINIMUM_FLIGHT_ALTITUDE_M=0.5

# AirSim connection
AIRSIM_HOST="127.0.0.1"
AIRSIM_PORT=41451
AIRSIM_TIMEOUT_SECONDS=5
RESET_SETTLE_SECONDS=2

# Output paths
PROJECT_ROOT="/mnt/code/lab/yopo/OpenSeek"
LOG_ROOT="$PROJECT_ROOT/log_scalenav_train"
RESULTS_ROOT="$PROJECT_ROOT/scalenav_ws/src/aut_test/results/route_yopo_mpc"

# Runtime switches
IGNORE_COLLISION="false"
GRAPH_FIXED_LAYER="true"
DEVICE="cuda"

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "ROS2 Humble not found: /opt/ros/humble/setup.bash" >&2
  exit 1
}
[[ -f "$WS/install/setup.bash" ]] || {
  echo "ScaleNav is not built: $WS/install/setup.bash" >&2
  exit 1
}
[[ -f "$MODEL" ]] || {
  echo "YOPO model not found: $MODEL" >&2
  exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export ROUTE_YOPO_MODEL="$MODEL"
export ROUTE_YOPO_MAX_SPEED="$MAXIMUM_SPEED_MPS"
export ROUTE_YOPO_ORDERED_BUBBLE_MPC="$ORDERED_BUBBLE_MPC"
export ROUTE_YOPO_REPLAN_RATE="$REPLAN_RATE_HZ"
export MINIMUM_TRAJECTORY_HOLD="$MINIMUM_TRAJECTORY_HOLD_SECONDS"
export PROMPT
export DEVICE
export IGNORE_COLLISION
export GRAPH_FIXED_LAYER

ARGS=(
  --stack route_yopo
  --count "$TEST_COUNT"
  --timeout "$MISSION_TIMEOUT_SECONDS"
  --startup-timeout "$STARTUP_TIMEOUT_SECONDS"
  --cooldown "$COOLDOWN_SECONDS"
  --start-x "$START_X"
  --start-y "$START_Y"
  --start-z "$START_Z"
  --goal-x "$GOAL_X"
  --goal-y "$GOAL_Y"
  --goal-z "$GOAL_Z"
  --start-tolerance "$START_TOLERANCE_M"
  --position-tolerance "$POSITION_TOLERANCE_M"
  --speed-tolerance "$SPEED_TOLERANCE_MPS"
  --minimum-flight-altitude "$MINIMUM_FLIGHT_ALTITUDE_M"
  --airsim-host "$AIRSIM_HOST"
  --airsim-port "$AIRSIM_PORT"
  --airsim-timeout "$AIRSIM_TIMEOUT_SECONDS"
  --reset-settle "$RESET_SETTLE_SECONDS"
  --log-root "$LOG_ROOT"
  --results-root "$RESULTS_ROOT"
)

if [[ "$SEMANTIC" != "true" && "$SEMANTIC" != "false" ]]; then
  echo "SEMANTIC must be true or false" >&2
  exit 2
fi
if [[ "$SEMANTIC" == "false" ]]; then
  ARGS+=(--no-semantic)
fi

mkdir -p "$LOG_ROOT" "$RESULTS_ROOT"
echo "Starting Route-YOPO + ordered bubble MPC"
echo "model=$MODEL"
echo "max_speed=${MAXIMUM_SPEED_MPS}m/s mpc=$ORDERED_BUBBLE_MPC semantic=$SEMANTIC"
echo "missions=$TEST_COUNT goal=($GOAL_X, $GOAL_Y, $GOAL_Z)"
echo "log_root=$LOG_ROOT"
echo "results_root=$RESULTS_ROOT"

exec /usr/bin/python3 "$SCRIPT_DIR/run_repeated_test.py" "${ARGS[@]}"
