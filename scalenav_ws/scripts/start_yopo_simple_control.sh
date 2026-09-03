#!/usr/bin/env bash
set -Eeuo pipefail

# Standalone original YOPO-Simple control stack for repeated AirSim tests.
# Unlike start.sh, this intentionally omits the ScaleNav graph and semantic
# front end; the model consumes the mission goal directly on /goal_pose.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SRC="$WS/src"
ROOT="$WS/.."
PYTHON="$WS/../../YOPO-Rally/.venv/bin/python"
MODEL="$SRC/models/original_yopo_simple/model.pt"
CONFIG="$SRC/config/config.yaml"
LOG_ROOT="${SCALENAV_LOG_DIR:-$ROOT/log_scalenav}"
MAX_SPEED="${YOPO_SIMPLE_MAX_SPEED:-6.0}"
IGNORE_COLLISION="${IGNORE_COLLISION:-false}"

[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "ScaleNav is not built" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "Python not found: $PYTHON" >&2; exit 1; }
[[ -f "$MODEL" ]] || { echo "model not found: $MODEL" >&2; exit 1; }
[[ -f "$CONFIG" ]] || { echo "config not found: $CONFIG" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export PYTHONPATH="$SRC/scalenav:$SRC:${PYTHONPATH:-}"

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

run ros2 launch scalenav_log scalenav_log.launch.py output_dir:="$LOG_ROOT"
run ros2 launch airsim_renderer controller_airsim.launch.py \
  maximum_linear_speed:="$MAX_SPEED" ignore_collision:="$IGNORE_COLLISION"
run ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py
run "$PYTHON" "$SRC/scalenav/online_planner_ros2.py" \
  --model "$MODEL" --device cuda --config-file "$CONFIG" \
  --control --original-goal-input --goal-topic /goal_pose \
  --world-frame world_enu --odom-twist-frame body \
  --model-image-width 160 --model-image-height 96 --model-vertical-num 3 \
  --trajectory-speed-color-max-mps "$MAX_SPEED" \
  --fixed-altitude --plan-from-reference --disable-event-log

echo "started YOPO-Simple control; model=$MODEL max_speed=${MAX_SPEED}m/s log_root=$LOG_ROOT"
wait -n $PIDS
