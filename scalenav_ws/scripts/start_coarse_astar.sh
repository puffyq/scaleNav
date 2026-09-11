#!/usr/bin/env bash
set -Eeuo pipefail

# Real-time coarse 3D occupancy A* + YOPO. Live depth fills a rolling
# 20 m window of 0.5 m voxels; 3D A* replans every tick to a clipped
# local horizon. This is not a 2D slice and not an offline global map.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SRC="$WS/src"
ROOT="$WS/.."
PYTHON="$WS/../../YOPO-Rally/.venv/bin/python"
MODEL="$SRC/models/original_yopo_simple/model.pt"
CONFIG="$SRC/config/config.yaml"
LOG_ROOT="${SCALENAV_LOG_DIR:-$ROOT/log_scalenav}"
MAX_SPEED="${COARSE_ASTAR_MAX_SPEED:-6.0}"
IGNORE_COLLISION="${IGNORE_COLLISION:-false}"
RESOLUTION_M="${COARSE_ASTAR_RESOLUTION_M:-0.5}"
INFLATE_RADIUS_M="${COARSE_ASTAR_INFLATE_RADIUS_M:-0.61}"
LOCAL_RADIUS_M="${COARSE_ASTAR_LOCAL_RADIUS_M:-20.0}"

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
run ros2 launch scalenav_graph_ros2 coarse_grid_astar.launch.py \
  goal_topic:=/goal_pose next_goal_topic:=/scalenav/local_goal \
  resolution_m:="$RESOLUTION_M" inflate_radius_m:="$INFLATE_RADIUS_M"   local_radius_m:="$LOCAL_RADIUS_M"
run "$PYTHON" "$SRC/scalenav/online_planner_ros2.py" \
  --model "$MODEL" --device cuda --config-file "$CONFIG" \
  --control --original-goal-input --goal-topic /scalenav/local_goal \
  --mission-goal-topic /goal_pose --world-frame world_enu --odom-twist-frame body \
  --model-image-width 160 --model-image-height 96 --model-vertical-num 3 \
  --trajectory-speed-color-max-mps "$MAX_SPEED" \
  --plan-from-reference --disable-event-log

echo "started online coarse 3D A* + YOPO; resolution=${RESOLUTION_M}m inflate=${INFLATE_RADIUS_M}m local_radius=${LOCAL_RADIUS_M}m local_goal=/scalenav/local_goal occupancy=/scalenav/coarse_astar/occupancy path=/scalenav/coarse_astar/path log_root=$LOG_ROOT"
wait -n $PIDS
