#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CONFIG="$WS/install/scalenav_graph_ros2/share/scalenav_graph_ros2/config/epic_graph.rviz"
[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "run $SCRIPT_DIR/build.sh first" >&2; exit 1; }
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
exec rviz2 -f world_enu -d "$CONFIG" "$@"
