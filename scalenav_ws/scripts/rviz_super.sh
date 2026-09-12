#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
SUPER_ROOT="$PROJECT_ROOT/bc/third_party/compare/SUPER"
RVIZ_CONFIG="$SUPER_ROOT/mars_uav_sim/perfect_drone_sim/rviz2/top_down.rviz"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "ROS2 Humble not found" >&2
  exit 1
}
[[ -f "$SUPER_ROOT/install/setup.bash" ]] || {
  echo "SUPER workspace is not built: $SUPER_ROOT/install/setup.bash" >&2
  exit 1
}
[[ -f "$RVIZ_CONFIG" ]] || {
  echo "SUPER RViz config not found: $RVIZ_CONFIG" >&2
  exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "$SUPER_ROOT/install/setup.bash"
set -u

exec rviz2 -f world_enu -d "$RVIZ_CONFIG" "$@"
