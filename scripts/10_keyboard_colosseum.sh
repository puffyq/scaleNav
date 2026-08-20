#!/usr/bin/env bash
set -Eeuo pipefail

COLOSSEUM_ROOT="${COLOSSEUM_ROOT:-/mnt/code/lab/airsim/Colosseum}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS_INSTALL="$COLOSSEUM_ROOT/ros2/${ROS_INSTALL_BASE:-install}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

[[ -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]] || {
  echo "错误: 未找到 ROS2: /opt/ros/$ROS_DISTRO_NAME/setup.bash" >&2
  exit 1
}
[[ -f "$ROS_INSTALL/setup.bash" ]] || {
  echo "错误: 请先运行 03_build_colosseum_ros2.sh" >&2
  exit 1
}

export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
set +u
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
source "$ROS_INSTALL/setup.bash"
set -u

exec /usr/bin/python3 "$SCRIPT_DIR/10_keyboard_colosseum.py" "$@"
