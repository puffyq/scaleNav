#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
[[ -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]] || { echo "错误: 未找到 ROS2 Humble" >&2; exit 1; }
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
set +u
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
set -u
exec /usr/bin/python3 "$SCRIPT_DIR/check_rgbd_stream.py" "$@"
