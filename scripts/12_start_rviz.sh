#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
RVIZ_CONFIG="${RVIZ_CONFIG:-$PROJECT_ROOT/config/openseek_colosseum.rviz}"

[[ -f "$ROS_SETUP" ]] || { echo "错误: 未找到 ROS2: $ROS_SETUP" >&2; exit 1; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "错误: RViz 配置不存在: $RVIZ_CONFIG" >&2; exit 1; }

export PATH="/usr/bin:/bin:$PATH"
set +u
source "$ROS_SETUP"
set -u

echo "启动 RViz: $RVIZ_CONFIG"
exec rviz2 -d "$RVIZ_CONFIG" "$@"
