#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

command -v colcon >/dev/null || {
  echo "错误: 未找到 colcon" >&2
  exit 1
}
[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "错误: 未找到 ROS2 Humble" >&2
  exit 1
}

export PATH="/usr/bin:/bin:$PATH"
set +u
source /opt/ros/humble/setup.bash
set -u

cd -- "$WORKSPACE_ROOT"
exec colcon --log-base log build \
  --base-paths src/controller_airsim/src src/global_graph/scalenav_graph_ros2 \
  --build-base build \
  --install-base install \
  --symlink-install \
  --packages-select \
    airsim_renderer \
    scalenav_graph_ros2 \
    depth2points_ros2 \
    uav_sim \
  "$@"
