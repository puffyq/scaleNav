#!/usr/bin/env bash
set -Eeuo pipefail

COLOSSEUM_ROOT="${COLOSSEUM_ROOT:-/mnt/code/lab/airsim/Colosseum}"
ROS_ROOT="$COLOSSEUM_ROOT/ros2"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"

[[ -f "$ROS_SETUP" ]] || { echo "错误: 未找到 ROS2: $ROS_SETUP" >&2; exit 1; }
[[ -d "$ROS_ROOT/src" ]] || { echo "错误: 找不到 Colosseum ROS2 源码: $ROS_ROOT/src" >&2; exit 1; }
command -v colcon >/dev/null || { echo "错误: 未找到 colcon" >&2; exit 1; }

export PATH="/usr/bin:/bin:$PATH"
set +u
source "$ROS_SETUP"
set -u
missing_packages=()
for package_name in geographic_msgs mavros_msgs tf2; do
  if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    missing_packages+=("$package_name")
  fi
done
if ((${#missing_packages[@]} > 0)); then
  echo "错误: 缺少 ROS2 依赖: ${missing_packages[*]}" >&2
  echo "请安装: sudo apt-get install ros-${ROS_DISTRO_NAME}-geographic-msgs ros-${ROS_DISTRO_NAME}-mavros-msgs ros-${ROS_DISTRO_NAME}-tf2" >&2
  exit 1
fi
cd -- "$ROS_ROOT"
colcon --log-base "${ROS_LOG_BASE:-log}" build \
  --base-paths src \
  --symlink-install \
  --build-base "${ROS_BUILD_BASE:-build}" \
  --install-base "${ROS_INSTALL_BASE:-install}"
[[ -f "${ROS_INSTALL_BASE:-install}/setup.bash" ]] || {
  echo "错误: colcon 完成但没有生成 $ROS_ROOT/${ROS_INSTALL_BASE:-install}/setup.bash" >&2
  exit 1
}
echo "Colosseum ROS2 已编译: $ROS_ROOT/install"
