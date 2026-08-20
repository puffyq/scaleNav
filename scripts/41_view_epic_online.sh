#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_HOME="${ROS_HOME:-/tmp/openseek_ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/openseek_ros_logs}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

exec rviz2 -f world_enu \
  -d "${ROOT_DIR}/install/openseek_epic_ros2/share/openseek_epic_ros2/config/epic_graph.rviz"
