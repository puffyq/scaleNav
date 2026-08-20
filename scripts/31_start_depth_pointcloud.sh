#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

exec ros2 run openseek_frgraph_ros2 depth_planar_to_pointcloud_node \
  --ros-args \
  --params-file "${ROOT_DIR}/install/openseek_frgraph_ros2/share/openseek_frgraph_ros2/config/depth_planar_to_pointcloud.yaml"
