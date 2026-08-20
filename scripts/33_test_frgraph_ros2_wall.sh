#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/openseek_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"
set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch openseek_frgraph_ros2 frgraph_gap_pipeline.launch.py \
  > /tmp/openseek_frgraph_ros2_wall.log 2>&1 &
launch_pid=$!

sleep 2
/usr/bin/python3 "${ROOT_DIR}/scripts/frgraph_ros2_wall_publisher.py"

echo "FRGraph ROS2 wall log: /tmp/openseek_frgraph_ros2_wall.log"
