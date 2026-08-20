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
publisher_pid=""
cleanup() {
  for pid in "${publisher_pid}" "${launch_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

ros2 launch openseek_frgraph_ros2 frgraph_gap_pipeline.launch.py \
  > /tmp/openseek_frgraph_ros2_wall_view.log 2>&1 &
launch_pid=$!

sleep 2
WALL_HOLD=1 /usr/bin/python3 "${ROOT_DIR}/scripts/frgraph_ros2_wall_publisher.py" &
publisher_pid=$!

sleep 1
echo "RViz Fixed Frame: odom"
echo "White: wall point cloud; blue/orange: graph; yellow: convex free space; purple: goal"
rviz2 -d "${ROOT_DIR}/config/frgraph_wall.rviz"
