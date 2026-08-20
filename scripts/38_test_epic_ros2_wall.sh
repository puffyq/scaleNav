#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_HOME="${ROS_HOME:-/tmp/openseek_ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/openseek_ros_logs}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

frgraph_pid=""
epic_pid=""
cleanup() {
  for pid in "${epic_pid}" "${frgraph_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

ros2 launch openseek_frgraph_ros2 frgraph_gap_pipeline.launch.py \
  > /tmp/openseek_frgraph_ros2_wall.log 2>&1 &
frgraph_pid=$!
ros2 launch openseek_epic_ros2 epic_graph.launch.py \
  > /tmp/openseek_epic_ros2_wall.log 2>&1 &
epic_pid=$!
sleep 2

REQUIRE_EPIC_WALL=1 /usr/bin/python3 "${ROOT_DIR}/scripts/frgraph_ros2_wall_publisher.py"

echo
echo "EPIC timing summary:"
grep -E "EPIC timing|EPIC goal" /tmp/openseek_epic_ros2_wall.log | tail -12 || true
echo "EPIC log: /tmp/openseek_epic_ros2_wall.log"
