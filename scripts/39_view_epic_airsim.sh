#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${DATA_ROOT:-${ROOT_DIR}/data/Map2GraphData}"
SCENE="${SCENE:-Scene_0002}"
FRAME="${FRAME:-0}"
GOAL_DISTANCE="${GOAL_DISTANCE:-20.0}"
export ROS_HOME="${ROS_HOME:-/tmp/openseek_ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/openseek_ros_logs}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u

frgraph_pid=""
epic_pid=""
publisher_pid=""
cleanup() {
  for pid in "${publisher_pid}" "${epic_pid}" "${frgraph_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

ros2 launch openseek_frgraph_ros2 frgraph_gap_pipeline.launch.py \
  > /tmp/openseek_frgraph_epic_view.log 2>&1 &
frgraph_pid=$!
ros2 launch openseek_epic_ros2 epic_graph.launch.py \
  > /tmp/openseek_epic_view.log 2>&1 &
epic_pid=$!
sleep 2

FRGRAPH_HOLD=1 REQUIRE_EPIC=1 /usr/bin/python3 \
  "${ROOT_DIR}/scripts/frgraph_airsim_frame_publisher.py" \
  "${DATA_ROOT}" "${SCENE}" "${FRAME}" "${GOAL_DISTANCE}" \
  > /tmp/openseek_epic_view_publisher.log 2>&1 &
publisher_pid=$!

sleep 2
echo "EPIC RViz 已启动"
echo "点云: /frgraph/points (Best Effort)"
echo "图:   /epic/graph   /epic/bubbles   /epic/path"
echo "Fixed Frame: odom"
rviz2 -d "${ROOT_DIR}/install/openseek_epic_ros2/share/openseek_epic_ros2/config/epic_graph.rviz"
