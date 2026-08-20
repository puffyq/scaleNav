#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${DATA_ROOT:-${ROOT_DIR}/data/Map2GraphData}"
SCENE="${SCENE:-Scene_0002}"
FRAME="${FRAME:-0}"
GOAL_DISTANCE="${GOAL_DISTANCE:-20.0}"
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
  > /tmp/openseek_frgraph_airsim_view.log 2>&1 &
launch_pid=$!
sleep 2

FRGRAPH_HOLD=1 /usr/bin/python3 \
  "${ROOT_DIR}/scripts/frgraph_airsim_frame_publisher.py" \
  "${DATA_ROOT}" "${SCENE}" "${FRAME}" "${GOAL_DISTANCE}" \
  > /tmp/openseek_frgraph_airsim_view_publisher.log 2>&1 &
publisher_pid=$!

sleep 2
echo "RViz topics: /frgraph/points /frgraph/graph /frgraph/free_space"
echo "Fixed Frame: odom; press F after selecting a marker if the AirSim frame is outside the initial view"
echo "C++ timing summary:"
grep -E "FRGraph timing|expandNodePrimaryOnly total elapsed|Background expansion finished" \
  /tmp/openseek_frgraph_airsim_view.log | tail -10 || true
rviz2 -d "${ROOT_DIR}/config/frgraph_wall.rviz"
