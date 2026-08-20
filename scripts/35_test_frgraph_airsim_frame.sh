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
cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -TERM "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch openseek_frgraph_ros2 frgraph_gap_pipeline.launch.py \
  > /tmp/openseek_frgraph_airsim_frame.log 2>&1 &
launch_pid=$!
sleep 2

/usr/bin/python3 "${ROOT_DIR}/scripts/frgraph_airsim_frame_publisher.py" \
  "${DATA_ROOT}" "${SCENE}" "${FRAME}" "${GOAL_DISTANCE}"

echo "C++ timing summary:"
if command -v rg >/dev/null 2>&1; then
  rg "FRGraph timing|expandNodePrimaryOnly total elapsed|Background expansion finished" \
    /tmp/openseek_frgraph_airsim_frame.log | tail -12 || true
else
  grep -E "FRGraph timing|expandNodePrimaryOnly total elapsed|Background expansion finished" \
    /tmp/openseek_frgraph_airsim_frame.log | tail -12 || true
fi
echo "AirSim frame FRGraph log: /tmp/openseek_frgraph_airsim_frame.log"
