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
  > /tmp/openseek_frgraph_airsim_frame.log 2>&1 &
frgraph_pid=$!
ros2 launch openseek_epic_ros2 epic_graph.launch.py \
  > /tmp/openseek_epic_airsim_frame.log 2>&1 &
epic_pid=$!
sleep 2

REQUIRE_EPIC=1 /usr/bin/python3 "${ROOT_DIR}/scripts/frgraph_airsim_frame_publisher.py" \
  "${DATA_ROOT}" "${SCENE}" "${FRAME}" "${GOAL_DISTANCE}"

minimum_incremental_updates="${EPIC_MIN_INCREMENTAL_UPDATES:-0}"
if (( minimum_incremental_updates > 0 )); then
  incremental_updates="$(grep -c '\[EPIC timing\]\[background incremental\]' \
    /tmp/openseek_epic_airsim_frame.log || true)"
  if (( incremental_updates < minimum_incremental_updates )); then
    echo "EPIC incremental update check failed: expected >=${minimum_incremental_updates}, got ${incremental_updates}" >&2
    exit 1
  fi
  if grep -q 'EPIC background rebuild failed' /tmp/openseek_epic_airsim_frame.log; then
    echo "EPIC incremental update check failed: background worker reported an error" >&2
    exit 1
  fi
  echo "EPIC incremental update check: ${incremental_updates} updates (required >=${minimum_incremental_updates})"
fi

echo
echo "EPIC timing summary:"
if command -v rg >/dev/null 2>&1; then
  rg "EPIC timing" /tmp/openseek_epic_airsim_frame.log | tail -12 || true
else
  grep -E "EPIC timing" /tmp/openseek_epic_airsim_frame.log | tail -12 || true
fi
echo "EPIC log: /tmp/openseek_epic_airsim_frame.log"
