#!/usr/bin/env bash
set -Eeuo pipefail

# Edit only these three values for the selected UE map.
# GOAL_X="${GOAL_X:--140.0}"
# GOAL_Y="${GOAL_Y:-140.0}"
# GOAL_Z="${GOAL_Z:-2.0}"

GOAL_X="${GOAL_X:--60.0}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_Z="${GOAL_Z:-2.0}"


ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_HOME="${ROS_HOME:-/tmp/openseek_ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/openseek_ros_logs}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
set -u

echo "发布全局目标: (${GOAL_X}, ${GOAL_Y}, ${GOAL_Z}) world_enu，持续 5 秒"
message="{header: {frame_id: 'world_enu'}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}, orientation: {w: 1.0}}}"
set +e
timeout --signal=INT --kill-after=1 5 \
  ros2 topic pub --rate 2 /goal_pose geometry_msgs/msg/PoseStamped "$message"
status=$?
set -e
if [[ "$status" != "0" && "$status" != "124" && "$status" != "130" ]]; then
  exit "$status"
fi
