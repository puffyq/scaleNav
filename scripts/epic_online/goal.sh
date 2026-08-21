#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
# GOAL_X="0.0"
# GOAL_Y="140.0"
# GOAL_Z="1.6"

# GOAL_X="-0.0"
# GOAL_Y="0.0"
# GOAL_Z="1.6"

GOAL_X="-80.0"
GOAL_Y="0.0"
GOAL_Z="1.6"

if (( $# >= 1 )); then GOAL_X="$1"; fi
if (( $# >= 2 )); then GOAL_Y="$2"; fi
if (( $# >= 3 )); then GOAL_Z="$3"; fi

ROS_HOME="${ROS_HOME:-/tmp/openseek_ros_home}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/openseek_ros_logs}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
set -u

echo "Publishing mission goal: (${GOAL_X}, ${GOAL_Y}, ${GOAL_Z}) world_enu"
message="{header: {frame_id: 'world_enu'}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}, orientation: {w: 1.0}}}"
set +e
env ROS_HOME="$ROS_HOME" ROS_LOG_DIR="$ROS_LOG_DIR" \
  timeout --signal=INT --kill-after=1 5 \
  ros2 topic pub --rate 2 /goal_pose geometry_msgs/msg/PoseStamped "$message"
status=$?
set -e
[[ "$status" == "0" || "$status" == "124" || "$status" == "130" ]] || exit "$status"
