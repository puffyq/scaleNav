#!/usr/bin/env bash
set -Eeuo pipefail

GOAL_X="${GOAL_X:-60.0}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_Z="${GOAL_Z:-0.05}"
GOAL_TOPIC="${GOAL_TOPIC:-/goal_pose}"
GOAL_FRAME="${GOAL_FRAME:-world_enu}"

[[ -f /opt/ros/humble/setup.bash ]] || { echo "错误: 未找到 ROS2 Humble" >&2; exit 1; }
export PATH="/usr/bin:/bin:$PATH"
set +u
source /opt/ros/humble/setup.bash
set -u

echo "发布 YOPO-Simple 目标: frame=$GOAL_FRAME xyz=($GOAL_X, $GOAL_Y, $GOAL_Z)"
exec ros2 topic pub --once "$GOAL_TOPIC" geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: '$GOAL_FRAME'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}, orientation: {w: 1.0}}}"
