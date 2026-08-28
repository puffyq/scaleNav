#!/usr/bin/env bash
set -Eeuo pipefail

x="0.0"
y="0.0"
z="1.6"

if (($# >= 1)); then x="$1"; fi
if (($# >= 2)); then y="$2"; fi
if (($# >= 3)); then z="$3"; fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo "ROS2 Humble not found" >&2; exit 1; }
set +u; source /opt/ros/humble/setup.bash; set -u
msg="{header: {frame_id: 'world_enu'}, pose: {position: {x: $x, y: $y, z: $z}, orientation: {w: 1.0}}}"
echo "goal=($x, $y, $z) frame=world_enu"
set +e
timeout --signal=INT --kill-after=1 5 ros2 topic pub --rate 2 /goal_pose geometry_msgs/msg/PoseStamped "$msg"
status=$?
set -e
[[ "$status" == 0 || "$status" == 124 || "$status" == 130 ]]
