# 终端1：先 UE BlocksV2 里按 Play（不要起官方 Colosseum ROS bridge）

# 终端2：仿真 + 渲染
source /opt/ros/humble/setup.bash
source /mnt/code/lab/yopo/OpenSeek/scalenav_ws/install/setup.bash
ros2 launch airsim_renderer controller_airsim.launch.py


source /opt/ros/humble/setup.bash
source /mnt/code/lab/yopo/OpenSeek/scalenav_ws/install/setup.bash
ros2 launch depth2points_ros2 depth_planar_to_pointcloud.launch.py


# 终端3：YOPO-Simple（目标直接吃 /goal_pose，不经 EPIC）
source /opt/ros/humble/setup.bash
source /mnt/code/lab/yopo/OpenSeek/scalenav_ws/install/setup.bash
export PYTHONPATH=/mnt/code/lab/yopo/OpenSeek/scalenav_ws/src/scalenav:$PYTHONPATH

/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  /mnt/code/lab/yopo/OpenSeek/scalenav_ws/src/scalenav/online_planner_ros2.py \
  --model /mnt/code/lab/yopo/OpenSeek/scalenav_ws/src/models/original_yopo_simple/model.pt \
  --device cuda \
  --control --original-goal-input \
  --goal-topic /goal_pose \
  --world-frame world_enu --odom-twist-frame body \
  --model-image-width 160 --model-image-height 96 --model-vertical-num 3 \
  --fixed-altitude --plan-from-reference --disable-event-log

# 终端4：发目标
# bash /mnt/code/lab/yopo/OpenSeek/scalenav_ws/scripts/goal.sh 0 140 1.6