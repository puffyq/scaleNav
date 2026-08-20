# OpenSeek UAV simulation

> 当前 Colosseum BlocksV2 运行链请使用仓库根目录 README 中的编号脚本：
>
> ```bash
> bash scripts/01_setup_colosseum_settings.sh
> bash scripts/02_build_colosseum.sh
> bash scripts/03_build_colosseum_ros2.sh
> bash scripts/04_build_blocks_v2.sh
> bash scripts/05_open_blocks_v2.sh
> bash scripts/06_start_colosseum_ros2.sh
> bash scripts/07_check_colosseum_rgbd.sh
> bash scripts/08_start_openseek_planner.sh
> ```
>
> 本文以下内容是旧 Unity endpoint 的历史参考，不适用于当前 Colosseum
> bridge。

The active simulation path is ROS2 Humble. The older ROS1 packages under
`controller/src/so3_control` and `controller/src/so3_quadrotor_simulator` are
kept as upstream references; `openseek_uav_sim` compiles the existing
`Quadrotor.cpp` dynamics directly and exposes a ROS2 interface.

`YOPO-Sim` did not contain a UAV prefab. OpenSeek imports the Apache-2.0
`uav.dae` model from the controller assets and builds an `OpenSeekUav` prefab
with a forward RGB-D camera and ROS2 odometry follower.

## Topic contract

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| Planner -> simulator | `/openseek/trajectory_point` | `trajectory_msgs/msg/MultiDOFJointTrajectoryPoint` | Position, velocity, acceleration and yaw setpoint |
| Debug -> simulator | `/openseek/position_cmd` | `geometry_msgs/msg/PoseStamped` | Low-level position/yaw setpoint |
| Simulator -> Unity/planner | `/sim/odom` | `nav_msgs/msg/Odometry` | Body pose and body-frame velocity |
| Simulator -> planner | `/sim/imu` | `sensor_msgs/msg/Imu` | Orientation, angular velocity and proper acceleration |
| Simulator -> diagnostics | `/sim/motor_rpm` | `std_msgs/msg/Float64MultiArray` | Four motor speeds |
| Unity -> simulator | `/sim/collision` | `std_msgs/msg/Bool` | Collision; `true` latches a hover stop |
| Safety -> simulator | `/openseek/emergency_stop` | `std_msgs/msg/Bool` | Explicit latched hover stop |
| User -> simulator | `/openseek/reset_sim` | `std_srvs/srv/Trigger` | Reset pose and clear the stop latch |

All world poses use ROS FLU coordinates in the `map` frame. Unity converts
between its left-handed coordinates and ROS FLU with the official
ROS-TCP-Connector geometry helpers.

## Setup and start

Run the Unity scene configuration once:

```bash
cd /mnt/code/lab/yopo/OpenSeek
bash scripts/01_setup_colosseum_settings.sh
```

If `YOPO-Sim` is already open, Unity will not allow a second batch-mode Editor.
Press `Ctrl+R` in the existing Editor, wait for compilation, then run
`OpenSeek > Configure UAV Simulation` from the Unity menu instead.

To open the correct UAV scene directly (and avoid Unity reopening the old
vehicle data-generation scene), use:

```bash
bash scripts/05_open_blocks_v2.sh
```

Start the ROS2 endpoint, SO3 controller and quadrotor dynamics:

```bash
bash scripts/06_start_colosseum_ros2.sh
```

Then open `YOPO-Sim/Assets/Scenes/EvaluationScene.unity`, enter Play mode, and
click Connect if the scene connection UI is not already connected. Test motion
from another terminal:

```bash
bash scripts/08_start_openseek_planner.sh
```

For interactive body-relative keyboard control, run a separate node:

```bash
bash scripts/08_start_openseek_planner.sh
```

Keys are `W/S` forward/backward, `A/D` left/right, `Q/E` yaw, `R/F` up/down,
and `T` to synchronize the command target with the current pose. The Unity
main camera follows the UAV in third-person view while the RGB-D sensor remains
fixed to the nose.

## RGB-D contract

The online planner and Unity use one fixed camera contract:

- resolution: `640x480`
- horizontal field of view: `90 degrees` (`73.7398 degrees` vertical)
- valid range: `0.05-20 m`
- depth topic: `/camera/depth/image`, `sensor_msgs/msg/Image`, `32FC1` meters
- RGB topic: `/camera/color/image`

The Unity depth panel uses a fixed absolute `0-20 m` grayscale and displays the
center-pixel distance. It does not normalize each frame. To inspect the ROS
messages numerically while Unity is in Play mode, run:

```bash
bash scripts/07_check_colosseum_rgbd.sh
```

The checker reports image encoding, camera FOV, center depth, percentiles,
range violations, and receive rate. Validate depth here before collecting data
or starting the online planner.

## Online planner

Start with inference-only mode. This subscribes to `/sim/odom` and
`/camera/depth/image`, uses a fixed forward search heatmap, and publishes the
selected local trajectory as `/openseek/planned_path`. It does not move the
UAV:

```bash
cd /mnt/code/lab/yopo/OpenSeek
bash scripts/start_online_planner.sh
```

The planner log reports the selected candidate, score, body-frame endpoint,
and average inference latency. In another terminal, verify output:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /openseek/planned_path
ros2 topic echo /openseek/planned_path --once
```

Only after candidate directions and endpoints are reasonable should closed
loop control be enabled:

```bash
CONTROL=1 bash scripts/start_online_planner.sh
```

Do not run a second control publisher at the same time as closed-loop planning;
both publish `/openseek/trajectory_point`.

Reset after a collision:

```bash
source /opt/ros/humble/setup.bash
source controller/install_ros2/setup.bash
ros2 service call /openseek/reset_sim std_srvs/srv/Trigger '{}'
```
