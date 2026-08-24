# ScaleNav controller + AirSim renderer

This ROS2 workspace keeps flight control outside AirSim:

```text
YOPO trajectory -> uav_sim (SO3 + rigid-body dynamics) -> /sim/odom
                                                               -> AirSim pose
AirSim external-physics vehicle -> synchronized RGB + DepthPlanar -> YOPO
```

AirSim does not generate odometry and no AirSim flight-control API is used.
`airsim_renderer` applies the only ENU/FLU to NED/FRD conversion. A
high-rate pose RPC stream drives the visible vehicle while a separate RGB-D RPC
stream obtains Scene and DepthPlanar frames. Both images carry the latest pose
timestamp. AirSim uses
`ExternalPhysicsEngine`, so it does not update the vehicle dynamics.

## Build

Install the runtime dependency, then build the workspace from the repository root:

```bash
cd /path/to/OpenSeek/scalenav_ws
sudo apt-get install python3-msgpack python3-numpy

source /opt/ros/humble/setup.bash
bash scripts/build.sh
```

The legacy ROS1 workspace under `controller/` contains `COLCON_IGNORE`, so a
root-level build discovers the ROS2 packages under `controller_airsim` only.

The renderer uses ROS2 Python and a small synchronous MessagePack-RPC transport.

## Run

Install the supplied external-physics settings before starting UE:

```bash
mkdir -p "$HOME/Documents/Colosseum"
cp src/controller_airsim/src/airsim_renderer/config/settings.json \
  "$HOME/Documents/Colosseum/settings.json"
```

Start BlocksV2 and press Play. Do not start the official Colosseum ROS bridge,
because this package owns the RGB-D topics.

```bash
bash scripts/start.sh
```

The script starts the YOPO online planner and launches the existing
`uav_sim` controller/dynamics node and renderer node. Do not also run
`06_start_colosseum_ros2.sh` or `09_start_colosseum_yopo_simple.sh`. The public
contract remains:

The planner uses reference-state continuation by default (`PLAN_FROM_REFERENCE=1`),
which is the mode intended for this position controller. Set it to `0` only when
testing direct measured-state replanning.

| Topic | Type | Direction |
| --- | --- | --- |
| `/scalenav/trajectory_point` | `trajectory_msgs/msg/MultiDOFJointTrajectoryPoint` | YOPO to controller |
| `/sim/odom` | `nav_msgs/msg/Odometry` | controller state |
| `/camera/color/image` | `sensor_msgs/msg/Image` (`bgr8`) | AirSim image |
| `/camera/depth/image` | `sensor_msgs/msg/Image` (`32FC1`, meters) | AirSim DepthPlanar |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Fixed camera calibration |
| `/depth/points` | `sensor_msgs/msg/PointCloud2` (current-frame body FLU XYZ) | DepthPlanar adapter output |
| `/sim/collision` | `std_msgs/msg/Bool` | AirSim diagnostic collision state |

`ignore_collision` defaults to `true`, so AirSim never changes the controller
pose. Consequently `/sim/collision` is diagnostic only; do not use it as the
sole swept-path collision test. Set `ignore_collision: false` temporarily when
validating collision behavior in a particular scene.

`airsim_origin_enu` is the controller-world ENU coordinate represented by the
AirSim local NED origin. Leave it at zero when both worlds share an origin.
Camera resolution and FOV in `settings.json` must match `renderer.yaml`.
