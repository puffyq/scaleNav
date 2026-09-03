# OpenSeek

OpenSeek is a platform-independent, open-vocabulary target-search system for a
specified region. The intended runtime combines a persistent global graph, a
frozen semantic vision-language frontend, and a fast depth-aware local
trajectory planner.

The repository was split from the useful parts of `YOPO-Simple`. It is now the
main algorithm workspace. The sibling `YOPO-Sim` Unity project remains the
RGB-D simulator and data source.

## Current status

The copied `text_tracker` implementation and the models under
`models/baseline_v0` are a reproducible baseline, not the final OpenSeek
architecture. A first depth-native sparse Graph now exists under
`openseek/graph`: it retains nodes and edges, distinguishes CERTIFIED,
UNVALIDATED, and INVALID topology, and emits a certified 3-D waypoint in
offline Map2 replay. It is not yet connected to the ROS2 runtime. The final
signed attraction/repulsion-map contract also remains future work.

Some internal Python class and file names retain the upstream name so existing
weights remain loadable. New modules and user-facing commands use OpenSeek.

## Repository layout

```text
OpenSeek/
|-- scalenav_ws/               canonical ROS2 workspace
|   `-- src/
|       |-- openseek/           planner, losses and baseline semantic fusion
|       |-- controller_airsim/  active ROS2 SO3 dynamics and AirSim renderer
|       |-- global_map/         EPIC, PEARL and map-side dependencies
|       `-- models/             inference weights and training initialization
|-- scripts/                   environment, data and baseline commands
|-- tools/                     HTML dataset and offline-result viewers
`-- docs/                      current architecture diagrams
```

The canonical workspace entry points are under `scalenav_ws/scripts`:

```bash
bash scalenav_ws/scripts/build.sh   # build ROS2 packages
bash scalenav_ws/scripts/start.sh   # start AirSim + EPIC + YOPO
bash scalenav_ws/scripts/goal.sh  # one-way paper goal: (0, 140, 1.6)
bash scalenav_ws/scripts/rviz.sh
```

### RGB-D/Odom bag replay

Record only the three sensor streams needed for deterministic planner replay:

```bash
bash scalenav_ws/scripts/record_rgbd_odom.sh \
  --output scalenav_ws/bags/run_001 --duration 60
```

The recorder stores `/sim/odom`, `/camera/color/image`, and
`/camera/depth/image` only. Replay the bag through the depth projector, graph,
Route-YOPO and RViz by supplying the mission goal in `world_enu`:

```bash
bash scalenav_ws/scripts/replay_rgbd_odom.sh scalenav_ws/bags/run_001 \
  --goal 0 40 1.6 --rate 1.0
```

Replay does not require AirSim. Since `CameraInfo` is deliberately absent from
the bag, the depth projector derives focal lengths from its configured 90°
horizontal and 60° vertical FOV. Use `--no-rviz` for headless runs and
`--loop` for repeated playback. The goal is not part of the sensor-only bag;
omit `--goal` only when publishing `/goal_pose` from another node.

The goal script accepts `x y z` in the `world_enu` frame. AirSim/UE must be
running before `start.sh`; run `build.sh` after source changes.

Not copied from the old workspace: Git history, virtual environments, caches,
historical checkpoints, ground-vehicle ROS runtime, GeoTIFF guidance tooling,
point-cloud depth renderer, hardware CAD files, and old demo media.

## External dependency

Keep Unity beside this repository:

```text
/mnt/code/lab/yopo/
|-- OpenSeek/
`-- YOPO-Sim/
```

`YOPO-Sim` is the Unity project. OpenSeek datasets are kept locally under
`OpenSeek/data/TrainingData` and `OpenSeek/data/TestingData`; the simulator is
only the RGB-D data source.

## Environment

Python is selected in this order: `PYTHON` supplied by the caller,
`OpenSeek/.venv/bin/python`, the active Conda environment, the existing sibling
`YOPO-Rally/.venv`, then `python3` on `PATH`. Candidates without the core
OpenCV, PyTorch, and TOML dependencies are skipped automatically. A separate
OpenSeek environment is therefore optional.

Use `08_start_openseek_planner.sh` after the Colosseum ROS2 stream is ready.
It currently defaults to the exported original YOPO-Simple model. The exact
NED/ENU and FRD/FLU interface contract is documented in
[`docs/colosseum_ros2_coordinates.md`](docs/colosseum_ros2_coordinates.md).

## Colosseum / AirSim UE5.7

The runtime uses the official ROS2 packages shipped inside the same Colosseum
checkout as the UE plugin. The defaults match this local layout:

```text
/mnt/code/lab/airsim/Colosseum
/mnt/code/lab/ue5_7/Linux_Unreal_Engine_5.7.1
```

Run these once, in order (the native build must finish before ROS2 and UE):

```bash
bash scripts/01_setup_colosseum_settings.sh
bash scripts/02_build_colosseum.sh
bash scripts/03_build_colosseum_ros2.sh
bash scripts/04_build_blocks_v2.sh
```

如果 `02_build_colosseum.sh` 提示 `ColosseumLib/src` 不完整，说明源码曾
被误删，先恢复源码再编译：

```bash
cd /mnt/code/lab/airsim/Colosseum
git restore --source=HEAD -- ColosseumLib/src
cd /mnt/code/lab/yopo/OpenSeek
bash scripts/02_build_colosseum.sh
```

Start UE and the official bridge separately:

```bash
bash scripts/05_open_blocks_v2.sh
# In UE: press Play.
bash scripts/06_start_colosseum_ros2.sh
```

Or use the combined lifecycle entry point. It starts UE, waits up to 180
seconds for the Colosseum RPC port, then starts the official ROS2 node. You
still need to press Play in the UE editor during that wait:

```bash
bash scripts/09_start_colosseum_yopo_simple.sh
```

To start the bridge and YOPO-Simple together in inference-only mode:

```bash
START_YOPO=1 YOPO_CONTROL=0 bash scripts/09_start_colosseum_yopo_simple.sh
```

To rebuild before launching or install the settings template in the same
command:

```bash
BUILD=1 SETUP_SETTINGS=1 bash scripts/09_start_colosseum_yopo_simple.sh
```

The official node is remapped by `06_start_colosseum_ros2.sh` to publish
`/sim/odom`, `/camera/color/image`, `/camera/depth/image`, and depth
`CameraInfo`. It accepts Colosseum's `VelCmd` command interface. Check the
streams and start YOPO-Simple in inference-only mode separately:

```bash
bash scripts/07_check_colosseum_rgbd.sh
CONTROL=0 bash scripts/08_start_openseek_planner.sh
```

The original model requires an ENU goal. Publish one after the planner starts:

```bash
GOAL_X=10 GOAL_Y=0 GOAL_Z=2 bash scripts/11_send_yopo_goal.sh
```

After the inference logs and `/openseek/planned_path` look correct, stop the
inference-only process and set `CONTROL=1` to publish world-ENU velocity
commands to the official Colosseum node:

```bash
CONTROL=1 bash scripts/08_start_openseek_planner.sh
```

For manual keyboard flight, stop the planner and run this in a separate
terminal after `06_start_colosseum_ros2.sh` is running:

```bash
bash scripts/10_keyboard_colosseum.sh
```

Keys: `W/S` forward/back, `A/D` left/right, `R/F` up/down, `Q/E` yaw,
Space hover, `X` quit. Each key command expires automatically after a short
timeout; releasing control or quitting sends zero velocity.

Every numbered script accepts path overrides such as
`UE_ROOT=/path/to/ue bash scripts/04_build_blocks_v2.sh`.

Paths can be overridden without editing scripts:

```bash
PYTHON="$CONDA_PREFIX/bin/python" \
bash scripts/08_start_openseek_planner.sh
```

## Next implementation boundary

The current frozen runtime interface is:

```text
Depth + raw PEARL heatmap + vehicle state + Graph 3-D waypoint
    -> local trajectories
```

The sparse Graph owns large-obstacle route choice and supplies the waypoint;
YOPO owns local dynamics and immediate avoidance. Physical obstacles remain in
Depth and the hard safety filter. RGB, Depth, and PEARL must use the same camera
calibration and timestamps. A signed semantic cost is not part of v1.

## EPIC online long-range goal

UE/AirSim, the SO3/controller planning process, and RViz run separately. The
official Colosseum ROS2 bridge must not run: the local SO3 node owns odometry
and the AirSim renderer owns RGB-D. Commands use absolute paths.

Terminal 1 opens UE; select Map2 or Map4 and press Play:

```bash
bash /mnt/code/lab/yopo/OpenSeek/scripts/05_open_blocks_v2.sh
```

Terminal 2 starts SO3 dynamics, the AirSim renderer, EPIC, and YOPO. It does not
open another UE instance:

```bash
bash /mnt/code/lab/yopo/OpenSeek/scripts/40_start_epic_online.sh
```

Terminal 3 opens the preconfigured RViz view:

```bash
bash /mnt/code/lab/yopo/OpenSeek/scripts/41_view_epic_online.sh
```

Terminal 4 publishes the configured global target:

```bash
bash /mnt/code/lab/yopo/OpenSeek/scripts/42_send_epic_goal.sh
```

The wrapper uses the conservative initial settings: global route/A* at 5 Hz,
Bubble skeleton rebuilding at 1 Hz, CUDA inference, and a 0.5 m local-goal
tolerance. The full global path is continuously published on `/epic/path`; the
magenta `/epic/yopo_goal` marker is the current next node. After the first stable
run, increase the two frequency constants in `scripts/40_start_epic_online.sh`.

When EPIC online mode is enabled, YOPO uses a 0.5 m local-goal tolerance so it
can advance through successive graph nodes. The normal YOPO-only mode keeps its
2 m tolerance.

## Sparse Graph v1

Prepare the 2-channel, 9-D-state YOPO executor from the original YOPO weights:

```bash
bash scripts/20_prepare_graph_executor.sh
```

Run the deterministic synthetic large-wall regression:

```bash
SYNTHETIC=1 bash scripts/22_test_map2_graph.sh
```

The local Map2 test data contains ten random frames in
`Scene_0001` and one fixed large-obstacle acceptance frame in
`Scene_0002`. Replay the latter with:

```bash
SCENE=Scene_0002 FRAME=0 bash scripts/22_test_map2_graph.sh
```

To recollect random Map2 frames, launch
`/Game/FlyingCPP/Maps/FlyingExampleMapV2` in UE and run:

```bash
bash scripts/21_collect_map2_graph_data.sh
```

The Graph uses the current DepthPlanar image directly for swept-volume edge
checks. The offline `tree.ply` is dataset geometry and is not queried by the
runtime Graph.

FRGraph-style free-region waypoint generation can be replayed on the fixed
Map2 frame with:

```bash
bash scripts/30_test_frgraph_reproduction.sh
bash scripts/28_run_frgraph_map2.sh
PORT=8768 PEARL_DEVICE=cuda bash scripts/29_start_frgraph_viewer.sh
```

See `docs/frgraph_integration.md` for the upstream ROS1 boundary and the
DepthPlanar adapter contract.

## Attribution

The local trajectory planner and controller originate from the Apache-2.0
licensed TJU Aerial Robotics YOPO project. PEARL and SCLIP retain their own
upstream notices and licenses.
