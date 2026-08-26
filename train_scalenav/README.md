# ScaleNav Route-Conditioned YOPO Training

This directory is the isolated workspace for batch `001` in
`scalenav_ws/docs/TODO_001.md`. It contains offline data generation and model
training code only. Online ROS nodes, deployment inference, TensorRT export,
and evaluation entry points are intentionally excluded.

## Layout

```text
train_scalenav/
  config/                 YOPO trajectory and training configuration
  data/                   ScaleNav AirSim RGB-D snapshot generation/validation
  dataset/                local generated datasets (contents are ignored by Git)
  loss/                   differentiable trajectory costs
  policy/                 Dataset, primitive transforms, network, and trainer
  route_generation/       topology sources needed by the offline route labeler
  reference/              upstream YOPO-Simple data-generator source snapshot
  tests/                  tests owned by this training workspace
  train_yopo.py           baseline training entry point
```

## Source Boundaries

The active Python baseline was copied from:

```text
scalenav_ws/src/scalenav/{config,data,loss,policy}
scalenav_ws/src/scalenav/train_yopo.py
```

The original YOPO-Simple CUDA data generator is retained only as a reference:

```text
/mnt/code/lab/yopo/YOPO-Simple/Simulator/src
```

The route-generation snapshot contains only TopoGraph, Bubble A*, and iKD-tree
sources from:

```text
scalenav_ws/src/global_graph/scalenav_graph_ros2
```

It does not contain `epic_graph_node.cpp`. Before implementing the labeler,
the production package must expose these sources as the shared
`scalenav_topology` CMake target described in `TODO_001.md`; the labeler must
link that target rather than evolve an independent A* implementation here.

## Current Status

This is a source consolidation baseline, not yet the route-conditioned model.
In particular, `policy/yopo_dataset.py` still reads the legacy YOPO image and
`pose-N.csv` format and samples a random goal. It does not yet read
`Scene_N/data.toml + routes.npz`, frontier goals, dense witness paths, or
witness corridor bubbles. The route losses are also not implemented yet.

The implementation order remains:

1. Coordinate and `routes.npz` contracts.
2. Safe AirSim pose sampling and route labeling.
3. Pilot data validation and visualization.
4. Route-conditioned Dataset and network.
5. Corridor, progress, tangent, and frontier losses.
6. Staged training.

## Commands

Run commands from this directory so the existing top-level imports resolve:

```bash
cd /mnt/code/lab/yopo/OpenSeek/train_scalenav
python -m data.snapshot_dataset --help
python -m data.validate_snapshot_dataset --help
python train_yopo.py --help
```

Training data is configured at `dataset/`, and checkpoints/TensorBoard logs are
written under `saved/`.
