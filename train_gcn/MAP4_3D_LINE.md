# Map4 3D GCN (`line`)

This experiment is a separate 3D checkpoint and does not replace the existing
five-column 2D models.

## Truth and labels

- Geometry source: complete Unreal runtime render-mesh triangles, exported
  through `simGetMeshPositionVertexBuffers` after applying spline deformation
  to Map4 `USplineMeshComponent` power lines.
- No RGB-D survey, online point-cloud union, or flight trajectory is used as
  geometry truth.
- The start and mission goal are `(0, 0, 3.5)` and `(0, 140, 3.5)` in
  `world_enu`; this height intersects the power-line corridor.
- A 26-connected 3D A* searches the inflated mesh volume. The label is the
  direction of the A* route point at least 35 m ahead of the current sample.
- Class encoding is `row * 5 + column`: row 0 is +20 deg pitch (up), row 1 is
  horizontal, row 2 is -20 deg pitch (down); columns 0..4 are -40..+40 deg
  yaw relative to the body forward direction.

## Files

```text
paper/scalenav/pics/map4_mesh_truth_3d_20260906.npz
train_gcn/dataset_privileged_map4_3d_line.pt
train_gcn/frontier_gcn_map4_3d_line.pt
```

The trained model is a three-layer weighted GCN followed by a joint candidate
head over all 15 directions. Its input has 24 node features including 3D
position, body-frame displacement, goal displacement, graph flags, and the
planner's 5-column semantic ranking repeated across the three pitch rows.

## Offline result

The current prompt-conditioned dataset contains 2,722 graph states: 522 states
from eligible 3-D logs plus 2,200 privileged mesh-route states. The route from
the start to the goal is 145.51 m and its A* height range is -1.75..3.75 m, so
the route actually performs a vertical detour at the obstacle corridor. The
dataset includes non-horizontal labels, so this is no longer just a 5-column
horizontal classifier wrapped as 15 classes.

Using the session-level split in `train.py`:

```text
majority-class baseline   22.7%
planner semantic baseline 78.1%
GCN accuracy              87.1%
GCN macro accuracy        76.6%
within one direction      96.9%
mean column error         0.238
best epoch                15
```

## Online command

The 3D runner is independent from the existing 2D runner:

```bash
bash scalenav_ws/src/aut_test/run_3d_0_140.sh --count 1
```

The default prompt is `line` and the default checkpoint is
`train_gcn/frontier_gcn_map4_3d_line.pt`. To switch to the sibling `trees`
checkpoint, set `PROMPT_3D=trees` and `GCN_3D_MODEL=...trees.pt` before
launching.

It sets `GCN_MODEL` to the 15-class checkpoint, `GRAPH_FIXED_LAYER=false`,
`FIXED_ALTITUDE=false`, start/goal `z=3.5`, and semantic prompt `line`.
The downstream ScaleNav A*, route continuity and YOPO controller remain the
same; only the frontier direction hint changes from 5 to 15 directions.
