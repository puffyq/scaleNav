# ScaleNav Route-Conditioned YOPO Training

This directory contains the offline data and training implementation for batch
`001` in `scalenav_ws/docs/TODO_001.md`. Deployment inference, online ROS
control, TensorRT export, and evaluation nodes are intentionally excluded.

## Implemented Pipeline

```text
YOPO-style ground-truth blocks/forest OR AirSim RGB-D
  + ground-truth A* route OR production EPIC accepted route
  -> RouteQualityGate + continuous clearance audit
  -> Scene_N/routes.npz
  -> fixed K witness bubbles for model conditioning
  + dense witness route for differentiable loss
  -> route-conditioned YOPO
  -> safety + smoothness + acceleration + frontier
     + corridor + progress + tangent costs
  -> detached total cost as primitive score label
```

The trainer does not regress the witness polyline as a time-parameterized
expert trajectory. Witness geometry constrains the trajectory through route
costs and also conditions primitive scoring.

## Layout

```text
config/                 trajectory, route, and loss configuration
data/coordinates.py     NED/FRD to ENU/FLU conversion
data/snapshot_dataset.py
                        safe AirSim RGB-D collection
data/route_contract.py  routes.npz, quality gate, corridor sampling
data/epic_route_labeler.py
                        accepted EPIC output to routes.npz
data/ground_truth_dataset.py
                        Map2-style blocks/forest, A*, depth, routes and audit
data/synthetic_dataset.py
                        deterministic end-to-end smoke data
loss/route_loss.py      corridor, progress, and tangent loss
policy/yopo_dataset.py  route-conditioned Dataset
policy/yopo_network.py  primitive-frame route conditioning
policy/yopo_trainer.py  training, metrics, checkpointing
train_yopo.py           training CLI
```

`reference/yopo_simple_generator/` preserves the original YOPO-Simple CUDA
data generator. The ground-truth generator is an offline simulator labeler: it
uses the exact generated occupancy to produce safe A* guidance and does not run
at deployment. For real captured data, `route_generation/topology_core/`
records the production source boundary and the Python labeler only consumes
accepted EPIC witness output.

## Data Contract

Each scene contains:

```text
Scene_0000/
  data.toml
  tree.ply
  routes.npz
  Textures/
    depth_000000.exr
    rgb_000000.png
```

`data.toml`, `tree.ply`, frontier goals, witness points, and topology centers
all use `world_enu`; body-relative model inputs use `body_flu`. The source
AirSim NED/FRD pose is retained in each frame record for auditing.

Production EPIC accepted routes are passed to the labeler as JSONL. Each line
has this shape:

```json
{"frame_index":0,"mission_goal_world":[0,20,1.6],"frontier_goal_world":[0,10,1.6],"path_points_world":[[0,0,1.6],[0,10,1.6]],"topo_centers_world":[],"topo_bubble_radius_m":[],"topo_persistent_id":[],"found":true,"blocked":false,"committed":true,"route_seed":0}
```

The path must be the production EPIC accepted edge-witness polyline. Failed
searches should also be emitted with `found=false` or `blocked=true`; they are
stored with stable quality flags but excluded from training.

## Commands

Install dependencies and run from this directory:

```bash
cd /mnt/code/lab/yopo/OpenSeek/train_scalenav
python -m pip install -r requirements.txt
```

Generate deterministic smoke data:

```bash
python -m data.synthetic_dataset \
  --output /tmp/scalenav_route_smoke --scenes 2 --frames 4 --overwrite
```

Generate the batch-001 pilot directly from scene truth. Scene 0 uses Map2-style
large blocks and Scene 1 mixes large blocks with YOPO-style forest obstacles.
At least one route per frame must be a real obstacle detour. The command also
writes 100 top-down route previews and `generation_report.json`:

```bash
python -m data.ground_truth_dataset \
  --output dataset --scenes 2 --frames 500 --routes-per-frame 3 \
  --scene-style alternating --obstacles 40 --preview-routes 100 --seed 0 \
  --overwrite

python -m data.validate_snapshot_dataset dataset --require-routes
```

Available scene distributions are `blocks`, `mixed`, `forest`, and
`alternating`. `blocks` uses 2.5-6.5 m wide, full-height rotated boxes. Route
search runs on the obstacle grid inflated by robot radius, safety margin, and
an additional smoothing margin. Every accepted route is then independently
checked against the continuous point-cloud clearance and curvature gates.

Collect an AirSim scene with obstacle-safe pose rejection:

```bash
python -m data.snapshot_dataset \
  --output dataset \
  --airsim-root /path/to/Colosseum/PythonClient \
  --obstacle-ply /path/to/tree_ned.ply \
  --scene-id 0000 --count 500 --seed 0 --safe-dist 0.6
```

Convert accepted EPIC output and validate the scene:

```bash
python -m data.epic_route_labeler \
  --scene dataset/Scene_0000 \
  --epic-jsonl /path/to/accepted_routes.jsonl

python -m data.validate_snapshot_dataset dataset --require-routes
```

Run one end-to-end smoke update:

```bash
python train_yopo.py \
  --data /tmp/scalenav_route_smoke \
  --output /tmp/scalenav_train_runs \
  --epochs 1 --batch-size 1 --workers 0 \
  --max-train-batches 1 --max-val-batches 1
```

Run normal training:

```bash
python train_yopo.py \
  --data dataset --output saved \
  --epochs 50 --batch-size 16 --workers 4 \
  --freeze-backbone-epochs 3 --save-interval 5
```

Checkpoints contain the model and optimizer states, route dataset version,
bubble count, anchor distances, and all loss weights. Selection uses validation
`selected_total_cost`; TensorBoard also records oracle cost, regret, top-1, and
every individual trajectory cost.

## Verification

```bash
python -m pytest -q tests
```

The suite covers coordinate conversion, safe pose sampling, route NPZ
round-trips, quality rejection, conservative bubble sampling, route loss
gradients/dropout, scene-level splitting, path-conditioned output changes, and
a complete ESDF-backed optimizer update with checkpoint serialization. It also
covers large-block A* detours, analytic ground-truth depth rendering, and a
generated-scene contract round trip.
