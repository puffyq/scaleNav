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

The corridor term follows the Bubble Planner feasible-set definition. Dense
witness bubbles form a union of safe spheres, and each sampled trajectory point
uses `min_i(||x-c_i||-r_i)`. Points inside any sphere have zero corridor
penalty; points outside all spheres receive mean-squared and worst-point
penalties. The ESDF safety loss also includes a worst-point term so a brief
collision cannot disappear inside a trajectory average.

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
  --output dataset/pilot_001 --scenes 2 --frames 500 --routes-per-frame 3 \
  --scene-style alternating --obstacles 40 --preview-routes 100 --seed 0 \
  --overwrite

python -m data.validate_snapshot_dataset dataset/pilot_001 --require-routes
```

Generate the independent offline test set with a disjoint seed. It is never
passed to the trainer and is marked `dataset_role=offline_test` in its report:

```bash
python -m data.ground_truth_dataset \
  --output dataset/test_001 --scenes 2 --frames 200 --routes-per-frame 3 \
  --scene-style alternating --obstacles 40 --preview-routes 100 \
  --seed 900001 --dataset-role offline_test --overwrite

python -m data.validate_snapshot_dataset dataset/test_001 --require-routes
```

Build static HTML viewers for either dataset. The generated page works from
`file://` and does not require an HTTP server:

```bash
python -m data.build_dataset_viewer dataset/pilot_001 --overwrite
python -m data.build_dataset_viewer dataset/test_001 --overwrite
```

Open `dataset/pilot_001/viewer/index.html` for training data or
`dataset/test_001/viewer/index.html` for offline test data. The viewer exposes
scene/frame/route controls, RGB, colorized depth, the ground-truth obstacle
map, witness path, corridor bubbles, topology nodes, route alternatives, pose,
and quality metrics.

Available scene distributions are `blocks`, `mixed`, `forest`, `yopo_forest`,
`yopo_real_forest`, and `alternating`. `yopo_forest` is the fast cylindrical
approximation of YOPO-Simple's 4 m forest layout. `yopo_real_forest` loads the
original YOPO-Simple binary `tree.ply`; its occupancy, clearance, depth, and
exported scene cloud use transformed trunk, branch, and crown points. Override
the asset location with `--yopo-tree-ply` when YOPO-Simple is installed
elsewhere. `blocks` uses full-height rotated boxes with a long-tailed
2.5-30 m side distribution (60% small, 28% medium, 12% large sides), and each
default block-bearing scene is guaranteed to contain a 15-30 m structure. The
default 80 x 80 m map leaves enough space around these building-scale blocks. Route
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
  --data dataset/pilot_001 --output saved \
  --epochs 50 --batch-size 16 --workers 0 \
  --freeze-backbone-epochs 3 --save-interval 5
```

Use `--workers 0` in restricted containers that disallow multiprocessing
sockets. On a normal workstation, increasing it to 4 only affects data loading.

Batch 001 used a two-stage run: the original 50-epoch route-conditioned model,
then a 30-epoch Bubble-Planner safety fine-tune that loads model weights while
resetting optimizer and checkpoint-selection state:

```bash
python train_yopo.py \
  --data dataset/pilot_001 --output saved \
  --checkpoint saved/YOPO_2/best.pth --finetune \
  --epochs 30 --batch-size 16 --workers 0 \
  --freeze-backbone-epochs 0 --save-interval 5
```

After training, run the model itself on the isolated test set and build a
static HTML with the selected polynomial trajectory overlaid in orange:

```bash
python evaluate_yopo.py \
  --data dataset/test_001 \
  --checkpoint saved/YOPO_3/best.pth \
  --output dataset/test_001/model_eval_001 \
  --batch-size 32 --workers 0
```

Open `dataset/test_001/model_eval_001/index.html`. The evaluator samples each
selected fifth-order trajectory, independently queries `tree.ply`, and reports
collision rate, minimum clearance, corridor violation, and route progress. The
offline dataset has no recorded vehicle dynamics, so batch 001 explicitly uses
zero initial velocity and acceleration and records that assumption in
`evaluation_report.json`.

Use `--split valid --report-only` for checkpoint selection without rebuilding
HTML assets. Batch 001 evaluates the 300 held-out pilot routes this way before
touching `test_001`.

The completed batch-001 model is `saved/YOPO_3/best.pth` (safety fine-tune
epoch 15). On the fixed 300-route validation split it has zero collisions,
0.745 m minimum clearance, 0.0109 m mean maximum bubble-union violation, and
5.339 m mean route progress. On all 1200 isolated `test_001` routes it has zero
collisions, 0.623 m minimum clearance, 0.0090 m mean maximum violation, and
5.348 m mean route progress. Validation selected the checkpoint; test metrics
were computed only after selection.

Checkpoints contain the model and optimizer states, route dataset version,
bubble count, anchor distances, and all loss weights. Selection uses validation
`selected_total_cost`; TensorBoard also records oracle cost, regret, top-1, and
every individual trajectory cost. Every scene uses a seeded, disjoint
frame-group holdout, so all obstacle distributions occur in both train and
validation while routes from the same depth frame never cross the split.

## Verification

```bash
python -m pytest -q tests
```

The suite covers coordinate conversion, safe pose sampling, route NPZ
round-trips, quality rejection, conservative bubble sampling, route loss
gradients and invalid-route handling, scene-level splitting, path-conditioned output changes, and
a complete ESDF-backed optimizer update with checkpoint serialization. It also
covers large-block A* detours, analytic ground-truth depth rendering, and a
generated-scene contract round trip.

## Paired YOPO-Simple benchmark

`compare_yopo.py` runs the original YOPO-Simple `3x5` checkpoint and the
route-conditioned checkpoint on exactly the same samples. `benchmark_001` is a
two-scene group of 1200 routes: 600 routes from a YOPO-Simple-style forest
(`4 m` jittered tree cells, `0.5-1.0` asset scale) and 600 routes from the
Map2-style rotated large-block distribution (sides up to 30 m). Both models
receive the same `160x96` depth, pose, zero motion state and frontier goal;
Route-YOPO additionally receives the witness bubbles by design.

Reproduce the batch with:

```bash
cd /mnt/code/lab/yopo/OpenSeek/train_scalenav
bash run_benchmark_001.sh
```

The offline artifacts are:

- `dataset/benchmark_001/generation_report.json`: scene and route audit;
- `dataset/benchmark_001/comparison_001/comparison_report.json`: aggregate and
  per-scene metrics;
- `dataset/benchmark_001/comparison_001/comparison_predictions.json`: all 1200
  paired trajectories and metrics;
- `dataset/benchmark_001/comparison_001/viewer/index.html`: frame-by-frame HTML
  showing witness, Route-YOPO (orange) and YOPO-Simple (purple).

The benchmark uses the same 41-point polynomial trajectory sampling and point
cloud clearance test for both policies. It is an offline, zero-motion,
single-step test, not a closed-loop flight evaluation.

`benchmark_002` adds a third, real-tree scene without changing or overwriting
batch 001. Its three 600-route scenes are the cylindrical YOPO forest, the
original-tree-asset forest, and the Map2-style block distribution. Reproduce
the full 1800-route paired evaluation with:

```bash
bash run_benchmark_002.sh
```

The real-tree source is recorded by absolute path and SHA-256 in
`dataset/benchmark_002/generation_report.json`. Results and model outputs are
under `dataset/benchmark_002/comparison_001/`; open
`dataset/benchmark_002/comparison_001/viewer/index.html` for the model-output
viewer. On the current checkpoints, real-tree collision rates are 17.33% for
Route-YOPO and 20.33% for YOPO-Simple. This is a materially harder input
distribution and must be reported separately from the cylindrical forest.

## Batch 002 retraining and three-model benchmark

`pilot_002` applies the CHG-0002 witness fix to 2250 routes: 750 cylindrical
forest, 750 original-tree-asset forest, and 750 large-block routes. Generate
and inspect it with `bash run_pilot_002.sh`. The two-stage CPU training is
captured by `run_training_002.sh`; stage 1 uses `wprogress=0.8`, while stage 2
uses `wprogress=1.2` to recover progress without relaxing the corridor loss.
The selected final checkpoint is `saved/YOPO_6/best.pth` (stage-2 epoch 10).

`benchmark_003` uses disjoint seed `820003` and 1800 routes. Reproduce the
final paired evaluation with `bash run_benchmark_003.sh`. Every sample gives
the same depth, pose, zero motion and frontier goal to the current Route-YOPO,
previous Route-YOPO and YOPO-Simple; both Route-YOPO versions also receive the
same witness bubbles. The final HTML is
`dataset/benchmark_003/comparison_002/viewer/index.html`.
