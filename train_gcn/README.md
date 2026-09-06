# ScaleNav GCN Training

This directory builds a real-log dataset and trains a five-column frontier policy.

Collect samples from all available sessions:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python train_gcn/collect_dataset.py \
  --output train_gcn/dataset_goal.pt
```

Train with disjoint session-level train/validation/test splits:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python train_gcn/train.py \
  --dataset train_gcn/dataset_goal.pt --device cuda \
  --save train_gcn/frontier_gcn.pt
```

The goal-aware label is generated from the logged skeleton map. By default, the
oracle uses the final accumulated skeleton graph of the whole session (while
the model input remains the graph available at that frame), so the label does
not depend on a single frame's partial map. For every five-column first step,
Dijkstra computes the shortest feasible graph route from the current odometry
node through that direction to the mission goal. The column with the lowest
finite route cost is the target. `--oracle-scope current` restores per-frame
labels for experiments. `selected_semantic_column` from the planner is retained
only as an analysis field, never as the training target. Candidate marker
positions are virtual rays from the current vehicle pose; the label uses real
skeleton nodes and edges.

The best checkpoint is selected on validation macro accuracy. Evaluation then
uses the untouched test sessions and reports ordinary accuracy, macro accuracy
(to expose class imbalance), and column switch rate for the learned policy,
the original logged planner, and the majority-column baseline.

View the goal-aware samples, synchronized RGB/depth/heatmap, world-aligned
obstacle cloud, and the recomputed A* path:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python train_gcn/make_goal_viewer.py
```

Open `train_gcn/goal_dataset_viewer.html` in a browser. The cyan polyline is
the feasible A* route to the mission goal; the five column costs on the right
are the costs of feasible first-step directions.

For privileged labels based on the static Map2 world point-cloud map (rather
than the learned skeleton graph), build and train the independent dataset:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  train_gcn/collect_privileged_dataset.py \
  --output train_gcn/dataset_privileged_map2_35m.pt \
  --map-ply paper/scalenav/pics/map2_ground_truth_airsim_20260904.ply \
  --occupancy-cache train_gcn/global_occupancy_map2_r075_i12.pt \
  --map-scope global --resolution 0.75 --inflate 1.2 --stride 1 \
  --lookahead 35
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python train_gcn/train.py \
  --dataset train_gcn/dataset_privileged_map2_35m.pt --device cuda \
  --architecture legacy --epochs 30 --no-class-weights \
  --save train_gcn/frontier_gcn_map2_35m.pt
```

This oracle voxelizes and inflates the complete static world map by the vehicle
radius, then runs an ordinary 8-connected grid A* from the current odometry
cell to the actual mission goal. The static map is essential: merging depth
returns from different sessions and clearing historical flight paths can erase
real walls and produce an A* route that visibly passes through an obstacle.
The direction label is taken from the route point at least 35 m ahead, not from
the immediate `path[1]` grid neighbor, so the policy is trained to commit to
detours early. Since the label is body-relative, `sin(yaw)` and `cos(yaw)` are
included in each node feature.
Sessions/frames without a complete route are excluded instead of snapping the
goal to a distant skeleton node.

The five output classes are `+40`, `+20`, `0`, `-20`, and `-40` degrees in the
vehicle frame (left to right). On the 5337-sample Map2 dataset, the saved
checkpoint was selected only with validation sessions and achieved 89.3%
five-class exact accuracy and 89.4% macro accuracy on 707 untouched test
frames. 99.4% of predictions were within one adjacent column. The logged
original planner matched the same strict static-map labels on only 22.2% of
those frames.

Generate the checked visualization with:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python \
  train_gcn/make_goal_viewer.py \
  --dataset train_gcn/dataset_privileged_map2_35m.pt \
  --model train_gcn/frontier_gcn_map2_35m.pt \
  --occupancy-cache train_gcn/global_occupancy_map2_r075_i12.pt \
  --output train_gcn/map2_35m_gcn_viewer.html --max-samples 300
```

The viewer reports `gt_visual_mismatches`; it must be zero. A causal switching
penalty can be selected on validation sessions and audited on test sessions
with `evaluate_temporal.py`. In the latest reproducible sweep, validation
selected penalty `0.00`. The untouched test split reached 89.3% exact accuracy,
89.4% macro accuracy, 99.4% within-one-column accuracy, and 0.115 mean absolute
column error. Forcing penalty `0.05` instead gives 89.1%, 89.3%, 99.4%, and
0.116, respectively; it is therefore not the selected result.

Online GCN test (independent startup):

```bash
cd /mnt/code/lab/yopo/OpenSeek
bash scalenav_ws/scripts/start_gcn_online.sh
```

This starts AirSim, the depth-to-point-cloud converter, ScaleNav graph, the
GCN selector, and the existing YOPO trajectory controller. Publish a mission
goal in `world_enu`, for example:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world_enu}, pose: {position: {x: 0.0, y: 140.0, z: 1.6}}}'
```

The GCN selector reads `/scalenav/graph` and `/sim/odom`, then publishes only
its selected body-relative column (`0..4`) on
`/scalenav/gcn_frontier_column`. ScaleNav applies that direction when ranking
reachable frontier nodes and retains its original topology A*, route checks,
path processing, and `/scalenav/local_goal` output. The existing YOPO
controller consumes `/scalenav/local_goal` and publishes
`/scalenav/trajectory_point`. Bubble markers are not an input or gate for the
GCN selector; the original `start.sh` and `start_route_yopo.sh` are unchanged.
The GCN startup enables `gcn_frontier_required:=true`: after the graph is
available, every frontier A* search uses the latest GCN column (a short
inference delay does not silently switch back to the ordinary ranking). The
only intentional exception is the final direct-mission-goal phase, where no
frontier choice is needed. The logger records each column as
`gcn_frontier_column` and planner timing records include
`gcn_frontier_hint_used` and cumulative receive/use counters.
