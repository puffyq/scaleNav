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
