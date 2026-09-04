# ScaleNav GCN Training

This directory builds a real-log dataset and trains a five-column frontier policy.

Collect samples from all available sessions:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python train_gcn/collect_dataset.py \
  --output train_gcn/dataset.pt
```

Train with disjoint session-level train/validation/test splits:

```bash
/mnt/code/lab/yopo/YOPO-Rally/.venv/bin/python train_gcn/train.py \
  --dataset train_gcn/dataset.pt --device cuda \
  --save train_gcn/frontier_gcn.pt
```

The label is generated from the logged skeleton map: starting at the nearest
logged odometry node, each of five forward angular columns receives the maximum
reachable forward distance. The column with the largest distance is the map
oracle target. `selected_semantic_column` from the planner is retained only as
an analysis field, never as the training target. The graph snapshot does not
contain semantic-column poses, so candidate marker positions are reconstructed
around the logged frontier marker; the map label itself uses the real skeleton
nodes and edges.

The best checkpoint is selected on validation macro accuracy. Evaluation then
uses the untouched test sessions and reports ordinary accuracy, macro accuracy
(to expose class imbalance), and column switch rate for the learned policy,
the original logged planner, and the majority-column baseline.
