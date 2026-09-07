# Map4 3D GCN (`trees`)

This is the sibling checkpoint of the 3-D `line` model.

## What changes

- Same 3-D graph construction and 15-class output space as `line`.
- Same mesh-truth A* label source.
- Different semantic prompt: `trees`.
- The online 3-D selector now also consumes the planner's 5-column semantic
  ranking and repeats it across the 3 pitch rows.

## Expected files

```text
train_gcn/dataset_privileged_map4_3d_trees.pt
train_gcn/frontier_gcn_map4_3d_trees.pt
```

## Online run

```bash
PROMPT_3D=trees \
GCN_3D_MODEL=train_gcn/frontier_gcn_map4_3d_trees.pt \
bash scalenav_ws/src/aut_test/run_3d_0_140.sh --count 1
```

## Training

Use the same 3-D training recipe as `line`, but point it at the trees dataset
and save path.
