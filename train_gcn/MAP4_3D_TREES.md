# Map4 3D GCN (`trees`)

This is the sibling checkpoint of the 3-D `line` model.

## What changes

- Same 3-D graph construction and 15-class output space as `line`.
- Same mesh-truth A* label source.
- Different semantic prompt: `trees`.
- The online 3-D selector now also consumes the planner's 5-column semantic
  ranking and repeats it across the 3 pitch rows.

## Files

```text
train_gcn/dataset_privileged_map4_3d_trees.pt
train_gcn/frontier_gcn_map4_3d_trees.pt
```

## Offline result

The current prompt-conditioned dataset contains 2,722 graph states: 522 states
from eligible 3-D logs plus 2,200 privileged mesh-route states. The same 3-D
mesh-truth A* route is used as `line`: route length 145.51 m and A* height
range -1.75..3.75 m. The dataset includes non-horizontal labels, so this is no
longer just a 5-column horizontal classifier wrapped as 15 classes.

Using the session-level split in `train.py`:

```text
majority-class baseline   22.7%
planner semantic baseline 78.1%
GCN accuracy              91.0%
GCN macro accuracy        71.0%
within one direction      98.4%
mean column error         0.148
best epoch                28
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

```bash
PROMPT_3D=trees DEVICE=cuda bash train_gcn/train_map4_3d_semantic.sh
```
