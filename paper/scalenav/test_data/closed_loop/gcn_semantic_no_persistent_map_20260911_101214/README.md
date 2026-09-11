# Map2 Semantic GCN Without Full-History Map Persistence — Backup Experiment

This two-trial batch is retained as a backup experiment only. It is excluded
from the main paper tables and `test_data/aggregate_metrics.csv`, and does
not replace an existing main or ablation batch.

## Configuration

- source batch: `run_20260911_101214_2140124`
- stack: `gcn`
- checkpoint reported by both launch logs: `train_gcn/frontier_gcn_map2_35m.pt`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in both
  trial launch logs)
- prompt: `blocks, walls, box`
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`

## Memory condition and provenance

The user identifies this batch as the condition without persistent memory
(full-history map persistence disabled). The archived `config.json` does not
record `persist_map`, map-history radius, point cap, or local-sliding-graph
settings, and the inspected trial launch lines do not record these settings.
Exact runtime values are therefore not asserted here. This label does not
imply that graph, accepted-route, or semantic memory is entirely disabled.

Both batches in this memory comparison have semantics enabled. The launch
logs report `geometry_map=SLIDING_WINDOW` in both conditions; that text alone
does not distinguish a bounded local history from the full-mission radius.
The original configuration files are preserved without inferred fields.

## Batch result

Both attempts are valid and successful (trials 1 and 2); no collision,
timeout, or excluded attempt. Success rate is 100% (2/2). Values below are
mean +/- sample standard deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 34.387 +/- 0.403 | 34.102 | 34.672 |
| Path length (m) | 178.918 +/- 0.775 | 178.370 | 179.466 |
| Average speed (m/s) | 5.203 +/- 0.038 | 5.176 | 5.230 |
| Maximum speed (m/s) | 6.397 +/- 0.005 | 6.394 | 6.400 |
| Final error (m) | 0.078 +/- 0.010 | 0.071 | 0.085 |
| Path efficiency (%) | 78.249 +/- 0.339 | 78.009 | 78.488 |

This small backup batch does not establish a statistically reliable benefit
or disadvantage of persistent memory. The paired backup summary is in
`test_data/README.md`.

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_101214_2140124/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
