# Map2 Semantic GCN With Full-History Map Persistence — Backup Experiment

This two-trial batch is retained as a backup experiment only. It is excluded
from the main paper tables and `test_data/aggregate_metrics.csv`, and does
not replace an existing main or ablation batch.

## Configuration

- source batch: `run_20260911_101453_2144759`
- stack: `gcn`
- checkpoint reported by both launch logs: `train_gcn/frontier_gcn_map2_35m.pt`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in both
  trial launch logs)
- prompt: `blocks, walls, box`
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`

## Memory condition and provenance

The user identifies this batch as the persistent-memory condition. The
user-provided launcher console for this exact source run reports:

- `Full history mode enabled (persistent map)`
- `persist_map=true`
- `map_history_radius=1000.0`
- `local_sliding_graph=false`

These settings are transcribed from the supplied console, not from the
archived `config.json`, which does not record map-persistence parameters.
Persistence here refers to map history within a trial, not accumulated
experience across trials; the supplied console resets AirSim before each
trial.

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
| Duration (s) | 33.253 +/- 0.902 | 32.616 | 33.891 |
| Path length (m) | 171.448 +/- 5.670 | 167.438 | 175.457 |
| Average speed (m/s) | 5.155 +/- 0.031 | 5.134 | 5.177 |
| Maximum speed (m/s) | 6.422 +/- 0.008 | 6.416 | 6.428 |
| Final error (m) | 0.048 +/- 0.007 | 0.043 | 0.053 |
| Path efficiency (%) | 81.702 +/- 2.702 | 79.792 | 83.613 |

This small backup batch does not establish a statistically reliable benefit
or disadvantage of persistent memory. The paired backup summary is in
`test_data/README.md`.

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_101453_2144759/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
