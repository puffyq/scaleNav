# Map2 GCN Without Semantics — Backup Experiment

This two-trial batch is retained as a backup experiment only. It is excluded
from the main paper tables and `test_data/aggregate_metrics.csv`, and does not
replace the ten-trial no-semantic GCN ablation from September 5, 2026.

## Configuration

- source batch: `run_20260911_094902_2114755`
- stack: `gcn`
- checkpoint reported by the launch log: `train_gcn/frontier_gcn_map2_35m.pt`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- semantic front end: disabled (`semantic=false` in `config.json`, `pearl=0`
  in both trial launch logs)
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`
- stored prompt: `blocks, walls, box` (not an enabled semantic condition)

## Batch result

Both attempts are valid and successful (trials 1 and 2); no collision,
timeout, or excluded attempt. Success rate is 100% (2/2), based only on this
small backup batch. Values below are mean +/- sample standard deviation
over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 31.387 +/- 0.219 | 31.231 | 31.542 |
| Path length (m) | 168.052 +/- 0.539 | 167.670 | 168.433 |
| Average speed (m/s) | 5.354 +/- 0.020 | 5.340 | 5.369 |
| Maximum speed (m/s) | 6.212 +/- 0.042 | 6.182 | 6.242 |
| Final error (m) | 0.130 +/- 0.010 | 0.122 | 0.137 |
| Path efficiency (%) | 83.308 +/- 0.267 | 83.119 | 83.497 |

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_094902_2114755/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
