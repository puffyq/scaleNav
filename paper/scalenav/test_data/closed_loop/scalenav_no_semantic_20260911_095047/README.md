# Map2 ScaleNav Without Semantics — Backup Experiment

This two-trial batch is retained as a backup experiment only. It is excluded
from the main paper tables and `test_data/aggregate_metrics.csv`, and does not
replace the ten-trial no-semantic ScaleNav ablation from August 29, 2026.

## Configuration

- source batch: `run_20260911_095047_2118305`
- stack: `scalenav`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- semantics: disabled (`semantic=false` in `config.json`, `semantic=0`
  in both trial launch logs)
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`,
  `local_sliding_graph=false`
- stored prompt: `blocks, walls, box` (not an enabled semantic condition)

## Batch result

Both attempts are valid and successful (trials 1 and 2); no collision,
timeout, or excluded attempt. Success rate is 100% (2/2), based only on this
small backup batch. Values below are mean +/- sample standard deviation
over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 35.924 +/- 0.294 | 35.716 | 36.132 |
| Path length (m) | 192.315 +/- 7.471 | 187.032 | 197.598 |
| Average speed (m/s) | 5.353 +/- 0.164 | 5.237 | 5.469 |
| Maximum speed (m/s) | 6.377 +/- 0.075 | 6.324 | 6.430 |
| Final error (m) | 0.111 +/- 0.024 | 0.094 | 0.127 |
| Path efficiency (%) | 72.852 +/- 2.830 | 70.851 | 74.853 |

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_095047_2118305/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
