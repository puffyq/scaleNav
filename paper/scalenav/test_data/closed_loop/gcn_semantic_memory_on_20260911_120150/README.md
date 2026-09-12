# Map2 GCN With Semantic Information and With Memory Enabled

This ten-trial batch is the selected semantic-on/memory-enabled result.
Its semantic-ablation row is used in `root.tex`; `test_data/aggregate_metrics.csv`
still contains older batches and has not been regenerated.

## Configuration and provenance

- source batch: `run_20260911_120150_2234401`
- stack: `gcn`
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in launch logs)
- memory: enabled, as corrected by the user
- prompt: `blocks, walls, box`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`

The user corrected the earlier no-memory classification to memory enabled.
The original logs contain `history=NO_HISTORY`, and the saved `config.json`
does not record the memory switch. This discrepancy remains unresolved:
the condition label follows the user's correction, not an independently
verified runtime setting. Original data and log text are not modified.

## Batch result

All ten attempts are valid and successful, with no collision, timeout, or
excluded attempt. Values are mean +/- sample standard deviation.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 33.787 +/- 0.848 | 32.694 | 35.178 |
| Path length (m) | 173.224 +/- 3.637 | 168.029 | 177.069 |
| Average speed (m/s) | 5.128 +/- 0.054 | 5.029 | 5.201 |
| Maximum speed (m/s) | 6.409 +/- 0.021 | 6.372 | 6.428 |
| Final error (m) | 0.081 +/- 0.028 | 0.033 | 0.127 |
| Path efficiency (%) | 80.852 +/- 1.705 | 79.065 | 83.319 |

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and ten
`trial_*.json` records are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_120150_2234401/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
