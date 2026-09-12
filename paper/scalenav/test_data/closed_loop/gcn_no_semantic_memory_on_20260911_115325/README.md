# Map2 GCN Without Semantic Information and With Memory Enabled

This ten-trial batch is the selected semantic-off/memory-enabled result.
Its semantic-ablation row is used in `root.tex`; `test_data/aggregate_metrics.csv`
still contains older batches and has not been regenerated.

## Configuration

- source batch: `run_20260911_115325_2220771`
- stack: `gcn`
- semantic input: disabled (`semantic=false` in `config.json`, `pearl=0` in
  the launch logs)
- memory: enabled, as corrected by the user
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`

The user corrected the earlier no-memory classification to memory enabled.
The original logs contain `history=NO_HISTORY`, and the saved `config.json`
does not record the memory switch. This discrepancy remains unresolved:
the condition label follows the user's correction, not an independently
verified runtime setting. Original data and log text are not modified.

## Batch result

All ten attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (10/10). Values below are mean +/- sample
standard deviation over the ten successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 32.825 +/- 2.302 | 30.864 | 38.182 |
| Path length (m) | 173.988 +/- 9.336 | 166.094 | 190.733 |
| Average speed (m/s) | 5.307 +/- 0.165 | 4.995 | 5.478 |
| Maximum speed (m/s) | 6.308 +/- 0.181 | 6.181 | 6.788 |
| Final error (m) | 0.083 +/- 0.038 | 0.023 | 0.143 |
| Path efficiency (%) | 80.663 +/- 4.100 | 73.401 | 84.290 |

## Records

The copied JSON/CSV files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_115325_2220771/`.
Raw stack logs remain in the source batch directory; flight-log paths are
recorded in `summary.csv`.
