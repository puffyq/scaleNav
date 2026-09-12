# Map5 GCN at z=140.6 Without Semantic Input

This two-trial batch is paired with the same-height semantic-enabled GCN
batch `run_20260911_183259_3884141`. It is not mixed with the `z=170.6` or
Map2 experiments.

## Configuration

- source batch: `run_20260911_183455_3889027`
- stack: `gcn`
- checkpoint: `train_gcn/frontier_gcn_map5_35m.pt`
- mission: `(0, 0, 140.6)` m to `(0, 200, 140.6)` m
- direct mission distance: `200 m`
- semantics: disabled (`semantic=false` in `config.json`, `pearl=0` in the
  launch log)
- stored prompt: `buildings, walls, obstacles` (inactive)
- timeout: `90 s`
- planning layer: `z=140.6`, fixed altitude enabled

## Batch result

Both attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (2/2). Values are mean +/- sample standard
deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 41.338 +/- 0.301 | 41.124 | 41.551 |
| Path length (m) | 228.050 +/- 0.209 | 227.902 | 228.198 |
| Average speed (m/s) | 5.517 +/- 0.045 | 5.485 | 5.549 |
| Maximum speed (m/s) | 6.334 +/- 0.096 | 6.266 | 6.401 |
| Final error (m) | 0.043 +/- 0.027 | 0.024 | 0.062 |
| Path efficiency (%) | 87.700 +/- 0.081 | 87.643 | 87.757 |

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and
`trial_0001.json`, `trial_0002.json` are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_183455_3889027/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
