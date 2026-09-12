# Map5 GCN at z=140.6 With Semantic Input

This two-trial batch is archived as a separate Map5 GCN result at planning
height `z=140.6`. It is paired with the same-height ScaleNav and FAR results,
and is not mixed with the `z=170.6` or Map2 experiments.

## Configuration

- source batch: `run_20260911_183259_3884141`
- stack: `gcn`
- checkpoint: `train_gcn/frontier_gcn_map5_35m.pt`
- mission: `(0, 0, 140.6)` m to `(0, 200, 140.6)` m
- direct mission distance: `200 m`
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in the
  launch log)
- prompt: `buildings, walls, obstacles`
- timeout: `90 s`
- planning layer: `z=140.6`, fixed altitude enabled

## Batch result

Both attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (2/2). Values are mean +/- sample standard
deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 42.315 +/- 1.560 | 41.212 | 43.418 |
| Path length (m) | 228.259 +/- 5.372 | 224.461 | 232.058 |
| Average speed (m/s) | 5.396 +/- 0.072 | 5.345 | 5.447 |
| Maximum speed (m/s) | 6.165 +/- 0.097 | 6.096 | 6.234 |
| Final error (m) | 0.059 +/- 0.001 | 0.058 | 0.060 |
| Path efficiency (%) | 87.644 +/- 2.063 | 86.185 | 89.102 |

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and
`trial_0001.json`, `trial_0002.json` are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_183259_3884141/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
