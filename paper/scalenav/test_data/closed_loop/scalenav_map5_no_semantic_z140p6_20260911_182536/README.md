# Map5 ScaleNav at z=140.6 Without Semantic Input

This two-trial batch is archived as a separate Map5 ScaleNav no-semantic
condition. It is paired with the semantic-enabled ScaleNav batch at the same
planning height and is not mixed with the `z=170.6` results or Map2 tables.

## Configuration

- source batch: `run_20260911_182536_3864939`
- stack: `scalenav`
- mission: `(0, 0, 140.6)` m to `(0, 200, 140.6)` m
- direct mission distance: `200 m`
- semantic input: disabled (`semantic=false`, `semantic=0` in the launch log)
- stored prompt: `buildings, walls, obstacles` (inactive because semantics are disabled)
- timeout: `90 s`
- planning layer: `z=140.6`, fixed altitude enabled
- local graph: sliding, radius `40 m`
- map history: launch log reports `map_history_radius_m=40.0` and
  `map_persist_history=false`

## Batch result

Both attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (2/2). Values are mean +/- sample standard
deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 45.487 +/- 0.052 | 45.451 | 45.524 |
| Path length (m) | 251.551 +/- 0.285 | 251.350 | 251.753 |
| Average speed (m/s) | 5.530 +/- 0.013 | 5.521 | 5.539 |
| Maximum speed (m/s) | 6.303 +/- 0.019 | 6.289 | 6.317 |
| Final error (m) | 0.055 +/- 0.000 | 0.055 | 0.055 |
| Path efficiency (%) | 79.507 +/- 0.090 | 79.443 | 79.570 |

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and
`trial_0001.json`, `trial_0002.json` are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_182536_3864939/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
