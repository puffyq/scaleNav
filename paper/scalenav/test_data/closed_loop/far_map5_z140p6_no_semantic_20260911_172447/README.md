# Map5 FAR Planner at z=140.6 Without Semantic Input

This two-trial batch is archived as a separate FAR Map5 baseline at a
different planning height. It must not replace or be pooled with the earlier
`z=170.6` FAR batch, because the start and goal states differ.

## Configuration

- source batch: `run_20260911_172447_2510463`
- stack: `far`
- mission: `(0, 0, 140.6)` m to `(0, 200, 140.6)` m
- direct mission distance: `200 m`
- semantic input: disabled (`semantic=false` in `config.json`)
- stored prompt: `buildings, walls, obstacles` (inactive because semantics are disabled)
- timeout: `90 s`

## Batch result

Both attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (2/2). Values are mean +/- sample standard
deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 45.188 +/- 0.291 | 44.982 | 45.394 |
| Path length (m) | 230.862 +/- 0.590 | 230.445 | 231.279 |
| Average speed (m/s) | 5.109 +/- 0.046 | 5.077 | 5.142 |
| Maximum speed (m/s) | 6.258 +/- 0.026 | 6.240 | 6.277 |
| Final error (m) | 0.058 +/- 0.002 | 0.057 | 0.060 |
| Path efficiency (%) | 86.632 +/- 0.221 | 86.476 | 86.789 |

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and
`trial_0001.json`, `trial_0002.json` are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_172447_2510463/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
