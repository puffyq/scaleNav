# Map5 FAR Planner Without Semantic Input

This two-trial FAR Planner batch is archived as a separate Map5 baseline. It
is not mixed with the Map5 GCN results or the Map2 ablation tables.

## Configuration

- source batch: `run_20260911_165048_2446092`
- stack: `far`
- mission: `(0, 0, 170.6)` m to `(0, 200, 170.6)` m
- direct mission distance: `200 m`
- semantic input: disabled (`semantic=false` in `config.json`)
- stored prompt: `building, walls` (inactive because semantic input is disabled)
- timeout: `90 s`

## Batch result

Both attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (2/2). Values are mean +/- sample standard
deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 40.547 +/- 0.134 | 40.452 | 40.642 |
| Path length (m) | 216.928 +/- 1.476 | 215.884 | 217.972 |
| Average speed (m/s) | 5.350 +/- 0.019 | 5.337 | 5.363 |
| Maximum speed (m/s) | 6.299 +/- 0.021 | 6.284 | 6.313 |
| Final error (m) | 0.055 +/- 0.001 | 0.054 | 0.056 |
| Path efficiency (%) | 92.199 +/- 0.628 | 91.755 | 92.642 |

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and
`trial_0001.json`, `trial_0002.json` are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_165048_2446092/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
