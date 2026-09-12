# Map5 ScaleNav With Semantic Input

This two-trial batch is archived as a separate Map5 ScaleNav result. It is
not mixed with Map5 GCN or FAR results, and is not included in the Map2
tables.

## Configuration

- source batch: `run_20260911_165737_2460029`
- stack: `scalenav`
- mission: `(0, 0, 170.6)` m to `(0, 200, 170.6)` m
- direct mission distance: `200 m`
- semantics: enabled (`semantic=true` in `config.json`, `semantic=1` in the
  launch log)
- prompt: `building, walls`
- timeout: `90 s`
- planning layer: `z=170.6`, fixed altitude enabled
- local graph: sliding, radius `40 m`
- map history: launch log reports `map_history_radius_m=40.0` and
  `map_persist_history=false`

## Batch result

Both attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (2/2). Values are mean +/- sample standard
deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 42.498 +/- 0.006 | 42.493 | 42.502 |
| Path length (m) | 229.058 +/- 1.699 | 227.857 | 230.259 |
| Average speed (m/s) | 5.390 +/- 0.039 | 5.362 | 5.418 |
| Maximum speed (m/s) | 6.643 +/- 0.436 | 6.335 | 6.952 |
| Final error (m) | 0.058 +/- 0.003 | 0.056 | 0.060 |
| Path efficiency (%) | 87.317 +/- 0.648 | 86.859 | 87.774 |

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and
`trial_0001.json`, `trial_0002.json` are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_165737_2460029/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
