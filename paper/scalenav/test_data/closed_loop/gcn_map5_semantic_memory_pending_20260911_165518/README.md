# Map5 Semantic GCN — Selected Replacement Backup Experiment

This two-trial batch replaces the earlier Map5 GCN semantic batch
`run_20260911_162440_2391420` in the working Map5 comparison. The earlier
batch remains archived for provenance. Map5 results are kept separate from
the Map2 experiments and are not included in the Map2 tables.

## Configuration

- source batch: `run_20260911_165518_2454893`
- stack: `gcn`
- mission: `(0, 0, 170.6)` m to `(0, 200, 170.6)` m
- direct mission distance: `200 m`
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in the
  launch log)
- prompt: `building, walls`
- memory condition: pending confirmation
- timeout: `90 s`
- planning layer: `z=170.6`, fixed altitude enabled

## Batch result

Both attempts are valid and successful; no collision, timeout, or excluded
attempt. Success rate is 100% (2/2). Values are mean +/- sample standard
deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 42.550 +/- 1.025 | 41.825 | 43.274 |
| Path length (m) | 231.028 +/- 5.711 | 226.989 | 235.066 |
| Average speed (m/s) | 5.430 +/- 0.003 | 5.427 | 5.432 |
| Maximum speed (m/s) | 6.145 +/- 0.064 | 6.100 | 6.191 |
| Final error (m) | 0.060 +/- 0.002 | 0.059 | 0.062 |
| Path efficiency (%) | 86.596 +/- 2.141 | 85.082 | 88.110 |

The memory condition remains pending because this message did not specify
whether Map5 route/map memory was enabled. No condition is inferred from the
Map2 experiments or from omitted launcher parameters.

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and
`trial_0001.json`, `trial_0002.json` are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_165518_2454893/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
