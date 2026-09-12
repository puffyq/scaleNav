# Map2 Semantic GCN With Memory Disabled — Replacement Backup Experiment

This two-trial batch is retained as a backup experiment only, excluded from
the main paper tables and `test_data/aggregate_metrics.csv`. It replaces the
earlier memory-disabled backup batch `run_20260911_104200_2168931` in the
working comparison; the earlier source files remain archived separately.

## Configuration and provenance

- source batch: `run_20260911_114420_2214278`
- stack: `gcn`
- memory condition: disabled, as confirmed by the user
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in both
  trial launch logs)
- prompt: `blocks, walls, box`
- checkpoint reported by both launch logs: `train_gcn/frontier_gcn_map2_35m.pt`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`

The memory-disabled label is confirmed by the user. The saved configuration
does not record which memory components are disabled or map-persistence
parameters. Both trial logs report `geometry_map=SLIDING_WINDOW`; that text
alone does not identify every memory component. No missing parameters are
inferred or inserted into the original configuration.

## Batch result

Both attempts are valid and successful (trials 1 and 2), with no collision,
timeout, or excluded attempt. Success rate is 100% (2/2). Values below are
mean +/- sample standard deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 34.502 +/- 0.755 | 33.968 | 35.036 |
| Path length (m) | 175.021 +/- 3.743 | 172.374 | 177.668 |
| Average speed (m/s) | 5.073 +/- 0.003 | 5.071 | 5.075 |
| Maximum speed (m/s) | 6.394 +/- 0.010 | 6.387 | 6.401 |
| Final error (m) | 0.086 +/- 0.004 | 0.084 | 0.089 |
| Path efficiency (%) | 80.009 +/- 1.711 | 78.799 | 81.219 |

This small replacement batch does not establish statistically reliable
effects of memory. Its values replace the earlier `104200` row in the
working comparison only.

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_114420_2214278/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
