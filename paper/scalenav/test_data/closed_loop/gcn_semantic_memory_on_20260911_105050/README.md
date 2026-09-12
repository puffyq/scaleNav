# Map2 Semantic GCN With Memory Enabled — Replacement Backup Experiment

This two-trial batch is retained as a backup experiment only, excluded from
the main paper tables and `test_data/aggregate_metrics.csv`. It replaces the
earlier memory-enabled backup batch `run_20260911_104442_2172019` in the
working comparison; the earlier source files remain archived separately.

## Configuration and provenance

- source batch: `run_20260911_105050_2180000`
- stack: `gcn`
- memory condition: enabled, as confirmed by the user
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in both
  trial launch logs)
- prompt: `blocks, walls, box`
- checkpoint reported by both launch logs: `train_gcn/frontier_gcn_map2_35m.pt`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`

The memory-enabled label is confirmed by the user. The saved configuration
does not record the enabled memory components or map-persistence settings.
Both trial logs report `geometry_map=SLIDING_WINDOW`; that text alone does
not distinguish individual memory components. No missing parameters are
inferred or inserted into the original configuration.

## Batch result

Both attempts are valid and successful (trials 1 and 2), with no collision,
timeout, or excluded attempt. Success rate is 100% (2/2). Values below are
mean +/- sample standard deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 34.222 +/- 0.195 | 34.084 | 34.360 |
| Path length (m) | 177.578 +/- 0.280 | 177.380 | 177.775 |
| Average speed (m/s) | 5.189 +/- 0.038 | 5.162 | 5.216 |
| Maximum speed (m/s) | 6.404 +/- 0.040 | 6.376 | 6.432 |
| Final error (m) | 0.075 +/- 0.038 | 0.048 | 0.102 |
| Path efficiency (%) | 78.839 +/- 0.124 | 78.751 | 78.927 |

This small replacement batch does not establish statistically reliable
effects of memory. Its values replace the earlier `104442` row in the
working comparison only.

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_105050_2180000/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
