# Map2 Semantic GCN With Memory Enabled — Backup Experiment

This two-trial batch is retained as a backup experiment only, excluded from
the main paper tables and `test_data/aggregate_metrics.csv`. It is paired
with `gcn_semantic_memory_off_20260911_104200` for a descriptive backup
comparison, without replacing or merging with any earlier batch.

## Configuration and provenance

- source batch: `run_20260911_104442_2172019`
- stack: `gcn`
- memory condition: enabled, as explicitly identified by the user
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in both
  trial launch logs)
- prompt: `blocks, walls, box`
- checkpoint reported by both launch logs: `train_gcn/frontier_gcn_map2_35m.pt`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`

The memory-enabled label comes from the user's experiment annotation.
The saved configuration does not identify the enabled memory components or
record map-persistence parameters. Both trial logs report
`geometry_map=SLIDING_WINDOW`; that text alone does not distinguish the
memory-enabled and memory-disabled conditions. No missing parameters are
inferred or inserted into the original configuration, and settings from
the earlier 10:14:53 persistent-map batch are not assumed for this run.

## Batch result

Both attempts are valid and successful (trials 1 and 2), with no collision,
timeout, or excluded attempt. Success rate is 100% (2/2). Values below are
mean +/- sample standard deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 39.877 +/- 6.428 | 35.332 | 44.422 |
| Path length (m) | 194.496 +/- 18.297 | 181.558 | 207.434 |
| Average speed (m/s) | 4.904 +/- 0.332 | 4.670 | 5.139 |
| Maximum speed (m/s) | 6.405 +/- 0.017 | 6.393 | 6.417 |
| Final error (m) | 0.084 +/- 0.016 | 0.073 | 0.095 |
| Path efficiency (%) | 72.301 +/- 6.802 | 67.491 | 77.110 |

This small backup batch does not establish statistically reliable effects
of memory. The backup comparison is recorded in `test_data/README.md`.

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_104442_2172019/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
