# Map2 Semantic GCN With Memory Disabled — Backup Experiment

This two-trial batch is retained as a backup experiment only, excluded from
the main paper tables and `test_data/aggregate_metrics.csv`. It does not
replace or merge with any earlier batch, including the September 11 10:12:14
batch labeled as without full-history map persistence.

## Configuration and provenance

- source batch: `run_20260911_104200_2168931`
- stack: `gcn`
- memory condition: disabled, as explicitly identified by the user
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in both
  trial launch logs)
- prompt: `blocks, walls, box`
- checkpoint reported by both launch logs: `train_gcn/frontier_gcn_map2_35m.pt`
- mission: `(0, 0, 1.6)` m to `(0, 140, 1.6)` m
- timeout: `90 s`
- launch settings: `graph_fixed_layer=true`, `fixed_altitude=true`

The memory-disabled label comes from the user's experiment annotation.
The saved configuration does not identify the disabled memory components or
record map-persistence parameters. Both trial logs report
`geometry_map=SLIDING_WINDOW`; this does not establish that every graph,
route, map, or semantic memory mechanism is disabled. No missing parameters
are inferred or inserted into the original configuration.

## Batch result

Both attempts are valid and successful (trials 1 and 2), with no collision,
timeout, or excluded attempt. Success rate is 100% (2/2). Values below are
mean +/- sample standard deviation over the two successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 33.550 +/- 0.434 | 33.243 | 33.857 |
| Path length (m) | 174.202 +/- 4.056 | 171.334 | 177.069 |
| Average speed (m/s) | 5.192 +/- 0.054 | 5.154 | 5.230 |
| Maximum speed (m/s) | 6.556 +/- 0.221 | 6.400 | 6.713 |
| Final error (m) | 0.073 +/- 0.040 | 0.045 | 0.100 |
| Path efficiency (%) | 80.388 +/- 1.872 | 79.065 | 81.712 |

This small backup batch does not establish statistically reliable effects
of memory. The backup comparison is recorded in `test_data/README.md`.

## Records

The following files are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_104200_2168931/`:

- `config.json`: batch configuration
- `metrics.json`: aggregate batch metrics
- `summary.csv`: per-trial outcomes and flight-log session paths
- `trial_0001.json`, `trial_0002.json`: per-trial terminal records

Raw `trial_0001_stack.log` and `trial_0002_stack.log` remain in the source
batch directory; flight logs remain at the session paths in `summary.csv`.
