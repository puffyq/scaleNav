# Map5 Semantic GCN — Memory Condition Pending

This ten-trial Map5 batch is archived separately from the Map2 experiments.
It is not included in the Map2 tables or `test_data/aggregate_metrics.csv`.

## Configuration

- source batch: `run_20260911_162440_2391420`
- stack: `gcn`
- map/mission: `(0, 0, 170.6)` m to `(0, 200, 170.6)` m
- direct mission distance: `200 m`
- semantics: enabled (`semantic=true` in `config.json`, `pearl=1` in launch logs)
- prompt: `building, walls`
- memory condition: pending confirmation
- timeout: `90 s`
- planning layer: `z=170.6`, fixed altitude enabled

The saved configuration and logs establish the Map5 mission and semantic
condition. The logs report `history=NO_HISTORY`, but prior experiment labeling
has shown that this field must not be used alone to override the user's memory
condition. This batch is therefore not assigned to memory-enabled or
memory-disabled comparisons until confirmed.

## Batch result

Nine of ten valid trials succeeded and one timed out; no collision occurred.
Success rate is 90% and timeout rate is 10%. Successful-flight values are
mean +/- sample standard deviation over the nine successful trials.

| Metric | Mean +/- SD | Minimum | Maximum |
| --- | ---: | ---: | ---: |
| Duration (s) | 48.893 +/- 2.153 | 47.113 | 53.917 |
| Path length (m) | 267.693 +/- 12.227 | 256.330 | 295.035 |
| Average speed (m/s) | 5.475 +/- 0.027 | 5.440 | 5.509 |
| Maximum speed (m/s) | 6.480 +/- 0.362 | 6.143 | 7.148 |
| Final error (m) | 0.057 +/- 0.002 | 0.055 | 0.061 |
| Path efficiency (%) | 74.844 +/- 3.240 | 67.789 | 78.025 |

The failed trial timed out after 90.000 s with an observed path of 523.292 m;
this diagnostic value is not mixed into successful-flight means.

## Records

The archived `config.json`, `metrics.json`, `summary.csv`, and ten
`trial_*.json` records are preserved byte-for-byte from
`scalenav_ws/src/aut_test/results/run_20260911_162440_2391420/`.
Raw stack logs remain in the source directory; flight-log paths are recorded
in `summary.csv`.
