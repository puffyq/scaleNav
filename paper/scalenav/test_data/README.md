# ScaleNav Experiment Data

This directory collects the closed-loop data used for paper analysis. Source
files are copied without changing their contents.

## Closed-loop batches

Most selected batches contain ten valid trial records for the Map2 mission
from `(0, 0, 1.6)` m to `(0, 140, 1.6)` m. Goal acceptance requires position
error at most 0.5 m and settling speed at most 0.3 m/s. Most archived batches
use a 90-s mission timeout. The latest ScaleNav (EGO) regression uses a 200-s
mission timeout, while the latest ScaleNav (SUPER) batch uses a 90-s mission
timeout. All reported closed-loop conditions use this same static map, start,
goal, and ordered obstacle course: a dense small-obstacle field in the first
approximately 25 m, followed by an approximately 60-m-scale block spanning
roughly `y=68..122 m` and `x=-27..26 m`. AirSim is reset before every trial;
the runner accepts the start only after pose, speed, and heading stabilize.

| Directory | Source run | Condition | Outcomes |
| --- | --- | --- | --- |
| `closed_loop/scalenav_semantic_20260904_162304` | `run_20260904_162304_2716212` | ScaleNav, semantic enabled, prompt `blocks, wall` | 10 success |
| `closed_loop/scalenav_semantic_20260829_171743` | `run_20260829_171743_48724` | ScaleNav, semantic enabled, prompt `tree, blocks, wall, line` | 10 success |
| `closed_loop/scalenav_semantic_20260903_204517` | `run_20260903_204517_1504786` | ScaleNav, semantic enabled, prompt `blocks, wall` | 10 success |
| `closed_loop/scalenav_semantic_20260830_105607` | `run_20260830_105607_424081` | Latest previously analyzed ScaleNav semantic batch | 8 success, 1 collision, 1 timeout |
| `closed_loop/scalenav_no_semantic_20260829_172657` | `run_20260829_172657_72293` | ScaleNav, semantic disabled | 7 success, 3 collision |
| `closed_loop/yopo_simple_20260903_085045` | `run_20260903_085045_1991187` | Original YOPO-Simple, direct mission-goal input | 10 collision |
| `closed_loop/ego_20260903_120321` | `run_20260903_120321_2832657` | EGO-Planner, no low-altitude early termination | 10 collision |
| `closed_loop/super_20260902_220642` | `run_20260902_220642_4189191` | SUPER | 10 collision |
| `closed_loop/scalenav_ego_20260904_170548` | `run_20260904_170548_2795763` | ScaleNav graph route layer with EGO execution; 600-s/60-s no-progress protocol | 5 collision, 5 no-progress timeout |
| `closed_loop/scalenav_ego_20260905_184439` | `run_20260905_184439_765736` | Latest ScaleNav (EGO) regression; 200-s mission timeout | 7 success, 3 collision |
| `closed_loop/scalenav_super_20260904_171954` | `run_20260904_171954_2826552` | Superseded ScaleNav graph route layer with SUPER execution; 600-s/60-s no-progress protocol | 1 success, 7 collision, 2 no-progress timeout |
| `closed_loop/scalenav_super_20260906_105512` | `run_20260906_105512_1098813` | Latest ScaleNav (SUPER) regression; 90-s mission timeout | 9 success, 1 collision |
| `closed_loop/gcn_semantic_20260905_162219` | `run_20260905_162219_4136411` | GCN stack with semantic front end enabled; prompt `blocks, wall` | 10 success |
| `closed_loop/gcn_no_semantic_20260905_213245` | `run_20260905_213245_1532617` | GCN stack with PEARL, semantic nodes, and semantic cost disabled | 6 success, 4 collision |
| `closed_loop/gcn_semantic_trees_20260905_215529` | `run_20260905_215529_1641723` | GCN stack with semantic front end enabled; irrelevant-query condition, prompt `trees` | 8 success, 2 collision |
| `closed_loop/scalenav_no_persistent_geometry_20260905_222135` | `run_20260905_222135_1771760` | ScaleNav sliding-geometry ablation (`local_sliding_graph=true`, 40-m radius); semantic enabled | 9 success, 1 collision |
| `closed_loop/scalenav_ego_20260904_110038` | `run_20260904_110038_2376695` | ScaleNav semantic route layer with EGO execution | 9 collision, 1 timeout |
| `closed_loop/scalenav_super_20260904_110656` | `run_20260904_110656_2391835` | ScaleNav semantic route layer with SUPER execution | 5 success, 4 collision, 1 timeout |

### Backup experiments (excluded from main tables)

The September 12 z=60.6 m comparison archives all two ScaleNav flights and
the single FAR flight. See `backup_z60p6_20260912/README.md` for analysis and
the trajectory/observed-point-cloud figure. These runs do not contain a GCN
condition and are not a paired semantic ablation.

| Directory | Source run | Condition | Outcomes |
| --- | --- | --- | --- |
| `closed_loop/scalenav_z60p6_semantic_20260912_183241` | `run_20260912_183241_179330` | ScaleNav, semantic enabled, z=60.6 m, local sliding graph and map radii 40 m; backup only | 2 success |
| `closed_loop/far_z60p6_no_semantic_20260912_183409` | `run_20260912_183409_184093` | FAR, semantic disabled, z=60.6 m; backup only | 1 success |
| `closed_loop/gcn_no_semantic_20260911_094902` | `run_20260911_094902_2114755` | Map2 fixed-altitude GCN, semantic disabled (`semantic=false`, `pearl=0`); two-trial backup batch | 2 success |
| `closed_loop/scalenav_no_semantic_20260911_095047` | `run_20260911_095047_2118305` | Map2 fixed-altitude ScaleNav, semantic disabled (`semantic=false`, launch `semantic=0`); two-trial backup batch | 2 success |
| `closed_loop/gcn_semantic_no_persistent_map_20260911_101214` | `run_20260911_101214_2140124` | Map2 GCN with semantics; without full-history map persistence (user-labeled); two-trial backup batch | 2 success |
| `closed_loop/gcn_semantic_persistent_map_20260911_101453` | `run_20260911_101453_2144759` | Map2 GCN with semantics; full-history map persistence (user-provided console: `persist_map=true`, radius 1000 m); two-trial backup batch | 2 success |
| `closed_loop/gcn_semantic_memory_off_20260911_104200` | `run_20260911_104200_2168931` | Superseded Map2 GCN memory-disabled backup batch; retained for provenance | 2 success |
| `closed_loop/gcn_semantic_memory_off_20260911_114420` | `run_20260911_114420_2214278` | **Selected** replacement Map2 GCN with semantics; memory disabled (user-confirmed) | 2 success |
| `closed_loop/gcn_no_semantic_memory_on_20260911_115325` | `run_20260911_115325_2220771` | **Selected** Map2 GCN without semantics (`pearl=0`), memory enabled (user-corrected; see provenance below) | 10 success |
| `closed_loop/gcn_semantic_memory_on_20260911_120150` | `run_20260911_120150_2234401` | **Selected replacement** Map2 GCN with semantics (`pearl=1`), memory enabled (user-corrected; see provenance below) | 10 success |
| `closed_loop/gcn_map5_semantic_memory_pending_20260911_162440` | `run_20260911_162440_2391420` | Superseded Map5 GCN semantic batch; retained for provenance | 9 success, 1 timeout |
| `closed_loop/gcn_map5_semantic_memory_pending_20260911_165518` | `run_20260911_165518_2454893` | **Selected replacement** Map5 GCN, semantic enabled, prompt `building, walls`; memory condition pending confirmation | 2 success |
| `closed_loop/far_map5_no_semantic_20260911_165048` | `run_20260911_165048_2446092` | Map5 FAR Planner baseline, semantic disabled (`semantic=false`); two-trial batch | 2 success |
| `closed_loop/far_map5_z140p6_no_semantic_20260911_172447` | `run_20260911_172447_2510463` | Map5 FAR Planner baseline at planning height `z=140.6`, semantic disabled; two-trial batch | 2 success |
| `closed_loop/scalenav_map5_semantic_20260911_165737` | `run_20260911_165737_2460029` | Map5 ScaleNav, semantic enabled, prompt `building, walls`, local sliding graph; two-trial batch | 2 success |
| `closed_loop/scalenav_map5_no_semantic_z140p6_20260911_182536` | `run_20260911_182536_3864939` | Map5 ScaleNav at planning height `z=140.6`, semantic disabled, local sliding graph; two-trial batch | 2 success |
| `closed_loop/gcn_map5_z140p6_semantic_20260911_183259` | `run_20260911_183259_3884141` | Map5 GCN at planning height `z=140.6`, semantic enabled, prompt `buildings, walls, obstacles`; two-trial batch | 2 success |
| `closed_loop/gcn_map5_z140p6_no_semantic_20260911_183455` | `run_20260911_183455_3889027` | Map5 GCN at planning height `z=140.6`, semantic disabled, prompt inactive; two-trial batch | 2 success |
| `closed_loop/gcn_semantic_memory_on_20260911_104442` | `run_20260911_104442_2172019` | Superseded Map2 GCN memory-enabled backup batch; retained for provenance | 2 success |
| `closed_loop/gcn_semantic_memory_on_20260911_105050` | `run_20260911_105050_2180000` | **Selected** replacement Map2 fixed-altitude GCN with semantics; memory enabled (user-confirmed) | 2 success |

These backup batches are not included in `aggregate_metrics.csv` or the main
paper tables and do not replace the existing ten-trial main or ablation
batches. Each backup directory includes a README, aggregate metrics, and
per-trial JSON records in addition to the files listed below.

The September 11 GCN memory comparison has semantics enabled in all five batches
(`semantic=true`, `pearl=1`, prompt `blocks, walls, box`). Memory-condition
labels come from the user's experiment annotation and, for the persistent
batch, the supplied launcher console output; the saved `config.json` files
do not record map-persistence parameters. Without full-history persistence
does not mean all graph, route, or semantic memory is disabled.
The later `104200`, `104442`, and `105050` batches are labeled "memory
disabled", "memory enabled", and "memory enabled", respectively, by the user;
their exact memory-component settings are not recorded in `config.json`. They
form a separate backup comparison and are not merged with or substituted for
the earlier batches.

| Memory condition | Success | Duration (s) | Path (m) | Path efficiency (%) |
| --- | ---: | ---: | ---: | ---: |
| Without full-history persistence | 2/2 | 34.387 +/- 0.403 | 178.918 +/- 0.775 | 78.249 +/- 0.339 |
| With full-history persistence | 2/2 | 33.253 +/- 0.902 | 171.448 +/- 5.670 | 81.702 +/- 2.702 |
| Memory disabled (replacement, `114420`) | 2/2 | 34.502 +/- 0.755 | 175.021 +/- 3.743 | 80.009 +/- 1.711 |
| Memory enabled (replacement, `105050`) | 2/2 | 34.222 +/- 0.195 | 177.578 +/- 0.280 | 78.839 +/- 0.124 |

Values are mean +/- sample standard deviation over successful trials. These
two-trial batches are descriptive backup results, not evidence of a
statistically established performance improvement.

The `105050` batch is the selected memory-enabled result and replaces the
earlier `104442` batch. The `114420` batch is the selected memory-disabled
result and replaces the earlier `104200` batch. Superseded files remain
archived for provenance and are not used in the selected comparison.

The `115325` and `120150` batches form the selected ten-trial memory-enabled
semantic ablation: both complete 10/10 trials. The semantic-on batch records
`173.224 +/- 3.637 m` and `33.787 +/- 0.848 s`; the semantic-off batch records
`173.988 +/- 9.336 m` and `32.825 +/- 2.302 s`.

The user corrected both batches to memory enabled. Their original logs
contain `history=NO_HISTORY`, while `config.json` does not record the memory
switch. The user-confirmed condition is used for experiment labeling; the
conflicting log text is preserved, and its cause remains unresolved. These
batches are not evidence for a no-memory condition. Their semantic-ablation
rows are already used in `root.tex`; `aggregate_metrics.csv` still contains
the older batches and has not been regenerated.

| Semantic input | History memory | Success | Duration (s) | Path (m) | Path efficiency (%) |
| --- | --- | ---: | ---: | ---: | ---: |
| Disabled | Enabled (user-corrected) | 10/10 | 32.825 +/- 2.302 | 173.988 +/- 9.336 | 80.663 +/- 4.100 |
| Enabled | Enabled (user-corrected) | 10/10 | 33.787 +/- 0.848 | 173.224 +/- 3.637 | 80.852 +/- 1.705 |

Each directory contains:

- `summary.csv`: one row per trial, copied from the automated test output.
- `config.json`: mission and test-run configuration.

`aggregate_metrics.csv` is regenerated from the current-scene `summary.csv`
files with `scalenav_ws/src/aut_test/aggregate_results.py`.
Completion time, path length, average speed, and maximum speed in the
`successful_*` columns use successful trials only. Early collision duration
and path must not be interpreted as completion performance; they are retained
separately, with mean and sample standard deviation, in the
`failure_observed_*` columns for diagnostic use. The paper denotes the latter
path as $L_{obs}$.

`graph_memory_over_time.csv` is a raw per-snapshot export generated from one
complete ScaleNav session with
`scalenav_ws/docs/test_reports/analyze_graph_memory.py`. They report graph
nodes/edges and the retained cumulative map over time. Because process RSS was
not logged, memory columns are explicitly payload proxies: retained map points
times the logged 16-byte point stride, serialized graph-snapshot bytes, and
their sum. Cumulative point-cloud files are reported separately as on-disk log
cost and are not presented as runtime memory.

The paper figure `pics/experiments/map2_0_140_1p6/graph_resource_scaling.pdf`
compares the active online working set and module wall times between the
persistent-geometry batch (`run_20260905_130923_2952687`, 27 valid sessions)
and the sliding-geometry ablation (`run_20260905_222135_1771760`, 10 valid
sessions). Its summary values are in `graph_resource_scaling.csv`, and it is
generated by `scalenav_ws/docs/test_reports/plot_graph_resource_scaling.py`.
The batches use the same fixed mission but were scheduled separately; the
figure is a workload profile rather than a paired causal timing estimate.
Retained global nodes are deliberately excluded from
the active-search node count. `graph_resource_timeline.csv` contains the P50
and P95 local-node time series used in panel (a), with each valid session
normalized to 0--100% before aggregation.

Reproduce the raw export with:

```bash
python3 scalenav_ws/docs/test_reports/analyze_graph_memory.py \
  --session log_scalenav/session_20260904_162306_809 \
  --output-csv paper/scalenav/test_data/graph_memory_over_time.csv \
  --output-svg /tmp/graph_memory_diagnostic.svg
```

The generic automated-test configuration records semantic fields for every
stack. They are not applicable to YOPO-Simple, EGO-Planner, or SUPER; those
baselines do not run the ScaleNav semantic front end.

## Legacy validation

`legacy_validation/` preserves the 2026-08-28 single-run pipeline validation
and its supporting flight-statistics CSV files. It is retained because the
current trajectory figure and older manuscript text refer to it. It is not a
ten-trial aggregate and must not be combined with the closed-loop batches
above.

Excluded data include startup failures, interrupted batches, smoke tests, and
superseded EGO runs that used the removed low-altitude early-termination rule.
