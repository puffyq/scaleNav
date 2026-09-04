# ScaleNav Experiment Data

This directory collects the closed-loop data used for paper analysis. Source
files are copied without changing their contents.

## Closed-loop batches

All selected batches contain ten completed trial records for the Map2 mission
from `(0, 0, 1.6)` m to `(0, 140, 1.6)` m. Goal acceptance requires position
error at most 0.5 m and settling speed at most 0.3 m/s. The mission timeout is
90 s.

| Directory | Source run | Condition | Outcomes |
| --- | --- | --- | --- |
| `closed_loop/scalenav_semantic_20260829_171743` | `run_20260829_171743_48724` | ScaleNav, semantic enabled, prompt `tree, blocks, wall, line` | 10 success |
| `closed_loop/scalenav_semantic_20260903_204517` | `run_20260903_204517_1504786` | ScaleNav, semantic enabled, prompt `blocks, wall` | 10 success |
| `closed_loop/scalenav_semantic_20260830_105607` | `run_20260830_105607_424081` | Latest previously analyzed ScaleNav semantic batch | 8 success, 1 collision, 1 timeout |
| `closed_loop/scalenav_no_semantic_20260829_172657` | `run_20260829_172657_72293` | ScaleNav, semantic disabled | 7 success, 3 collision |
| `closed_loop/yopo_simple_20260903_085045` | `run_20260903_085045_1991187` | Original YOPO-Simple, direct mission-goal input | 10 collision |
| `closed_loop/ego_20260903_120321` | `run_20260903_120321_2832657` | EGO-Planner, no low-altitude early termination | 10 collision |
| `closed_loop/super_20260902_220642` | `run_20260902_220642_4189191` | SUPER | 10 collision |
| `closed_loop/scalenav_ego_20260904_110038` | `run_20260904_110038_2376695` | ScaleNav semantic route layer with EGO execution | 9 collision, 1 timeout |
| `closed_loop/scalenav_super_20260904_110656` | `run_20260904_110656_2391835` | ScaleNav semantic route layer with SUPER execution | 5 success, 4 collision, 1 timeout |

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
