# ScaleNav paper metric audit

This audit uses the closed-loop conventions in EGO-Planner and SUPER as the
external baseline.  SUPER defines success as reaching the goal without a
collision while satisfying the velocity and acceleration constraints, and
reports success/safe rate, flight speed, and mapping/planning computation time.
Its released logger records position, velocity, acceleration, jerk, and replan
time.  ScaleNav should use the same primary quantities and reserve semantic and
route-memory metrics for its own ablations.

## Decision

| Metric | Status | Required definition or action |
|---|---|---|
| Success, safe, collision | Valid; collision observable in `scalenav_log.v2` | Use `/sim/collision` for collision. Supply kinematic-validity and timeout labels until those constraints are configured in the logger. Never infer collision from clearance. |
| Completion time `T` | Valid | Time from the one-way mission command to reaching `0.5 m` and slowing below `0.3 m/s`. Do not include a return leg. |
| Path length `L` | Valid | Integrate consecutive 3-D odometry positions over the same active interval as `T`. |
| Average speed | Valid | Use `L/T`, not the last instantaneous speed printed by EPIC. |
| Path efficiency | Valid after definition | Use `eta_SPL = S L*/max(L,L*)` across all trials. Report raw `L`, `T`, and speed on successful trials; assign failures zero SPL instead of allowing early termination to look efficient. |
| Planning latency | Valid in `scalenav_log.v2` | Use every `/epic/timing` planner event for mean/P99. For old logs, throttled ROS timing percentiles remain provisional. |
| Minimum clearance | Secondary diagnostic | Use clearance at the vehicle position. Planned-witness clearance is not executed-flight clearance and is not collision ground truth. |
| Acceleration and jerk | Diagnostic with current log | Odom-derived filtered values are estimates. In `scalenav_log.v2`, timestamped `control` records also include commanded acceleration and finite-difference jerk. |
| Route switches | ScaleNav ablation only | Count committed route generations/reasons from structured planner timing. Local-goal changes are not route switches. |
| Semantic exposure and decision distance | ScaleNav ablation only | Require world-frame risk samples and scenario branch labels. The raw image heatmap alone is insufficient. |
| PEARL latency, drops, memory | Valid system metrics, not observable here | Add structured per-frame latency/drop counters and process RSS; RGB-to-semantic count ratio is not a drop counter. |

## Current paper findings

1. The offline corridor-conditioned numbers in `root.tex` are traceable to
   `TRAINING_REPORT_002.md`: 1,800 independent zero-motion samples, 6 collisions,
   8.67% corridor violation, 1.358 m mean minimum clearance, and 4.230 m mean
   progress.  They are reasonable as an offline single-step result only.
2. Calling that model the "current checkpoint" is stale.  The repository now
   contains `YOPO_54` and `TRAINING_REPORT_004.md`; the older result should be
   called the "reported checkpoint" unless the paper is updated to the newer
   benchmark.
3. The current online launcher loads
   `scalenav_ws/src/models/original_yopo_simple/model.pt`.  Closed-loop logs from
   that launcher cannot be labeled as Corridor-Conditioned YOPO results.
4. The previous statement that failed trials participate directly in raw path
   and time means is biased: a method that crashes early obtains a short path
   and time.  Report raw flight quantities success-conditioned, and use
   success/SPL plus timeout-capped time across all trials.
5. At least 20 paired seeds, mean, sample standard deviation, 95% confidence
   intervals, and a paired test are reasonable.  The test protocol still needs
   a fixed timeout, collision source, goal schedule, compute hardware, and the
   exact number of bootstrap resamples.

## Analyzer

Run a single session:

```bash
python3 paper/scalenav/analyze_flight_logs.py \
  log_scalenav/session_20260828_111856_991 \
  --output-dir paper/scalenav/generated/latest
```

Run a batch with outcome labels:

```bash
python3 paper/scalenav/analyze_flight_logs.py log_scalenav/session_* \
  --metadata-csv trial_labels.csv \
  --output-dir paper/scalenav/generated/benchmark_a
```

The optional CSV columns are:

```text
session,method,scenario,seed,collision,kinematic_violation,timeout
```

The analyzer writes `flight_metrics.json`, `flight_metrics.csv`, and a
paper-oriented `flight_metrics.md`.  It analyzes the first non-trivial one-way
mission by default; `--mission-mode all` is available only for multi-leg log
diagnosis.  For v2 sessions it reads full-rate structured timing directly.  It
auto-matches a nearby EPIC ROS log for old sessions and marks throttled timing
percentiles as sampled.
