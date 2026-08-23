# EPIC Online Test Contract

This document defines the input/output contract used by the unit tests, the
integration test, and the timing simulation. A test is not considered useful
unless its input and expected output are explicit.

## Module contracts

| Module | Input | Required output |
| --- | --- | --- |
| Route memory | Ordered world-frame polyline, vehicle position, horizon and lateral tolerance | Only forward route points/edges are retained; points behind the vehicle and parallel corridors outside tolerance are rejected. |
| Raycast shortcut | Witness polyline, sample step, clearance threshold, clearance query | A segment is accepted only when every sampled interior point has finite clearance above threshold; unknown interiors reject the shortcut. |
| Lidar map | 3D hit cloud, free-ray endpoints, pose and quaternion | Hit endpoints remain occupied; ray prefixes become free; a new hit invalidates old free evidence; repeated unchanged input is a no-op. |
| Topo semantic cost | TopoNode endpoints, witness polyline, speculative node score/confidence | Risk is computed on the executed witness polyline and contributes a continuous edge cost. |
| Topo semantic memory | Node, score, EMA alpha and timestamp | Node identity and score persist across node recreation; current low evidence decays stale high evidence. |
| Goal-directed search | Connected TopoGraph, goal, geometry/risk weights and previous edges | A reachable path is returned; a high-risk branch loses to a clear branch; speculative nodes remain valid candidates. |
| Online cadence | Depth/free-ray 10 Hz, odom 50 Hz, semantic 2 Hz, planner 5 Hz, route 0.5 Hz | Exact event counts are produced; semantic input changes the next route decision; rebuild overrun is reported rather than hidden. |

## Integration scenario

The synthetic scenario contains two branches:

```text
start -> direct branch -> goal
start -> side-safe branch -> goal
                 ^
       speculative semantic risk at the direct branch
```

The integration test first checks that the direct branch is selected without
semantic evidence. It then injects one high-confidence speculative observation
and checks that the next planning result selects the side-safe branch. The map
test runs 51 depth/free-ray updates at 10 Hz and checks both occupied and free
space invariants after the stream.

## Timing simulation

`tools/epic_online_sim.py` uses the values in `scripts/epic_online/start.sh`:

```text
depth/free-ray  10 Hz
odom            50 Hz
semantic         2 Hz
planner          5 Hz
route            0.5 Hz
skeleton         1 Hz
```

The rebuild duration is varied deterministically across the 0.67--1.34 s
range observed in the AirSim log. The HTML report therefore shows both the
expected stream counts and the rebuild backlog that explains stale graph use.

Run it with:

```bash
scripts/epic_online/run_simulation.sh
```

Outputs:

```text
log_event/epic_online_sim.json
log_event/epic_online_sim_report.html
```

Open the HTML file directly in a browser. It contains contract results, a
stream timeline, the selected route, and the exact simulation parameters.
