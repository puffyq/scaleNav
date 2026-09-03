# Semantic Frontier Selection Contract

## 1. Evidence model

One heatmap frame is a `3 x 5` grid: upper, middle, and lower rows, with five
horizontal directions in the columns. A patch score is the arithmetic mean of
its finite pixels. In planar forward flight, only the middle row is a frontier
candidate; therefore a valid frame contributes at most five candidates, one
for each horizontal column. Upper and lower rows may be retained for
diagnostics, but must not change the planar candidate scores or directions.

For each middle-row column, the configured virtual distance always produces
the semantic frontier candidate. All five rays have the same Euclidean range
from the camera; an off-axis column must not silently exceed the local A*
radius. Synchronized depth may additionally project the
same column onto an observed surface for ordinary-node semantic annotation;
that measured point never replaces or removes the virtual candidate. Every
candidate keeps its frame timestamp, column index, score, confidence, and
projected position.

Semantic evidence and node type are separate properties. A Verified geometric
node may carry a semantic score from a measured depth projection, but it
remains an ordinary backbone node. Only an Unknown node created by the
fixed-depth projection is a virtual semantic frontier endpoint. Code must not
use `semantic_observations > 0` by itself as a node-type test.

The score used for relative choice is the patch mean. Frame-wide baseline
subtraction or max-pixel selection must not change the ordering of the five
columns. Confidence can suppress invalid/poorly synchronized evidence, but it
does not multiply the risk of an otherwise valid candidate. In particular,
the lower FOV confidence of an outer column must not make a high-risk direction
appear safer.

## 2. Complete candidate handling

The planner evaluates all active semantic frames and all five columns; it does
not encode only three named cases. The following rules cover the combinations:

* A column with invalid pixels, invalid projection, expired timestamp, or no
  collision-free graph connection is unavailable. Missing columns do not get a
  synthetic zero score.
* Every available column is a frontier candidate. The semantic score is a
  continuous cost term; there is no low/high threshold gate that removes a
  candidate from ranking.
* All candidates in the frame use the same weighted objective. Lower semantic
  score is preferred, while route cost and mission-goal distance can outweigh
  a small score difference according to their configured weights.
* A frame with fewer than two usable columns is still valid, but receives no
  special diversity bonus. This avoids inventing evidence when only one ray is
  available.
* Ordinary Verified nodes are not normal frontier goals. They are selected only
  when no active semantic candidate from any frame is reachable. This fallback
  is explicit in diagnostics.

The five columns are relative alternatives from one observation. Their
semantic term can be written as `frame_min + (score - frame_min)`, which equals
the raw patch mean with equal weights. Do not stretch every frame's smallest
and largest score to `0..1`: doing so would make tiny low-risk differences
dominate distance. Treating each column as an unrelated untyped global
frontier also recreates the current failure mode.

## 3. Objective and temporal stability

For candidates that remain eligible after connectivity filtering, use one
dimensionless objective:

```
(A* route cost + frontier_goal_distance_weight * distance_to_mission_goal)
  / max(1 m, current_to_mission_goal_distance)
  + frontier_semantic_score_weight * patch_mean
```

The mission-distance term is therefore comparable across replans and cannot be
silently overwhelmed by metres of route length. The semantic term participates
continuously in the same comparison as the distance terms.

The active route segment is held until its progress/replan condition is met;
selection must not oscillate on every heatmap callback. At a replan, retain the
incumbent column/frame when its candidate remains eligible and the new winner
does not improve the objective by a configured margin. When switching columns,
use a deterministic tie break (objective, then greater forward mission
progress, then smaller column change) so equal evidence cannot cause left/right
flapping. Expiry of the incumbent removes this hold immediately.

Temporal route holding and deterministic tie breaks still prevent oscillation,
but no threshold hysteresis is applied to semantic scores.

## 4. Safety and reachability

Candidate endpoint classification never bypasses graph connectivity. Every
ordinary-to-semantic edge must have a current collision-free witness; a failed
witness removes that edge before frontier ranking. A* starts at the odometry
node and uses the same edge costs and semantic-risk penalties as execution.
The selected route is the fresh A* node route for that replan; no stale route
prefix or witness path is spliced into it. If DP bubble shortcutting removes
intermediate nodes for execution, the complete unshortened A* sequence remains
the accepted route's topology contract. The shortened sequence is kept
separately for node-center trajectory generation; a shortcut chord must never
be checked as though it were a `neighbors_`/`paths_` topology edge.

The direct endpoint check applies only between an Unknown virtual semantic
endpoint and a Verified/Odom backbone endpoint. Edges between two Verified
backbone nodes keep their EPIC witness semantics even when one or both nodes
carry measured semantic annotations. Reclassifying such an edge as a semantic
chord can delete a valid curved corridor, invalidate the accepted route, and
bypass the configured progress trigger through repeated `ROUTE_UNREACHABLE`
replans.

A valid route may initially move opposite the mission direction to leave a
dead end or go around a large obstacle. Backtracking is charged through the A*
route cost; it is not rejected by a per-edge forward-direction gate.

Reachability of an accepted route has two independent parts.  If the endpoint
is an Unknown virtual semantic frontier, the preceding node is its Verified
anchor.  A BFS from the current odometry node to that anchor may traverse only
Odom/Verified backbone nodes with current topology edges and witnesses.  The
final Verified-to-Unknown edge is a provisional terminal connection: it is
allowed for frontier selection and execution after its live safety check, but
it must never make the Verified prefix appear connected.  Unknown nodes are
terminal-only and cannot bridge two disconnected Verified components.

Holding an accepted route therefore requires `verified_prefix_reachable` and,
when present, `terminal_unknown_edge_usable`.  The diagnostic
`accepted_head_edge_usable` is not a hold requirement: the rolling odom edge
to the route's original second node is expected to disappear as the vehicle
moves. This does not
reject Unknown routes.  It rejects only a missing Verified prefix or a missing
or blocked final provisional edge.  Stable interior edges of the accepted A*
node sequence must still exist; when one disappears, the planner runs a fresh
odom-rooted A* instead of publishing the stale node sequence or splicing an
alternate prefix into it.

The rolling odom connection is checked after odom reconnection. Endpoint BFS
may discover an alternate Verified path to the old anchor; that is sufficient
to keep the already-progressing route segment. A fresh A* is required only
when the Verified prefix is disconnected or the terminal ordinary-semantic
edge fails its live safety check.

## 5. Diagnostics and tests

Each frame log must report all five patch means, validity, projected depth,
column, risk class, reachability, objective, and the final winner/rejection
reason. Frame-by-frame tests must cover uniform low, uniform high, mixed scores,
ties/deadband, missing columns, stale frames, invalid depth, blocked semantic
edges, ordinary fallback, mission-distance normalization, and incumbent hold.

The real-log HTML must distinguish the online A* endpoint from its offline
diagnostic recomputation and identify whether the online endpoint matches a
fixed-depth virtual candidate or a measured surface projection. This prevents
a measured annotation accidentally selected as a semantic frontier from being
reported as a valid five-column virtual choice.
