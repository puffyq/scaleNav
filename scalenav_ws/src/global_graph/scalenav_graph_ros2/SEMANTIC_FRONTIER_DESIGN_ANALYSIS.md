# Semantic Frontier Design Analysis

## 1. Requirement

One PEARL heatmap frame is a `3 x 5` image partition:

- three rows: upper, middle, lower;
- five columns: five horizontal directions;
- each patch score is the arithmetic mean of its finite heatmap pixels.

ScaleNav is currently planar and normally flies forward. Therefore only the
middle row participates in frontier selection. Each frame contributes exactly
five virtual semantic alternatives, one per column, projected at the same
configured radial distance. Upper and lower rows remain available to the
diagnostic view but do not create additional frontier goals.

Depth is a separate product. If a synchronized depth sample exists, it may
project the same patch onto the measured surface and annotate an existing
ordinary Verified node. That measured projection is not one of the five
frontier alternatives and must never be promoted to a virtual frontier just
because it has semantic observations.

## 2. Why The Current Behavior Is Wrong

The latest complete pre-fix real session available during this analysis
(`session_20260903_124647_923`) shows:

- frame 0 contains five directional scores `0.340..0.363`;
- the planner reports `selected_semantic_column=1` and
  `best_semantic_frontier_id=30`;
- the published A* path nevertheless ends at `(-5.51, 15.18)`, an ordinary
  measured topology position rather than any 35 m virtual candidate;
- an offline replay of the same RGB/depth/heatmap/graph inputs under this
  contract selects 35 m candidate V2, while the historical endpoint is
  `19.33 m` from its nearest valid virtual ray.

The statistics and the submitted route are therefore not the same decision.
The old implementation also used `semantic_observations > 0` as a proxy for
node type. A depth projection can carry semantic observations while remaining
ordinary geometry, so this proxy can both select the wrong initial goal and
apply the semantic-edge collision check to ordinary-to-ordinary backbone
edges.

That session also visualizes the old optical-Z projection: its outer rays are
approximately `(x=+/-28 m, y=35.5 m)` from the initial pose. They exceed the
configured 35 m local search radius and are not equal-range alternatives. It
is diagnostic evidence for the bug, not a post-fix acceptance run.

## 3. Candidate Model

Every middle-row column is represented by a record with:

```
frame_stamp, column, patch_mean, confidence, virtual_position,
measured_position(optional), reachable, astar_route_cost,
mission_goal_distance, objective, rejection_reason
```

The virtual position is always retained independently of depth validity. A
candidate is usable only when its projection, timestamp, and odom-to-candidate
graph connection are valid. An invalid or blocked column is unavailable; it is
not assigned a synthetic zero score.

The graph stores the source explicitly:

```
is_virtual_semantic = true  => fixed-depth semantic frontier endpoint
is_virtual_semantic = false => measured semantic annotation / ordinary node
```

Only `Unknown && is_virtual_semantic` nodes can be semantic frontier goals or
participate in ordinary-to-semantic direct-edge validation. A Verified node
with semantic evidence remains an ordinary backbone node.

## 4. Selection Algorithm

Selection is continuous and covers every score configuration without a
special-case state machine. For each active heatmap frame `f`, let

```
m_f = min(score of usable columns in f)
relative_risk(j) = score_j - m_f
```

The frame minimum represents the absolute quality of the safest direction in
that observation. The relative term represents the directional difference
between the five columns. The semantic contribution for column `j` is:

```
semantic_term(f,j) =
    frontier_semantic_frame_weight * m_f
  + frontier_semantic_relative_weight * relative_risk(j)
```

With equal default weights this is exactly the raw patch mean, written as an
absolute frame term plus an intra-frame difference so the two meanings remain
explicit. It has the required limiting behavior without hard low/high
thresholds:

- if all five scores are low, the absolute semantic term is small and the
  normalized route/mission-distance terms decide among similarly safe paths;
- if one or more columns are safer than the others, the relative term prefers
  the lowest column in that frame;
- if an entire current frame is high, its frame minimum is high and an active
  older frame with a lower minimum can win;
- if all five scores are equal, `relative_risk` is zero for all columns and
  deterministic route/mission-distance tie breaking applies.

Do not normalize every non-uniform frame to a full `0..1` risk span. That
would make a harmless difference such as `0.10` versus `0.11` as influential
as a real difference such as `0.05` versus `0.90`, contradicting the rule that
distance dominates when all alternatives are similarly low.

For every reachable virtual candidate, use the dimensionless objective:

```
route_cost / scale
+ frontier_goal_distance_weight * mission_goal_distance / scale
+ semantic_term(f,j)
```

where `scale = max(1 m, current_odom_to_mission_goal_distance)`. Route cost is
the actual A* cost from the odom node to that candidate, including edge
clearance and semantic-field penalties. This prevents raw metres from changing
meaning as the vehicle advances.

Ordinary Verified nodes are not normal frontier goals. They are an explicit
safety fallback only when no active virtual candidate is reachable. Near the
mission endpoint, the terminal rule sets the frontier goal to the mission goal
itself rather than selecting an ordinary node.

## 5. Temporal And Route Rules

- Virtual semantic memory remains eligible for the configured memory window
  (currently 1.5 s) and keeps its frame/column identity.
- A route is held until its configured progress trigger or a real edge failure;
  a new heatmap callback alone does not replan.
- A replan replaces the complete route segment. It never splices a new prefix
  onto an old route.
- A* always starts at the current odom node and may backtrack when that is the
  least-cost collision-free way around an obstacle.
- The committed frontier goal, selected node type, frame, column, score,
  route cost, mission distance, and objective must be logged from the same
  candidate record. Search diagnostics must not report a different winner.

## 6. Safety Rules

Every virtual candidate must pass the ordinary-backbone connection test before
it can be ranked. A failed direct ordinary-to-virtual edge is removed and the
candidate becomes unreachable for that search. Only ordinary-to-virtual edges
receive this direct semantic edge check; ordinary-to-ordinary Verified edges
retain their EPIC witness and are never reclassified because one endpoint has a
measured semantic annotation.

## 7. Test Matrix

The module test replays frames and checks both the five-column math and the
online decision:

1. uniform low scores: route/mission terms choose the best reachable column;
2. uniform high scores: an older active frame with a lower minimum wins;
3. mixed scores: the lowest column in the frame wins when route terms are
   comparable;
4. equal-score ties: deterministic column tie break;
5. missing/invalid column: unavailable, never treated as zero;
6. stale frame: excluded after the memory window;
7. blocked virtual edge: candidate rejected before ranking;
8. measured depth annotation: ordinary node, never a semantic frontier;
9. no reachable virtual candidate: explicit ordinary safety fallback is logged;
10. terminal horizon: mission goal is the frontier goal;
11. progress hold: unchanged route between progress triggers;
12. route replacement: replan starts from odom and does not reuse a stale
    route prefix.

The HTML report must show RGB, depth, heatmap, all `3 x 5` patch means, the
five middle-row virtual candidates, measured projections separately, the
candidate objective table, the online A* node route, and the exact committed
frontier endpoint for every replay frame.
