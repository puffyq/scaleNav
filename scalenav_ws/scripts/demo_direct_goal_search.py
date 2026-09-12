#!/usr/bin/env python3
"""
Demo: direct odom -> mission_goal graph search with:
  - local graph radius (only in-window nodes expanded)
  - outside window assumed safe for planning (goal-directed heuristic)
  - real-time forward-prefix collision probe on current-frame obstacles

Compares against a simplified multi-terminal enumerator (legacy goalDirectedSearch).

Usage:
  python3 scalenav_ws/scripts/demo_direct_goal_search.py
  python3 scalenav_ws/scripts/demo_direct_goal_search.py --json /tmp/out.json --html /tmp/out.html
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class Node:
    nid: int
    x: float
    y: float

    def as_vec(self) -> Vec2:
        return (self.x, self.y)


@dataclass
class Graph:
    nodes: Dict[int, Node] = field(default_factory=dict)
    neighbors: Dict[int, Set[int]] = field(default_factory=dict)
    edge_len: Dict[Tuple[int, int], float] = field(default_factory=dict)
    edge_cost_scale: Dict[Tuple[int, int], float] = field(default_factory=dict)

    def add_node(self, nid: int, x: float, y: float) -> Node:
        node = Node(nid, x, y)
        self.nodes[nid] = node
        self.neighbors.setdefault(nid, set())
        return node

    def connect(self, a: int, b: int, cost_scale: float = 1.0) -> None:
        na, nb = self.nodes[a], self.nodes[b]
        length = math.hypot(na.x - nb.x, na.y - nb.y)
        self.neighbors[a].add(b)
        self.neighbors[b].add(a)
        self.edge_len[(a, b)] = length
        self.edge_len[(b, a)] = length
        self.edge_cost_scale[(a, b)] = cost_scale
        self.edge_cost_scale[(b, a)] = cost_scale

    def edge_length(self, a: int, b: int) -> float:
        return self.edge_len[(a, b)]

    def edge_cost(self, a: int, b: int) -> float:
        return self.edge_len[(a, b)] * self.edge_cost_scale.get((a, b), 1.0)


def dist2(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def sub(a: Vec2, b: Vec2) -> Vec2:
    return (a[0] - b[0], a[1] - b[1])


def unit(v: Vec2) -> Vec2:
    n = math.hypot(v[0], v[1])
    if n < 1e-9:
        return (0.0, 0.0)
    return (v[0] / n, v[1] / n)


def within_radius(node: Node, center: Vec2, radius: float) -> bool:
    return dist2(node.as_vec(), center) <= radius + 1e-6


def nearest_node(graph: Graph, position: Vec2) -> int:
    best_id = min(graph.nodes, key=lambda nid: dist2(graph.nodes[nid].as_vec(), position))
    return best_id


def path_length(graph: Graph, node_ids: Sequence[int]) -> float:
    total = 0.0
    for i in range(1, len(node_ids)):
        total += graph.edge_length(node_ids[i - 1], node_ids[i])
    return total


def reconstruct(parent: Dict[int, int], end: int) -> List[int]:
    out = [end]
    while end in parent:
        end = parent[end]
        out.append(end)
    out.reverse()
    return out


def forward_route_polyline(
    graph: Graph, path: Sequence[int], vehicle: Vec2
) -> List[Vec2]:
    if len(path) < 2:
        if path:
            return [graph.nodes[path[0]].as_vec()]
        return []

    points = [graph.nodes[nid].as_vec() for nid in path]
    best_dist_sq = float("inf")
    best_progress = 0.0
    total = 0.0
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        seg = sub(b, a)
        seg_len_sq = dot(seg, seg)
        if seg_len_sq < 1e-10:
            continue
        t = max(0.0, min(1.0, dot(sub(vehicle, a), seg) / seg_len_sq))
        proj = (a[0] + t * seg[0], a[1] + t * seg[1])
        d_sq = dist2(vehicle, proj) ** 2
        progress = total + math.sqrt(seg_len_sq) * t
        if d_sq < best_dist_sq:
            best_dist_sq = d_sq
            best_progress = progress
        total += math.sqrt(seg_len_sq)

    clipped: List[Vec2] = []
    walked = 0.0
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        seg_len = dist2(a, b)
        if seg_len < 1e-9:
            continue
        if walked + seg_len <= best_progress + 1e-6:
            walked += seg_len
            continue
        if not clipped:
            t = max(0.0, min(1.0, (best_progress - walked) / seg_len))
            clipped.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        clipped.append(b)
        walked += seg_len
    return clipped if clipped else [points[-1]]


def segment_point_distance(a: Vec2, b: Vec2, p: Vec2) -> float:
    seg = sub(b, a)
    seg_len_sq = dot(seg, seg)
    if seg_len_sq < 1e-10:
        return dist2(a, p)
    t = max(0.0, min(1.0, dot(sub(p, a), seg) / seg_len_sq))
    proj = (a[0] + t * seg[0], a[1] + t * seg[1])
    return dist2(p, proj)


def min_clearance(polyline: Sequence[Vec2], obstacles: Sequence[Vec2]) -> float:
    if not polyline or not obstacles:
        return float("inf")
    best = float("inf")
    for i in range(len(polyline) - 1):
        a, b = polyline[i], polyline[i + 1]
        for obs in obstacles:
            best = min(best, segment_point_distance(a, b, obs))
    for pt in polyline:
        for obs in obstacles:
            best = min(best, dist2(pt, obs))
    return best


def collision_blocked(
    polyline: Sequence[Vec2], obstacles: Sequence[Vec2], clearance: float
) -> bool:
    return min_clearance(polyline, obstacles) < clearance


def search_direct_to_goal(
    graph: Graph,
    start_id: int,
    mission_goal: Vec2,
    vehicle: Vec2,
    local_radius: float,
    min_exec_m: float,
    goal_snap_m: float = 1.5,
) -> Tuple[List[int], int, str]:
    start = graph.nodes[start_id]
    goal_node_id: Optional[int] = None
    best_snap = float("inf")
    for node in graph.nodes.values():
        d = dist2(node.as_vec(), mission_goal)
        if d <= goal_snap_m and d < best_snap and within_radius(node, vehicle, local_radius):
            best_snap = d
            goal_node_id = node.nid

    mission_dir = unit(sub(mission_goal, vehicle))
    g_score: Dict[int, float] = {start_id: 0.0}
    parent: Dict[int, int] = {}
    route_len: Dict[int, float] = {start_id: 0.0}
    progress: Dict[int, float] = {
        start_id: dot(sub(start.as_vec(), vehicle), mission_dir)
    }

    def heuristic(nid: int) -> float:
        return dist2(graph.nodes[nid].as_vec(), mission_goal)

    open_heap: List[Tuple[float, int]] = [(heuristic(start_id), start_id)]
    closed: Set[int] = set()
    best_frontier = start_id
    best_frontier_progress = progress[start_id]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if goal_node_id is not None and current == goal_node_id:
            return reconstruct(parent, current), len(closed), "GOAL_SNAP"

        if (
            route_len[current] + 1e-3 >= min_exec_m
            and progress[current] > best_frontier_progress + 1e-3
        ):
            best_frontier = current
            best_frontier_progress = progress[current]

        for nbr in graph.neighbors[current]:
            if nbr in closed:
                continue
            if not within_radius(graph.nodes[nbr], vehicle, local_radius):
                continue
            tentative_g = g_score[current] + graph.edge_cost(current, nbr)
            if nbr not in g_score or tentative_g < g_score[nbr] - 1e-6:
                g_score[nbr] = tentative_g
                parent[nbr] = current
                route_len[nbr] = route_len[current] + graph.edge_length(current, nbr)
                progress[nbr] = dot(sub(graph.nodes[nbr].as_vec(), vehicle), mission_dir)
                heapq.heappush(open_heap, (tentative_g + heuristic(nbr), nbr))

    reachable = [nid for nid in route_len if route_len[nid] + 1e-3 >= min_exec_m]
    if reachable:
        best_frontier = max(reachable, key=lambda nid: progress.get(nid, -1e9))

    return reconstruct(parent, best_frontier), len(closed), "FRONTIER_TRUNC"


def search_multi_terminal_legacy(
    graph: Graph,
    start_id: int,
    mission_goal: Vec2,
    vehicle: Vec2,
    local_radius: float,
    min_exec_m: float,
) -> Tuple[List[int], int, str]:
    g_score: Dict[int, float] = {start_id: 0.0}
    parent: Dict[int, int] = {}
    route_len: Dict[int, float] = {start_id: 0.0}

    def heuristic(nid: int) -> float:
        return dist2(graph.nodes[nid].as_vec(), mission_goal)

    open_heap: List[Tuple[float, int]] = [(heuristic(start_id), start_id)]
    closed: Set[int] = set()
    candidates: List[int] = []

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current != start_id and route_len[current] + 1e-3 >= min_exec_m:
            candidates.append(current)

        for nbr in graph.neighbors[current]:
            if nbr in closed:
                continue
            if not within_radius(graph.nodes[nbr], vehicle, local_radius):
                continue
            tentative_g = g_score[current] + graph.edge_cost(current, nbr)
            if nbr not in g_score or tentative_g < g_score[nbr] - 1e-6:
                g_score[nbr] = tentative_g
                parent[nbr] = current
                route_len[nbr] = route_len[current] + graph.edge_length(current, nbr)
                heapq.heappush(open_heap, (tentative_g + heuristic(nbr), nbr))

    if not candidates:
        return [start_id], 0, "NO_CANDIDATE"

    terminal = min(candidates, key=lambda nid: g_score[nid] + heuristic(nid))
    return reconstruct(parent, terminal), len(candidates), "MULTI_TERMINAL"


def build_corridor_graph() -> Tuple[Graph, int]:
    """
    Forked 140 m task topology:
      - left narrow corridor x=-5 with higher edge cost (semantic/narrow proxy)
      - right wide corridor x=12 aligned with long-range progress
    """
    g = Graph()
    start = g.add_node(0, 0.0, 0.0)
    left_ids: List[int] = []
    right_ids: List[int] = []
    for i in range(1, 15):
        left_ids.append(g.add_node(i, -5.0, 10.0 * i).nid)
        right_ids.append(g.add_node(100 + i, 12.0, 10.0 * i).nid)
    goal = g.add_node(200, 0.0, 140.0)

    g.connect(start.nid, left_ids[0], cost_scale=1.4)
    g.connect(start.nid, right_ids[0], cost_scale=1.0)
    for i in range(len(left_ids) - 1):
        g.connect(left_ids[i], left_ids[i + 1], cost_scale=1.4)
    for i in range(len(right_ids) - 1):
        g.connect(right_ids[i], right_ids[i + 1], cost_scale=1.0)
    g.connect(left_ids[-1], goal.nid, cost_scale=1.4)
    g.connect(right_ids[-1], goal.nid, cost_scale=1.0)
    return g, start.nid


@dataclass
class TickScenario:
    name: str
    vehicle: Vec2
    local_radius: float
    min_exec_m: float
    clearance_m: float
    obstacles: List[Vec2]


def run_tick(
    graph: Graph,
    mission_goal: Vec2,
    scenario: TickScenario,
) -> dict:
    start_id = nearest_node(graph, scenario.vehicle)
    direct_path, direct_expanded, direct_mode = search_direct_to_goal(
        graph,
        start_id,
        mission_goal,
        scenario.vehicle,
        scenario.local_radius,
        scenario.min_exec_m,
    )
    legacy_path, legacy_candidates, legacy_mode = search_multi_terminal_legacy(
        graph,
        start_id,
        mission_goal,
        scenario.vehicle,
        scenario.local_radius,
        scenario.min_exec_m,
    )

    def summarize(path: Sequence[int], label: str, mode: str, work: int) -> dict:
        pts = [graph.nodes[nid].as_vec() for nid in path]
        forward = forward_route_polyline(graph, path, scenario.vehicle)
        min_clr = min_clearance(forward, scenario.obstacles)
        blocked = collision_blocked(forward, scenario.obstacles, scenario.clearance_m)
        frontier = pts[-1] if pts else scenario.vehicle
        return {
            "label": label,
            "mode": mode,
            "start_node": start_id,
            "node_path": list(path),
            "polyline": pts,
            "forward_polyline": forward,
            "frontier": frontier,
            "path_length_m": path_length(graph, path),
            "forward_length_m": sum(
                dist2(forward[i - 1], forward[i]) for i in range(1, len(forward))
            ),
            "min_clearance_m": min_clr,
            "blocked": blocked,
            "work_units": work,
        }

    direct = summarize(direct_path, "direct", direct_mode, direct_expanded)
    legacy = summarize(legacy_path, "legacy", legacy_mode, legacy_candidates)

    return {
        "scenario": scenario.name,
        "vehicle": scenario.vehicle,
        "local_radius_m": scenario.local_radius,
        "mission_goal": mission_goal,
        "direct": direct,
        "legacy": legacy,
        "direct_on_right_corridor": direct["frontier"][0] > 5.0,
        "legacy_short_left_terminal": legacy["frontier"][0] < 0.0,
        "direct_reaches_farther_than_legacy": direct["path_length_m"] > legacy["path_length_m"] + 5.0,
        "direct_blocked_on_forward_probe": direct["blocked"],
        "legacy_not_blocked_while_off_route": (not legacy["blocked"]) and legacy["path_length_m"] < 20.0,
        "work_direct_leq_legacy_candidates": direct["work_units"] <= max(legacy["work_units"], 1),
    }


def simulate_mission() -> dict:
    graph, _ = build_corridor_graph()
    mission_goal = graph.nodes[200].as_vec()

    scenarios = [
        TickScenario(
            name="t0_start_pick_corridor_frontier",
            vehicle=(0.0, 0.0),
            local_radius=35.0,
            min_exec_m=10.0,
            clearance_m=0.6,
            obstacles=[],
        ),
        TickScenario(
            name="t1_right_prefix_blocked_replan_signal",
            vehicle=(12.0, 25.0),
            local_radius=35.0,
            min_exec_m=10.0,
            clearance_m=0.6,
            obstacles=[(10.0 + 0.08 * i, 34.0 + 0.04 * i) for i in range(30)],
        ),
        TickScenario(
            name="t2_goal_window_direct_snap",
            vehicle=(12.0, 125.0),
            local_radius=35.0,
            min_exec_m=10.0,
            clearance_m=0.6,
            obstacles=[],
        ),
    ]

    ticks = [run_tick(graph, mission_goal, s) for s in scenarios]
    checks = {
        "t0_direct_on_right_corridor": ticks[0]["direct_on_right_corridor"],
        "t0_legacy_stops_at_near_left_bubble": ticks[0]["legacy_short_left_terminal"],
        "t0_direct_frontier_farther_than_legacy": ticks[0]["direct_reaches_farther_than_legacy"],
        "t1_realtime_forward_probe_blocks_right_prefix": ticks[1]["direct_blocked_on_forward_probe"],
        "t2_goal_snap_within_window": dist2(ticks[2]["direct"]["frontier"], mission_goal) < 2.0,
    }
    return {
        "description": (
            "Direct toward-goal search: expand only inside local radius; outside is "
            "assumed safe for planning. Real-time safety applies to the forward execution "
            "prefix only (like EPIC route_blocked probe)."
        ),
        "parameters": {
            "local_graph_radius_m": 35.0,
            "min_execution_path_m": 10.0,
            "clearance_m": 0.6,
            "goal_snap_m": 1.5,
        },
        "ticks": ticks,
        "checks": checks,
        "passed": sum(1 for v in checks.values() if v),
        "total": len(checks),
    }


def render_svg(result: dict, width: int = 980, height: int = 560) -> str:
    tick = result["ticks"][0]
    xs: List[float] = []
    ys: List[float] = []
    for key in ("direct", "legacy"):
        for pt in tick[key]["polyline"] + tick[key]["forward_polyline"]:
            xs.append(pt[0])
            ys.append(pt[1])
    xs.extend([tick["vehicle"][0], tick["mission_goal"][0], -8, 15])
    ys.extend([tick["vehicle"][1], tick["mission_goal"][1], 0, 145])
    min_x, max_x = min(xs) - 5, max(xs) + 5
    min_y, max_y = min(ys) - 5, max(ys) + 5

    def tx(x: float) -> float:
        return (x - min_x) / (max_x - min_x) * (width - 80) + 40

    def ty(y: float) -> float:
        return height - 40 - (y - min_y) / (max_y - min_y) * (height - 80)

    lines = [
        "<svg xmlns='http://www.w3.org/2000/svg' "
        f"width='{width}' height='{height}' viewBox='0 0 {width} {height}'>",
        "<rect width='100%' height='100%' fill='#0f172a'/>",
        "<text x='40' y='28' fill='#e2e8f0' font-size='16' font-family='sans-serif'>"
        "t0: blue=direct toward-goal frontier, orange=legacy multi-terminal</text>",
    ]
    for label, color in (("direct", "#38bdf8"), ("legacy", "#fb923c")):
        poly = tick[label]["polyline"]
        if len(poly) >= 2:
            pts = " ".join(f"{tx(x):.1f},{ty(y):.1f}" for x, y in poly)
            lines.append(
                f"<polyline points='{pts}' fill='none' stroke='{color}' stroke-width='2'/>"
            )
        fwd = tick[label]["forward_polyline"]
        if len(fwd) >= 2:
            pts = " ".join(f"{tx(x):.1f},{ty(y):.1f}" for x, y in fwd)
            lines.append(
                f"<polyline points='{pts}' fill='none' stroke='{color}' "
                f"stroke-width='4' stroke-dasharray='6 4'/>"
            )
    vx, vy = tick["vehicle"]
    gx, gy = tick["mission_goal"]
    lines.append(f"<circle cx='{tx(vx):.1f}' cy='{ty(vy):.1f}' r='6' fill='#22c55e'/>")
    lines.append(
        f"<rect x='{tx(gx)-5:.1f}' y='{ty(gy)-5:.1f}' width='10' height='10' fill='#f472b6'/>"
    )
    y = 48
    for name, ok in result["checks"].items():
        color = "#34d399" if ok else "#f87171"
        lines.append(
            f"<text x='40' y='{y}' fill='{color}' font-size='13' "
            f"font-family='monospace'>{name}: {'PASS' if ok else 'FAIL'}</text>"
        )
        y += 18
    lines.append("</svg>")
    return "\n".join(lines)


def render_html(result: dict) -> str:
    svg = render_svg(result)
    payload = json.dumps(result, indent=2)
    rows = "".join(
        f"<tr><td>{name}</td><td>{'PASS' if ok else 'FAIL'}</td></tr>"
        for name, ok in result["checks"].items()
    )
    return f"""<!doctype html>
<html><head><meta charset='utf-8'>
<title>Direct Goal Search Demo</title>
<style>
body{{font:14px sans-serif;margin:24px;background:#111827;color:#e5e7eb}}
table{{border-collapse:collapse;margin-top:16px}}
td,th{{padding:8px 14px;border-bottom:1px solid #374151;text-align:left}}
pre{{background:#1f2937;padding:16px;overflow:auto;font-size:12px}}
</style></head><body>
<h1>Direct Goal Search Demo</h1>
<p>{result["description"]}</p>
<p><b>{result["passed"]}/{result["total"]}</b> checks passed.</p>
{svg}
<table><tr><th>Check</th><th>Result</th></tr>{rows}</table>
<pre>{payload}</pre>
</body></html>"""


def print_summary(result: dict) -> None:
    print(result["description"])
    print(f"checks: {result['passed']}/{result['total']}")
    for tick in result["ticks"]:
        print(f"\n=== {tick['scenario']} vehicle={tick['vehicle']} ===")
        for key in ("direct", "legacy"):
            r = tick[key]
            print(
                f"  {key:6s} mode={r['mode']:16s} frontier=({r['frontier'][0]:.1f}, "
                f"{r['frontier'][1]:.1f}) path_len={r['path_length_m']:.1f}m "
                f"work={r['work_units']} blocked={r['blocked']} "
                f"min_clr={r['min_clearance_m']:.2f}m"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("/tmp/direct_goal_search_demo.json"),
    )
    parser.add_argument(
        "--html",
        type=Path,
        default=Path("/tmp/direct_goal_search_demo.html"),
    )
    args = parser.parse_args()

    result = simulate_mission()
    args.json.write_text(json.dumps(result, indent=2), encoding="utf-8")
    args.html.write_text(render_html(result), encoding="utf-8")
    print_summary(result)
    print(f"\njson: {args.json}")
    print(f"html: {args.html}")
    return 0 if result["passed"] == result["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
