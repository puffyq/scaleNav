#!/usr/bin/env python3
"""Collect graph samples and geometry labels from ScaleNav log sessions."""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from heapq import heappop, heappush

import torch


SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(SCRIPT_DIR, "scalenav_ws", "scripts"))
from demo_gnn_frontier_policy import build_log_graph, load_log_frames  # noqa: E402


def prepare_global_oracle(snapshot, goal, goal_tolerance=8.0):
    """Build one final-session graph and cache distances to the mission goal."""
    marker = next((m for m in snapshot["markers"]
                   if m.get("ns") == "scalenav_skeleton_nodes"), None)
    edge_marker = next((m for m in snapshot["markers"]
                        if m.get("ns") == "scalenav_skeleton_edges"), None)
    if not marker or not marker.get("points"):
        return None
    points = [tuple(float(v) for v in p) for p in marker["points"]]
    adjacency = {i: set() for i in range(len(points))}
    edge_points = edge_marker.get("points", []) if edge_marker else []
    for offset in range(0, len(edge_points) - 1, 2):
        left, right = edge_points[offset], edge_points[offset + 1]
        a = min(range(len(points)), key=lambda i: math.dist(left, points[i]))
        b = min(range(len(points)), key=lambda i: math.dist(right, points[i]))
        if a != b:
            adjacency[a].add(b)
            adjacency[b].add(a)
    goal_id = min(range(len(points)), key=lambda i: math.dist(points[i][:2], goal[:2]))
    if math.dist(points[goal_id][:2], goal[:2]) > goal_tolerance:
        return None
    distances = {i: float("inf") for i in range(len(points))}
    distances[goal_id] = 0.0
    pending = [(0.0, goal_id)]
    while pending:
        distance, current = heappop(pending)
        if distance != distances[current]:
            continue
        for neighbor in adjacency[current]:
            edge = math.dist(points[current], points[neighbor])
            candidate = distance + edge
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                heappush(pending, (candidate, neighbor))
    return points, adjacency, distances


def global_oracle_target(cache, position, orientation):
    """Return the first-hop column on the cached global shortest route."""
    if cache is None:
        raise ValueError("global graph is unavailable")
    points, adjacency, distances = cache
    start = min(range(len(points)), key=lambda i: math.dist(points[i][:2], position[:2]))
    yaw = math.atan2(2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]),
                     1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2))
    limit = math.radians(50.0)
    costs = [float("inf")] * 5
    for neighbor in adjacency[start]:
        if not math.isfinite(distances[neighbor]):
            continue
        dx = points[neighbor][0] - points[start][0]
        dy = points[neighbor][1] - points[start][1]
        angle = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                           math.cos(yaw) * dx + math.sin(yaw) * dy)
        if abs(angle) > limit:
            continue
        column = max(0, min(4, int(round((angle / limit + 1.0) * 2.0))))
        costs[column] = min(costs[column], math.hypot(dx, dy) + distances[neighbor])
    if not any(math.isfinite(value) for value in costs):
        raise ValueError("global graph has no feasible route to mission goal")
    return min(range(5), key=lambda column: costs[column])


def load_final_graph(session):
    """Load the last graph snapshot recorded in the complete session."""
    graphs = []
    with open(os.path.join(session, "index.jsonl"), encoding="utf-8") as stream:
        for line in stream:
            try:
                item = json.loads(line, parse_constant=lambda value: float(value))
            except json.JSONDecodeError:
                continue
            if item.get("kind") == "graph" and item.get("file"):
                graphs.append(item)
    if not graphs:
        return None
    record = max(graphs, key=lambda item: item.get("stamp_ns", 0))
    with open(os.path.join(session, record["file"]), encoding="utf-8") as stream:
        return json.load(stream)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs", nargs="*", default=None,
                        help="session directories; default: log_scalenav/session_*")
    parser.add_argument("--output", default="train_gcn/dataset.pt")
    parser.add_argument("--max-frames-per-session", type=int, default=0)
    parser.add_argument("--oracle-scope", choices=("global", "current"), default="global",
                        help="GT map scope: final accumulated session graph (global) or frame graph")
    args = parser.parse_args()

    sessions = args.logs or sorted(glob.glob("log_scalenav/session_*"))
    sessions = [path for path in sessions if os.path.isfile(os.path.join(path, "index.jsonl"))]
    if not sessions:
        raise SystemExit("no sessions containing index.jsonl")

    samples = []
    skipped = 0
    for session in sessions:
        try:
            frames = load_log_frames(session, args.max_frames_per_session)
        except (OSError, ValueError):
            skipped += 1
            continue
        # The final logged graph is the offline global map for this session.
        # Inputs remain frame-local snapshots; only the oracle target uses it.
        global_snapshot = load_final_graph(session) if frames else None
        global_cache = prepare_global_oracle(global_snapshot, frames[-1][4]) if global_snapshot else None
        for timing, snapshot, position, orientation, goal in frames:
            try:
                data, target, planner_target, map_target = build_log_graph(
                    torch, _Data, timing, snapshot, position, orientation, goal,
                    "map", allow_unreachable=True)
                if args.oracle_scope == "global":
                    if global_cache is None:
                        raise ValueError("final session map does not cover mission goal")
                    target = global_oracle_target(global_cache, position, orientation)
                    map_target = target
            except (KeyError, ValueError, IndexError):
                skipped += 1
                continue
            samples.append({
                "x": data.x.cpu(),
                "edge_index": data.edge_index.cpu(),
                "edge_weight": data.edge_weight.cpu(),
                "frontier_index": data.frontier_index.cpu(),
                "frontier_columns": data.frontier_columns.cpu(),
                "safe_columns": data.safe_columns.cpu(),
                "target": int(target),
                "planner_target": int(planner_target),
                "map_target": int(map_target),
                "session": os.path.basename(session),
                "seq": int(timing["seq"]),
                "position": list(position),
            })
        print(f"session={session} frames={len(frames)} collected={len(samples)}")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    torch.save({"schema": "scalenav_gcn_dataset.v1", "samples": samples,
                "sessions": sessions}, args.output)
    print(f"wrote={args.output} samples={len(samples)} skipped={skipped}")


class _Data:
    """Minimal Data constructor expected by build_log_graph."""

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


if __name__ == "__main__":
    main()
