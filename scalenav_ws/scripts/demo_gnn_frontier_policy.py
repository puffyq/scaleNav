#!/usr/bin/env python3
"""Minimal GNN policy demo for ScaleNav-style topology graphs.

The policy ranks five semantic frontier columns.  It is deliberately not a
planner: the safety mask represents checks that must still be performed by
Bubble geometry and A*.  Replace ``build_demo_graph`` with a converter from
TopoNode/TopoGraph or a logged graph snapshot when integrating it.

Install (in the training environment):
    pip install torch torch-geometric

Examples:
    python3 scalenav_ws/scripts/demo_gnn_frontier_policy.py
    python3 scalenav_ws/scripts/demo_gnn_frontier_policy.py --train-steps 80
    python3 scalenav_ws/scripts/demo_gnn_frontier_policy.py \
        --session log_scalenav/session_20260903_130005_740
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class DemoNode:
    node_id: int
    x: float
    y: float
    radius: float
    semantic_score: float = 0.0
    semantic_confidence: float = 0.0
    column: int = -1
    is_frontier: bool = False
    is_odom: bool = False


def build_demo_graph(frame: int = 0):
    """Build a small planar graph with one odom node and five frontiers."""
    # The frame-dependent scores imitate a changing heatmap.  Column 2 is the
    # best long-term direction, while column 4 is deliberately tempting in
    # the first frame so the recurrent state has something to remember.
    score_sets = (
        [0.62, 0.48, 0.31, 0.52, 0.22],
        [0.58, 0.43, 0.29, 0.49, 0.35],
        [0.55, 0.39, 0.25, 0.46, 0.34],
    )
    scores = score_sets[min(frame, len(score_sets) - 1)]
    nodes: List[DemoNode] = [
        DemoNode(0, 0.0, 0.0, 2.0, is_odom=True),
        DemoNode(1, 0.0, 8.0, 1.7),
        DemoNode(2, -5.0, 12.0, 1.5),
        DemoNode(3, 5.0, 12.0, 1.5),
    ]
    for column, score in enumerate(scores):
        # The five columns fan out in the world XY plane.
        x = (column - 2) * 5.0
        nodes.append(DemoNode(
            10 + column, x, 25.0 + abs(column - 2) * 1.5, 1.4,
            semantic_score=score, semantic_confidence=0.95,
            column=column, is_frontier=True,
        ))

    # Undirected topology edges.  Edge weights are geometric lengths and are
    # passed to GCNConv as normalized adjacency weights.
    edges: List[Tuple[int, int]] = [(0, 1), (1, 2), (1, 3)]
    for column in range(5):
        parent = 2 if column <= 1 else 3 if column >= 3 else 1
        edges.append((parent, 10 + column))
    safe_columns = [True, True, True, True, True]
    # This is the place where a real integration should use:
    # edge_clearance >= vehicle_radius + margin AND A* reachability.
    if frame == 1:
        safe_columns[3] = False
    return nodes, edges, safe_columns


def make_data(torch, Data, nodes: Sequence[DemoNode],
              edges: Sequence[Tuple[int, int]], safe_columns: Sequence[bool],
              goal: Tuple[float, float] = (0.0, 80.0)):
    """Convert the demo graph to a PyG Data object."""
    # TopoNode persistent IDs are not contiguous tensor indices in the real
    # graph, so always build an explicit lookup table before making edge_index.
    index_of = {node.node_id: index for index, node in enumerate(nodes)}
    feature_rows: List[List[float]] = []
    degree: Dict[int, int] = {node.node_id: 0 for node in nodes}
    for left, right in edges:
        degree[left] += 1
        degree[right] += 1
    for node in nodes:
        distance_goal = math.hypot(node.x - goal[0], node.y - goal[1])
        feature_rows.append([
            node.x / 25.0, node.y / 80.0, node.radius / 3.0,
            min(degree[node.node_id], 8) / 8.0,
            node.semantic_score, node.semantic_confidence,
            float(node.is_frontier), float(node.column >= 0) * node.column / 4.0,
            math.hypot(node.x, node.y) / 80.0, distance_goal / 80.0,
            float(node.is_odom), float(node.column == 2),
        ])

    directed: List[Tuple[int, int]] = []
    weights: List[float] = []
    for left_id, right_id in edges:
        left, right = index_of[left_id], index_of[right_id]
        distance = math.hypot(nodes[left].x - nodes[right].x,
                              nodes[left].y - nodes[right].y)
        weight = 1.0 / max(distance, 1e-3)
        directed.extend([(left, right), (right, left)])
        weights.extend([weight, weight])
    edge_index = torch.tensor(directed, dtype=torch.long).t().contiguous()
    edge_weight = torch.tensor(weights, dtype=torch.float32)
    frontier_index = torch.tensor(
        [index_of[node.node_id] for node in nodes if node.is_frontier],
        dtype=torch.long)
    columns = torch.tensor(
        [node.column for node in nodes if node.is_frontier], dtype=torch.long)
    safe = torch.tensor(safe_columns, dtype=torch.bool)
    return Data(
        x=torch.tensor(feature_rows, dtype=torch.float32),
        edge_index=edge_index,
        edge_weight=edge_weight,
        frontier_index=frontier_index,
        frontier_columns=columns,
        safe_columns=safe,
    )


def _parse_log_json(line: str):
    """Read logger JSON, whose diagnostic fields may contain inf/nan."""
    return json.loads(line, parse_constant=lambda value: float(value))


def load_log_frames(session: str, max_frames: int = 0):
    """Load planner frames and their nearest graph/odom snapshots."""
    index_path = os.path.join(session, "index.jsonl")
    entries = []
    with open(index_path, encoding="utf-8") as stream:
        for line in stream:
            try:
                entries.append(_parse_log_json(line))
            except json.JSONDecodeError:
                continue
    graphs = [e for e in entries if e.get("kind") == "graph" and e.get("file")]
    odom = [e for e in entries if e.get("kind") == "odom"]
    timings = [e for e in entries
               if e.get("kind") == "timing"
               and e.get("data", {}).get("module") == "planner"
               and e.get("data", {}).get("searched")
               and e.get("data", {}).get("selected_semantic_column", -1) >= 0]
    frames = []
    for timing in timings[:max_frames or None]:
        stamp = timing["stamp_ns"]
        graph = min(graphs, key=lambda item: abs(item["stamp_ns"] - stamp))
        pose = min(odom, key=lambda item: abs(item["stamp_ns"] - stamp))
        graph_path = os.path.join(session, graph["file"])
        with open(graph_path, encoding="utf-8") as stream:
            snapshot = json.load(stream)
        frames.append((timing, snapshot, pose["data"]["position"]))
    return frames


def map_oracle_label(nodes, edges, vehicle_position):
    """Derive a geometry-only five-column label from the logged free-space map."""
    adjacency = {node.node_id: [] for node in nodes}
    for left, right in edges:
        adjacency.setdefault(left, []).append(right)
        adjacency.setdefault(right, []).append(left)
    start = min(nodes, key=lambda node: math.dist(
        (node.x, node.y), vehicle_position[:2])).node_id
    reachable = {start}
    queue = [start]
    while queue:
        current = queue.pop()
        for neighbor in adjacency.get(current, []):
            if neighbor not in reachable:
                reachable.add(neighbor)
                queue.append(neighbor)
    # The log starts aligned with +Y in world coordinates. Columns are
    # left-to-right angles [-40,-20,0,20,40] degrees around that direction.
    limits = math.radians(50.0)
    best = [-float("inf")] * 5
    for node in nodes:
        if node.node_id not in reachable:
            continue
        dx = node.x - vehicle_position[0]
        dy = node.y - vehicle_position[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dx, dy)
        if dy < 3.0 or distance < 3.0 or abs(angle) > limits:
            continue
        column = int(round((angle / limits + 1.0) * 2.0))
        column = max(0, min(4, column))
        best[column] = max(best[column], dy - 0.20 * abs(dx))
    safe = [math.isfinite(value) for value in best]
    if not any(safe):
        return 2, [True] * 5
    return max(range(5), key=lambda column: best[column]), safe


def build_log_graph(torch, Data, timing, snapshot, vehicle_position,
                    label_source: str = "map"):
    """Convert one RViz graph marker snapshot into a policy Data object."""
    node_marker = next((m for m in snapshot["markers"]
                        if m.get("ns") == "scalenav_skeleton_nodes"), None)
    edge_marker = next((m for m in snapshot["markers"]
                        if m.get("ns") == "scalenav_skeleton_edges"), None)
    if node_marker is None or not node_marker.get("points"):
        raise ValueError("graph snapshot has no scalenav_skeleton_nodes")
    points = [tuple(float(v) for v in point) for point in node_marker["points"]]
    nodes = [DemoNode(i, point[0], point[1], max(node_marker["scale"][0] / 2, 0.1))
             for i, point in enumerate(points)]
    odom_index = min(range(len(nodes)), key=lambda i: math.dist(
        (nodes[i].x, nodes[i].y), vehicle_position[:2]))
    nodes[odom_index] = DemoNode(**{**nodes[odom_index].__dict__, "is_odom": True})
    edges = set()
    edge_points = edge_marker.get("points", []) if edge_marker else []
    for offset in range(0, len(edge_points) - 1, 2):
        left = tuple(float(v) for v in edge_points[offset])
        right = tuple(float(v) for v in edge_points[offset + 1])
        a = min(range(len(nodes)), key=lambda i: math.dist(left, (nodes[i].x, nodes[i].y, 1.6)))
        b = min(range(len(nodes)), key=lambda i: math.dist(right, (nodes[i].x, nodes[i].y, 1.6)))
        if a != b:
            edges.add((a, b))

    ranking = timing["data"].get("semantic_frontier_ranking", [])
    ranking_by_column = {int(item.get("column", -1)): item for item in ranking}
    frontier_marker = next((m for m in snapshot["markers"]
                            if m.get("ns") == "scalenav_frontier_goal"
                            and m.get("action") == 0), None)
    center = (frontier_marker or {}).get("pose", {}).get("position", vehicle_position)
    base_id = -1
    safe = [False] * 5
    for column in range(5):
        item = ranking_by_column.get(column, {})
        # Candidate positions are diagnostic only: the graph snapshot does not
        # store semantic-column coordinates, so spread them around the logged goal.
        x = float(center[0]) + (column - 2) * 3.0
        y = float(center[1])
        objective = float(item.get("objective", float("inf")))
        score = 1.0 / (1.0 + max(float(item.get("risk", 1.0)), 0.0))
        candidate = DemoNode(base_id, x, y, 1.0, score, 1.0, column, True)
        nearest = min(range(len(nodes)), key=lambda i: math.dist(
            (nodes[i].x, nodes[i].y), (x, y)))
        nodes.append(candidate)
        edges.add((nodes[nearest].node_id, base_id))
        safe[column] = math.isfinite(objective)
        base_id -= 1
    map_target, map_safe = map_oracle_label(nodes[:len(points)],
                                            [(a, b) for a, b in edges
                                             if a >= 0 and b >= 0],
                                            vehicle_position)
    planner_target = int(timing["data"]["selected_semantic_column"])
    selected_safe = map_safe if label_source == "map" else safe
    target = map_target if label_source == "map" else planner_target
    return (make_data(torch, Data, nodes, sorted(edges), selected_safe,
                      goal=(0.0, 140.0)), target, planner_target, map_target)


def load_torch_geometric():
    try:
        import torch
        from torch import nn
        from torch_geometric.data import Data
        from torch_geometric.nn import GCNConv
        return torch, nn, Data, GCNConv, True
    except ImportError:
        # Keep the demo runnable in YOPO-Rally's environment, which may have
        # PyTorch but not the optional PyG package. This fallback implements
        # the same weighted message aggregation used by a small GCN layer.
        import torch
        from torch import nn

        class Data:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)

        class GCNConv(nn.Module):
            def __init__(self, input_dim: int, output_dim: int):
                super().__init__()
                self.linear = nn.Linear(input_dim, output_dim)

            def forward(self, x, edge_index, edge_weight=None):
                source, target = edge_index
                if edge_weight is None:
                    edge_weight = torch.ones(source.shape[0], device=x.device)
                messages = x[source] * edge_weight[:, None]
                aggregate = torch.zeros_like(x)
                aggregate.index_add_(0, target, messages)
                degree = torch.zeros(x.shape[0], device=x.device)
                degree.index_add_(0, target, edge_weight)
                aggregate = aggregate / degree.clamp_min(1e-6)[:, None]
                return self.linear(aggregate)

        return torch, nn, Data, GCNConv, False


def run(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-steps", type=int, default=40,
                        help="synthetic supervised steps; use 0 for inference only")
    parser.add_argument("--session", type=str, default="",
                        help="ScaleNav log session directory for real-graph replay")
    parser.add_argument("--log-frames", type=int, default=0,
                        help="maximum searched planner frames to replay (0=all)")
    parser.add_argument("--log-train-steps", type=int, default=10,
                        help="imitation epochs over log frames")
    parser.add_argument("--label-source", choices=("map", "planner"), default="map",
                        help="real-map geometry oracle or logged planner decision")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)

    torch, nn, Data, GCNConv, using_pyg = load_torch_geometric()
    torch.manual_seed(args.seed)
    random.seed(args.seed)

    class FrontierPolicy(nn.Module):
        """GCN frame encoder + GRU temporal memory + five-column head."""

        def __init__(self, input_dim: int = 12, hidden_dim: int = 48):
            super().__init__()
            self.conv1 = GCNConv(input_dim, hidden_dim)
            self.conv2 = GCNConv(hidden_dim, hidden_dim)
            self.gru = nn.GRUCell(hidden_dim + 5, hidden_dim)
            self.head = nn.Sequential(
                nn.Linear(hidden_dim * 2, hidden_dim), nn.ReLU(), nn.Linear(hidden_dim, 1)
            )

        def forward(self, data, hidden=None, previous_column: int = -1):
            x = torch.relu(self.conv1(data.x, data.edge_index, data.edge_weight))
            x = torch.relu(self.conv2(x, data.edge_index, data.edge_weight))
            graph_embedding = x.mean(dim=0, keepdim=True)
            previous = torch.zeros((1, 5), device=x.device)
            if 0 <= previous_column < 5:
                previous[0, previous_column] = 1.0
            hidden = self.gru(torch.cat([graph_embedding, previous], dim=-1), hidden)
            frontier_embeddings = x[data.frontier_index]
            temporal = hidden.expand(frontier_embeddings.shape[0], -1)
            frontier_values = self.head(
                torch.cat([frontier_embeddings, temporal], dim=-1)).squeeze(-1)
            logits = torch.full((5,), torch.finfo(x.dtype).min, device=x.device)
            logits[data.frontier_columns] = frontier_values
            # Invalid columns are hard-masked.  This is where the real system
            # must inject Bubble clearance and A* reachability results.
            safe = data.safe_columns.to(logits.device)
            logits = logits.masked_fill(~safe, torch.finfo(logits.dtype).min)
            return logits, hidden

    model = FrontierPolicy()
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-3)
    hidden = None
    previous_column = -1

    print(f"backend={'PyG' if using_pyg else 'pure-torch fallback'}")

    if args.session:
        frames = load_log_frames(args.session, args.log_frames)
        if not frames:
            raise RuntimeError("no searched planner frames with selected semantic columns")
        print(f"log_session={args.session} frames={len(frames)}")
        # This is deliberately an imitation check: it measures whether the
        # network can represent the currently logged decisions, not whether
        # those decisions are globally optimal.
        for epoch in range(max(0, args.log_train_steps)):
            model.train()
            log_hidden = None
            previous = -1
            losses = []
            for timing, snapshot, position in frames:
                data, target, _, _ = build_log_graph(
                    torch, Data, timing, snapshot, position, args.label_source)
                logits, log_hidden = model(data, log_hidden, previous)
                loss = nn.functional.cross_entropy(logits[None, :],
                                                   torch.tensor([target]))
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                log_hidden = log_hidden.detach()
                previous = target
                losses.append(float(loss))
            if epoch == 0 or epoch + 1 == args.log_train_steps:
                print(f"log epoch={epoch + 1:03d} mean_loss={sum(losses) / len(losses):.4f}")

        model.eval()
        log_hidden = None
        previous = -1
        selected = []
        print("\nlog replay:")
        for timing, snapshot, position in frames:
            data, target, planner_target, map_target = build_log_graph(
                torch, Data, timing, snapshot, position, args.label_source)
            with torch.no_grad():
                logits, log_hidden = model(data, log_hidden, previous)
                prediction = int(torch.argmax(logits).item())
            selected.append((target, prediction, planner_target, map_target))
            info = timing["data"]
            print(f"  seq={timing['seq']} pos=({position[0]:.2f},{position[1]:.2f}) "
                  f"planner={planner_target} map={map_target} predicted={prediction} "
                  f"reason={info.get('switch_reason','')}")
            previous = prediction
        matches = sum(item[0] == item[1] for item in selected)
        target_switches = sum(item[0] != selected[i - 1][0]
                              for i, item in enumerate(selected) if i)
        planner_switches = sum(item[2] != selected[i - 1][2]
                               for i, item in enumerate(selected) if i)
        predicted_switches = sum(item[1] != selected[i - 1][1]
                                 for i, item in enumerate(selected) if i)
        print(f"\nlog imitation accuracy={matches / len(selected):.3f} "
              f"({matches}/{len(selected)}) target_switches={target_switches} "
              f"planner_switches={planner_switches} "
              f"predicted_switches={predicted_switches}")
        print(f"label_source={args.label_source}; map labels use reachable skeleton "
              "nodes and maximum forward progress per column.")
        print("Candidate coordinates are reconstructed around the logged frontier "
              "marker because the snapshot omits semantic-column poses.")
        return 0

    # The target is an example label: in a real run it should come from future
    # progress/backtracking outcomes, not from the current handcrafted policy.
    target_column = 2
    for step in range(max(0, args.train_steps)):
        model.train()
        frame = step % 3
        nodes, edges, safe = build_demo_graph(frame)
        data = make_data(torch, Data, nodes, edges, safe)
        logits, hidden = model(data, hidden, previous_column)
        loss = nn.functional.cross_entropy(logits[None, :],
                                           torch.tensor([target_column]))
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        hidden = hidden.detach()
        previous_column = int(torch.argmax(logits).item())
        if step == 0 or step + 1 == args.train_steps:
            print(f"train step={step + 1:03d} loss={loss.item():.4f} "
                  f"predicted_column={previous_column}")

    model.eval()
    hidden = None
    previous_column = -1
    print("\nsequence:")
    for frame in range(3):
        nodes, edges, safe = build_demo_graph(frame)
        data = make_data(torch, Data, nodes, edges, safe)
        with torch.no_grad():
            logits, hidden = model(data, hidden, previous_column)
            probabilities = torch.softmax(logits, dim=0)
            selected = int(torch.argmax(logits).item())
        print(f"  frame={frame} scores="
              f"{[round(n.semantic_score, 3) for n in nodes if n.is_frontier]} "
              f"safe={safe} values={[round(float(v), 3) for v in logits]} "
              f"prob={[round(float(v), 3) for v in probabilities]} "
              f"selected_column={selected}")
        previous_column = selected

    print("\nThe selected column is only a learned preference. Run A* and the "
          "final Bubble/witness collision check before executing it.")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
