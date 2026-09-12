#!/usr/bin/env python3
"""Collect a prompt-conditioned 3-D semantic GCN dataset from ScaleNav logs."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from heapq import heappop, heappush
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scalenav_ws" / "src" / "scalenav"))
from text_tracker.heatmap import sample_heatmap_at_body_directions  # noqa: E402
from text_tracker.pearl_adapter import PEARLHeatmapEncoder  # noqa: E402


def neighbors26():
    return [
        (dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz))
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]


N26 = neighbors26()


def inflate_occupancy(occupied, radius_cells):
    out = occupied.copy()
    offsets = [
        (dx, dy, dz)
        for dx in range(-radius_cells, radius_cells + 1)
        for dy in range(-radius_cells, radius_cells + 1)
        for dz in range(-radius_cells, radius_cells + 1)
        if dx * dx + dy * dy + dz * dz <= radius_cells * radius_cells
    ]
    source = np.argwhere(occupied)
    shape = occupied.shape
    for dx, dy, dz in offsets:
        cells = source + np.array([dx, dy, dz])
        valid = (
            (cells[:, 0] >= 0) & (cells[:, 0] < shape[0]) &
            (cells[:, 1] >= 0) & (cells[:, 1] < shape[1]) &
            (cells[:, 2] >= 0) & (cells[:, 2] < shape[2])
        )
        cells = cells[valid]
        out[cells[:, 0], cells[:, 1], cells[:, 2]] = 1
    return out


def astar(start, goal, blocked, max_expanded=2_000_000):
    shape = blocked.shape
    if blocked[start] or blocked[goal]:
        return []
    h = lambda a: math.dist(a, goal)
    queue = [(h(start), 0.0, start)]
    cost = {start: 0.0}
    parent = {}
    expanded = 0
    while queue and expanded < max_expanded:
        _, g, u = heappop(queue)
        if g != cost.get(u):
            continue
        expanded += 1
        if u == goal:
            route = [u]
            while route[-1] != start:
                route.append(parent[route[-1]])
            return route[::-1]
        for dx, dy, dz, step in N26:
            v = (u[0] + dx, u[1] + dy, u[2] + dz)
            if (
                not (0 <= v[0] < shape[0] and 0 <= v[1] < shape[1] and 0 <= v[2] < shape[2])
                or blocked[v]
            ):
                continue
            ng = g + step
            if ng < cost.get(v, float("inf")):
                cost[v] = ng
                parent[v] = u
                heappush(queue, (ng + h(v), ng, v))
    return []


def world_to_cell(point, bounds, resolution):
    return tuple(int(math.floor((float(point[i]) - bounds[i][0]) / resolution)) for i in range(3))


def cell_to_world(cell, bounds, resolution):
    return np.asarray(
        [bounds[i][0] + (cell[i] + 0.5) * resolution for i in range(3)],
        dtype=np.float32,
    )


def route_target(route, start_index, resolution, lookahead):
    distance = 0.0
    for i in range(start_index, len(route) - 1):
        distance += math.dist(route[i], route[i + 1]) * resolution
        if distance >= lookahead:
            return route[i + 1]
    return route[-1] if route else None


def class_for_direction(delta, yaw):
    dx, dy, dz = map(float, delta)
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = c * dx + s * dy, -s * dx + c * dy
    azimuth = math.atan2(by, bx)
    elevation = math.atan2(dz, max(math.hypot(bx, by), 1e-6))
    col = max(0, min(4, int(round(2.0 - azimuth / math.radians(20.0)))))
    row = 0 if elevation > math.radians(10.0) else 2 if elevation < -math.radians(10.0) else 1
    return row * 5 + col


def yaw_from_quaternion(orientation):
    x, y, z, w = orientation
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def parse_entries(session):
    entries = []
    with open(os.path.join(session, "index.jsonl"), encoding="utf-8") as stream:
        for line in stream:
            try:
                entries.append(json.loads(line, parse_constant=lambda x: float(x)))
            except json.JSONDecodeError:
                pass
    return entries


def nearest(records, stamp_ns):
    return min(records, key=lambda item: abs(item["stamp_ns"] - stamp_ns))


def graph_snapshot(entry):
    with open(entry["file"], encoding="utf-8") as stream:
        return json.load(stream)


def make_sample(position, yaw, goal, route, route_index, blocked, bounds, resolution, semantic_ranking):
    target_cell = route_target(route, route_index, resolution, 35.0)
    if target_cell is None:
        return None
    target_world = cell_to_world(target_cell, bounds, resolution)
    target = class_for_direction(target_world - position, yaw)

    node_marker = None
    edge_marker = None
    for marker in semantic_ranking["snapshot"]["markers"]:
        if marker.get("ns") == "scalenav_skeleton_nodes":
            node_marker = marker
        elif marker.get("ns") == "scalenav_skeleton_edges":
            edge_marker = marker
    if node_marker is None or not node_marker.get("points"):
        return None

    points = [tuple(float(v) for v in point) for point in node_marker["points"]]
    nodes = np.asarray(points, dtype=np.float32)
    if len(nodes) == 0:
        return None
    edges = set()
    if edge_marker is not None:
        edge_points = edge_marker.get("points", [])
        for offset in range(0, len(edge_points) - 1, 2):
            left = np.asarray(edge_points[offset], dtype=np.float32)
            right = np.asarray(edge_points[offset + 1], dtype=np.float32)
            a = int(np.argmin(np.linalg.norm(nodes - left[None], axis=1)))
            b = int(np.argmin(np.linalg.norm(nodes - right[None], axis=1)))
            if a != b:
                edges.add((a, b))
                edges.add((b, a))

    base_count = len(nodes)
    candidate_indices = []
    pitch_rows = (20.0, 0.0, -20.0)
    graph_nodes = nodes.tolist()
    for row, pitch_deg in enumerate(pitch_rows):
        pitch = math.radians(pitch_deg)
        for col in range(5):
            angle = yaw + (col - 2) * math.radians(20.0)
            candidate = position + 10.0 * np.asarray(
                [
                    math.cos(pitch) * math.cos(angle),
                    math.cos(pitch) * math.sin(angle),
                    math.sin(pitch),
                ],
                dtype=np.float32,
            )
            candidate_indices.append(len(graph_nodes))
            graph_nodes.append(candidate.tolist())

    graph_nodes = np.asarray(graph_nodes, dtype=np.float32)
    if base_count:
        base = graph_nodes[:base_count]
        for j in candidate_indices:
            nearest_base = int(np.argmin(np.linalg.norm(base - graph_nodes[j][None], axis=1)))
            edges.add((nearest_base, j))
            edges.add((j, nearest_base))

    degree = np.zeros(len(graph_nodes), dtype=np.float32)
    for a, _ in edges:
        degree[a] += 1.0

    x = np.zeros((len(graph_nodes), 24), dtype=np.float32)
    x[:, :3] = graph_nodes / np.asarray([50.0, 150.0, 12.0], dtype=np.float32)
    x[:, 3] = np.minimum(degree, 8.0) / 8.0
    for row in range(3):
        for col in range(5):
            idx = candidate_indices[row * 5 + col]
            semantic = semantic_ranking["ranking"].get(col)
            if semantic is None:
                score = 0.5
                confidence = 1.0
            else:
                score = float(semantic["score"])
                confidence = float(semantic["confidence"])
            x[idx, 4] = score
            x[idx, 5] = confidence
            x[idx, 6] = 1.0
            x[idx, 7] = (row * 5 + col) / 14.0
    x[:, 8] = np.linalg.norm(graph_nodes, axis=1) / 160.0
    x[:, 9] = np.linalg.norm(graph_nodes - goal[None], axis=1) / 160.0
    odom_index = int(np.argmin(np.linalg.norm(graph_nodes[:base_count] - position[None], axis=1)))
    x[odom_index, 10] = 1.0
    x[candidate_indices[5:10], 11] = 1.0
    x[:, 12] = math.sin(yaw)
    x[:, 13] = math.cos(yaw)
    delta = graph_nodes - position[None]
    c, s = math.cos(yaw), math.sin(yaw)
    body_x = c * delta[:, 0] + s * delta[:, 1]
    body_y = -s * delta[:, 0] + c * delta[:, 1]
    x[:, 14] = body_x / 80.0
    x[:, 15] = body_y / 80.0
    x[:, 16] = delta[:, 2] / 12.0
    x[:, 17] = np.linalg.norm(delta, axis=1) / 80.0
    x[:, 18] = np.arctan2(body_y, body_x) / math.pi
    x[:, 19] = np.arctan2(delta[:, 2], np.maximum(np.linalg.norm(delta[:, :2], axis=1), 1e-6)) / math.pi
    goal_delta = goal - position
    x[:, 20] = (c * goal_delta[0] + s * goal_delta[1]) / 140.0
    x[:, 21] = (-s * goal_delta[0] + c * goal_delta[1]) / 140.0
    x[:, 22] = goal_delta[2] / 12.0
    x[:, 23] = float(np.clip(semantic_ranking["progress_t"], 0.0, 1.0))

    directed = sorted(edges)
    edge_index = torch.tensor(directed, dtype=torch.long).t().contiguous() if directed else torch.empty((2, 0), dtype=torch.long)
    edge_weight = torch.tensor(
        [1.0 / max(float(np.linalg.norm(graph_nodes[a] - graph_nodes[b])), 1e-3) for a, b in directed],
        dtype=torch.float32,
    )
    return {
        "x": torch.from_numpy(x),
        "edge_index": edge_index,
        "edge_weight": edge_weight,
        "frontier_index": torch.tensor(candidate_indices),
        "frontier_columns": torch.arange(15),
        "safe_columns": torch.ones(15, dtype=torch.bool),
        "target": target,
        "planner_target": int(semantic_ranking["planner_target"]),
        "session": semantic_ranking["session"],
        "seq": int(semantic_ranking["seq"]),
        "position": position.tolist(),
        "prompt": semantic_ranking["prompt"],
    }


def make_privileged_sample(position, yaw, goal, route, route_index, free_points, bounds, resolution, rng, semantic_ranking):
    target_cell = route_target(route, route_index, resolution, 35.0)
    if target_cell is None:
        return None
    target_world = cell_to_world(target_cell, bounds, resolution)
    target = class_for_direction(target_world - position, yaw)

    dist = np.linalg.norm(free_points - position[None], axis=1)
    local = free_points[dist < 30.0]
    if len(local) > 180:
        local = local[rng.choice(len(local), 180, replace=False)]

    graph_nodes = local.tolist()
    candidate_indices = []
    for row, pitch_deg in enumerate((20.0, 0.0, -20.0)):
        pitch = math.radians(pitch_deg)
        for col in range(5):
            angle = yaw + (col - 2) * math.radians(20.0)
            candidate = position + 10.0 * np.asarray(
                [
                    math.cos(pitch) * math.cos(angle),
                    math.cos(pitch) * math.sin(angle),
                    math.sin(pitch),
                ],
                dtype=np.float32,
            )
            candidate_indices.append(len(graph_nodes))
            graph_nodes.append(candidate.tolist())

    graph_nodes = np.asarray(graph_nodes, dtype=np.float32)
    local_count = len(local)
    odom_index = int(np.argmin(np.linalg.norm(graph_nodes[:local_count] - position[None], axis=1))) if local_count else 0

    edges = set()
    for i in range(local_count):
        nearest_local = np.argsort(np.linalg.norm(graph_nodes[:local_count] - graph_nodes[i], axis=1))[1:7]
        edges.update((i, int(j)) for j in nearest_local)
    for j in candidate_indices:
        if local_count:
            nearest_base = int(np.argmin(np.linalg.norm(graph_nodes[:local_count] - graph_nodes[j], axis=1)))
            edges.add((nearest_base, j))
            edges.add((j, nearest_base))

    degree = np.zeros(len(graph_nodes), dtype=np.float32)
    for a, _ in edges:
        degree[a] += 1.0

    x = np.zeros((len(graph_nodes), 24), dtype=np.float32)
    x[:, :3] = graph_nodes / np.asarray([50.0, 150.0, 12.0], dtype=np.float32)
    x[:, 3] = np.minimum(degree, 8.0) / 8.0
    for row in range(3):
        for col in range(5):
            idx = candidate_indices[row * 5 + col]
            semantic = semantic_ranking.get(col)
            if semantic is None:
                score = 0.5
                confidence = 1.0
            else:
                score = float(semantic["score"])
                confidence = float(semantic["confidence"])
            x[idx, 4] = score
            x[idx, 5] = confidence
            x[idx, 6] = 1.0
            x[idx, 7] = (row * 5 + col) / 14.0
    x[:, 8] = np.linalg.norm(graph_nodes, axis=1) / 160.0
    x[:, 9] = np.linalg.norm(graph_nodes - goal[None], axis=1) / 160.0
    x[odom_index, 10] = 1.0
    x[candidate_indices[5:10], 11] = 1.0
    x[:, 12] = math.sin(yaw)
    x[:, 13] = math.cos(yaw)
    delta = graph_nodes - position[None]
    c, s = math.cos(yaw), math.sin(yaw)
    body_x = c * delta[:, 0] + s * delta[:, 1]
    body_y = -s * delta[:, 0] + c * delta[:, 1]
    x[:, 14] = body_x / 80.0
    x[:, 15] = body_y / 80.0
    x[:, 16] = delta[:, 2] / 12.0
    x[:, 17] = np.linalg.norm(delta, axis=1) / 80.0
    x[:, 18] = np.arctan2(body_y, body_x) / math.pi
    x[:, 19] = np.arctan2(delta[:, 2], np.maximum(np.linalg.norm(delta[:, :2], axis=1), 1e-6)) / math.pi
    goal_delta = goal - position
    x[:, 20] = (c * goal_delta[0] + s * goal_delta[1]) / 140.0
    x[:, 21] = (-s * goal_delta[0] + c * goal_delta[1]) / 140.0
    x[:, 22] = goal_delta[2] / 12.0
    x[:, 23] = float(route_index) / max(1, len(route))

    directed = sorted(edges)
    edge_index = torch.tensor(directed, dtype=torch.long).t().contiguous() if directed else torch.empty((2, 0), dtype=torch.long)
    edge_weight = torch.tensor(
        [1.0 / max(float(np.linalg.norm(graph_nodes[a] - graph_nodes[b])), 1e-3) for a, b in directed],
        dtype=torch.float32,
    )
    return {
        "x": torch.from_numpy(x),
        "edge_index": edge_index,
        "edge_weight": edge_weight,
        "frontier_index": torch.tensor(candidate_indices),
        "frontier_columns": torch.arange(15),
        "safe_columns": torch.ones(15, dtype=torch.bool),
        "target": target,
        "planner_target": target,
        "session": semantic_ranking.get("session", "map4_3d_privileged"),
        "seq": int(semantic_ranking.get("seq", route_index)),
        "position": position.tolist(),
        "prompt": semantic_ranking.get("prompt", ""),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs-root", default="log_scalenav")
    parser.add_argument("--output", required=True)
    parser.add_argument("--occupancy", required=True)
    parser.add_argument("--resolution", type=float, default=0.5)
    parser.add_argument("--inflate", type=float, default=1.0)
    parser.add_argument("--lookahead", type=float, default=35.0)
    parser.add_argument("--start-z", type=float, default=3.5)
    parser.add_argument("--goal-z", type=float, default=3.5)
    parser.add_argument("--prompt", default="line")
    parser.add_argument("--semantic-prompt", default="")
    parser.add_argument("--pearl-root", default=str(ROOT / "scalenav_ws" / "src" / "global_graph" / "heatmap_ws" / "pearl_ws"))
    parser.add_argument("--checkpoint", default="ViT-B/16")
    parser.add_argument("--synthetic-samples", type=int, default=2200)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    torch.manual_seed(args.seed)

    volume = np.load(args.occupancy)
    occupied = volume["occupied"].astype(bool)
    bounds = volume["bounds"].tolist()
    resolution = float(volume["resolution"])
    if abs(resolution - args.resolution) > 1e-5:
        raise ValueError(f"occupancy resolution is {resolution}, requested {args.resolution}")
    blocked = inflate_occupancy(occupied, max(1, math.ceil(args.inflate / resolution)))

    semantic_prompt = args.semantic_prompt.strip() or args.prompt.strip()
    encoder = None
    if semantic_prompt:
        encoder = PEARLHeatmapEncoder(
            args.pearl_root,
            checkpoint=args.checkpoint,
            device=torch.device("cuda" if torch.cuda.is_available() else "cpu"),
        )
        encoder.prepare_prompt(semantic_prompt)

    start_world = np.asarray([0.0, 0.0, args.start_z], dtype=np.float32)
    goal_world = np.asarray([0.0, 140.0, args.goal_z], dtype=np.float32)
    start = world_to_cell(start_world, bounds, resolution)
    goal = world_to_cell(goal_world, bounds, resolution)
    for center in (start, goal):
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                for dz in range(-2, 3):
                    c = (center[0] + dx, center[1] + dy, center[2] + dz)
                    if all(0 <= c[i] < blocked.shape[i] for i in range(3)):
                        blocked[c] = False

    route = astar(start, goal, blocked)
    if not route:
        raise SystemExit("3-D A* found no route; reduce --inflate or inspect mesh truth")
    route_world = np.asarray([cell_to_world(c, bounds, resolution) for c in route])
    print(
        f"route_cells={len(route)} route_length_m={sum(np.linalg.norm(np.diff(route_world, axis=0), axis=1)):.2f} "
        f"z_range={route_world[:,2].min():.2f}..{route_world[:,2].max():.2f}"
    )

    samples = []
    semantic_profiles = []
    skipped = 0
    sessions = sorted(
        str(path) for path in Path(args.logs_root).glob("session_*")
        if (path / "index.jsonl").is_file()
    )
    for session in sessions:
        entries = parse_entries(session)
        odom_records = [e for e in entries if e.get("kind") == "odom"]
        timing_records = [
            e for e in entries
            if e.get("kind") == "timing"
            and e.get("data", {}).get("module") == "planner"
            and e.get("data", {}).get("selected_semantic_column", -1) >= 0
        ]
        graph_records = [e for e in entries if e.get("kind") == "graph" and e.get("file")]
        rgb_records = [e for e in entries if e.get("kind") == "rgb" and e.get("file")]
        if not odom_records or not timing_records or not graph_records:
            continue
        first_z = float(odom_records[0].get("data", {}).get("position", [0.0, 0.0, 0.0])[2])
        if abs(first_z - args.start_z) > 0.75:
            continue
        goal_record = next((e for e in entries if e.get("kind") == "goal"), None)
        frame_goal = np.asarray(
            (goal_record or {}).get("data", {}).get("position", goal_world.tolist()),
            dtype=np.float32,
        )
        for timing in timing_records:
            stamp = int(timing["stamp_ns"])
            graph = nearest(graph_records, stamp)
            pose = nearest(odom_records, stamp)
            rgb_record = nearest(rgb_records, stamp) if rgb_records else None
            snapshot = graph_snapshot(
                {
                    "file": os.path.join(session, graph["file"]),
                }
            )
            semantic_ranking = {}
            if encoder is not None and rgb_record is not None:
                rgb_path = os.path.join(session, rgb_record["file"])
                rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
                if rgb is None:
                    skipped += 1
                    continue
                rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
                heatmap = encoder.encode_rgb(rgb, semantic_prompt)
                directions = torch.tensor(
                    [[[
                        [math.cos((column - 2) * math.radians(20.0)),
                         math.sin((column - 2) * math.radians(20.0)),
                         0.0]
                        for column in range(5)
                    ]]],
                    dtype=torch.float32,
                    device=encoder.device,
                )
                heatmap_tensor = torch.from_numpy(heatmap).unsqueeze(0).to(encoder.device)
                column_scores = sample_heatmap_at_body_directions(
                    heatmap_tensor,
                    directions,
                    horizontal_fov_deg=90.0,
                    vertical_fov_deg=60.0,
                    horizontal_only=False,
                )[0, 0].detach().cpu().numpy()
                semantic_ranking = {
                    column: {"score": float(column_scores[column]), "confidence": 1.0}
                    for column in range(5)
                }
            else:
                semantic_ranking = {
                    int(item.get("column", -1)): {
                        "score": 1.0 / (1.0 + max(float(item.get("risk", 1.0)), 0.0)),
                        "confidence": 1.0,
                    }
                    for item in timing.get("data", {}).get("semantic_frontier_ranking", [])
                    if 0 <= int(item.get("column", -1)) < 5
                }
            position = np.asarray(pose["data"]["position"], dtype=np.float32)
            if semantic_ranking:
                semantic_profiles.append((float(position[1]), dict(semantic_ranking)))
            orientation = pose["data"].get("orientation", [0.0, 0.0, 0.0, 1.0])
            yaw = yaw_from_quaternion(orientation)
            route_index = int(np.argmin(np.linalg.norm(route_world - position[None], axis=1)))
            target_cell = route_target(route, route_index, resolution, args.lookahead)
            if target_cell is None:
                skipped += 1
                continue
            semantic_bundle = {
                "snapshot": snapshot,
                "ranking": semantic_ranking,
                "progress_t": float(timing.get("data", {}).get("frontier_progress_t", 0.0)),
                "planner_target": int(min(semantic_ranking, key=lambda k: semantic_ranking[k]["score"]))
                if semantic_ranking else int(timing["data"].get("selected_semantic_column", -1)),
                "session": os.path.basename(session),
                "seq": int(timing.get("seq", 0)),
                "prompt": semantic_prompt,
            }
            sample = make_sample(position, yaw, frame_goal, route, route_index, blocked, bounds, resolution, semantic_bundle)
            if sample is None:
                skipped += 1
                continue
            sample["session"] = f"{semantic_prompt}_{sample['session']}"
            samples.append(sample)
        print(f"session={os.path.basename(session)} collected={len(samples)}")

    if args.synthetic_samples > 0:
        free_cells = np.argwhere(~blocked)
        free_points = np.asarray([cell_to_world(tuple(c), bounds, resolution) for c in free_cells], dtype=np.float32)
        semantic_profiles.sort(key=lambda item: item[0])

        def semantic_for_y(y_value):
            if not semantic_profiles:
                return {column: {"score": 0.5, "confidence": 1.0} for column in range(5)}
            idx = int(np.argmin([abs(y - y_value) for y, _ in semantic_profiles]))
            return dict(semantic_profiles[idx][1])

        positions = []
        stride = max(1, int(2.0 / resolution))
        for i in range(0, len(route), stride):
            positions.append((i, route_world[i]))
        while len(positions) < args.synthetic_samples:
            idx = int(rng.integers(0, len(route)))
            base = route_world[idx]
            jitter = rng.normal(0.0, 1.5, 3).astype(np.float32)
            candidate = base + jitter
            cell = world_to_cell(candidate, bounds, resolution)
            if all(0 <= cell[i] < blocked.shape[i] for i in range(3)) and not blocked[cell]:
                positions.append((idx, candidate))

        synthetic_added = 0
        for sample_index, (idx, position) in enumerate(positions[:args.synthetic_samples]):
            if idx < len(route) - 1:
                tangent = route_world[min(len(route) - 1, idx + max(1, int(4.0 / resolution)))] - route_world[idx]
            else:
                tangent = goal_world - position
            yaw = math.atan2(float(tangent[1]), float(tangent[0]))
            semantic_ranking = semantic_for_y(float(position[1]))
            semantic_ranking["session"] = f"{semantic_prompt}_map4_3d_privileged_{sample_index // 200:02d}"
            semantic_ranking["seq"] = sample_index % 200
            semantic_ranking["prompt"] = semantic_prompt
            sample = make_privileged_sample(
                position,
                yaw,
                goal_world,
                route,
                min(idx, len(route) - 1),
                free_points,
                bounds,
                resolution,
                rng,
                semantic_ranking,
            )
            if sample is not None:
                samples.append(sample)
                synthetic_added += 1
        print(f"synthetic_privileged_added={synthetic_added} total={len(samples)}")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    torch.save(
        {
            "schema": "scalenav_gcn_3d_semantic.v1",
            "num_classes": 15,
            "input_dim": 24,
            "prompt": args.prompt,
            "samples": samples,
            "truth": {
                "source": str(Path(args.occupancy).resolve()),
                "start": start_world.tolist(),
                "goal": goal_world.tolist(),
                "route": route_world.tolist(),
                "lookahead_m": args.lookahead,
                "synthetic_samples": int(args.synthetic_samples),
            },
        },
        args.output,
    )
    print(f"wrote={args.output} samples={len(samples)} skipped={skipped}")


if __name__ == "__main__":
    main()
