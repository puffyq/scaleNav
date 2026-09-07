#!/usr/bin/env python3
"""Create a Map4 3-D privileged GCN dataset from UE mesh occupancy truth.

Labels come from a 26-connected A* route in the complete 3-D mesh volume.  A
sample contains a local graph, fifteen (3 pitch x 5 yaw) candidates, and the
class of the route direction at least ``lookahead`` metres ahead.
"""
from __future__ import annotations

import argparse
import math
import random
from heapq import heappop, heappush
from pathlib import Path

import numpy as np
import torch


def neighbors26():
    return [(dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz))
            for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
            if (dx, dy, dz) != (0, 0, 0)]


N26 = neighbors26()


def inflate_occupancy(occupied, radius_cells):
    out = occupied.copy()
    offsets = [(dx, dy, dz) for dx in range(-radius_cells, radius_cells + 1)
               for dy in range(-radius_cells, radius_cells + 1)
               for dz in range(-radius_cells, radius_cells + 1)
               if dx * dx + dy * dy + dz * dz <= radius_cells * radius_cells]
    source = np.argwhere(occupied)
    shape = occupied.shape
    for dx, dy, dz in offsets:
        cells = source + np.array([dx, dy, dz])
        valid = ((cells[:, 0] >= 0) & (cells[:, 0] < shape[0]) &
                 (cells[:, 1] >= 0) & (cells[:, 1] < shape[1]) &
                 (cells[:, 2] >= 0) & (cells[:, 2] < shape[2]))
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
            if not (0 <= v[0] < shape[0] and 0 <= v[1] < shape[1] and 0 <= v[2] < shape[2]) or blocked[v]:
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
    return np.asarray([bounds[i][0] + (cell[i] + 0.5) * resolution for i in range(3)], dtype=np.float32)


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
    col = max(0, min(4, int(round(2.0 - azimuth / math.radians(20.0)))) )
    row = 0 if elevation > math.radians(10.0) else 2 if elevation < -math.radians(10.0) else 1
    return row * 5 + col


def make_sample(position, yaw, goal, route, route_index, free_points, blocked, bounds, resolution, rng):
    target_cell = route_target(route, route_index, resolution, 35.0)
    if target_cell is None:
        return None
    target_world = cell_to_world(target_cell, bounds, resolution)
    target = class_for_direction(target_world - position, yaw)
    # Local graph nodes are real free-volume samples plus the 15 virtual
    # candidates.  A small random subset keeps every graph bounded.
    dist = np.linalg.norm(free_points - position[None], axis=1)
    local = free_points[dist < 30.0]
    if len(local) > 180:
        local = local[rng.choice(len(local), 180, replace=False)]
    nodes = local.tolist()
    candidate_indices = []
    for row, pitch_deg in enumerate((20.0, 0.0, -20.0)):
        pitch = math.radians(pitch_deg)
        for col in range(5):
            angle = yaw + (col - 2) * math.radians(20.0)
            candidate = position + 10.0 * np.asarray([
                math.cos(pitch) * math.cos(angle), math.cos(pitch) * math.sin(angle), math.sin(pitch)], dtype=np.float32)
            candidate_indices.append(len(nodes)); nodes.append(candidate.tolist())
    nodes = np.asarray(nodes, dtype=np.float32)
    odom_index = int(np.argmin(np.linalg.norm(nodes[:len(local)] - position[None], axis=1))) if len(local) else 0
    edges = set()
    for i in range(len(local)):
        nearest = np.argsort(np.linalg.norm(nodes[:len(local)] - nodes[i], axis=1))[1:7]
        edges.update((i, int(j)) for j in nearest)
    for j in candidate_indices:
        if len(local):
            nearest = int(np.argmin(np.linalg.norm(nodes[:len(local)] - nodes[j], axis=1)))
            edges.update(((nearest, j), (j, nearest)))
    degree = np.zeros(len(nodes), dtype=np.float32)
    for a, _ in edges:
        degree[a] += 1
    x = np.zeros((len(nodes), 24), dtype=np.float32)
    x[:, :3] = nodes / np.asarray([50.0, 150.0, 12.0], dtype=np.float32)
    x[:, 3] = np.minimum(degree, 8) / 8.0
    for row in range(3):
        for col in range(5):
            idx = candidate_indices[row * 5 + col]
            x[idx, 4] = 0.5; x[idx, 5] = 1.0; x[idx, 6] = 1.0
            x[idx, 7] = (row * 5 + col) / 14.0
    x[:, 8] = np.linalg.norm(nodes, axis=1) / 160.0
    x[:, 9] = np.linalg.norm(nodes - goal[None], axis=1) / 160.0
    x[odom_index, 10] = 1.0
    x[candidate_indices[5:10], 11] = 1.0
    x[:, 12] = math.sin(yaw); x[:, 13] = math.cos(yaw)
    delta = nodes - position[None]; c, s = math.cos(yaw), math.sin(yaw)
    x[:, 14] = (c * delta[:, 0] + s * delta[:, 1]) / 80.0
    x[:, 15] = (-s * delta[:, 0] + c * delta[:, 1]) / 80.0
    x[:, 16] = delta[:, 2] / 12.0
    x[:, 17] = np.linalg.norm(delta, axis=1) / 80.0
    x[:, 18] = np.arctan2(x[:, 15], x[:, 14]) / math.pi
    x[:, 19] = np.arctan2(delta[:, 2], np.maximum(np.linalg.norm(delta[:, :2], axis=1), 1e-6)) / math.pi
    goal_delta = goal - position
    x[:, 20] = (c * goal_delta[0] + s * goal_delta[1]) / 140.0
    x[:, 21] = (-s * goal_delta[0] + c * goal_delta[1]) / 140.0
    x[:, 22] = goal_delta[2] / 12.0
    x[:, 23] = float(route_index) / max(1, len(route))
    directed = sorted(edges)
    edge_index = torch.tensor(directed, dtype=torch.long).t().contiguous() if directed else torch.empty((2, 0), dtype=torch.long)
    weights = torch.tensor([1.0 / max(float(np.linalg.norm(nodes[a] - nodes[b])), 1e-3) for a, b in directed], dtype=torch.float32)
    return {"x": torch.from_numpy(x), "edge_index": edge_index, "edge_weight": weights,
            "frontier_index": torch.tensor(candidate_indices), "frontier_columns": torch.arange(15),
            "safe_columns": torch.ones(15, dtype=torch.bool), "target": target,
            "planner_target": target, "session": "map4_3d_line", "seq": route_index,
            "position": position.tolist()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--occupancy", required=True); p.add_argument("--output", required=True)
    p.add_argument("--resolution", type=float, default=0.5); p.add_argument("--inflate", type=float, default=1.0)
    p.add_argument("--lookahead", type=float, default=35.0); p.add_argument("--samples", type=int, default=1800)
    p.add_argument("--seed", type=int, default=7)
    a = p.parse_args(); rng = np.random.default_rng(a.seed); random.seed(a.seed)
    volume = np.load(a.occupancy); occupied = volume["occupied"].astype(bool); bounds = volume["bounds"].tolist(); resolution = float(volume["resolution"])
    if abs(resolution - a.resolution) > 1e-5:
        raise ValueError(f"occupancy resolution is {resolution}, requested {a.resolution}")
    blocked = inflate_occupancy(occupied, max(1, math.ceil(a.inflate / resolution)))
    start_world = np.asarray([0.0, 0.0, 3.5], dtype=np.float32); goal_world = np.asarray([0.0, 140.0, 3.5], dtype=np.float32)
    start = world_to_cell(start_world, bounds, resolution); goal = world_to_cell(goal_world, bounds, resolution)
    for center in (start, goal):
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                for dz in range(-2, 3):
                    c = (center[0] + dx, center[1] + dy, center[2] + dz)
                    if all(0 <= c[i] < blocked.shape[i] for i in range(3)): blocked[c] = False
    route = astar(start, goal, blocked)
    if not route: raise SystemExit("3-D A* found no route; reduce --inflate or inspect mesh truth")
    route_world = np.asarray([cell_to_world(c, bounds, resolution) for c in route])
    print(f"route_cells={len(route)} route_length_m={sum(np.linalg.norm(np.diff(route_world,axis=0),axis=1)):.2f} z_range={route_world[:,2].min():.2f}..{route_world[:,2].max():.2f}")
    free_cells = np.argwhere(~blocked)
    free_points = np.asarray([cell_to_world(tuple(c), bounds, resolution) for c in free_cells], dtype=np.float32)
    positions = []
    stride = max(1, int(2.0 / resolution))
    for i in range(0, len(route), stride):
        positions.append((i, route_world[i]))
    while len(positions) < a.samples:
        idx = int(rng.integers(0, len(route))); base = route_world[idx]
        jitter = rng.normal(0.0, 1.5, 3).astype(np.float32); candidate = base + jitter
        cell = world_to_cell(candidate, bounds, resolution)
        if all(0 <= cell[i] < blocked.shape[i] for i in range(3)) and not blocked[cell]: positions.append((idx, candidate))
    samples = []
    for sample_index, (idx, position) in enumerate(positions[:a.samples]):
        if idx < len(route) - 1:
            tangent = route_world[min(len(route)-1, idx + max(1, int(4.0 / resolution)))] - route_world[idx]
        else: tangent = goal_world - position
        yaw = math.atan2(float(tangent[1]), float(tangent[0]))
        sample = make_sample(position, yaw, goal_world, route, min(idx, len(route)-1), free_points, blocked, bounds, resolution, rng)
        if sample is not None:
            sample["session"] = f"map4_3d_line_{sample_index // 200:02d}"
            sample["seq"] = sample_index % 200
            samples.append(sample)
    Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    torch.save({"schema": "scalenav_gcn_3d.v1", "num_classes": 15, "input_dim": 24,
                "samples": samples, "truth": {"source": str(Path(a.occupancy).resolve()), "start": start_world.tolist(), "goal": goal_world.tolist(), "route": route_world.tolist(), "lookahead_m": a.lookahead}}, a.output)
    print(f"wrote={a.output} samples={len(samples)} labels={[sum(s['target']==i for s in samples) for i in range(15)]}")


if __name__ == "__main__": main()
