#!/usr/bin/env python3
"""Build a privileged GCN dataset from the complete logged point-cloud map."""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from heapq import heappop, heappush

import torch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scalenav_ws", "scripts"))
from demo_gnn_frontier_policy import build_log_graph, load_log_frames  # noqa: E402


def rotate(q, v):
    x, y, z, w = q; vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return [vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx]


def read_pcd(path, stride=20):
    out = []; data = False; index = 0
    try: stream = open(path, encoding="ascii", errors="ignore")
    except OSError: return out
    with stream:
        for line in stream:
            if line.lower().startswith("data"): data = True; continue
            if not data: continue
            if index % stride == 0:
                f = line.split()
                if len(f) >= 3:
                    try: out.append((float(f[0]), float(f[1]), float(f[2])))
                    except ValueError: pass
            index += 1
    return out


def parse_entries(session):
    entries = []
    with open(os.path.join(session, "index.jsonl"), encoding="utf-8") as stream:
        for line in stream:
            try: entries.append(json.loads(line, parse_constant=lambda x: float(x)))
            except json.JSONDecodeError: pass
    return entries


def build_occupancy(session, entries, resolution=0.5, inflate=1.2, stride=20):
    odom = [e for e in entries if e.get("kind") == "odom"]
    clouds = [e for e in entries if e.get("kind") == "pointcloud" and e.get("file")]
    occupied = set()
    for record in clouds:
        pose = min(odom, key=lambda e: abs(e["stamp_ns"] - record["stamp_ns"]), default=None)
        if pose is None: continue
        p = pose.get("data", {}).get("position", [0, 0, 0]); q = pose.get("data", {}).get("orientation", [0, 0, 0, 1])
        for point in read_pcd(os.path.join(session, record["file"]), stride):
            world = rotate(q, point)
            z = world[2] + p[2]
            # The vehicle flies at z~1.6; retain obstacles intersecting its body band.
            if z < 0.1 or z > 3.2: continue
            cell = (math.floor((world[0] + p[0]) / resolution), math.floor((world[1] + p[1]) / resolution))
            occupied.add(cell)
    radius = max(1, math.ceil(inflate / resolution))
    inflated = set()
    for x, y in occupied:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius: inflated.add((x + dx, y + dy))
    # Logged returns contain the vehicle body and ground returns.  Treat the
    # measured vehicle trajectory as free space so the start cell is not
    # occupied by the aircraft's own point cloud.
    clear_radius = max(1, math.ceil(1.5 / resolution))
    for pose in odom:
        p = pose.get("data", {}).get("position", [0, 0, 0])
        cx, cy = math.floor(p[0] / resolution), math.floor(p[1] / resolution)
        for dx in range(-clear_radius, clear_radius + 1):
            for dy in range(-clear_radius, clear_radius + 1):
                if dx * dx + dy * dy <= clear_radius * clear_radius: inflated.discard((cx + dx, cy + dy))
    return inflated


def astar(start, goal, blocked, bounds):
    if start in blocked or goal in blocked: return []
    minx, maxx, miny, maxy = bounds
    def h(a): return math.hypot(a[0] - goal[0], a[1] - goal[1])
    open_set = [(h(start), 0.0, start)]; cost = {start: 0.0}; parent = {}
    while open_set:
        _, g, u = heappop(open_set)
        if g != cost[u]: continue
        if u == goal:
            path = [u]
            while path[-1] != start: path.append(parent[path[-1]])
            return path[::-1]
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            v = (u[0] + dx, u[1] + dy)
            if not (minx <= v[0] <= maxx and miny <= v[1] <= maxy) or v in blocked: continue
            ng = g + math.hypot(dx, dy)
            if ng < cost.get(v, float("inf")):
                cost[v] = ng; parent[v] = u; heappush(open_set, (ng + h(v), ng, v))
    return []


def label_path(path, start_xy, yaw, resolution):
    if len(path) < 2: return None
    dx = (path[1][0] - path[0][0]); dy = (path[1][1] - path[0][1])
    angle = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                       math.cos(yaw) * dx + math.sin(yaw) * dy)
    limit = math.radians(50.0)
    if abs(angle) > limit: return None
    return max(0, min(4, int(round((angle / limit + 1.0) * 2.0))))


def main():
    p = argparse.ArgumentParser(); p.add_argument("--output", default="train_gcn/dataset_privileged.pt"); p.add_argument("--resolution", type=float, default=0.5); p.add_argument("--inflate", type=float, default=1.2); p.add_argument("--stride", type=int, default=20); p.add_argument("--logs", nargs="*", default=None)
    a = p.parse_args(); samples = []; skipped = 0
    for session in (a.logs or sorted(glob.glob("log_scalenav/session_*"))):
        if not os.path.isfile(os.path.join(session, "index.jsonl")): continue
        try: entries = parse_entries(session); frames = load_log_frames(session)
        except (OSError, ValueError): skipped += 1; continue
        if not frames: continue
        goal = frames[0][4]; blocked = build_occupancy(session, entries, a.resolution, a.inflate, a.stride)
        positions = [e.get("data", {}).get("position", [0,0,0]) for e in entries if e.get("kind") == "odom"]
        all_xy = [(int(math.floor(goal[0]/a.resolution)), int(math.floor(goal[1]/a.resolution)))]
        all_xy += [(int(math.floor(x[0]/a.resolution)), int(math.floor(x[1]/a.resolution))) for x in positions]
        for timing, snapshot, position, orientation, frame_goal in frames:
            start = (int(math.floor(position[0]/a.resolution)), int(math.floor(position[1]/a.resolution)))
            goal_cell = (int(math.floor(frame_goal[0]/a.resolution)), int(math.floor(frame_goal[1]/a.resolution)))
            margin = max(8, math.ceil(40.0 / a.resolution))
            bounds = (min(x[0] for x in all_xy)-margin, max(x[0] for x in all_xy)+margin, min(x[1] for x in all_xy)-margin, max(x[1] for x in all_xy)+margin)
            path = astar(start, goal_cell, blocked, bounds)
            yaw = math.atan2(2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]), 1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2))
            target = label_path(path, position[:2], yaw, a.resolution)
            if target is None: skipped += 1; continue
            try: data, _, planner_target, _ = build_log_graph(torch, Data, timing, snapshot, position, orientation, frame_goal, "planner", allow_unreachable=True)
            except (KeyError, ValueError, IndexError): skipped += 1; continue
            data.safe_columns = torch.ones(5, dtype=torch.bool)
            samples.append({"x": data.x.cpu(), "edge_index": data.edge_index.cpu(), "edge_weight": data.edge_weight.cpu(), "frontier_index": data.frontier_index.cpu(), "frontier_columns": data.frontier_columns.cpu(), "safe_columns": data.safe_columns.cpu(), "target": int(target), "planner_target": int(planner_target), "map_target": int(target), "session": os.path.basename(session), "seq": int(timing["seq"]), "position": list(position)})
        print(f"session={os.path.basename(session)} frames={len(frames)} collected={len(samples)}")
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True); torch.save({"schema":"scalenav_gcn_privileged.v1", "samples":samples}, a.output); print(f"wrote={a.output} samples={len(samples)} skipped={skipped}")


class Data:
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


if __name__ == "__main__": main()
