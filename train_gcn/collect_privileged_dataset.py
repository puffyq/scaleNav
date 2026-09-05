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


def read_ascii_ply(path, stride=1):
    """Read XYZ vertices from an ASCII PLY obstacle map."""
    out = []
    if not path or not os.path.isfile(path):
        return out
    data = False
    with open(path, encoding="ascii", errors="ignore") as stream:
        for index, line in enumerate(stream):
            if line.startswith("end_header"):
                data = True
                continue
            if not data or index % max(1, stride) != 0:
                continue
            fields = line.split()
            if len(fields) >= 3:
                try:
                    out.append((float(fields[0]), float(fields[1]), float(fields[2])))
                except ValueError:
                    pass
    return out


def build_static_occupancy(path, resolution=0.5, inflate=1.2, stride=1):
    """Voxelize a privileged static world map without clearing trajectories."""
    occupied = set()
    for x, y, z in read_ascii_ply(path, stride):
        if 0.1 <= z <= 3.2:
            occupied.add((math.floor(x / resolution), math.floor(y / resolution)))
    radius = max(1, math.ceil(inflate / resolution))
    return {(x + dx, y + dy)
            for x, y in occupied
            for dx in range(-radius, radius + 1)
            for dy in range(-radius, radius + 1)
            if dx * dx + dy * dy <= radius * radius}


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
    return inflated


def clear_trajectories(blocked, positions, resolution, radius_m=1.5):
    radius = max(1, math.ceil(radius_m / resolution))
    for p in positions:
        cx, cy = math.floor(p[0] / resolution), math.floor(p[1] / resolution)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    blocked.discard((cx + dx, cy + dy))


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


def label_path(path, start_xy, yaw, resolution, lookahead=35.0):
    if len(path) < 2: return None
    # Use a route point at least lookahead meters away.  The immediate grid
    # neighbor is too local and makes the label react after it is already too
    # late to commit to a detour.
    distance = 0.0
    selected = None
    for current, following in zip(path, path[1:]):
        segment = math.hypot(following[0] - current[0], following[1] - current[1]) * resolution
        distance += segment
        if distance >= lookahead:
            selected = following
            break
    if selected is None:
        return None
    selected_world = ((selected[0] + 0.5) * resolution,
                      (selected[1] + 0.5) * resolution)
    dx = selected_world[0] - start_xy[0]
    dy = selected_world[1] - start_xy[1]
    angle = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                       math.cos(yaw) * dx + math.sin(yaw) * dy)
    limit = math.radians(50.0)
    if abs(angle) > limit: return None
    # ScaleNav columns are ordered left-to-right: +40, +20, 0, -20, -40 deg.
    return max(0, min(4, int(round(2.0 - angle / math.radians(20.0)))))


def main():
    p = argparse.ArgumentParser(); p.add_argument("--output", default="train_gcn/dataset_privileged.pt"); p.add_argument("--resolution", type=float, default=0.5); p.add_argument("--inflate", type=float, default=1.2); p.add_argument("--stride", type=int, default=20); p.add_argument("--lookahead", type=float, default=35.0); p.add_argument("--map-scope", choices=("global", "session"), default="global"); p.add_argument("--occupancy-cache", default="train_gcn/global_occupancy.pt"); p.add_argument("--map-ply", default="", help="privileged static ASCII PLY world map"); p.add_argument("--logs", nargs="*", default=None)
    a = p.parse_args(); samples = []; skipped = 0
    sessions = a.logs or sorted(glob.glob("log_scalenav/session_*"))
    global_blocked = None
    if a.map_scope == "global":
        if a.map_ply:
            global_blocked = build_static_occupancy(a.map_ply, a.resolution, a.inflate, a.stride)
            print(f"loaded_static_map={a.map_ply} global_occupied_cells={len(global_blocked)}")
            if a.occupancy_cache:
                os.makedirs(os.path.dirname(os.path.abspath(a.occupancy_cache)), exist_ok=True)
                torch.save({"blocked": sorted(global_blocked), "resolution": a.resolution,
                            "inflate": a.inflate, "stride": a.stride,
                            "source": os.path.abspath(a.map_ply)}, a.occupancy_cache)
        elif a.occupancy_cache and os.path.isfile(a.occupancy_cache):
            cached = torch.load(a.occupancy_cache, weights_only=False)
            global_blocked = set(map(tuple, cached["blocked"]))
            print(f"loaded_global_occupied_cells={len(global_blocked)}")
        else:
            global_blocked = set(); trajectories = []
            for session in sessions:
                if not os.path.isfile(os.path.join(session, "index.jsonl")): continue
                try: entries = parse_entries(session)
                except OSError: continue
                global_blocked.update(build_occupancy(session, entries, a.resolution, a.inflate, a.stride))
                trajectories.extend(e.get("data", {}).get("position", [0, 0, 0])
                                    for e in entries if e.get("kind") == "odom")
            # A trajectory is not evidence that nearby cells are obstacle-free:
            # another run may have observed a wall at the same world location.
            # Never erase occupied cells from the unified map based on odometry.
            print(f"global_occupied_cells={len(global_blocked)} trajectories={len(trajectories)} "
                  "trajectory_clear=disabled")
            if a.occupancy_cache:
                os.makedirs(os.path.dirname(os.path.abspath(a.occupancy_cache)), exist_ok=True)
                torch.save({"blocked": sorted(global_blocked), "resolution": a.resolution,
                            "inflate": a.inflate, "stride": a.stride}, a.occupancy_cache)
    for session in sessions:
        if not os.path.isfile(os.path.join(session, "index.jsonl")): continue
        try: entries = parse_entries(session); frames = load_log_frames(session)
        except (OSError, ValueError): skipped += 1; continue
        if not frames: continue
        goal = frames[0][4]
        blocked = global_blocked if global_blocked is not None else build_occupancy(session, entries, a.resolution, a.inflate, a.stride)
        positions = [e.get("data", {}).get("position", [0,0,0]) for e in entries if e.get("kind") == "odom"]
        all_xy = [(int(math.floor(goal[0]/a.resolution)), int(math.floor(goal[1]/a.resolution)))]
        all_xy += [(int(math.floor(x[0]/a.resolution)), int(math.floor(x[1]/a.resolution))) for x in positions]
        for timing, snapshot, position, orientation, frame_goal in frames:
            start = (int(math.floor(position[0]/a.resolution)), int(math.floor(position[1]/a.resolution)))
            goal_cell = (int(math.floor(frame_goal[0]/a.resolution)), int(math.floor(frame_goal[1]/a.resolution)))
            # The current vehicle cell is free by definition.  Clear only this
            # one cell for search; do not clear a radius or historical path.
            search_blocked = blocked - {start}
            margin = max(8, math.ceil(40.0 / a.resolution))
            bounds = (min(x[0] for x in all_xy)-margin, max(x[0] for x in all_xy)+margin, min(x[1] for x in all_xy)-margin, max(x[1] for x in all_xy)+margin)
            path = astar(start, goal_cell, search_blocked, bounds)
            yaw = math.atan2(2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]), 1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2))
            target = label_path(path, position[:2], yaw, a.resolution, a.lookahead)
            if target is None: skipped += 1; continue
            try: data, _, planner_target, _ = build_log_graph(torch, Data, timing, snapshot, position, orientation, frame_goal, "planner", allow_unreachable=True)
            except (KeyError, ValueError, IndexError): skipped += 1; continue
            data.safe_columns = torch.ones(5, dtype=torch.bool)
            # The label is body-relative, so the student must observe pose
            # orientation.  Repeat sin/cos(yaw) on every node to keep the
            # graph schema simple while making world-to-body conversion
            # identifiable from the input.
            pose_yaw = math.atan2(2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]),
                                  1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2))
            pose = torch.tensor([math.sin(pose_yaw), math.cos(pose_yaw)], dtype=data.x.dtype)
            # Add body-frame geometry and goal bearing.  These are invariant
            # to the arbitrary world map axes and directly expose what the
            # 35 m route-direction label depends on.
            world_x, world_y = data.x[:, 0] * 25.0, data.x[:, 1] * 80.0
            dx, dy = world_x - float(position[0]), world_y - float(position[1])
            body_x = math.cos(pose_yaw) * dx + math.sin(pose_yaw) * dy
            body_y = -math.sin(pose_yaw) * dx + math.cos(pose_yaw) * dy
            gdx, gdy = float(frame_goal[0]) - float(position[0]), float(frame_goal[1]) - float(position[1])
            goal_body = torch.tensor([math.cos(pose_yaw) * gdx + math.sin(pose_yaw) * gdy,
                                      -math.sin(pose_yaw) * gdx + math.cos(pose_yaw) * gdy], dtype=data.x.dtype)
            extra = torch.stack([body_x / 80.0, body_y / 80.0,
                                 torch.hypot(body_x, body_y) / 80.0,
                                 torch.atan2(body_y, body_x) / math.pi,
                                 goal_body[0].expand_as(body_x) / 140.0,
                                 goal_body[1].expand_as(body_x) / 140.0], dim=1)
            data.x = torch.cat([data.x, pose.expand(data.x.shape[0], -1), extra], dim=1)
            samples.append({"x": data.x.cpu(), "edge_index": data.edge_index.cpu(), "edge_weight": data.edge_weight.cpu(), "frontier_index": data.frontier_index.cpu(), "frontier_columns": data.frontier_columns.cpu(), "safe_columns": data.safe_columns.cpu(), "target": int(target), "planner_target": int(planner_target), "map_target": int(target), "session": os.path.basename(session), "seq": int(timing["seq"]), "position": list(position)})
        print(f"session={os.path.basename(session)} frames={len(frames)} collected={len(samples)}")
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True); torch.save({"schema":"scalenav_gcn_privileged.v1", "samples":samples}, a.output); print(f"wrote={a.output} samples={len(samples)} skipped={skipped}")


class Data:
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


if __name__ == "__main__": main()
