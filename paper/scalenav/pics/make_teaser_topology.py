#!/usr/bin/env python3
"""Render a topology-first ScaleNav teaser from a recorded flight log.

The figure uses one atomic graph snapshot and the nearest sensor/odometry
records from the same session.  The persistent skeleton and its A* / polynomial
witness are the primary visual; UE mesh truth is rendered only as a faint
geometric reference.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from PIL import Image


BG = "#FFFFFF"
TRUTH = "#B8C4C8"
EDGE = "#9EADB2"
NODE = "#506D78"
ROUTE = "#007C83"
WITNESS = "#E28A17"
VEHICLE = "#24343D"
FRONTIER = "#C46E13"
LOCAL = "#A64E88"
GOAL = "#B83E3E"


def nearest(items: list[dict], stamp: int) -> dict:
    return min(items, key=lambda item: abs(int(item["stamp_ns"]) - stamp))


def load_events(session: Path) -> dict[str, list[dict]]:
    events: dict[str, list[dict]] = {}
    with (session / "index.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            line = (line.replace(":-inf", ":-Infinity")
                    .replace(":inf", ":Infinity")
                    .replace(":-nan", ":-NaN")
                    .replace(":nan", ":NaN"))
            event = json.loads(line, parse_constant=lambda value: float(value))
            events.setdefault(event["kind"], []).append(event)
    required = ("odom", "rgb", "depth", "graph", "path", "goal")
    missing = [kind for kind in required if not events.get(kind)]
    if missing:
        raise ValueError(f"session is missing required streams: {', '.join(missing)}")
    return events


def marker_dict(graph: dict) -> dict[str, dict]:
    return {
        marker["ns"]: marker
        for marker in graph.get("markers", [])
        if marker.get("action", 0) == 0
    }


def marker_poses(graph: dict, name: str) -> np.ndarray:
    values = []
    for marker in graph.get("markers", []):
        if marker.get("ns") != name or marker.get("action", 0) != 0:
            continue
        position = marker.get("pose", {}).get("position")
        if position is not None:
            values.append(position)
    return np.asarray(values, dtype=float).reshape(-1, 3) if values else np.empty((0, 3))


def points(markers: dict[str, dict], name: str) -> np.ndarray:
    values = markers.get(name, {}).get("points", [])
    return np.asarray(values, dtype=float).reshape(-1, 3) if values else np.empty((0, 3))


def pose(markers: dict[str, dict], name: str) -> np.ndarray | None:
    value = markers.get(name, {}).get("pose", {}).get("position")
    return np.asarray(value, dtype=float) if value is not None else None


def load_ply(path: Path, max_points: int = 90000) -> np.ndarray:
    with path.open(encoding="ascii") as stream:
        count = None
        for line in stream:
            if line.startswith("element vertex"):
                count = int(line.split()[-1])
            if line.strip() == "end_header":
                break
        if count is None:
            raise ValueError(f"missing vertex count in {path}")
        cloud = np.loadtxt(stream, max_rows=count, usecols=(0, 1, 2))
    cloud = np.asarray(cloud, dtype=float).reshape(-1, 3)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    cloud = cloud[
        (cloud[:, 0] >= -30.0) & (cloud[:, 0] <= 30.0)
        & (cloud[:, 1] >= 0.0) & (cloud[:, 1] <= 140.0)
    ]
    if len(cloud) > max_points:
        indices = np.linspace(0, len(cloud) - 1, max_points, dtype=np.int64)
        cloud = cloud[indices]
    return cloud


def load_truth_footprint(path: Path, voxel_size: float = 0.35) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """Build the world-XY truth footprint used by the Map2 comparisons."""
    with path.open(encoding="ascii") as stream:
        count = None
        for line in stream:
            if line.startswith("element vertex"):
                count = int(line.split()[-1])
            if line.strip() == "end_header":
                break
        if count is None:
            raise ValueError(f"missing vertex count in {path}")
        cloud = np.loadtxt(stream, max_rows=count, usecols=(0, 1, 2))
    cloud = np.asarray(cloud, dtype=float).reshape(-1, 3)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    cloud = cloud[(cloud[:, 2] >= 0.0) & (cloud[:, 2] <= 40.0)
                  & (cloud[:, 0] >= -45.0) & (cloud[:, 0] <= 45.0)
                  & (cloud[:, 1] >= 0.0) & (cloud[:, 1] <= 140.0)]
    x_min, x_max, y_min, y_max = -45.0, 45.0, 0.0, 140.0
    nx = int(np.ceil((x_max - x_min) / voxel_size))
    ny = int(np.ceil((y_max - y_min) / voxel_size))
    ix = np.floor((cloud[:, 0] - x_min) / voxel_size).astype(np.int64)
    iy = np.floor((cloud[:, 1] - y_min) / voxel_size).astype(np.int64)
    valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    footprint = np.zeros((nx, ny), dtype=np.uint8)
    footprint[ix[valid], iy[valid]] = 1
    return footprint, (x_min, x_max, y_min, y_max)


def read_rgb(path: Path) -> np.ndarray:
    # PPM files are written after the logger converts the ROS bgr8 payload.
    return np.asarray(Image.open(path).convert("RGB"), dtype=float) / 255.0


def read_depth(path: Path) -> np.ndarray:
    return np.asarray(Image.open(path), dtype=float) / 1000.0


def load_snapshot(session: Path, events: dict[str, list[dict]], graph_file: str) -> tuple[dict, dict]:
    event = next((item for item in events["graph"] if item.get("file") == graph_file), None)
    if event is None:
        raise ValueError(f"graph event not found: {graph_file}")
    with (session / graph_file).open(encoding="utf-8") as stream:
        return event, json.load(stream)


def quat_rotate(quaternion: list[float], vectors: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    q = q / max(np.linalg.norm(q), 1.0e-9)
    v, w = q[:3], q[3]
    cross = 2.0 * np.cross(v, vectors)
    return vectors + w * cross + np.cross(v, cross)


def read_pcd(path: Path) -> np.ndarray:
    with path.open(encoding="ascii") as stream:
        data = False
        rows = []
        for line in stream:
            if data:
                values = line.split()
                if len(values) >= 3:
                    rows.append(values[:3])
            elif line.strip().lower().startswith("data"):
                data = True
    return np.asarray(rows, dtype=float).reshape(-1, 3)


def aggregate_logged_cloud(session: Path, events: dict[str, list[dict]], end_stamp: int) -> np.ndarray:
    odometry = events["odom"]
    clouds = []
    for event in events.get("pointcloud", []):
        if event.get("stamp_ns", 0) > end_stamp or not event.get("file"):
            continue
        local = read_pcd(session / event["file"])
        if not len(local):
            continue
        odom = nearest(odometry, event["stamp_ns"])
        world = quat_rotate(odom["data"]["orientation"], local)
        world += np.asarray(odom["data"]["position"], dtype=float)
        clouds.append(world[::4])
    if not clouds:
        return np.empty((0, 3))
    cloud = np.concatenate(clouds)
    # Keep the map readable and bounded while retaining the actual logged map.
    cloud = cloud[(cloud[:, 0] >= -35.0) & (cloud[:, 0] <= 35.0)
                  & (cloud[:, 1] >= -2.0) & (cloud[:, 1] <= 142.0)
                  & (cloud[:, 2] >= -2.0) & (cloud[:, 2] <= 12.0)]
    if len(cloud) > 120000:
        cloud = cloud[np.linspace(0, len(cloud) - 1, 120000, dtype=np.int64)]
    return cloud


def draw_sensor(ax_rgb, ax_semantic, ax_depth, rgb: np.ndarray,
                semantic: np.ndarray, depth: np.ndarray) -> None:
    ax_rgb.imshow(rgb, interpolation="nearest")
    ax_rgb.set_title(r"(a) Logged sensor state $t^*$", fontsize=10, pad=4)
    ax_rgb.text(0.03, 0.06, "RGB", transform=ax_rgb.transAxes, color="white",
                fontsize=8, fontweight="bold",
                bbox=dict(boxstyle="square,pad=0.18", facecolor=VEHICLE,
                          edgecolor="none", alpha=0.88))
    ax_semantic.imshow(semantic, cmap="magma", vmin=0.0, vmax=1.0,
                       interpolation="nearest")
    ax_semantic.set_title(r"PEARL response $S_t$", fontsize=9, pad=3)
    ax_semantic.text(0.03, 0.06, "open-vocabulary score", transform=ax_semantic.transAxes,
                     color="white", fontsize=7, fontweight="bold",
                     bbox=dict(boxstyle="square,pad=0.18", facecolor=VEHICLE,
                               edgecolor="none", alpha=0.88))
    clipped = depth >= 20.0
    shown = np.ma.masked_where(clipped, depth)
    cmap = matplotlib.colormaps["viridis"].copy()
    cmap.set_bad("white")
    ax_depth.imshow(shown, cmap=cmap, vmin=0.0, vmax=20.0, interpolation="nearest")
    ax_depth.set_title(r"Depth $D_t$ (m)", fontsize=9, pad=3)
    ax_depth.text(0.03, 0.06, "white: no return (>20 m)", transform=ax_depth.transAxes,
                  color=VEHICLE, fontsize=7, fontweight="bold",
                  bbox=dict(boxstyle="square,pad=0.18", facecolor="white",
                            edgecolor="none", alpha=0.84))
    for axis in (ax_rgb, ax_semantic, ax_depth):
        axis.set_xticks([])
        axis.set_yticks([])
        for spine in axis.spines.values():
            spine.set_color(EDGE)
            spine.set_linewidth(0.6)


def draw_topology(axis, footprint: np.ndarray, bounds: tuple[float, float, float, float],
                  markers: dict[str, dict], graph: dict, trajectory: np.ndarray) -> None:
    nodes = points(markers, "scalenav_skeleton_nodes")
    edge_points = points(markers, "scalenav_skeleton_edges")
    astar = points(markers, "scalenav_astar_topology_path")
    witness = points(markers, "scalenav_polynomial_witness_path")
    vehicle = pose(markers, "scalenav_vehicle_pose")
    local = pose(markers, "scalenav_local_goal")
    frontier = pose(markers, "scalenav_frontier_goal")
    goal = pose(markers, "scalenav_global_goal")

    # Complete AirSim truth is a subdued context layer; route marks below are
    # plotted in the same world-ENU projection as the logged graph.
    x_min, x_max, y_min, y_max = bounds
    # ``footprint`` is indexed [world-x, world-y].  With the screen axes
    # (horizontal=world-y, vertical=world-x), imshow must receive it without a
    # transpose, exactly as in plot_speed_trajectories.py.
    occupied = np.ma.masked_where(footprint == 0, footprint)
    axis.imshow(occupied, origin="lower", extent=(y_min, y_max, x_min, x_max),
                cmap=matplotlib.colors.ListedColormap([TRUTH]),
                vmin=0.5, vmax=1.5, alpha=0.34, interpolation="nearest",
                aspect="auto", zorder=1)
    if len(edge_points) >= 2:
        segments = edge_points[: len(edge_points) // 2 * 2].reshape(-1, 2, 3)
        axis.add_collection(LineCollection(
            [segment[:, [1, 0]] for segment in segments],
            colors=EDGE, linewidths=0.38, alpha=0.58, zorder=2,
        ))
    if len(nodes):
        axis.scatter(nodes[:, 1], nodes[:, 0], s=6.0, color=NODE, alpha=0.85,
                     edgecolors="white", linewidths=0.22, rasterized=True, zorder=3)
    semantic_labels = marker_poses(graph, "scalenav_semantic_point_labels")
    if len(semantic_labels):
        axis.scatter(semantic_labels[:, 1], semantic_labels[:, 0], marker="X",
                     s=24, color="#D14E46", alpha=0.88, edgecolors="white",
                     linewidths=0.5, zorder=9)
    if len(trajectory):
        axis.plot(trajectory[:, 1], trajectory[:, 0], color="white", lw=3.0,
                  solid_capstyle="round", zorder=4)
        axis.plot(trajectory[:, 1], trajectory[:, 0], color=VEHICLE, lw=1.45,
                  solid_capstyle="round", zorder=5)
    if len(astar) > 1:
        axis.plot(astar[:, 1], astar[:, 0], color="white", lw=4.3, zorder=6)
        axis.plot(astar[:, 1], astar[:, 0], color=ROUTE, lw=2.8, zorder=7)
    if len(witness) > 1:
        axis.plot(witness[:, 1], witness[:, 0], color="white", lw=3.8, zorder=7.5)
        axis.plot(witness[:, 1], witness[:, 0], color=WITNESS, lw=2.1,
                  linestyle=(0, (5, 2)), zorder=8)

    if vehicle is not None:
        axis.scatter(vehicle[1], vehicle[0], marker="^", s=70, color=VEHICLE,
                     edgecolors="white", linewidths=0.8, zorder=10)
        axis.annotate(r"vehicle at $t^*$", (vehicle[1], vehicle[0]),
                      xytext=(7, 13), textcoords="offset points", fontsize=7.2,
                      color=VEHICLE, fontweight="bold")
    if frontier is not None:
        axis.scatter(frontier[1], frontier[0], marker="D", s=52, color=FRONTIER,
                     edgecolors="white", linewidths=0.7, zorder=10)
        axis.annotate("frontier_goal", (frontier[1], frontier[0]), xytext=(16, 14),
                      textcoords="offset points", fontsize=7.0, color=FRONTIER,
                      fontweight="bold")
    if local is not None:
        axis.scatter(local[1], local[0], marker="o", s=49, color=LOCAL,
                     edgecolors="white", linewidths=0.7, zorder=10)
        axis.annotate("local_goal", (local[1], local[0]), xytext=(14, -17),
                      textcoords="offset points", fontsize=7.0, color=LOCAL,
                      fontweight="bold")
    if goal is not None:
        axis.scatter(goal[1], goal[0], marker="*", s=140, color=GOAL,
                     edgecolors="white", linewidths=0.7, zorder=10)
        axis.annotate("mission_goal", (goal[1], goal[0]), xytext=(-7, 7),
                      textcoords="offset points", ha="right", fontsize=7.0,
                      color=GOAL, fontweight="bold")

    axis.set_xlim(-2.0, 142.0)
    axis.set_ylim(-42.0, 42.0)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xticks([])
    axis.set_yticks([])
    axis.set_title("(b) Persistent topology and route state", fontsize=10, pad=4)
    axis.text(0.015, 0.03, "ScaleNav skeleton graph: nodes + collision-checked edges",
              transform=axis.transAxes, fontsize=7.2, color=NODE,
              bbox=dict(boxstyle="square,pad=0.22", facecolor="white",
                        edgecolor="none", alpha=0.86))
    handles = [
        Line2D([], [], color=NODE, marker="o", markersize=3.8, lw=0,
               label="persistent skeleton nodes"),
        Line2D([], [], color=EDGE, lw=1.1, label="verified graph edges"),
        Line2D([], [], color=ROUTE, lw=2.5, label="A* topology path"),
        Line2D([], [], color=WITNESS, lw=2.0, ls=(0, (5, 2)),
               label="polynomial witness"),
        Line2D([], [], color=VEHICLE, lw=1.5, label="flown trajectory"),
        Line2D([], [], color="#D14E46", marker="X", markersize=4.5, lw=0,
               label="far-field semantic nodes"),
    ]
    axis.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.055),
                ncol=3, fontsize=6.5, frameon=False, handlelength=2.0,
                columnspacing=0.9)


def main() -> None:
    global SESSION
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("out_prefix", type=Path)
    parser.add_argument("--graph", default="graph/graph_203.json",
                        help="logged graph snapshot to render")
    args = parser.parse_args()
    SESSION = args.session_dir.resolve()
    events = load_events(SESSION)
    graph_event, graph = load_snapshot(SESSION, events, args.graph)
    markers = marker_dict(graph)
    stamp = int(graph_event["stamp_ns"])
    semantic_event = nearest(events["semantic"], stamp)
    rgb_event = nearest(events["rgb"], semantic_event["stamp_ns"])
    depth_event = nearest(events["depth"], semantic_event["stamp_ns"])
    rgb = read_rgb(SESSION / rgb_event["file"])
    semantic = np.clip(read_depth(SESSION / semantic_event["file"]), 0.0, 1.0)
    depth = read_depth(SESSION / depth_event["file"])
    odometry = [event for event in events["odom"] if event["stamp_ns"] <= stamp]
    trajectory = np.asarray([event["data"]["position"] for event in odometry], dtype=float)
    truth_path = Path(__file__).with_name("map2_ground_truth_airsim_20260904.ply")
    if not truth_path.is_file():
        raise FileNotFoundError(f"complete Map2 truth map not found: {truth_path}")
    footprint, bounds = load_truth_footprint(truth_path)

    plt.rcParams.update({
        "font.family": "serif", "font.size": 8, "pdf.fonttype": 42,
        "ps.fonttype": 42, "figure.facecolor": BG, "savefig.facecolor": BG,
    })
    figure = plt.figure(figsize=(9.4, 4.25), facecolor=BG)
    grid = figure.add_gridspec(1, 2, width_ratios=[0.34, 1.30], wspace=0.025,
                               left=0.012, right=0.995, top=0.90, bottom=0.16)
    sensors = grid[0].subgridspec(3, 1, hspace=0.18)
    ax_rgb = figure.add_subplot(sensors[0])
    ax_semantic = figure.add_subplot(sensors[1])
    ax_depth = figure.add_subplot(sensors[2])
    draw_sensor(ax_rgb, ax_semantic, ax_depth, rgb, semantic, depth)
    ax_topology = figure.add_subplot(grid[1])
    draw_topology(ax_topology, footprint, bounds, markers, graph, trajectory)
    args.out_prefix.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.out_prefix.with_suffix(".pdf"), dpi=300)
    figure.savefig(args.out_prefix.with_suffix(".png"), dpi=300)
    plt.close(figure)
    print(f"wrote {args.out_prefix.with_suffix('.pdf')} and .png from {SESSION.name}; "
          f"graph={graph_event['file']} graph_stamp={stamp} "
          f"nodes={len(points(markers, 'scalenav_skeleton_nodes'))} "
          f"edges={len(points(markers, 'scalenav_skeleton_edges')) // 2} "
          f"astar={len(points(markers, 'scalenav_astar_topology_path'))} "
          f"witness={len(points(markers, 'scalenav_polynomial_witness_path'))} "
          f"sensor_offsets_ms={{'rgb': {(rgb_event['stamp_ns']-stamp)/1e6:.3f}, "
          f"'depth': {(depth_event['stamp_ns']-stamp)/1e6:.3f}}}")


if __name__ == "__main__":
    main()
