#!/usr/bin/env python3
"""Render global task context plus two local GCN training-label panels."""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import matplotlib
import numpy as np
import torch
from matplotlib.collections import PolyCollection
from matplotlib.patches import FancyArrowPatch, Polygon

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SESSION = REPO / "log_scalenav" / "session_20260906_223557_314"
RGB_FILE = "rgb/rgb_145.ppm"
GRAPH_FILE = "graph/graph_104.json"
OCCUPANCY_FILE = REPO / "train_gcn" / "global_occupancy_map2_r075_i12.pt"


def read_events() -> list[dict]:
    events = []
    for line in (SESSION / "index.jsonl").open(encoding="utf-8"):
        try:
            events.append(json.loads(line, parse_constant=lambda value: float(value)))
        except json.JSONDecodeError:
            pass
    return events


def body_view(points: np.ndarray, origin: np.ndarray, yaw: float) -> np.ndarray:
    delta = points[:, :2] - origin[None, :2]
    forward = np.cos(yaw) * delta[:, 0] + np.sin(yaw) * delta[:, 1]
    right = np.sin(yaw) * delta[:, 0] - np.cos(yaw) * delta[:, 1]
    return np.column_stack((right, forward))


def marker(graph: dict, name: str) -> dict:
    return next(m for m in graph["markers"] if m.get("ns") == name)


def marker_points(graph: dict, name: str) -> np.ndarray:
    values = marker(graph, name).get("points", [])
    return np.asarray(values, dtype=float).reshape(-1, 3)


def yaw_from_quaternion(q: list[float]) -> float:
    qx, qy, qz, qw = map(float, q)
    return float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


def draw_vehicle(ax: plt.Axes, heading: float, size: float = 2.0, zorder: int = 6) -> None:
    """Draw a nose-forward triangle. heading is CCW from +y in the current view."""
    verts = np.array([[0.0, size], [-0.55 * size, -0.45 * size], [0.55 * size, -0.45 * size]])
    cosine, sine = np.cos(heading), np.sin(heading)
    verts = verts @ np.array([[cosine, sine], [-sine, cosine]])
    ax.add_patch(Polygon(verts, closed=True, facecolor="#273941",
                         edgecolor="white", lw=0.3, zorder=zorder))


def fill_occupancy_cells(cells: np.ndarray) -> np.ndarray:
    """Fill enclosed interiors so privileged buildings render as solid footprints."""
    minimum = cells.min(axis=0) - 1
    maximum = cells.max(axis=0) + 1
    shape = maximum - minimum + 1
    occupied = np.zeros(tuple(shape), dtype=bool)
    local = cells - minimum
    occupied[local[:, 0], local[:, 1]] = True
    exterior = np.zeros_like(occupied)
    queue: deque[tuple[int, int]] = deque()
    for x in range(shape[0]):
        queue.extend(((x, 0), (x, shape[1] - 1)))
    for y in range(shape[1]):
        queue.extend(((0, y), (shape[0] - 1, y)))
    while queue:
        x, y = queue.popleft()
        if exterior[x, y] or occupied[x, y]:
            continue
        exterior[x, y] = True
        if x > 0:
            queue.append((x - 1, y))
        if x + 1 < shape[0]:
            queue.append((x + 1, y))
        if y > 0:
            queue.append((x, y - 1))
        if y + 1 < shape[1]:
            queue.append((x, y + 1))
    return np.argwhere(~exterior) + minimum


def load_privileged_cells(path: Path) -> tuple[np.ndarray, float]:
    data = torch.load(path, map_location="cpu", weights_only=False)
    cells = np.asarray(data["blocked"], dtype=np.int64)
    return fill_occupancy_cells(cells), float(data["resolution"])


def occupancy_polygons(cells: np.ndarray, resolution: float,
                       origin: np.ndarray | None = None, yaw: float | None = None,
                       x_limits: tuple[float, float] | None = None,
                       y_limits: tuple[float, float] | None = None) -> np.ndarray:
    """Return (N, 4, 2) cell quads in the requested view."""
    half = 0.5 * resolution
    corners = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
    centers = (cells.astype(float) + 0.5) * resolution
    if origin is not None and yaw is not None:
        viewed = body_view(np.column_stack((centers, np.zeros(len(centers)))), origin, yaw)
    else:
        viewed = centers
    if x_limits is not None and y_limits is not None:
        keep = ((viewed[:, 0] >= x_limits[0] - resolution) &
                (viewed[:, 0] <= x_limits[1] + resolution) &
                (viewed[:, 1] >= y_limits[0] - resolution) &
                (viewed[:, 1] <= y_limits[1] + resolution))
        centers = centers[keep]
    if not len(centers):
        return np.empty((0, 4, 2))
    world = centers[:, None, :] + corners[None, :, :]
    if origin is None or yaw is None:
        return world
    flat = world.reshape(-1, 2)
    viewed_corners = body_view(np.column_stack((flat, np.zeros(len(flat)))), origin, yaw)
    return viewed_corners.reshape(-1, 4, 2)


def draw_occupancy(ax: plt.Axes, polygons: np.ndarray) -> None:
    if not len(polygons):
        return
    ax.add_collection(PolyCollection(
        polygons, facecolors="#AEB8BB", edgecolors="none", alpha=0.84,
        antialiaseds=False, rasterized=True, zorder=0,
    ))


def main() -> None:
    events = read_events()
    next(e for e in events if e.get("kind") == "graph" and e.get("file") == GRAPH_FILE)
    rgb_event = next(e for e in events if e.get("kind") == "rgb" and e.get("file") == RGB_FILE)
    stamp = int(rgb_event["stamp_ns"])
    odom = min((e for e in events if e.get("kind") == "odom"),
               key=lambda e: abs(int(e["stamp_ns"]) - stamp))
    pose = odom["data"]
    origin = np.asarray(pose["position"], dtype=float)
    vehicle_yaw = yaw_from_quaternion(pose["orientation"])
    # Align the local panels with the mission axis. Candidate rays still live
    # in the vehicle yaw frame, so they are rotated by the residual heading.
    mission_world = np.asarray([0.0, 140.0], dtype=float)
    view_yaw = float(np.arctan2(mission_world[1] - origin[1],
                                mission_world[0] - origin[0]))
    heading_rel = vehicle_yaw - view_yaw
    gcn = min((e for e in events if e.get("kind") == "gcn_frontier_column"),
              key=lambda e: abs(int(e["stamp_ns"]) - stamp))
    target_column = int(gcn["data"]["column"])
    graph = json.loads((SESSION / GRAPH_FILE).read_text(encoding="utf-8"))

    odom_events = [e for e in events if e.get("kind") == "odom"]
    mission_start = np.asarray(odom_events[0]["data"]["position"], dtype=float)
    mission_goal_world = np.asarray([0.0, 140.0, origin[2]], dtype=float)
    flight_world = np.asarray([e["data"]["position"] for e in odom_events], dtype=float)

    nodes = body_view(marker_points(graph, "scalenav_skeleton_nodes"), origin, view_yaw)
    edge_points = body_view(marker_points(graph, "scalenav_skeleton_edges"), origin, view_yaw)
    edges = edge_points.reshape(-1, 2, 2)
    path = body_view(marker_points(graph, "scalenav_astar_topology_path"), origin, view_yaw)
    cells, resolution = load_privileged_cells(OCCUPANCY_FILE)
    global_x_limits = (-80.0, 76.0)
    global_y_limits = (-16.0, 148.0)
    local_x_limits = (-34.0, 26.0)
    local_y_limits = (-6.0, 58.0)
    global_polys = occupancy_polygons(cells, resolution,
                                      x_limits=global_x_limits, y_limits=global_y_limits)
    local_polys = occupancy_polygons(cells, resolution, origin, view_yaw,
                                     local_x_limits, local_y_limits)
    frontier_goal = body_view(
        np.asarray([marker(graph, "scalenav_frontier_goal")["pose"]["position"]]),
        origin, view_yaw)[0]

    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 7,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })
    fig, axes = plt.subplots(
        1, 3, figsize=(3.48, 1.28),
        gridspec_kw={"wspace": 0.26, "width_ratios": [1.0, 1.0, 1.0]},
    )
    fig.subplots_adjust(left=0.01, right=0.99, top=0.78, bottom=0.04)

    def local_base(ax, show_graph=False):
        draw_occupancy(ax, local_polys)
        if show_graph:
            for edge in edges[::5]:
                ax.plot(edge[:, 0], edge[:, 1], color="#B4C0C4", lw=0.30, alpha=0.58)
            ax.scatter(nodes[::2, 0], nodes[::2, 1], s=2.5, color="#526D77", alpha=0.86)
        draw_vehicle(ax, heading_rel)
        ax.set_xlim(*local_x_limits)
        ax.set_ylim(*local_y_limits)
        ax.set_aspect("equal", adjustable="box")

    draw_occupancy(axes[0], global_polys)
    axes[0].plot(flight_world[:, 0], flight_world[:, 1], color="#D3DBDD", lw=0.55,
                 alpha=0.85, zorder=2)
    axes[0].plot([mission_start[0], mission_goal_world[0]],
                 [mission_start[1], mission_goal_world[1]],
                 color="#167F78", lw=1.0, ls=(0, (2.0, 1.2)), zorder=3)
    axes[0].scatter(mission_start[0], mission_start[1], marker="o", s=15,
                    color="#273941", edgecolors="white", lw=0.3, zorder=4)
    axes[0].scatter(origin[0], origin[1], marker="^", s=19,
                    color="#273941", edgecolors="white", lw=0.3, zorder=4)
    axes[0].scatter(mission_goal_world[0], mission_goal_world[1], marker="*", s=28,
                    color="#167F78", edgecolors="white", lw=0.3, zorder=4)
    axes[0].text(mission_goal_world[0] - 3.0, mission_goal_world[1] - 5.0, "mission goal",
                 fontsize=3.8, color="#167F78", fontweight="bold", ha="right",
                 bbox=dict(facecolor="white", edgecolor="none", alpha=0.88, pad=0.7))
    axes[0].set_xlim(*global_x_limits)
    axes[0].set_ylim(*global_y_limits)
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_title("task overview", fontsize=5.2, loc="left", pad=1)

    local_base(axes[1])
    if len(path) > 1:
        axes[1].plot(path[:, 0], path[:, 1], color="white", lw=3.4,
                     solid_capstyle="round", zorder=3)
        axes[1].plot(path[:, 0], path[:, 1], color="#00878B", lw=2.2,
                     solid_capstyle="round", zorder=4)
        axes[1].scatter(path[-1, 0], path[-1, 1], marker="o", s=18,
                        color="#C97820", edgecolors="white", lw=0.3)
    axes[1].text(-15.0, 37.0, r"GT path (A*)", fontsize=4.0,
                 color="#00878B", fontweight="bold", ha="left",
                 bbox=dict(facecolor="white", edgecolor="none", alpha=0.88, pad=1.0))
    axes[1].set_title("local expert A*", fontsize=5.2, loc="left", pad=1)

    local_base(axes[2], show_graph=True)
    offsets = np.deg2rad([40, 20, 0, -20, -40])
    selected_end = None
    for column, offset in enumerate(offsets):
        angle = heading_rel + offset
        end = np.array([-16.0 * np.sin(angle), 16.0 * np.cos(angle)])
        selected = column == target_column
        axes[2].plot([0, end[0]], [0, end[1]],
                     color="#7856D8" if selected else "#D8D0ED",
                     lw=1.35 if selected else 0.35,
                     alpha=1.0 if selected else 0.7, zorder=5)
        if selected:
            selected_end = end
    axes[2].scatter(frontier_goal[0], frontier_goal[1], marker="*", s=25,
                    color="#C97820", edgecolors="white", lw=0.3, zorder=6)
    if selected_end is not None:
        axes[2].text(selected_end[0] - 1.2, selected_end[1] + 1.4, "C",
                     fontsize=4.2, color="#7856D8", fontweight="bold", ha="center")
    axes[2].text(-15.0, 37.0, r"FrontierGCN $\rightarrow$ C", fontsize=3.8,
                 color="#7856D8", fontweight="bold", ha="left",
                 bbox=dict(facecolor="white", edgecolor="none", alpha=0.88, pad=1.0))
    axes[2].set_title("local FrontierGCN", fontsize=5.2, loc="left", pad=1)

    for ax in axes:
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#A5B0B4")
            spine.set_linewidth(0.45)

    fig.canvas.draw()
    for left, right in zip(axes[:-1], axes[1:]):
        box_left = left.get_position()
        box_right = right.get_position()
        y = 0.5 * (box_left.y0 + box_left.y1)
        fig.add_artist(FancyArrowPatch(
            (box_left.x1 + 0.006, y), (box_right.x0 - 0.006, y),
            transform=fig.transFigure, arrowstyle="-|>", mutation_scale=6,
            color="#26363E", lw=0.7, clip_on=False,
        ))

    out = HERE / "gcn_training_data_log"
    fig.savefig(out.with_suffix(".pdf"), dpi=400)
    fig.savefig(out.with_suffix(".png"), dpi=400)
    plt.close(fig)
    print(f"wrote {out}.pdf and .png; session={SESSION.name}; graph={GRAPH_FILE}; "
          f"target={target_column}; heading_rel_deg={np.degrees(heading_rel):.1f}; "
          f"cells={len(cells)}")


if __name__ == "__main__":
    main()
