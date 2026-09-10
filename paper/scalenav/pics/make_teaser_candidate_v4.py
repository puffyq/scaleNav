#!/usr/bin/env python3
"""Render a two-column TopoGuide teaser from one synchronized GCN flight log."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SESSION = REPO / "log_scalenav" / "session_20260906_223557_314"
RGB_FILE = "rgb/rgb_145.ppm"
SEMANTIC_FILE = "semantic/semantic_165.pgm"
DEPTH_FILE = "depth/depth_164.pgm"
GRAPH_FILE = "graph/graph_104.json"

BG = "#FFFFFF"
EDGE = "#A9B7BC"
NODE = "#526D77"
SEMANTIC = "#D6544D"
SEMANTIC_LINK = "#E49A3A"
ROUTE = "#00878B"
WITNESS = "#E58A17"
GCN = "#7856D8"
GCN_MUTED = "#C9BCEC"
VEHICLE = "#273941"
LOCAL = "#B44D8A"
FRONTIER = "#D37714"
DEPTH_CLIP_M = 20.0


def load_events() -> list[dict]:
    rows = []
    for line in (SESSION / "index.jsonl").open(encoding="utf-8"):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def event_for_file(events: list[dict], kind: str, file: str) -> dict:
    return next(e for e in events if e.get("kind") == kind and e.get("file") == file)


def nearest(events: list[dict], kind: str, stamp: int) -> dict:
    return min((e for e in events if e.get("kind") == kind),
               key=lambda e: abs(int(e["stamp_ns"]) - stamp))


def marker(graph: dict, name: str) -> dict:
    return next((m for m in graph.get("markers", []) if m.get("ns") == name), {})


def points(graph: dict, name: str) -> np.ndarray:
    values = marker(graph, name).get("points", [])
    return np.asarray(values, dtype=float).reshape(-1, 3) if values else np.empty((0, 3))


def pose(graph: dict, name: str) -> np.ndarray:
    value = marker(graph, name).get("pose", {}).get("position")
    if value is None:
        raise ValueError(f"missing graph pose: {name}")
    return np.asarray(value, dtype=float)


def yaw_from_quaternion(q: list[float]) -> float:
    qx, qy, qz, qw = map(float, q)
    return float(np.arctan2(2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz)))


def body_view(xyz: np.ndarray, origin: np.ndarray, yaw: float) -> np.ndarray:
    if not len(xyz):
        return np.empty((0, 2))
    delta = xyz[:, :2] - origin[None, :2]
    forward = np.cos(yaw) * delta[:, 0] + np.sin(yaw) * delta[:, 1]
    right = np.sin(yaw) * delta[:, 0] - np.cos(yaw) * delta[:, 1]
    return np.column_stack((right, forward))


def read_scalar(file: str) -> np.ndarray:
    return np.asarray(Image.open(SESSION / file), dtype=float) / 1000.0


def draw_inputs(axes: list[plt.Axes], rgb: np.ndarray,
                semantic: np.ndarray, depth: np.ndarray) -> None:
    heatmap = LinearSegmentedColormap.from_list(
        "pearl", ["#160B39", "#482173", "#8B2981", "#D94E63", "#F6D746"])
    axes[0].imshow(rgb, interpolation="nearest")
    axes[1].imshow(semantic, cmap=heatmap, vmin=0.0, vmax=1.0,
                   interpolation="nearest")
    shown_depth = np.ma.masked_where(depth >= DEPTH_CLIP_M, depth)
    depth_cmap = plt.get_cmap("viridis").copy()
    depth_cmap.set_bad("white")
    axes[2].imshow(shown_depth, cmap=depth_cmap, vmin=0.0,
                   vmax=DEPTH_CLIP_M, interpolation="nearest")

    titles = ["RGB", r"PEARL response $S_t$", r"Depth $D_t$ (m)"]
    notes = [None, "open-vocabulary score", r"white: no return ($>20$ m)"]
    for axis, title, note in zip(axes, titles, notes):
        axis.set_title(title, loc="left", fontsize=5.8, pad=1)
        axis.set_xticks([])
        axis.set_yticks([])
        for spine in axis.spines.values():
            spine.set_color("#89979D")
            spine.set_linewidth(0.65)
        if note:
            axis.text(0.025, 0.055, note, transform=axis.transAxes,
                      ha="left", va="bottom", color="white", fontsize=3.7,
                      fontweight="bold",
                      bbox=dict(boxstyle="square,pad=0.12", facecolor="#26343C",
                                edgecolor="none", alpha=0.92))


def draw_route(axis: plt.Axes, graph: dict, origin: np.ndarray, yaw: float,
               selected: int) -> None:
    node_points = points(graph, "scalenav_skeleton_nodes")
    nodes = body_view(node_points, origin, yaw)
    edges = body_view(points(graph, "scalenav_skeleton_edges"), origin, yaw).reshape(-1, 2, 2)
    semantic_points = points(graph, "scalenav_semantic_points")
    semantic = body_view(semantic_points, origin, yaw)
    current_semantic_points = points(graph, "scalenav_current_semantic_points")
    current_semantic = body_view(current_semantic_points, origin, yaw)
    semantic_links_raw = body_view(points(graph, "scalenav_semantic_links"), origin, yaw)
    semantic_links = semantic_links_raw.reshape(-1, 2, 2) if len(semantic_links_raw) else np.empty((0, 2, 2))
    topology_path = body_view(points(graph, "scalenav_astar_topology_path"), origin, yaw)
    witness = body_view(points(graph, "scalenav_polynomial_witness_path"), origin, yaw)
    local_goal = body_view(pose(graph, "scalenav_local_goal")[None, :], origin, yaw)[0]
    frontier_goal = body_view(pose(graph, "scalenav_frontier_goal")[None, :], origin, yaw)[0]

    # The obstacle is schematic context; all graph, route, and goal geometry is logged.
    axis.fill_between([5.5, 31.0], [19.0, 19.0], [38.0, 38.0],
                      color="#D9DEDF", alpha=0.88, zorder=0)

    axis.add_collection(LineCollection(edges, colors=EDGE, linewidths=0.45,
                                       alpha=0.64, zorder=2))
    axis.scatter(nodes[:, 0], nodes[:, 1], s=4.2, color=NODE, alpha=0.85,
                 linewidths=0, zorder=3)
    if len(semantic_links):
        axis.add_collection(LineCollection(semantic_links, colors=SEMANTIC_LINK,
                                           linewidths=0.58, linestyles=(0, (1.2, 1.5)),
                                           alpha=0.48, zorder=2))
    axis.scatter(semantic[:, 0], semantic[:, 1], marker="x", s=13,
                 color=SEMANTIC, linewidths=0.75, alpha=0.62, zorder=3)
    axis.scatter(current_semantic[:, 0], current_semantic[:, 1], marker="x", s=23,
                 color=SEMANTIC, linewidths=1.15, alpha=0.92, zorder=5)

    if len(topology_path) > 1:
        axis.plot(topology_path[:, 0], topology_path[:, 1], color="white",
                  lw=3.6, zorder=5)
        axis.plot(topology_path[:, 0], topology_path[:, 1], color=ROUTE,
                  lw=2.2, zorder=6)
    if len(witness) > 1:
        axis.plot(witness[:, 0], witness[:, 1], color=WITNESS, lw=1.55,
                  ls=(0, (4, 2)), zorder=7)

    offsets = np.deg2rad([40, 20, 0, -20, -40])
    labels = ["L2", "L1", "C", "R1", "R2"]
    ray_length = 17.0
    for column, angle in enumerate(offsets):
        endpoint = np.array([-ray_length * np.sin(angle), ray_length * np.cos(angle)])
        active = column == selected
        axis.annotate("", xy=endpoint, xytext=(0, 0),
                      arrowprops=dict(arrowstyle="-|>", mutation_scale=8,
                                      color=GCN if active else GCN_MUTED,
                                      lw=2.3 if active else 0.65,
                                      alpha=1.0 if active else 0.55), zorder=8)
        if active:
            axis.text(endpoint[0], endpoint[1] + 0.7, labels[column], ha="center",
                      fontsize=5.6, color=GCN, fontweight="bold", zorder=9)

    axis.scatter(0, 0, marker="^", s=52, color=VEHICLE,
                 edgecolors="white", linewidths=0.7, zorder=10)
    axis.scatter(local_goal[0], local_goal[1], marker="D", s=39, color=LOCAL,
                 edgecolors="white", linewidths=0.65, zorder=10)
    axis.scatter(frontier_goal[0], frontier_goal[1], marker="*", s=88,
                 color=FRONTIER, edgecolors="white", linewidths=0.65, zorder=10)
    axis.annotate("local goal", local_goal, xytext=(1.8, 9.2), textcoords="data",
                  fontsize=4.8, color=LOCAL, fontweight="bold",
                  ha="left", va="center",
                  bbox=dict(boxstyle="round,pad=0.16", facecolor="white",
                            edgecolor="none", alpha=0.86),
                  arrowprops=dict(arrowstyle="->", color=LOCAL, lw=0.7,
                                  shrinkA=2, shrinkB=3))
    axis.annotate("frontier goal", frontier_goal, xytext=(7, 5),
                  textcoords="offset points", fontsize=4.8, color=FRONTIER,
                  fontweight="bold",
                  arrowprops=dict(arrowstyle="-", color=FRONTIER, lw=0.7))
    axis.annotate("GCN selects C (0 deg)", (0, 16.2), xytext=(4.5, 17.3),
                  textcoords="data", fontsize=4.6, color=GCN,
                  fontweight="bold",
                  arrowprops=dict(arrowstyle="->", color=GCN, lw=0.8))
    axis.annotate("semantic frontier", frontier_goal,
                  xytext=(-11.5, 33.4), textcoords="data", fontsize=4.5,
                  color=SEMANTIC, ha="left",
                  arrowprops=dict(arrowstyle="->", color=SEMANTIC, lw=0.7))

    axis.set_xlim(-13, 19)
    axis.set_ylim(-4, 38)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_visible(False)


def main() -> None:
    events = load_events()
    rgb_event = event_for_file(events, "rgb", RGB_FILE)
    semantic_event = event_for_file(events, "semantic", SEMANTIC_FILE)
    depth_event = event_for_file(events, "depth", DEPTH_FILE)
    graph_event = event_for_file(events, "graph", GRAPH_FILE)
    stamp = int(rgb_event["stamp_ns"])
    gcn_event = nearest(events, "gcn_frontier_column", stamp)
    odom_event = nearest(events, "odom", stamp)

    assert semantic_event["stamp_ns"] == stamp
    assert depth_event["stamp_ns"] == stamp
    assert abs(int(graph_event["stamp_ns"]) - stamp) <= 15_000_000
    assert abs(int(gcn_event["stamp_ns"]) - stamp) <= 35_000_000
    assert int(gcn_event["data"]["column"]) == 2

    rgb = np.asarray(Image.open(SESSION / RGB_FILE).convert("RGB"))
    semantic = np.clip(read_scalar(SEMANTIC_FILE), 0.0, 1.0)
    depth = read_scalar(DEPTH_FILE)
    graph = json.loads((SESSION / GRAPH_FILE).read_text(encoding="utf-8"))
    origin = np.asarray(odom_event["data"]["position"], dtype=float)
    yaw = yaw_from_quaternion(odom_event["data"]["orientation"])

    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 7,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "figure.facecolor": BG,
        "savefig.facecolor": BG,
    })
    figure = plt.figure(figsize=(3.48, 2.28), facecolor=BG)
    grid = figure.add_gridspec(1, 2, width_ratios=[0.39, 0.61], wspace=0.035,
                               left=0.025, right=0.99, top=0.89, bottom=0.035)
    left = grid[0].subgridspec(3, 1, hspace=0.24)
    input_axes = [figure.add_subplot(left[i]) for i in range(3)]
    draw_inputs(input_axes, rgb, semantic, depth)
    route_axis = figure.add_subplot(grid[1])
    draw_route(route_axis, graph, origin, yaw, int(gcn_event["data"]["column"]))
    figure.text(0.025, 0.965, r"(a) Inputs at $t^*$", fontsize=6.5,
                ha="left", va="top")
    figure.text(0.415, 0.965, "(b) FrontierGCN route decision", fontsize=6.5,
                ha="left", va="top")

    out = HERE / "teaser_candidate_v4"
    figure.savefig(out.with_suffix(".pdf"), dpi=400)
    figure.savefig(out.with_suffix(".png"), dpi=400)
    plt.close(figure)
    print(
        f"wrote {out.with_suffix('.pdf')} and .png; session={SESSION.name}; "
        f"rgb_stamp={stamp}; graph_offset_ms={(graph_event['stamp_ns']-stamp)/1e6:.3f}; "
        f"gcn_column={gcn_event['data']['column']}; "
        f"gcn_offset_ms={(gcn_event['stamp_ns']-stamp)/1e6:.3f}"
    )


if __name__ == "__main__":
    main()
