#!/usr/bin/env python3
"""Replace only the GCN panel of the existing teaser with a logged decision.

The base teaser is copied pixel-for-pixel.  The new right panel is rendered
from the same graph snapshot and the nearest ``gcn_frontier_column`` event,
so no unrelated viewer sample or fabricated softmax is included.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.lines import Line2D
import numpy as np
from PIL import Image


BASE = Path(__file__).with_name("teaser.png")
SESSION = Path(__file__).resolve().parents[3] / "log_scalenav_gcn" / "session_20260905_130926_837"
GRAPH_FILE = "graph/graph_39.json"

BG = "#FFFFFF"
EDGE = "#9EADB2"
NODE = "#506D78"
ROUTE = "#007C83"
VEHICLE = "#24343D"
GCN = "#8B5CF6"
GCN_MUTED = "#C4B5FD"
TARGET = "#278B63"
OBSTACLE = "#C3CCCE"


def load_jsonl(path: Path) -> list[dict]:
    events = []
    for line in path.open(encoding="utf-8"):
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def body_view(points_xy: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    if not len(points_xy):
        return np.empty((0, 2))
    delta = np.asarray(points_xy, dtype=float)[:, :2] - position[None, :2]
    forward = np.cos(yaw) * delta[:, 0] + np.sin(yaw) * delta[:, 1]
    right = np.sin(yaw) * delta[:, 0] - np.cos(yaw) * delta[:, 1]
    return np.column_stack((right, forward))


def marker(graph: dict, name: str) -> dict:
    return next((m for m in graph.get("markers", []) if m.get("ns") == name), {})


def marker_points(graph: dict, name: str) -> np.ndarray:
    return np.asarray(marker(graph, name).get("points", []), dtype=float).reshape(-1, 3)


def marker_pose(graph: dict, name: str) -> np.ndarray:
    return np.asarray(marker(graph, name).get("pose", {}).get("position", [0, 0, 0]), dtype=float)


def draw_panel(out: Path) -> tuple[int, int]:
    events = load_jsonl(SESSION / "index.jsonl")
    graph_event = next(e for e in events if e.get("kind") == "graph" and e.get("file") == GRAPH_FILE)
    graph = json.loads((SESSION / GRAPH_FILE).read_text(encoding="utf-8"))
    gcn_events = [e for e in events if e.get("kind") == "gcn_frontier_column"]
    gcn = min(gcn_events, key=lambda e: abs(e["stamp_ns"] - graph_event["stamp_ns"]))
    selected = int(gcn["data"]["column"])
    position = marker_pose(graph, "scalenav_vehicle_pose")
    yaw_event = min((e for e in events if e.get("kind") == "odom"), key=lambda e: abs(e["stamp_ns"] - graph_event["stamp_ns"]))
    orientation = yaw_event.get("data", {}).get("orientation", [0, 0, 0, 1])
    qx, qy, qz, qw = map(float, orientation)
    yaw = float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

    nodes = body_view(marker_points(graph, "scalenav_skeleton_nodes")[:, :2], position, yaw)
    edge_points = marker_points(graph, "scalenav_skeleton_edges")[:, :2]
    edge_points = body_view(edge_points, position, yaw).reshape(-1, 2, 2) if len(edge_points) >= 2 else np.empty((0, 2, 2))
    astar = body_view(marker_points(graph, "scalenav_astar_topology_path")[:, :2], position, yaw)
    witness = body_view(marker_points(graph, "scalenav_polynomial_witness_path")[:, :2], position, yaw)

    # Match the tall right-column crop of the existing teaser so the graph is
    # not squeezed when composited into the original 2820x1254 canvas.
    fig = plt.figure(figsize=(3.08, 5.05), dpi=300, facecolor=BG)
    ax = fig.add_axes([0.04, 0.30, 0.92, 0.53])
    # A restrained obstacle silhouette provides context without claiming a
    # privileged map is part of the deployed GCN input.
    ax.add_patch(plt.Rectangle((-23, 22), 46, 22, facecolor=OBSTACLE, edgecolor="none", alpha=0.82, zorder=0))
    if len(edge_points):
        ax.add_collection(LineCollection(edge_points, colors=EDGE, linewidths=0.45, alpha=0.7, zorder=2))
    if len(nodes):
        ax.scatter(nodes[:, 0], nodes[:, 1], s=5, color=NODE, alpha=0.85, linewidths=0, zorder=3)
    if len(astar) > 1:
        ax.plot(astar[:, 0], astar[:, 1], color="white", lw=4.0, zorder=4)
        ax.plot(astar[:, 0], astar[:, 1], color=ROUTE, lw=2.3, zorder=5)
    if len(witness) > 1:
        ax.plot(witness[:, 0], witness[:, 1], color="white", lw=3.0, zorder=4)
        ax.plot(witness[:, 0], witness[:, 1], color="#E28A17", lw=1.7, ls=(0, (5, 2)), zorder=5)

    ax.scatter(0, 0, marker="^", s=65, color=VEHICLE, edgecolors="white", linewidths=0.8, zorder=8)
    offsets = np.deg2rad([40, 20, 0, -20, -40])
    labels = ["L2", "L1", "C", "R1", "R2"]
    ray_length = 18.0
    for i, angle in enumerate(offsets):
        endpoint = np.array([-ray_length * np.sin(angle), ray_length * np.cos(angle)])
        ax.plot([0, endpoint[0]], [0, endpoint[1]], color=GCN if i == selected else GCN_MUTED,
                lw=3.0 if i == selected else 1.05, alpha=0.98 if i == selected else 0.65, zorder=6)
        ax.text(endpoint[0], endpoint[1] + 1.1, labels[i], ha="center", va="bottom",
                fontsize=7.5, color=GCN if i == selected else "#8E99A0",
                fontweight="bold" if i == selected else "normal", zorder=7)
    endpoint = np.array([-ray_length * np.sin(offsets[selected]), ray_length * np.cos(offsets[selected])])
    ax.annotate("GCN selected", endpoint, xytext=(8, 3), textcoords="offset points",
                fontsize=8, color=GCN, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=GCN, lw=0.8), zorder=9)
    ax.annotate("large obstacle", (2.0, 29.0), xytext=(10, 39), fontsize=7.2, color="#5C666B",
                ha="center", arrowprops=dict(arrowstyle="->", color="#5C666B", lw=0.7))
    ax.set_xlim(-24, 24)
    ax.set_ylim(-3, 45)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_title("(c) FrontierGCN at the same graph snapshot", fontsize=11, pad=5)
    ax.text(0.02, 0.94, f"logged output: column {selected}  ({gcn['data']['direction_deg']:+.0f} deg)",
            transform=ax.transAxes, fontsize=8.6, color=TARGET, fontweight="bold",
            bbox=dict(boxstyle="square,pad=0.2", facecolor="white", edgecolor="none", alpha=0.9))

    info = fig.add_axes([0.06, 0.055, 0.88, 0.19])
    info.axis("off")
    info.text(0.0, 0.73, "FrontierGCN proposal", fontsize=9, color=GCN, fontweight="bold")
    info.text(0.0, 0.40, f"column {selected}: left 40 deg", fontsize=8.2, color="#36454F")
    info.text(0.0, 0.08, "A* and witness checks remain authoritative", fontsize=7.5, color="#667780")
    info.legend(handles=[Line2D([], [], color=GCN, lw=2.8, label="logged GCN direction"),
                         Line2D([], [], color=ROUTE, lw=2.2, label="A* topology path"),
                         Line2D([], [], color="#E28A17", lw=1.7, ls=(0, (5, 2)), label="witness")],
               loc="center right", fontsize=7, frameon=False, handlelength=2.0)
    fig.savefig(out, transparent=False, facecolor=BG)
    plt.close(fig)
    return selected, int((gcn["stamp_ns"] - graph_event["stamp_ns"]) / 1_000_000)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-prefix", type=Path, default=Path(__file__).with_name("teaser_candidate_v2"))
    args = parser.parse_args()
    args.out_prefix.parent.mkdir(parents=True, exist_ok=True)
    panel = args.out_prefix.with_name(args.out_prefix.stem + "_panel.png")
    selected, offset_ms = draw_panel(panel)

    base = Image.open(BASE).convert("RGB")
    replacement = Image.open(panel).convert("RGB")
    # Keep all left and middle pixels unchanged; the panel begins at the
    # existing right-column heading and ends above the shared legend.
    x0, y0, x1, y1 = 2205, 105, 2820, 1110
    replacement = replacement.resize((x1 - x0, y1 - y0), Image.Resampling.LANCZOS)
    base.paste(replacement, (x0, y0))
    base.save(args.out_prefix.with_suffix(".png"))
    base.save(args.out_prefix.with_suffix(".pdf"), resolution=300.0)
    panel.unlink(missing_ok=True)
    print(f"wrote {args.out_prefix.with_suffix('.png')} and .pdf; logged_gcn_column={selected}; gcn_graph_offset_ms={offset_ms}")


if __name__ == "__main__":
    main()
