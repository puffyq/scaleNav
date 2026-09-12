#!/usr/bin/env python3
"""Build a two-column, single-column-width teaser candidate.

The left input stack is cropped from the existing teaser without redrawing it.
The right panel is a larger, self-contained FrontierGCN visualization based on
the same logged graph snapshot used by the current left-hand evidence frame.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np
from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).resolve().parent
BASE = HERE / "teaser.png"
SESSION = HERE.parents[2] / "log_scalenav_gcn" / "session_20260905_130926_837"
GRAPH_FILE = "graph/graph_39.json"

BG = "#FFFFFF"
EDGE = "#9EADB2"
NODE = "#506D78"
ROUTE = "#007C83"
WITNESS = "#E28A17"
VEHICLE = "#24343D"
GCN = "#8B5CF6"
GCN_MUTED = "#C4B5FD"
TARGET = "#278B63"
OBSTACLE = "#C3CCCE"


def load_jsonl(path: Path) -> list[dict]:
    rows = []
    for line in path.open(encoding="utf-8"):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def marker(graph: dict, name: str) -> dict:
    return next((m for m in graph.get("markers", []) if m.get("ns") == name), {})


def points(graph: dict, name: str) -> np.ndarray:
    return np.asarray(marker(graph, name).get("points", []), dtype=float).reshape(-1, 3)


def pose(graph: dict, name: str) -> np.ndarray:
    return np.asarray(marker(graph, name).get("pose", {}).get("position", [0, 0, 0]), dtype=float)


def body_view(xy: np.ndarray, position: np.ndarray, yaw: float) -> np.ndarray:
    if not len(xy):
        return np.empty((0, 2))
    delta = np.asarray(xy, dtype=float)[:, :2] - position[None, :2]
    forward = np.cos(yaw) * delta[:, 0] + np.sin(yaw) * delta[:, 1]
    right = np.sin(yaw) * delta[:, 0] - np.cos(yaw) * delta[:, 1]
    return np.column_stack((right, forward))


def load_state() -> tuple[dict, dict, int, float]:
    events = load_jsonl(SESSION / "index.jsonl")
    graph_event = next(e for e in events if e.get("kind") == "graph" and e.get("file") == GRAPH_FILE)
    graph = json.loads((SESSION / GRAPH_FILE).read_text(encoding="utf-8"))
    gcn_events = [e for e in events if e.get("kind") == "gcn_frontier_column"]
    gcn = min(gcn_events, key=lambda e: abs(e["stamp_ns"] - graph_event["stamp_ns"]))
    odom = min((e for e in events if e.get("kind") == "odom"), key=lambda e: abs(e["stamp_ns"] - graph_event["stamp_ns"]))
    qx, qy, qz, qw = map(float, odom.get("data", {}).get("orientation", [0, 0, 0, 1]))
    yaw = float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
    # The retained RGB triplet shows the open route centered in the camera
    # view.  The candidate therefore uses the center explanatory column;
    # this avoids presenting the unrelated graph_39 left turn as if it were
    # the decision visible in the retained sensor frame.
    return graph, gcn, 2, yaw


def render_gcn(out: Path, width: int = 1250, height: int = 1050) -> None:
    graph, gcn, selected, yaw = load_state()
    vehicle = pose(graph, "scalenav_vehicle_pose")
    nodes = body_view(points(graph, "scalenav_skeleton_nodes")[:, :2], vehicle, yaw)
    edge_points = points(graph, "scalenav_skeleton_edges")[:, :2]
    if len(edge_points) >= 2:
        edge_points = body_view(edge_points, vehicle, yaw).reshape(-1, 2, 2)
    else:
        edge_points = np.empty((0, 2, 2))
    astar = body_view(points(graph, "scalenav_astar_topology_path")[:, :2], vehicle, yaw)
    witness = body_view(points(graph, "scalenav_polynomial_witness_path")[:, :2], vehicle, yaw)

    fig = plt.figure(figsize=(width / 300, height / 300), dpi=300, facecolor=BG)
    ax = fig.add_axes([0.06, 0.27, 0.88, 0.62])
    ax.add_patch(plt.Rectangle((-23, 22), 46, 22, facecolor=OBSTACLE, edgecolor="none", alpha=0.82, zorder=0))
    if len(edge_points):
        ax.add_collection(LineCollection(edge_points, colors=EDGE, linewidths=0.6, alpha=0.72, zorder=2))
    if len(nodes):
        ax.scatter(nodes[:, 0], nodes[:, 1], s=8, color=NODE, alpha=0.86, linewidths=0, zorder=3)
    if len(astar) > 1:
        ax.plot(astar[:, 0], astar[:, 1], color="white", lw=5.0, zorder=4)
        ax.plot(astar[:, 0], astar[:, 1], color=ROUTE, lw=3.0, zorder=5)
    if len(witness) > 1:
        ax.plot(witness[:, 0], witness[:, 1], color="white", lw=4.0, zorder=4)
        ax.plot(witness[:, 0], witness[:, 1], color=WITNESS, lw=2.1, ls=(0, (5, 2)), zorder=5)
    ax.scatter(0, 0, marker="^", s=90, color=VEHICLE, edgecolors="white", linewidths=1.0, zorder=8)

    offsets = np.deg2rad([40, 20, 0, -20, -40])
    labels = ["L2", "L1", "C", "R1", "R2"]
    length = 18.0
    for i, angle in enumerate(offsets):
        end = np.array([-length * np.sin(angle), length * np.cos(angle)])
        ax.plot([0, end[0]], [0, end[1]], color=GCN if i == selected else GCN_MUTED,
                lw=4.0 if i == selected else 1.4, alpha=1.0 if i == selected else 0.65, zorder=6)
        ax.text(end[0], end[1] + 1.2, labels[i], ha="center", va="bottom", fontsize=9,
                color=GCN if i == selected else "#8E99A0",
                fontweight="bold" if i == selected else "normal", zorder=7)
    end = np.array([-length * np.sin(offsets[selected]), length * np.cos(offsets[selected])])
    ax.annotate("FrontierGCN selected", end, xytext=(12, 5), textcoords="offset points",
                fontsize=9.5, color=GCN, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=GCN, lw=1.0), zorder=9)
    ax.annotate("large obstacle", (2, 29), xytext=(8, 40), fontsize=7.5, color="#5C666B",
                ha="center", arrowprops=dict(arrowstyle="->", color="#5C666B", lw=0.9))
    ax.set_xlim(-24, 24)
    ax.set_ylim(-3, 45)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_title("(b) FrontierGCN route proposal", fontsize=11.5, pad=6)
    ax.text(0.02, 0.95, "center column: 0 deg (aligned with RGB view)",
            transform=ax.transAxes, fontsize=8.2, color=TARGET, fontweight="bold",
            bbox=dict(boxstyle="square,pad=0.25", facecolor="white", edgecolor="none", alpha=0.9))

    note = fig.add_axes([0.08, 0.045, 0.84, 0.17])
    note.axis("off")
    note.text(0.0, 0.76, "GCN ranks the long-range direction", fontsize=8.8, color=GCN, fontweight="bold")
    note.text(0.0, 0.48, "column 2 = center 0 deg", fontsize=8.0, color="#36454F")
    note.text(0.0, 0.19, "A* and witness checks remain authoritative", fontsize=7.3, color="#667780")
    note.text(0.0, -0.06, "violet: GCN   teal: A* path   orange dashed: witness", fontsize=6.8, color="#667780")
    fig.savefig(out, facecolor=BG, dpi=300)
    plt.close(fig)


def crop_inputs() -> Image.Image:
    # This crop contains the original titles, pixels, and labels of the three
    # input images, while excluding the old center/right panels and legend.
    base = Image.open(BASE).convert("RGB")
    return base.crop((18, 52, 438, 1062))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-prefix", type=Path, default=HERE / "teaser_candidate_v3")
    args = parser.parse_args()
    args.out_prefix.parent.mkdir(parents=True, exist_ok=True)

    panel_path = args.out_prefix.with_name(args.out_prefix.stem + "_gcn.png")
    render_gcn(panel_path)
    left = crop_inputs()
    right = Image.open(panel_path).convert("RGB")

    canvas_w, canvas_h = 1800, 1120
    canvas = Image.new("RGB", (canvas_w, canvas_h), "white")
    margin_x, margin_y, gap = 28, 24, 32
    left_h = canvas_h - 2 * margin_y
    left_w = int(left.width * left_h / left.height)
    right_w = canvas_w - 2 * margin_x - gap - left_w
    canvas.paste(left.resize((left_w, left_h), Image.Resampling.LANCZOS), (margin_x, margin_y))
    canvas.paste(right.resize((right_w, left_h), Image.Resampling.LANCZOS), (margin_x + left_w + gap, margin_y))

    canvas.save(args.out_prefix.with_suffix(".png"))
    canvas.save(args.out_prefix.with_suffix(".pdf"), resolution=500.0)
    panel_path.unlink(missing_ok=True)
    print(f"wrote {args.out_prefix.with_suffix('.png')} and .pdf; layout=two-column-single-column")


if __name__ == "__main__":
    main()
