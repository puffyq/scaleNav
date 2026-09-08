#!/usr/bin/env python3
"""Render a compact training-sample strip from one recorded ScaleNav log."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SESSION = REPO / "log_scalenav" / "session_20260902_082638_86"
RGB_FILE = "rgb/rgb_66.ppm"
DEPTH_FILE = "depth/depth_76.pgm"
GRAPH_FILE = "graph/graph_55.json"
TARGET_COLUMN = 2


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


def main() -> None:
    events = read_events()
    timing = min(
        (e for e in events if e.get("kind") == "timing" and
         e.get("data", {}).get("module") == "planner" and
         e.get("data", {}).get("searched")),
        key=lambda e: abs(int(e["seq"]) - 1766),
    )
    stamp = int(timing["stamp_ns"])
    odom = min((e for e in events if e.get("kind") == "odom"),
               key=lambda e: abs(int(e["stamp_ns"]) - stamp))
    pose = odom["data"]
    origin = np.asarray(pose["position"], dtype=float)
    qx, qy, qz, qw = map(float, pose["orientation"])
    yaw = float(np.arctan2(2 * (qw * qz + qx * qy),
                           1 - 2 * (qy * qy + qz * qz)))
    graph = json.loads((SESSION / GRAPH_FILE).read_text(encoding="utf-8"))

    rgb = np.asarray(Image.open(SESSION / RGB_FILE).convert("RGB"))
    depth = np.asarray(Image.open(SESSION / DEPTH_FILE), dtype=float) / 1000.0
    shown_depth = np.ma.masked_where(depth >= 20.0, depth)

    nodes = body_view(marker_points(graph, "scalenav_skeleton_nodes"), origin, yaw)
    edge_points = body_view(marker_points(graph, "scalenav_skeleton_edges"), origin, yaw)
    edges = edge_points.reshape(-1, 2, 2)
    path = body_view(marker_points(graph, "scalenav_astar_topology_path"), origin, yaw)
    semantic = body_view(marker_points(graph, "scalenav_semantic_points"), origin, yaw)

    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 7,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })
    fig, axes = plt.subplots(1, 3, figsize=(3.48, 0.98),
                             gridspec_kw={"wspace": 0.30})
    fig.subplots_adjust(left=0.01, right=0.99, top=0.80, bottom=0.04)

    axes[0].imshow(rgb, interpolation="nearest")
    axes[0].set_title("logged RGB", fontsize=5.2, loc="left", pad=1)
    axes[0].text(0.03, 0.05, "same frame as depth", transform=axes[0].transAxes,
                 fontsize=3.8, color="white", fontweight="bold",
                 bbox=dict(facecolor="#26363E", edgecolor="none", pad=1.2))

    axes[1].imshow(shown_depth, cmap="viridis", vmin=0.0, vmax=20.0,
                   interpolation="nearest")
    axes[1].set_title("logged depth", fontsize=5.2, loc="left", pad=1)
    axes[1].text(0.03, 0.05, "depth-limited input", transform=axes[1].transAxes,
                 fontsize=3.8, color="white", fontweight="bold",
                 bbox=dict(facecolor="#26363E", edgecolor="none", pad=1.2))

    ax = axes[2]
    for edge in edges:
        ax.plot(edge[:, 0], edge[:, 1], color="#B4C0C4", lw=0.34, alpha=0.8)
    ax.scatter(nodes[:, 0], nodes[:, 1], s=2.7, color="#526D77", alpha=0.9)
    if len(semantic):
        ax.scatter(semantic[:, 0], semantic[:, 1], marker="x", s=8,
                   color="#D6544D", lw=0.55, alpha=0.7)
    if len(path) > 1:
        ax.plot(path[:, 0], path[:, 1], color="#00878B", lw=1.35)
    ax.scatter(0, 0, marker="^", s=21, color="#273941", edgecolors="white", lw=0.3)
    offsets = np.deg2rad([40, 20, 0, -20, -40])
    for column, angle in enumerate(offsets):
        end = np.array([-16 * np.sin(angle), 16 * np.cos(angle)])
        ax.plot([0, end[0]], [0, end[1]],
                color="#7856D8" if column == TARGET_COLUMN else "#D8D0ED",
                lw=1.25 if column == TARGET_COLUMN else 0.35,
                alpha=1.0 if column == TARGET_COLUMN else 0.7)
    ax.text(-16.0, 29.2, r"privileged A* $c=2$ (C)", fontsize=4.0,
            color="#7856D8", fontweight="bold", ha="left",
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.88, pad=1.0))
    ax.set_title("graph + expert label", fontsize=5.2, loc="left", pad=1)
    ax.set_xlim(-17, 17)
    ax.set_ylim(-1, 32)
    ax.set_aspect("equal", adjustable="box")

    for ax in axes:
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#A5B0B4")
            spine.set_linewidth(0.45)

    for left, right in zip(axes[:-1], axes[1:]):
        left.annotate("", xy=(1.16, 0.50), xytext=(1.02, 0.50),
                      xycoords="axes fraction", textcoords="axes fraction",
                      arrowprops=dict(arrowstyle="->", color="#26363E", lw=0.7))

    out = HERE / "gcn_training_data_log"
    fig.savefig(out.with_suffix(".pdf"), dpi=400)
    fig.savefig(out.with_suffix(".png"), dpi=400)
    plt.close(fig)
    print(f"wrote {out}.pdf and .png; session={SESSION.name}; target={TARGET_COLUMN}")


if __name__ == "__main__":
    main()
