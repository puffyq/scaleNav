#!/usr/bin/env python3
"""Render a compact training-sample strip from one recorded ScaleNav log."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SESSION = REPO / "log_scalenav" / "session_20260902_082638_86"
RGB_FILE = "rgb/rgb_66.ppm"
DEPTH_FILE = "depth/depth_76.pgm"
GRAPH_FILE = "graph/graph_55.json"
POINTCLOUD_FILE = "pointcloud/pointcloud_131.pcd"
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


def read_pcd(path: Path) -> np.ndarray:
    """Read XYZ points from a logged ASCII PCD."""
    points = []
    data = False
    with path.open(encoding="ascii", errors="ignore") as stream:
        for line in stream:
            if line.lower().startswith("data"):
                data = True
                continue
            if not data:
                continue
            fields = line.split()
            if len(fields) >= 3:
                try:
                    points.append((float(fields[0]), float(fields[1]), float(fields[2])))
                except ValueError:
                    pass
    return np.asarray(points, dtype=float)


def rotate_points(points: np.ndarray, q: list[float]) -> np.ndarray:
    qx, qy, qz, qw = map(float, q)
    t = 2.0 * np.cross(np.array([qx, qy, qz]), points)
    return points + qw * t + np.cross(np.array([qx, qy, qz]), t)


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

    nodes = body_view(marker_points(graph, "scalenav_skeleton_nodes"), origin, yaw)
    edge_points = body_view(marker_points(graph, "scalenav_skeleton_edges"), origin, yaw)
    edges = edge_points.reshape(-1, 2, 2)
    path = body_view(marker_points(graph, "scalenav_astar_topology_path"), origin, yaw)
    cloud_cam = read_pcd(SESSION / POINTCLOUD_FILE)
    cloud_world = rotate_points(cloud_cam, pose["orientation"]) + origin[None, :]
    cloud = body_view(cloud_world, origin, yaw)
    mission_goal = body_view(np.asarray([[0.0, 140.0, origin[2]]]), origin, yaw)[0]
    mission_dir = mission_goal[:2] / max(np.linalg.norm(mission_goal[:2]), 1e-6)
    mission_tip = 31.0 * mission_dir
    frontier_goal = body_view(
        np.asarray([marker(graph, "scalenav_frontier_goal")["pose"]["position"]]),
        origin, yaw)[0]

    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 7,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    })
    fig, axes = plt.subplots(1, 3, figsize=(3.48, 1.02),
                             gridspec_kw={"wspace": 0.34})
    fig.subplots_adjust(left=0.01, right=0.99, top=0.80, bottom=0.04)

    def base(ax, show_graph=False):
        if len(cloud):
            ax.scatter(cloud[:, 0], cloud[:, 1], s=1.0, color="#88969C", alpha=0.38,
                       linewidths=0, zorder=1)
        if show_graph:
            for edge in edges[::5]:
                ax.plot(edge[:, 0], edge[:, 1], color="#B4C0C4", lw=0.30, alpha=0.58)
            ax.scatter(nodes[::2, 0], nodes[::2, 1], s=2.5, color="#526D77", alpha=0.86)
        ax.scatter(0, 0, marker="^", s=21, color="#273941", edgecolors="white", lw=0.3)
        ax.set_xlim(-22, 22)
        ax.set_ylim(-1, 38)
        ax.set_aspect("equal", adjustable="box")

    base(axes[0])
    axes[0].plot([0.0, mission_tip[0]], [0.0, mission_tip[1]], color="#167F78", lw=1.15,
                 solid_capstyle="round")
    axes[0].scatter(mission_tip[0], mission_tip[1], marker="*", s=24,
                    color="#167F78", edgecolors="white", lw=0.3)
    axes[0].text(mission_tip[0] - 4.0, mission_tip[1] - 2.0, "mission",
                 fontsize=4.0, color="#167F78", fontweight="bold", ha="right")
    axes[0].set_title("point cloud + mission", fontsize=5.2, loc="left", pad=1)

    base(axes[1])
    if len(path) > 1:
        axes[1].plot(path[:, 0], path[:, 1], color="#00878B", lw=1.35)
        axes[1].scatter(path[-1, 0], path[-1, 1], marker="o", s=12,
                        color="#C97820", edgecolors="white", lw=0.3)
    axes[1].text(-20.5, 34.0, r"GT path (A*)", fontsize=4.0,
                 color="#00878B", fontweight="bold", ha="left",
                 bbox=dict(facecolor="white", edgecolor="none", alpha=0.88, pad=1.0))
    axes[1].set_title("privileged GT path", fontsize=5.2, loc="left", pad=1)

    base(axes[2], show_graph=True)
    offsets = np.deg2rad([40, 20, 0, -20, -40])
    for column, angle in enumerate(offsets):
        end = np.array([-16 * np.sin(angle), 16 * np.cos(angle)])
        axes[2].plot([0, end[0]], [0, end[1]],
                     color="#7856D8" if column == TARGET_COLUMN else "#D8D0ED",
                     lw=1.25 if column == TARGET_COLUMN else 0.35,
                     alpha=1.0 if column == TARGET_COLUMN else 0.7)
    axes[2].scatter(frontier_goal[0], frontier_goal[1], marker="*", s=25,
                    color="#C97820", edgecolors="white", lw=0.3, zorder=5)
    axes[2].text(-20.5, 34.0, r"FrontierGCN $\rightarrow c=2$ (C)", fontsize=3.8,
                 color="#7856D8", fontweight="bold", ha="left",
                 bbox=dict(facecolor="white", edgecolor="none", alpha=0.88, pad=1.0))
    axes[2].set_title("FrontierGCN output", fontsize=5.2, loc="left", pad=1)

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
