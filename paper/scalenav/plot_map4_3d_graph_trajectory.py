#!/usr/bin/env python3
"""Publication figure: online 3-D skeleton graph + height-changing trajectory (Map4).

Three-panel single-row figure* (7.0 in wide):
  (a) oblique 3-D view of the logged online skeleton (nodes + edges) at the
      mid-mission snapshot when the vehicle is deepest in the power-line dip
      (graph_96.json, t = 16.0 s, vehicle z = 1.82 m), the logged polynomial
      witness plan, and the flown trial-1 trajectory;
  (b) top view of the same snapshot (y horizontal, x vertical, equal aspect);
  (c) altitude profile z(y) with the nominal 3.5 m reference and a marker at
      the snapshot position.

All geometry comes from the trial-1 flight log
(log_scalenav/session_20260906_225714_272): marker-array snapshots under
graph/graph_*.json (world_enu frame) and odometry in index.jsonl.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patheffects
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection

HERE = Path(__file__).resolve().parent
BATCH = HERE / "test_data/closed_loop/gcn_3d_line_20260906_225711"
SESSION = Path("/mnt/code/lab/yopo/OpenSeek/log_scalenav/session_20260906_225714_272")
SNAPSHOT = SESSION / "graph/graph_96.json"
OUT = HERE / "pics/experiments/map4_3d_line/map4_3d_graph_trajectory"

C_TRAJ = "#d62728"
C_NODE = "#3d5a80"
C_EDGE = "#aebccf"
C_PLAN = "#123c69"
C_TEXT = "#222222"


def read_record(line: str) -> dict:
    line = (line.replace(":-inf", ":-Infinity")
            .replace(":inf", ":Infinity")
            .replace(":nan", ":NaN"))
    return json.loads(line, parse_constant=lambda value: float(value))


def load_trial_trajectory(trial_json: Path):
    info = json.loads(trial_json.read_text(encoding="utf-8"))
    index_path = Path(info["session_dir"]) / "index.jsonl"
    start_ns = end_ns = None
    with index_path.open(encoding="utf-8") as stream:
        for line in stream:
            record = read_record(line)
            if record.get("kind") != "mission":
                continue
            event = (record.get("data") or {}).get("event")
            if event == "start" and start_ns is None:
                start_ns = int(record["stamp_ns"])
            elif event in {"complete", "stop", "collision", "timeout"}:
                end_ns = int(record["stamp_ns"])
    positions = []
    with index_path.open(encoding="utf-8") as stream:
        for line in stream:
            record = read_record(line)
            if record.get("kind") != "odom":
                continue
            stamp = int(record.get("stamp_ns", 0))
            if start_ns <= stamp <= end_ns:
                positions.append((record.get("data") or {})["position"])
    if len(positions) < 2:
        raise ValueError(f"no odometry inside mission interval of {index_path}")
    return np.asarray(positions, dtype=float), start_ns, end_ns


def load_graph_snapshot(path: Path):
    markers = json.loads(path.read_text(encoding="utf-8"))["markers"]
    data = {}
    for marker in markers:
        ns = marker.get("ns")
        points = np.asarray(marker.get("points", []), dtype=float)
        if ns == "scalenav_skeleton_nodes":
            data["nodes"] = points
        elif ns == "scalenav_skeleton_edges":
            data["edges"] = points.reshape(-1, 2, 3)
        elif ns == "scalenav_polynomial_witness_path":
            data["witness"] = points
        elif ns == "scalenav_vehicle_pose":
            data["vehicle"] = np.asarray(marker["pose"]["position"], dtype=float)
    missing = {"nodes", "edges", "witness", "vehicle"} - data.keys()
    if missing:
        raise ValueError(f"snapshot {path} missing {sorted(missing)}")
    return data


def style_3d(ax) -> None:
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((1.0, 1.0, 1.0, 0.0))
        axis.pane.set_edgecolor("#d5d5d5")
        axis.pane.set_linewidth(0.5)
    ax.grid(False)
    ax.tick_params(labelsize=6.5, pad=1.5, colors="#555555")


def main() -> None:
    trajectory, start_ns, end_ns = load_trial_trajectory(BATCH / "trial_0001.json")
    graph = load_graph_snapshot(SNAPSHOT)
    nodes, edges, witness, vehicle = (
        graph["nodes"], graph["edges"], graph["witness"], graph["vehicle"],
    )
    z_span = float(trajectory[:, 2].max() - trajectory[:, 2].min())

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 8.0,
        "axes.labelsize": 8.0,
        "xtick.labelsize": 7.0,
        "ytick.labelsize": 7.0,
        "axes.linewidth": 0.6,
        "text.color": C_TEXT,
        "axes.labelcolor": C_TEXT,
    })

    fig = plt.figure(figsize=(7.0, 2.5))
    grid = fig.add_gridspec(
        1, 3, width_ratios=(1.5, 1.05, 1.0),
        left=0.04, right=0.995, bottom=0.155, top=0.93, wspace=0.34,
    )

    # (a) oblique 3-D view -------------------------------------------------
    ax3d = fig.add_subplot(grid[0, 0], projection="3d")
    ax3d.add_collection3d(Line3DCollection(
        edges[:, :, [1, 0, 2]], colors=C_EDGE, linewidths=0.25, alpha=0.16,
    ))
    ax3d.scatter(
        nodes[:, 1], nodes[:, 0], nodes[:, 2],
        s=1.6, c=C_NODE, alpha=0.35, linewidths=0, depthshade=False,
    )
    ax3d.plot(
        witness[:, 1], witness[:, 0], witness[:, 2],
        color=C_PLAN, linewidth=1.3, linestyle=(0, (3, 2)),
    )
    traj3d, = ax3d.plot(
        trajectory[:, 1], trajectory[:, 0], trajectory[:, 2],
        color=C_TRAJ, linewidth=2.1, solid_capstyle="round",
    )
    traj3d.set_path_effects([
        patheffects.Stroke(linewidth=3.4, foreground="white", alpha=0.8),
        patheffects.Normal(),
    ])
    ax3d.scatter(
        trajectory[0, 1], trajectory[0, 0], trajectory[0, 2],
        s=36, c="#111111", edgecolors="white", linewidths=0.7,
        depthshade=False,
    )
    ax3d.scatter(
        140.0, 0.0, 3.5, marker="*", s=80, c=C_TRAJ,
        edgecolors="white", linewidths=0.7, depthshade=False,
    )
    ax3d.scatter(
        vehicle[1], vehicle[0], vehicle[2], marker="D",
        s=22, c="#f2a900", edgecolors="white", linewidths=0.6,
        depthshade=False,
    )
    ax3d.set_xlim(-6, 146)
    ax3d.set_ylim(-30, 30)
    ax3d.set_zlim(0, 13)
    ax3d.set_box_aspect((4.4, 2.2, 2.3))
    ax3d.view_init(elev=16, azim=-60)
    ax3d.set_xticks([0, 35, 70, 105, 140])
    ax3d.set_yticks([-30, -15, 0, 15, 30])
    ax3d.set_zticks([0, 4, 8, 12])
    style_3d(ax3d)
    ax3d.text2D(0.30, 0.055, "$y$ (m)", transform=ax3d.transAxes, fontsize=8)
    ax3d.text2D(0.955, 0.015, "$x$ (m)", transform=ax3d.transAxes, fontsize=8)
    ax3d.text2D(1.005, 0.47, "$z$ (m)", transform=ax3d.transAxes,
                fontsize=8, rotation=90, va="center")
    ax3d.text2D(0.02, 0.94, "(a)", transform=ax3d.transAxes,
                fontsize=8.5, fontweight="bold", color=C_TEXT)

    # (b) top view ----------------------------------------------------------
    axt = fig.add_subplot(grid[0, 1])
    axt.add_collection(LineCollection(
        edges[:, :, [1, 0]], colors=C_EDGE, linewidths=0.25, alpha=0.16,
        rasterized=True,
    ))
    axt.scatter(
        nodes[:, 1], nodes[:, 0], s=1.2, c=C_NODE, alpha=0.3,
        linewidths=0, rasterized=True, label="Skeleton",
    )
    axt.plot(
        witness[:, 1], witness[:, 0], color=C_PLAN, linewidth=1.2,
        linestyle=(0, (3, 2)), zorder=4, label="Witness plan",
    )
    trajt, = axt.plot(
        trajectory[:, 1], trajectory[:, 0], color=C_TRAJ, linewidth=1.8,
        solid_capstyle="round", zorder=5, label="Trajectory",
    )
    trajt.set_path_effects([
        patheffects.Stroke(linewidth=3.0, foreground="white", alpha=0.8),
        patheffects.Normal(),
    ])
    axt.scatter(
        trajectory[0, 1], trajectory[0, 0], s=22, c="#111111",
        edgecolors="white", linewidths=0.6, zorder=6, label="Start",
    )
    axt.scatter(
        140.0, 0.0, marker="*", s=60, c=C_TRAJ,
        edgecolors="white", linewidths=0.6, zorder=6, label="Goal",
    )
    axt.scatter(
        vehicle[1], vehicle[0], marker="D", s=18, c="#f2a900",
        edgecolors="white", linewidths=0.5, zorder=6, label="Vehicle (snapshot)",
    )
    axt.set_xlim(-6, 146)
    axt.set_ylim(-32, 32)
    axt.set_aspect("equal", adjustable="box")
    axt.set_xticks([0, 35, 70, 105, 140])
    axt.set_yticks([-30, -15, 0, 15, 30])
    axt.set_xlabel("$y$ (m)")
    axt.set_ylabel("$x$ (m)")
    axt.tick_params(colors="#555555", length=2.5)
    for spine in axt.spines.values():
        spine.set_color("#999999")
    axt.text(0.03, 0.965, "(b)", transform=axt.transAxes,
             fontsize=8.5, fontweight="bold", va="top", color=C_TEXT)
    axt.legend(
        loc="lower right", fontsize=5.8, frameon=True, framealpha=0.92,
        edgecolor="#cccccc", borderpad=0.25, handletextpad=0.35,
        labelspacing=0.22, markerscale=0.8,
    )

    # (c) altitude profile ---------------------------------------------------
    axz = fig.add_subplot(grid[0, 2])
    axz.plot(
        trajectory[:, 1], trajectory[:, 2],
        color=C_TRAJ, linewidth=1.8, solid_capstyle="round",
    )
    axz.axhline(3.5, color="#777777", linestyle=(0, (4, 3)), linewidth=0.9)
    axz.text(
        3, 3.58, "nominal 3.5 m", fontsize=6.5, color="#666666",
        ha="left", va="bottom",
    )
    axz.axvline(vehicle[1], color="#f2a900", linestyle=(0, (2, 2)),
                linewidth=0.9)
    axz.text(
        vehicle[1] + 2, 1.32, "snapshot", fontsize=6.2, color="#c07f00",
        ha="left", va="bottom", rotation=0,
    )
    z_min = float(trajectory[:, 2].min())
    y_at_min = float(trajectory[np.argmin(trajectory[:, 2]), 1])
    axz.annotate(
        f"$\\Delta z = {z_span:.2f}$ m",
        xy=(y_at_min, z_min), xytext=(y_at_min + 16, z_min + 0.05),
        fontsize=7.0, color=C_TEXT, ha="left", va="center",
        arrowprops={"arrowstyle": "-", "color": "#888888", "linewidth": 0.7},
    )
    axz.set_xlim(-6, 146)
    axz.set_ylim(1.2, 4.4)
    axz.set_xticks([0, 35, 70, 105, 140])
    axz.set_yticks([1.5, 2.0, 2.5, 3.0, 3.5, 4.0])
    axz.set_xlabel("$y$ (m)")
    axz.set_ylabel("$z$ (m)")
    axz.grid(axis="y", color="#e3e3e3", linewidth=0.5)
    axz.set_axisbelow(True)
    axz.tick_params(colors="#555555", length=2.5)
    for spine in ("top", "right"):
        axz.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        axz.spines[spine].set_color("#999999")
    axz.text(0.04, 0.945, "(c)", transform=axz.transAxes,
             fontsize=8.5, fontweight="bold", va="top", color=C_TEXT)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT.with_suffix(".pdf"), bbox_inches="tight", pad_inches=0.02)
    fig.savefig(OUT.with_suffix(".png"), dpi=300, bbox_inches="tight",
                pad_inches=0.02)
    print(f"snapshot vehicle at {vehicle.round(2)}; nodes {len(nodes)}, "
          f"edges {len(edges)}, witness {len(witness)} pts; "
          f"trajectory {len(trajectory)} samples, z span {z_span:.3f} m")
    print(f"wrote {OUT}.pdf / .png")


if __name__ == "__main__":
    main()
