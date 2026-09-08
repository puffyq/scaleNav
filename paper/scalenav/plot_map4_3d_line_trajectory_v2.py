#!/usr/bin/env python3
"""Publication-quality 3-D trajectory figure for the Map4 height-changing experiment.

Three-panel single-row figure* (7.0 in wide):
  (a) oblique 3-D view with a faint voxel backdrop and the trial-1 trajectory,
  (b) top view (y horizontal, x vertical, equal aspect),
  (c) altitude profile z(y) with the nominal 3.5 m reference.

Data:
  - trial-1 odometry from the closed-loop batch gcn_3d_line_20260906_225711
    (shortest successful run, 149.203 m / 30.934 s, z span 1.808 m),
  - full 3-D mesh-truth occupancy volume map4_mesh_truth_3d_20260906.npz
    (0.5 m voxels; the *_20260906.ply variants are only flat height slabs).
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patheffects

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
BATCH = HERE / "test_data/closed_loop/gcn_3d_line_20260906_225711"
TRUTH_NPZ = HERE / "pics/map4_mesh_truth_3d_20260906.npz"
OUT = HERE / "pics/experiments/map4_3d_line/map4_3d_line_trajectory"

C_TRAJ = "#d62728"
C_OBST = "#bdbdbd"
C_TEXT = "#222222"


def read_record(line: str) -> dict:
    line = (line.replace(":-inf", ":-Infinity")
            .replace(":inf", ":Infinity")
            .replace(":nan", ":NaN"))
    return json.loads(line, parse_constant=lambda value: float(value))


def load_trial_trajectory(trial_json: Path) -> np.ndarray:
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
    return np.asarray(positions, dtype=float)


def load_obstacles(max_points: int = 13000) -> np.ndarray:
    data = np.load(TRUTH_NPZ)
    bounds = data["bounds"].astype(float)
    resolution = float(data["resolution"])
    cells = np.argwhere(data["occupied"] > 0)
    points = bounds[:, 0] + (cells.astype(float) + 0.5) * resolution
    mask = (
        (points[:, 0] >= -40.0) & (points[:, 0] <= 40.0)
        & (points[:, 1] >= -6.0) & (points[:, 1] <= 146.0)
        & (points[:, 2] >= 0.0) & (points[:, 2] <= 9.0)
    )
    points = points[mask]
    if len(points) > max_points:
        rng = np.random.default_rng(11)
        points = points[rng.choice(len(points), max_points, replace=False)]
    return points


def style_3d(ax) -> None:
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((1.0, 1.0, 1.0, 0.0))
        axis.pane.set_edgecolor("#d5d5d5")
        axis.pane.set_linewidth(0.5)
    ax.grid(False)
    ax.tick_params(labelsize=6.5, pad=1.5, colors="#555555")


def main() -> None:
    trajectory = load_trial_trajectory(BATCH / "trial_0001.json")
    obstacles = load_obstacles()
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
        1, 3, width_ratios=(1.45, 1.1, 1.0),
        left=0.045, right=0.995, bottom=0.155, top=0.93, wspace=0.34,
    )

    # (a) oblique 3-D view ------------------------------------------------
    ax3d = fig.add_subplot(grid[0, 0], projection="3d")
    ax3d.scatter(
        obstacles[:, 1], obstacles[:, 0], obstacles[:, 2],
        s=1.2, c=C_OBST, alpha=0.3, linewidths=0, depthshade=False,
    )
    traj3d, = ax3d.plot(
        trajectory[:, 1], trajectory[:, 0], trajectory[:, 2],
        color=C_TRAJ, linewidth=2.1, solid_capstyle="round", zorder=10,
    )
    traj3d.set_path_effects([
        patheffects.Stroke(linewidth=3.4, foreground="white", alpha=0.75),
        patheffects.Normal(),
    ])
    ax3d.scatter(
        trajectory[0, 1], trajectory[0, 0], trajectory[0, 2],
        s=26, c="#111111", edgecolors="white", linewidths=0.6,
        depthshade=False, zorder=11,
    )
    ax3d.scatter(
        140.0, 0.0, 3.5, marker="*", s=70, c=C_TRAJ,
        edgecolors="white", linewidths=0.6, depthshade=False, zorder=11,
    )
    ax3d.set_xlim(-6, 146)
    ax3d.set_ylim(-40, 40)
    ax3d.set_zlim(0, 8)
    ax3d.set_box_aspect((4.4, 2.4, 1.9))
    ax3d.view_init(elev=18, azim=-60)
    ax3d.set_xlabel("$y$ (m)", labelpad=0)
    ax3d.set_ylabel("$x$ (m)", labelpad=0)
    ax3d.set_zlabel("$z$ (m)", labelpad=-2)
    ax3d.set_xticks([0, 35, 70, 105, 140])
    ax3d.set_yticks([-40, -20, 0, 20, 40])
    ax3d.set_zticks([0, 4, 8])
    style_3d(ax3d)
    ax3d.text2D(0.02, 0.94, "(a)", transform=ax3d.transAxes,
                fontsize=8.5, fontweight="bold", color=C_TEXT)

    # (b) top view ---------------------------------------------------------
    axt = fig.add_subplot(grid[0, 1])
    axt.scatter(
        obstacles[:, 1], obstacles[:, 0],
        s=0.55, c=C_OBST, alpha=0.5, linewidths=0, rasterized=True,
    )
    axt.plot(
        trajectory[:, 1], trajectory[:, 0],
        color=C_TRAJ, linewidth=1.8, solid_capstyle="round", zorder=5,
    )
    axt.scatter(
        trajectory[0, 1], trajectory[0, 0], s=22, c="#111111",
        edgecolors="white", linewidths=0.6, zorder=6, label="Start",
    )
    axt.scatter(
        140.0, 0.0, marker="*", s=60, c=C_TRAJ,
        edgecolors="white", linewidths=0.6, zorder=6, label="Goal",
    )
    axt.set_xlim(-6, 146)
    axt.set_ylim(-40, 40)
    axt.set_aspect("equal", adjustable="box")
    axt.set_xticks([0, 35, 70, 105, 140])
    axt.set_yticks([-40, -20, 0, 20, 40])
    axt.set_xlabel("$y$ (m)")
    axt.set_ylabel("$x$ (m)")
    axt.tick_params(colors="#555555", length=2.5)
    for spine in axt.spines.values():
        spine.set_color("#999999")
    axt.text(0.03, 0.965, "(b)", transform=axt.transAxes,
             fontsize=8.5, fontweight="bold", va="top", color=C_TEXT)
    axt.legend(
        loc="upper right", fontsize=6.5, frameon=False, borderpad=0.1,
        handletextpad=0.3, labelspacing=0.2,
    )

    # (c) altitude profile -------------------------------------------------
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
    z_min = float(trajectory[:, 2].min())
    y_at_min = float(trajectory[np.argmin(trajectory[:, 2]), 1])
    axz.annotate(
        f"$\\Delta z = {z_span:.2f}$ m",
        xy=(y_at_min, z_min), xytext=(y_at_min + 14, z_min - 0.02),
        fontsize=7.0, color=C_TEXT, ha="left", va="top",
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
    print(f"z span {z_span:.3f} m; obstacles {len(obstacles)} pts; "
          f"trajectory {len(trajectory)} samples")
    print(f"wrote {OUT}.pdf / .png")


if __name__ == "__main__":
    main()
