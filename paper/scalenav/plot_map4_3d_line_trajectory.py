#!/usr/bin/env python3
"""Plot Map4 3-D line-bypass trajectory evidence for the backup experiment."""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patheffects


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
HELPER_PATH = HERE / "plot_obstacle_heatmap_trajectory.py"
SUMMARY = REPO_ROOT / "scalenav_ws/src/aut_test/results/run_20260906_225711_315755/summary.csv"
TRUTH = HERE / "pics/map4_mesh_truth_3d_20260906.npz"
OUT = HERE / "pics/candidates/map4_3d_line_trajectory_candidate"


def load_helper():
    spec = importlib.util.spec_from_file_location("map4_plot_helper", HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(HELPER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def shortest_success_session(summary_path: Path) -> tuple[Path, dict[str, str]]:
    with summary_path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    successes = [row for row in rows if row.get("outcome") == "success"]
    if not successes:
        raise RuntimeError("summary has no successful trials")
    row = min(successes, key=lambda item: float(item["path_m"]))
    return Path(row["session_dir"]) / "index.jsonl", row


def occupied_points(path: Path) -> np.ndarray:
    data = np.load(path)
    occupied = np.argwhere(data["occupied"] > 0)
    bounds = data["bounds"].astype(float)
    resolution = float(data["resolution"])
    points = bounds[:, 0] + (occupied.astype(float) + 0.5) * resolution
    mask = (
        (points[:, 0] >= -45.0) & (points[:, 0] <= 45.0) &
        (points[:, 1] >= -5.0) & (points[:, 1] <= 145.0) &
        (points[:, 2] >= 0.0) & (points[:, 2] <= 8.0)
    )
    return points[mask]


def sample_points(points: np.ndarray, max_points: int, seed: int) -> np.ndarray:
    if len(points) <= max_points:
        return points
    rng = np.random.default_rng(seed)
    return points[rng.choice(len(points), max_points, replace=False)]


def trajectory_line(ax, x, y, *args, **kwargs):
    line, = ax.plot(x, y, *args, **kwargs)
    line.set_path_effects([
        patheffects.Stroke(linewidth=kwargs.get("linewidth", 2.0) + 2.0,
                           foreground="white", alpha=0.95),
        patheffects.Normal(),
    ])
    return line


def set_clean_3d(ax):
    ax.xaxis.pane.set_facecolor((1, 1, 1, 0))
    ax.yaxis.pane.set_facecolor((1, 1, 1, 0))
    ax.zaxis.pane.set_facecolor((1, 1, 1, 0))
    ax.grid(True, color="#d9dde1", linewidth=0.45)


def main() -> None:
    helper = load_helper()
    index_path, row = shortest_success_session(SUMMARY)
    start_ns, end_ns = helper.mission_interval(index_path)
    trajectory, _ = helper.load_trajectory(index_path, start_ns, end_ns)
    obstacles = occupied_points(TRUTH)

    line_band = obstacles[
        (obstacles[:, 2] >= 3.0) & (obstacles[:, 2] <= 4.2) &
        (np.abs(obstacles[:, 0]) <= 13.0) &
        (obstacles[:, 1] >= -2.0) & (obstacles[:, 1] <= 125.0)
    ]
    context = sample_points(obstacles, 4200, seed=7)
    line_context = sample_points(line_band, 2200, seed=11)

    plt.rcParams.update({"font.size": 7.8, "font.family": "DejaVu Sans"})
    fig = plt.figure(figsize=(9.5, 3.9))
    grid = fig.add_gridspec(
        2, 2, width_ratios=(1.55, 1.0), height_ratios=(1.0, 0.76),
        left=0.035, right=0.985, bottom=0.13, top=0.91,
        wspace=0.23, hspace=0.36,
    )

    ax3d = fig.add_subplot(grid[:, 0], projection="3d")
    ax3d.scatter(
        context[:, 1], context[:, 0], context[:, 2],
        s=0.55, c="#5f6b73", alpha=0.050, linewidths=0, depthshade=False,
    )
    ax3d.scatter(
        line_context[:, 1], line_context[:, 0], line_context[:, 2],
        s=1.2, c="#1f4e79", alpha=0.18, linewidths=0, depthshade=False,
    )
    y_plane, x_plane = np.meshgrid(np.linspace(0, 125, 8), np.linspace(-12, 12, 3))
    z_plane = np.full_like(y_plane, 3.5)
    ax3d.plot_surface(
        y_plane, x_plane, z_plane, color="#1f4e79", alpha=0.075,
        linewidth=0, shade=False,
    )
    line3d, = ax3d.plot(
        trajectory[:, 1], trajectory[:, 0], trajectory[:, 2],
        color="#d62828", linewidth=3.2, label="TopoGuide trajectory",
    )
    line3d.set_path_effects([
        patheffects.Stroke(linewidth=5.0, foreground="white", alpha=0.95),
        patheffects.Normal(),
    ])
    ax3d.scatter(trajectory[0, 1], trajectory[0, 0], trajectory[0, 2],
                 s=38, c="#111111", edgecolors="white", linewidths=0.7)
    ax3d.scatter(140.0, 0.0, 3.5, marker="*", s=80, c="#d62828",
                 edgecolors="white", linewidths=0.7)
    ax3d.set_xlabel("Mission progress y (m)", labelpad=6)
    ax3d.set_ylabel("Lateral x (m)", labelpad=6)
    ax3d.set_zlabel("Altitude z (m)", labelpad=6)
    ax3d.set_xlim(-5, 145)
    ax3d.set_ylim(-45, 45)
    ax3d.set_zlim(0, 8)
    ax3d.set_box_aspect((2.7, 1.35, 0.85))
    ax3d.view_init(elev=20, azim=-64)
    ax3d.set_title("Map4 3-D line-bypass trajectory", fontweight="semibold", pad=8)
    set_clean_3d(ax3d)

    ax_top = fig.add_subplot(grid[0, 1])
    hist_xy, y_edges, x_edges = np.histogram2d(
        obstacles[:, 1], obstacles[:, 0],
        bins=(150, 90), range=[[-5, 145], [-45, 45]],
    )
    ax_top.imshow(
        hist_xy.T, origin="lower", extent=(-5, 145, -45, 45),
        cmap="Greys", alpha=0.34, aspect="auto", vmin=0,
        vmax=max(1.0, np.percentile(hist_xy, 98.5)),
    )
    ax_top.scatter(line_context[:, 1], line_context[:, 0], s=1.0,
                   c="#1f4e79", alpha=0.30, linewidths=0)
    trajectory_line(ax_top, trajectory[:, 1], trajectory[:, 0],
                    color="#d62828", linewidth=2.1)
    ax_top.scatter(trajectory[0, 1], trajectory[0, 0], s=28, c="#111111",
                   edgecolors="white", linewidths=0.6, zorder=4)
    ax_top.scatter(140.0, 0.0, marker="*", s=60, c="#d62828",
                   edgecolors="white", linewidths=0.6, zorder=4)
    ax_top.set_xlim(-5, 145)
    ax_top.set_ylim(-45, 45)
    ax_top.set_ylabel("x (m)")
    ax_top.set_title("Top view", fontweight="semibold", pad=3)
    ax_top.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.85)

    ax_side = fig.add_subplot(grid[1, 1], sharex=ax_top)
    ax_side.axhspan(3.0, 4.2, color="#1f4e79", alpha=0.10, linewidth=0)
    ax_side.scatter(line_context[:, 1], line_context[:, 2], s=1.0,
                    c="#1f4e79", alpha=0.12, linewidths=0)
    trajectory_line(ax_side, trajectory[:, 1], trajectory[:, 2],
                    color="#d62828", linewidth=2.1)
    ax_side.axhline(3.5, color="#777777", linestyle=":", linewidth=0.9)
    ax_side.set_xlim(-5, 145)
    ax_side.set_ylim(0, 8)
    ax_side.set_xlabel("Mission progress y (m)")
    ax_side.set_ylabel("z (m)")
    ax_side.set_title("Altitude profile", fontweight="semibold", pad=3)
    ax_side.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.85)

    z_span = float(np.max(trajectory[:, 2]) - np.min(trajectory[:, 2]))
    fig.text(
        0.035, 0.025,
        "Shortest successful 3-D trial: "
        f"{float(row['path_m']):.1f} m, {float(row['duration_s']):.1f} s, "
        f"z span {z_span:.2f} m; query: line.",
        fontsize=7.2, color="#333333",
    )

    for suffix in (".png", ".pdf"):
        fig.savefig(OUT.with_suffix(suffix), dpi=300, bbox_inches="tight")
    print(f"wrote {OUT}.png and {OUT}.pdf")


if __name__ == "__main__":
    main()
