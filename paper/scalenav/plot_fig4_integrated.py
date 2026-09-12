#!/usr/bin/env python3
"""Build the integrated trajectory and mean-speed comparison, including FAR."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
PLOT_IMPL = HERE / "plot_speed_trajectories.py"
TRUTH_MAP = HERE / "pics/map2_ground_truth_airsim_20260904.ply"
OUT = HERE / "pics/experiments/map2_0_140_1p6/figure4_integrated_topoguide"
FAR_SESSION = REPO_ROOT / "log_scalenav/session_20260910_212024_960/index.jsonl"


RUNS = {
    "YOPO": (
        REPO_ROOT / "log_scalenav_yopo_simple/session_20260903_085047_534/index.jsonl",
        REPO_ROOT / "log_scalenav/session_20260905_162438_747/index.jsonl",
        "#2f6db0",
    ),
    "EGO": (
        REPO_ROOT / "log_ego/session_20260903_120531_738/index.jsonl",
        REPO_ROOT / "log_scalenav/session_20260905_185019_616/index.jsonl",
        "#e07a22",
    ),
    "SUPER": (
        REPO_ROOT / "log_scalenav/session_20260904_203305_270/index.jsonl",
        REPO_ROOT / "log_scalenav/session_20260906_110011_374/index.jsonl",
        "#2a9d72",
    ),
}

OUTCOMES = {
    "YOPO": {"success": 100, "path": 171.59, "time": 33.61},
    "EGO": {"success": 70, "path": 181.59, "time": 114.05},
    "SUPER": {"success": 90, "path": 179.76, "time": 39.32},
}


def load_impl():
    spec = importlib.util.spec_from_file_location("trajectory_plot_impl", PLOT_IMPL)
    if spec is None or spec.loader is None:
        raise ImportError(PLOT_IMPL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def plot_overlay(ax, impl, label, baseline_path, ours_path, baseline_color):
    goal = np.array([0.0, 140.0, 1.6])
    baseline = impl.load_flight(
        baseline_path, goal, 0.5, 0.3, require_goal=False, return_timestamps=True,
    )
    ours = impl.load_flight(
        ours_path, goal, 0.5, 0.3, require_goal=True, return_timestamps=True,
    )
    image, bounds, _, _ = impl.load_truth_voxel_map(TRUTH_MAP, 0.15)
    impl.draw_voxels(ax, impl.fill_voxel_footprints(image), bounds)

    def line(positions, color, width, zorder):
        points = np.column_stack((positions[:, 1], positions[:, 0]))
        ax.plot(points[:, 0], points[:, 1], color=color, linewidth=width, zorder=zorder)
        ax.scatter(points[0, 0], points[0, 1], s=18, c="#202124", zorder=zorder + 1)
        return points

    base_points = line(baseline[0], baseline_color, 1.7, 3)
    ours_points = line(ours[0], "#d62828", 2.3, 4)
    if baseline[2]:
        ax.scatter(base_points[-1, 0], base_points[-1, 1], marker="*", s=45,
                   c=baseline_color, edgecolors="white", linewidths=0.5, zorder=5)
    else:
        ax.scatter(base_points[-1, 0], base_points[-1, 1], marker="X", s=38,
                   c=baseline_color, edgecolors="white", linewidths=0.5, zorder=5)
    ax.scatter(ours_points[-1, 0], ours_points[-1, 1], marker="*", s=55,
               c="#d62828", edgecolors="white", linewidths=0.5, zorder=5)
    success = OUTCOMES[label]["success"]
    ax.set_title(f"{label}\nsuccess 0% -> {success}%", fontsize=8.2,
                 fontweight="semibold", pad=3)
    ax.set_xlabel("Mission progress $y$ (m)")
    ax.set_ylabel("Lateral $x$ (m)")
    ax.set_xlim(-8, 150)
    ax.set_ylim(-35, 35)
    ax.set_aspect(1.25, adjustable="box")
    ax.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.8)
    return baseline, ours


def plot_far(ax, impl, reference_flight):
    goal = np.array([0.0, 140.0, 1.6])
    flight = impl.load_flight(
        FAR_SESSION, goal, 0.5, 0.3, require_goal=True, return_timestamps=True,
    )
    image, bounds, _, _ = impl.load_truth_voxel_map(TRUTH_MAP, 0.15)
    impl.draw_voxels(ax, impl.fill_voxel_footprints(image), bounds)
    points = np.column_stack((flight[0][:, 1], flight[0][:, 0]))
    ax.plot(points[:, 0], points[:, 1], color="#7a5195", linewidth=1.9, zorder=3)
    ax.scatter(points[0, 0], points[0, 1], s=18, c="#202124", zorder=4)
    ax.scatter(points[-1, 0], points[-1, 1], marker="*", s=52,
               c="#7a5195", edgecolors="white", linewidths=0.5, zorder=5)
    reference_points = np.column_stack((reference_flight[0][:, 1], reference_flight[0][:, 0]))
    ax.plot(reference_points[:, 0], reference_points[:, 1], color="#d62828",
            linewidth=2.0, alpha=0.48, zorder=4)
    ax.scatter(reference_points[-1, 0], reference_points[-1, 1], marker="*", s=52,
               c="#d62828", alpha=0.48, edgecolors="white", linewidths=0.5, zorder=6)
    ax.set_title("FAR Planner\nsuccess 100%", fontsize=8.2,
                 fontweight="semibold", pad=3)
    ax.set_xlabel("Mission progress $y$ (m)")
    ax.set_ylabel("")
    ax.tick_params(axis="y", labelleft=False)
    ax.set_xlim(-8, 150)
    ax.set_ylim(-35, 35)
    ax.set_aspect(1.25, adjustable="box")
    ax.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.8)
    return flight


def binned_mean_speed(flight, bin_seconds=2.0):
    """Average speeds in elapsed-time bins, plotted at each interval's end."""
    speeds = np.asarray(flight[1], dtype=float)
    timestamps = np.asarray(flight[4], dtype=np.int64)
    if len(timestamps) < 2 or speeds.shape != timestamps.shape:
        raise ValueError("Expected matching speed and timestamp arrays with at least two samples")
    elapsed = (timestamps - timestamps[0]) / 1e9
    if np.any(np.diff(elapsed) <= 0) or not np.isfinite(speeds).all() or np.any(speeds < 0):
        raise ValueError("Expected increasing timestamps and finite nonnegative speeds")
    if not math.isfinite(bin_seconds) or bin_seconds <= 0:
        raise ValueError("Expected a positive finite bin width")
    edges = np.append(np.arange(0.0, elapsed[-1], bin_seconds), elapsed[-1])
    bins = len(edges) - 1
    indices = np.minimum(bins - 1, np.searchsorted(edges, elapsed, side="right") - 1)
    counts = np.bincount(indices, minlength=bins)
    totals = np.bincount(indices, weights=speeds, minlength=bins)
    means = np.full(bins, np.nan)
    np.divide(totals, counts, out=means, where=counts > 0)
    return edges[1:], means, float(elapsed[-1])


def speed_panel(axis, label, curves):
    for curve in curves:
        name, flight, color = curve[:3]
        alpha = curve[3] if len(curve) > 3 else 1.0
        times, means, duration = binned_mean_speed(flight)
        outcome = "complete" if flight[2] else "failed"
        axis.plot(times, means, color=color, linewidth=1.6,
                  marker="*" if flight[2] else "X", markevery=[-1],
                  markersize=6, markeredgecolor="white", markeredgewidth=0.5,
                  alpha=alpha, label=f"{name} ({duration:.1f} s, {outcome})")
    axis.set_title(f"{label}: mean speed", fontsize=8.5,
                   fontweight="semibold", pad=3)
    axis.legend(fontsize=5.8, ncol=1, frameon=False, loc="upper left",
                handlelength=2.0)


def main():
    impl = load_impl()
    plt.rcParams.update({"font.size": 7.2, "font.family": "DejaVu Sans"})
    fig = plt.figure(figsize=(13.8, 5.25))
    left, right, gap = 0.045, 0.985, 0.055
    width = (right - left - 3.0 * gap) / 4.0
    top_y, top_h = 0.59, 0.34
    bottom_y, bottom_h = 0.225, 0.28

    top_axes = [
        fig.add_axes([left + i * (width + gap), top_y, width, top_h])
        for i in range(4)
    ]
    comparisons = []
    for ax, (label, (base, ours, color)) in zip(top_axes, RUNS.items()):
        baseline_flight, guided_flight = plot_overlay(ax, impl, label, base, ours, color)
        comparisons.append((label, [
            ("Standalone", baseline_flight, color),
            ("TopoGuide", guided_flight, "#d62828"),
        ]))
    best_yopo_gcn = comparisons[0][1][1][1]
    far_flight = plot_far(top_axes[-1], impl, best_yopo_gcn)
    comparisons.append(("FAR Planner", [
        ("FAR", far_flight, "#7a5195"),
        ("YOPO+GCN", best_yopo_gcn, "#d62828", 0.48),
    ]))

    speed_axes = []
    for index in range(4):
        axis = fig.add_axes(
            [left + index * (width + gap), bottom_y, width, bottom_h],
            sharey=speed_axes[0] if speed_axes else None,
        )
        speed_axes.append(axis)
    for axis, (label, curves) in zip(speed_axes, comparisons):
        speed_panel(axis, label, curves)
    peak_speed = max(
        float(np.nanmax(line.get_ydata()))
        for axis in speed_axes for line in axis.lines
        if np.isfinite(line.get_ydata()).any()
    )
    upper_speed = max(1.0, float(math.ceil(1.25 * peak_speed)))
    for index, axis in enumerate(speed_axes):
        max_duration = max(float(line.get_xdata()[-1]) for line in axis.lines)
        upper_time = 10.0 * math.ceil(max_duration / 10.0)
        tick_step = 10 if upper_time <= 70 else 20
        axis.set_xlim(0, upper_time)
        axis.set_xticks(np.arange(0, upper_time + 1, tick_step))
        axis.set_ylim(0, upper_speed)
        axis.set_yticks(np.arange(0, upper_speed + 1, 1))
        axis.set_xlabel("Elapsed flight time (s)")
        axis.set_ylabel("Mean speed (m/s)" if index == 0 else "")
        axis.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.8)
    for suffix in (".png", ".pdf"):
        fig.savefig(OUT.with_suffix(suffix), dpi=300, bbox_inches="tight")
    print(f"wrote {OUT}.png and {OUT}.pdf")


if __name__ == "__main__":
    main()
