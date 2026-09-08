#!/usr/bin/env python3
"""Build the integrated Fig. 4: route overlays, outcomes, and resources."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
PLOT_IMPL = HERE / "plot_speed_trajectories.py"
TRUTH_MAP = HERE / "pics/map2_ground_truth_airsim_20260904.ply"
OUT = HERE / "pics/experiments/map2_0_140_1p6/figure4_integrated_topoguide"


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
    baseline = impl.load_flight(baseline_path, goal, 0.5, 0.3, require_goal=False)
    ours = impl.load_flight(ours_path, goal, 0.5, 0.3, require_goal=True)
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


def load_resource_samples(path):
    records = []
    mission_start = None
    mission_end = None
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            line = (line.replace(":-inf", ":-Infinity")
                    .replace(":inf", ":Infinity")
                    .replace(":nan", ":NaN"))
            event = json.loads(line, parse_constant=lambda value: float(value))
            stamp = int(event.get("stamp_ns", 0))
            if event.get("kind") == "mission":
                mission_event = (event.get("data") or {}).get("event")
                if mission_event == "start" and mission_start is None:
                    mission_start = stamp
                elif mission_event == "complete":
                    mission_end = stamp
            elif event.get("kind") == "timing":
                records.append((stamp, event.get("data") or {}))
    if mission_start is None:
        mission_start = min(stamp for stamp, _ in records)
    if mission_end is None:
        mission_end = max(stamp for stamp, _ in records)
    duration = max(1, mission_end - mission_start)
    return [
        (100.0 * (stamp - mission_start) / duration, data)
        for stamp, data in records
        if mission_start <= stamp <= mission_end
    ]


def binned_median(records, module, key, positive=False, bins=20):
    edges = np.linspace(0.0, 100.0, bins + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    buckets = [[] for _ in range(bins)]
    for time_pct, data in records:
        if data.get("module") != module:
            continue
        value = data.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            continue
        if positive and value <= 0:
            continue
        index = min(bins - 1, max(0, int(time_pct / 100.0 * bins)))
        buckets[index].append(float(value))
    medians = np.array([
        np.median(values) if values else np.nan for values in buckets
    ])
    return centers, medians


def resource_panel(ax, label, path, show_left_label=False, show_right_label=False):
    records = load_resource_samples(path)
    timing_specs = (
        ("Point", "cloud", "total_ms", "#2a9d72", "-."),
        ("Graph", "background", "total_ms", "#6b4c9a", "--"),
        ("Plan tick", "planner", "total_ms", "#202124", "-"),
        ("A* time", "planner", "astar_ms", "#e07a22", ":"),
    )
    handles = []
    for name, module, key, color, style in timing_specs:
        x, y = binned_median(records, module, key)
        line, = ax.plot(x, y, label=name, color=color, linestyle=style,
                        linewidth=1.5, alpha=0.95)
        handles.append(line)
    ax.set_title(f"{label}: route-layer workload", fontsize=8.5,
                 fontweight="semibold", pad=3)
    ax.set_xlim(0, 100)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("Normalized mission time (%)")
    ax.set_ylabel("Wall time (ms)" if show_left_label else "")
    if not show_left_label:
        ax.tick_params(axis="y", labelleft=False)
    ax.grid(True, color="#d9dde1", linewidth=0.45, alpha=0.8)
    ax.legend(handles=handles, labels=[line.get_label() for line in handles],
              fontsize=5.5, ncol=3, frameon=False, loc="upper left",
              handlelength=2.0, columnspacing=0.65)


def main():
    impl = load_impl()
    plt.rcParams.update({"font.size": 7.2, "font.family": "DejaVu Sans"})
    fig = plt.figure(figsize=(10.7, 5.25))
    left, right, gap = 0.045, 0.985, 0.055
    width = (right - left - 2.0 * gap) / 3.0
    top_y, top_h = 0.59, 0.34
    bottom_y, bottom_h = 0.225, 0.28

    top_axes = [
        fig.add_axes([left + i * (width + gap), top_y, width, top_h])
        for i in range(3)
    ]
    for ax, (label, (base, ours, color)) in zip(top_axes, RUNS.items()):
        plot_overlay(ax, impl, label, base, ours, color)

    for index, (label, (_, ours, _)) in enumerate(RUNS.items()):
        resource_panel(
            fig.add_axes([
                left + index * (width + gap), bottom_y, width, bottom_h
            ]),
            label, ours,
            show_left_label=index == 0,
        )
    for suffix in (".png", ".pdf"):
        fig.savefig(OUT.with_suffix(suffix), dpi=300, bbox_inches="tight")
    print(f"wrote {OUT}.png and {OUT}.pdf")


if __name__ == "__main__":
    main()
