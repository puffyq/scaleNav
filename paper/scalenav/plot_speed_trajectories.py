#!/usr/bin/env python3
"""Plot speed-colored Map2 trajectories from ScaleNav JSONL event logs."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize


REPO_ROOT = Path(__file__).resolve().parents[2]
DENSITY_ANALYSIS = (
    REPO_ROOT / "scalenav_ws/docs/test_reports/analyze_round_trip_density.py"
)


def load_density(session: Path, start_ns: int | None, end_ns: int | None):
    """Rebuild the report's world-aligned flight-height obstacle density."""
    spec = importlib.util.spec_from_file_location(
        "round_trip_density_analysis", DENSITY_ANALYSIS
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load density analysis from {DENSITY_ANALYSIS}")
    analysis = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analysis)

    odom, pointclouds, _ = analysis.load_index(session)
    if not odom or not pointclouds:
        raise ValueError(f"density session has no odometry or point clouds: {session}")
    if start_ns is None:
        start_ns = min(item[0] for item in pointclouds)
    if end_ns is None:
        end_ns = max(item[0] for item in pointclouds)
    occupied, frame_count, _ = analysis.load_occupied_cells(
        session, odom, pointclouds, start_ns, end_ns
    )
    bounds = (-30.0, 30.0, -5.0, 145.0)
    density = analysis.density_grid(occupied, 0.5, bounds)

    world_x = np.arange(bounds[0], bounds[1], 1.0)
    world_y = np.arange(bounds[2], bounds[3], 1.0)
    # Rows are world x and columns are world y because trajectories are shown
    # as longitudinal y (horizontal) versus lateral x (vertical).
    image = np.asarray([
        [density[(round(x, 6), round(y, 6))] for y in world_y]
        for x in world_x
    ])
    density_max = float(np.percentile(image, 98)) or 1.0
    return image, bounds, density_max, frame_count, len(occupied)


def density_colormap():
    colors = ["#f8faf7", "#efe49a", "#e29b5e", "#be4d3e", "#5b2331"]
    cmap = LinearSegmentedColormap.from_list("obstacle_density", colors)
    cmap.set_bad((1.0, 1.0, 1.0, 0.0))
    return cmap


def load_flight(path: Path, goal: np.ndarray, goal_radius: float, stop_speed: float):
    samples = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            event = json.loads(line)
            if event.get("event") != "odom":
                continue
            position = np.asarray(event["position_world"], dtype=float)
            velocity = np.asarray(event["velocity_world"], dtype=float)
            samples.append((position, float(np.linalg.norm(velocity))))

    if not samples:
        raise ValueError(f"no odometry samples in {path}")

    moving = next((i for i, (_, speed) in enumerate(samples) if speed > 0.1), None)
    if moving is None:
        raise ValueError(f"no flight motion in {path}")
    start = max(0, moving - 1)

    end = None
    for i in range(moving, len(samples)):
        position, speed = samples[i]
        if np.linalg.norm(position - goal) <= goal_radius and speed <= stop_speed:
            end = i + 1
            break
    if end is None:
        raise ValueError(f"goal was not reached in {path}")

    positions = np.asarray([sample[0] for sample in samples[start:end]])
    speeds = np.asarray([sample[1] for sample in samples[start:end]])
    return positions, speeds


def draw_density(ax, image, bounds, cmap, norm):
    x_min, x_max, y_min, y_max = bounds
    masked = np.ma.masked_less_equal(image, 0.0)
    return ax.imshow(
        masked,
        origin="lower",
        extent=(y_min, y_max, x_min, x_max),
        cmap=cmap,
        norm=norm,
        interpolation="nearest",
        aspect="auto",
        alpha=0.1,
        zorder=0,
    )


def colored_trajectory(ax, positions, speeds, label, norm):
    # Map2 is long in world y. Plot longitudinal progress horizontally so the
    # two-panel figure remains legible at a single-column paper width.
    points = np.column_stack((positions[:, 1], positions[:, 0]))
    segments = np.stack((points[:-1], points[1:]), axis=1)
    colors = 0.5 * (speeds[:-1] + speeds[1:])
    collection = LineCollection(
        segments, cmap="turbo", norm=norm, linewidth=2.2, zorder=3
    )
    collection.set_array(colors)
    ax.add_collection(collection)
    ax.scatter(points[0, 0], points[0, 1], marker="o", s=34, c="#202124", zorder=4)
    ax.scatter(
        points[-1, 0], points[-1, 1], marker="*", s=80,
        c="#d62728", edgecolors="white", linewidths=0.7, zorder=4,
    )
    ax.set_title(label, loc="left", fontweight="semibold")
    ax.set_ylabel("Lateral $x$ (m)")
    ax.grid(True, color="#d9dde1", linewidth=0.6, alpha=0.8)
    ax.margins(x=0.02, y=0.18)
    return collection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", action="append", required=True,
                        metavar="LABEL=JSONL")
    parser.add_argument("--scalenav", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--goal", nargs=3, type=float, default=(0.0, 140.0, 1.6))
    parser.add_argument("--goal-radius", type=float, default=0.5)
    parser.add_argument("--stop-speed", type=float, default=0.3)
    parser.add_argument("--speed-max", type=float, default=6.0)
    parser.add_argument(
        "--density-session", type=Path,
        help="round-trip session used to reconstruct the obstacle-density backdrop",
    )
    parser.add_argument("--density-start-ns", type=int)
    parser.add_argument("--density-end-ns", type=int)
    args = parser.parse_args()

    goal = np.asarray(args.goal, dtype=float)
    baselines = []
    for item in args.baseline:
        if "=" not in item:
            parser.error("--baseline must be LABEL=JSONL")
        label, raw_path = item.split("=", 1)
        baselines.append((label, *load_flight(
            Path(raw_path), goal, args.goal_radius, args.stop_speed
        )))
    ours = load_flight(args.scalenav, goal, args.goal_radius, args.stop_speed)

    plt.rcParams.update({
        "font.size": 8.5,
        "axes.titlesize": 9.5,
        "axes.labelsize": 8.5,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "font.family": "DejaVu Sans",
    })
    row_count = len(baselines) + 1
    figure, axes = plt.subplots(
        row_count, 1, figsize=(7.0, 1.75 * row_count + 1.35), sharex=True,
        squeeze=False,
    )
    axes = axes[:, 0]
    norm = Normalize(0.0, args.speed_max)

    density_artist = None
    if args.density_session:
        image, bounds, density_max, frame_count, occupied_cells = load_density(
            args.density_session, args.density_start_ns, args.density_end_ns
        )
        print(
            f"Density backdrop: {frame_count} sampled frames, "
            f"{occupied_cells} occupied cells"
        )
        density_norm = Normalize(0.0, density_max)
        cmap = density_colormap()
        for axis in axes:
            density_artist = draw_density(axis, image, bounds, cmap, density_norm)

    collection = None
    for axis, (label, positions, speeds) in zip(axes, baselines):
        collection = colored_trajectory(axis, positions, speeds, label, norm)
    collection = colored_trajectory(
        axes[-1], ours[0], ours[1], "ScaleNav (ours)", norm
    )
    axes[-1].set_xlabel("Longitudinal $y$ (m)")
    for ax in axes:
        ax.set_xlim(-2, 142)

    figure.suptitle(
        "Map2 trajectories: start (0, 0, 1.6 m), goal (0, 140, 1.6 m)",
        x=0.11, y=0.975, ha="left", fontsize=9,
    )
    figure.subplots_adjust(left=0.11, right=0.98, top=0.91, bottom=0.19,
                           hspace=0.48)
    speed_colorbar_axis = figure.add_axes((0.11, 0.060, 0.36, 0.020))
    colorbar = figure.colorbar(collection, cax=speed_colorbar_axis,
                               orientation="horizontal")
    colorbar.set_label("Speed (m/s)", labelpad=2)
    if density_artist is not None:
        density_colorbar_axis = figure.add_axes((0.60, 0.060, 0.36, 0.020))
        density_colorbar = figure.colorbar(
            density_artist, cax=density_colorbar_axis, orientation="horizontal"
        )
        density_colorbar.set_label("Local obstacle density", labelpad=2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=300, bbox_inches="tight")
    figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight")


if __name__ == "__main__":
    main()
