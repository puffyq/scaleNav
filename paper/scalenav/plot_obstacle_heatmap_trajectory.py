#!/usr/bin/env python3
"""Plot a batch obstacle-density heatmap with its shortest successful flight."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patheffects


REPO_ROOT = Path(__file__).resolve().parents[2]
ANALYSIS_PATH = (
    REPO_ROOT / "scalenav_ws/docs/test_reports/analyze_round_trip_density.py"
)


def load_analysis_module():
    spec = importlib.util.spec_from_file_location("obstacle_density", ANALYSIS_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load obstacle analysis from {ANALYSIS_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_summary(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def read_record(line: str) -> dict:
    line = (line.replace(":-inf", ":-Infinity")
            .replace(":inf", ":Infinity")
            .replace(":nan", ":NaN"))
    return json.loads(line, parse_constant=lambda value: float(value))


def mission_interval(index_path: Path) -> tuple[int, int]:
    mission_start = None
    mission_end = None
    first_odom = None
    last_odom = None
    with index_path.open(encoding="utf-8") as stream:
        for line in stream:
            record = read_record(line)
            stamp = int(record.get("stamp_ns", 0))
            if record.get("kind") == "mission":
                event = (record.get("data") or {}).get("event")
                if event == "start" and mission_start is None:
                    mission_start = stamp
                elif event in {"complete", "stop", "collision", "timeout"}:
                    mission_end = stamp
            elif record.get("kind") == "odom":
                first_odom = stamp if first_odom is None else first_odom
                last_odom = stamp
    start = mission_start if mission_start is not None else first_odom
    end = mission_end if mission_end is not None else last_odom
    if start is None or end is None or end <= start:
        raise ValueError(f"no complete mission interval in {index_path}")
    return start, end


def load_trajectory(index_path: Path, start_ns: int, end_ns: int):
    positions = []
    speeds = []
    with index_path.open(encoding="utf-8") as stream:
        for line in stream:
            record = read_record(line)
            if record.get("kind") != "odom":
                continue
            stamp = int(record.get("stamp_ns", 0))
            if not start_ns <= stamp <= end_ns:
                continue
            data = record.get("data") or {}
            positions.append(data["position"])
            speeds.append(float(np.linalg.norm(data["velocity"])))
    if len(positions) < 2:
        raise ValueError(f"mission interval has no trajectory in {index_path}")
    return np.asarray(positions, dtype=float), np.asarray(speeds, dtype=float)


def density_image(grid, bounds, output_cell: float):
    x_min, x_max, y_min, y_max = bounds
    x_values = np.arange(x_min, x_max, output_cell)
    y_values = np.arange(y_min, y_max, output_cell)
    image = np.zeros((len(x_values), len(y_values)), dtype=float)
    for x_index, x in enumerate(x_values):
        for y_index, y in enumerate(y_values):
            image[x_index, y_index] = grid[(round(float(x), 6), round(float(y), 6))]
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-trials", type=int, default=10)
    parser.add_argument("--cell-size", type=float, default=0.5)
    parser.add_argument("--output-cell", type=float, default=1.0)
    parser.add_argument("--density-radius", type=float, default=3.0)
    parser.add_argument("--sample-period", type=float, default=0.5)
    parser.add_argument("--min-z", type=float, default=0.3)
    parser.add_argument("--max-z", type=float, default=3.2)
    parser.add_argument("--x-min", type=float, default=-50.0)
    parser.add_argument("--x-max", type=float, default=50.0)
    parser.add_argument("--y-min", type=float, default=-5.0)
    parser.add_argument("--y-max", type=float, default=145.0)
    args = parser.parse_args()

    rows = load_summary(args.summary)
    if len(rows) != args.expected_trials:
        raise RuntimeError(
            f"expected {args.expected_trials} completed trials, found {len(rows)}"
        )
    successes = [row for row in rows if row.get("outcome") == "success"]
    if not successes:
        raise RuntimeError("batch contains no successful trajectory")
    best = min(successes, key=lambda row: float(row["path_m"]))

    analysis = load_analysis_module()
    occupied = set()
    sampled_frames = 0
    accepted_points = 0
    intervals = {}
    for row in rows:
        session = Path(row["session_dir"])
        start_ns, end_ns = mission_interval(session / "index.jsonl")
        intervals[row["trial"]] = (start_ns, end_ns)
        odom, pointclouds, _ = analysis.load_index(session)
        cells, frames, points = analysis.load_occupied_cells(
            session,
            odom,
            pointclouds,
            start_ns,
            end_ns,
            cell_size=args.cell_size,
            sample_period_ns=round(args.sample_period * 1e9),
            min_z=args.min_z,
            max_z=args.max_z,
        )
        occupied.update(cells)
        sampled_frames += frames
        accepted_points += points

    bounds = (args.x_min, args.x_max, args.y_min, args.y_max)
    grid = analysis.density_grid(
        occupied,
        args.cell_size,
        bounds,
        radius=args.density_radius,
        output_cell=args.output_cell,
    )
    image = density_image(grid, bounds, args.output_cell)
    positive = image[image > 0.0]
    color_max = float(np.percentile(positive, 98.0)) if positive.size else 1.0

    best_session = Path(best["session_dir"])
    positions, _ = load_trajectory(
        best_session / "index.jsonl", *intervals[best["trial"]]
    )

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 8.0,
        "axes.titlesize": 10.0,
        "axes.labelsize": 8.5,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
    })
    figure, axis = plt.subplots(figsize=(7.2, 4.8))
    heatmap = axis.imshow(
        image,
        origin="lower",
        extent=(args.y_min, args.y_max, args.x_min, args.x_max),
        cmap="YlOrRd",
        vmin=0.0,
        vmax=color_max,
        interpolation="nearest",
        aspect="equal",
        zorder=0,
    )
    route, = axis.plot(
        positions[:, 1],
        positions[:, 0],
        color="#007f78",
        linewidth=2.6,
        zorder=4,
    )
    route.set_path_effects([
        patheffects.Stroke(linewidth=4.2, foreground="white", alpha=0.9),
        patheffects.Normal(),
    ])
    axis.scatter(
        positions[0, 1], positions[0, 0], marker="o", s=48,
        color="#202124", edgecolor="white", linewidth=0.8, zorder=5,
        label="Start",
    )
    axis.scatter(
        positions[-1, 1], positions[-1, 0], marker="*", s=110,
        color="#c62828", edgecolor="white", linewidth=0.8, zorder=5,
        label="Goal",
    )
    axis.set_xlim(args.y_min, args.y_max)
    axis.set_ylim(args.x_min, args.x_max)
    axis.set_xlabel("Longitudinal $y$ (m)")
    axis.set_ylabel("Lateral $x$ (m)")
    axis.set_title("Map4 obstacle density and shortest successful trajectory", pad=7)
    axis.grid(color="white", alpha=0.45, linewidth=0.55)
    for spine in axis.spines.values():
        spine.set_linewidth(0.7)
        spine.set_color("#30363b")

    info = (
        f"Trial {best['trial']}  |  {float(best['path_m']):.2f} m  |  "
        f"{float(best['duration_s']):.2f} s  |  "
        f"{float(best['average_speed_mps']):.2f} m/s"
    )
    axis.text(
        0.015, 0.975, info, transform=axis.transAxes, ha="left", va="top",
        color="#202124", fontsize=7.8,
        bbox={"boxstyle": "square,pad=0.25", "facecolor": "white",
              "edgecolor": "none", "alpha": 0.94},
        zorder=6,
    )
    axis.legend(
        [route, axis.collections[-2], axis.collections[-1]],
        ["Trajectory", "Start", "Goal"],
        loc="lower right", frameon=True, framealpha=0.94,
    )
    colorbar = figure.colorbar(heatmap, ax=axis, fraction=0.035, pad=0.025)
    colorbar.set_label(
        f"Occupied-cell fraction within {args.density_radius:g} m"
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=300, bbox_inches="tight", pad_inches=0.04)
    figure.savefig(
        args.output.with_suffix(".pdf"), bbox_inches="tight", pad_inches=0.04
    )
    metadata = {
        "batch_summary": str(args.summary.resolve()),
        "trial_count": len(rows),
        "success_count": len(successes),
        "selected_trial": int(best["trial"]),
        "selected_session": str(best_session),
        "selection_rule": "minimum path_m among successful trials",
        "selected_metrics": {
            key: float(best[key])
            for key in ("path_m", "duration_s", "average_speed_mps", "max_speed_mps")
        },
        "obstacle_map": {
            "sessions": len(rows),
            "sampled_pointcloud_frames": sampled_frames,
            "accepted_flight_height_points": accepted_points,
            "occupied_cells": len(occupied),
            "cell_size_m": args.cell_size,
            "density_radius_m": args.density_radius,
            "height_slab_m": [args.min_z, args.max_z],
            "bounds_xy_m": list(bounds),
        },
    }
    metadata_path = args.output.with_name(args.output.stem + "_source.json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
