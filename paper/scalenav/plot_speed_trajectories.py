#!/usr/bin/env python3
"""Plot speed-colored Map2 trajectories from ScaleNav JSONL event logs."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import shutil
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, Normalize


REPO_ROOT = Path(__file__).resolve().parents[2]
POINTCLOUD_ANALYSIS = (
    REPO_ROOT / "scalenav_ws/docs/test_reports/analyze_round_trip_density.py"
)
DEFAULT_TRUTH_MAP = Path(__file__).resolve().parent / "pics/map2_ground_truth.ply"


def load_ply_xyz(path: Path) -> np.ndarray:
    """Read an ASCII PLY vertex cloud exported in world_enu metres."""
    with path.open(encoding="ascii") as stream:
        vertex_count = None
        for line in stream:
            if line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            if line.strip() == "end_header":
                break
        if vertex_count is None:
            raise ValueError(f"PLY has no vertex count: {path}")
        points = np.loadtxt(stream, max_rows=vertex_count)
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"PLY contains no XYZ vertices: {path}")
    points = points[:, :3]
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError(f"PLY contains no finite vertices: {path}")
    return points


def load_truth_voxel_map(
    path: Path,
    voxel_size: float,
    z_min: float = 0.3,
    z_max: float = 3.2,
):
    """Build a complete Map2 top view from the static UE truth cloud."""
    points = load_ply_xyz(path)
    points = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
    if not len(points):
        raise ValueError(f"truth map has no points in z=[{z_min}, {z_max}] m")
    cells = np.floor(points[:, :2] / voxel_size).astype(np.int64)
    cells = np.unique(cells, axis=0)
    x_cell_min, y_cell_min = cells.min(axis=0)
    x_cell_max, y_cell_max = cells.max(axis=0)
    image = np.zeros(
        (x_cell_max - x_cell_min + 1, y_cell_max - y_cell_min + 1),
        dtype=np.uint8,
    )
    image[cells[:, 0] - x_cell_min, cells[:, 1] - y_cell_min] = 1
    bounds = (
        float(x_cell_min * voxel_size),
        float((x_cell_max + 1) * voxel_size),
        float(y_cell_min * voxel_size),
        float((y_cell_max + 1) * voxel_size),
    )
    return image, bounds, len(points), len(cells)


def load_voxel_map(
    session: Path,
    start_ns: int | None,
    end_ns: int | None,
    voxel_size: float,
):
    """Build a binary top view of world-aligned, flight-height point voxels."""
    spec = importlib.util.spec_from_file_location(
        "round_trip_pointcloud_analysis", POINTCLOUD_ANALYSIS
    )
    if spec is None or spec.loader is None:
        raise ImportError(
            f"cannot load point-cloud analysis from {POINTCLOUD_ANALYSIS}"
        )
    analysis = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analysis)

    odom, pointclouds, _ = analysis.load_index(session)
    if not odom or not pointclouds:
        raise ValueError(
            f"point-cloud session has no odometry or point clouds: {session}"
        )
    if start_ns is None:
        start_ns = min(item[0] for item in pointclouds)
    if end_ns is None:
        end_ns = max(item[0] for item in pointclouds)
    occupied, frame_count, accepted_points = analysis.load_occupied_cells(
        session, odom, pointclouds, start_ns, end_ns, cell_size=voxel_size
    )
    bounds = (-30.0, 30.0, -5.0, 145.0)
    x_min, x_max, y_min, y_max = bounds
    x_cell_min = int(np.floor(x_min / voxel_size))
    y_cell_min = int(np.floor(y_min / voxel_size))
    x_cells = int(np.ceil((x_max - x_min) / voxel_size))
    y_cells = int(np.ceil((y_max - y_min) / voxel_size))
    image = np.zeros((x_cells, y_cells), dtype=np.uint8)
    for x_cell, y_cell in occupied:
        row = x_cell - x_cell_min
        column = y_cell - y_cell_min
        if 0 <= row < x_cells and 0 <= column < y_cells:
            image[row, column] = 1
    return image, bounds, frame_count, len(occupied), accepted_points


def load_flight(
    path: Path,
    goal: np.ndarray,
    goal_radius: float,
    stop_speed: float,
    require_goal: bool = True,
):
    samples = []
    mission_start_ns = None
    mission_end_ns = None
    collision_stamps_ns = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            # Planner timing records may contain non-standard JSON ``inf``
            # values for unavailable objectives; they are irrelevant to the
            # odometry/collision fields used by this plot.
            line = (line.replace(":-inf", ":-Infinity")
                    .replace(":inf", ":Infinity")
                    .replace(":nan", ":NaN"))
            event = json.loads(line, parse_constant=lambda value: float(value))
            if event.get("kind") == "mission":
                mission_event = (event.get("data") or {}).get("event")
                if mission_event == "start" and mission_start_ns is None:
                    mission_start_ns = int(event["stamp_ns"])
                elif mission_event == "complete" and mission_start_ns is not None:
                    mission_end_ns = int(event["stamp_ns"])
                continue
            if event.get("kind") == "collision":
                data = event.get("data") or {}
                # Count the rising/active event only.  A latched collision may
                # remain true after contact has cleared, and pre-mission
                # takeoff/landing contacts are outside the evaluated flight.
                if data.get("active", data.get("latched", False)):
                    collision_stamps_ns.append(int(event["stamp_ns"]))
                continue
            if event.get("kind") == "odom":
                data = event.get("data") or {}
                stamp_ns = int(event.get("stamp_ns", 0))
                position = np.asarray(data["position"], dtype=float)
                velocity = np.asarray(data["velocity"], dtype=float)
            elif event.get("event") == "odom":
                stamp_ns = int(float(event.get("stamp", 0.0)) * 1e9)
                position = np.asarray(event["position_world"], dtype=float)
                velocity = np.asarray(event["velocity_world"], dtype=float)
            else:
                continue
            samples.append((stamp_ns, position, float(np.linalg.norm(velocity))))

    if not samples:
        raise ValueError(f"no odometry samples in {path}")

    collision_ns = None
    if mission_start_ns is not None:
        collision_ns = next(
            (
                stamp_ns
                for stamp_ns in collision_stamps_ns
                if stamp_ns >= mission_start_ns
                and (mission_end_ns is None or stamp_ns <= mission_end_ns)
            ),
            None,
        )
        samples = [
            sample for sample in samples
            if sample[0] >= mission_start_ns
            and (mission_end_ns is None or sample[0] <= mission_end_ns)
        ]
        if not samples:
            raise ValueError(f"mission interval has no odometry samples in {path}")
    elif collision_stamps_ns:
        collision_ns = collision_stamps_ns[0]

    moving = next((i for i, (_, _, speed) in enumerate(samples) if speed > 0.1), None)
    if moving is None and require_goal:
        raise ValueError(f"no flight motion in {path}")
    start = 0 if mission_start_ns is not None else max(0, (moving or 1) - 1)

    end = None
    for i in range(moving or 0, len(samples)):
        _, position, speed = samples[i]
        if np.linalg.norm(position - goal) <= goal_radius and speed <= stop_speed:
            end = i + 1
            break
    completed = end is not None
    if end is None and require_goal:
        raise ValueError(f"goal was not reached in {path}")
    if end is None:
        if collision_ns is not None:
            end = next(
                (i + 1 for i, sample in enumerate(samples) if sample[0] >= collision_ns),
                len(samples),
            )
        elif moving is not None:
            last_moving = max(
                i for i, (_, _, speed) in enumerate(samples) if speed > 0.1
            )
            end = min(len(samples), last_moving + 2)
        else:
            end = len(samples)

    end = max(start + 2, min(end, len(samples)))

    positions = np.asarray([sample[1] for sample in samples[start:end]])
    speeds = np.asarray([sample[2] for sample in samples[start:end]])
    collision_position = None
    if collision_ns is not None and samples[start][0] <= collision_ns <= samples[end - 1][0]:
        collision_sample = min(
            samples[start:end], key=lambda sample: abs(sample[0] - collision_ns)
        )
        collision_position = collision_sample[1]
    return positions, speeds, completed, collision_position


def dilate_voxels(image: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate occupied cells for display without changing source-map metrics."""
    if radius_cells <= 0:
        return image
    rows, columns = image.shape
    padded = np.pad(image, radius_cells, mode="constant")
    dilated = np.zeros_like(image)
    diameter = 2 * radius_cells + 1
    for row_offset in range(diameter):
        for column_offset in range(diameter):
            window = padded[
                row_offset:row_offset + rows,
                column_offset:column_offset + columns,
            ]
            np.maximum(dilated, window, out=dilated)
    return dilated


def draw_voxels(ax, image, bounds):
    x_min, x_max, y_min, y_max = bounds
    masked = np.ma.masked_equal(image, 0)
    return ax.imshow(
        masked,
        origin="lower",
        extent=(y_min, y_max, x_min, x_max),
        cmap=ListedColormap(["#20272d"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
        # Keep the voxel cells square in world coordinates.  ``auto`` makes
        # the long mission corridor visibly stretch the UE geometry.
        aspect="equal",
        alpha=0.82,
        zorder=0,
    )


def archive_existing_outputs(output: Path) -> None:
    """Preserve previous plot files before replacing them."""
    suffixes = (output.suffix, ".pdf")
    existing = [output.with_suffix(suffix) for suffix in suffixes]
    existing = [path for path in existing if path.is_file()]
    if not existing:
        return
    archive_dir = output.parent / "history"
    archive_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    for source in existing:
        destination = archive_dir / f"{output.stem}_{stamp}{source.suffix}"
        counter = 2
        while destination.exists():
            destination = archive_dir / (
                f"{output.stem}_{stamp}_{counter}{source.suffix}"
            )
            counter += 1
        shutil.copy2(source, destination)


def colored_trajectory(
    ax, positions, speeds, label, norm, completed=True, collision_position=None
):
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
    if completed:
        ax.scatter(
            points[-1, 0], points[-1, 1], marker="*", s=80,
            c="#d62728", edgecolors="white", linewidths=0.7, zorder=4,
        )
    else:
        ax.scatter(
            points[-1, 0], points[-1, 1], marker="X", s=55,
            c="#c62828", edgecolors="white", linewidths=0.7, zorder=4,
        )
    if collision_position is not None and completed:
        ax.scatter(
            collision_position[1], collision_position[0], marker="X", s=55,
            c="#c62828", edgecolors="white", linewidths=0.7, zorder=5,
        )
    ax.text(
        0.015, 0.94, label,
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=plt.rcParams["axes.titlesize"],
        fontweight="semibold",
        color="#202124",
        zorder=6,
    )
    ax.set_ylabel("Lateral $x$ (m)")
    ax.grid(True, color="#d9dde1", linewidth=0.6, alpha=0.8)
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_color("#202124")
        spine.set_linewidth(0.65)
    ax.margins(x=0.02, y=0.18)
    return collection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", action="append", default=[],
                        metavar="LABEL=JSONL")
    parser.add_argument(
        "--failed-baseline", action="append", default=[],
        metavar="LABEL=JSONL",
        help="plot a recorded non-goal-reaching flight with an X endpoint",
    )
    parser.add_argument("--scalenav", required=True, type=Path)
    parser.add_argument("--scalenav-label", default="ScaleNav (ours)")
    parser.add_argument(
        "--layout-order", action="append", default=[], metavar="LABEL",
        help="place runs in this label order after loading them",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--goal", nargs=3, type=float, default=(0.0, 140.0, 1.6))
    parser.add_argument("--goal-radius", type=float, default=0.5)
    parser.add_argument("--stop-speed", type=float, default=0.3)
    parser.add_argument("--speed-max", type=float, default=6.0)
    parser.add_argument(
        "--columns", type=int, default=1,
        help="number of subplot columns (default: 1)",
    )
    parser.add_argument(
        "--pointcloud-session", "--density-session", dest="pointcloud_session",
        type=Path,
        help="session used to reconstruct the voxelized point-cloud backdrop",
    )
    parser.add_argument(
        "--truth-map", type=Path, default=DEFAULT_TRUTH_MAP,
        help="complete UE Map2 truth PLY used for the voxel backdrop",
    )
    parser.add_argument(
        "--pointcloud-start-ns", "--density-start-ns",
        dest="pointcloud_start_ns", type=int,
    )
    parser.add_argument(
        "--pointcloud-end-ns", "--density-end-ns",
        dest="pointcloud_end_ns", type=int,
    )
    parser.add_argument(
        "--voxel-size", "--density-voxel-size", dest="voxel_size",
        type=float, default=0.1,
    )
    parser.add_argument(
        "--voxel-dilation-cells", type=int, default=0,
        help="display-only Chebyshev dilation radius in voxel cells",
    )
    args = parser.parse_args()
    if args.voxel_size <= 0.0:
        parser.error("--voxel-size must be positive")
    if args.voxel_dilation_cells < 0:
        parser.error("--voxel-dilation-cells must be nonnegative")
    if args.columns < 1:
        parser.error("--columns must be positive")

    goal = np.asarray(args.goal, dtype=float)
    baselines = []
    for item in args.baseline:
        if "=" not in item:
            parser.error("--baseline must be LABEL=JSONL")
        label, raw_path = item.split("=", 1)
        baselines.append((label, *load_flight(
            Path(raw_path), goal, args.goal_radius, args.stop_speed
        )))
    for item in args.failed_baseline:
        if "=" not in item:
            parser.error("--failed-baseline must be LABEL=JSONL")
        label, raw_path = item.split("=", 1)
        baselines.append((label, *load_flight(
            Path(raw_path), goal, args.goal_radius, args.stop_speed,
            require_goal=False,
        )))
    ours = load_flight(args.scalenav, goal, args.goal_radius, args.stop_speed)

    plt.rcParams.update({
        "font.size": 7.5,
        "axes.titlesize": 8.2,
        "axes.labelsize": 7.5,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "font.family": "DejaVu Sans",
    })
    runs = baselines + [(args.scalenav_label, *ours)]
    if args.layout_order:
        requested = {label: index for index, label in enumerate(args.layout_order)}
        labels = {run[0] for run in runs}
        unknown = set(args.layout_order) - labels
        if unknown:
            parser.error(
                "--layout-order contains unknown labels: "
                + ", ".join(sorted(unknown))
            )
        runs.sort(key=lambda run: requested.get(run[0], len(requested)))
    row_count = len(runs)
    column_count = min(args.columns, row_count)
    grid_rows = math.ceil(row_count / column_count)
    figure, axes = plt.subplots(
        grid_rows, column_count,
        figsize=(4.5 * column_count, 2.30 * grid_rows + 1.20),
        sharex=True, sharey=True,
        squeeze=False,
    )
    axes = axes.ravel()
    norm = Normalize(0.0, args.speed_max)

    if args.pointcloud_session:
        image, bounds, frame_count, occupied_cells, accepted_points = load_voxel_map(
            args.pointcloud_session,
            args.pointcloud_start_ns,
            args.pointcloud_end_ns,
            args.voxel_size,
        )
        print(
            f"Point-cloud backdrop: {frame_count} sampled frames, "
            f"{accepted_points} accepted points, {occupied_cells} occupied "
            f"{args.voxel_size:g} m voxels; display dilation "
            f"{args.voxel_dilation_cells} cells"
        )
        display_image = dilate_voxels(image, args.voxel_dilation_cells)
        for axis in axes:
            draw_voxels(axis, display_image, bounds)
    elif args.truth_map and args.truth_map.is_file():
        image, bounds, accepted_points, occupied_cells = load_truth_voxel_map(
            args.truth_map, args.voxel_size,
        )
        print(
            f"UE truth backdrop: {accepted_points} points, {occupied_cells} "
            f"occupied {args.voxel_size:g} m voxels; display dilation "
            f"{args.voxel_dilation_cells} cells"
        )
        display_image = dilate_voxels(image, args.voxel_dilation_cells)
        for axis in axes:
            draw_voxels(axis, display_image, bounds)
    collection = None
    for axis, (
        label, positions, speeds, completed, collision_position
    ) in zip(axes, runs):
        collection = colored_trajectory(
            axis, positions, speeds, label, norm, completed=completed,
            collision_position=collision_position,
        )
    for index, axis in enumerate(axes):
        if index >= row_count:
            axis.set_visible(False)
            continue
        if index % column_count:
            axis.set_ylabel("")
        if index // column_count == grid_rows - 1:
            axis.set_xlabel("Longitudinal $y$ (m)")
    # Keep the original mission-corridor framing so lateral detours remain
    # legible while every panel shares the selected backdrop.
    for ax in axes:
        ax.set_xlim(-5, 145)
        ax.set_ylim(-45, 45)
        # Trajectory and obstacle geometry must share the same metre-to-pixel
        # scale in both directions.  ``box`` preserves the requested world
        # limits and lets Matplotlib size the axes accordingly.
        ax.set_aspect("equal", adjustable="box")

    # The x-axis label is the only content below the last panel.  Keep a
    # compact bottom margin so the exported figure does not carry a large
    # empty strip underneath the map.
    if column_count == 1:
        margins = dict(left=0.11, right=0.86, bottom=0.045)
        colorbar_bounds = (0.89, 0.22, 0.025, 0.56)
    else:
        margins = dict(left=0.07, right=0.92, bottom=0.065)
        colorbar_bounds = (0.94, 0.22, 0.012, 0.56)
    figure.subplots_adjust(top=0.98, hspace=0.12, wspace=0.10, **margins)
    speed_colorbar_axis = figure.add_axes(colorbar_bounds)
    colorbar = figure.colorbar(collection, cax=speed_colorbar_axis,
                               orientation="vertical")
    colorbar.set_label("Speed (m/s)", labelpad=6)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    archive_existing_outputs(args.output)
    # Trim only the outer canvas whitespace.  The axes retain equal world
    # scales, so this changes the page footprint without distorting the map.
    figure.savefig(args.output, dpi=300, bbox_inches="tight", pad_inches=0.02)
    figure.savefig(
        args.output.with_suffix(".pdf"), bbox_inches="tight", pad_inches=0.02
    )


if __name__ == "__main__":
    main()
