#!/usr/bin/env python3
"""Reconstruct the Map2 occupied point cloud from one ScaleNav log.

The recorded PCDs are body-FLU points (``frame_id=base_link``).  The odometry
orientation is a ROS xyzw body-to-world quaternion and the position is in
world_enu.  This script deliberately aggregates sparse snapshots rather than
rendering individual frames, then provides top-down and side-view checks.

Usage:
    python3 build_map2_pointcloud.py SESSION_DIR OUT_PREFIX
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D


BACKGROUND = "#FAFAF7"
POINT_LOW = "#D9DEE3"
POINT_MID = "#7E9BA8"
POINT_HIGH = "#315E78"
TRAJECTORY = "#24343D"
GOAL = "#315E78"
START = "#FAFAF7"
GRID = "#B5C1C8"

EVENT_STRIDE = 1
POINT_STRIDE = 1
VOXEL_SIZE_M = 0.10
MAX_POINTS = 180_000
ARRIVAL_RADIUS_M = 5.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("out_prefix", type=Path)
    return parser.parse_args()


def load_events(session: Path) -> dict[str, list[dict]]:
    events: dict[str, list[dict]] = {}
    with (session / "index.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            event = json.loads(line)
            events.setdefault(event["kind"], []).append(event)
    for required in ("odom", "pointcloud", "graph", "goal"):
        if not events.get(required):
            raise ValueError(f"session is missing required stream: {required}")
    return events


def nearest_event(events: list[dict], stamp_ns: int) -> dict:
    return min(events, key=lambda event: abs(event["stamp_ns"] - stamp_ns))


def first_mission(events: dict[str, list[dict]]) -> tuple[int, int, np.ndarray, np.ndarray]:
    start_ns = events["graph"][0]["stamp_ns"]
    goal = np.asarray(events["goal"][0]["data"]["position"], dtype=float)
    odometry = [event for event in events["odom"] if event["stamp_ns"] >= start_ns]
    if not odometry:
        raise ValueError("no odometry after the first graph update")
    end_index = len(odometry) - 1
    for index, event in enumerate(odometry):
        position = np.asarray(event["data"]["position"], dtype=float)
        if np.linalg.norm(position[:2] - goal[:2]) <= ARRIVAL_RADIUS_M:
            end_index = index
            break
    mission = np.asarray(
        [event["data"]["position"] for event in odometry[: end_index + 1]],
        dtype=float,
    )
    return start_ns, odometry[end_index]["stamp_ns"], goal, mission


def quat_rotate(quaternion: list[float], vectors: np.ndarray) -> np.ndarray:
    """Rotate vectors with a normalized ROS xyzw quaternion."""
    vector_part = np.asarray(quaternion[:3], dtype=float)
    scalar = float(quaternion[3])
    norm = np.linalg.norm(np.r_[vector_part, scalar])
    if not np.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("odom orientation is not a valid quaternion")
    vector_part /= norm
    scalar /= norm
    cross = 2.0 * np.cross(vector_part, vectors)
    return vectors + scalar * cross + np.cross(vector_part, cross)


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if not len(points):
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def aggregate_point_cloud(
    session: Path,
    events: dict[str, list[dict]],
    start_ns: int,
    end_ns: int,
) -> np.ndarray:
    odometry = events["odom"]
    cloud_events = [
        event
        for event in events["pointcloud"]
        if event.get("file") and start_ns <= event["stamp_ns"] <= end_ns
    ]
    clouds: list[np.ndarray] = []
    for event in cloud_events[::EVENT_STRIDE]:
        frame_id = event.get("data", {}).get("frame_id")
        if frame_id != "base_link":
            raise ValueError(f"expected base_link point cloud, got {frame_id!r}")
        points = np.loadtxt(session / event["file"], skiprows=11)
        if points.ndim != 2 or points.shape[1] < 3:
            continue
        points = points[::POINT_STRIDE, :3]
        pose = nearest_event(odometry, event["stamp_ns"])
        # PCD -> world_enu: body-FLU point, then ROS body-to-world pose.
        world = quat_rotate(pose["data"]["orientation"], points)
        world += np.asarray(pose["data"]["position"], dtype=float)
        clouds.append(world)
    if not clouds:
        raise ValueError("mission contains no usable point cloud")
    cloud = voxel_downsample(np.concatenate(clouds), VOXEL_SIZE_M)
    if len(cloud) > MAX_POINTS:
        indices = np.linspace(0, len(cloud) - 1, MAX_POINTS, dtype=np.int64)
        cloud = cloud[indices]
    return cloud


def set_equal_bounds(axis, x: np.ndarray, y: np.ndarray, margin: float = 5.0) -> None:
    xmin, xmax = float(np.percentile(x, 0.3)), float(np.percentile(x, 99.7))
    ymin, ymax = float(np.percentile(y, 0.3)), float(np.percentile(y, 99.7))
    axis.set_xlim(xmin - margin, xmax + margin)
    axis.set_ylim(ymin - margin, ymax + margin)
    axis.set_aspect("equal", adjustable="box")


def style_axis(axis) -> None:
    axis.set_facecolor(BACKGROUND)
    axis.grid(color=GRID, alpha=0.28, linewidth=0.45)
    axis.tick_params(labelsize=8, length=2.5, color=POINT_MID)
    for spine in axis.spines.values():
        spine.set_color(POINT_MID)
        spine.set_linewidth(0.65)


def draw_map(cloud: np.ndarray, trajectory: np.ndarray, goal: np.ndarray, out: Path) -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 9,
        "figure.facecolor": BACKGROUND,
        "savefig.facecolor": BACKGROUND,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })
    cmap = LinearSegmentedColormap.from_list("height", [POINT_LOW, POINT_MID, POINT_HIGH])
    low, high = np.percentile(cloud[:, 2], [2.0, 98.0])
    figure, axes = plt.subplots(1, 2, figsize=(12.0, 4.8), facecolor=BACKGROUND)
    top, side = axes
    top.scatter(cloud[:, 1], cloud[:, 0], c=cloud[:, 2], cmap=cmap,
                vmin=low, vmax=high, s=0.45, alpha=0.55, linewidths=0,
                rasterized=True)
    top.plot(trajectory[:, 1], trajectory[:, 0], color="white", linewidth=3.2, zorder=4)
    top.plot(trajectory[:, 1], trajectory[:, 0], color=TRAJECTORY, linewidth=1.6, zorder=5)
    top.scatter(trajectory[0, 1], trajectory[0, 0], marker="o", s=46,
                facecolor=START, edgecolor=TRAJECTORY, linewidth=1.2, zorder=6)
    top.scatter(goal[1], goal[0], marker="*", s=165, facecolor=GOAL,
                edgecolor="white", linewidth=0.8, zorder=6)
    set_equal_bounds(top, np.r_[cloud[:, 1], trajectory[:, 1], goal[1]],
                     np.r_[cloud[:, 0], trajectory[:, 0], goal[0]])
    top.set_xlabel("world Y / forward (m)")
    top.set_ylabel("world X / lateral (m)")
    top.set_title("Map2 accumulated depth point cloud", fontsize=12, pad=7)
    top.text(0.02, 0.03, "body-FLU PCD + world_enu odometry", transform=top.transAxes,
             fontsize=8, color=TRAJECTORY,
             bbox=dict(boxstyle="round,pad=0.25", facecolor=BACKGROUND, edgecolor=GRID))
    style_axis(top)

    side.scatter(cloud[:, 1], cloud[:, 2], c=cloud[:, 2], cmap=cmap,
                 vmin=low, vmax=high, s=0.45, alpha=0.55, linewidths=0,
                 rasterized=True)
    side.plot(trajectory[:, 1], trajectory[:, 2], color="white", linewidth=3.2, zorder=4)
    side.plot(trajectory[:, 1], trajectory[:, 2], color=TRAJECTORY, linewidth=1.6, zorder=5)
    side.scatter(trajectory[0, 1], trajectory[0, 2], marker="o", s=46,
                 facecolor=START, edgecolor=TRAJECTORY, linewidth=1.2, zorder=6)
    side.scatter(goal[1], goal[2], marker="*", s=165, facecolor=GOAL,
                 edgecolor="white", linewidth=0.8, zorder=6)
    side.set_xlim(top.get_xlim())
    side.set_ylim(float(np.percentile(np.r_[cloud[:, 2], trajectory[:, 2], goal[2]], 0.3) - 2),
                  float(np.percentile(np.r_[cloud[:, 2], trajectory[:, 2], goal[2]], 99.7) + 2))
    side.set_xlabel("world Y / forward (m)")
    side.set_ylabel("world Z / up (m)")
    side.set_title("Vertical consistency check", fontsize=12, pad=7)
    style_axis(side)

    handles = [
        Line2D([], [], color=TRAJECTORY, lw=1.7, label="flown trajectory"),
        Line2D([], [], color=START, marker="o", markeredgecolor=TRAJECTORY,
               markersize=6, lw=0, label="start"),
        Line2D([], [], color=GOAL, marker="*", markeredgecolor="white",
               markersize=10, lw=0, label="mission goal"),
    ]
    figure.legend(handles=handles, loc="lower center", ncol=3, frameon=False,
                  fontsize=8.5, bbox_to_anchor=(0.5, 0.005))
    figure.tight_layout(rect=(0.0, 0.075, 1.0, 1.0), w_pad=2.0)
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out.with_suffix(".png"), dpi=300)
    figure.savefig(out.with_suffix(".pdf"), dpi=300)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    session = args.session_dir.resolve()
    events = load_events(session)
    start_ns, end_ns, goal, trajectory = first_mission(events)
    cloud = aggregate_point_cloud(session, events, start_ns, end_ns)
    draw_map(cloud, trajectory, goal, args.out_prefix)
    print(
        f"wrote {args.out_prefix.with_suffix('.png')} and "
        f"{args.out_prefix.with_suffix('.pdf')}; points={len(cloud)} "
        f"trajectory={len(trajectory)} bounds="
        f"{np.min(cloud, axis=0).round(2).tolist()}..{np.max(cloud, axis=0).round(2).tolist()}"
    )


if __name__ == "__main__":
    main()
