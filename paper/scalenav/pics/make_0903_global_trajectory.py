#!/usr/bin/env python3
"""Render the complete 0903 ENU flight trajectory from the recorded odometry."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


def load_odom(bag: Path) -> tuple[np.ndarray, np.ndarray]:
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    typestore = get_typestore(Stores.ROS1_NOETIC)
    rows: list[tuple[int, float, float, float]] = []
    with Reader(bag) as reader:
        connection = next(item for item in reader.connections if item.topic == "/omni_record/odom")
        first_stamp = None
        first_position = None
        for conn, stamp, raw in reader.messages(connections=[connection]):
            message = typestore.deserialize_ros1(raw, conn.msgtype)
            position = message.pose.pose.position
            if first_stamp is None:
                first_stamp = int(stamp)
                first_position = np.array([float(position.x), float(position.y), float(position.z)])
            rows.append((int(stamp), float(position.x), float(position.y), float(position.z)))
    values = np.asarray(rows, dtype=np.float64)
    trajectory = values[:, 1:]
    if len(trajectory) < 2 or first_position is None:
        raise RuntimeError("the bag does not contain enough odometry samples")
    # Match replay_ros1_ds_bag_ros2.py: normalize horizontal origin only and
    # preserve the recorded altitude in the ENU world frame.
    trajectory[:, :2] -= first_position[:2]
    time_s = (values[:, 0] - values[0, 0]) / 1e9
    return trajectory, time_s


def load_odom_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((float(row["time_s"]), float(row["x"]),
                         float(row["y"]), float(row["z"])))
    values = np.asarray(rows, dtype=np.float64)
    if len(values) < 2:
        raise RuntimeError("the odometry CSV does not contain enough samples")
    return values[:, 1:], values[:, 0]


def set_bounds(axis, x: np.ndarray, y: np.ndarray, pad: float = 0.06) -> None:
    low = np.array([np.min(x), np.min(y)], dtype=float)
    high = np.array([np.max(x), np.max(y)], dtype=float)
    span = np.maximum(high - low, 1.0)
    axis.set_xlim(low[0] - pad * span[0], high[0] + pad * span[0])
    axis.set_ylim(low[1] - pad * span[1], high[1] + pad * span[1])


def render(trajectory: np.ndarray, time_s: np.ndarray, out: Path) -> None:
    forest_exit_t = 72.0
    building_start_t = 80.0
    forest_exit = trajectory[np.argmin(np.abs(time_s - forest_exit_t))]
    building_start = trajectory[np.argmin(np.abs(time_s - building_start_t))]
    start = trajectory[0]
    finish = trajectory[-1]

    plt.rcParams.update({
        "font.family": "serif", "font.serif": ["DejaVu Serif"],
        "font.size": 9.0, "pdf.fonttype": 42, "ps.fonttype": 42,
        "svg.fonttype": "none",
    })
    fig = plt.figure(figsize=(10.2, 5.6), dpi=300)
    grid = fig.add_gridspec(1, 2, width_ratios=(1.08, 1.0), wspace=0.12)
    ax3d = fig.add_subplot(grid[0, 0], projection="3d")
    ax2d = fig.add_subplot(grid[0, 1])

    forest = time_s <= forest_exit_t
    transition = (time_s > forest_exit_t) & (time_s <= building_start_t)
    building = time_s > building_start_t
    segments = ((forest, "#2a9d8f", "woodland flight"),
                (transition, "#e9a23b", "climb / transition"),
                (building, "#277da1", "building flight"))
    for mask, color, _ in segments:
        ax3d.plot(trajectory[mask, 0], trajectory[mask, 1], trajectory[mask, 2],
                  color="white", linewidth=3.3, alpha=0.9)
        ax3d.plot(trajectory[mask, 0], trajectory[mask, 1], trajectory[mask, 2],
                  color=color, linewidth=1.8)
        ax2d.plot(trajectory[mask, 0], trajectory[mask, 1], color="white", linewidth=3.1)
        ax2d.plot(trajectory[mask, 0], trajectory[mask, 1], color=color, linewidth=1.7)

    for point, color, label in ((start, "#264653", "start"),
                                (forest_exit, "#2a9d8f", "woodland exit"),
                                (building_start, "#e9a23b", "building segment"),
                                (finish, "#d1495b", "building endpoint")):
        ax3d.scatter(*point, s=34, color=color, edgecolors="white", linewidths=0.7, depthshade=False)
        ax2d.scatter(point[0], point[1], s=44, color=color, edgecolors="white", linewidths=0.8, zorder=8)

    ax3d.set_title("Complete recorded flight in ENU", loc="left", fontsize=11, fontweight="bold", pad=8)
    ax3d.set_xlabel("East [m]", labelpad=4)
    ax3d.set_ylabel("North [m]", labelpad=4)
    ax3d.set_zlabel("Up [m]", labelpad=4)
    ax3d.view_init(elev=28, azim=-63)
    ax3d.grid(True, alpha=0.25)
    ax3d.set_box_aspect((1.6, 1.0, 0.58))

    ax2d.set_title("Top view and scene endpoints", loc="left", fontsize=11, fontweight="bold", pad=8)
    ax2d.set_xlabel("East [m]")
    ax2d.set_ylabel("North [m]")
    ax2d.set_aspect("equal", adjustable="box")
    ax2d.grid(True, alpha=0.25, linewidth=0.5)
    set_bounds(ax2d, trajectory[:, 0], trajectory[:, 1])
    ax2d.annotate("woodland exit", forest_exit[:2], xytext=(8, 9), textcoords="offset points",
                  color="#237f72", fontsize=8, fontweight="bold",
                  arrowprops=dict(arrowstyle="-", color="#237f72", lw=0.7))
    ax2d.annotate("building endpoint", finish[:2], xytext=(-82, -17), textcoords="offset points",
                  color="#a8364a", fontsize=8, fontweight="bold",
                  arrowprops=dict(arrowstyle="-", color="#a8364a", lw=0.7))

    legend = [
        Line2D([], [], color="#2a9d8f", lw=2, label="woodland segment (0–72 s)"),
        Line2D([], [], color="#e9a23b", lw=2, label="climb / transition (72–80 s)"),
        Line2D([], [], color="#277da1", lw=2, label="building segment (80–165 s)"),
        Patch(facecolor="none", edgecolor="none", label="markers: start, scene endpoints"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=3, frameon=False,
               bbox_to_anchor=(0.52, -0.005), fontsize=8.2, handlelength=2.4)
    fig.suptitle("0903 dataset: global trajectory and scene-specific endpoints",
                 fontsize=13, fontweight="bold", x=0.03, ha="left", y=0.995)
    fig.subplots_adjust(left=0.035, right=0.985, top=0.91, bottom=0.14)
    for suffix in (".png", ".pdf", ".svg"):
        fig.savefig(out.with_suffix(suffix), dpi=300, bbox_inches="tight", pad_inches=0.04)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--odom-csv", type=Path,
                        help="optional normalized odometry CSV, avoiding ROS bag dependencies")
    args = parser.parse_args()
    trajectory, time_s = load_odom_csv(args.odom_csv) if args.odom_csv else load_odom(args.bag)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    render(trajectory, time_s, args.out)
    print(f"trajectory samples={len(trajectory)} duration={time_s[-1]:.3f}s")
    print(f"outputs={args.out.with_suffix('.png')} {args.out.with_suffix('.pdf')} {args.out.with_suffix('.svg')}")


if __name__ == "__main__":
    main()
