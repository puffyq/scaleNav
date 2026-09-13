#!/usr/bin/env python3
"""Plot recorded trajectories without changing the UE/AirSim runtime."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_ROOT = Path("/mnt/code/lab/yopo/OpenSeek/scalenav_ws/results/far_vs_scalenav_20260912")
DEFAULT_SCALE = DEFAULT_ROOT / "scalenav.csv"
DEFAULT_FAR = DEFAULT_ROOT / "far.csv"
DEFAULT_OUTPUT = DEFAULT_ROOT / "far_vs_scalenav_top_view.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Draw the best ScaleNav and worst FAR tracks as a local top-view PNG."
    )
    parser.add_argument("--scalenav", type=Path, default=DEFAULT_SCALE)
    parser.add_argument("--far", type=Path, default=DEFAULT_FAR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--origin-enu", type=float, nargs=3, default=(0.0, 0.0, 1.6))
    parser.add_argument(
        "--display-frame",
        choices=("raw", "enu", "ned"),
        default="raw",
        help="display axes; raw/enu preserve CSV x,y, ned swaps x/y and negates z",
    )
    parser.add_argument("--line-width", type=float, default=2.8)
    parser.add_argument("--figure-width", type=float, default=12.0)
    parser.add_argument("--figure-height", type=float, default=8.5)
    parser.add_argument("--max-points", type=int, default=1800)
    # Kept for command-line compatibility with the old UE drawing helper.
    parser.add_argument("--rpc-host", default="127.0.0.1", help=argparse.SUPPRESS)
    parser.add_argument("--rpc-port", type=int, default=41451, help=argparse.SUPPRESS)
    parser.add_argument("--vehicle-name", default="drone_1", help=argparse.SUPPRESS)
    parser.add_argument("--camera-name", default="0", help=argparse.SUPPRESS)
    parser.add_argument("--camera-fov", type=float, default=90.0, help=argparse.SUPPRESS)
    parser.add_argument("--camera-height-m", type=float, default=800.0, help=argparse.SUPPRESS)
    parser.add_argument("--line-thickness", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--point-size", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--settle-seconds", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--draw-only", action="store_true")
    return parser.parse_args()


def read_track(path: Path) -> list[tuple[float, float, float]]:
    if not path.is_file():
        raise FileNotFoundError(f"trajectory CSV not found: {path}")
    points: list[tuple[float, float, float]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                x = float(row["x"])
                y = float(row["y"])
                z = float(row.get("z", "1.6"))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"invalid trajectory row in {path}: {row}") from error
            if all(math.isfinite(value) for value in (x, y, z)):
                points.append((x, y, z))
    if len(points) < 2:
        raise ValueError(f"trajectory needs at least two finite points: {path}")
    return points


def sample(points: list[tuple[float, float, float]], max_points: int) -> list[tuple[float, float, float]]:
    if max_points < 2:
        raise ValueError("--max-points must be at least 2")
    if len(points) <= max_points:
        return points
    step = (len(points) - 1) / (max_points - 1)
    indices = [round(index * step) for index in range(max_points)]
    return [points[index] for index in indices]


def display_point(
    point: tuple[float, float, float], frame: str,
    origin_enu: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Apply an optional display-only axis mapping; source logs are untouched."""
    if frame == "raw":
        return point
    relative = tuple(point[index] - origin_enu[index] for index in range(3))
    if frame == "ned":
        east, north, up = relative
        return north, east, -up
    return relative


def enu_to_ned(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return the conventional ENU-to-NED mapping for plotting only."""
    east, north, up = point
    return north, east, -up


def world_vectors(
    points: list[tuple[float, float, float]],
    origin_enu: tuple[float, float, float],
    display_frame: str = "ned",
) -> list[tuple[float, float, float]]:
    return [display_point(point, display_frame, origin_enu) for point in points]


def main() -> int:
    args = parse_args()
    if args.capture and args.draw_only:
        raise ValueError("--capture and --draw-only cannot be used together")
    scalenav = read_track(args.scalenav)
    far = read_track(args.far)
    origin_enu = tuple(args.origin_enu)
    scale_points = world_vectors(sample(scalenav, args.max_points), origin_enu, args.display_frame)
    far_points = world_vectors(sample(far, args.max_points), origin_enu, args.display_frame)
    figure, axis = plt.subplots(figsize=(args.figure_width, args.figure_height), dpi=150)
    for points, label, color in (
        (scale_points, "ScaleNav (best)", "#159447"),
        (far_points, "FAR (worst)", "#d62828"),
    ):
        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]
        axis.plot(x_values, y_values, color=color, linewidth=args.line_width, label=label)
        axis.scatter(x_values[0], y_values[0], color=color, s=70, zorder=4, marker="o")
        axis.scatter(x_values[-1], y_values[-1], color=color, s=95, zorder=4, marker="^")
    axis.set_aspect("equal", adjustable="datalim")
    if args.display_frame == "ned":
        axis.set_xlabel("NED y / ENU x (m)")
        axis.set_ylabel("NED x / ENU y (m)")
    elif args.display_frame == "enu":
        axis.set_xlabel("ENU x (m)")
        axis.set_ylabel("ENU y (m)")
    else:
        axis.set_xlabel("CSV x (m)")
        axis.set_ylabel("CSV y (m)")
    axis.set_title("FAR vs ScaleNav trajectories (display only)")
    axis.grid(True, color="#d9dde1", linewidth=0.6, alpha=0.8)
    axis.legend(frameon=False)
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, format="png", facecolor="white")
    plt.close(figure)
    metadata = args.output.with_suffix(".json")
    metadata.write_text(
        json.dumps(
            {
                "image": str(args.output),
                "scalenav_csv": str(args.scalenav),
                "far_csv": str(args.far),
                "scalenav_points": len(scalenav),
                "far_points": len(far),
                "display_frame": args.display_frame,
                "origin_enu_m": list(origin_enu),
                "simulator_changed": False,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print("Local plot only: UE coordinates, simulator state, and camera were not changed")
    print(f"ScaleNav: {len(scalenav)} points, green, best trajectory")
    print(f"FAR: {len(far)} points, red, worst trajectory")
    print(f"PNG: {args.output}")
    print(f"metadata: {metadata}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
