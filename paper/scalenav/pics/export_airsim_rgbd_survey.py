#!/usr/bin/env python3
"""Export a privileged AirSim DepthPlanar survey in world_enu coordinates."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

import colosseum


def quaternion_matrix(quaternion) -> np.ndarray:
    x = float(quaternion.x_val)
    y = float(quaternion.y_val)
    z = float(quaternion.z_val)
    w = float(quaternion.w_val)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


def voxel_downsample(points: np.ndarray, resolution: float) -> np.ndarray:
    keys = np.floor(points / resolution).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def write_ply(path: Path, points: np.ndarray) -> None:
    with path.open("w", encoding="ascii") as stream:
        stream.write(
            "ply\nformat ascii 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "end_header\n"
        )
        np.savetxt(stream, points, fmt="%.4f")


def project_response(response, pixel_stride: int, max_range: float) -> np.ndarray:
    width = int(response.width)
    height = int(response.height)
    depth = np.asarray(response.image_data_float, dtype=np.float64)
    if depth.size != width * height:
        raise ValueError("DepthPlanar response has an invalid payload size")
    depth = depth.reshape(height, width)
    rows = np.arange(0, height, pixel_stride)
    columns = np.arange(0, width, pixel_stride)
    u, v = np.meshgrid(columns, rows)
    selected = depth[np.ix_(rows, columns)]
    valid = np.isfinite(selected) & (selected > 0.0) & (selected < max_range)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    focal = (width * 0.5) / math.tan(math.radians(90.0 * 0.5))
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    distance = selected[valid]
    # AirSim's camera frame is forward/right/down and DepthPlanar is optical-Z.
    camera_points = np.column_stack((
        distance,
        (u[valid] - cx) * distance / focal,
        (v[valid] - cy) * distance / focal,
    ))
    camera_position = np.asarray([
        response.camera_position.x_val,
        response.camera_position.y_val,
        response.camera_position.z_val,
    ], dtype=np.float64)
    world_ned = camera_points @ quaternion_matrix(response.camera_orientation).T
    world_ned += camera_position
    return np.column_stack((world_ned[:, 1], world_ned[:, 0], -world_ned[:, 2]))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_prefix", type=Path)
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="")
    parser.add_argument("--camera-name", default="0")
    parser.add_argument("--x", type=float, nargs=3, default=(-40.0, 40.0, 20.0),
                        metavar=("MIN", "MAX", "STEP"))
    parser.add_argument("--y", type=float, nargs=3, default=(0.0, 140.0, 20.0),
                        metavar=("MIN", "MAX", "STEP"))
    parser.add_argument("--survey-z", type=float, default=1.6)
    parser.add_argument("--keep-z", type=float, nargs=2, default=(0.6, 2.6),
                        metavar=("MIN", "MAX"))
    parser.add_argument("--max-range", type=float, default=30.0)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--voxel", type=float, default=0.25)
    parser.add_argument("--settle", type=float, default=0.04)
    return parser.parse_args()


def inclusive_range(start: float, stop: float, step: float) -> np.ndarray:
    if step <= 0.0 or stop < start:
        raise ValueError("survey ranges require MIN <= MAX and STEP > 0")
    return np.arange(start, stop + step * 0.25, step)


def main() -> None:
    args = parse_args()
    x_values = inclusive_range(*args.x)
    y_values = inclusive_range(*args.y)
    if args.pixel_stride < 1 or args.max_range <= 0.0 or args.voxel <= 0.0:
        raise ValueError("pixel stride, max range, and voxel size must be positive")

    client = colosseum.VehicleClient(port=args.port, timeout_value=600)
    if not client.ping():
        raise RuntimeError(f"Colosseum did not answer on port {args.port}")
    original_pose = client.simGetVehiclePose(args.vehicle_name)
    request = colosseum.ImageRequest(
        args.camera_name,
        colosseum.ImageType.DepthPlanar,
        pixels_as_float=True,
        compress=False,
    )
    yaw_values = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
    clouds = []
    captures = 0
    try:
        for y_enu in y_values:
            for x_enu in x_values:
                for yaw_ned in yaw_values:
                    pose = colosseum.Pose(
                        colosseum.Vector3r(
                            float(y_enu), float(x_enu), -float(args.survey_z)
                        ),
                        colosseum.to_quaternion(0.0, 0.0, yaw_ned),
                    )
                    client.simSetVehiclePose(
                        pose, ignore_collision=True, vehicle_name=args.vehicle_name
                    )
                    if args.settle > 0.0:
                        time.sleep(args.settle)
                    responses = client.simGetImages(
                        [request], vehicle_name=args.vehicle_name
                    )
                    if len(responses) != 1:
                        raise RuntimeError("AirSim returned no DepthPlanar frame")
                    points = project_response(
                        responses[0], args.pixel_stride, args.max_range
                    )
                    if len(points):
                        clouds.append(points)
                    captures += 1
    finally:
        client.simSetVehiclePose(
            original_pose, ignore_collision=True, vehicle_name=args.vehicle_name
        )

    if not clouds:
        raise RuntimeError("survey produced no finite depth points")
    points = np.concatenate(clouds)
    x_min = args.x[0] - args.x[2] / 2.0
    x_max = args.x[1] + args.x[2] / 2.0
    y_min = args.y[0] - args.y[2] / 2.0
    y_max = args.y[1] + args.y[2] / 2.0
    keep = (
        (points[:, 0] >= x_min) & (points[:, 0] <= x_max)
        & (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
        & (points[:, 2] >= args.keep_z[0]) & (points[:, 2] <= args.keep_z[1])
    )
    points = voxel_downsample(points[keep], args.voxel)

    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    ply_path = args.output_prefix.with_suffix(".ply").resolve()
    metadata_path = args.output_prefix.with_suffix(".json").resolve()
    write_ply(ply_path, points)
    metadata = {
        "source": "privileged AirSim DepthPlanar grid survey",
        "frame": "world_enu",
        "rpc_port": args.port,
        "camera": args.camera_name,
        "survey_positions": int(len(x_values) * len(y_values)),
        "yaw_views_per_position": len(yaw_values),
        "captures": captures,
        "survey_x_enu_m": x_values.tolist(),
        "survey_y_enu_m": y_values.tolist(),
        "survey_z_enu_m": args.survey_z,
        "retained_bounds_enu_m": {
            "x": [x_min, x_max],
            "y": [y_min, y_max],
            "z": list(args.keep_z),
        },
        "max_depth_m": args.max_range,
        "pixel_stride": args.pixel_stride,
        "voxel_size_m": args.voxel,
        "surface_voxels": int(len(points)),
        "ply": str(ply_path),
        "vehicle_pose_restored": True,
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
