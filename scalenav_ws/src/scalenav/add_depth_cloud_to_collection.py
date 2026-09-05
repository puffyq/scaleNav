#!/usr/bin/env python3
"""Add a world-frame point-cloud sample projected from a 0903 depth bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

from scalenav.ds_camera import (
    DoubleSphereIntrinsics,
    RECORDED_MAX_DISTANCE_M,
    depth_to_original_camera_points,
    double_sphere_unproject_grid,
)


DEPTH_INTRINSICS = DoubleSphereIntrinsics(
    fx=90.80628858336394, fy=90.933561368389,
    cx=253.42890566638994, cy=259.24566303155916,
    xi=-0.31058127136562713, alpha=0.56406562076283007,
    width=512, height=512,
)


def rotation(q) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("collection", type=Path)
    parser.add_argument("--sensor-max-distance-m", type=float, default=RECORDED_MAX_DISTANCE_M)
    parser.add_argument("--cloud-stride", type=int, default=1)
    parser.add_argument("--fixed-altitude", type=float)
    args = parser.parse_args()
    typestore = get_typestore(Stores.ROS1_NOETIC)
    rays, ray_valid = double_sphere_unproject_grid(DEPTH_INTRINSICS)
    odom = {}
    depth = []
    with Reader(args.bag) as reader:
        connections = {connection.topic: connection for connection in reader.connections}
        odom_connection = connections["/omni_record/odom"]
        depth_connection = connections["/omni_record/depth_visual"]
        for connection, timestamp, raw in reader.messages(connections=[odom_connection]):
            odom[int(timestamp)] = typestore.deserialize_ros1(raw, connection.msgtype)
        for connection, timestamp, raw in reader.messages(connections=[depth_connection]):
            depth.append((int(timestamp), typestore.deserialize_ros1(raw, connection.msgtype)))
    if not odom or not depth:
        raise RuntimeError("bag has no odometry or depth_visual")
    first = odom[min(odom)]
    origin = np.array([first.pose.pose.position.x, first.pose.pose.position.y, 0.0])
    points = []
    frames = 0
    for stamp, message in depth:
        pose = odom.get(stamp)
        if pose is None:
            continue
        image = np.asarray(message.data, dtype=np.uint8).reshape(message.height, message.step)[:, : message.width]
        body = depth_to_original_camera_points(
            image, DEPTH_INTRINSICS,
            rays=rays, ray_valid=ray_valid,
            max_distance_m=args.sensor_max_distance_m,
            include_far_plane=True,
            stride=args.cloud_stride,
        ).astype(np.float64)
        if not len(body):
            continue
        if len(body) > 600:
            body = body[np.linspace(0, len(body) - 1, 600, dtype=np.int64)]
        p = pose.pose.pose.position
        position = np.array([p.x, p.y, p.z], dtype=np.float64) - origin
        if args.fixed_altitude is not None:
            position[2] = args.fixed_altitude
        points.extend((body @ rotation(pose.pose.pose.orientation).T + position).tolist())
        frames += 1
    data = json.loads(args.collection.read_text(encoding="utf-8"))
    data["pointcloud"] = points
    data.setdefault("counts", {})["depth_cloud"] = frames
    temporary = args.collection.with_suffix(args.collection.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=True), encoding="utf-8")
    temporary.replace(args.collection)
    print(f"added {len(points)} projected cloud points from {frames} depth frames")


if __name__ == "__main__":
    main()
