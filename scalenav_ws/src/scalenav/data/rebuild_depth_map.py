"""Augment existing scene PLY files with points reconstructed from depth frames."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import rtoml

from .snapshot_dataset import (
    PoseSample,
    depth_planar_to_world_ned,
    read_ascii_point_cloud_ply,
    write_point_cloud_ply,
)


def rebuild_scene(scene_dir: Path, *, stride: int, max_points_per_frame: int) -> int:
    data_path = scene_dir / "data.toml"
    tree_path = scene_dir / "tree.ply"
    if not data_path.is_file() or not tree_path.is_file():
        raise FileNotFoundError(f"incomplete scene: {scene_dir}")
    with data_path.open("r", encoding="utf-8") as file:
        document = rtoml.load(file)
    static = read_ascii_point_cloud_ply(tree_path)
    static = static[np.linalg.norm(static, axis=1) > 1.0e-4]
    points = [static]
    horizontal_fov = float(document.get("depthCameraHorizontalFOV", 90.0))
    vertical_fov = float(document.get("depthCameraVerticalFOV", 60.0))
    max_depth = float(document.get("depthMaxMeters", 20.0))
    frame_count = 0
    texture_dir = scene_dir / "Textures"
    for record in document.get("dataArray", []):
        depth_path = texture_dir / str(record.get("depthFileName", ""))
        depth = cv2.imread(str(depth_path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
        if depth is None:
            raise FileNotFoundError(depth_path)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        pose = PoseSample(
            tuple(float(value) for value in record["posStart"]),
            tuple(float(value) for value in record["orientationWxyz"]),
        )
        points.append(
            depth_planar_to_world_ned(
                depth,
                pose,
                horizontal_fov,
                vertical_fov,
                max_depth,
                stride=stride,
                max_points=max_points_per_frame,
            )
        )
        frame_count += 1
    merged = np.concatenate(points, axis=0)
    temporary = tree_path.with_name(f".{tree_path.name}.depth.tmp")
    backup = tree_path.with_name(f".{tree_path.name}.before_depth")
    if not backup.exists():
        shutil.copyfile(tree_path, backup)
    write_point_cloud_ply(temporary, merged)
    temporary.replace(tree_path)
    return frame_count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, type=Path)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--max-points-per-frame", type=int, default=1500)
    args = parser.parse_args()
    if args.stride <= 0 or args.max_points_per_frame <= 0:
        parser.error("--stride and --max-points-per-frame must be positive")
    scenes = sorted(args.data.glob("Scene_*"))
    if not scenes:
        raise FileNotFoundError(f"no Scene_* directories under {args.data}")
    for scene in scenes:
        frames = rebuild_scene(
            scene,
            stride=args.stride,
            max_points_per_frame=args.max_points_per_frame,
        )
        print(f"{scene}: merged {frames} depth frames into tree.ply")


if __name__ == "__main__":
    main()
