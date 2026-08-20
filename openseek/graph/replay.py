from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import cv2
import numpy as np
import rtoml
from scipy.spatial.transform import Rotation

from .depth_query import DepthSafeVolumeQuery
from .frgraph_adapter import FRGraphAdapter
from .sparse_graph import GraphConfig, SparseDepthGraph


_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def synthetic_wall_depth(
    *,
    width: int = 160,
    height: int = 96,
    far_depth_m: float = 20.0,
    wall_depth_m: float = 3.0,
    wall_half_width_px: int = 22,
) -> np.ndarray:
    depth = np.full((height, width), far_depth_m, dtype=np.float32)
    center = width // 2
    depth[:, center - wall_half_width_px : center + wall_half_width_px] = wall_depth_m
    return depth


def run_frame(
    depth_m: np.ndarray,
    *,
    position_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    goal_body: np.ndarray,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    robot_radius_m: float,
    use_frgraph: bool = False,
    heatmap: np.ndarray | None = None,
) -> tuple[SparseDepthGraph, dict]:
    goal_world = position_world + rotation_body_to_world @ goal_body
    query = DepthSafeVolumeQuery(
        depth_m,
        horizontal_fov_deg=horizontal_fov_deg,
        vertical_fov_deg=vertical_fov_deg,
        robot_radius_m=robot_radius_m,
    )
    graph = SparseDepthGraph(GraphConfig())
    frgraph_regions = []
    candidate_directions = None
    if use_frgraph:
        frgraph_regions = FRGraphAdapter(
            horizontal_fov_deg=horizontal_fov_deg,
            vertical_fov_deg=vertical_fov_deg,
            candidate_distance_m=5.0,
            robot_radius_m=robot_radius_m,
        ).extract(depth_m, heatmap)
        candidate_directions = [region.direction_body_flu for region in frgraph_regions]
    update = graph.update(
        position_world=position_world,
        rotation_body_to_world=rotation_body_to_world,
        goal_world=goal_world,
        depth_query=query,
        heatmap=heatmap,
        candidate_directions_body=candidate_directions,
    )

    def waypoint_body(waypoint_world: np.ndarray | None) -> list[float] | None:
        if waypoint_world is None:
            return None
        return (rotation_body_to_world.T @ (waypoint_world - position_world)).tolist()

    result = {
        "goalBody": goal_body.tolist(),
        "certifiedWaypointBody": waypoint_body(update.certified_waypoint_world),
        "optimisticWaypointBody": waypoint_body(update.optimistic_waypoint_world),
        "certifiedPath": list(update.certified_path),
        "optimisticPath": list(update.optimistic_path),
        "stateCounts": update.state_counts,
        "graph": graph.to_dict(),
        "frgraph": {
            "enabled": use_frgraph,
            "regionCount": len(frgraph_regions),
            "regions": [region.to_dict() for region in frgraph_regions],
        },
    }
    return graph, result


def load_scene_frame(
    data_root: Path, scene_name: str, frame_index: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float, dict]:
    scene = data_root / scene_name
    document_path = scene / "data.toml"
    if not document_path.is_file():
        raise FileNotFoundError(f"Map2 scene metadata not found: {document_path}")
    document = rtoml.load(document_path)
    records = document.get("dataArray", [])
    if frame_index < 0 or frame_index >= len(records):
        raise IndexError(f"frame {frame_index} is outside [0, {len(records) - 1}]")
    record = records[frame_index]
    depth_path = scene / "Textures" / record["depthFileName"]
    depth = cv2.imread(str(depth_path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    if depth is None:
        raise FileNotFoundError(depth_path)
    if depth.ndim == 3:
        depth = depth[:, :, 0]

    position = np.asarray(record["posStart"], dtype=np.float64)
    quaternion = np.asarray(record["orientationWxyz"], dtype=np.float64)
    rotation_ned_frd = Rotation.from_quat(
        [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    ).as_matrix()
    rotation_ned_flu = rotation_ned_frd @ _FRD_TO_FLU
    horizontal_fov = float(document.get("depthCameraHorizontalFOV", 90.0))
    vertical_fov = float(document.get("depthCameraVerticalFOV", 60.0))
    return depth.astype(np.float32), position, rotation_ned_flu, horizontal_fov, vertical_fov, record


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay one Map2 depth frame through the sparse Graph.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--data", type=Path, help="Map2 snapshot dataset root")
    source.add_argument("--synthetic-wall", action="store_true")
    parser.add_argument("--scene", default="Scene_0001")
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--goal-distance", type=float, default=20.0)
    parser.add_argument("--robot-radius", type=float, default=0.6)
    parser.add_argument(
        "--frgraph",
        action="store_true",
        help="use the ROS-free FRGraph-style directional region adapter",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.goal_distance <= 0.0 or args.robot_radius <= 0.0:
        parser.error("goal distance and robot radius must be positive")

    if args.synthetic_wall:
        depth = synthetic_wall_depth()
        position = np.zeros(3, dtype=np.float64)
        rotation = np.eye(3, dtype=np.float64)
        horizontal_fov, vertical_fov = 90.0, 60.0
        source_label = "synthetic-map2-wall"
        record = None
    else:
        depth, position, rotation, horizontal_fov, vertical_fov, record = load_scene_frame(
            args.data, args.scene, args.frame
        )
        source_label = str(args.data / args.scene)

    _, result = run_frame(
        depth,
        position_world=position,
        rotation_body_to_world=rotation,
        goal_body=np.array([args.goal_distance, 0.0, 0.0]),
        horizontal_fov_deg=horizontal_fov,
        vertical_fov_deg=vertical_fov,
        robot_radius_m=args.robot_radius,
        use_frgraph=args.frgraph,
    )
    result["source"] = source_label
    result["frame"] = args.frame
    result["record"] = record
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
        print(f"wrote {args.output}")
    print(payload)


if __name__ == "__main__":
    main()
