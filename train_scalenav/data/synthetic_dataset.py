from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np

from .coordinates import enu_to_ned
from .route_contract import (
    RouteQualityConfig,
    RouteQualityGate,
    RouteRecord,
    build_witness_corridor,
    local_subgoal_on_witness,
    pack_route_records,
    save_route_table,
)
from .snapshot_dataset import CaptureConfig, PoseSample, SceneWriter, write_point_cloud_ply


def _obstacle_cloud_enu() -> np.ndarray:
    axis_x = np.arange(-6.0, 6.01, 0.5, dtype=np.float32)
    axis_y = np.arange(-5.0, 32.01, 0.5, dtype=np.float32)
    ground_x, ground_y = np.meshgrid(axis_x, axis_y, indexing="ij")
    ground = np.stack(
        (ground_x.ravel(), ground_y.ravel(), np.zeros(ground_x.size, dtype=np.float32)), axis=1
    )
    wall_y, wall_z = np.meshgrid(
        axis_y, np.arange(0.0, 4.01, 0.5, dtype=np.float32), indexing="ij"
    )
    walls = []
    for x in (-6.0, 6.0):
        walls.append(
            np.stack(
                (np.full(wall_y.size, x, dtype=np.float32), wall_y.ravel(), wall_z.ravel()),
                axis=1,
            )
        )
    return np.concatenate((ground, *walls), axis=0).astype(np.float32)


def _route_vertices(start: np.ndarray, lateral_sign: float) -> np.ndarray:
    distance = np.linspace(0.0, 10.0, 21, dtype=np.float32)
    lateral = lateral_sign * 1.5 * np.sin(np.pi * distance / 10.0)
    return np.stack(
        (start[0] - lateral, start[1] + distance, np.full_like(distance, start[2])), axis=1
    ).astype(np.float32)


def generate_synthetic_dataset(
    output: Path,
    *,
    scene_count: int = 2,
    frames_per_scene: int = 4,
    overwrite: bool = False,
) -> Path:
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        if not overwrite:
            raise FileExistsError(f"output directory is not empty: {output}")
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)
    obstacles_enu = _obstacle_cloud_enu()
    obstacles_ned = enu_to_ned(obstacles_enu)
    quality_gate = RouteQualityGate(RouteQualityConfig(maximum_curvature_rad_m=2.0))

    for scene_index in range(scene_count):
        scene = output / f"Scene_{scene_index:04d}"
        source_ply = output / f".scene_{scene_index:04d}_ned.ply"
        write_point_cloud_ply(source_ply, obstacles_ned)
        writer = SceneWriter(scene, CaptureConfig(settle_time_s=0.0))
        starts_enu: list[np.ndarray] = []
        for frame_index in range(frames_per_scene):
            north = float(frame_index) * 0.4
            east = float(scene_index) * 0.3
            pose = PoseSample((north, east, -1.6), (1.0, 0.0, 0.0, 0.0))
            start_enu = np.array([east, north, 1.6], dtype=np.float32)
            starts_enu.append(start_enu)
            rgb = np.full((96, 160, 3), 64 + scene_index * 16, dtype=np.uint8)
            depth = np.full((96, 160), 10.0, dtype=np.float32)
            writer.write_frame(frame_index, rgb, depth, pose, frame_index, "route")
        writer.finalize(source_ply)
        source_ply.unlink(missing_ok=True)

        records: list[RouteRecord] = []
        for frame_index, start in enumerate(starts_enu):
            for route_variant, lateral_sign in enumerate((0.0, -1.0, 1.0)):
                vertices = _route_vertices(start, lateral_sign)
                points, clearance, radius = build_witness_corridor(
                    vertices,
                    obstacles_enu,
                    robot_radius_m=0.3,
                    safety_margin_m=0.2,
                    max_step_m=0.2,
                )
                frontier = points[-1].copy()
                _, local_subgoal_distance = local_subgoal_on_witness(points, 10.0)
                result = quality_gate.evaluate(
                    path_points_world=points,
                    path_clearance_m=clearance,
                    path_bubble_radius_m=radius,
                    start_world=start,
                    frontier_world=frontier,
                    start_rotation_world_body=np.array(
                        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
                        dtype=np.float32,
                    ),
                )
                if not result.valid:
                    raise RuntimeError(f"synthetic route failed quality gate: {result.flags}")
                topo_indices = np.linspace(0, len(points) - 1, 5, dtype=np.int64)
                records.append(
                    RouteRecord(
                        frame_index=frame_index,
                        mission_goal_world=frontier + np.array([0.0, 5.0, 0.0], dtype=np.float32),
                        frontier_goal_world=frontier,
                        path_points_world=points,
                        path_clearance_m=clearance,
                        path_bubble_radius_m=radius,
                        topo_centers_world=points[topo_indices],
                        topo_bubble_radius_m=radius[topo_indices],
                        topo_persistent_id=np.arange(5, dtype=np.uint64)
                        + np.uint64(scene_index * 1000 + frame_index * 10),
                        route_valid=True,
                        route_quality_flags=int(result.flags),
                        route_quality_weight=result.weight,
                        route_seed=scene_index * 10000 + frame_index * 10 + route_variant,
                        local_subgoal_distance_m=local_subgoal_distance,
                    )
                )
        save_route_table(scene / "routes.npz", pack_route_records(records))
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a small route-conditioned YOPO dataset")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenes", type=int, default=2)
    parser.add_argument("--frames", type=int, default=4)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    generated = generate_synthetic_dataset(
        args.output,
        scene_count=args.scenes,
        frames_per_scene=args.frames,
        overwrite=args.overwrite,
    )
    print(generated)


if __name__ == "__main__":
    main()
