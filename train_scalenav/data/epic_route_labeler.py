from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.spatial.transform import Rotation

from .route_contract import (
    RouteQualityConfig,
    RouteQualityGate,
    RouteRecord,
    build_witness_corridor,
    pack_route_records,
    save_route_table,
)
from .snapshot_dataset import _load_toml, read_ascii_point_cloud_ply


def _vector(record: dict[str, Any], name: str) -> np.ndarray:
    value = np.asarray(record.get(name, []), dtype=np.float32)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain three finite values")
    return value


def _points(record: dict[str, Any], name: str) -> np.ndarray:
    value = np.asarray(record.get(name, []), dtype=np.float32)
    if value.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    if value.ndim != 2 or value.shape[1] != 3 or not np.isfinite(value).all():
        raise ValueError(f"{name} must be a finite Nx3 array")
    return value


def read_epic_jsonl(path: Path) -> list[dict[str, Any]]:
    records = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON at {path}:{line_number}") from error
            if not isinstance(record, dict):
                raise ValueError(f"route record at {path}:{line_number} is not an object")
            records.append(record)
    return records


def label_epic_routes(
    scene_dir: Path,
    epic_records: Iterable[dict[str, Any]],
    *,
    output_name: str = "routes.npz",
    quality_config: RouteQualityConfig = RouteQualityConfig(),
    clearance_step_m: float = 0.1,
) -> Path:
    """Convert accepted EPIC output into the route training contract.

    Search is deliberately not implemented here. `path_points_world` must be
    the accepted edge-witness polyline emitted by the production EPIC graph.
    """
    scene_dir = Path(scene_dir)
    document = _load_toml(scene_dir / "data.toml")
    if document.get("worldFrame") != "world_enu" or document.get("bodyFrame") != "body_flu":
        raise ValueError("scene must use world_enu/body_flu")
    frames = {int(frame["frameIndex"]): frame for frame in document.get("dataArray", [])}
    obstacles = read_ascii_point_cloud_ply(scene_dir / "tree.ply")
    gate = RouteQualityGate(quality_config)
    output: list[RouteRecord] = []

    for input_index, record in enumerate(epic_records):
        frame_index = int(record.get("frame_index", -1))
        if frame_index not in frames:
            raise ValueError(f"route {input_index} references unknown frame {frame_index}")
        frame = frames[frame_index]
        start = np.asarray(frame["posStart"], dtype=np.float32)
        w, x, y, z = np.asarray(frame["orientationWxyz"], dtype=np.float32)
        rotation = Rotation.from_quat([x, y, z, w]).as_matrix().astype(np.float32)
        mission = _vector(record, "mission_goal_world")
        frontier = _vector(record, "frontier_goal_world")
        found = bool(record.get("found", True))
        blocked = bool(record.get("blocked", False))
        committed = bool(record.get("committed", True))
        witness = _points(record, "path_points_world")

        if len(witness) >= 2 and np.isfinite(witness).all():
            dense, clearance, radii = build_witness_corridor(
                witness,
                obstacles,
                robot_radius_m=quality_config.robot_radius_m,
                safety_margin_m=quality_config.safety_margin_m,
                max_step_m=clearance_step_m,
            )
        else:
            dense = np.empty((0, 3), dtype=np.float32)
            clearance = np.empty((0,), dtype=np.float32)
            radii = np.empty((0,), dtype=np.float32)
        quality = gate.evaluate(
            path_points_world=dense,
            path_clearance_m=clearance,
            path_bubble_radius_m=radii,
            start_world=start,
            frontier_world=frontier,
            start_rotation_world_body=rotation,
            found=found,
            blocked=blocked,
            committed=committed,
            allow_short_terminal=bool(record.get("allow_short_terminal", False)),
        )
        topo_centers = _points(record, "topo_centers_world")
        topo_radii = np.asarray(record.get("topo_bubble_radius_m", []), dtype=np.float32)
        topo_ids = np.asarray(record.get("topo_persistent_id", []), dtype=np.uint64)
        if topo_radii.shape != (len(topo_centers),) or topo_ids.shape != (len(topo_centers),):
            raise ValueError(f"route {input_index} has inconsistent topology arrays")
        output.append(
            RouteRecord(
                frame_index=frame_index,
                mission_goal_world=mission,
                frontier_goal_world=frontier,
                path_points_world=dense,
                path_clearance_m=clearance,
                path_bubble_radius_m=radii,
                topo_centers_world=topo_centers,
                topo_bubble_radius_m=topo_radii,
                topo_persistent_id=topo_ids,
                route_valid=quality.valid,
                route_quality_flags=int(quality.flags),
                route_quality_weight=quality.weight,
                route_seed=int(record.get("route_seed", input_index)),
            )
        )
    if not output:
        raise ValueError("EPIC route input is empty")
    return save_route_table(scene_dir / output_name, pack_route_records(output))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build routes.npz from production EPIC accepted-route JSONL"
    )
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--epic-jsonl", type=Path, required=True)
    parser.add_argument("--output-name", default="routes.npz")
    parser.add_argument("--clearance-step", type=float, default=0.1)
    args = parser.parse_args()
    output = label_epic_routes(
        args.scene,
        read_epic_jsonl(args.epic_jsonl),
        output_name=args.output_name,
        clearance_step_m=args.clearance_step,
    )
    print(output)


if __name__ == "__main__":
    main()
