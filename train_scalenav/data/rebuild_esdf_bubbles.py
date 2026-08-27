"""Recompute witness bubble radii from the same voxel ESDF used by SafetyLoss.

The source routes, goals and RGB/depth frames are copied unchanged.  Only
clearance/radius fields and their route summary statistics are regenerated.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates

from config.config import cfg
from data.route_contract import RouteQualityFlag, load_route_table, save_route_table
from data.snapshot_dataset import read_ascii_point_cloud_ply


def _build_esdf(points: np.ndarray, voxel_size: float, expand_min: np.ndarray, expand_max: np.ndarray):
    minimum = points.min(axis=0).astype(np.float64) - expand_min
    maximum = points.max(axis=0).astype(np.float64) + expand_max
    shape = np.ceil((maximum - minimum) / voxel_size).astype(np.int64)
    indices = ((points - minimum) / voxel_size).astype(np.int64)
    valid = np.all((indices >= 0) & (indices < shape), axis=1)
    occupancy = np.zeros(tuple(shape.tolist()), dtype=np.uint8)
    occupancy[tuple(indices[valid].T)] = 1
    obstacle = occupancy == 1
    free = ~obstacle
    esdf = distance_transform_edt(free) * voxel_size
    inside = distance_transform_edt(obstacle) * voxel_size
    esdf[obstacle] = -inside[obstacle]
    return esdf.astype(np.float32), minimum.astype(np.float32)


def _sample(esdf: np.ndarray, origin: np.ndarray, points: np.ndarray, voxel_size: float) -> np.ndarray:
    coordinates = ((np.asarray(points, dtype=np.float32) - origin) / voxel_size).T
    return map_coordinates(esdf, coordinates, order=1, mode="constant", cval=0.0).astype(np.float32)


def _update_summaries(arrays: dict[str, np.ndarray], index: int, clearance: np.ndarray, radii: np.ndarray) -> None:
    start, end = int(arrays["path_offsets"][index]), int(arrays["path_offsets"][index + 1])
    points = arrays["path_points_world"][start:end]
    arrays["path_clearance_m"][start:end] = clearance
    arrays["path_bubble_radius_m"][start:end] = radii
    arrays["route_min_clearance_m"][index] = float(np.min(clearance))
    arrays["route_min_safe_radius_m"][index] = float(np.min(radii))
    arrays["route_safe_radius_p05_m"][index] = float(np.percentile(radii, 5))
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arrays["route_neck_length_m"][index] = float(np.sum(distances[np.minimum(radii[:-1], radii[1:]) < 1.2]))
    continuous = np.minimum(clearance[:-1], clearance[1:]) - 0.5 * distances
    arrays["route_continuous_min_clearance_m"][index] = float(min(np.min(clearance), np.min(continuous)))
    arrays["route_bubble_overlap_margin_m"][index] = float(np.min(radii[:-1] + radii[1:] - distances))


def rebuild(source: Path, output: Path, *, overwrite: bool = False) -> dict[str, object]:
    source = source.resolve()
    output = output.resolve()
    if output.exists():
        if not overwrite:
            raise FileExistsError(f"output exists: {output}; pass --overwrite")
        shutil.rmtree(output)
    shutil.copytree(source, output)
    voxel_size = 0.2
    expand_min = np.asarray(cfg["map_expand_min"], dtype=np.float32)
    expand_max = np.asarray(cfg["map_expand_max"], dtype=np.float32)
    required = float(cfg["robot_radius_m"] + cfg["safety_margin_m"])
    report: dict[str, object] = {"source": str(source), "voxelSizeM": voxel_size, "requiredClearanceM": required, "scenes": []}
    for scene_dir in sorted(path for path in output.glob("Scene_*") if path.is_dir()):
        points = read_ascii_point_cloud_ply(scene_dir / "tree.ply")
        esdf, origin = _build_esdf(points, voxel_size, expand_min, expand_max)
        table = load_route_table(scene_dir / "routes.npz")
        arrays = {name: np.array(value, copy=True) for name, value in table.arrays.items()}
        invalid = 0
        all_clearance: list[np.ndarray] = []
        for index in range(len(table)):
            start, end = int(arrays["path_offsets"][index]), int(arrays["path_offsets"][index + 1])
            path = arrays["path_points_world"][start:end]
            if len(path) < 2:
                continue
            clearance = _sample(esdf, origin, path, voxel_size)
            radii = clearance - required
            _update_summaries(arrays, index, clearance, radii)
            all_clearance.append(clearance)
            if bool(arrays["route_valid"][index]) and (np.any(clearance <= 0.0) or np.any(radii <= 0.0)):
                arrays["route_valid"][index] = 0
                arrays["route_quality_flags"][index] |= int(RouteQualityFlag.CLEARANCE)
                arrays["route_quality_weight"][index] = 0.0
                invalid += 1
            topo_start, topo_end = int(arrays["topo_offsets"][index]), int(arrays["topo_offsets"][index + 1])
            if topo_end > topo_start:
                topo_clearance = _sample(esdf, origin, arrays["topo_centers_world"][topo_start:topo_end], voxel_size)
                arrays["topo_bubble_radius_m"][topo_start:topo_end] = topo_clearance - required
        updated = type(table)(arrays)
        save_route_table(scene_dir / "routes.npz", updated)
        report["scenes"].append({
            "scene": scene_dir.name,
            "esdfShapeXYZ": list(esdf.shape),
            "routeCount": len(table),
            "invalidatedRoutes": invalid,
            "clearanceMinM": float(min(np.min(x) for x in all_clearance)) if all_clearance else None,
            "clearanceMeanM": float(np.mean(np.concatenate(all_clearance))) if all_clearance else None,
        })
    (output / "esdf_bubble_alignment.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    print(json.dumps(rebuild(args.source, args.output, overwrite=args.overwrite), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
