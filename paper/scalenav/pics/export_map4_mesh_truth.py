#!/usr/bin/env python3
"""Export Map4 obstacle truth from Unreal runtime render triangles.

This exporter reads the complete runtime static-mesh scene through the
Colosseum mesh RPC.  It clips every triangle against the UAV navigation-height
slab, projects the clipped polygons to the horizontal grid, and writes the
occupied cells as an ASCII PLY.  No camera observations or flight logs are
used to construct the map.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

import colosseum


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--x", type=float, nargs=2, default=(-50.0, 50.0))
    parser.add_argument("--y", type=float, nargs=2, default=(-10.0, 150.0))
    parser.add_argument("--z", type=float, nargs=2, default=(0.6, 2.6))
    parser.add_argument("--triangle-chunk", type=int, default=500_000)
    return parser.parse_args()


def clip_z(polygon: np.ndarray, plane: float, keep_above: bool) -> np.ndarray:
    """Clip a 3-D convex polygon by one horizontal half-space."""
    if len(polygon) == 0:
        return polygon
    output = []
    previous = polygon[-1]
    previous_inside = previous[2] >= plane if keep_above else previous[2] <= plane
    for current in polygon:
        current_inside = current[2] >= plane if keep_above else current[2] <= plane
        if current_inside != previous_inside:
            dz = current[2] - previous[2]
            if abs(float(dz)) > 1e-12:
                alpha = (plane - previous[2]) / dz
                output.append(previous + alpha * (current - previous))
        if current_inside:
            output.append(current)
        previous = current
        previous_inside = current_inside
    return np.asarray(output, dtype=np.float32).reshape(-1, 3)


def rasterize_triangles(
    occupancy: np.ndarray,
    triangles: np.ndarray,
    bounds_x: tuple[float, float],
    bounds_y: tuple[float, float],
    bounds_z: tuple[float, float],
    resolution: float,
) -> None:
    height, width = occupancy.shape
    for triangle in triangles:
        polygon = clip_z(triangle, bounds_z[0], True)
        polygon = clip_z(polygon, bounds_z[1], False)
        if len(polygon) < 2:
            continue
        pixels = np.empty((len(polygon), 2), dtype=np.int32)
        pixels[:, 0] = np.rint((polygon[:, 0] - bounds_x[0]) / resolution).astype(np.int32)
        pixels[:, 1] = np.rint((polygon[:, 1] - bounds_y[0]) / resolution).astype(np.int32)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, width - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, height - 1)
        cv2.fillConvexPoly(occupancy, pixels, 1, lineType=cv2.LINE_8)


def main() -> None:
    args = parse_args()
    if args.resolution <= 0.0 or args.triangle_chunk <= 0:
        raise ValueError("resolution and triangle chunk must be positive")
    bounds_x = tuple(sorted(args.x))
    bounds_y = tuple(sorted(args.y))
    bounds_z = tuple(sorted(args.z))
    width = int(np.ceil((bounds_x[1] - bounds_x[0]) / args.resolution))
    height = int(np.ceil((bounds_y[1] - bounds_y[0]) / args.resolution))
    occupancy = np.zeros((height, width), dtype=np.uint8)

    client = colosseum.VehicleClient(port=args.port, timeout_value=300)
    if not client.ping():
        raise RuntimeError(f"Colosseum did not answer on port {args.port}")
    started = time.monotonic()
    meshes = client.simGetMeshPositionVertexBuffers()
    rpc_seconds = time.monotonic() - started

    mesh_count = len(meshes)
    valid_meshes = 0
    rejected_meshes = 0
    source_vertices = 0
    source_triangles = 0
    candidate_triangles = 0
    contributing_meshes = []
    excluded_tokens = ("camera", "drone")

    # Pop responses so the very large expanded foliage arrays can be released
    # after each component is rasterized.
    while meshes:
        mesh = meshes.pop()
        name = mesh.name.lower()
        if any(token in name for token in excluded_tokens):
            continue
        vertices_ue = np.asarray(mesh.vertices, dtype=np.float32)
        indices = np.asarray(mesh.indices, dtype=np.int64)
        if vertices_ue.size % 3 or indices.size < 3:
            rejected_meshes += 1
            continue
        vertices_ue = vertices_ue.reshape(-1, 3)
        indices = indices[: indices.size // 3 * 3]
        if (not np.isfinite(vertices_ue).all() or indices.min() < 0
                or indices.max() >= len(vertices_ue)):
            rejected_meshes += 1
            continue
        valid_meshes += 1
        source_vertices += len(vertices_ue)
        source_triangles += len(indices) // 3
        mesh_candidates = 0
        occupied_before = int(occupancy.sum())
        chunk_indices = args.triangle_chunk * 3
        for offset in range(0, len(indices), chunk_indices):
            triangle_indices = indices[offset: offset + chunk_indices].reshape(-1, 3)
            # UE X/Y/Z centimetres -> logged world_enu Y/X/Z metres.
            triangles_ue = vertices_ue[triangle_indices]
            triangles = triangles_ue[..., [1, 0, 2]] * 0.01
            minimum = triangles.min(axis=1)
            maximum = triangles.max(axis=1)
            intersects = (
                (maximum[:, 0] >= bounds_x[0]) & (minimum[:, 0] <= bounds_x[1])
                & (maximum[:, 1] >= bounds_y[0]) & (minimum[:, 1] <= bounds_y[1])
                & (maximum[:, 2] >= bounds_z[0]) & (minimum[:, 2] <= bounds_z[1])
            )
            selected = triangles[intersects]
            mesh_candidates += len(selected)
            rasterize_triangles(
                occupancy, selected, bounds_x, bounds_y, bounds_z, args.resolution
            )
        candidate_triangles += mesh_candidates
        new_cells = int(occupancy.sum()) - occupied_before
        if mesh_candidates:
            contributing_meshes.append({
                "name": mesh.name,
                "slab_candidate_triangles": mesh_candidates,
                "new_occupied_cells": new_cells,
            })
        print(
            f"mesh={valid_meshes}/{mesh_count} name={mesh.name} "
            f"candidates={mesh_candidates:,} new_cells={new_cells:,} "
            f"occupied={int(occupancy.sum()):,}",
            flush=True,
        )

    rows, columns = np.nonzero(occupancy)
    points = np.column_stack((
        bounds_x[0] + (columns + 0.5) * args.resolution,
        bounds_y[0] + (rows + 0.5) * args.resolution,
        np.full(len(rows), 0.5 * (bounds_z[0] + bounds_z[1])),
    ))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="ascii") as stream:
        stream.write(
            "ply\nformat ascii 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\nend_header\n"
        )
        np.savetxt(stream, points, fmt="%.4f")

    metadata = {
        "source": "Unreal runtime static render-mesh triangle ground truth",
        "uses_camera_observations": False,
        "uses_flight_logs_for_geometry": False,
        "frame": "world_enu",
        "coordinate_conversion": "x=UE_Y/100, y=UE_X/100, z=UE_Z/100",
        "bounds_enu_m": {"x": bounds_x, "y": bounds_y, "z": bounds_z},
        "resolution_m": args.resolution,
        "grid_dimensions_xy": [width, height],
        "rpc_seconds": rpc_seconds,
        "runtime_meshes": mesh_count,
        "valid_meshes": valid_meshes,
        "rejected_meshes": rejected_meshes,
        "source_vertices": source_vertices,
        "source_triangles": source_triangles,
        "slab_candidate_triangles": candidate_triangles,
        "occupied_cells": len(points),
        "contributing_meshes": contributing_meshes,
        "ply": str(args.output.resolve()),
    }
    args.output.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, ensure_ascii=True), flush=True)


if __name__ == "__main__":
    main()
