#!/usr/bin/env python3
"""Export a surface-sampled Map2 truth cloud from the running UE scene.

The Colosseum mesh API exposes indexed static-mesh triangles in UE world
coordinates (centimetres).  Sampling triangle interiors is essential here:
exporting vertices alone reduces large walls and blocks to sparse corners.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import colosseum


# Include the tall Map2 landmark (its UE mesh reaches about 35 m).  The
# enormous extent check below removes the world-sized floor/sky mesh.
CORRIDOR_MIN = np.array([-75.0, -12.0, -20.0])
CORRIDOR_MAX = np.array([75.0, 145.0, 40.0])
EXCLUDED_NAMES = ("camera", "drone")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--spacing", type=float, default=0.16,
                        help="surface sample spacing in metres")
    parser.add_argument("--voxel", type=float, default=0.12,
                        help="output voxel size in metres")
    return parser.parse_args()


def sample_triangles(triangles: np.ndarray, spacing: float, rng: np.random.Generator) -> np.ndarray:
    side_a = triangles[:, 1] - triangles[:, 0]
    side_b = triangles[:, 2] - triangles[:, 0]
    areas = 0.5 * np.linalg.norm(np.cross(side_a, side_b), axis=1)
    counts = np.maximum(1, np.ceil(areas / spacing**2).astype(np.int64))
    sampled = []
    for triangle, count in zip(triangles, counts):
        # Reflection maps the unit square uniformly onto a triangle.
        uv = rng.random((min(int(count), 60_000), 2))
        reflected = uv.sum(axis=1) > 1.0
        uv[reflected] = 1.0 - uv[reflected]
        sampled.append(
            triangle[0] + uv[:, :1] * (triangle[1] - triangle[0])
            + uv[:, 1:] * (triangle[2] - triangle[0])
        )
    return np.concatenate(sampled) if sampled else np.empty((0, 3))


def voxel_downsample(points: np.ndarray, size: float) -> np.ndarray:
    keys = np.floor(points / size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def main() -> None:
    args = parse_args()
    client = colosseum.VehicleClient(port=args.port, timeout_value=300)
    if not client.ping():
        raise RuntimeError(f"Colosseum did not answer on port {args.port}")

    rng = np.random.default_rng(20260826)
    clouds = []
    accepted_meshes = 0
    for mesh in client.simGetMeshPositionVertexBuffers():
        name = mesh.name.lower()
        if any(token in name for token in EXCLUDED_NAMES):
            continue
        vertices_ue = np.asarray(mesh.vertices, dtype=np.float64)
        indices = np.asarray(mesh.indices, dtype=np.int64)
        if vertices_ue.size % 3 or indices.size < 3:
            continue
        # UE (X forward, Y right, Z up) cm -> logged world_enu (X, Y, Z) m.
        vertices = vertices_ue.reshape(-1, 3)[:, [1, 0, 2]] * 0.01
        extent = vertices.max(axis=0) - vertices.min(axis=0)
        if np.any(extent > 500.0):
            continue
        triangles_i = indices[: indices.size // 3 * 3].reshape(-1, 3)
        valid = (
            np.isfinite(vertices).all(axis=1).all()
            and np.all(triangles_i >= 0)
            and np.all(triangles_i < len(vertices))
        )
        if not valid:
            continue
        triangles = vertices[triangles_i]
        intersects = np.all(triangles.max(axis=1) >= CORRIDOR_MIN, axis=1) & np.all(
            triangles.min(axis=1) <= CORRIDOR_MAX, axis=1
        )
        triangles = triangles[intersects]
        if not len(triangles):
            continue
        sampled = sample_triangles(triangles, args.spacing, rng)
        inside = np.all(sampled >= CORRIDOR_MIN, axis=1) & np.all(
            sampled <= CORRIDOR_MAX, axis=1
        )
        sampled = sampled[inside]
        if len(sampled):
            clouds.append(sampled)
            accepted_meshes += 1

    if not clouds:
        raise RuntimeError("no Map2 mesh surfaces intersect the requested corridor")
    points = voxel_downsample(np.concatenate(clouds), args.voxel)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="ascii") as stream:
        stream.write(
            "ply\nformat ascii 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\nproperty float y\nproperty float z\nend_header\n"
        )
        np.savetxt(stream, points, fmt="%.4f")
    print(f"wrote {len(points):,} surface points from {accepted_meshes} meshes to {args.output}")


if __name__ == "__main__":
    main()
