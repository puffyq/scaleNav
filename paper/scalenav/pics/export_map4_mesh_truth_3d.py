#!/usr/bin/env python3
"""Export Map4's complete runtime render mesh as a 3-D occupancy volume.

The RPC returns world-space render triangles.  Vertices, edge midpoints and
triangle centroids are voxelized and then inflated by the training collector;
no RGB-D observations or flight logs are used for geometry truth.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import colosseum


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("output", type=Path)
    p.add_argument("--port", type=int, default=41452)
    p.add_argument("--resolution", type=float, default=0.5)
    p.add_argument("--x", type=float, nargs=2, default=(-50.0, 50.0))
    p.add_argument("--y", type=float, nargs=2, default=(-10.0, 150.0))
    p.add_argument("--z", type=float, nargs=2, default=(-2.0, 12.0))
    p.add_argument("--triangle-chunk", type=int, default=250_000)
    a = p.parse_args()
    bx, by, bz = tuple(sorted(a.x)), tuple(sorted(a.y)), tuple(sorted(a.z))
    shape = tuple(int(np.ceil((hi - lo) / a.resolution)) for lo, hi in (bx, by, bz))
    occupied = np.zeros(shape, dtype=np.uint8)  # x, y, z
    client = colosseum.VehicleClient(port=a.port, timeout_value=300)
    if not client.ping():
        raise RuntimeError(f"Colosseum did not answer on port {a.port}")
    started = time.monotonic()
    meshes = client.simGetMeshPositionVertexBuffers()
    rpc_seconds = time.monotonic() - started
    mesh_count = len(meshes)
    total_triangles = 0
    contributing = []
    excluded = ("camera", "drone", "sky")
    while meshes:
        mesh = meshes.pop()
        name = mesh.name.lower()
        if any(token in name for token in excluded):
            continue
        vertices = np.asarray(mesh.vertices, dtype=np.float32)
        indices = np.asarray(mesh.indices, dtype=np.int64)
        if vertices.size % 3 or indices.size < 3:
            continue
        vertices = vertices.reshape(-1, 3)
        indices = indices[: indices.size // 3 * 3]
        if indices.min() < 0 or indices.max() >= len(vertices):
            continue
        before = int(occupied.sum())
        count = 0
        for offset in range(0, len(indices), a.triangle_chunk * 3):
            tri = vertices[indices[offset:offset + a.triangle_chunk * 3].reshape(-1, 3)]
            # UE centimetres -> world_enu metres: ENU x=UE Y, y=UE X.
            tri = tri[..., [1, 0, 2]] * 0.01
            lo, hi = tri.min(axis=1), tri.max(axis=1)
            hit = ((hi[:, 0] >= bx[0]) & (lo[:, 0] <= bx[1]) &
                   (hi[:, 1] >= by[0]) & (lo[:, 1] <= by[1]) &
                   (hi[:, 2] >= bz[0]) & (lo[:, 2] <= bz[1]))
            tri = tri[hit]
            if not len(tri):
                continue
            count += len(tri)
            points = np.concatenate((tri, (tri[:, [1, 2, 0]] + tri[:, [2, 0, 1]]) * 0.5,
                                     tri.mean(axis=1, keepdims=True)), axis=1)
            points = points.reshape(-1, 3)
            cells = np.floor((points - np.array([bx[0], by[0], bz[0]])) / a.resolution).astype(np.int64)
            valid = ((cells[:, 0] >= 0) & (cells[:, 0] < shape[0]) &
                     (cells[:, 1] >= 0) & (cells[:, 1] < shape[1]) &
                     (cells[:, 2] >= 0) & (cells[:, 2] < shape[2]))
            cells = cells[valid]
            occupied[cells[:, 0], cells[:, 1], cells[:, 2]] = 1
        total_triangles += count
        new = int(occupied.sum()) - before
        if count:
            contributing.append({"name": mesh.name, "triangles": count, "new_voxels": new})
            print(f"mesh={mesh.name} triangles={count:,} new_voxels={new:,} total={int(occupied.sum()):,}", flush=True)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.output, occupied=occupied, bounds=np.asarray([bx, by, bz], dtype=np.float32), resolution=a.resolution)
    meta = {"source": "Unreal runtime static render-mesh triangles", "uses_camera_observations": False,
            "uses_flight_logs_for_geometry": False, "frame": "world_enu", "bounds_enu_m": [bx, by, bz],
            "resolution_m": a.resolution, "shape_xyz": shape, "rpc_seconds": rpc_seconds,
            "runtime_meshes": mesh_count, "source_triangles": total_triangles,
            "occupied_voxels": int(occupied.sum()), "contributing_meshes": contributing,
            "npz": str(a.output.resolve())}
    a.output.with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(meta, ensure_ascii=True), flush=True)


if __name__ == "__main__":
    main()
