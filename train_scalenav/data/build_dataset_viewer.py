from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path
from typing import Any, Mapping

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import cv2
import numpy as np

from .route_contract import RouteTable, load_route_table
from .snapshot_dataset import _load_toml, read_ascii_point_cloud_ply


VIEWER_VERSION = 1


def _json_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def _sample_indices(count: int, maximum: int) -> np.ndarray:
    if count <= maximum:
        return np.arange(count, dtype=np.int64)
    return np.unique(np.linspace(0, count - 1, maximum, dtype=np.int64))


def _route_payload(
    table: RouteTable,
    route_index: int,
    prediction: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    path, clearance, radii = table.path(route_index)
    topo, topo_radii, topo_ids = table.topology(route_index)
    path_indices = _sample_indices(len(path), 96)
    topo_indices = _sample_indices(len(topo), 24)
    arrays = table.arrays
    payload = {
        "routeIndex": route_index,
        "path": path[path_indices, :2].round(4).tolist(),
        "radii": radii[path_indices].round(4).tolist(),
        "clearance": clearance[path_indices].round(4).tolist(),
        "topology": topo[topo_indices, :2].round(4).tolist(),
        "topologyRadii": topo_radii[topo_indices].round(4).tolist(),
        "topologyIds": [int(value) for value in topo_ids[topo_indices]],
        "missionGoal": arrays["mission_goal_world"][route_index].round(4).tolist(),
        "frontierGoal": arrays["frontier_goal_world"][route_index].round(4).tolist(),
        "localSubgoal": arrays.get(
            "local_subgoal_world", arrays["frontier_goal_world"]
        )[route_index].round(4).tolist(),
        "lengthM": round(float(arrays["path_length_m"][route_index]), 4),
        "minimumClearanceM": round(float(arrays["route_min_clearance_m"][route_index]), 4),
        "maximumCurvature": round(float(arrays["route_max_curvature"][route_index]), 4),
        "qualityWeight": round(float(arrays["route_quality_weight"][route_index]), 4),
        "qualityFlags": int(arrays["route_quality_flags"][route_index]),
        "valid": bool(arrays["route_valid"][route_index]),
        "seed": int(arrays["route_seed"][route_index]),
    }
    optional_metrics = {
        "minimumSafeRadiusM": "route_min_safe_radius_m",
        "safeRadiusP05M": "route_safe_radius_p05_m",
        "neckLengthM": "route_neck_length_m",
        "continuousMinimumClearanceM": "route_continuous_min_clearance_m",
        "bubbleOverlapMarginM": "route_bubble_overlap_margin_m",
        "searchDetourRatio": "route_search_detour_ratio",
        "centerlineGainM": "route_centerline_gain_m",
    }
    for output_name, array_name in optional_metrics.items():
        if array_name in arrays:
            payload[output_name] = round(float(arrays[array_name][route_index]), 4)
    if prediction is not None:
        payload["prediction"] = _json_value(dict(prediction))
    return payload


def _depth_preview(source: Path, destination: Path, maximum_m: float) -> None:
    depth = cv2.imread(str(source), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    if depth is None:
        raise ValueError(f"unable to read depth image: {source}")
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    depth = np.nan_to_num(depth.astype(np.float32), nan=maximum_m, posinf=maximum_m, neginf=0.0)
    normalized = np.clip(depth / max(maximum_m, 1.0e-6), 0.0, 1.0)
    preview = cv2.applyColorMap(
        np.round((1.0 - normalized) * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(destination), preview):
        raise IOError(f"failed to write depth preview: {destination}")


def _map_preview(
    points: np.ndarray, destination: Path, *, size: int = 900
) -> tuple[list[float], int]:
    points = np.asarray(points, dtype=np.float32)
    above_ground = points[points[:, 2] > 0.25]
    projected = above_ground if len(above_ground) else points
    x_min, y_min = np.min(projected[:, :2], axis=0)
    x_max, y_max = np.max(projected[:, :2], axis=0)
    padding = max(1.0, 0.025 * max(float(x_max - x_min), float(y_max - y_min)))
    bounds = [float(x_min - padding), float(x_max + padding),
              float(y_min - padding), float(y_max + padding)]
    image = np.full((size, size, 3), 242, dtype=np.uint8)
    px = np.clip(
        np.round((projected[:, 0] - bounds[0]) / (bounds[1] - bounds[0]) * (size - 1)),
        0,
        size - 1,
    ).astype(np.int32)
    py = np.clip(
        np.round((bounds[3] - projected[:, 1]) / (bounds[3] - bounds[2]) * (size - 1)),
        0,
        size - 1,
    ).astype(np.int32)
    image[py, px] = (37, 40, 43)
    image = cv2.erode(image, np.ones((2, 2), dtype=np.uint8), iterations=1)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(destination), image):
        raise IOError(f"failed to write map preview: {destination}")
    return bounds, len(projected)


def build_dataset_viewer(
    data_root: Path,
    *,
    output_dir: Path | None = None,
    overwrite: bool = False,
    predictions: Mapping[tuple[str, int], Mapping[str, Any]] | None = None,
    evaluation_report: Mapping[str, Any] | None = None,
) -> Path:
    data_root = Path(data_root).resolve()
    output = Path(output_dir).resolve() if output_dir else data_root / "viewer"
    if output.exists() and any(output.iterdir()):
        if not overwrite:
            raise FileExistsError(f"viewer output is not empty: {output}")
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)

    report_path = data_root / "generation_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else {}
    report.setdefault(
        "dataset_role", "offline_test" if data_root.name.startswith("test_") else "train"
    )
    scenes_payload: list[dict[str, Any]] = []
    route_count = 0
    frame_count = 0
    for scene_dir in sorted(path for path in data_root.glob("Scene_*") if path.is_dir()):
        document = _load_toml(scene_dir / "data.toml")
        records = document.get("dataArray", [])
        table = load_route_table(scene_dir / "routes.npz", frame_count=len(records))
        routes_by_frame: dict[int, list[int]] = {}
        for route_index, frame_index in enumerate(table.arrays["frame_index"]):
            routes_by_frame.setdefault(int(frame_index), []).append(route_index)

        scene_asset = output / "assets" / scene_dir.name
        map_name = f"assets/{scene_dir.name}/map_topdown.png"
        bounds, map_point_count = _map_preview(
            read_ascii_point_cloud_ply(scene_dir / "tree.ply"), output / map_name
        )
        frames: list[dict[str, Any]] = []
        for record in records:
            frame_index = int(record["frameIndex"])
            depth_name = f"depth_{frame_index:06d}.png"
            _depth_preview(
                scene_dir / "Textures" / str(record["depthFileName"]),
                scene_asset / depth_name,
                float(record.get("depthMaxMeters", document.get("depthMaxMeters", 20.0))),
            )
            routes = [
                _route_payload(
                    table,
                    route_index,
                    None if predictions is None else predictions.get((scene_dir.name, route_index)),
                )
                for route_index in routes_by_frame.get(frame_index, [])
            ]
            frames.append(
                {
                    "frameIndex": frame_index,
                    "rgb": Path(
                        os.path.relpath(
                            scene_dir / "Textures" / str(record["rgbFileName"]),
                            output,
                        )
                    ).as_posix(),
                    "depth": f"assets/{scene_dir.name}/{depth_name}",
                    "position": [round(float(value), 4) for value in record["posStart"]],
                    "yawDeg": round(float(record.get("yawStart", 0.0)), 3),
                    "routes": routes,
                }
            )
            route_count += len(routes)
            frame_count += 1
        scenes_payload.append(
            {
                "name": scene_dir.name,
                "frameCount": len(frames),
                "routeCount": len(table),
                "bounds": [round(value, 4) for value in bounds],
                "mapImage": map_name,
                "mapPointCount": map_point_count,
                "frames": frames,
            }
        )

    if not scenes_payload:
        raise FileNotFoundError(f"no complete Scene_* directories under {data_root}")
    payload = {
        "viewerVersion": VIEWER_VERSION,
        "datasetName": data_root.name,
        "datasetPath": str(data_root),
        "sceneCount": len(scenes_payload),
        "frameCount": frame_count,
        "routeCount": route_count,
        "generationReport": _json_value(report),
        "evaluationReport": _json_value(dict(evaluation_report or {})),
        "scenes": scenes_payload,
    }
    data_text = "window.SCALENAV_DATASET = " + json.dumps(
        payload, ensure_ascii=True, separators=(",", ":"), allow_nan=False
    ) + ";\n"
    (output / "dataset.js").write_text(data_text, encoding="utf-8")
    template = Path(__file__).resolve().parent.parent / "tools" / "dataset_viewer.html"
    shutil.copyfile(template, output / "index.html")
    (output / "viewer_report.json").write_text(
        json.dumps(
            {
                "viewerVersion": VIEWER_VERSION,
                "dataset": str(data_root),
                "scenes": len(scenes_payload),
                "frames": frame_count,
                "routes": route_count,
                "predictions": 0 if predictions is None else len(predictions),
                "evaluation": _json_value(dict(evaluation_report or {})),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return output / "index.html"


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a static ScaleNav route dataset viewer")
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    print(build_dataset_viewer(args.data_root, output_dir=args.output, overwrite=args.overwrite))


if __name__ == "__main__":
    main()
