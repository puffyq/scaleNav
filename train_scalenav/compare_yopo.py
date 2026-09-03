"""Paired offline benchmark for current/previous Route-YOPO and YOPO-Simple.

All policies consume the same depth, pose, motion state and frontier goal.
Route-YOPO policies additionally consume the witness bubbles stored in each
route record. Outputs and aggregate metrics are written together so that every
comparison remains auditable.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import torch
from scipy.spatial import cKDTree
from torch.utils.data import DataLoader

from config.config import cfg
from data.build_dataset_viewer import build_dataset_viewer
from data.snapshot_dataset import read_ascii_point_cloud_ply
from policy.state_transform import rotate_body2world, state_body2world
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork
from policy.yopo_simple_baseline import YopoSimpleBaseline
from evaluate_yopo import (
    _centerline_metrics,
    _corridor_metrics,
    _guide_angle_rad,
    _sample_trajectory,
)


def _load_baseline(path: Path, device: torch.device) -> YopoSimpleBaseline:
    model = YopoSimpleBaseline().to(device).eval()
    state = torch.load(path, map_location=device, weights_only=False)
    if isinstance(state, dict) and "model_state_dict" in state:
        state = state["model_state_dict"]
    # The upstream checkpoint stores learnable network tensors only; lattice
    # angles/rotations are deterministic runtime buffers in this adapter.
    model.load_state_dict(state, strict=False)
    return model


def _single_metrics(
    trajectory: np.ndarray,
    route_path: np.ndarray,
    route_radii: np.ndarray,
    obstacle_tree: cKDTree,
) -> dict[str, Any]:
    clearance = obstacle_tree.query(trajectory, k=1)[0]
    maximum_violation, mean_violation, progress = _corridor_metrics(
        trajectory, route_path, route_radii
    )
    mean_centerline, maximum_centerline = _centerline_metrics(trajectory, route_path)
    minimum_clearance = float(np.min(clearance))
    trajectory_length = float(np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum())
    endpoint_distance = float(np.linalg.norm(trajectory[-1] - trajectory[0]))
    guide_angle = _guide_angle_rad(trajectory, route_path)
    return {
        "path": trajectory.round(4).tolist(),
        "minimumClearanceM": round(minimum_clearance, 4),
        "collision": bool(minimum_clearance < float(cfg["robot_radius_m"]) + float(cfg["safety_margin_m"])),
        "maximumCorridorViolationM": round(maximum_violation, 4),
        "meanCorridorViolationM": round(mean_violation, 4),
        "meanCenterlineDistanceM": round(mean_centerline, 4),
        "maximumCenterlineDistanceM": round(maximum_centerline, 4),
        "routeProgressM": round(progress, 4),
        "trajectoryLengthM": round(trajectory_length, 4),
        "endpointDistanceM": round(endpoint_distance, 4),
        "averageSpeedMps": round(trajectory_length / float(cfg["sgm_time"]), 4),
        "guideAngleRad": round(guide_angle, 6),
        "guideAngleDeg": round(math.degrees(guide_angle), 4),
    }


def _aggregate(values: list[dict[str, Any]]) -> dict[str, Any]:
    if not values:
        raise ValueError("cannot aggregate an empty comparison")
    return {
        "sampleCount": len(values),
        "collisionRate": float(np.mean([item["collision"] for item in values])),
        "collisionCount": int(sum(bool(item["collision"]) for item in values)),
        "minimumClearanceM": float(min(item["minimumClearanceM"] for item in values)),
        "meanMinimumClearanceM": float(np.mean([item["minimumClearanceM"] for item in values])),
        "meanMaximumCorridorViolationM": float(
            np.mean([item["maximumCorridorViolationM"] for item in values])
        ),
        "meanCenterlineDistanceM": float(
            np.mean([item["meanCenterlineDistanceM"] for item in values])
        ),
        "meanMaximumCenterlineDistanceM": float(
            np.mean([item["maximumCenterlineDistanceM"] for item in values])
        ),
        "corridorViolationRate": float(
            np.mean([item["maximumCorridorViolationM"] > 0.0 for item in values])
        ),
        "meanRouteProgressM": float(np.mean([item["routeProgressM"] for item in values])),
        "meanTrajectoryLengthM": float(
            np.mean([item["trajectoryLengthM"] for item in values])
        ),
        "meanEndpointDistanceM": float(
            np.mean([item["endpointDistanceM"] for item in values])
        ),
        "meanAverageSpeedMps": float(
            np.mean([item["averageSpeedMps"] for item in values])
        ),
        "progressP05M": float(np.percentile([item["routeProgressM"] for item in values], 5)),
        "progressMedianM": float(np.percentile([item["routeProgressM"] for item in values], 50)),
        "meanGuideAngleRad": float(np.mean([item["guideAngleRad"] for item in values])),
        "meanGuideAngleDeg": float(np.mean([item["guideAngleDeg"] for item in values])),
        "guideAngleP95Deg": float(np.percentile([item["guideAngleDeg"] for item in values], 95)),
        "guideAngleOver90Rate": float(np.mean([item["guideAngleDeg"] > 90.0 for item in values])),
    }


def evaluate_comparison(
    data_root: Path,
    route_checkpoint: Path,
    baseline_checkpoint: Path,
    output_dir: Path,
    *,
    previous_route_checkpoint: Path | None = None,
    batch_size: int = 32,
    workers: int = 0,
    device: str | None = None,
    max_samples: int | None = None,
    split: str = "all",
    build_viewer: bool = True,
    safety_shield: bool = False,
) -> Path:
    data_root = Path(data_root).resolve()
    output_dir = Path(output_dir).resolve()
    selected_device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    dataset = YOPODataset(split, data_root=data_root)
    samples = dataset.samples if max_samples is None else dataset.samples[:max_samples]
    dataset.samples = list(samples)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=workers)

    route_policy = YopoNetwork().to(selected_device).eval()
    route_state = torch.load(route_checkpoint, map_location=selected_device, weights_only=False)
    route_feature_order = route_policy.load_route_checkpoint(route_state)
    baseline = _load_baseline(baseline_checkpoint, selected_device)
    previous_route = None
    previous_route_state = None
    if previous_route_checkpoint is not None:
        previous_route = YopoNetwork().to(selected_device).eval()
        previous_route_state = torch.load(
            previous_route_checkpoint, map_location=selected_device, weights_only=False
        )
        previous_feature_order = previous_route.load_route_checkpoint(previous_route_state)
    else:
        previous_feature_order = None
    obstacle_trees = [cKDTree(read_ascii_point_cloud_ply(scene.path / "tree.ply")) for scene in dataset.scenes]

    predictions: dict[tuple[str, int], dict[str, Any]] = {}
    route_values: list[dict[str, Any]] = []
    previous_values: list[dict[str, Any]] = []
    baseline_values: list[dict[str, Any]] = []
    by_scene: dict[str, dict[str, list[dict[str, Any]]]] = {}
    cursor = 0
    with torch.inference_mode():
        for batch in loader:
            count = batch["depth"].shape[0]
            depth = batch["depth"].to(selected_device)
            motion = batch["motion_body"].to(selected_device)
            frontier = batch["frontier_body"].to(selected_device)
            route_end, route_score = route_policy(
                depth,
                motion,
                frontier,
                batch["route_bubbles"].to(selected_device),
            )
            if previous_route is not None:
                previous_end, previous_score = previous_route(
                    depth,
                    motion,
                    frontier,
                    batch["route_bubbles"].to(selected_device),
                )
            simple_end, simple_score = baseline(depth, motion, frontier)
            route_flat = route_end.permute(0, 2, 3, 1).reshape(count, -1, 9)
            simple_flat = simple_end.permute(0, 2, 3, 1).reshape(count, -1, 9)
            previous_flat = (
                previous_end.permute(0, 2, 3, 1).reshape(count, -1, 9)
                if previous_route is not None
                else None
            )
            route_scores = route_score.reshape(count, -1)
            previous_scores = (
                previous_score.reshape(count, -1) if previous_route is not None else None
            )
            simple_scores = simple_score.reshape(count, -1)
            route_index = route_scores.argmin(dim=1)
            previous_index = (
                previous_scores.argmin(dim=1) if previous_scores is not None else None
            )
            row = torch.arange(count, device=selected_device)
            position = batch["position_world"].to(selected_device)
            rotation = batch["rotation_world_body"].to(selected_device)
            start_velocity = rotate_body2world(rotation, motion[:, :3])
            start_acceleration = rotate_body2world(rotation, motion[:, 3:])
            starts = torch.stack((position, start_velocity, start_acceleration), dim=1).cpu().numpy()
            simple_index = simple_scores.argmin(dim=1)
            if safety_shield:
                # Match the online controller: reject candidates whose sampled
                # polynomial intersects the point-cloud safety radius before
                # using the learned score for selection.
                route_all_pos, route_all_vel, route_all_acc = state_body2world(
                    position.repeat_interleave(route_flat.shape[1], dim=0),
                    rotation.repeat_interleave(route_flat.shape[1], dim=0),
                    route_flat.reshape(-1, 9)[:, :3],
                    route_flat.reshape(-1, 9)[:, 3:6],
                    route_flat.reshape(-1, 9)[:, 6:9],
                )
                simple_all_pos, simple_all_vel, simple_all_acc = state_body2world(
                    position.repeat_interleave(simple_flat.shape[1], dim=0),
                    rotation.repeat_interleave(simple_flat.shape[1], dim=0),
                    simple_flat.reshape(-1, 9)[:, :3],
                    simple_flat.reshape(-1, 9)[:, 3:6],
                    simple_flat.reshape(-1, 9)[:, 6:9],
                )
                route_all = torch.stack((route_all_pos, route_all_vel, route_all_acc), dim=1).cpu().numpy().reshape(count, -1, 3, 3)
                simple_all = torch.stack((simple_all_pos, simple_all_vel, simple_all_acc), dim=1).cpu().numpy().reshape(count, -1, 3, 3)
                route_safe = np.zeros((count, route_all.shape[1]), dtype=bool)
                simple_safe = np.zeros((count, simple_all.shape[1]), dtype=bool)
                for local in range(count):
                    scene_index = samples[cursor + local][0]
                    for candidate in range(route_all.shape[1]):
                        route_safe[local, candidate] = obstacle_trees[scene_index].query(
                            _sample_trajectory(starts[local], route_all[local, candidate]), k=1
                        )[0].min() >= float(cfg["robot_radius_m"] + cfg["safety_margin_m"])
                        simple_safe[local, candidate] = obstacle_trees[scene_index].query(
                            _sample_trajectory(starts[local], simple_all[local, candidate]), k=1
                        )[0].min() >= float(cfg["robot_radius_m"] + cfg["safety_margin_m"])
                route_safe_t = torch.from_numpy(route_safe).to(selected_device)
                simple_safe_t = torch.from_numpy(simple_safe).to(selected_device)
                route_index = route_scores.masked_fill(~route_safe_t, float("inf")).argmin(dim=1)
                simple_index = simple_scores.masked_fill(~simple_safe_t, float("inf")).argmin(dim=1)
            route_body = route_flat[row, route_index]
            previous_body = (
                previous_flat[row, previous_index]
                if previous_flat is not None and previous_index is not None
                else None
            )
            simple_body = simple_flat[row, simple_index]
            route_end_pos, route_end_vel, route_end_acc = state_body2world(
                position, rotation, route_body[:, :3], route_body[:, 3:6], route_body[:, 6:9]
            )
            if previous_body is not None:
                previous_end_pos, previous_end_vel, previous_end_acc = state_body2world(
                    position,
                    rotation,
                    previous_body[:, :3],
                    previous_body[:, 3:6],
                    previous_body[:, 6:9],
                )
            simple_end_pos, simple_end_vel, simple_end_acc = state_body2world(
                position, rotation, simple_body[:, :3], simple_body[:, 3:6], simple_body[:, 6:9]
            )
            route_ends = torch.stack((route_end_pos, route_end_vel, route_end_acc), dim=1).cpu().numpy()
            previous_ends = (
                torch.stack((previous_end_pos, previous_end_vel, previous_end_acc), dim=1)
                .cpu()
                .numpy()
                if previous_body is not None
                else None
            )
            simple_ends = torch.stack((simple_end_pos, simple_end_vel, simple_end_acc), dim=1).cpu().numpy()
            for local in range(count):
                scene_index, route_id = samples[cursor + local]
                scene = dataset.scenes[scene_index]
                path, _, radii = scene.routes.path(route_id)
                route_result = _single_metrics(_sample_trajectory(starts[local], route_ends[local]), path, radii, obstacle_trees[scene_index])
                previous_result = (
                    _single_metrics(
                        _sample_trajectory(starts[local], previous_ends[local]),
                        path,
                        radii,
                        obstacle_trees[scene_index],
                    )
                    if previous_ends is not None
                    else None
                )
                simple_result = _single_metrics(_sample_trajectory(starts[local], simple_ends[local]), path, radii, obstacle_trees[scene_index])
                route_result["score"] = round(float(route_scores[local, route_index[local]].cpu()), 5)
                route_result["primitiveIndex"] = int(route_index[local].cpu())
                if previous_result is not None and previous_index is not None and previous_scores is not None:
                    previous_result["score"] = round(float(previous_scores[local, previous_index[local]].cpu()), 5)
                    previous_result["primitiveIndex"] = int(previous_index[local].cpu())
                simple_result["score"] = round(float(simple_scores[local, simple_index[local]].cpu()), 5)
                simple_result["primitiveIndex"] = int(simple_index[local].cpu())
                route_values.append(route_result)
                if previous_result is not None:
                    previous_values.append(previous_result)
                baseline_values.append(simple_result)
                scene_name = scene.path.name
                scene_bucket = by_scene.setdefault(
                    scene_name,
                    {"routeYopo": [], "previousRouteYopo": [], "yopoSimple": []},
                )
                scene_bucket["routeYopo"].append(route_result)
                if previous_result is not None:
                    scene_bucket["previousRouteYopo"].append(previous_result)
                scene_bucket["yopoSimple"].append(simple_result)
                predictions[(scene_name, route_id)] = {
                    "path": route_result["path"],
                    "score": route_result["score"],
                    "primitiveIndex": route_result["primitiveIndex"],
                    "minimumClearanceM": route_result["minimumClearanceM"],
                    "collision": route_result["collision"],
                    "maximumCorridorViolationM": route_result["maximumCorridorViolationM"],
                    "meanCorridorViolationM": route_result["meanCorridorViolationM"],
                    "routeProgressM": route_result["routeProgressM"],
                    "trajectoryLengthM": route_result["trajectoryLengthM"],
                    "endpointDistanceM": route_result["endpointDistanceM"],
                    "averageSpeedMps": route_result["averageSpeedMps"],
                    **(
                        {
                            "previousPath": previous_result["path"],
                            "previousScore": previous_result["score"],
                            "previousPrimitiveIndex": previous_result["primitiveIndex"],
                            "previousMinimumClearanceM": previous_result["minimumClearanceM"],
                            "previousCollision": previous_result["collision"],
                            "previousMaximumCorridorViolationM": previous_result["maximumCorridorViolationM"],
                            "previousMeanCorridorViolationM": previous_result["meanCorridorViolationM"],
                            "previousRouteProgressM": previous_result["routeProgressM"],
                            "previousTrajectoryLengthM": previous_result["trajectoryLengthM"],
                            "previousEndpointDistanceM": previous_result["endpointDistanceM"],
                            "previousAverageSpeedMps": previous_result["averageSpeedMps"],
                        }
                        if previous_result is not None
                        else {}
                    ),
                    "baselinePath": simple_result["path"],
                    "baselineScore": simple_result["score"],
                    "baselinePrimitiveIndex": simple_result["primitiveIndex"],
                    "baselineMinimumClearanceM": simple_result["minimumClearanceM"],
                    "baselineCollision": simple_result["collision"],
                    "baselineMaximumCorridorViolationM": simple_result["maximumCorridorViolationM"],
                    "baselineMeanCorridorViolationM": simple_result["meanCorridorViolationM"],
                    "baselineRouteProgressM": simple_result["routeProgressM"],
                    "baselineTrajectoryLengthM": simple_result["trajectoryLengthM"],
                    "baselineEndpointDistanceM": simple_result["endpointDistanceM"],
                    "baselineAverageSpeedMps": simple_result["averageSpeedMps"],
                }
            cursor += count
            print(f"evaluated {cursor}/{len(samples)} routes", flush=True)

    report = {
        "benchmark": (
            "paired_current_previous_route_yopo_simple"
            if previous_route_checkpoint is not None
            else "paired_yopo_simple_route_yopo"
        ),
        "dataset": str(data_root),
        "sampleCount": len(route_values),
        "split": split,
        "motionInput": "paired deterministic dataset velocity and acceleration",
        "inputContract": {
            "pairedDepthPoseMotionGoal": True,
            "localSubgoalDistanceM": float(cfg["goal_length"]),
            "maximumPrimitiveEndpointRadiusM": 2.0 * float(cfg["radio_range"]),
            "witnessPathMayExtendBeyondSubgoal": True,
        },
        "routeYopoCheckpoint": str(route_checkpoint.resolve()),
        "routeYopoFeatureOrder": route_feature_order,
        "previousRouteYopoFeatureOrder": previous_feature_order,
        "previousRouteYopoCheckpoint": (
            str(previous_route_checkpoint.resolve())
            if previous_route_checkpoint is not None
            else None
        ),
        "yopoSimpleCheckpoint": str(baseline_checkpoint.resolve()),
        "models": {
            "routeYopo": _aggregate(route_values),
            **(
                {"previousRouteYopo": _aggregate(previous_values)}
                if previous_values
                else {}
            ),
            "yopoSimple": _aggregate(baseline_values),
        },
        "scenes": {
            name: {
                "routeYopo": _aggregate(values["routeYopo"]),
                **(
                    {"previousRouteYopo": _aggregate(values["previousRouteYopo"])}
                    if values["previousRouteYopo"]
                    else {}
                ),
                "yopoSimple": _aggregate(values["yopoSimple"]),
            }
            for name, values in by_scene.items()
        },
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "comparison_report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "comparison_predictions.json").write_text(json.dumps({f"{scene}/{route}": value for (scene, route), value in predictions.items()}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if build_viewer:
        build_dataset_viewer(
            data_root,
            output_dir=output_dir / "viewer",
            overwrite=True,
            predictions=predictions,
            evaluation_report=report,
        )
    print(json.dumps(report, indent=2, sort_keys=True))
    return output_dir / "comparison_report.json"


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare current/previous Route-YOPO and YOPO-Simple on paired scenes")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--route-checkpoint", type=Path, default=Path("saved/YOPO_3/best.pth"))
    parser.add_argument("--previous-route-checkpoint", type=Path)
    parser.add_argument("--simple-checkpoint", type=Path, default=Path("/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", choices=("cpu", "cuda"))
    parser.add_argument("--max-samples", type=int)
    parser.add_argument("--split", choices=("all", "train", "valid"), default="all")
    parser.add_argument("--report-only", action="store_true")
    parser.add_argument("--safety-shield", action="store_true")
    args = parser.parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    evaluate_comparison(
        args.data,
        args.route_checkpoint,
        args.simple_checkpoint,
        args.output,
        previous_route_checkpoint=args.previous_route_checkpoint,
        batch_size=args.batch_size,
        workers=args.workers,
        device=args.device,
        max_samples=args.max_samples,
        split=args.split,
        build_viewer=not args.report_only,
        safety_shield=args.safety_shield,
    )


if __name__ == "__main__":
    main()
