from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from scipy.spatial import cKDTree
from torch.utils.data import DataLoader

from compare_yopo import _aggregate, _load_baseline, _single_metrics
from config.config import cfg
from data.build_dataset_viewer import build_dataset_viewer
from data.snapshot_dataset import read_ascii_point_cloud_ply
from evaluate_yopo import _sample_trajectory
from loss.route_loss import RouteLoss
from policy.spline_trajectory import ClampedCubicSpline
from policy.spline_yopo_network import SplineYopoNetwork
from policy.state_transform import rotate_body2world, state_body2world
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork


def evaluate(
    *,
    data_root: Path,
    spline_checkpoint: Path,
    route_checkpoint: Path,
    simple_checkpoint: Path,
    output_dir: Path,
    batch_size: int,
    device: str,
) -> Path:
    selected_device = torch.device(device)
    dataset = YOPODataset("all", data_root=data_root)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=0)
    spline_state = torch.load(
        spline_checkpoint, map_location=selected_device, weights_only=False
    )
    control_point_count = int(spline_state["control_point_count"])
    spline_policy = SplineYopoNetwork(
        control_point_count=control_point_count
    ).to(selected_device).eval()
    spline_policy.load_spline_checkpoint(spline_state)
    spline = ClampedCubicSpline(
        control_point_count=control_point_count,
        duration=float(cfg["sgm_time"]),
        sample_count=30,
    ).to(selected_device)
    position_basis = torch.from_numpy(
        spline.basis(101, include_start=True)[0]
    ).to(selected_device, dtype=torch.float32)
    velocity_basis = torch.from_numpy(
        spline.basis(101, include_start=True)[1]
    ).to(selected_device, dtype=torch.float32)
    acceleration_basis = torch.from_numpy(
        spline.basis(101, include_start=True)[2]
    ).to(selected_device, dtype=torch.float32)
    jerk_basis = torch.from_numpy(
        spline.basis(101, include_start=True)[3]
    ).to(selected_device, dtype=torch.float32)
    route_loss = RouteLoss(torch.eye(6, device=selected_device), eval_points=101).to(
        selected_device
    )

    route_policy = YopoNetwork().to(selected_device).eval()
    route_state = torch.load(
        route_checkpoint, map_location=selected_device, weights_only=False
    )
    route_policy.load_route_checkpoint(route_state)
    simple_policy = _load_baseline(simple_checkpoint, selected_device)
    obstacle_trees = [
        cKDTree(read_ascii_point_cloud_ply(scene.path / "tree.ply"))
        for scene in dataset.scenes
    ]

    selected_values = []
    oracle_values = []
    route_values = []
    simple_values = []
    predictions = {}
    cursor = 0
    with torch.inference_mode():
        for batch in loader:
            count = batch["depth"].shape[0]
            batch_gpu = {
                key: value.to(selected_device) for key, value in batch.items()
            }
            free_body, spline_score = spline_policy(
                batch_gpu["depth"],
                batch_gpu["motion_body"],
                batch_gpu["frontier_body"],
                batch_gpu["route_bubbles"],
            )
            start_body = torch.zeros(
                (count, int(cfg["traj_num"]), 3), device=selected_device
            )
            velocity_body = batch_gpu["motion_body"][:, None, :3].expand(
                -1, int(cfg["traj_num"]), -1
            )
            acceleration_body = batch_gpu["motion_body"][:, None, 3:6].expand(
                -1, int(cfg["traj_num"]), -1
            )
            controls = spline.assemble_controls(
                start_body, velocity_body, acceleration_body, free_body
            )
            position_body = spline.sample_with_basis(controls, position_basis)
            sampled_velocity_body = spline.sample_with_basis(controls, velocity_basis)
            sampled_acceleration_body = spline.sample_with_basis(controls, acceleration_basis)
            sampled_jerk_body = spline.sample_with_basis(controls, jerk_basis)
            rotation = batch_gpu["rotation_world_body"][:, None, None]
            spline_positions = batch_gpu["position_world"][:, None, None] + torch.matmul(
                rotation, position_body.unsqueeze(-1)
            ).squeeze(-1)
            flat_positions = spline_positions.reshape(-1, 101, 3)
            route_cost = route_loss.forward_positions(
                flat_positions,
                batch_gpu["position_world"].repeat_interleave(int(cfg["traj_num"]), 0),
                batch_gpu["route_points_world"].repeat_interleave(int(cfg["traj_num"]), 0),
            ).reshape(count, int(cfg["traj_num"]))
            selected_index = spline_score.reshape(count, -1).argmin(1)
            oracle_index = route_cost.argmin(1)
            row = torch.arange(count, device=selected_device)
            selected_paths = spline_positions[row, selected_index].cpu().numpy()
            oracle_paths = spline_positions[row, oracle_index].cpu().numpy()
            selected_speed = sampled_velocity_body[row, selected_index].norm(dim=-1).amax(dim=-1).cpu().numpy()
            selected_acceleration = sampled_acceleration_body[row, selected_index].norm(dim=-1).amax(dim=-1).cpu().numpy()
            selected_jerk = sampled_jerk_body[row, selected_index].norm(dim=-1).amax(dim=-1).cpu().numpy()

            route_end, route_score = route_policy(
                batch_gpu["depth"],
                batch_gpu["motion_body"],
                batch_gpu["frontier_body"],
                batch_gpu["route_bubbles"],
            )
            simple_end, simple_score = simple_policy(
                batch_gpu["depth"], batch_gpu["motion_body"], batch_gpu["frontier_body"]
            )
            route_flat = route_end.permute(0, 2, 3, 1).reshape(count, -1, 9)
            simple_flat = simple_end.permute(0, 2, 3, 1).reshape(count, -1, 9)
            route_index = route_score.reshape(count, -1).argmin(1)
            simple_index = simple_score.reshape(count, -1).argmin(1)
            route_body = route_flat[row, route_index]
            simple_body = simple_flat[row, simple_index]
            position = batch_gpu["position_world"]
            rotation_body = batch_gpu["rotation_world_body"]
            route_end_world = state_body2world(
                position,
                rotation_body,
                route_body[:, :3],
                route_body[:, 3:6],
                route_body[:, 6:9],
            )
            simple_end_world = state_body2world(
                position,
                rotation_body,
                simple_body[:, :3],
                simple_body[:, 3:6],
                simple_body[:, 6:9],
            )
            start_velocity = rotate_body2world(
                rotation_body, batch_gpu["motion_body"][:, :3]
            )
            start_acceleration = rotate_body2world(
                rotation_body, batch_gpu["motion_body"][:, 3:]
            )
            starts = torch.stack(
                (position, start_velocity, start_acceleration), dim=1
            ).cpu().numpy()
            route_ends = torch.stack(route_end_world, dim=1).cpu().numpy()
            simple_ends = torch.stack(simple_end_world, dim=1).cpu().numpy()

            for local in range(count):
                scene_index, route_id = dataset.samples[cursor + local]
                scene = dataset.scenes[scene_index]
                path, _, radii = scene.routes.path(route_id)
                selected_result = _single_metrics(
                    selected_paths[local], path, radii, obstacle_trees[scene_index]
                )
                selected_result["maximumSpeedMps"] = float(selected_speed[local])
                selected_result["maximumAccelerationMps2"] = float(selected_acceleration[local])
                selected_result["maximumJerkMps3"] = float(selected_jerk[local])
                oracle_result = _single_metrics(
                    oracle_paths[local], path, radii, obstacle_trees[scene_index]
                )
                route_result = _single_metrics(
                    _sample_trajectory(starts[local], route_ends[local]),
                    path,
                    radii,
                    obstacle_trees[scene_index],
                )
                simple_result = _single_metrics(
                    _sample_trajectory(starts[local], simple_ends[local]),
                    path,
                    radii,
                    obstacle_trees[scene_index],
                )
                selected_values.append(selected_result)
                oracle_values.append(oracle_result)
                route_values.append(route_result)
                simple_values.append(simple_result)
                predictions[(scene.path.name, route_id)] = {
                    "path": selected_result["path"],
                    "collision": selected_result["collision"],
                    "minimumClearanceM": selected_result["minimumClearanceM"],
                    "maximumCorridorViolationM": selected_result["maximumCorridorViolationM"],
                    "meanCorridorViolationM": selected_result["meanCorridorViolationM"],
                    "routeProgressM": selected_result["routeProgressM"],
                    "trajectoryLengthM": selected_result["trajectoryLengthM"],
                    "endpointDistanceM": selected_result["endpointDistanceM"],
                    "averageSpeedMps": selected_result["averageSpeedMps"],
                    "maximumSpeedMps": selected_result["maximumSpeedMps"],
                    "maximumAccelerationMps2": selected_result["maximumAccelerationMps2"],
                    "maximumJerkMps3": selected_result["maximumJerkMps3"],
                    "score": round(float(spline_score.reshape(count, -1)[local, selected_index[local]].cpu()), 5),
                    "primitiveIndex": int(selected_index[local].cpu()),
                    "splineOraclePath": oracle_result["path"],
                    "previousPath": route_result["path"],
                    "previousCollision": route_result["collision"],
                    "previousMinimumClearanceM": route_result["minimumClearanceM"],
                    "previousMaximumCorridorViolationM": route_result["maximumCorridorViolationM"],
                    "previousMeanCorridorViolationM": route_result["meanCorridorViolationM"],
                    "previousRouteProgressM": route_result["routeProgressM"],
                    "previousTrajectoryLengthM": route_result["trajectoryLengthM"],
                    "previousEndpointDistanceM": route_result["endpointDistanceM"],
                    "previousAverageSpeedMps": route_result["averageSpeedMps"],
                    "previousScore": 0.0,
                    "previousPrimitiveIndex": int(route_index[local].cpu()),
                    "baselinePath": simple_result["path"],
                    "baselineCollision": simple_result["collision"],
                    "baselineMinimumClearanceM": simple_result["minimumClearanceM"],
                    "baselineMaximumCorridorViolationM": simple_result["maximumCorridorViolationM"],
                    "baselineMeanCorridorViolationM": simple_result["meanCorridorViolationM"],
                    "baselineRouteProgressM": simple_result["routeProgressM"],
                    "baselineTrajectoryLengthM": simple_result["trajectoryLengthM"],
                    "baselineEndpointDistanceM": simple_result["endpointDistanceM"],
                    "baselineAverageSpeedMps": simple_result["averageSpeedMps"],
                    "baselineScore": 0.0,
                    "baselinePrimitiveIndex": int(simple_index[local].cpu()),
                }
            cursor += count
            if cursor % 320 == 0 or cursor == len(dataset):
                print(f"evaluated {cursor}/{len(dataset)} routes", flush=True)

    report = {
        "sampleCount": len(dataset),
        "models": {
            "splineSelected": _aggregate(selected_values),
            "splineOracle": _aggregate(oracle_values),
            "quinticRouteYopo": _aggregate(route_values),
            "yopoSimple": _aggregate(simple_values),
        },
        "splineCheckpoint": str(Path(spline_checkpoint).resolve()),
        "controlPointCount": control_point_count,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "comparison_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    build_dataset_viewer(
        Path(data_root),
        output_dir=output_dir / "viewer",
        overwrite=True,
        predictions=predictions,
        evaluation_report=report,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return output_dir / "comparison_report.json"


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare spline and quintic Route-YOPO")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--spline-checkpoint", type=Path, required=True)
    parser.add_argument("--route-checkpoint", type=Path, required=True)
    parser.add_argument("--simple-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    args = parser.parse_args()
    evaluate(
        data_root=args.data,
        spline_checkpoint=args.spline_checkpoint,
        route_checkpoint=args.route_checkpoint,
        simple_checkpoint=args.simple_checkpoint,
        output_dir=args.output,
        batch_size=args.batch_size,
        device=args.device,
    )


if __name__ == "__main__":
    main()
