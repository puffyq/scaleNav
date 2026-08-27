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
from data.route_contract import polyline_arclength
from data.snapshot_dataset import read_ascii_point_cloud_ply
from policy.state_transform import rotate_body2world, state_body2world
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork


def _coefficient_map(segment_time: float) -> np.ndarray:
    time = float(segment_time)
    system = np.zeros((6, 6), dtype=np.float64)
    for derivative in range(3):
        system[2 * derivative, derivative] = math.factorial(derivative)
        for power in range(derivative, 6):
            system[2 * derivative + 1, power] = (
                math.factorial(power)
                / math.factorial(power - derivative)
                * time ** (power - derivative)
            )
    reorder = np.zeros((6, 6), dtype=np.float64)
    reorder[[0, 2, 4, 1, 3, 5], np.arange(6)] = 1.0
    return np.linalg.inv(system) @ reorder


def _sample_trajectory(
    start_state: np.ndarray,
    end_state: np.ndarray,
    *,
    count: int = 101,
) -> np.ndarray:
    coefficient_map = _coefficient_map(float(cfg["sgm_time"]))
    boundary = np.concatenate((start_state, end_state), axis=0).T
    coefficients = (coefficient_map @ boundary.T).T
    times = np.linspace(0.0, float(cfg["sgm_time"]), count, dtype=np.float64)
    powers = np.stack([times**power for power in range(6)], axis=1)
    return (powers @ coefficients.T).astype(np.float32)


def _corridor_metrics(
    prediction: np.ndarray,
    route_points: np.ndarray,
    route_radii: np.ndarray,
) -> tuple[float, float, float]:
    safe_radii = np.clip(
        route_radii, 0.0, float(cfg["route_corridor_width_cap_m"])
    )
    bubble_distance = np.linalg.norm(
        prediction[:, None, :] - route_points[None, :, :], axis=2
    )
    violation = np.maximum(
        0.0, np.min(bubble_distance - safe_radii[None, :], axis=1)
    )

    segment_start = route_points[:-1]
    segment = route_points[1:] - segment_start
    length_squared = np.maximum(np.sum(segment * segment, axis=1), 1.0e-8)
    difference = prediction[:, None, :] - segment_start[None, :, :]
    alpha = np.clip(np.sum(difference * segment[None, :, :], axis=2) / length_squared, 0.0, 1.0)
    closest = segment_start[None, :, :] + alpha[:, :, None] * segment[None, :, :]
    distance = np.linalg.norm(prediction[:, None, :] - closest, axis=2)
    nearest = np.argmin(distance, axis=1)
    nearest_alpha = alpha[np.arange(len(prediction)), nearest]
    cumulative, _ = polyline_arclength(route_points)
    progress = cumulative[nearest[-1]] + nearest_alpha[-1] * math.sqrt(length_squared[nearest[-1]])
    return float(np.max(violation)), float(np.mean(violation)), float(progress)


def evaluate(
    data_root: Path,
    checkpoint_path: Path,
    output_dir: Path,
    *,
    batch_size: int,
    workers: int,
    device: str | None,
    max_samples: int | None,
    split: str = "all",
    build_viewer: bool = True,
) -> Path:
    data_root = Path(data_root).resolve()
    checkpoint_path = Path(checkpoint_path).resolve()
    selected_device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    dataset = YOPODataset(
        split, data_root=data_root, route_dropout_probability=0.0
    )
    samples = dataset.samples if max_samples is None else dataset.samples[:max_samples]
    if len(samples) != len(dataset.samples):
        dataset.samples = list(samples)
    loader = DataLoader(
        dataset,
        batch_size=int(batch_size),
        shuffle=False,
        num_workers=int(workers),
        pin_memory=selected_device.type == "cuda",
    )
    policy = YopoNetwork().to(selected_device).eval()
    checkpoint = torch.load(checkpoint_path, map_location=selected_device, weights_only=False)
    state_dict = checkpoint.get("model_state_dict", checkpoint)
    policy.load_state_dict(state_dict)

    obstacle_trees = [
        cKDTree(read_ascii_point_cloud_ply(scene.path / "tree.ply"))
        for scene in dataset.scenes
    ]
    predictions: dict[tuple[str, int], dict[str, Any]] = {}
    cursor = 0
    with torch.inference_mode():
        for batch in loader:
            count = batch["depth"].shape[0]
            motion = torch.zeros((count, 6), dtype=torch.float32, device=selected_device)
            endstate, score = policy(
                batch["depth"].to(selected_device),
                motion,
                batch["frontier_body"].to(selected_device),
                batch["route_bubbles"].to(selected_device),
                batch["route_mask"].to(selected_device),
            )
            endstate_flat = endstate.permute(0, 2, 3, 1).reshape(count, -1, 9)
            score_flat = score.reshape(count, -1)
            selected = score_flat.argmin(dim=1)
            row = torch.arange(count, device=selected_device)
            selected_body = endstate_flat[row, selected]
            position = batch["position_world"].to(selected_device)
            rotation = batch["rotation_world_body"].to(selected_device)
            end_position, end_velocity, end_acceleration = state_body2world(
                position,
                rotation,
                selected_body[:, :3],
                selected_body[:, 3:6],
                selected_body[:, 6:9],
            )
            start_velocity = rotate_body2world(rotation, motion[:, :3])
            start_acceleration = rotate_body2world(rotation, motion[:, 3:])
            start_states = torch.stack((position, start_velocity, start_acceleration), dim=1).cpu().numpy()
            end_states = torch.stack((end_position, end_velocity, end_acceleration), dim=1).cpu().numpy()
            selected_scores = score_flat[row, selected].cpu().numpy()
            selected_indices = selected.cpu().numpy()

            for local_index in range(count):
                scene_index, route_index = samples[cursor + local_index]
                scene = dataset.scenes[scene_index]
                path, _, radii = scene.routes.path(route_index)
                trajectory = _sample_trajectory(start_states[local_index], end_states[local_index])
                clearance = obstacle_trees[scene_index].query(trajectory, k=1)[0]
                maximum_violation, mean_violation, progress = _corridor_metrics(
                    trajectory, path, radii
                )
                minimum_clearance = float(np.min(clearance))
                predictions[(scene.path.name, route_index)] = {
                    "path": trajectory.round(4).tolist(),
                    "score": round(float(selected_scores[local_index]), 5),
                    "primitiveIndex": int(selected_indices[local_index]),
                    "minimumClearanceM": round(minimum_clearance, 4),
                    "collision": minimum_clearance < (
                        float(cfg["robot_radius_m"]) + float(cfg["safety_margin_m"])
                    ),
                    "maximumCorridorViolationM": round(maximum_violation, 4),
                    "meanCorridorViolationM": round(mean_violation, 4),
                    "routeProgressM": round(progress, 4),
                }
            cursor += count
            print(f"evaluated {cursor}/{len(samples)} routes", flush=True)

    values = list(predictions.values())
    report = {
        "dataset": str(data_root),
        "checkpoint": str(checkpoint_path),
        "checkpointEpoch": int(checkpoint.get("epoch", -1)) + 1,
        "sampleCount": len(values),
        "split": split,
        "device": str(selected_device),
        "motionInput": "zero velocity and zero acceleration",
        "collisionRate": float(np.mean([item["collision"] for item in values])),
        "minimumClearanceM": float(min(item["minimumClearanceM"] for item in values)),
        "meanMinimumClearanceM": float(np.mean([item["minimumClearanceM"] for item in values])),
        "meanMaximumCorridorViolationM": float(
            np.mean([item["maximumCorridorViolationM"] for item in values])
        ),
        "meanRouteProgressM": float(np.mean([item["routeProgressM"] for item in values])),
    }
    output_dir = Path(output_dir).resolve()
    if build_viewer:
        result_path = build_dataset_viewer(
            data_root,
            output_dir=output_dir,
            overwrite=True,
            predictions=predictions,
            evaluation_report=report,
        )
    else:
        output_dir.mkdir(parents=True, exist_ok=True)
        result_path = output_dir / "evaluation_report.json"
    (output_dir / "evaluation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return result_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate YOPO model output on an offline dataset")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", choices=("cpu", "cuda"))
    parser.add_argument("--max-samples", type=int)
    parser.add_argument("--split", choices=("all", "train", "valid"), default="all")
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    print(
        evaluate(
            args.data,
            args.checkpoint,
            args.output,
            batch_size=args.batch_size,
            workers=args.workers,
            device=args.device,
            max_samples=args.max_samples,
            split=args.split,
            build_viewer=not args.report_only,
        )
    )


if __name__ == "__main__":
    main()
