"""Separate primitive generation failures from score-selection failures."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
from scipy.spatial import cKDTree
from torch.utils.data import DataLoader

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config.config import cfg
from data.snapshot_dataset import read_ascii_point_cloud_ply
from evaluate_yopo import _corridor_metrics, _sample_trajectory
from policy.state_transform import state_body2world
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork


def run(data_root: Path, checkpoint: Path, output: Path, device: str) -> None:
    selected_device = torch.device(device)
    dataset = YOPODataset("all", data_root=data_root, route_dropout_probability=0.0)
    loader = DataLoader(dataset, batch_size=64, shuffle=False, num_workers=0)
    policy = YopoNetwork().to(selected_device).eval()
    state = torch.load(checkpoint, map_location=selected_device, weights_only=False)
    feature_order = policy.load_route_checkpoint(state)
    trees = [cKDTree(read_ascii_point_cloud_ply(scene.path / "tree.ply")) for scene in dataset.scenes]
    threshold = float(cfg["robot_radius_m"] + cfg["safety_margin_m"])
    selected_collision = safety_oracle_collision = corridor_oracle_collision = 0
    any_safe = 0
    selected_safety_oracle = 0
    samples = 0
    with torch.inference_mode():
        for batch in loader:
            count = batch["depth"].shape[0]
            motion = torch.zeros((count, 6), device=selected_device)
            endstate, score = policy(
                batch["depth"].to(selected_device), motion,
                batch["frontier_body"].to(selected_device),
                batch["route_bubbles"].to(selected_device),
                batch["route_mask"].to(selected_device),
            )
            flat = endstate.permute(0, 2, 3, 1).reshape(count, 15, 9)
            position = batch["position_world"].to(selected_device)[:, None].expand(-1, 15, -1).reshape(-1, 3)
            rotation = batch["rotation_world_body"].to(selected_device)[:, None].expand(-1, 15, -1, -1).reshape(-1, 3, 3)
            end_position, end_velocity, end_acceleration = state_body2world(
                position, rotation, flat[:, :, :3].reshape(-1, 3),
                flat[:, :, 3:6].reshape(-1, 3), flat[:, :, 6:9].reshape(-1, 3),
            )
            end_position = end_position.reshape(count, 15, 3).cpu().numpy()
            end_velocity = end_velocity.reshape(count, 15, 3).cpu().numpy()
            end_acceleration = end_acceleration.reshape(count, 15, 3).cpu().numpy()
            scores = score.reshape(count, 15).cpu().numpy()
            for row, (scene_index, route_index) in enumerate(dataset.samples[samples:samples + count]):
                scene = dataset.scenes[scene_index]
                path, _, radii = scene.routes.path(route_index)
                values = []
                start = np.stack((batch["position_world"][row].numpy(), np.zeros(3), np.zeros(3)))
                for primitive in range(15):
                    end = np.stack((end_position[row, primitive], end_velocity[row, primitive], end_acceleration[row, primitive]))
                    trajectory = _sample_trajectory(start, end)
                    clearance = float(trees[scene_index].query(trajectory, k=1)[0].min())
                    violation, _, progress = _corridor_metrics(trajectory, path, radii)
                    values.append((clearance, violation, progress))
                selected = int(np.argmin(scores[row]))
                safety_best = min(range(15), key=lambda i: (-values[i][0], values[i][1]))
                corridor_best = min(range(15), key=lambda i: (values[i][1], -values[i][2]))
                selected_collision += values[selected][0] < threshold
                safety_oracle_collision += values[safety_best][0] < threshold
                corridor_oracle_collision += values[corridor_best][0] < threshold
                any_safe += any(value[0] >= threshold for value in values)
                selected_safety_oracle += selected == safety_best
            samples += count
    report = {
        "checkpoint": str(checkpoint.resolve()),
        "dataset": str(data_root.resolve()),
        "featureOrder": feature_order,
        "sampleCount": samples,
        "collisionThresholdM": threshold,
        "selectedCollisionRate": selected_collision / samples,
        "safetyOracleCollisionRate": safety_oracle_collision / samples,
        "corridorOracleCollisionRate": corridor_oracle_collision / samples,
        "inputsWithAnySafeCandidateRate": any_safe / samples,
        "selectedEqualsSafetyOracleRate": selected_safety_oracle / samples,
        "interpretation": "candidate-generation failure" if safety_oracle_collision else "score-selection failure",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    args = parser.parse_args()
    run(args.data, args.checkpoint, args.output, args.device)
