"""Offline diagnostic for all 15 YOPO primitive candidates.

Unlike the normal evaluator, this keeps every candidate trajectory instead of
only the score-selected primitive.  The output is consumed by the static
candidate diagnostic viewer and makes score-head errors distinguishable from
trajectory/loss errors.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from scipy.spatial import cKDTree
from torch.utils.data import DataLoader

from config.config import cfg
from data.build_dataset_viewer import build_dataset_viewer
from data.snapshot_dataset import read_ascii_point_cloud_ply
from policy.state_transform import state_body2world
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork
from policy.yopo_simple_baseline import YopoSimpleBaseline
from compare_yopo import _load_baseline, _single_metrics
from evaluate_yopo import _sample_trajectory


def _candidate_record(
    trajectory: np.ndarray,
    score: float,
    index: int,
    route_path: np.ndarray,
    route_radii: np.ndarray,
    obstacle_tree: cKDTree,
    end_state_body: np.ndarray | None = None,
    end_state_world: np.ndarray | None = None,
) -> dict[str, Any]:
    result = _single_metrics(trajectory, route_path, route_radii, obstacle_tree)
    result.update({
        "primitiveIndex": int(index),
        "score": round(float(score), 5),
        "selected": False,
        "path": np.asarray(trajectory, dtype=np.float32).round(4).tolist(),
    })
    path = np.asarray(trajectory, dtype=np.float32)
    result.update({
        "minimumZM": round(float(path[:, 2].min()), 4),
        "maximumZM": round(float(path[:, 2].max()), 4),
        "verticalExcursionM": round(float(path[:, 2].max() - path[:, 2].min()), 4),
    })
    if end_state_body is not None:
        result["endStateBody"] = np.asarray(end_state_body, dtype=np.float32).round(5).tolist()
    if end_state_world is not None:
        result["endStateWorld"] = np.asarray(end_state_world, dtype=np.float32).round(5).tolist()
    return result


def evaluate_candidates(
    data_root: Path,
    checkpoint_path: Path,
    output_dir: Path,
    *,
    batch_size: int = 32,
    workers: int = 0,
    device: str | None = None,
    max_samples: int | None = None,
    trajectory_points: int = 101,
    baseline_checkpoint: Path | None = None,
) -> Path:
    data_root = Path(data_root).resolve()
    checkpoint_path = Path(checkpoint_path).resolve()
    output_dir = Path(output_dir).resolve()
    selected_device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    dataset = YOPODataset("all", data_root=data_root, route_dropout_probability=0.0)
    samples = dataset.samples if max_samples is None else dataset.samples[:max_samples]
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
    feature_order = policy.load_route_checkpoint(checkpoint)
    baseline = (
        _load_baseline(Path(baseline_checkpoint).resolve(), selected_device)
        if baseline_checkpoint is not None
        else None
    )
    obstacle_trees = [
        cKDTree(read_ascii_point_cloud_ply(scene.path / "tree.ply"))
        for scene in dataset.scenes
    ]

    predictions: dict[tuple[str, int], dict[str, Any]] = {}
    selected_values: list[dict[str, Any]] = []
    oracle_values: list[dict[str, Any]] = []
    selected_oracle_matches = 0
    simple_selected_values: list[dict[str, Any]] = []
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
            if baseline is not None:
                simple_endstate, simple_score = baseline(
                    batch["depth"].to(selected_device),
                    motion,
                    batch["frontier_body"].to(selected_device),
                )
            else:
                simple_endstate = simple_score = None
            endstate_flat = endstate.permute(0, 2, 3, 1).reshape(count, -1, 9)
            score_flat = score.reshape(count, -1)
            position = batch["position_world"].to(selected_device)
            rotation = batch["rotation_world_body"].to(selected_device)
            candidate_count = endstate_flat.shape[1]
            position_all = position[:, None, :].expand(-1, candidate_count, -1).reshape(-1, 3)
            rotation_all = rotation[:, None, :, :].expand(-1, candidate_count, -1, -1).reshape(-1, 3, 3)
            end_position, end_velocity, end_acceleration = state_body2world(
                position_all,
                rotation_all,
                endstate_flat.reshape(-1, 9)[:, :3],
                endstate_flat.reshape(-1, 9)[:, 3:6],
                endstate_flat.reshape(-1, 9)[:, 6:9],
            )
            end_positions = end_position.reshape(count, candidate_count, 3).cpu().numpy()
            end_velocities = end_velocity.reshape(count, candidate_count, 3).cpu().numpy()
            end_accelerations = end_acceleration.reshape(count, candidate_count, 3).cpu().numpy()
            scores = score_flat.cpu().numpy()

            def world_endstates(flat: torch.Tensor) -> tuple[np.ndarray, np.ndarray]:
                flat_count = flat.shape[1]
                positions = position[:, None, :].expand(-1, flat_count, -1).reshape(-1, 3)
                rotations = rotation[:, None, :, :].expand(-1, flat_count, -1, -1).reshape(-1, 3, 3)
                flattened = flat.reshape(-1, 9)
                p, v, a = state_body2world(
                    positions, rotations, flattened[:, :3], flattened[:, 3:6], flattened[:, 6:9]
                )
                world = torch.cat((p, v, a), dim=1).reshape(count, flat_count, 9)
                return flat.cpu().numpy(), world.cpu().numpy()

            route_body_states, route_world_states = world_endstates(endstate_flat)
            if simple_endstate is not None and simple_score is not None:
                simple_flat = simple_endstate.permute(0, 2, 3, 1).reshape(count, -1, 9)
                simple_body_states, simple_world_states = world_endstates(simple_flat)
                simple_scores = simple_score.reshape(count, -1).cpu().numpy()
            else:
                simple_body_states = simple_world_states = simple_scores = None

            for local in range(count):
                scene_index, route_index = samples[cursor + local]
                scene = dataset.scenes[scene_index]
                path, _, radii = scene.routes.path(route_index)
                starts = np.repeat(position[local].cpu().numpy()[None, :], candidate_count, axis=0)
                candidates: list[dict[str, Any]] = []
                for primitive in range(candidate_count):
                    start_state = np.stack(
                        (starts[primitive], np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float32)),
                        axis=0,
                    )
                    end_state = np.stack(
                        (end_positions[local, primitive], end_velocities[local, primitive], end_accelerations[local, primitive]),
                        axis=0,
                    )
                    trajectory = _sample_trajectory(start_state, end_state, count=trajectory_points)
                    candidates.append(_candidate_record(
                        trajectory,
                        scores[local, primitive],
                        primitive,
                        path,
                        radii,
                        obstacle_trees[scene_index],
                        route_body_states[local, primitive],
                        route_world_states[local, primitive],
                    ))
                selected_index = int(np.argmin(scores[local]))
                safe_indices = [
                    idx for idx, item in enumerate(candidates) if not bool(item["collision"])
                ]
                oracle_pool = safe_indices or list(range(candidate_count))
                oracle_index = min(
                    oracle_pool,
                    key=lambda idx: (
                        float(candidates[idx]["meanCenterlineDistanceM"]),
                        float(candidates[idx]["maximumCorridorViolationM"]),
                        -float(candidates[idx]["routeProgressM"]),
                    ),
                )
                for item in candidates:
                    item["selected"] = int(item["primitiveIndex"]) == selected_index
                    item["centerlineOracle"] = int(item["primitiveIndex"]) == oracle_index
                selected = candidates[selected_index]
                oracle = candidates[oracle_index]
                selected_values.append(selected)
                oracle_values.append(oracle)
                selected_oracle_matches += int(selected_index == oracle_index)
                simple_candidates: list[dict[str, Any]] = []
                simple_selected_index = None
                if simple_scores is not None and simple_body_states is not None and simple_world_states is not None:
                    for primitive in range(candidate_count):
                        simple_end_state = np.stack(
                            (
                                simple_world_states[local, primitive, :3],
                                simple_world_states[local, primitive, 3:6],
                                simple_world_states[local, primitive, 6:9],
                            ),
                            axis=0,
                        )
                        simple_trajectory = _sample_trajectory(
                            start_state, simple_end_state, count=trajectory_points
                        )
                        simple_candidates.append(_candidate_record(
                            simple_trajectory,
                            simple_scores[local, primitive],
                            primitive,
                            path,
                            radii,
                            obstacle_trees[scene_index],
                            simple_body_states[local, primitive],
                            simple_world_states[local, primitive],
                        ))
                    simple_selected_index = int(np.argmin(simple_scores[local]))
                    simple_candidates[simple_selected_index]["selected"] = True
                    simple_selected_values.append(simple_candidates[simple_selected_index])
                predictions[(scene.path.name, route_index)] = {
                    "path": selected["path"],
                    "score": selected["score"],
                    "primitiveIndex": selected_index,
                    "minimumClearanceM": selected["minimumClearanceM"],
                    "collision": selected["collision"],
                    "maximumCorridorViolationM": selected["maximumCorridorViolationM"],
                    "meanCorridorViolationM": selected["meanCorridorViolationM"],
                    "meanCenterlineDistanceM": selected["meanCenterlineDistanceM"],
                    "maximumCenterlineDistanceM": selected["maximumCenterlineDistanceM"],
                    "routeProgressM": selected["routeProgressM"],
                    "trajectoryLengthM": selected["trajectoryLengthM"],
                    "endpointDistanceM": selected["endpointDistanceM"],
                    "averageSpeedMps": selected["averageSpeedMps"],
                    "selectedIndex": selected_index,
                    "oracleCenterlineIndex": oracle_index,
                    "selectionOracleMatch": selected_index == oracle_index,
                    "selectionCenterlineGapM": round(
                        float(selected["meanCenterlineDistanceM"] - oracle["meanCenterlineDistanceM"]), 5
                    ),
                    "candidates": candidates,
                    "routeCandidates": candidates,
                    "simpleCandidates": simple_candidates,
                    "simpleSelectedIndex": simple_selected_index,
                }
            cursor += count
            print(f"evaluated {cursor}/{len(samples)} routes", flush=True)

    def aggregate(values: list[dict[str, Any]]) -> dict[str, Any]:
        return {
            "sampleCount": len(values),
            "collisionRate": float(np.mean([bool(x["collision"]) for x in values])),
            "meanCenterlineDistanceM": float(np.mean([x["meanCenterlineDistanceM"] for x in values])),
            "meanMaximumCenterlineDistanceM": float(np.mean([x["maximumCenterlineDistanceM"] for x in values])),
            "meanMaximumCorridorViolationM": float(np.mean([x["maximumCorridorViolationM"] for x in values])),
            "meanRouteProgressM": float(np.mean([x["routeProgressM"] for x in values])),
        }

    def vertical_summary(candidate_key: str, selected_key: str) -> dict[str, Any] | None:
        values = list(predictions.values())
        if not values or not values[0].get(candidate_key):
            return None
        body = np.asarray(
            [[candidate["endStateBody"] for candidate in value[candidate_key]] for value in values],
            dtype=np.float32,
        )
        selected_indices = np.asarray([value[selected_key] for value in values], dtype=np.int64)
        selected_body = body[np.arange(len(body)), selected_indices]
        layer_counts = np.bincount(
            selected_indices // int(cfg["horizon_num"]), minlength=int(cfg["vertical_num"])
        )
        primitive_counts = np.bincount(selected_indices, minlength=int(cfg["traj_num"]))
        reshaped = body.reshape(len(body), int(cfg["vertical_num"]), int(cfg["horizon_num"]), 9)
        return {
            "endpointBodyZMeanByOutputRowM": reshaped[:, :, :, 2].mean(axis=(0, 2)).tolist(),
            "endpointBodyVzMeanByOutputRowMps": reshaped[:, :, :, 5].mean(axis=(0, 2)).tolist(),
            "selectedOutputRowCounts": layer_counts.tolist(),
            "selectedOutputRowFractions": (layer_counts / max(layer_counts.sum(), 1)).tolist(),
            "selectedPrimitiveCounts": primitive_counts.tolist(),
            "selectedEndpointBodyZMeanM": float(selected_body[:, 2].mean()),
            "selectedEndpointBodyZStdM": float(selected_body[:, 2].std()),
        }

    report = {
        "benchmark": "all_15_primitive_candidate_diagnostic",
        "dataset": str(data_root),
        "checkpoint": str(checkpoint_path),
        "checkpointEpoch": int(checkpoint.get("epoch", -1)) + 1,
        "featureOrder": feature_order,
        "baselineCheckpoint": (
            str(Path(baseline_checkpoint).resolve()) if baseline_checkpoint is not None else None
        ),
        "sampleCount": len(samples),
        "candidateCount": int(candidate_count) if samples else int(cfg["traj_num"]),
        "trajectoryPoints": int(trajectory_points),
        "device": str(selected_device),
        "selected": aggregate(selected_values),
        "centerlineOracle": aggregate(oracle_values),
        "selectionOracleMatchRate": selected_oracle_matches / max(len(samples), 1),
        "selectionOracleMatchCount": selected_oracle_matches,
        "yopoSimpleSelected": aggregate(simple_selected_values) if simple_selected_values else None,
        "routeYopo3D": vertical_summary("routeCandidates", "selectedIndex"),
        "yopoSimple3D": vertical_summary("simpleCandidates", "simpleSelectedIndex"),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "candidate_diagnostic_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output_dir / "candidate_predictions.json").write_text(
        json.dumps({f"{scene}/{route}": value for (scene, route), value in predictions.items()},
                   separators=(",", ":"), ensure_ascii=True),
        encoding="utf-8",
    )
    build_dataset_viewer(
        data_root,
        output_dir=output_dir / "viewer",
        overwrite=True,
        predictions=predictions,
        evaluation_report=report,
    )
    template = Path(__file__).resolve().parent / "tools" / "candidate_diagnostic.html"
    (output_dir / "viewer" / "index.html").write_text(template.read_text(encoding="utf-8"), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    print(output_dir / "viewer" / "index.html")
    return output_dir / "viewer" / "index.html"


def main() -> None:
    parser = argparse.ArgumentParser(description="Save and visualize all 15 YOPO primitive candidates")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", choices=("cpu", "cuda"))
    parser.add_argument("--max-samples", type=int)
    parser.add_argument("--trajectory-points", type=int, default=101)
    parser.add_argument(
        "--simple-checkpoint",
        type=Path,
        default=Path("/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth"),
    )
    args = parser.parse_args()
    if args.trajectory_points < 2:
        raise ValueError("--trajectory-points must be at least 2")
    evaluate_candidates(
        args.data,
        args.checkpoint,
        args.output,
        batch_size=args.batch_size,
        workers=args.workers,
        device=args.device,
        max_samples=args.max_samples,
        trajectory_points=args.trajectory_points,
        baseline_checkpoint=args.simple_checkpoint,
    )


if __name__ == "__main__":
    main()
