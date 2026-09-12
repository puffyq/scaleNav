"""Offline point-cloud comparison of YOPO-Simple and ordered-bubble MPC."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from scipy.spatial import cKDTree
from torch.utils.data import DataLoader

from compare_yopo import _aggregate, _load_baseline, _single_metrics
from config.config import cfg
from data.build_dataset_viewer import build_dataset_viewer
from data.route_contract import polyline_arclength
from data.snapshot_dataset import read_ascii_point_cloud_ply
from evaluate_yopo import _sample_trajectory
from mpc.ordered_bubble_ocp import (
    OrderedBubbleMPC,
    OrderedBubbleMPCConfig,
    sample_stage_bubbles,
)
from policy.state_transform import rotate_body2world, state_body2world
from policy.yopo_dataset import YOPODataset


def _project_to_route(point: np.ndarray, route: np.ndarray) -> tuple[float, np.ndarray]:
    segment = route[1:] - route[:-1]
    length_squared = np.maximum(np.sum(segment * segment, axis=1), 1.0e-10)
    alpha = np.clip(
        np.sum((point[None] - route[:-1]) * segment, axis=1) / length_squared,
        0.0,
        1.0,
    )
    closest = route[:-1] + alpha[:, None] * segment
    index = int(np.argmin(np.linalg.norm(point[None] - closest, axis=1)))
    cumulative, _ = polyline_arclength(route)
    progress = float(cumulative[index] + alpha[index] * math.sqrt(length_squared[index]))
    tangent = segment[index] / math.sqrt(length_squared[index])
    return progress, tangent


def _dense_mpc_positions(
    states: np.ndarray,
    controls: np.ndarray,
    *,
    dt: float,
    samples_per_stage: int = 8,
) -> np.ndarray:
    pieces: list[np.ndarray] = [states[0, :3][None]]
    local_times = np.linspace(0.0, dt, samples_per_stage + 1, dtype=np.float64)[1:]
    for stage, jerk in enumerate(controls):
        position = states[stage, :3]
        velocity = states[stage, 3:6]
        acceleration = states[stage, 6:9]
        segment = (
            position[None]
            + local_times[:, None] * velocity[None]
            + 0.5 * local_times[:, None] ** 2 * acceleration[None]
            + (local_times[:, None] ** 3 / 6.0) * jerk[None]
        )
        pieces.append(segment)
    return np.concatenate(pieces, axis=0).astype(np.float32)


def _candidate_corridors(
    route: np.ndarray,
    route_radii: np.ndarray,
    initial_velocity: np.ndarray,
    terminal_states: np.ndarray,
    config: OrderedBubbleMPCConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    first_segment = route[1] - route[0]
    first_tangent = first_segment / max(float(np.linalg.norm(first_segment)), 1.0e-8)
    initial_speed = max(0.0, float(np.dot(initial_velocity, first_tangent)))
    centers = []
    radii = []
    progress = []
    _, path_length = polyline_arclength(route)
    for terminal in terminal_states:
        projected_progress, terminal_tangent = _project_to_route(terminal[:3], route)
        projected_progress = float(np.clip(projected_progress, 0.5, min(10.0, path_length)))
        terminal_speed = max(0.0, float(np.dot(terminal[3:6], terminal_tangent)))
        candidate_centers, candidate_radii = sample_stage_bubbles(
            route,
            route_radii,
            horizon_steps=config.horizon_steps,
            travel_distance_m=projected_progress,
            horizon_time_s=config.horizon_time_s,
            initial_speed_mps=initial_speed,
            terminal_speed_mps=terminal_speed,
        )
        centers.append(candidate_centers)
        radii.append(candidate_radii)
        progress.append(projected_progress)
    return np.stack(centers), np.stack(radii), np.asarray(progress)


def _prediction_payload(
    certified: dict[str, Any], raw: dict[str, Any], baseline: dict[str, Any]
) -> dict[str, Any]:
    return {
        **certified,
        "previousPath": raw["path"],
        "previousMinimumClearanceM": raw["minimumClearanceM"],
        "previousCollision": raw["collision"],
        "previousRouteProgressM": raw["routeProgressM"],
        "baselinePath": baseline["path"],
        "baselineMinimumClearanceM": baseline["minimumClearanceM"],
        "baselineCollision": baseline["collision"],
        "baselineRouteProgressM": baseline["routeProgressM"],
    }


def evaluate(
    data_root: Path,
    checkpoint: Path,
    output_dir: Path,
    *,
    max_samples: int | None = 50,
    inference_batch_size: int = 16,
    device: str | None = None,
    build_viewer: bool = True,
) -> Path:
    data_root = data_root.resolve()
    checkpoint = checkpoint.resolve()
    output_dir = output_dir.resolve()
    selected_device = torch.device(
        device or ("cuda" if torch.cuda.is_available() else "cpu")
    )
    dataset = YOPODataset("all", data_root=data_root)
    if max_samples is None or max_samples >= len(dataset.samples):
        samples = list(dataset.samples)
        sample_selection = "all"
    else:
        selected_indices = np.unique(
            np.linspace(0, len(dataset.samples) - 1, max_samples, dtype=np.int64)
        )
        samples = [dataset.samples[int(index)] for index in selected_indices]
        sample_selection = "evenly_spaced_across_all_valid_routes"
    dataset.samples = list(samples)
    loader = DataLoader(dataset, batch_size=inference_batch_size, shuffle=False, num_workers=0)
    baseline = _load_baseline(checkpoint, selected_device)
    # The bubble is a safety set, while its ordered center is the route timing
    # reference.  A meaningful center weight prevents dynamically smooth
    # trajectories from cutting between large neighboring bubbles.
    # Match the sampled YOPO state envelope.  The previous 24/8 limits could
    # not redirect a 7.2 m/s, 7.2 m/s^2 initial state before the first obstacle.
    config = OrderedBubbleMPCConfig(
        horizon_steps=12,
        max_velocity_mps=9.0,
        max_acceleration_mps2=12.0,
        max_jerk_mps3=40.0,
    )
    mpc = OrderedBubbleMPC(
        config,
        batch_size=baseline.trajectory_count,
        model_name="yopo_simple_ordered_bubble_eval",
    )
    obstacle_trees = [
        cKDTree(read_ascii_point_cloud_ply(scene.path / "tree.ply"))
        for scene in dataset.scenes
    ]
    safety_radius = float(cfg["robot_radius_m"] + cfg["safety_margin_m"])

    baseline_values: list[dict[str, Any]] = []
    raw_mpc_values: list[dict[str, Any]] = []
    certified_values: list[dict[str, Any]] = []
    predictions: dict[tuple[str, int], dict[str, Any]] = {}
    statuses: list[int] = []
    solve_times_ms: list[float] = []
    no_certified_candidate_count = 0
    no_mpc_but_original_safe_count = 0
    no_original_safe_candidate_count = 0
    cursor = 0
    with torch.inference_mode():
        for batch in loader:
            count = batch["depth"].shape[0]
            depth = batch["depth"].to(selected_device)
            motion = batch["motion_body"].to(selected_device)
            goal = batch["frontier_body"].to(selected_device)
            end_body, score = baseline(depth, motion, goal)
            candidates_body = end_body.permute(0, 2, 3, 1).reshape(count, -1, 9)
            candidate_scores = score.reshape(count, -1)
            position = batch["position_world"].to(selected_device)
            rotation = batch["rotation_world_body"].to(selected_device)
            repeated_position = position.repeat_interleave(baseline.trajectory_count, dim=0)
            repeated_rotation = rotation.repeat_interleave(baseline.trajectory_count, dim=0)
            flat = candidates_body.reshape(-1, 9)
            end_position, end_velocity, end_acceleration = state_body2world(
                repeated_position,
                repeated_rotation,
                flat[:, :3],
                flat[:, 3:6],
                flat[:, 6:9],
            )
            candidates_world = (
                torch.cat((end_position, end_velocity, end_acceleration), dim=1)
                .reshape(count, baseline.trajectory_count, 9)
                .cpu()
                .numpy()
            )
            start_velocity = rotate_body2world(rotation, motion[:, :3]).cpu().numpy()
            start_acceleration = rotate_body2world(rotation, motion[:, 3:]).cpu().numpy()
            start_position = position.cpu().numpy()
            scores_np = candidate_scores.cpu().numpy()

            for local in range(count):
                scene_index, route_index = samples[cursor + local]
                scene = dataset.scenes[scene_index]
                route, _, route_radii = scene.routes.path(route_index)
                tree = obstacle_trees[scene_index]
                initial = np.concatenate(
                    (start_position[local], start_velocity[local], start_acceleration[local])
                )
                terminals = candidates_world[local]
                original_paths = [
                    _sample_trajectory(
                        initial.reshape(3, 3), terminal.reshape(3, 3)
                    )
                    for terminal in terminals
                ]
                original_clearance = np.asarray(
                    [tree.query(path, k=1)[0].min() for path in original_paths]
                )
                original_safe = original_clearance >= safety_radius
                centers, stage_radii, proposal_progress = _candidate_corridors(
                    route, route_radii, start_velocity[local], terminals, config
                )
                initial_batch = np.repeat(initial[None], baseline.trajectory_count, axis=0)
                solve_start = time.perf_counter()
                ctx, _, states, controls, _ = mpc(
                    initial_batch, terminals, centers, stage_radii
                )
                solve_times_ms.append((time.perf_counter() - solve_start) * 1.0e3)
                states_np = states.detach().cpu().numpy()
                controls_np = controls.detach().cpu().numpy()
                status = np.asarray(ctx.status, dtype=np.int64)
                statuses.extend(status.tolist())
                mpc_paths = [
                    _dense_mpc_positions(states_np[index], controls_np[index], dt=config.dt)
                    for index in range(baseline.trajectory_count)
                ]
                clearance = np.asarray(
                    [tree.query(path, k=1)[0].min() for path in mpc_paths], dtype=np.float64
                )
                raw_index = int(np.argmin(scores_np[local]))
                certified_mask = (status == 0) & (clearance >= safety_radius)
                original_fallback_index = None
                if certified_mask.any():
                    certified_scores = np.where(certified_mask, scores_np[local], np.inf)
                    certified_index = int(np.argmin(certified_scores))
                else:
                    no_certified_candidate_count += 1
                    if original_safe.any():
                        no_mpc_but_original_safe_count += 1
                        # Never replace a point-cloud-certified original
                        # polynomial with a less safe optimizer output.
                        original_fallback_index = int(
                            np.argmin(np.where(original_safe, scores_np[local], np.inf))
                        )
                        certified_index = original_fallback_index
                    else:
                        no_original_safe_candidate_count += 1
                    if original_fallback_index is None:
                        # There is no safe output for this sample. Keep a
                        # deterministic top-1 placeholder for reporting;
                        # callers must use the no-certified counter rather
                        # than treating this path as executable.
                        certified_index = raw_index

                baseline_path = original_paths[raw_index]
                baseline_result = _single_metrics(
                    baseline_path, route, route_radii, tree
                )
                raw_result = _single_metrics(
                    mpc_paths[raw_index], route, route_radii, tree
                )
                certified_result = _single_metrics(
                    original_paths[certified_index]
                    if original_fallback_index is not None
                    else mpc_paths[certified_index],
                    route,
                    route_radii,
                    tree,
                )
                for result, index in (
                    (raw_result, raw_index),
                    (certified_result, certified_index),
                ):
                    result["primitiveIndex"] = index
                    result["score"] = round(float(scores_np[local, index]), 6)
                    result["solverStatus"] = int(status[index])
                    result["proposalProgressM"] = round(float(proposal_progress[index]), 4)
                baseline_result["primitiveIndex"] = raw_index
                baseline_result["score"] = round(float(scores_np[local, raw_index]), 6)
                if original_fallback_index is not None:
                    certified_result["primitiveIndex"] = original_fallback_index
                    certified_result["score"] = round(
                        float(scores_np[local, original_fallback_index]), 6
                    )
                    certified_result["solverStatus"] = None
                    certified_result["proposalProgressM"] = None
                    certified_result["selectionSource"] = "original_yopo_safety_fallback"
                else:
                    certified_result["selectionSource"] = (
                        "yopo_candidate" if certified_mask.any() else "no_certified_candidate"
                    )
                baseline_values.append(baseline_result)
                raw_mpc_values.append(raw_result)
                certified_values.append(certified_result)
                predictions[(scene.path.name, route_index)] = _prediction_payload(
                    certified_result, raw_result, baseline_result
                )
                predictions[(scene.path.name, route_index)]["originalSafeCandidateCount"] = int(
                    original_safe.sum()
                )
            cursor += count
            print(f"evaluated {cursor}/{len(samples)} routes", flush=True)

    status_array = np.asarray(statuses)
    report = {
        "benchmark": "yopo_simple_ordered_bubble_mpc_004_dynamics_fixed",
        "dataset": str(data_root),
        "sampleCount": len(samples),
        "sampleSelection": sample_selection,
        "checkpoint": str(checkpoint),
        "device": str(selected_device),
        "inputContract": {
            "yopoInputs": ["depth", "motion", "route-derived local goal"],
            "routeEntersYopo": False,
            "mpcInputs": ["YOPO terminal proposal", "ordered route bubbles"],
            "pointCloudCertificationOutsideMpc": True,
        },
        "models": {
            "routeYopo": _aggregate(certified_values),
            "previousRouteYopo": _aggregate(raw_mpc_values),
            "yopoSimple": _aggregate(baseline_values),
        },
        "modelLabels": {
            "routeYopo": "YOPO-Simple + bubble MPC + point-cloud certification",
            "previousRouteYopo": "YOPO-Simple top-1 + bubble MPC",
            "yopoSimple": "original YOPO-Simple fifth-order polynomial",
        },
        "solver": {
            "horizonSteps": config.horizon_steps,
            "horizonTimeS": config.horizon_time_s,
            "candidateCount": baseline.trajectory_count,
            "solveSuccessRate": float(np.mean(status_array == 0)),
            "failedCandidateCount": int(np.sum(status_array != 0)),
            "noCertifiedCandidateCount": no_certified_candidate_count,
            "noCertifiedCandidateRate": no_certified_candidate_count / len(samples),
            "noMpcButOriginalSafeCandidateCount": no_mpc_but_original_safe_count,
            "noOriginalSafeCandidateCount": no_original_safe_candidate_count,
            "batchLatencyMsMean": float(np.mean(solve_times_ms)),
            "batchLatencyMsP95": float(np.percentile(solve_times_ms, 95)),
            "perCandidateLatencyMsMean": float(
                np.mean(solve_times_ms) / baseline.trajectory_count
            ),
        },
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "comparison_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (output_dir / "comparison_predictions.json").write_text(
        json.dumps(
            {f"{scene}/{route}": value for (scene, route), value in predictions.items()},
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if build_viewer:
        viewer_path = build_dataset_viewer(
            data_root,
            output_dir=output_dir / "viewer",
            overwrite=True,
            predictions=predictions,
            evaluation_report=report,
        )
        html = viewer_path.read_text(encoding="utf-8")
        html = html.replace("Route-YOPO", "YOPO + bubble MPC (certified)")
        html = html.replace("Previous YOPO + bubble MPC (certified)", "YOPO + bubble MPC top-1")
        html = html.replace("previous YOPO + bubble MPC (certified)", "YOPO + bubble MPC top-1")
        viewer_path.write_text(html, encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=Path("/mnt/code/lab/yopo/YOPO-Simple/YOPO/saved/YOPO_1/epoch50.pth"),
    )
    parser.add_argument(
        "--output", type=Path, default=Path("tmp/mpc_002_yopo_simple")
    )
    parser.add_argument("--max-samples", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--device", choices=("cpu", "cuda"))
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args()
    evaluate(
        args.data,
        args.checkpoint,
        args.output,
        max_samples=args.max_samples,
        inference_batch_size=args.batch_size,
        device=args.device,
        build_viewer=not args.report_only,
    )


if __name__ == "__main__":
    main()
