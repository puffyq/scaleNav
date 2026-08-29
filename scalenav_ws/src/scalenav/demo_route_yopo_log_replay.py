#!/usr/bin/env python3
"""Replay one Route-YOPO planning frame from a scalenav_log.v2 session.

The demo compares the current unconstrained 3-D primitives with the same model
outputs projected onto the fixed route altitude. It does not publish ROS
commands or modify the recorded session.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Sequence

import cv2
import numpy as np
import torch

from graph.depth_query import DepthSafeVolumeQuery
from route_yopo_control_core import (
    build_route_features,
    clip_goal_to_camera_fov,
    enforce_route_progress,
    project_endstates_to_altitude,
    quaternion_xyzw_to_matrix,
    reanchor_route_path,
    sample_poly5_candidate_states,
    select_first_certified,
    validate_depth_trajectory,
    validate_route_corridor,
    world_to_body_flu,
)


def load_index(session: Path) -> list[dict[str, Any]]:
    index_path = session / "index.jsonl"
    if not index_path.is_file():
        raise FileNotFoundError(f"session index not found: {index_path}")
    with index_path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def select_depth_record(
    records: Sequence[dict[str, Any]],
    *,
    depth_file: str | None,
    seconds_after_goal: float,
) -> dict[str, Any]:
    depth_records = [
        record
        for record in records
        if record.get("kind") == "depth" and record.get("file")
    ]
    if depth_file is not None:
        normalized = depth_file.removeprefix("./")
        for record in depth_records:
            if record["file"] == normalized or Path(record["file"]).name == Path(normalized).name:
                return record
        raise ValueError(f"depth frame not found in session index: {depth_file}")
    goals = [record for record in records if record.get("kind") == "goal"]
    if not goals:
        raise ValueError("session contains no mission goal")
    target_stamp = int(goals[0]["stamp_ns"] + seconds_after_goal * 1.0e9)
    candidates = [record for record in depth_records if record["stamp_ns"] >= goals[0]["stamp_ns"]]
    if not candidates:
        raise ValueError("session contains no depth frame after the mission goal")
    return min(candidates, key=lambda record: abs(int(record["stamp_ns"]) - target_stamp))


def latest_record_before(
    records: Sequence[dict[str, Any]], kind: str, stamp_ns: int
) -> dict[str, Any]:
    candidates = [
        record
        for record in records
        if record.get("kind") == kind and int(record.get("stamp_ns", -1)) <= stamp_ns
    ]
    if not candidates:
        raise ValueError(f"no {kind} record exists at or before the replay frame")
    return max(candidates, key=lambda record: int(record["stamp_ns"]))


def load_frontier(
    session: Path,
    records: Sequence[dict[str, Any]],
    stamp_ns: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    graphs = sorted(
        (
            record
            for record in records
            if record.get("kind") == "graph"
            and record.get("file")
            and int(record.get("stamp_ns", -1)) <= stamp_ns
        ),
        key=lambda record: int(record["stamp_ns"]),
        reverse=True,
    )
    for record in graphs:
        payload = json.loads((session / record["file"]).read_text(encoding="utf-8"))
        for marker in payload.get("markers", []):
            if marker.get("ns") != "scalenav_frontier_goal" or marker.get("action") != 0:
                continue
            values = np.asarray(marker["pose"]["position"], dtype=np.float64)
            if values.shape == (3,) and np.isfinite(values).all():
                return values, record
    raise ValueError("no active scalenav_frontier_goal marker exists before the replay frame")


def reconstruct_motion(
    records: Sequence[dict[str, Any]], stamp_ns: int, acceleration_limit: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    odometry = sorted(
        (
            record
            for record in records
            if record.get("kind") == "odom" and int(record.get("stamp_ns", -1)) <= stamp_ns
        ),
        key=lambda record: int(record["stamp_ns"]),
    )
    if not odometry:
        raise ValueError("no odometry exists before the replay frame")
    acceleration_world = np.zeros(3, dtype=np.float32)
    previous_velocity: np.ndarray | None = None
    previous_stamp: int | None = None
    velocity_world = np.zeros(3, dtype=np.float32)
    for record in odometry:
        data = record["data"]
        rotation = quaternion_xyzw_to_matrix(data["orientation"])
        velocity_world = (
            rotation @ np.asarray(data["velocity"], dtype=np.float64)
        ).astype(np.float32)
        if previous_velocity is not None and previous_stamp is not None:
            elapsed = (int(record["stamp_ns"]) - previous_stamp) * 1.0e-9
            if 0.002 <= elapsed <= 0.2:
                measured = np.clip(
                    (velocity_world - previous_velocity) / elapsed,
                    -acceleration_limit,
                    acceleration_limit,
                )
                acceleration_world = (
                    0.85 * acceleration_world + 0.15 * measured
                ).astype(np.float32)
        previous_velocity = velocity_world
        previous_stamp = int(record["stamp_ns"])

    selected = odometry[-1]
    data = selected["data"]
    position_world = np.asarray(data["position"], dtype=np.float64)
    rotation_body_to_world = quaternion_xyzw_to_matrix(data["orientation"])
    return (
        position_world,
        rotation_body_to_world,
        velocity_world,
        acceleration_world,
        selected,
    )


def decode_logged_depth(path: Path) -> np.ndarray:
    encoded = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if encoded is None or encoded.ndim != 2:
        raise ValueError(f"failed to decode logged depth image: {path}")
    if encoded.dtype == np.uint16:
        return encoded.astype(np.float32) * 0.001
    return encoded.astype(np.float32)


def model_depth(raw_depth_m: np.ndarray, minimum_depth_m: float, max_depth_m: float) -> np.ndarray:
    normalized = np.minimum(raw_depth_m, max_depth_m) / max_depth_m
    invalid = ~np.isfinite(normalized) | (normalized < minimum_depth_m / max_depth_m)
    return cv2.inpaint(
        np.uint8(
            np.nan_to_num(normalized, nan=0.0, posinf=1.0, neginf=0.0) * 255.0
        ),
        np.uint8(invalid),
        1,
        cv2.INPAINT_NS,
    ).astype(np.float32) / 255.0


def safety_summary(safety: Sequence[dict[str, Any]]) -> dict[str, int]:
    names = (
        "CERTIFIED",
        "INVALID",
        "UNVALIDATED",
        "ALTITUDE",
        "ROUTE_ALTITUDE",
        "ROUTE_CORRIDOR",
        "NON_FINITE",
    )
    return {name: sum(item["state"] == name for item in safety) for name in names}


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "session",
        nargs="?",
        default=str(root / "log_scalenav/session_20260828_190205_247"),
        help="scalenav_log.v2 session directory",
    )
    parser.add_argument("--depth-file", help="exact or basename of a recorded depth frame")
    parser.add_argument("--seconds-after-goal", type=float, default=3.0)
    parser.add_argument(
        "--model",
        default=str(root / "train_scalenav/saved_fixed_altitude/YOPO_0/best.pth"),
    )
    parser.add_argument("--train-root", default=str(root / "train_scalenav"))
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--robot-radius", type=float, default=0.3)
    parser.add_argument("--safety-margin", type=float, default=0.2)
    parser.add_argument("--minimum-altitude", type=float, default=0.25)
    parser.add_argument("--minimum-depth", type=float, default=0.04)
    parser.add_argument("--max-depth", type=float, default=20.0)
    parser.add_argument("--source-horizontal-fov", type=float, default=90.0)
    parser.add_argument("--source-vertical-fov", type=float, default=73.7398)
    parser.add_argument("--camera-translation-flu", type=float, nargs=3, default=(0.5, 0.0, -0.1))
    parser.add_argument("--route-start-tolerance", type=float, default=1.5)
    parser.add_argument("--route-reanchor-tolerance", type=float, default=5.0)
    parser.add_argument("--route-terminal-tolerance", type=float, default=2.0)
    parser.add_argument("--route-altitude-tolerance", type=float, default=0.25)
    parser.add_argument("--route-corridor-tracking-tolerance", type=float, default=0.1)
    parser.add_argument("--output", help="optional JSON output path")
    args = parser.parse_args()
    if args.seconds_after_goal < 0.0:
        parser.error("seconds after goal must be non-negative")
    if args.robot_radius <= 0.0 or args.safety_margin < 0.0:
        parser.error("robot radius and safety margin are invalid")
    if not 0.0 <= args.route_corridor_tracking_tolerance <= args.safety_margin:
        parser.error("route corridor tracking tolerance must be within the safety margin")
    return args


def main() -> None:
    args = parse_args()
    session = Path(args.session).expanduser().resolve()
    train_root = Path(args.train_root).expanduser().resolve()
    checkpoint_path = Path(args.model).expanduser().resolve()
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Route-YOPO checkpoint not found: {checkpoint_path}")
    train_path = str(train_root)
    sys.path[:] = [entry for entry in sys.path if entry != train_path]
    sys.path.insert(0, train_path)
    from config.config import cfg
    from data.route_contract import sample_route_bubbles
    from policy.yopo_network import YopoNetwork

    records = load_index(session)
    goal_record = min(
        (record for record in records if record.get("kind") == "goal"),
        key=lambda record: int(record["stamp_ns"]),
    )
    depth_record = select_depth_record(
        records,
        depth_file=args.depth_file,
        seconds_after_goal=args.seconds_after_goal,
    )
    stamp_ns = int(depth_record["stamp_ns"])
    position, rotation, velocity, acceleration, odom_record = reconstruct_motion(
        records, stamp_ns, acceleration_limit=6.0
    )
    path_record = latest_record_before(records, "path", stamp_ns)
    path_payload = json.loads((session / path_record["file"]).read_text(encoding="utf-8"))
    path_world = np.asarray(path_payload["poses"], dtype=np.float64)
    clearance_record = latest_record_before(records, "clearance", stamp_ns)
    clearance_m = float(clearance_record["data"]["global_witness_min_m"])
    frontier_world, graph_record = load_frontier(session, records, stamp_ns)

    route_safe_radius = clearance_m - args.robot_radius - args.safety_margin
    route_start_error = float(np.linalg.norm(path_world[0] - position))
    route_terminal_error = float(np.linalg.norm(path_world[-1] - frontier_world))
    route_valid = bool(
        path_world.ndim == 2
        and path_world.shape[1:] == (3,)
        and len(path_world) >= 2
        and np.isfinite(path_world).all()
        and route_terminal_error <= args.route_terminal_tolerance
    )
    if route_valid and route_start_error > args.route_start_tolerance:
        anchored, _ = reanchor_route_path(
            path_world,
            position,
            maximum_distance_m=args.route_reanchor_tolerance,
        )
        if anchored is None:
            route_valid = False
        else:
            path_world = anchored
            route_start_error = 0.0
    route_corridor_enabled = route_valid and route_safe_radius > 0.0
    if route_valid and route_start_error > 1.0e-3:
        # Match the online adapter: the live vehicle pose is the first cell of
        # the corridor map, so the short hand-off into the witness is covered.
        path_world = np.concatenate((position[None], path_world), axis=0)
    anchors = np.asarray(cfg["route_anchor_distances_m"], dtype=np.float32)
    if route_valid:
        point_radii = np.full(len(path_world), max(route_safe_radius, 0.25), dtype=np.float32)
        centers, radii, mask, distances = sample_route_bubbles(
            path_world, point_radii, anchors
        )
        route_features, route_mask = build_route_features(
            centers,
            radii,
            mask,
            distances,
            position,
            rotation,
            radius_clip_m=float(cfg["route_clearance_clip_m"]),
        )
    else:
        route_features = np.zeros((len(anchors), 4), dtype=np.float32)
        route_mask = np.zeros(len(anchors), dtype=np.float32)

    raw_depth = decode_logged_depth(session / depth_record["file"])
    prepared_depth = model_depth(raw_depth, args.minimum_depth, args.max_depth)
    motion_body = np.concatenate((rotation.T @ velocity, rotation.T @ acceleration)).astype(
        np.float32
    )
    frontier_body = clip_goal_to_camera_fov(
        world_to_body_flu(frontier_world, position, rotation),
        horizontal_fov_deg=args.source_horizontal_fov,
        vertical_fov_deg=args.source_vertical_fov,
    ).astype(np.float32)
    requested_device = args.device
    if requested_device == "auto":
        requested_device = "cuda" if torch.cuda.is_available() else "cpu"
    if requested_device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    device = torch.device(requested_device)
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model = YopoNetwork().to(device).eval()
    feature_order = model.load_route_checkpoint(checkpoint)
    with torch.inference_mode():
        endstate, score = model(
            torch.from_numpy(prepared_depth[None, None]).to(device),
            torch.from_numpy(motion_body[None]).to(device),
            torch.from_numpy(frontier_body[None]).to(device),
            torch.from_numpy(route_features[None]).to(device),
            torch.from_numpy(route_mask[None]).to(device),
        )
    endstates = endstate[0].permute(1, 2, 0).reshape(-1, 9).cpu().numpy()
    scores = score[0].reshape(-1).cpu().numpy()
    if route_valid:
        endstates = enforce_route_progress(
            endstates,
            frontier_body,
            minimum_forward_m=2.0,
            maximum_forward_m=8.0,
        )
    route_altitude = float(frontier_world[2])
    fixed_endstates = project_endstates_to_altitude(
        endstates, position, rotation, route_altitude
    )
    fixed_endpoints_world = position[None] + fixed_endstates[:, :3] @ rotation.T

    def evaluate(states: np.ndarray, *, enforce_route_altitude: bool):
        trajectories, _, _ = sample_poly5_candidate_states(
            position,
            velocity,
            acceleration,
            states,
            rotation,
            segment_time_s=float(cfg["sgm_time"]),
            sample_count=101,
        )
        query = DepthSafeVolumeQuery(
            raw_depth,
            horizontal_fov_deg=args.source_horizontal_fov,
            vertical_fov_deg=args.source_vertical_fov,
            robot_radius_m=args.robot_radius,
            safety_margin_m=args.safety_margin,
            sample_step_m=0.2,
            far_depth_m=args.max_depth,
            max_unknown_fraction=0.2,
        )
        camera_world = position + rotation @ np.asarray(
            args.camera_translation_flu, dtype=np.float64
        )
        safety = []
        for trajectory in trajectories:
            corridor = (
                validate_route_corridor(
                    trajectory,
                    path_world,
                    route_safe_radius,
                    args.route_corridor_tracking_tolerance,
                )
                if enforce_route_altitude and route_corridor_enabled
                else None
            )
            result = validate_depth_trajectory(
                query,
                trajectory,
                camera_world,
                rotation,
                minimum_altitude_m=args.minimum_altitude,
                route_altitude_m=route_altitude if enforce_route_altitude else None,
                route_altitude_tolerance_m=args.route_altitude_tolerance
                if enforce_route_altitude
                else None,
            )
            if result["state"] == "UNVALIDATED" and corridor is not None and corridor["state"] == "CERTIFIED":
                result = dict(result)
                result["state"] = "CERTIFIED"
                result["validation_source"] = "route_corridor_map_unknown_depth"
            if corridor is not None:
                result.update(
                    {
                        name: value
                        for name, value in corridor.items()
                        if name != "state"
                    }
                )
            safety.append(result)
        selected = select_first_certified(
            scores,
            trajectories,
            [item["state"] for item in safety],
            minimum_altitude_m=args.minimum_altitude,
        )
        return trajectories, safety, selected

    free_trajectories, free_safety, free_selected = evaluate(
        endstates, enforce_route_altitude=False
    )
    fixed_trajectories, fixed_safety, fixed_selected = evaluate(
        fixed_endstates, enforce_route_altitude=True
    )
    fixed_altitude_errors = np.max(
        np.abs(fixed_trajectories[:, :, 2] - route_altitude), axis=1
    )
    candidates = []
    for index, candidate_score in enumerate(scores):
        candidates.append(
            {
                "primitive": index,
                "score": float(candidate_score),
                "unconstrained_endpoint_z_m": float(free_trajectories[index, -1, 2]),
                "fixed_endpoint_z_m": float(fixed_trajectories[index, -1, 2]),
                "fixed_endpoint_world_m": fixed_endpoints_world[index].tolist(),
                "fixed_endpoint_forward_m": float(
                    np.dot(
                        fixed_endpoints_world[index] - position,
                        frontier_world - position,
                    )
                    / max(float(np.linalg.norm(frontier_world - position)), 1.0e-6)
                ),
                "unconstrained_safety": free_safety[index]["state"],
                "fixed_safety": fixed_safety[index]["state"],
                "fixed_minimum_corridor_margin_m": fixed_safety[index].get(
                    "minimum_corridor_margin_m"
                ),
                "fixed_maximum_corridor_violation_m": fixed_safety[index].get(
                    "maximum_corridor_violation_m"
                ),
                "fixed_minimum_clearance_m": fixed_safety[index].get(
                    "minimum_clearance_m"
                ),
                "fixed_known_fraction": fixed_safety[index].get("known_fraction"),
                "fixed_checked_samples": fixed_safety[index].get("checked_samples"),
                "fixed_swept_radius_m": fixed_safety[index].get("swept_radius_m"),
            }
        )

    report = {
        "demo": "Route-YOPO logged-frame fixed-altitude comparison",
        "session": str(session),
        "depth_frame": depth_record["file"],
        "stamp_ns": stamp_ns,
        "requested_seconds_after_goal": args.seconds_after_goal,
        "actual_seconds_after_goal": (
            stamp_ns - int(goal_record["stamp_ns"])
        )
        * 1.0e-9,
        "checkpoint": str(checkpoint_path),
        "feature_order": feature_order,
        "device": str(device),
        "inputs": {
            "position_world_m": position.tolist(),
            "velocity_world_mps": velocity.tolist(),
            "acceleration_world_mps2": acceleration.tolist(),
            "frontier_world_m": frontier_world.tolist(),
            "route_altitude_m": route_altitude,
            "path_altitude_min_max_m": [
                float(np.min(path_world[:, 2])),
                float(np.max(path_world[:, 2])),
            ],
            "global_witness_clearance_m": clearance_m,
            "route_safe_radius_m": route_safe_radius,
            "route_corridor_tracking_tolerance_m": args.route_corridor_tracking_tolerance,
            "route_start_error_m": route_start_error,
            "route_terminal_error_m": route_terminal_error,
            "route_features_active": route_valid,
            "route_corridor_enabled": route_corridor_enabled,
            "depth_to_odom_stamp_delta_ms": (
                stamp_ns - int(odom_record["stamp_ns"])
            )
            / 1.0e6,
            "source_files": {
                "path": path_record["file"],
                "graph": graph_record["file"],
                "depth": depth_record["file"],
            },
        },
        "unconstrained": {
            "score_argmin": int(np.argmin(scores)),
            "selected_certified": free_selected,
            "selected_endpoint_z_m": None
            if free_selected is None
            else float(free_trajectories[free_selected, -1, 2]),
            "endpoint_z_min_max_m": [
                float(np.min(free_trajectories[:, -1, 2])),
                float(np.max(free_trajectories[:, -1, 2])),
            ],
            "safety_counts": safety_summary(free_safety),
        },
        "fixed_altitude": {
            "selected_certified": fixed_selected,
            "selected_endpoint_z_m": None
            if fixed_selected is None
            else float(fixed_trajectories[fixed_selected, -1, 2]),
            "endpoint_z_min_max_m": [
                float(np.min(fixed_trajectories[:, -1, 2])),
                float(np.max(fixed_trajectories[:, -1, 2])),
            ],
            "trajectory_altitude_error_min_max_m": [
                float(np.min(fixed_altitude_errors)),
                float(np.max(fixed_altitude_errors)),
            ],
            "safety_counts": safety_summary(fixed_safety),
        },
        "candidates": candidates,
    }
    output = json.dumps(report, indent=2, ensure_ascii=True)
    print(output)
    if args.output:
        output_path = Path(args.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(output + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
