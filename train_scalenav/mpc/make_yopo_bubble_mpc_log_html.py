#!/usr/bin/env python3
"""Replay a recorded flight through YOPO and ordered-bubble MPC into HTML.

The route is used only by the MPC.  Panel one therefore shows the unmodified
YOPO-Simple output, panel two shows the ordered route bubbles, and panel three
shows the MPC result for the same fifteen terminal proposals.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path
import sys
from typing import Any, Sequence

import numpy as np
import torch

try:
    import cv2
except ModuleNotFoundError:  # leap-c's Python environment has no OpenCV.
    cv2 = None
    from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
TRAIN_ROOT = ROOT / "train_scalenav"
SCALENAV_ROOT = ROOT / "scalenav_ws/src/scalenav"
for import_path in (str(TRAIN_ROOT), str(SCALENAV_ROOT)):
    if import_path not in sys.path:
        sys.path.insert(0, import_path)

from config.config import cfg  # noqa: E402
from mpc.ordered_bubble_ocp import (  # noqa: E402
    OrderedBubbleMPC,
    OrderedBubbleMPCConfig,
    maximum_bubble_violation,
    maximum_reachable_progress,
    project_path_progress,
    resolve_target_progress,
    sample_reachable_stage_bubbles,
)
from route_yopo_control_core import (  # noqa: E402
    clip_goal_to_camera_fov,
    enforce_route_progress,
    project_endstates_to_altitude,
    quaternion_xyzw_to_matrix,
    reanchor_route_path,
    trim_route_for_motion,
    sample_poly5_candidate_states,
    world_to_body_flu,
)
from yopo_inference_scaling import YopoInferenceScaling  # noqa: E402


def _records(session: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in (session / "index.jsonl").open(encoding="utf-8")
        if line.strip()
    ]


class Timeline:
    def __init__(self, records: Sequence[dict[str, Any]], kind: str) -> None:
        self.items = sorted(
            (item for item in records if item.get("kind") == kind),
            key=lambda item: int(item["stamp_ns"]),
        )
        self.stamps = [int(item["stamp_ns"]) for item in self.items]

    def before(self, stamp_ns: int) -> dict[str, Any] | None:
        index = bisect_right(self.stamps, stamp_ns) - 1
        return None if index < 0 else self.items[index]

    def recent(self, stamp_ns: int, count: int) -> list[dict[str, Any]]:
        end = bisect_right(self.stamps, stamp_ns)
        return self.items[max(0, end - count) : end]

    def nearest(self, stamp_ns: int) -> dict[str, Any] | None:
        """Return the timestamp-nearest record, including one after stamp_ns."""
        if not self.items:
            return None
        index = bisect_right(self.stamps, stamp_ns)
        candidates = []
        if index:
            candidates.append(self.items[index - 1])
        if index < len(self.items):
            candidates.append(self.items[index])
        return min(candidates, key=lambda item: abs(int(item["stamp_ns"]) - stamp_ns))


def _load_points(session: Path, record: dict[str, Any] | None) -> np.ndarray:
    if record is None or not record.get("file"):
        return np.empty((0, 3), dtype=np.float64)
    payload = json.loads((session / record["file"]).read_text(encoding="utf-8"))
    return np.asarray(payload.get("poses", payload.get("points", [])), dtype=np.float64)


def _load_pcd_xyz(path: Path) -> np.ndarray:
    """Read x/y/z from the ASCII or binary PCD files written by scalenav_log."""
    blob = path.read_bytes()
    marker = b"\nDATA "
    header_end = blob.find(marker)
    if header_end < 0:
        return np.empty((0, 3), dtype=np.float64)
    data_start = blob.find(b"\n", header_end + 1)
    if data_start < 0:
        return np.empty((0, 3), dtype=np.float64)
    header = blob[:data_start].decode("ascii", errors="ignore").splitlines()
    values: dict[str, list[str]] = {}
    for line in header:
        parts = line.strip().split()
        if parts and not parts[0].startswith("#"):
            values[parts[0].upper()] = parts[1:]
    fields = values.get("FIELDS", [])
    if not fields or not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)
    counts = [int(value) for value in values.get("COUNT", ["1"] * len(fields))]
    sizes = [int(value) for value in values.get("SIZE", ["4"] * len(fields))]
    types = values.get("TYPE", ["F"] * len(fields))
    offsets = np.cumsum([0] + [size * count for size, count in zip(sizes, counts)])[:-1]
    xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
    data_mode = values.get("DATA", [""])[0].lower()
    payload = blob[data_start + 1 :]
    point_count = int(values.get("POINTS", values.get("WIDTH", ["0"]))[0])
    if data_mode == "ascii":
        # The logger writes one scalar per field and uses finite xyz only.
        raw = np.fromstring(payload.decode("ascii", errors="ignore"), sep=" ", dtype=np.float64)
        scalar_count = sum(counts)
        if scalar_count <= 0:
            return np.empty((0, 3), dtype=np.float64)
        row_count = min(point_count, raw.size // scalar_count) if point_count else raw.size // scalar_count
        if row_count <= 0:
            return np.empty((0, 3), dtype=np.float64)
        rows = raw[: row_count * scalar_count].reshape(row_count, scalar_count)
        scalar_offsets = np.cumsum([0] + counts)[:-1]
        points = rows[:, [scalar_offsets[i] for i in xyz_indices]]
        return points[np.all(np.isfinite(points), axis=1)]
    if data_mode != "binary" or not point_count:
        # binary_compressed needs an LZF decoder; current scalenav logs are ASCII.
        return np.empty((0, 3), dtype=np.float64)
    endian = ">" if values.get("DATA", [""])[0].lower() == "binary_be" else "<"
    dtype_fields = []
    for name, size, type_code, count in zip(fields, sizes, types, counts):
        if type_code == "F" and size == 4:
            code = "f4"
        elif type_code == "F" and size == 8:
            code = "f8"
        elif type_code == "I" and size in (1, 2, 4, 8):
            code = f"i{size}"
        elif type_code == "U" and size in (1, 2, 4, 8):
            code = f"u{size}"
        else:
            return np.empty((0, 3), dtype=np.float64)
        dtype_fields.append((name, endian + code, (count,)))
    try:
        structured = np.frombuffer(payload, dtype=np.dtype(dtype_fields), count=point_count)
        points = np.column_stack([structured[name][:, 0] for name in ("x", "y", "z")]).astype(np.float64)
    except (ValueError, TypeError):
        return np.empty((0, 3), dtype=np.float64)
    return points[np.all(np.isfinite(points), axis=1)]


def _downsample_points(points: np.ndarray, maximum: int = 1800, voxel_m: float = 0.12) -> np.ndarray:
    """Keep a deterministic, spatially representative subset for the HTML viewer."""
    points = np.asarray(points, dtype=np.float64)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) <= maximum:
        return points
    keys = np.floor(points / voxel_m).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    selected = points[np.sort(first)]
    if len(selected) > maximum:
        indices = np.linspace(0, len(selected) - 1, maximum, dtype=np.int64)
        selected = selected[indices]
    return selected


def _decode_depth(path: Path) -> np.ndarray:
    if cv2 is None:
        encoded = np.asarray(Image.open(path))
    else:
        encoded = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if encoded is None or encoded.ndim != 2:
            raise ValueError(f"cannot decode depth frame {path}")
    if encoded.dtype == np.uint16:
        return encoded.astype(np.float32) * 0.001
    return encoded.astype(np.float32)


def _model_depth(raw_depth: np.ndarray) -> np.ndarray:
    if cv2 is None:
        resized = np.asarray(
            Image.fromarray(np.asarray(raw_depth, dtype=np.float32)).resize(
                (int(cfg["image_width"]), int(cfg["image_height"])),
                resample=Image.Resampling.NEAREST,
            ),
            dtype=np.float32,
        )
    else:
        resized = cv2.resize(
            raw_depth,
            (int(cfg["image_width"]), int(cfg["image_height"])),
            interpolation=cv2.INTER_NEAREST,
        ).astype(np.float32)
    normalized = np.minimum(resized, 20.0) / 20.0
    invalid = ~np.isfinite(normalized) | (normalized < 0.04 / 20.0)
    encoded = np.uint8(
        np.nan_to_num(normalized, nan=0.0, posinf=1.0, neginf=0.0) * 255.0
    )
    if cv2 is not None:
        return cv2.inpaint(encoded, np.uint8(invalid), 1, cv2.INPAINT_NS).astype(np.float32) / 255.0
    if np.any(invalid):
        valid = encoded[~invalid]
        encoded[invalid] = int(np.median(valid)) if valid.size else 255
    return encoded.astype(np.float32) / 255.0


def _motion_history(odom: Timeline) -> dict[int, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    result: dict[int, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
    previous_velocity: np.ndarray | None = None
    previous_stamp: int | None = None
    acceleration = np.zeros(3, dtype=np.float32)
    for record in odom.items:
        data = record["data"]
        stamp = int(record["stamp_ns"])
        position = np.asarray(data["position"], dtype=np.float64)
        rotation = quaternion_xyzw_to_matrix(data["orientation"])
        velocity = (rotation @ np.asarray(data["velocity"], dtype=np.float64)).astype(np.float32)
        if previous_velocity is not None and previous_stamp is not None:
            elapsed = (stamp - previous_stamp) * 1.0e-9
            if 0.002 <= elapsed <= 0.2:
                measured = np.clip((velocity - previous_velocity) / elapsed, -6.0, 6.0)
                acceleration = (0.85 * acceleration + 0.15 * measured).astype(np.float32)
        result[stamp] = (position, rotation, velocity, acceleration.copy())
        previous_velocity = velocity
        previous_stamp = stamp
    return result


def _state_at(
    odom: Timeline,
    motion: dict[int, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    stamp_ns: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    record = odom.before(stamp_ns)
    if record is None:
        raise ValueError("no odometry before replay frame")
    return motion[int(record["stamp_ns"])]


def _frontier_at(local_goals: Timeline, goals: Timeline, stamp_ns: int) -> np.ndarray:
    record = local_goals.before(stamp_ns) or goals.before(stamp_ns)
    if record is None:
        raise ValueError("no local or mission goal before replay frame")
    return np.asarray(record["data"]["position"], dtype=np.float64)


def _round_points(points: np.ndarray, digits: int = 3) -> list[list[float]]:
    return np.round(np.asarray(points, dtype=np.float64), digits).tolist()


def _dense_mpc_path(states: np.ndarray, controls: np.ndarray, dt: float) -> np.ndarray:
    times = np.linspace(0.0, dt, 5, dtype=np.float64)[1:]
    parts = [states[0, :3][None]]
    for stage, jerk in enumerate(controls):
        position, velocity, acceleration = np.split(states[stage], 3)
        parts.append(
            position[None]
            + times[:, None] * velocity[None]
            + 0.5 * times[:, None] ** 2 * acceleration[None]
            + times[:, None] ** 3 * jerk[None] / 6.0
        )
    return np.concatenate(parts)


class Replayer:
    def __init__(
        self, model_path: Path, device: str, maximum_speed_mps: float,
        method: str = "mpc",
    ) -> None:
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.model = torch.jit.load(str(model_path), map_location=self.device).eval()
        if not math.isfinite(maximum_speed_mps) or maximum_speed_mps <= 0.0:
            raise ValueError("maximum_speed_mps must be finite and positive")
        self.maximum_speed_mps = float(maximum_speed_mps)
        if method not in ("mpc", "sampling"):
            raise ValueError("method must be 'mpc' or 'sampling'")
        self.method = method
        self.scaling = YopoInferenceScaling(
            training_speed_mps=6.0,
            training_acceleration_mps2=6.0,
            inference_speed_mps=self.maximum_speed_mps,
            base_segment_time_s=float(cfg["sgm_time"]),
        )
        self.duration = self.scaling.segment_time_s
        self.config = OrderedBubbleMPCConfig(
            horizon_steps=max(
                12,
                int(round(12 * self.duration / float(cfg["sgm_time"]))),
            ),
            horizon_time_s=self.duration,
            max_velocity_mps=self.maximum_speed_mps,
            max_acceleration_mps2=6.0,
            max_jerk_mps3=40.0,
        )
        self.mpc = None if self.method == "sampling" else OrderedBubbleMPC(
            self.config,
            batch_size=1,
            model_name="yopo_bubble_mpc_log_replay",
        )

    @torch.inference_mode()
    def yopo(
        self,
        depth_path: Path,
        position: np.ndarray,
        rotation: np.ndarray,
        velocity: np.ndarray,
        acceleration: np.ndarray,
        frontier: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        depth = torch.from_numpy(_model_depth(_decode_depth(depth_path))[None, None]).to(
            self.device
        )
        motion_body = np.concatenate((rotation.T @ velocity, rotation.T @ acceleration))
        frontier_body = clip_goal_to_camera_fov(
            world_to_body_flu(frontier, position, rotation),
            horizontal_fov_deg=90.0,
            vertical_fov_deg=73.7398,
        )
        observation = torch.from_numpy(
            np.concatenate((motion_body, frontier_body)).astype(np.float32)[None]
        ).to(self.device)
        output, score = self.model(depth, self.scaling.model_input(observation))
        output = self.scaling.physical_endstate(output)
        return (
            output[0].permute(1, 2, 0).reshape(-1, 9).cpu().numpy(),
            score[0].reshape(-1).cpu().numpy(),
        )

    def replay(
        self,
        raw_endstates: np.ndarray,
        scores: np.ndarray,
        position: np.ndarray,
        rotation: np.ndarray,
        velocity: np.ndarray,
        acceleration: np.ndarray,
        frontier: np.ndarray,
        route: np.ndarray,
        safe_radii: np.ndarray,
    ) -> dict[str, Any]:
        raw_paths, _, _ = sample_poly5_candidate_states(
            position,
            velocity,
            acceleration,
            raw_endstates,
            rotation,
            segment_time_s=self.duration,
            sample_count=31,
        )
        selected = int(np.argmin(scores))
        conditioned = enforce_route_progress(
            raw_endstates[selected : selected + 1],
            frontier_body=world_to_body_flu(frontier, position, rotation),
            minimum_forward_m=2.0, maximum_forward_m=8.0,
        )
        altitude = float(np.median(route[:, 2]))
        conditioned = project_endstates_to_altitude(
            conditioned, position, rotation, altitude
        )
        terminal_world = np.concatenate(
            (
                position[None] + conditioned[:, :3] @ rotation.T,
                conditioned[:, 3:6] @ rotation.T,
                conditioned[:, 6:9] @ rotation.T,
            ),
            axis=1,
        )
        original_route = np.asarray(route, dtype=np.float64)
        original_radii = np.asarray(safe_radii, dtype=np.float64)
        route = trim_route_for_motion(original_route, position, velocity)
        radius_indices = np.argmin(
            np.linalg.norm(route[:, None, :] - original_route[None, :, :], axis=2),
            axis=1,
        )
        safe_radii = original_radii[radius_indices]
        route_segments = np.linalg.norm(np.diff(route, axis=0), axis=1)
        first_segment_index = int(np.flatnonzero(route_segments > 1.0e-6)[0])
        route_tangent = (
            route[first_segment_index + 1] - route[first_segment_index]
        ) / route_segments[first_segment_index]
        initial_forward_speed = max(0.0, float(np.dot(velocity, route_tangent)))
        reachable_progress = maximum_reachable_progress(
            horizon_time_s=self.config.horizon_time_s,
            initial_speed_mps=initial_forward_speed,
            max_velocity_mps=self.config.max_velocity_mps,
            max_acceleration_mps2=self.config.max_acceleration_mps2,
        )
        target_progress, target_adjusted = resolve_target_progress(
            terminal_world[0, :3], route, reachable_progress_m=reachable_progress
        )
        centers, radii, stage_progress = sample_reachable_stage_bubbles(
            route,
            np.asarray(safe_radii, dtype=np.float64),
            horizon_steps=self.config.horizon_steps,
            horizon_time_s=self.config.horizon_time_s,
            initial_speed_mps=initial_forward_speed,
            max_velocity_mps=self.config.max_velocity_mps,
            max_acceleration_mps2=self.config.max_acceleration_mps2,
            target_progress_m=target_progress,
        )
        if self.method == "sampling":
            # Certify each YOPO polynomial by sampled distance to the ordered
            # bubble union.  This is a filter, not a trajectory optimizer.
            distances = np.linalg.norm(
                raw_paths[:, :, None, :] - centers[None, None, :, :], axis=3
            ) - radii[None, None, :]
            sampled_violations = np.maximum(
                np.max(np.min(distances, axis=2), axis=1), 0.0
            )
            valid = np.flatnonzero(sampled_violations <= 1.0e-6)
            selected = int(valid[np.argmin(scores[valid])]) if len(valid) else int(np.argmin(scores))
            paths: list[list[list[float]]] = [[] for _ in range(len(scores))]
            paths[selected] = _round_points(raw_paths[selected])
            full_status = [-2] * len(scores)
            full_status[selected] = 0 if len(valid) else -3
            return {
                "route": _round_points(route),
                "rawPaths": [_round_points(path) for path in raw_paths],
                "rawEndpoints": _round_points(raw_paths[:, -1]),
                "conditionedEndpoints": _round_points(terminal_world[:, :3]),
                "scores": np.round(scores, 5).tolist(),
                "bubbles": [
                    {"center": _round_points(center[None])[0], "radius": round(float(radius), 3)}
                    for center, radius in zip(centers, radii)
                ],
                "bubbleStageProgressM": np.round(stage_progress, 3).tolist(),
                "top1RouteProgressM": round(float(target_progress), 3),
                "reachableProgressM": round(float(stage_progress[-1]), 3),
                "top1BeyondReachableM": round(max(0.0, float(target_progress - stage_progress[-1])), 3),
                "top1ProgressAdjusted": bool(target_adjusted),
                "mpcPaths": paths,
                "mpcStatus": full_status,
                "mpcValue": [None] * len(scores),
                "bubbleViolationM": [
                    round(float(value), 4) if np.isfinite(value) else None
                    for value in sampled_violations
                ],
                "offlineSelected": selected,
                "selectionMethod": "sampled_bubble_filter",
            }
        initial = np.concatenate((position, velocity, acceleration))
        ctx, _, states, controls, values = self.mpc(
            initial[None],
            terminal_world,
            centers[None],
            radii[None],
        )
        states_np = states.detach().cpu().numpy()
        controls_np = controls.detach().cpu().numpy()
        status = np.asarray(ctx.status, dtype=np.int64).reshape(-1)
        paths: list[list[list[float]]] = [[] for _ in range(len(scores))]
        violations: list[float | None] = [None] * len(scores)
        full_status = [-2] * len(scores)
        full_values: list[float | None] = [None] * len(scores)
        full_status[selected] = int(status[0])
        if status[0] == 0:
            selected_path = _dense_mpc_path(states_np[0], controls_np[0], self.config.dt)
            paths[selected] = _round_points(selected_path)
            violations[selected] = maximum_bubble_violation(
                states_np[0, :, :3], centers, radii
            )
            if violations[selected] > self.config.maximum_accepted_bubble_violation_m:
                full_status[selected] = -3
        values_np = values.detach().cpu().numpy().reshape(-1)
        full_values[selected] = round(float(values_np[0]), 4)
        return {
            "route": _round_points(route),
            "rawPaths": [_round_points(path) for path in raw_paths],
            "rawEndpoints": _round_points(raw_paths[:, -1]),
            "conditionedEndpoints": _round_points(terminal_world[:, :3]),
            "scores": np.round(scores, 5).tolist(),
            "bubbles": [
                {"center": _round_points(center[None])[0], "radius": round(float(radius), 3)}
                for center, radius in zip(centers, radii)
            ],
            "bubbleStageProgressM": np.round(stage_progress, 3).tolist(),
            "top1RouteProgressM": round(float(target_progress), 3),
            "reachableProgressM": round(float(stage_progress[-1]), 3),
            "top1BeyondReachableM": round(
                max(0.0, float(target_progress - stage_progress[-1])), 3
            ),
            "top1ProgressAdjusted": bool(target_adjusted),
            "mpcPaths": paths,
            "mpcStatus": full_status,
            "mpcValue": full_values,
            "bubbleViolationM": [None if value is None else round(float(value), 4) for value in violations],
            "offlineSelected": selected,
        }


def _route_for_frame(
    session: Path,
    paths: Timeline,
    clearances: Timeline,
    bubbles: Timeline,
    stamp_ns: int,
    position: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, str] | None:
    path_record = paths.before(stamp_ns)
    clearance_record = clearances.before(stamp_ns)
    route = _load_points(session, path_record)
    if len(route) < 2 or clearance_record is None:
        return None
    try:
        anchored, _ = reanchor_route_path(route, position, maximum_distance_m=10.0)
        if anchored is not None:
            route = anchored
    except ValueError:
        pass
    if np.linalg.norm(route[0] - position) > 1.0e-3:
        route = np.concatenate((position[None], route), axis=0)
    clearance = float(clearance_record["data"]["global_witness_min_m"])
    # Use the original ScaleNav clearance as the route bubble radius. Vehicle
    # dimensions are handled by the separate depth/collision certification,
    # not by shrinking the geometric bubble constraint.
    safe_radius = clearance
    # The log exposes a global minimum clearance, not one radius per route
    # vertex. Keep the same bounded scalar contract used by route features.
    safe_radius = float(
        np.clip(
            safe_radius,
            0.05,
            float(getattr(cfg, "_data", {}).get("route_clearance_clip_m", 3.0)),
        )
    )
    route_radii = np.full(len(route), safe_radius, dtype=np.float64)
    bubble_record = bubbles.before(stamp_ns)
    if (
        bubble_record is not None
        and bubble_record.get("file")
        and path_record is not None
        and abs(int(bubble_record["stamp_ns"]) - int(path_record["stamp_ns"])) <= 200_000_000
    ):
        payload = json.loads((session / bubble_record["file"]).read_text(encoding="utf-8"))
        centers: list[list[float]] = []
        raw_radii: list[float] = []
        for marker in payload.get("markers", []):
            if marker.get("ns") != "scalenav_route_bubble_radius" or marker.get("action") != 0:
                continue
            center = marker.get("pose", {}).get("position")
            scale = marker.get("scale", [])
            if center is None or len(scale) < 1:
                continue
            centers.append(center)
            raw_radii.append(0.5 * float(scale[0]))
        if centers:
            center_array = np.asarray(centers, dtype=np.float64)
            raw_radius_array = np.asarray(raw_radii, dtype=np.float64)
            distances = np.linalg.norm(route[:, None, :] - center_array[None, :, :], axis=2)
            nearest = np.argmin(distances, axis=1)
            matched = distances[np.arange(len(route)), nearest] <= 1.0
            route_radii[matched] = raw_radius_array[nearest[matched]]
            route_radii = np.clip(
                route_radii,
                0.05,
                float(getattr(cfg, "_data", {}).get("route_clearance_clip_m", 3.0)),
            )
            return route, route_radii, "route_topology_bubble_raw_radius_with_clearance_fallback"
    return route, route_radii, "path_min_clearance_fallback"


def build(
    session: Path,
    output: Path,
    model_path: Path,
    device: str,
    maximum_speed_mps: float = 6.0,
    method: str = "mpc",
) -> Path:
    records = _records(session)
    timelines = {
        kind: Timeline(records, kind)
        for kind in (
            "route_yopo_status", "depth", "odom", "path", "clearance", "bubbles",
            "local_goal", "goal", "route_yopo_planned_path", "collision", "pointcloud",
        )
    }
    # A native ScaleNav run uses ``scalenav_online_planner`` (original
    # YOPO-Simple) and therefore has no Route-YOPO status topic. In that case
    # replay one frame per logged depth image and apply the bubble-MPC
    # post-process offline. Route-YOPO logs still use their status timestamps
    # so recorded candidate scores can be matched exactly.
    status_items = timelines["route_yopo_status"].items
    original_yopo_log = not status_items
    if original_yopo_log and not timelines["depth"].items:
        raise ValueError("session has neither route_yopo_status nor depth records")
    motion = _motion_history(timelines["odom"])
    replayer = Replayer(model_path, device, maximum_speed_mps, method=method)
    # A score vector is not a frame identity: the same network scores can
    # occur after the vehicle has moved or the depth image has changed. Keep
    # replay results scoped to the status timestamp to avoid cross-frame
    # position/depth reuse in the viewer.
    cache: dict[tuple[int, tuple[float, ...]], dict[str, Any]] = {}
    pointcloud_cache: dict[str, np.ndarray] = {}
    frames: list[dict[str, Any]] = []
    frame_items = status_items if status_items else [
        item for item in timelines["depth"].items
        if timelines["local_goal"].before(int(item["stamp_ns"])) is not None
        or timelines["goal"].before(int(item["stamp_ns"])) is not None
    ]
    first_stamp = min(
        int((timelines["goal"].items or frame_items)[0]["stamp_ns"]),
        int(frame_items[0]["stamp_ns"]),
    )
    collision_records = timelines["collision"].items
    collision_stamp = next(
        (int(item["stamp_ns"]) for item in collision_records if item.get("data", {}).get("active")),
        None,
    )

    for frame_index, status_record in enumerate(frame_items):
        stamp = int(status_record["stamp_ns"])
        status_data = status_record.get("data", {}) if not original_yopo_log else {}
        position, rotation, velocity, acceleration = _state_at(
            timelines["odom"], motion, stamp
        )
        # Point clouds are logged in the sensor/body frame. Match the closest
        # cloud to this frame and use the odometry at the cloud timestamp for
        # the body-to-world transform, so moving the slider never shows a
        # cloud transformed with a stale pose.
        obstacle_record = timelines["pointcloud"].nearest(stamp)
        obstacles = np.empty((0, 3), dtype=np.float64)
        obstacle_file = None
        obstacle_delta_ms = None
        if obstacle_record is not None:
            obstacle_delta_ms = abs(int(obstacle_record["stamp_ns"]) - stamp) * 1.0e-6
            if obstacle_delta_ms <= 300.0 and obstacle_record.get("file"):
                obstacle_file = obstacle_record["file"]
                if obstacle_file not in pointcloud_cache:
                    pointcloud_cache[obstacle_file] = _load_pcd_xyz(session / obstacle_file)
                local_obstacles = pointcloud_cache[obstacle_file]
                cloud_position, cloud_rotation, _, _ = _state_at(
                    timelines["odom"], motion, int(obstacle_record["stamp_ns"])
                )
                if len(local_obstacles):
                    obstacles = local_obstacles @ cloud_rotation.T + cloud_position
                    obstacles = _downsample_points(obstacles)
        route_info = _route_for_frame(
            session, timelines["path"], timelines["clearance"], timelines["bubbles"],
            stamp, position
        )
        frontier = _frontier_at(timelines["local_goal"], timelines["goal"], stamp)
        replay: dict[str, Any] | None = None
        depth_file = None
        score_match_mae = None
        target_scores = status_data.get("candidate_scores")
        cache_key = (
            stamp,
            tuple(round(float(value), 5) for value in target_scores),
        ) if target_scores else (stamp, ())
        if route_info is not None and timelines["depth"].before(stamp) is not None:
            if cache_key and cache_key in cache:
                replay = cache[cache_key]
                depth_file = replay.get("depthFile")
                score_match_mae = replay.get("scoreMatchMae")
            else:
                candidates = timelines["depth"].recent(stamp, 8 if target_scores else 1)
                best: tuple[float, dict[str, Any], np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
                for depth_record in candidates:
                    depth_stamp = int(depth_record["stamp_ns"])
                    d_position, d_rotation, d_velocity, d_acceleration = _state_at(
                        timelines["odom"], motion, depth_stamp
                    )
                    d_frontier = _frontier_at(
                        timelines["local_goal"], timelines["goal"], depth_stamp
                    )
                    endstates, scores = replayer.yopo(
                        session / depth_record["file"], d_position, d_rotation,
                        d_velocity, d_acceleration, d_frontier,
                    )
                    error = (
                        float(np.mean(np.abs(scores - np.asarray(target_scores))))
                        if target_scores else 0.0
                    )
                    candidate = (
                        error, depth_record, endstates, scores, d_position,
                        d_rotation, d_velocity, d_acceleration, d_frontier,
                    )
                    if best is None or error < best[0]:
                        best = candidate
                assert best is not None
                error, depth_record, endstates, scores, d_position, d_rotation, d_velocity, d_acceleration, d_frontier = best
                depth_stamp = int(depth_record["stamp_ns"])
                matched_route = _route_for_frame(
                    session, timelines["path"], timelines["clearance"], timelines["bubbles"],
                    depth_stamp, d_position
                ) or route_info
                replay = replayer.replay(
                    endstates, scores, d_position, d_rotation, d_velocity,
                    d_acceleration, d_frontier, matched_route[0], matched_route[1],
                )
                replay["radiusSource"] = matched_route[2]
                replay["depthFile"] = depth_record["file"]
                replay["scoreMatchMae"] = round(error, 6) if target_scores else None
                replay["replayPosition"] = _round_points(d_position[None])[0]
                # Keep the exact route snapshot used to build the bubbles in
                # the frame payload.  The status timestamp and matched depth
                # timestamp can straddle a graph update; displaying a newer
                # route beside older bubbles makes a valid corridor look
                # geometrically wrong.
                # ``replay`` carries the exact cleaned route used for bubble
                # sampling; do not replace it with the pre-cleanup snapshot.
                depth_file = depth_record["file"]
                score_match_mae = replay["scoreMatchMae"]
                if cache_key:
                    cache[cache_key] = replay
        elif original_yopo_log and timelines["depth"].before(stamp) is not None:
            # Native YOPO logs may publish a depth frame before the first
            # graph path. Keep the frame visible even when route post-process
            # is not yet available.
            replay = None
        planned_record = timelines["route_yopo_planned_path"].before(stamp)
        planned_path = _load_points(session, planned_record)
        route = (
            np.asarray(replay.get("route"), dtype=np.float64)
            if replay is not None and replay.get("route")
            else route_info[0]
            if route_info is not None
            else np.empty((0, 3))
        )
        frames.append(
            {
                "index": frame_index,
                "t": round((stamp - first_stamp) * 1.0e-9, 3),
                "stampNs": stamp,
                "position": _round_points(position[None])[0],
                "obstacles": _round_points(obstacles),
                "obstaclePointcloudFile": obstacle_file,
                "obstaclePointcloudDeltaMs": None if obstacle_delta_ms is None else round(obstacle_delta_ms, 3),
                "speedMps": round(float(np.linalg.norm(velocity)), 3),
                "route": _round_points(route),
                "loggedPlan": _round_points(planned_path),
                "loggedSelected": status_data.get("selected_primitive"),
                "loggedMode": status_data.get("mode", "ORIGINAL_YOPO" if original_yopo_log else "UNKNOWN"),
                "loggedReason": status_data.get("reason", ""),
                "trajectoryReplaced": bool(status_data.get("trajectory_replaced", False)),
                "recoveryUsed": bool(status_data.get("mpc_recovery_used", False)),
                "trajectorySource": status_data.get("trajectory_source"),
                "depthFile": depth_file,
                "scoreMatchMae": score_match_mae,
                "collision": collision_stamp is not None and stamp >= collision_stamp,
                "replay": replay,
            }
        )
        print(f"replayed {frame_index + 1}/{len(frame_items)}", flush=True)

    odom_trace = []
    for odom_index, record in enumerate(timelines["odom"].items):
        stamp = int(record["stamp_ns"])
        if stamp < first_stamp or (collision_stamp is not None and stamp > collision_stamp):
            continue
        if odom_index % 5 == 0:
            odom_trace.append(
                [round((stamp - first_stamp) * 1.0e-9, 3)]
                + [round(float(value), 3) for value in record["data"]["position"]]
            )
    metadata = {
        "session": str(session),
        "model": str(model_path),
        "device": str(replayer.device),
        "maximumSpeedMps": replayer.maximum_speed_mps,
        "frameCount": len(frames),
        "yopoCandidateCount": 15,
        "mpcCandidateCountPerFrame": 1,
        "horizonSteps": replayer.config.horizon_steps,
        "horizonTimeS": replayer.config.horizon_time_s,
        "collisionTimeS": None if collision_stamp is None else round((collision_stamp - first_stamp) * 1.0e-9, 3),
        "dataContract": (
            "recorded odometry/path/depth + original YOPO/leap-c replay"
            if original_yopo_log
            else "recorded odometry/path/status + offline YOPO/leap-c replay"
        ),
        "method": method,
        "obstaclePointcloudFrameCount": sum(bool(frame.get("obstacles")) for frame in frames),
        "obstacleDisplayPoints": sum(len(frame.get("obstacles", [])) for frame in frames),
    }
    replayed = [frame["replay"] for frame in frames if frame.get("replay")]
    mpc_statuses = [
        int(status)
        for replay in replayed
        for status in replay.get("mpcStatus", [])
        if int(status) != -2
    ]
    radius_sources: dict[str, int] = {}
    for replay in replayed:
        source = str(replay.get("radiusSource", "unknown"))
        radius_sources[source] = radius_sources.get(source, 0) + 1
    metadata.update(
        {
            "replayFrameCount": len(replayed),
            "mpcSuccessCount": sum(status == 0 for status in mpc_statuses),
            "mpcRejectedCount": sum(status == -3 for status in mpc_statuses),
            "mpcFailureCount": sum(status not in (0, -3) for status in mpc_statuses),
            "radiusSourceCounts": radius_sources,
            "collisionDetected": collision_stamp is not None,
        }
    )
    html = TEMPLATE.replace("__META__", json.dumps(metadata, separators=(",", ":")))
    html = html.replace("__FRAMES__", json.dumps(frames, separators=(",", ":")))
    html = html.replace("__ODOM__", json.dumps(odom_trace, separators=(",", ":")))
    html = html.replace(
        "__METHOD_LABEL__",
        "采样 bubble 筛选" if method == "sampling" else "ordered bubble MPC",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    report = output.with_suffix(".json")
    report.write_text(json.dumps({"metadata": metadata, "frames": frames}, indent=2) + "\n", encoding="utf-8")
    summary = output.with_name(output.stem + "_summary.txt")
    source_text = ", ".join(
        f"{name}: {count}" for name, count in sorted(radius_sources.items())
    ) or "无"
    summary.write_text(
        "离线测试结果\n"
        f"session: {session}\n"
        f"model: {model_path}\n"
        f"总帧数: {len(frames)}\n"
        f"YOPO + {'采样筛选' if method == 'sampling' else 'MPC'} 重放帧数: {len(replayed)}\n"
        f"YOPO 候选数: 15（每帧选 score 最小的 top-1）\n"
        f"{'采样通过' if method == 'sampling' else 'MPC 成功'}: {metadata['mpcSuccessCount']}\n"
        f"bubble 拒绝: {metadata['mpcRejectedCount']}\n"
        f"{'求解失败' if method == 'sampling' else 'MPC 求解失败'}: {metadata['mpcFailureCount']}\n"
        f"bubble 半径来源: {source_text}\n"
        f"真实障碍物点云: {metadata['obstaclePointcloudFrameCount']} 帧，显示点数 {metadata['obstacleDisplayPoints']}（每帧最多 1800）\n"
        f"碰撞: {'是' if collision_stamp is not None else '无'}\n"
        f"最大速度配置: {replayer.maximum_speed_mps:.1f} m/s\n",
        encoding="utf-8",
    )
    print(output)
    print(summary)
    return output


TEMPLATE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>YOPO -> __METHOD_LABEL__ 完整 Log 离线重放</title>
<style>
:root{color-scheme:dark;--bg:#0d1117;--panel:#151b23;--line:#303944;--text:#e6edf3;--muted:#9ba7b4;--raw:#ff9f43;--route:#45c8c2;--mpc:#67a9ff;--selected:#ffdf5d;--bad:#ff6b6b;--ok:#61d095;--obstacle:#f06b6b}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px system-ui,sans-serif}header{display:flex;align-items:center;justify-content:space-between;padding:12px 18px;border-bottom:1px solid var(--line);background:#11161d}h1{font-size:18px;margin:0;letter-spacing:0}button{width:34px;height:32px;border:1px solid var(--line);background:#202833;color:var(--text);border-radius:5px;cursor:pointer}button:hover{background:#2a3542}.toolbar{display:flex;gap:6px}.timeline{display:grid;grid-template-columns:110px 1fr 90px;gap:12px;align-items:center;padding:10px 18px;border-bottom:1px solid var(--line)}.summary{margin:10px;padding:10px 14px;background:#1b2631;border:1px solid #3d4d5d;border-left:4px solid var(--ok);border-radius:5px;font-weight:600}.summary .muted{color:var(--muted);font-weight:400}input[type=range]{width:100%;accent-color:#67a9ff}.content{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;padding:10px}.stage{min-width:0;background:var(--panel);border:1px solid var(--line);border-radius:6px;overflow:hidden}.stage h2{height:42px;margin:0;padding:10px 12px;font-size:14px;border-bottom:1px solid var(--line)}canvas{display:block;width:100%;height:440px}.legend{min-height:50px;padding:8px 12px;color:var(--muted);font-size:12px;border-top:1px solid var(--line)}.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin:0 4px 0 10px}.dot:first-child{margin-left:0}.bottom{display:grid;grid-template-columns:2fr 1fr;gap:10px;padding:0 10px 10px}.overview,.details{background:var(--panel);border:1px solid var(--line);border-radius:6px}.overview h2,.details h2{font-size:14px;margin:0;padding:10px 12px;border-bottom:1px solid var(--line)}#overview{height:220px}.facts{display:grid;grid-template-columns:repeat(3,1fr);gap:1px;background:var(--line)}.fact{background:var(--panel);padding:9px 11px;min-width:0}.key{display:block;color:var(--muted);font-size:11px}.value{display:block;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;margin-top:2px;font-weight:600}.flow{padding:9px 12px;color:var(--muted);border-top:1px solid var(--line)}.arrow{color:#657180;padding:0 6px}.bad{color:var(--bad)}.ok{color:var(--ok)}@media(max-width:1000px){.content{grid-template-columns:1fr}.bottom{grid-template-columns:1fr}canvas{height:380px}}
</style></head><body>
<header><h1>YOPO -> __METHOD_LABEL__：完整 Log 离线重放</h1><div class="toolbar"><button id="prev" title="上一帧">&#9664;</button><button id="play" title="播放">&#9654;</button><button id="next" title="下一帧">&#9654;&#124;</button></div></header>
<div class="timeline"><span id="clock"></span><input id="slider" type="range" min="0" value="0" step="1"><span id="frameNo"></span></div>
<div class="summary" id="summary"></div>
<main class="content">
<section class="stage"><h2>1. YOPO 输出并选 top-1</h2><canvas id="raw"></canvas><div class="legend"><span class="dot" style="background:var(--obstacle)"></span>真实障碍物点云 <span class="dot" style="background:var(--raw)"></span>15 条原始 poly5 <span class="dot" style="background:var(--selected)"></span>YOPO score 最小的 top-1</div></section>
<section class="stage"><h2>2. top-1 + bubble 约束</h2><canvas id="constraint"></canvas><div class="legend"><span class="dot" style="background:var(--obstacle)"></span>真实障碍物点云 <span class="dot" style="background:var(--route)"></span>引导路径与逐时刻 bubble <span class="dot" style="background:var(--raw)"></span>YOPO 候选终点</div></section>
<section class="stage"><h2>3. 约束后的 top-1 轨迹</h2><canvas id="result"></canvas><div class="legend"><span class="dot" style="background:var(--obstacle)"></span>真实障碍物点云 <span class="dot" style="background:var(--mpc)"></span>通过 bubble 的轨迹 <span class="dot" style="background:var(--raw)"></span>越界/拒绝 <span class="dot" style="background:var(--bad)"></span>处理失败 <span class="dot" style="background:var(--ok)"></span>旧 log 下发轨迹（虚线）</div></section>
</main>
<div class="bottom"><section class="overview"><h2>原始记录的完整执行轨迹（离线重放背景）</h2><canvas id="overview"></canvas></section><section class="details"><h2>当前帧</h2><div class="facts" id="facts"></div><div class="flow">深度 + 状态 -> YOPO 产生 15 个候选（不输入路径）<span class="arrow">-></span>逐点采样 bubble 并筛选候选轨迹<span class="arrow">-></span>安全认证与执行</div></section></div>
<script>
const META=__META__,FRAMES=__FRAMES__,ODOM=__ODOM__;let index=Math.max(0,FRAMES.findIndex(f=>f.replay)),timer=null;slider.max=FRAMES.length-1;
function resize(c){const d=devicePixelRatio||1,r=c.getBoundingClientRect();if(c.width!==Math.round(r.width*d)||c.height!==Math.round(r.height*d)){c.width=Math.round(r.width*d);c.height=Math.round(r.height*d)}return[c.getContext('2d'),c.width,c.height]}
function bounds(groups){const pts=[];function visit(value){if(Array.isArray(value)&&value.length>=2&&Number.isFinite(value[0])&&Number.isFinite(value[1])){pts.push(value);return}if(Array.isArray(value))value.forEach(visit)}visit(groups);if(!pts.length)return[-5,5,-5,5];let xs=pts.map(p=>p[0]),ys=pts.map(p=>p[1]),a=Math.min(...xs),b=Math.max(...xs),c=Math.min(...ys),d=Math.max(...ys),pad=Math.max(2,.12*Math.max(b-a,d-c));return[a-pad,b+pad,c-pad,d+pad]}
function projector(w,h,b){const pad=28,s=Math.min((w-2*pad)/Math.max(1,b[1]-b[0]),(h-2*pad)/Math.max(1,b[3]-b[2]));return[p=>[pad+(p[0]-b[0])*s,h-pad-(p[1]-b[2])*s],s]}
function grid(ctx,w,h,P,b){ctx.fillStyle='#10151c';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#25303b';ctx.lineWidth=1;for(let x=Math.ceil(b[0]/5)*5;x<=b[1];x+=5){let a=P([x,b[2]]),z=P([x,b[3]]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...z);ctx.stroke()}for(let y=Math.ceil(b[2]/5)*5;y<=b[3];y+=5){let a=P([b[0],y]),z=P([b[1],y]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...z);ctx.stroke()}}
function line(ctx,P,pts,color,width=2,alpha=1,dash=[]){if(!pts||pts.length<2)return;ctx.save();ctx.strokeStyle=color;ctx.globalAlpha=alpha;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();pts.forEach((p,i)=>i?ctx.lineTo(...P(p)):ctx.moveTo(...P(p)));ctx.stroke();ctx.restore()}
function point(ctx,P,p,color,r=4){if(!p)return;ctx.fillStyle=color;ctx.beginPath();ctx.arc(...P(p),r,0,Math.PI*2);ctx.fill()}
function label(ctx,P,p,text,color){ctx.fillStyle=color;ctx.font=`${11*(devicePixelRatio||1)}px system-ui`;let q=P(p);ctx.fillText(text,q[0]+5,q[1]-5)}
function stage(canvas,kind,f){let [ctx,w,h]=resize(canvas),r=f.replay||{},raw=r.rawPaths||[],mpc=r.mpcPaths||[],route=r.route||f.route||[],bubbles=r.bubbles||[],bubblePts=bubbles.map(x=>x.center),obstacles=f.obstacles||[],replayPosition=r.replayPosition||f.position,selected=r.offlineSelected??f.loggedSelected,b=bounds([raw,mpc,route,bubblePts,obstacles,[replayPosition]]),[P,s]=projector(w,h,b);grid(ctx,w,h,P,b);obstacles.forEach(p=>point(ctx,P,p,'#f06b6b',2));line(ctx,P,route,'#45c8c2',3,1);if(kind==='raw'){raw.forEach((p,i)=>{let sel=i===selected;line(ctx,P,p,sel?'#ffdf5d':'#ff9f43',sel?4:1.5,sel?1:.35);if(p.length)label(ctx,P,p[p.length-1],String(i),sel?'#ffdf5d':'#d8a56c')})}else if(kind==='constraint'){let labelStride=Math.max(1,Math.ceil(bubbles.length/6));bubbles.forEach((x,i)=>{let q=P(x.center);ctx.beginPath();ctx.arc(q[0],q[1],x.radius*s,0,Math.PI*2);ctx.fillStyle='#45c8c219';ctx.fill();ctx.strokeStyle='#45c8c288';ctx.lineWidth=1;ctx.stroke();if(i%labelStride===0||i===bubbles.length-1)label(ctx,P,x.center,String(i),'#84ddd8')});(r.conditionedEndpoints||[]).forEach(p=>{point(ctx,P,p,'#ff9f43',5);label(ctx,P,p,`top-1: ${selected}`,'#ffb56f')})}else{mpc.forEach((p,i)=>{let ok=(r.mpcStatus||[])[i]===0,violation=(r.bubbleViolationM||[])[i],outside=ok&&violation!==null&&violation>.01,sel=i===selected,color=sel&&!outside?'#ffdf5d':!ok?'#ff6b6b':outside?'#ff9f43':'#67a9ff';line(ctx,P,p,color,sel?4:1.7,1,ok?[]:[5,4]);if(p.length)label(ctx,P,p[p.length-1],String(i),color)});line(ctx,P,f.loggedPlan||[],'#61d095',2,.65,[7,4])}point(ctx,P,replayPosition,'#ffffff',5);label(ctx,P,replayPosition,'UAV','#ffffff')}
function drawOverview(f){let [ctx,w,h]=resize(overview),pts=ODOM.map(x=>x.slice(1)),routes=FRAMES.map(x=>x.route),obstacles=f.obstacles||[],b=bounds([pts,routes,obstacles]),[P]=projector(w,h,b);grid(ctx,w,h,P,b);obstacles.forEach(p=>point(ctx,P,p,'#f06b6b',1.5));line(ctx,P,pts,'#52606d',2,.8);let past=ODOM.filter(x=>x[0]<=f.t).map(x=>x.slice(1));line(ctx,P,past,f.collision?'#ff6b6b':'#61d095',3,1);line(ctx,P,f.route,'#45c8c2',2,.7);point(ctx,P,f.position,f.collision?'#ff6b6b':'#ffdf5d',6);if(META.collisionTimeS!==null){let cp=ODOM.reduce((a,x)=>Math.abs(x[0]-META.collisionTimeS)<Math.abs(a[0]-META.collisionTimeS)?x:a,ODOM[0]);point(ctx,P,cp.slice(1),'#ff6b6b',7);label(ctx,P,cp.slice(1),'COLLISION','#ff6b6b')}}
function updateSummary(){let sources=Object.entries(META.radiusSourceCounts||{}).map(([k,v])=>`${k} ${v}`).join('，');summary.innerHTML=`离线测试结果：${META.frameCount} 总帧 | ${META.replayFrameCount} 重放 | <span class="ok">${META.mpcSuccessCount} MPC 成功</span> | ${META.mpcRejectedCount} bubble 拒绝 | ${META.mpcFailureCount} 求解失败 | 碰撞：${META.collisionDetected?'是':'无'}<span class="muted">　半径：${sources||'无'}</span>`}
function render(){let f=FRAMES[index];slider.value=index;clock.textContent=`t = ${f.t.toFixed(2)} s`;frameNo.textContent=`${index+1} / ${FRAMES.length}`;updateSummary();stage(raw,'raw',f);stage(constraint,'constraint',f);stage(result,'result',f);drawOverview(f);let r=f.replay||{},submitted=(r.mpcStatus||[]).filter(x=>x!==-2).length,ok=(r.mpcStatus||[]).filter(x=>x===0).length,mae=f.scoreMatchMae,violations=(r.bubbleViolationM||[]).filter(x=>x!==null),maxViolation=violations.length?Math.max(...violations):null;facts.innerHTML=[['旧 Log 状态',f.loggedMode],['YOPO top-1',r.offlineSelected??'无'],['速度',`${f.speedMps.toFixed(2)} m/s`],['障碍物点数',String((f.obstacles||[]).length)],['点云记录',f.obstaclePointcloudFile||'无'],['点云时间差',f.obstaclePointcloudDeltaMs===null||f.obstaclePointcloudDeltaMs===undefined?'N/A':`${f.obstaclePointcloudDeltaMs.toFixed(1)} ms`],['MPC 输入数',f.replay?`${submitted}（成功 ${ok}）`:'无重放'],['top-1 路径进度',r.top1RouteProgressM===undefined?'N/A':`${r.top1RouteProgressM.toFixed(2)} m`],['bubble 可达进度',r.reachableProgressM===undefined?'N/A':`${r.reachableProgressM.toFixed(2)} m`],['超出可达范围',r.top1BeyondReachableM===undefined?'N/A':`${r.top1BeyondReachableM.toFixed(2)} m`],['最大 bubble 越界',maxViolation===null?'N/A':`${maxViolation.toFixed(3)} m`],['旧 Log 恢复 MPC',f.recoveryUsed?'是':'否'],['碰撞',f.collision?'是':'否'],['深度帧',f.depthFile||'无'],['score 匹配 MAE',mae===null||mae===undefined?'N/A':mae.toFixed(5)]].map(x=>`<div class="fact"><span class="key">${x[0]}</span><span class="value ${x[0]==='碰撞'&&f.collision?'bad':''}">${x[1]}</span></div>`).join('')}
slider.oninput=()=>{index=+slider.value;render()};prev.onclick=()=>{index=Math.max(0,index-1);render()};next.onclick=()=>{index=Math.min(FRAMES.length-1,index+1);render()};play.onclick=()=>{if(timer){clearInterval(timer);timer=null;play.innerHTML='&#9654;'}else{play.innerHTML='&#10074;&#10074;';timer=setInterval(()=>{index=(index+1)%FRAMES.length;render()},350)}};addEventListener('resize',render);render();
</script></body></html>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "train_scalenav/tmp/yopo_bubble_mpc_full_log.html")
    parser.add_argument("--model", type=Path, default=ROOT / "scalenav_ws/src/models/original_yopo_simple/model.pt")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument(
        "--maximum-speed",
        type=float,
        default=6.0,
        help="YOPO/MPC inference speed limit in m/s (default: 6.0)",
    )
    parser.add_argument(
        "--method",
        choices=("mpc", "sampling"),
        default="mpc",
        help="post-process with leap-c bubble MPC or sampled bubble filtering",
    )
    args = parser.parse_args()
    build(
        args.session.resolve(),
        args.output.resolve(),
        args.model.resolve(),
        args.device,
        args.maximum_speed,
        args.method,
    )


if __name__ == "__main__":
    main()
