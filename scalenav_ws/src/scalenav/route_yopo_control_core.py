"""ROS-independent contracts for the Route-YOPO online controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import hashlib
import math
from typing import Any, Sequence

import numpy as np


class RouteMode(str, Enum):
    ROUTE = "ROUTE"
    FRONTIER_ONLY = "FRONTIER_ONLY"
    SAFETY_HOLD = "SAFETY_HOLD"


@dataclass(frozen=True)
class ModeDecision:
    mode: RouteMode
    reason: str


def decide_route_mode(
    *,
    frontier_fresh: bool,
    route_fresh: bool,
    route_coherent: bool,
    route_valid: bool,
) -> ModeDecision:
    if not frontier_fresh:
        return ModeDecision(RouteMode.SAFETY_HOLD, "frontier_missing_or_stale")
    if not route_fresh:
        return ModeDecision(RouteMode.FRONTIER_ONLY, "route_missing_or_stale")
    if not route_coherent:
        return ModeDecision(RouteMode.FRONTIER_ONLY, "compat_topics_not_coherent")
    if not route_valid:
        return ModeDecision(RouteMode.FRONTIER_ONLY, "route_contract_invalid")
    return ModeDecision(RouteMode.ROUTE, "accepted_route_ready")


def quaternion_xyzw_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    values = np.asarray(quaternion, dtype=np.float64)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("quaternion must contain four finite XYZW values")
    norm = float(np.linalg.norm(values))
    if norm < 1.0e-9:
        raise ValueError("quaternion must not be zero")
    x, y, z, w = values / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def world_to_body_flu(
    points_world: np.ndarray,
    position_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
) -> np.ndarray:
    points = np.asarray(points_world, dtype=np.float64)
    position = np.asarray(position_world, dtype=np.float64)
    rotation = np.asarray(rotation_body_to_world, dtype=np.float64)
    if points.shape[-1:] != (3,) or position.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("invalid world/body transform shapes")
    if not all(np.isfinite(value).all() for value in (points, position, rotation)):
        raise ValueError("world/body transform inputs must be finite")
    return (points - position) @ rotation


def conservative_depth_reduce(
    depth_m: np.ndarray,
    target_height: int,
    target_width: int,
) -> np.ndarray:
    """Reduce depth without hiding a near obstacle or an unknown source ray."""
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("safety depth must be a two-dimensional image")
    if target_height <= 0 or target_width <= 0:
        raise ValueError("safety depth target dimensions must be positive")
    if depth.shape == (target_height, target_width):
        return depth.copy()
    if depth.shape[0] < target_height or depth.shape[1] < target_width:
        return depth.copy()
    block_h = int(math.ceil(depth.shape[0] / target_height))
    block_w = int(math.ceil(depth.shape[1] / target_width))
    padded = np.full(
        (target_height * block_h, target_width * block_w),
        np.nan,
        dtype=np.float32,
    )
    padded[: depth.shape[0], : depth.shape[1]] = depth
    blocks = padded.reshape(target_height, block_h, target_width, block_w)
    finite = np.isfinite(blocks)
    reduced = np.min(np.where(finite, blocks, np.inf), axis=(1, 3))
    reduced[~np.all(finite, axis=(1, 3))] = np.nan
    return reduced.astype(np.float32, copy=False)


def validate_depth_trajectory(
    query: Any,
    trajectory_world: np.ndarray,
    position_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    *,
    minimum_altitude_m: float,
    route_altitude_m: float | None = None,
    route_altitude_tolerance_m: float | None = None,
) -> dict[str, Any]:
    """Conservatively certify a sampled trajectory against one depth frame."""
    trajectory = np.asarray(trajectory_world, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1:] != (3,):
        raise ValueError("trajectory_world must have shape [S, 3]")
    if not np.isfinite(trajectory).all():
        return {"state": "NON_FINITE", "minimum_clearance_m": None, "known_fraction": 0.0}
    if float(np.min(trajectory[:, 2])) < minimum_altitude_m:
        return {"state": "ALTITUDE", "minimum_clearance_m": None, "known_fraction": 0.0}
    if route_altitude_m is not None or route_altitude_tolerance_m is not None:
        if route_altitude_m is None or route_altitude_tolerance_m is None:
            raise ValueError("route altitude and tolerance must be provided together")
        if (
            not math.isfinite(route_altitude_m)
            or not math.isfinite(route_altitude_tolerance_m)
            or route_altitude_tolerance_m <= 0.0
        ):
            raise ValueError("route altitude constraint must be finite and positive")
        maximum_error = float(np.max(np.abs(trajectory[:, 2] - route_altitude_m)))
        if maximum_error > route_altitude_tolerance_m:
            return {
                "state": "ROUTE_ALTITUDE",
                "minimum_clearance_m": None,
                "known_fraction": 0.0,
                "maximum_altitude_error_m": maximum_error,
            }

    points_body = world_to_body_flu(
        trajectory, position_world, rotation_body_to_world
    )
    sample_spacing = np.linalg.norm(np.diff(points_body, axis=0), axis=1)
    if not np.isfinite(sample_spacing).all():
        return {"state": "NON_FINITE", "minimum_clearance_m": None, "known_fraction": 0.0}
    swept_radius = query.swept_radius_m + 0.5 * float(np.max(sample_spacing, initial=0.0))
    minimum_clearance = math.inf
    known_pixels = 0
    requested_pixels = 0
    checked_samples = 0
    start_volume_forward = 2.0 * query.swept_radius_m
    any_projectable = False
    saw_unknown = False
    for point in points_body:
        forward = float(point[0])
        if forward <= start_volume_forward:
            continue
        projection = query.project(point)
        if projection is None:
            saw_unknown = True
            requested_pixels += 1
            continue
        any_projectable = True
        checked_samples += 1
        u, v = projection
        radius_u = max(1, int(math.ceil(query.fx * swept_radius / forward)))
        radius_v = max(1, int(math.ceil(query.fy * swept_radius / forward)))
        requested_pixels += query._ellipse_pixel_count(radius_u, radius_v)
        u0 = max(0, int(math.floor(u)) - radius_u)
        u1 = min(query.width - 1, int(math.ceil(u)) + radius_u)
        v0 = max(0, int(math.floor(v)) - radius_v)
        v1 = min(query.height - 1, int(math.ceil(v)) + radius_v)
        columns = np.arange(u0, u1 + 1, dtype=np.float32)
        rows = np.arange(v0, v1 + 1, dtype=np.float32)
        grid_u, grid_v = np.meshgrid(columns, rows)
        mask = ((grid_u - u) / radius_u) ** 2 + ((grid_v - v) / radius_v) ** 2 <= 1.0
        values = query.depth[v0 : v1 + 1, u0 : u1 + 1][mask]
        measured = np.isfinite(values) & (values >= query.min_depth_m)
        saturated = measured & (values >= query.far_depth_m - 1.0e-3)
        saturated_safe = saturated & (
            forward + swept_radius < query.far_depth_m - 1.0e-3
        )
        known = (measured & ~saturated) | saturated_safe
        known_pixels += int(known.sum())
        if np.any(saturated & ~saturated_safe):
            saw_unknown = True
        if not known.any():
            continue
        clearance_values = values[known].copy()
        clearance_values[saturated_safe[known]] = query.far_depth_m
        minimum_clearance = min(
            minimum_clearance, float(np.min(clearance_values - forward))
        )
        if np.any(values[measured & ~saturated] <= forward + swept_radius):
            return {
                "state": "INVALID",
                "minimum_clearance_m": float(minimum_clearance),
                "known_fraction": min(1.0, known_pixels / max(requested_pixels, 1)),
                "checked_samples": checked_samples,
                "swept_radius_m": swept_radius,
            }
    known_fraction = min(1.0, known_pixels / max(requested_pixels, 1))
    state = (
        "CERTIFIED"
        if any_projectable
        and not saw_unknown
        and known_fraction >= 1.0 - query.max_unknown_fraction
        else "UNVALIDATED"
    )
    return {
        "state": state,
        "minimum_clearance_m": None
        if not math.isfinite(minimum_clearance)
        else float(minimum_clearance),
        "known_fraction": known_fraction,
        "checked_samples": checked_samples,
        "swept_radius_m": swept_radius,
    }


def project_endstates_to_altitude(
    endstates_body: np.ndarray,
    position_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    altitude_m: float,
) -> np.ndarray:
    """Apply the fixed-height Route contract to body-frame YOPO end states."""
    states = np.asarray(endstates_body, dtype=np.float64).copy()
    position = np.asarray(position_world, dtype=np.float64)
    rotation = np.asarray(rotation_body_to_world, dtype=np.float64)
    if states.ndim != 2 or states.shape[1:] != (9,):
        raise ValueError("endstates_body must have shape [N, 9]")
    if position.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("position and rotation shapes are invalid")
    if not math.isfinite(altitude_m):
        raise ValueError("fixed altitude must be finite")
    if not all(np.isfinite(value).all() for value in (states, position, rotation)):
        raise ValueError("fixed altitude projection inputs must be finite")
    endpoint_world = position[None] + states[:, :3] @ rotation.T
    end_velocity_world = states[:, 3:6] @ rotation.T
    end_acceleration_world = states[:, 6:9] @ rotation.T
    endpoint_world[:, 2] = altitude_m
    end_velocity_world[:, 2] = 0.0
    end_acceleration_world[:, 2] = 0.0
    states[:, :3] = (endpoint_world - position[None]) @ rotation
    states[:, 3:6] = end_velocity_world @ rotation
    states[:, 6:9] = end_acceleration_world @ rotation
    return states.astype(np.float32)


def build_route_features(
    centers_world: np.ndarray,
    safe_radii_m: np.ndarray,
    route_mask: np.ndarray,
    sample_distances_m: np.ndarray,
    position_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    *,
    radius_clip_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    centers = np.asarray(centers_world, dtype=np.float64)
    radii = np.asarray(safe_radii_m, dtype=np.float64)
    mask = np.asarray(route_mask, dtype=np.float32)
    distances = np.asarray(sample_distances_m, dtype=np.float64)
    if centers.ndim != 2 or centers.shape[1:] != (3,):
        raise ValueError("route centers must have shape [K, 3]")
    if radii.shape != (len(centers),) or mask.shape != radii.shape or distances.shape != radii.shape:
        raise ValueError("route radius, mask and distance shapes must match centers")
    if radius_clip_m <= 0.0 or np.any(distances <= 0.0):
        raise ValueError("normalization distances and radius clip must be positive")
    centers_body = world_to_body_flu(
        centers, position_world, rotation_body_to_world
    )
    normalized_centers = centers_body / np.maximum(distances[:, None], 1.0)
    normalized_radii = np.clip(radii, 0.0, radius_clip_m) / radius_clip_m
    features = np.concatenate((normalized_centers, normalized_radii[:, None]), axis=1)
    features *= mask[:, None]
    if not np.isfinite(features).all():
        raise ValueError("route features are non-finite")
    return features.astype(np.float32), mask.astype(np.float32)


def route_signature(
    frontier_world: np.ndarray,
    path_world: np.ndarray,
    *,
    resolution_m: float = 0.10,
) -> bytes:
    frontier = np.asarray(frontier_world, dtype=np.float64)
    path = np.asarray(path_world, dtype=np.float64)
    if frontier.shape != (3,) or path.ndim != 2 or path.shape[1:] != (3,) or len(path) < 2:
        raise ValueError("route signature requires a frontier and at least two path points")
    if resolution_m <= 0.0 or not np.isfinite(frontier).all() or not np.isfinite(path).all():
        raise ValueError("route signature inputs must be finite")
    quantized = np.rint(np.concatenate((frontier[None], path), axis=0) / resolution_m)
    return hashlib.sha256(quantized.astype("<i8", copy=False).tobytes()).digest()


class LocalRouteId:
    """Monotonic compatibility id used until EPIC publishes a source route_id."""

    def __init__(self) -> None:
        self._route_id = 0
        self._signature: bytes | None = None

    @property
    def value(self) -> int:
        return self._route_id

    def observe(self, signature: bytes) -> int:
        if not isinstance(signature, bytes) or not signature:
            raise ValueError("route signature must be non-empty bytes")
        if signature != self._signature:
            self._route_id += 1
            self._signature = signature
        return self._route_id


def _coefficient_map(segment_time_s: float) -> np.ndarray:
    duration = float(segment_time_s)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("segment time must be positive and finite")
    system = np.zeros((6, 6), dtype=np.float64)
    for derivative in range(3):
        system[2 * derivative, derivative] = math.factorial(derivative)
        for power in range(derivative, 6):
            system[2 * derivative + 1, power] = (
                math.factorial(power)
                / math.factorial(power - derivative)
                * duration ** (power - derivative)
            )
    reorder = np.zeros((6, 6), dtype=np.float64)
    reorder[[0, 2, 4, 1, 3, 5], np.arange(6)] = 1.0
    return np.linalg.inv(system) @ reorder


def sample_poly5_candidates(
    start_position_world: np.ndarray,
    start_velocity_world: np.ndarray,
    start_acceleration_world: np.ndarray,
    endstates_body: np.ndarray,
    rotation_body_to_world: np.ndarray,
    *,
    segment_time_s: float,
    sample_count: int = 101,
) -> np.ndarray:
    positions, _, _ = sample_poly5_candidate_states(
        start_position_world,
        start_velocity_world,
        start_acceleration_world,
        endstates_body,
        rotation_body_to_world,
        segment_time_s=segment_time_s,
        sample_count=sample_count,
    )
    return positions


def sample_poly5_candidate_states(
    start_position_world: np.ndarray,
    start_velocity_world: np.ndarray,
    start_acceleration_world: np.ndarray,
    endstates_body: np.ndarray,
    rotation_body_to_world: np.ndarray,
    *,
    segment_time_s: float,
    sample_count: int = 101,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = np.asarray(start_position_world, dtype=np.float64)
    velocity = np.asarray(start_velocity_world, dtype=np.float64)
    acceleration = np.asarray(start_acceleration_world, dtype=np.float64)
    endstates = np.asarray(endstates_body, dtype=np.float64)
    rotation = np.asarray(rotation_body_to_world, dtype=np.float64)
    if position.shape != (3,) or velocity.shape != (3,) or acceleration.shape != (3,):
        raise ValueError("start position, velocity and acceleration must be 3-D")
    if endstates.ndim != 2 or endstates.shape[1:] != (9,):
        raise ValueError("endstates_body must have shape [N, 9]")
    if rotation.shape != (3, 3) or sample_count < 2:
        raise ValueError("invalid rotation or sample count")
    if not all(np.isfinite(value).all() for value in (position, velocity, acceleration, rotation)):
        raise ValueError("start state and rotation must be finite")

    count = len(endstates)
    positions = np.full((count, sample_count, 3), np.nan, dtype=np.float32)
    velocities = np.full_like(positions, np.nan)
    accelerations = np.full_like(positions, np.nan)
    coefficient_map = _coefficient_map(segment_time_s)
    times = np.linspace(0.0, segment_time_s, sample_count, dtype=np.float64)
    powers = np.stack([times**power for power in range(6)], axis=1)
    velocity_powers = np.stack(
        [
            np.zeros_like(times),
            np.ones_like(times),
            2.0 * times,
            3.0 * times**2,
            4.0 * times**3,
            5.0 * times**4,
        ],
        axis=1,
    )
    acceleration_powers = np.stack(
        [
            np.zeros_like(times),
            np.zeros_like(times),
            2.0 * np.ones_like(times),
            6.0 * times,
            12.0 * times**2,
            20.0 * times**3,
        ],
        axis=1,
    )
    start = np.stack((position, velocity, acceleration), axis=0)
    for index, endstate in enumerate(endstates):
        if not np.isfinite(endstate).all():
            continue
        end = np.stack(
            (
                position + rotation @ endstate[:3],
                rotation @ endstate[3:6],
                rotation @ endstate[6:9],
            ),
            axis=0,
        )
        boundary = np.concatenate((start, end), axis=0).T
        coefficients = (coefficient_map @ boundary.T).T
        positions[index] = (powers @ coefficients.T).astype(np.float32)
        velocities[index] = (velocity_powers @ coefficients.T).astype(np.float32)
        accelerations[index] = (acceleration_powers @ coefficients.T).astype(np.float32)
    return positions, velocities, accelerations


def select_first_certified(
    scores: np.ndarray,
    trajectories_world: np.ndarray,
    safety_states: Sequence[str],
    *,
    minimum_altitude_m: float,
) -> int | None:
    values = np.asarray(scores, dtype=np.float64).reshape(-1)
    trajectories = np.asarray(trajectories_world, dtype=np.float64)
    if trajectories.ndim != 3 or trajectories.shape[0] != len(values) or trajectories.shape[2] != 3:
        raise ValueError("trajectories must have shape [N, S, 3]")
    if len(safety_states) != len(values):
        raise ValueError("one safety state is required per candidate")
    for index in np.argsort(np.where(np.isfinite(values), values, np.inf), kind="stable"):
        trajectory = trajectories[index]
        if not np.isfinite(values[index]) or not np.isfinite(trajectory).all():
            continue
        if safety_states[index] != "CERTIFIED":
            continue
        if float(np.min(trajectory[:, 2])) < minimum_altitude_m:
            continue
        return int(index)
    return None
