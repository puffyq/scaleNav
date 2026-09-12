from __future__ import annotations

import math
import os
import tempfile
from dataclasses import dataclass, field
from enum import IntFlag
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.spatial import cKDTree


ROUTE_DATASET_VERSION = 2
SUPPORTED_ROUTE_DATASET_VERSIONS = frozenset((1, 2))


class RouteQualityFlag(IntFlag):
    NONE = 0
    NOT_FOUND = 1 << 0
    BLOCKED = 1 << 1
    NOT_COMMITTED = 1 << 2
    NON_FINITE = 1 << 3
    EMPTY_PATH = 1 << 4
    START_MISMATCH = 1 << 5
    TERMINAL_MISMATCH = 1 << 6
    CLEARANCE = 1 << 7
    BUBBLE_GAP = 1 << 8
    SHORT_HORIZON = 1 << 9
    LATTICE_DIRECTION = 1 << 10
    CURVATURE = 1 << 11
    STALE = 1 << 12
    ROUTE_ID_REGRESSION = 1 << 13
    ROUTE_JUMP = 1 << 14


@dataclass(frozen=True)
class RouteQualityConfig:
    robot_radius_m: float = 0.3
    safety_margin_m: float = 0.2
    start_tolerance_m: float = 1.0
    terminal_tolerance_m: float = 1.5
    minimum_execution_length_m: float = 2.0
    maximum_curvature_rad_m: float = 1.6
    maximum_heading_deg: float = 51.0
    target_safe_radius_m: float = 1.2


@dataclass(frozen=True)
class RouteQualityResult:
    flags: RouteQualityFlag
    path_length_m: float
    minimum_clearance_m: float
    maximum_curvature_rad_m: float
    weight: float
    minimum_safe_radius_m: float = 0.0
    safe_radius_p05_m: float = 0.0
    neck_length_m: float = 0.0
    continuous_minimum_clearance_m: float = 0.0
    bubble_overlap_margin_m: float = 0.0

    @property
    def valid(self) -> bool:
        return self.flags == RouteQualityFlag.NONE


@dataclass
class RouteRecord:
    frame_index: int
    mission_goal_world: np.ndarray
    frontier_goal_world: np.ndarray
    path_points_world: np.ndarray
    path_clearance_m: np.ndarray
    path_bubble_radius_m: np.ndarray
    topo_centers_world: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3), dtype=np.float32)
    )
    topo_bubble_radius_m: np.ndarray = field(
        default_factory=lambda: np.empty((0,), dtype=np.float32)
    )
    topo_persistent_id: np.ndarray = field(
        default_factory=lambda: np.empty((0,), dtype=np.uint64)
    )
    route_valid: bool = True
    route_quality_flags: int = 0
    route_quality_weight: float = 1.0
    route_seed: int = 0
    route_search_detour_ratio: float = 1.0
    route_centerline_gain_m: float = 0.0
    local_subgoal_distance_m: float = 0.0


def polyline_arclength(points: np.ndarray) -> tuple[np.ndarray, float]:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("path points must be Nx3")
    if len(points) == 0:
        return np.zeros(0, dtype=np.float32), 0.0
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths, dtype=np.float64)))
    return cumulative.astype(np.float32), float(cumulative[-1])


def maximum_polyline_curvature(points: np.ndarray) -> float:
    points = np.asarray(points, dtype=np.float32)
    if len(points) < 3:
        return 0.0
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    valid = lengths > 1.0e-5
    if valid.sum() < 2:
        return 0.0
    unit = segments / np.maximum(lengths[:, None], 1.0e-6)
    cosine = np.clip(np.sum(unit[:-1] * unit[1:], axis=1), -1.0, 1.0)
    angles = np.arccos(cosine)
    scales = np.maximum(0.5 * (lengths[:-1] + lengths[1:]), 1.0e-3)
    usable = valid[:-1] & valid[1:]
    return float(np.max(angles[usable] / scales[usable])) if usable.any() else 0.0


def resample_polyline(
    points: np.ndarray,
    *,
    max_step_m: float,
    values: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray | None]:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        raise ValueError("path must contain at least two Nx3 points")
    if max_step_m <= 0.0 or not math.isfinite(max_step_m):
        raise ValueError("max_step_m must be finite and positive")
    if not np.isfinite(points).all():
        raise ValueError("path contains non-finite points")
    values_array = None if values is None else np.asarray(values, dtype=np.float32)
    if values_array is not None and values_array.shape != (len(points),):
        raise ValueError("values must contain one scalar per path point")

    output_points: list[np.ndarray] = [points[0]]
    output_values: list[float] | None = None if values_array is None else [float(values_array[0])]
    for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
        length = float(np.linalg.norm(end - start))
        if length <= 1.0e-6:
            continue
        steps = max(1, int(math.ceil(length / max_step_m)))
        for step in range(1, steps + 1):
            alpha = step / steps
            output_points.append(start + alpha * (end - start))
            if output_values is not None:
                output_values.append(
                    float(values_array[index] + alpha * (values_array[index + 1] - values_array[index]))
                )
    if len(output_points) < 2:
        raise ValueError("path has no non-zero segment")
    return (
        np.asarray(output_points, dtype=np.float32),
        None if output_values is None else np.asarray(output_values, dtype=np.float32),
    )


def build_witness_corridor(
    path_points_world: np.ndarray,
    obstacle_points_world: np.ndarray,
    *,
    robot_radius_m: float,
    safety_margin_m: float,
    max_step_m: float = 0.25,
    obstacle_tree: cKDTree | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    obstacles = np.asarray(obstacle_points_world, dtype=np.float32)
    if obstacles.ndim != 2 or obstacles.shape[1] != 3 or len(obstacles) == 0:
        raise ValueError("obstacle_points_world must be a non-empty Nx3 array")
    if not np.isfinite(obstacles).all():
        raise ValueError("obstacle points contain non-finite values")
    dense_points, _ = resample_polyline(path_points_world, max_step_m=max_step_m)
    tree = cKDTree(obstacles) if obstacle_tree is None else obstacle_tree
    clearance, _ = tree.query(dense_points, k=1, workers=-1)
    clearance = np.asarray(clearance, dtype=np.float32)
    radius = clearance - float(robot_radius_m) - float(safety_margin_m)
    return dense_points, clearance, radius.astype(np.float32)


class RouteQualityGate:
    def __init__(self, config: RouteQualityConfig = RouteQualityConfig()) -> None:
        self.config = config

    def evaluate(
        self,
        *,
        path_points_world: np.ndarray,
        path_clearance_m: np.ndarray,
        path_bubble_radius_m: np.ndarray,
        start_world: Sequence[float],
        frontier_world: Sequence[float],
        start_rotation_world_body: np.ndarray | None = None,
        found: bool = True,
        blocked: bool = False,
        committed: bool = True,
        allow_short_terminal: bool = False,
    ) -> RouteQualityResult:
        flags = RouteQualityFlag.NONE
        if not found:
            flags |= RouteQualityFlag.NOT_FOUND
        if blocked:
            flags |= RouteQualityFlag.BLOCKED
        if not committed:
            flags |= RouteQualityFlag.NOT_COMMITTED

        points = np.asarray(path_points_world, dtype=np.float32)
        clearance = np.asarray(path_clearance_m, dtype=np.float32)
        radii = np.asarray(path_bubble_radius_m, dtype=np.float32)
        start = np.asarray(start_world, dtype=np.float32)
        frontier = np.asarray(frontier_world, dtype=np.float32)
        shapes_valid = (
            points.ndim == 2
            and points.shape[1:] == (3,)
            and clearance.shape == (len(points),)
            and radii.shape == (len(points),)
            and start.shape == (3,)
            and frontier.shape == (3,)
        )
        finite = shapes_valid and all(
            np.isfinite(array).all() for array in (points, clearance, radii, start, frontier)
        )
        if not finite:
            flags |= RouteQualityFlag.NON_FINITE
        if not shapes_valid or len(points) < 2:
            flags |= RouteQualityFlag.EMPTY_PATH
            return RouteQualityResult(flags, 0.0, -math.inf, math.inf, 0.0)
        if not finite:
            return RouteQualityResult(flags, 0.0, 0.0, 0.0, 0.0)

        cumulative, path_length = polyline_arclength(points)
        if path_length <= 1.0e-5:
            flags |= RouteQualityFlag.EMPTY_PATH
        if float(np.linalg.norm(points[0] - start)) > self.config.start_tolerance_m:
            flags |= RouteQualityFlag.START_MISMATCH
        if float(np.linalg.norm(points[-1] - frontier)) > self.config.terminal_tolerance_m:
            flags |= RouteQualityFlag.TERMINAL_MISMATCH

        minimum_clearance = float(np.min(clearance)) if len(clearance) else -math.inf
        required_clearance = self.config.robot_radius_m + self.config.safety_margin_m
        safe_radius = clearance - required_clearance
        minimum_safe_radius = float(np.min(safe_radius))
        safe_radius_p05 = float(np.percentile(safe_radius, 5))
        if minimum_clearance + 1.0e-5 < required_clearance or np.any(radii <= 0.0):
            flags |= RouteQualityFlag.CLEARANCE
        distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
        overlap_margin = radii[:-1] + radii[1:] - distances
        minimum_overlap_margin = float(np.min(overlap_margin)) if len(overlap_margin) else 0.0
        if minimum_overlap_margin < -1.0e-5:
            flags |= RouteQualityFlag.BUBBLE_GAP
        segment_safe_radius = np.minimum(safe_radius[:-1], safe_radius[1:])
        neck_length = float(
            np.sum(distances[segment_safe_radius < self.config.target_safe_radius_m])
        )
        # Distance-to-obstacle is 1-Lipschitz. Endpoint samples therefore give
        # a conservative lower bound for the unobserved middle of each segment.
        continuous_segment_clearance = (
            np.minimum(clearance[:-1], clearance[1:]) - 0.5 * distances
        )
        continuous_minimum_clearance = float(
            min(minimum_clearance, np.min(continuous_segment_clearance))
        )
        if continuous_minimum_clearance + 1.0e-5 < required_clearance:
            flags |= RouteQualityFlag.CLEARANCE
        if path_length < self.config.minimum_execution_length_m and not allow_short_terminal:
            flags |= RouteQualityFlag.SHORT_HORIZON

        curvature = maximum_polyline_curvature(points)
        if curvature > self.config.maximum_curvature_rad_m:
            flags |= RouteQualityFlag.CURVATURE
        if start_rotation_world_body is not None and path_length > 1.0e-5:
            rotation = np.asarray(start_rotation_world_body, dtype=np.float32)
            target_s = min(max(1.0, 0.1 * path_length), path_length)
            target = _interpolate_at_arclength(points, cumulative, target_s)
            direction_body = (target - start) @ rotation
            heading = math.degrees(math.atan2(float(direction_body[1]), float(direction_body[0])))
            if direction_body[0] <= 0.0 or abs(heading) > self.config.maximum_heading_deg:
                flags |= RouteQualityFlag.LATTICE_DIRECTION

        clearance_margin = max(0.0, minimum_clearance - required_clearance)
        clearance_score = min(1.0, clearance_margin / max(required_clearance, 1.0e-3))
        curvature_score = max(0.0, 1.0 - curvature / max(self.config.maximum_curvature_rad_m, 1.0e-3))
        weight = 0.5 + 0.25 * clearance_score + 0.25 * curvature_score
        if flags != RouteQualityFlag.NONE:
            weight = 0.0
        return RouteQualityResult(
            flags,
            path_length,
            minimum_clearance,
            curvature,
            float(weight),
            minimum_safe_radius,
            safe_radius_p05,
            neck_length,
            continuous_minimum_clearance,
            minimum_overlap_margin,
        )


def _interpolate_at_arclength(
    points: np.ndarray, cumulative: np.ndarray, distance_m: float
) -> np.ndarray:
    distance = float(np.clip(distance_m, 0.0, float(cumulative[-1])))
    right = int(np.searchsorted(cumulative, distance, side="right"))
    if right <= 0:
        return points[0].copy()
    if right >= len(points):
        return points[-1].copy()
    left = right - 1
    span = float(cumulative[right] - cumulative[left])
    alpha = 0.0 if span <= 1.0e-8 else (distance - float(cumulative[left])) / span
    return points[left] + alpha * (points[right] - points[left])


def local_subgoal_on_witness(
    path_points_world: np.ndarray, distance_m: float = 10.0
) -> tuple[np.ndarray, float]:
    """Interpolate YOPO's local goal while retaining the complete witness."""
    if distance_m <= 0.0 or not math.isfinite(distance_m):
        raise ValueError("local subgoal distance must be finite and positive")
    points = np.asarray(path_points_world, dtype=np.float32)
    cumulative, length = polyline_arclength(points)
    if len(points) < 2 or length <= 1.0e-5:
        raise ValueError("witness must contain a non-zero segment")
    actual_distance = min(float(distance_m), length)
    return (
        _interpolate_at_arclength(points, cumulative, actual_distance).astype(
            np.float32, copy=False
        ),
        actual_distance,
    )


def sample_route_bubbles(
    path_points: np.ndarray,
    safe_radii: np.ndarray,
    anchors_m: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = np.asarray(path_points, dtype=np.float32)
    radii = np.asarray(safe_radii, dtype=np.float32)
    anchors = np.asarray(anchors_m, dtype=np.float32)
    if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 2:
        raise ValueError("path_points must contain at least two Nx3 points")
    if radii.shape != (len(points),) or anchors.ndim != 1 or len(anchors) == 0:
        raise ValueError("invalid route radii or anchors")
    if np.any(np.diff(anchors) <= 0.0) or anchors[0] <= 0.0:
        raise ValueError("route anchors must be positive and strictly increasing")
    cumulative, length = polyline_arclength(points)
    valid_slots = int(np.count_nonzero(anchors <= length + 1.0e-5))
    candidates: list[tuple[float, float]] = [
        (float(anchor), 1.0) for anchor in anchors if anchor <= length + 1.0e-5
    ]
    if len(points) >= 3 and valid_slots:
        segments = np.diff(points, axis=0)
        segment_lengths = np.linalg.norm(segments, axis=1)
        units = segments / np.maximum(segment_lengths[:, None], 1.0e-6)
        angles = np.arccos(np.clip(np.sum(units[:-1] * units[1:], axis=1), -1.0, 1.0))
        scales = np.maximum(0.5 * (segment_lengths[:-1] + segment_lengths[1:]), 1.0e-3)
        curvature = angles / scales
        for index in np.flatnonzero(curvature > 0.15):
            candidates.append((float(cumulative[index + 1]), 3.0 + float(curvature[index])))
        for index in range(1, len(radii) - 1):
            if radii[index] < radii[index - 1] and radii[index] <= radii[index + 1]:
                candidates.append((float(cumulative[index]), 2.5 + float(radii.max() - radii[index])))
    deduplicated: list[tuple[float, float]] = []
    for distance, priority in sorted(candidates, key=lambda item: item[0]):
        if deduplicated and abs(distance - deduplicated[-1][0]) < 0.2:
            if priority > deduplicated[-1][1]:
                deduplicated[-1] = (distance, priority)
        else:
            deduplicated.append((distance, priority))
    selected = sorted(deduplicated, key=lambda item: item[1], reverse=True)[:valid_slots]
    if len(selected) < valid_slots:
        for anchor in anchors[:valid_slots]:
            if all(abs(float(anchor) - item[0]) >= 0.2 for item in selected):
                selected.append((float(anchor), 1.0))
            if len(selected) == valid_slots:
                break
    selected = sorted(selected, key=lambda item: item[0])
    sample_distances = np.asarray(
        [item[0] for item in selected] + [float(anchor) for anchor in anchors[valid_slots:]],
        dtype=np.float32,
    )
    if len(sample_distances) != len(anchors):
        raise RuntimeError("route sampler did not produce the configured bubble count")
    centers = np.stack(
        [
            _interpolate_at_arclength(points, cumulative, min(float(distance), length))
            for distance in sample_distances
        ]
    ).astype(np.float32)
    sampled_radii = np.empty(len(anchors), dtype=np.float32)
    for index in range(len(anchors)):
        # The sphere is certified at its own center.  Using the minimum radius
        # over the whole interval creates an artificial narrow waist whenever
        # a single dense path sample is close to an obstacle.  Keep the local
        # ESDF radius at the interpolated center; neighboring spheres remain
        # independently safe and their overlap is audited by the route gate.
        sampled_radii[index] = float(np.interp(
            min(float(sample_distances[index]), length), cumulative, radii
        ))
    # Every configured slot is a usable point.  Anchors beyond a short route
    # are clamped to the terminal point, so the ordered polyline remains a
    # fixed-size input without a separate validity-mask concept.
    return centers, sampled_radii, sample_distances


def dense_route_arrays(
    path_points: np.ndarray,
    safe_radii: np.ndarray,
    *,
    count: int,
    step_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    points, radii = resample_polyline(path_points, max_step_m=step_m, values=safe_radii)
    assert radii is not None
    if len(points) > count:
        points = points[:count]
        radii = radii[:count]
    valid_count = len(points)
    output_points = np.repeat(points[-1:], count, axis=0)
    output_radii = np.repeat(radii[-1:], count, axis=0)
    output_points[:valid_count] = points
    output_radii[:valid_count] = radii
    return output_points, output_radii


class RouteTable:
    REQUIRED_FIELDS = {
        "frame_index",
        "mission_goal_world",
        "frontier_goal_world",
        "path_offsets",
        "path_points_world",
        "path_clearance_m",
        "path_bubble_radius_m",
        "topo_offsets",
        "topo_centers_world",
        "topo_bubble_radius_m",
        "topo_persistent_id",
        "path_length_m",
        "route_valid",
        "route_quality_flags",
        "route_quality_weight",
        "route_min_clearance_m",
        "route_max_curvature",
        "route_seed",
        "route_dataset_version",
    }

    def __init__(self, arrays: dict[str, np.ndarray]) -> None:
        self.arrays = arrays
        self.validate()

    def __len__(self) -> int:
        return int(len(self.arrays["frame_index"]))

    def path(self, index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        offsets = self.arrays["path_offsets"]
        start, end = int(offsets[index]), int(offsets[index + 1])
        return (
            self.arrays["path_points_world"][start:end],
            self.arrays["path_clearance_m"][start:end],
            self.arrays["path_bubble_radius_m"][start:end],
        )

    def topology(self, index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        offsets = self.arrays["topo_offsets"]
        start, end = int(offsets[index]), int(offsets[index + 1])
        return (
            self.arrays["topo_centers_world"][start:end],
            self.arrays["topo_bubble_radius_m"][start:end],
            self.arrays["topo_persistent_id"][start:end],
        )

    def validate(self, frame_count: int | None = None) -> None:
        missing = self.REQUIRED_FIELDS.difference(self.arrays)
        if missing:
            raise ValueError(f"routes.npz is missing fields: {sorted(missing)}")
        version = int(np.asarray(self.arrays["route_dataset_version"]).item())
        if version not in SUPPORTED_ROUTE_DATASET_VERSIONS:
            raise ValueError(f"unsupported route dataset version: {version}")
        route_count = len(self.arrays["frame_index"])
        expected_route_shapes = {
            "mission_goal_world": (route_count, 3),
            "frontier_goal_world": (route_count, 3),
            "path_offsets": (route_count + 1,),
            "topo_offsets": (route_count + 1,),
        }
        for name, shape in expected_route_shapes.items():
            if self.arrays[name].shape != shape:
                raise ValueError(f"{name} has shape {self.arrays[name].shape}, expected {shape}")
        for name in (
            "path_length_m", "route_valid", "route_quality_flags", "route_quality_weight",
            "route_min_clearance_m", "route_max_curvature", "route_seed",
        ):
            if self.arrays[name].shape != (route_count,):
                raise ValueError(f"{name} must have one value per route")
        for name in (
            "route_min_safe_radius_m",
            "route_safe_radius_p05_m",
            "route_neck_length_m",
            "route_continuous_min_clearance_m",
            "route_bubble_overlap_margin_m",
            "route_search_detour_ratio",
            "route_centerline_gain_m",
        ):
            if name in self.arrays and self.arrays[name].shape != (route_count,):
                raise ValueError(f"{name} must have one value per route")
        if version >= 2:
            distances = self.arrays.get("local_subgoal_distance_m")
            if distances is None or distances.shape != (route_count,):
                raise ValueError("V2 routes require local_subgoal_distance_m per route")
            local_goals = self.arrays.get("local_subgoal_world")
            if local_goals is None or local_goals.shape != (route_count, 3):
                raise ValueError("V2 routes require local_subgoal_world per route")
        _validate_offsets(self.arrays["path_offsets"], len(self.arrays["path_points_world"]), "path")
        _validate_offsets(self.arrays["topo_offsets"], len(self.arrays["topo_centers_world"]), "topology")
        path_count = len(self.arrays["path_points_world"])
        topo_count = len(self.arrays["topo_centers_world"])
        if self.arrays["path_points_world"].shape != (path_count, 3):
            raise ValueError("path_points_world must be Px3")
        if (
            self.arrays["path_clearance_m"].shape != (path_count,)
            or self.arrays["path_bubble_radius_m"].shape != (path_count,)
        ):
            raise ValueError("path scalar arrays must have P values")
        if self.arrays["topo_centers_world"].shape != (topo_count, 3):
            raise ValueError("topo_centers_world must be Tx3")
        if (
            self.arrays["topo_bubble_radius_m"].shape != (topo_count,)
            or self.arrays["topo_persistent_id"].shape != (topo_count,)
        ):
            raise ValueError("topology scalar arrays must have T values")
        for name, array in self.arrays.items():
            if np.issubdtype(array.dtype, np.floating) and not np.isfinite(array).all():
                raise ValueError(f"{name} contains non-finite values")
        frame_indices = self.arrays["frame_index"]
        if np.any(frame_indices < 0) or (frame_count is not None and np.any(frame_indices >= frame_count)):
            raise ValueError("frame_index is outside data.toml")
        for index in range(route_count):
            points, clearance, radii = self.path(index)
            if bool(self.arrays["route_valid"][index]):
                if len(points) < 2 or np.any(clearance <= 0.0) or np.any(radii <= 0.0):
                    raise ValueError(f"valid route {index} has an invalid witness corridor")
                if version >= 2:
                    requested_distance = float(
                        self.arrays["local_subgoal_distance_m"][index]
                    )
                    expected_goal, actual_distance = local_subgoal_on_witness(
                        points, requested_distance
                    )
                    if abs(actual_distance - requested_distance) > 1.0e-4:
                        raise ValueError(f"valid route {index} local subgoal exceeds witness")
                    if np.linalg.norm(
                        expected_goal - self.arrays["local_subgoal_world"][index]
                    ) > 1.0e-3:
                        raise ValueError(
                            f"valid route {index} local subgoal is not on witness"
                        )


def _validate_offsets(offsets: np.ndarray, final_size: int, name: str) -> None:
    if not np.issubdtype(offsets.dtype, np.integer):
        raise ValueError(f"{name}_offsets must be integral")
    if int(offsets[0]) != 0 or int(offsets[-1]) != final_size or np.any(np.diff(offsets) < 0):
        raise ValueError(f"invalid {name}_offsets")


def pack_route_records(records: Iterable[RouteRecord]) -> RouteTable:
    records = list(records)
    path_offsets = [0]
    topo_offsets = [0]
    path_points: list[np.ndarray] = []
    path_clearance: list[np.ndarray] = []
    path_radii: list[np.ndarray] = []
    topo_centers: list[np.ndarray] = []
    topo_radii: list[np.ndarray] = []
    topo_ids: list[np.ndarray] = []
    path_lengths: list[float] = []
    path_min_clearance: list[float] = []
    path_min_safe_radius: list[float] = []
    path_safe_radius_p05: list[float] = []
    path_neck_length: list[float] = []
    path_continuous_min_clearance: list[float] = []
    path_overlap_margin: list[float] = []
    path_curvature: list[float] = []
    local_subgoal_distances: list[float] = []
    local_subgoals: list[np.ndarray] = []
    for record in records:
        points = np.asarray(record.path_points_world, dtype=np.float32).reshape(-1, 3)
        clearance = np.asarray(record.path_clearance_m, dtype=np.float32).reshape(-1)
        radii = np.asarray(record.path_bubble_radius_m, dtype=np.float32).reshape(-1)
        centers = np.asarray(record.topo_centers_world, dtype=np.float32).reshape(-1, 3)
        topo_radius = np.asarray(record.topo_bubble_radius_m, dtype=np.float32).reshape(-1)
        ids = np.asarray(record.topo_persistent_id, dtype=np.uint64).reshape(-1)
        path_points.append(points)
        path_clearance.append(clearance)
        path_radii.append(radii)
        topo_centers.append(centers)
        topo_radii.append(topo_radius)
        topo_ids.append(ids)
        path_offsets.append(path_offsets[-1] + len(points))
        topo_offsets.append(topo_offsets[-1] + len(centers))
        path_length = polyline_arclength(points)[1] if len(points) else 0.0
        path_lengths.append(path_length)
        requested_subgoal = float(record.local_subgoal_distance_m)
        local_distance = (
            requested_subgoal if requested_subgoal > 0.0 else min(10.0, path_length)
        )
        local_subgoal_distances.append(local_distance)
        if len(points) >= 2 and local_distance > 0.0:
            local_subgoals.append(local_subgoal_on_witness(points, local_distance)[0])
        else:
            # Failed/blocked records are retained for diagnostics, but have no
            # witness from which a local goal can be interpolated.
            local_subgoals.append(
                np.asarray(record.frontier_goal_world, dtype=np.float32).reshape(3)
            )
        path_min_clearance.append(float(np.min(clearance)) if len(clearance) else 0.0)
        path_min_safe_radius.append(float(np.min(radii)) if len(radii) else 0.0)
        path_safe_radius_p05.append(float(np.percentile(radii, 5)) if len(radii) else 0.0)
        distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
        segment_radii = np.minimum(radii[:-1], radii[1:])
        path_neck_length.append(float(np.sum(distances[segment_radii < 1.2])))
        segment_clearance = np.minimum(clearance[:-1], clearance[1:]) - 0.5 * distances
        path_continuous_min_clearance.append(
            float(min(np.min(clearance), np.min(segment_clearance))) if len(clearance) > 1 else 0.0
        )
        overlap = radii[:-1] + radii[1:] - distances
        path_overlap_margin.append(float(np.min(overlap)) if len(overlap) else 0.0)
        path_curvature.append(maximum_polyline_curvature(points))

    def concatenate(parts: list[np.ndarray], shape: tuple[int, ...], dtype: np.dtype) -> np.ndarray:
        return np.concatenate(parts, axis=0).astype(dtype, copy=False) if parts else np.empty(shape, dtype=dtype)

    route_count = len(records)
    arrays = {
        "route_dataset_version": np.asarray(ROUTE_DATASET_VERSION, dtype=np.int64),
        "frame_index": np.asarray([r.frame_index for r in records], dtype=np.int64),
        "mission_goal_world": np.asarray(
            [r.mission_goal_world for r in records], dtype=np.float32
        ).reshape(route_count, 3),
        "frontier_goal_world": np.asarray(
            [r.frontier_goal_world for r in records], dtype=np.float32
        ).reshape(route_count, 3),
        "local_subgoal_world": np.asarray(
            local_subgoals, dtype=np.float32
        ).reshape(route_count, 3),
        "path_offsets": np.asarray(path_offsets, dtype=np.int64),
        "path_points_world": concatenate(path_points, (0, 3), np.float32),
        "path_clearance_m": concatenate(path_clearance, (0,), np.float32),
        "path_bubble_radius_m": concatenate(path_radii, (0,), np.float32),
        "topo_offsets": np.asarray(topo_offsets, dtype=np.int64),
        "topo_centers_world": concatenate(topo_centers, (0, 3), np.float32),
        "topo_bubble_radius_m": concatenate(topo_radii, (0,), np.float32),
        "topo_persistent_id": concatenate(topo_ids, (0,), np.uint64),
        "path_length_m": np.asarray(path_lengths, dtype=np.float32),
        "route_valid": np.asarray([r.route_valid for r in records], dtype=np.uint8),
        "route_quality_flags": np.asarray([r.route_quality_flags for r in records], dtype=np.uint32),
        "route_quality_weight": np.asarray([r.route_quality_weight for r in records], dtype=np.float32),
        "route_min_clearance_m": np.asarray(path_min_clearance, dtype=np.float32),
        "route_min_safe_radius_m": np.asarray(path_min_safe_radius, dtype=np.float32),
        "route_safe_radius_p05_m": np.asarray(path_safe_radius_p05, dtype=np.float32),
        "route_neck_length_m": np.asarray(path_neck_length, dtype=np.float32),
        "route_continuous_min_clearance_m": np.asarray(
            path_continuous_min_clearance, dtype=np.float32
        ),
        "route_bubble_overlap_margin_m": np.asarray(path_overlap_margin, dtype=np.float32),
        "route_max_curvature": np.asarray(path_curvature, dtype=np.float32),
        "route_seed": np.asarray([r.route_seed for r in records], dtype=np.int64),
        "route_search_detour_ratio": np.asarray(
            [r.route_search_detour_ratio for r in records], dtype=np.float32
        ),
        "route_centerline_gain_m": np.asarray(
            [r.route_centerline_gain_m for r in records], dtype=np.float32
        ),
        "local_subgoal_distance_m": np.asarray(
            local_subgoal_distances, dtype=np.float32
        ),
    }
    return RouteTable(arrays)


def save_route_table(path: Path, table: RouteTable) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, suffix=".npz", delete=False) as temporary:
        temporary_path = Path(temporary.name)
        np.savez_compressed(temporary, **table.arrays)
    try:
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return path


def load_route_table(path: Path, *, frame_count: int | None = None) -> RouteTable:
    path = Path(path)
    with np.load(path, allow_pickle=False) as archive:
        arrays = {name: archive[name] for name in archive.files}
    table = RouteTable(arrays)
    table.validate(frame_count=frame_count)
    return table
