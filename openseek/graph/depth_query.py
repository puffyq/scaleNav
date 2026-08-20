from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


class ValidationState(str, Enum):
    CERTIFIED = "CERTIFIED"
    UNVALIDATED = "UNVALIDATED"
    INVALID = "INVALID"


@dataclass(frozen=True)
class ValidationResult:
    state: ValidationState
    clearance_m: float
    known_fraction: float
    checked_samples: int


class DepthSafeVolumeQuery:
    """Conservative swept-sphere checks directly against one DepthPlanar frame."""

    def __init__(
        self,
        depth_m: np.ndarray,
        *,
        horizontal_fov_deg: float = 90.0,
        vertical_fov_deg: float = 60.0,
        robot_radius_m: float = 0.6,
        safety_margin_m: float = 0.15,
        sample_step_m: float = 0.25,
        min_depth_m: float = 0.05,
        far_depth_m: float = 20.0,
        max_unknown_fraction: float = 0.35,
    ) -> None:
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim != 2 or min(depth.shape) < 2:
            raise ValueError("depth_m must be a two-dimensional image")
        if not 0.0 < horizontal_fov_deg < 180.0:
            raise ValueError("horizontal_fov_deg must be between 0 and 180")
        if not 0.0 < vertical_fov_deg < 180.0:
            raise ValueError("vertical_fov_deg must be between 0 and 180")
        if robot_radius_m <= 0.0 or safety_margin_m < 0.0:
            raise ValueError("robot radius must be positive and margin non-negative")
        if sample_step_m <= 0.0:
            raise ValueError("sample_step_m must be positive")
        if far_depth_m <= min_depth_m:
            raise ValueError("far depth must be greater than minimum depth")
        if not 0.0 <= max_unknown_fraction < 1.0:
            raise ValueError("max_unknown_fraction must be in [0, 1)")

        self.depth = depth
        self.height, self.width = depth.shape
        self.robot_radius_m = float(robot_radius_m)
        self.safety_margin_m = float(safety_margin_m)
        self.swept_radius_m = self.robot_radius_m + self.safety_margin_m
        self.sample_step_m = float(sample_step_m)
        self.min_depth_m = float(min_depth_m)
        self.far_depth_m = float(far_depth_m)
        self.max_unknown_fraction = float(max_unknown_fraction)
        self.fx = 0.5 * self.width / math.tan(math.radians(horizontal_fov_deg) * 0.5)
        self.fy = 0.5 * self.height / math.tan(math.radians(vertical_fov_deg) * 0.5)
        self.cx = 0.5 * (self.width - 1)
        self.cy = 0.5 * (self.height - 1)

    def project(self, point_body_flu: np.ndarray) -> tuple[float, float] | None:
        point = np.asarray(point_body_flu, dtype=np.float64)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("point_body_flu must contain three finite values")
        forward, left, up = point
        if forward <= self.min_depth_m:
            return None
        u = self.cx - self.fx * left / forward
        v = self.cy - self.fy * up / forward
        if u < 0.0 or u > self.width - 1 or v < 0.0 or v > self.height - 1:
            return None
        return float(u), float(v)

    def validate_segment(
        self,
        start_body_flu: np.ndarray,
        end_body_flu: np.ndarray,
    ) -> ValidationResult:
        start = np.asarray(start_body_flu, dtype=np.float64)
        end = np.asarray(end_body_flu, dtype=np.float64)
        if start.shape != (3,) or end.shape != (3,):
            raise ValueError("segment endpoints must be 3-D")
        if not np.isfinite(start).all() or not np.isfinite(end).all():
            raise ValueError("segment endpoints must be finite")
        length = float(np.linalg.norm(end - start))
        if length <= 1e-6:
            return ValidationResult(ValidationState.CERTIFIED, math.inf, 1.0, 0)

        sample_count = max(2, int(math.ceil(length / self.sample_step_m)) + 1)
        minimum_clearance = math.inf
        known_pixels = 0
        requested_pixels = 0
        checked_samples = 0
        any_projectable = False
        saw_far_unknown = False
        for progress in np.linspace(0.0, 1.0, sample_count):
            point = start + progress * (end - start)
            forward = float(point[0])
            # The volume surrounding the camera is occupied by the vehicle and
            # cannot be observed by a forward camera. Treat it as the certified
            # starting volume instead of turning every edge into UNKNOWN.
            if forward <= max(self.min_depth_m, 2.0 * self.swept_radius_m):
                continue
            projection = self.project(point)
            if projection is None:
                requested_pixels += 1
                continue
            any_projectable = True
            checked_samples += 1
            u, v = projection
            radius_u = max(1, int(math.ceil(self.fx * self.swept_radius_m / forward)))
            radius_v = max(1, int(math.ceil(self.fy * self.swept_radius_m / forward)))
            requested_pixels += self._ellipse_pixel_count(radius_u, radius_v)

            u0 = max(0, int(math.floor(u)) - radius_u)
            u1 = min(self.width - 1, int(math.ceil(u)) + radius_u)
            v0 = max(0, int(math.floor(v)) - radius_v)
            v1 = min(self.height - 1, int(math.ceil(v)) + radius_v)
            columns = np.arange(u0, u1 + 1, dtype=np.float32)
            rows = np.arange(v0, v1 + 1, dtype=np.float32)
            grid_u, grid_v = np.meshgrid(columns, rows)
            mask = ((grid_u - u) / radius_u) ** 2 + ((grid_v - v) / radius_v) ** 2 <= 1.0
            values = self.depth[v0 : v1 + 1, u0 : u1 + 1][mask]
            measured = np.isfinite(values) & (values >= self.min_depth_m)
            saturated = measured & (values >= self.far_depth_m - 1e-3)
            # A far-clipped ray certifies this sample only while the swept
            # volume still ends before the camera far plane. At the far plane
            # itself the continuation is unknown, not occupied.
            saturated_safe = saturated & (
                forward + self.swept_radius_m < self.far_depth_m - 1e-3
            )
            saw_far_unknown = saw_far_unknown or bool(np.any(saturated & ~saturated_safe))
            known = (measured & ~saturated) | saturated_safe
            known_pixels += int(known.sum())
            if not known.any():
                continue
            clearance_values = values[known].copy()
            clearance_values[saturated_safe[known]] = self.far_depth_m
            clearance = clearance_values - forward
            minimum_clearance = min(minimum_clearance, float(clearance.min()))
            if np.any(
                values[measured & ~saturated] <= forward + self.swept_radius_m
            ):
                known_fraction = min(1.0, known_pixels / max(requested_pixels, 1))
                return ValidationResult(
                    ValidationState.INVALID,
                    minimum_clearance,
                    known_fraction,
                    checked_samples,
                )

        known_fraction = min(1.0, known_pixels / max(requested_pixels, 1))
        if (
            not any_projectable
            or saw_far_unknown
            or known_fraction < 1.0 - self.max_unknown_fraction
        ):
            state = ValidationState.UNVALIDATED
        else:
            state = ValidationState.CERTIFIED
        return ValidationResult(state, minimum_clearance, known_fraction, checked_samples)

    def validate_optimistic_segment(
        self,
        start_body_flu: np.ndarray,
        end_body_flu: np.ndarray,
    ) -> ValidationResult:
        """Reject only intersections with surfaces measured in this frame.

        This is for Graph edges whose start is not the current camera origin.
        Such an edge cannot be certified by this frame, but a foreground depth
        sample must not be treated as a collision merely because it occludes a
        later point on the projected edge.
        """
        start = np.asarray(start_body_flu, dtype=np.float64)
        end = np.asarray(end_body_flu, dtype=np.float64)
        if start.shape != (3,) or end.shape != (3,):
            raise ValueError("segment endpoints must be 3-D")
        if not np.isfinite(start).all() or not np.isfinite(end).all():
            raise ValueError("segment endpoints must be finite")
        segment = end - start
        length_squared = float(segment @ segment)
        if length_squared <= 1e-12:
            return ValidationResult(ValidationState.UNVALIDATED, math.inf, 0.0, 0)

        measured = (
            np.isfinite(self.depth)
            & (self.depth >= self.min_depth_m)
            & (self.depth < self.far_depth_m - 1e-3)
        )
        if not measured.any():
            return ValidationResult(ValidationState.UNVALIDATED, math.inf, 0.0, 0)

        rows, columns = np.nonzero(measured)
        forward = self.depth[rows, columns].astype(np.float64)
        points = np.column_stack(
            (
                forward,
                -(columns - self.cx) * forward / self.fx,
                -(rows - self.cy) * forward / self.fy,
            )
        )
        progress = np.clip(((points - start) @ segment) / length_squared, 0.0, 1.0)
        closest = start + progress[:, None] * segment
        distances = np.linalg.norm(points - closest, axis=1)
        pixel_half_diagonal = 0.5 * forward * math.sqrt(
            1.0 / (self.fx * self.fx) + 1.0 / (self.fy * self.fy)
        )
        surface_clearance = distances - pixel_half_diagonal
        minimum_clearance = float(surface_clearance.min())
        state = (
            ValidationState.INVALID
            if minimum_clearance <= 0.0
            else ValidationState.UNVALIDATED
        )
        return ValidationResult(
            state,
            minimum_clearance,
            float(measured.mean()),
            int(points.shape[0]),
        )

    @staticmethod
    def _ellipse_pixel_count(radius_u: int, radius_v: int) -> int:
        columns = np.arange(-radius_u, radius_u + 1, dtype=np.float32)
        rows = np.arange(-radius_v, radius_v + 1, dtype=np.float32)
        grid_u, grid_v = np.meshgrid(columns, rows)
        return int(((grid_u / radius_u) ** 2 + (grid_v / radius_v) ** 2 <= 1.0).sum())
