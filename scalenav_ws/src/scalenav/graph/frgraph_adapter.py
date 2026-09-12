"""ROS-free FRGraph-style direction and free-region extraction.

The upstream FRGraph implementation consumes a ROS point cloud and odometry.
This adapter keeps its useful front-end idea (range-map gaps followed by
direction-aware free regions) while accepting one OpenSeek DepthPlanar frame.
The generated regions are disposable; only the OpenSeek sparse Graph persists.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class FreeRegion:
    region_id: int
    center_yaw_rad: float
    center_elev_rad: float
    yaw_min_rad: float
    yaw_max_rad: float
    elev_min_rad: float
    elev_max_rad: float
    pixel_count: int
    measured_fraction: float
    depth_limit_m: float
    semantic_score: float = 0.0

    @property
    def direction_body_flu(self) -> np.ndarray:
        cp = math.cos(self.center_elev_rad)
        return np.asarray(
            [
                cp * math.cos(self.center_yaw_rad),
                cp * math.sin(self.center_yaw_rad),
                math.sin(self.center_elev_rad),
            ],
            dtype=np.float64,
        )

    def to_dict(self) -> dict:
        return {
            "id": self.region_id,
            "centerYawDeg": math.degrees(self.center_yaw_rad),
            "centerElevDeg": math.degrees(self.center_elev_rad),
            "yawMinDeg": math.degrees(self.yaw_min_rad),
            "yawMaxDeg": math.degrees(self.yaw_max_rad),
            "elevMinDeg": math.degrees(self.elev_min_rad),
            "elevMaxDeg": math.degrees(self.elev_max_rad),
            "pixelCount": self.pixel_count,
            "measuredFraction": self.measured_fraction,
            "depthLimitM": self.depth_limit_m,
            "semanticScore": self.semantic_score,
            "directionBodyFLU": self.direction_body_flu.tolist(),
        }


class FRGraphAdapter:
    """Extract direction-aware free regions from a single DepthPlanar frame."""

    def __init__(
        self,
        *,
        horizontal_fov_deg: float = 90.0,
        vertical_fov_deg: float = 60.0,
        candidate_distance_m: float = 5.0,
        robot_radius_m: float = 0.6,
        safety_margin_m: float = 0.15,
        far_depth_m: float = 20.0,
        max_yaw_span_deg: float = 30.0,
        min_region_pixels: int = 8,
    ) -> None:
        if candidate_distance_m <= 0.0 or robot_radius_m <= 0.0:
            raise ValueError("candidate distance and robot radius must be positive")
        if min_region_pixels < 1:
            raise ValueError("min_region_pixels must be positive")
        self.horizontal_fov_deg = float(horizontal_fov_deg)
        self.vertical_fov_deg = float(vertical_fov_deg)
        self.candidate_distance_m = float(candidate_distance_m)
        self.robot_radius_m = float(robot_radius_m)
        self.safety_margin_m = float(safety_margin_m)
        self.far_depth_m = float(far_depth_m)
        self.max_yaw_span_rad = math.radians(float(max_yaw_span_deg))
        self.min_region_pixels = int(min_region_pixels)

    def extract(
        self,
        depth_m: np.ndarray,
        heatmap: np.ndarray | None = None,
    ) -> list[FreeRegion]:
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim != 2 or min(depth.shape) < 2:
            raise ValueError("depth_m must be a two-dimensional image")
        height, width = depth.shape
        fx = 0.5 * width / math.tan(math.radians(self.horizontal_fov_deg) * 0.5)
        fy = 0.5 * height / math.tan(math.radians(self.vertical_fov_deg) * 0.5)
        cx = 0.5 * (width - 1)
        cy = 0.5 * (height - 1)

        rows, columns = np.indices(depth.shape, dtype=np.float32)
        forward = depth.astype(np.float64)
        left = -(columns - cx) * forward / fx
        up = -(rows - cy) * forward / fy
        yaw = np.arctan2(left, np.maximum(forward, 1e-6))
        elev = np.arctan2(up, np.maximum(np.hypot(forward, left), 1e-6))

        measured = np.isfinite(depth) & (depth >= 0.05) & (
            depth < self.far_depth_m - 1e-3
        )
        # Unknown and far-clipped rays are optimistic free space. A measured
        # ray is free at the candidate distance only when its surface is beyond
        # the robot envelope.
        required_depth = self.candidate_distance_m + self.robot_radius_m + self.safety_margin_m
        free = (~measured) | (depth >= required_depth)

        components = self._connected_components(free)
        regions: list[FreeRegion] = []
        next_id = 0
        for component in components:
            if len(component) < self.min_region_pixels:
                continue
            regions.extend(
                self._split_component(
                    next_id,
                    component,
                    yaw,
                    elev,
                    depth,
                    measured,
                    heatmap,
                )
            )
            next_id = len(regions)
        return regions

    @staticmethod
    def _connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
        height, width = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        components: list[list[tuple[int, int]]] = []
        for row in range(height):
            for column in range(width):
                if not mask[row, column] or visited[row, column]:
                    continue
                stack = [(row, column)]
                visited[row, column] = True
                component: list[tuple[int, int]] = []
                while stack:
                    current_row, current_column = stack.pop()
                    component.append((current_row, current_column))
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr = current_row + dr
                            nc = current_column + dc
                            if (
                                0 <= nr < height
                                and 0 <= nc < width
                                and mask[nr, nc]
                                and not visited[nr, nc]
                            ):
                                visited[nr, nc] = True
                                stack.append((nr, nc))
                components.append(component)
        return components

    def _split_component(
        self,
        first_id: int,
        component: list[tuple[int, int]],
        yaw: np.ndarray,
        elev: np.ndarray,
        depth: np.ndarray,
        measured: np.ndarray,
        heatmap: np.ndarray | None,
    ) -> list[FreeRegion]:
        values = np.asarray([yaw[row, column] for row, column in component])
        yaw_min = float(values.min())
        yaw_max = float(values.max())
        span = max(yaw_max - yaw_min, 1e-6)
        bins = max(1, int(math.ceil(span / self.max_yaw_span_rad)))
        result: list[FreeRegion] = []
        for bin_index in range(bins):
            lower = yaw_min + span * bin_index / bins
            upper = yaw_min + span * (bin_index + 1) / bins
            selected = [
                (row, column)
                for row, column in component
                if (yaw[row, column] >= lower)
                and (yaw[row, column] <= upper or bin_index == bins - 1)
            ]
            if len(selected) < self.min_region_pixels:
                continue
            selected_yaw = np.asarray([yaw[row, column] for row, column in selected])
            selected_elev = np.asarray([elev[row, column] for row, column in selected])
            selected_depth = np.asarray([depth[row, column] for row, column in selected])
            selected_measured = np.asarray(
                [measured[row, column] for row, column in selected], dtype=bool
            )
            finite_depth = selected_depth[selected_measured]
            depth_limit = (
                float(np.median(finite_depth))
                if finite_depth.size
                else self.far_depth_m
            )
            semantic_score = self._mean_heatmap(heatmap, selected, yaw.shape)
            result.append(
                FreeRegion(
                    region_id=first_id + len(result),
                    center_yaw_rad=float(selected_yaw.mean()),
                    center_elev_rad=float(selected_elev.mean()),
                    yaw_min_rad=float(selected_yaw.min()),
                    yaw_max_rad=float(selected_yaw.max()),
                    elev_min_rad=float(selected_elev.min()),
                    elev_max_rad=float(selected_elev.max()),
                    pixel_count=len(selected),
                    measured_fraction=float(selected_measured.mean()),
                    depth_limit_m=depth_limit,
                    semantic_score=semantic_score,
                )
            )
        return result

    @staticmethod
    def _mean_heatmap(
        heatmap: np.ndarray | None,
        pixels: list[tuple[int, int]],
        source_shape: tuple[int, int],
    ) -> float:
        if heatmap is None:
            return 0.0
        values = np.asarray(heatmap)
        if values.ndim != 2:
            raise ValueError("heatmap must be a two-dimensional image")
        rows = np.asarray([pixel[0] for pixel in pixels])
        columns = np.asarray([pixel[1] for pixel in pixels])
        scaled_rows = np.minimum(
            values.shape[0] - 1,
            np.rint(rows * (values.shape[0] - 1) / max(source_shape[0] - 1, 1)).astype(int),
        )
        scaled_columns = np.minimum(
            values.shape[1] - 1,
            np.rint(columns * (values.shape[1] - 1) / max(source_shape[1] - 1, 1)).astype(int),
        )
        sampled = values[scaled_rows, scaled_columns]
        return float(np.nanmean(sampled)) if sampled.size else 0.0
