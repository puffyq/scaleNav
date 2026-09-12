from __future__ import annotations

import argparse
import hashlib
import heapq
import json
import math
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence, Union

import cv2
import numpy as np
from scipy.ndimage import label, map_coordinates
from scipy.interpolate import CubicSpline
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import cKDTree

from .coordinates import enu_to_ned
from .route_contract import (
    RouteQualityConfig,
    RouteQualityGate,
    RouteQualityResult,
    RouteRecord,
    build_witness_corridor,
    local_subgoal_on_witness,
    maximum_polyline_curvature,
    pack_route_records,
    resample_polyline,
    save_route_table,
)
from .snapshot_dataset import CaptureConfig, PoseSample, SceneWriter, write_point_cloud_ply


@dataclass(frozen=True)
class CylinderObstacle:
    center_x_m: float
    center_y_m: float
    radius_m: float
    height_m: float


@dataclass(frozen=True)
class BoxObstacle:
    center_x_m: float
    center_y_m: float
    size_x_m: float
    size_y_m: float
    height_m: float
    yaw_rad: float = 0.0


Obstacle = Union[CylinderObstacle, BoxObstacle]


@dataclass(frozen=True)
class RouteSearchResult:
    cells: tuple[tuple[int, int], ...]
    shortest_length_m: float
    path_length_m: float
    minimum_safe_radius_m: float
    safe_radius_p05_m: float
    clearance_threshold_m: float

    @property
    def detour_ratio(self) -> float:
        return self.path_length_m / max(self.shortest_length_m, 1.0e-6)


@dataclass(frozen=True)
class CenterlineRefinementResult:
    points_world: np.ndarray
    minimum_safe_radius_before_m: float
    minimum_safe_radius_after_m: float
    safe_radius_p05_before_m: float
    safe_radius_p05_after_m: float
    clearance_risk_before: float
    clearance_risk_after: float
    iterations: int

    @property
    def gain_m(self) -> float:
        return self.safe_radius_p05_after_m - self.safe_radius_p05_before_m


@dataclass(frozen=True)
class CandidateRoute:
    points_world: np.ndarray
    clearance_m: np.ndarray
    safe_radius_m: np.ndarray
    search: RouteSearchResult
    refinement: CenterlineRefinementResult
    quality: RouteQualityResult


SCENE_STYLES = {
    "alternating",
    "blocks",
    "mixed",
    "forest",
    "yopo_forest",
    "yopo_real_forest",
}
DEFAULT_YOPO_TREE_ASSET = Path(
    "/mnt/code/lab/yopo/YOPO-Simple/Simulator/src/pointcloud/tree.ply"
)


@dataclass(frozen=True)
class GroundTruthConfig:
    map_size_x_m: float = 80.0
    map_size_y_m: float = 80.0
    map_height_m: float = 5.0
    altitude_m: float = 1.6
    grid_resolution_m: float = 0.2
    point_resolution_m: float = 0.2
    obstacle_count: int = 40
    scene_style: str = "alternating"
    forest_fraction: float = 0.20
    tree_radius_min_m: float = 0.3
    tree_radius_max_m: float = 0.75
    wall_length_min_m: float = 2.0
    wall_length_max_m: float = 5.0
    wall_thickness_m: float = 0.35
    block_size_min_m: float = 2.5
    block_size_max_m: float = 30.0
    robot_radius_m: float = 0.3
    safety_margin_m: float = 0.2
    planning_extra_margin_m: float = 0.35
    safe_pose_clearance_m: float = 0.8
    route_min_length_m: float = 7.0
    route_max_length_m: float = 24.0
    local_subgoal_distance_m: float = 10.0
    routes_per_frame: int = 3
    image_width: int = 160
    image_height: int = 96
    horizontal_fov_deg: float = 90.0
    vertical_fov_deg: float = 60.0
    max_depth_m: float = 20.0
    maximum_frame_attempts: int = 250
    maximum_route_attempts: int = 120
    widest_detour_ratio: float = 1.12
    widest_clearance_target_m: float = 1.2
    widest_clearance_step_m: float = 0.2
    clearance_cost_weight: float = 2.0
    clearance_anchor_relaxation_m: float = 1.0
    centerline_iterations: int = 12
    centerline_step_m: float = 0.12
    centerline_max_deviation_m: float = 0.8
    centerline_resample_step_m: float = 0.2

    def __post_init__(self) -> None:
        positive = (
            self.map_size_x_m,
            self.map_size_y_m,
            self.map_height_m,
            self.altitude_m,
            self.grid_resolution_m,
            self.point_resolution_m,
            self.robot_radius_m,
            self.safety_margin_m,
            self.safe_pose_clearance_m,
            self.route_min_length_m,
            self.route_max_length_m,
            self.local_subgoal_distance_m,
            self.max_depth_m,
            self.widest_detour_ratio,
            self.widest_clearance_target_m,
            self.widest_clearance_step_m,
            self.clearance_anchor_relaxation_m,
            self.centerline_step_m,
            self.centerline_max_deviation_m,
            self.centerline_resample_step_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("ground-truth scene dimensions and distances must be positive")
        if self.altitude_m >= self.map_height_m:
            raise ValueError("altitude must be below map height")
        if self.route_min_length_m >= self.route_max_length_m:
            raise ValueError("route_min_length_m must be smaller than route_max_length_m")
        if not 0.0 <= self.forest_fraction <= 1.0:
            raise ValueError("forest_fraction must be in [0, 1]")
        if self.scene_style not in SCENE_STYLES:
            raise ValueError(f"scene_style must be one of {sorted(SCENE_STYLES)}")
        if not 0.0 < self.block_size_min_m <= self.block_size_max_m:
            raise ValueError("invalid block size range")
        if self.obstacle_count < 0 or self.routes_per_frame <= 0:
            raise ValueError("obstacle_count and routes_per_frame are invalid")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image dimensions must be positive")
        if self.widest_detour_ratio < 1.0:
            raise ValueError("widest_detour_ratio must be at least one")
        if self.clearance_cost_weight < 0.0 or not math.isfinite(self.clearance_cost_weight):
            raise ValueError("clearance_cost_weight must be finite and non-negative")
        if self.centerline_iterations < 0:
            raise ValueError("centerline_iterations must be non-negative")


class GroundTruthScene:
    """Analytic obstacles, a fixed-height planning grid, and a shared renderer."""

    def __init__(
        self,
        config: GroundTruthConfig,
        obstacles: Sequence[Obstacle],
        *,
        point_obstacles_world: np.ndarray | None = None,
        route_blocker_centers_xy: np.ndarray | None = None,
        scene_metadata: dict[str, object] | None = None,
    ) -> None:
        self.config = config
        self.obstacles = tuple(obstacles)
        if point_obstacles_world is None:
            self.point_obstacles_world = None
            self._point_obstacle_tree = None
        else:
            points = np.asarray(point_obstacles_world, dtype=np.float32)
            if points.ndim != 2 or points.shape[1:] != (3,) or len(points) == 0:
                raise ValueError("point_obstacles_world must be a non-empty Nx3 array")
            if not np.isfinite(points).all():
                raise ValueError("point_obstacles_world contains non-finite values")
            self.point_obstacles_world = points
            self._point_obstacle_tree = cKDTree(points)
        if route_blocker_centers_xy is None:
            self.route_blocker_centers_xy = np.asarray(
                [_obstacle_center_and_reach(obstacle)[0] for obstacle in self.obstacles],
                dtype=np.float32,
            ).reshape(-1, 2)
        else:
            centers = np.asarray(route_blocker_centers_xy, dtype=np.float32)
            if centers.ndim != 2 or centers.shape[1:] != (2,) or not np.isfinite(centers).all():
                raise ValueError("route_blocker_centers_xy must be a finite Nx2 array")
            self.route_blocker_centers_xy = centers
        self.scene_metadata = dict(scene_metadata or {})
        self.x_min = -0.5 * config.map_size_x_m
        self.y_min = -0.5 * config.map_size_y_m
        self.nx = int(math.ceil(config.map_size_x_m / config.grid_resolution_m))
        self.ny = int(math.ceil(config.map_size_y_m / config.grid_resolution_m))
        self.raw_occupancy = self._build_occupancy()
        self.clearance_m = self._build_clearance_field()
        self.raw_occupancy |= self.clearance_m <= 0.5 * config.grid_resolution_m
        self.clearance_gradient_x, self.clearance_gradient_y = np.gradient(
            self.clearance_m.astype(np.float32), config.grid_resolution_m, edge_order=1
        )
        safety_clearance = config.robot_radius_m + config.safety_margin_m
        self.safety_occupancy = self.clearance_m < safety_clearance
        self.smoothing_occupancy = self.clearance_m < safety_clearance + 0.15
        required = config.robot_radius_m + config.safety_margin_m + config.planning_extra_margin_m
        self.planning_occupancy = self.clearance_m < required
        self.free_cells = np.argwhere(~self.planning_occupancy).astype(np.int32)
        self.free_cell_tree = cKDTree(self.free_cells)
        self.free_labels, self.free_component_count = label(
            ~self.planning_occupancy,
            structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8),
        )
        self.navigation_graph = self._build_navigation_graph()

    @classmethod
    def random(
        cls,
        config: GroundTruthConfig,
        seed: int,
        *,
        style: str | None = None,
        yopo_tree_ply: Path | None = None,
    ) -> "GroundTruthScene":
        rng = np.random.default_rng(seed)
        obstacles: list[Obstacle] = []
        boundary_margin = 1.5
        selected_style = config.scene_style if style is None else style
        if selected_style == "alternating":
            selected_style = "mixed"
        if selected_style == "yopo_forest":
            # Upstream YOPO-Simple maze_type=5 uses one independently jittered
            # tree per 4 m grid cell and scales its tree asset by 0.5-1.0.
            # At the 1.6 m planning altitude only the asset's 0.08-0.28 m trunk
            # cross-section is relevant, so cylinders preserve the collision
            # and depth geometry without copying a multi-million-point forest.
            tree_spacing_m = 4.0
            rows = int(config.map_size_x_m / tree_spacing_m)
            columns = int(config.map_size_y_m / tree_spacing_m)
            for row in range(rows):
                for column in range(columns):
                    x = row * tree_spacing_m + float(rng.uniform(0.0, tree_spacing_m)) \
                        - 0.5 * config.map_size_x_m
                    y = column * tree_spacing_m + float(rng.uniform(0.0, tree_spacing_m)) \
                        - 0.5 * config.map_size_y_m
                    scale = float(rng.uniform(0.5, 1.0))
                    radius = float(rng.uniform(0.08, 0.28) * scale)
                    height = 14.48 * scale
                    obstacles.append(CylinderObstacle(x, y, radius, height))
            return cls(config, obstacles)
        if selected_style == "yopo_real_forest":
            asset_path = Path(yopo_tree_ply or DEFAULT_YOPO_TREE_ASSET).resolve()
            return cls._yopo_real_forest(config, seed, asset_path)
        obstacle_count = config.obstacle_count
        if selected_style == "blocks":
            obstacle_count = max(8, int(round(0.45 * obstacle_count)))
        elif selected_style == "mixed":
            obstacle_count = max(12, int(round(0.50 * obstacle_count)))
        # Keep a navigable border around every sampled box. Small test maps
        # automatically cap the long tail; the default 80 m map reaches 30 m.
        map_limited_max = min(
            config.block_size_max_m,
            0.375 * min(config.map_size_x_m, config.map_size_y_m),
        )

        def sample_block_side(*, force_large: bool = False) -> float:
            minimum = config.block_size_min_m
            maximum = max(minimum, map_limited_max)
            if force_large and maximum > 15.0:
                return float(rng.uniform(max(15.0, 0.55 * maximum), maximum))
            draw = float(rng.random())
            if draw < 0.60 or maximum <= 6.5:
                upper = min(6.5, maximum)
                return float(rng.uniform(minimum, upper))
            if draw < 0.88 or maximum <= 15.0:
                lower = min(6.5, maximum)
                upper = min(15.0, maximum)
                return float(rng.uniform(lower, max(lower, upper)))
            lower = min(15.0, maximum)
            return float(rng.uniform(lower, maximum))

        block_index = 0
        for _ in range(obstacle_count):
            x = float(rng.uniform(-0.5 * config.map_size_x_m + boundary_margin,
                                  0.5 * config.map_size_x_m - boundary_margin))
            y = float(rng.uniform(-0.5 * config.map_size_y_m + boundary_margin,
                                  0.5 * config.map_size_y_m - boundary_margin))
            height = float(rng.uniform(max(2.5, config.altitude_m + 0.6), config.map_height_m))
            use_tree = selected_style == "forest" or (
                selected_style == "mixed" and rng.random() < config.forest_fraction
            )
            if use_tree:
                radius = float(rng.uniform(config.tree_radius_min_m, config.tree_radius_max_m))
                obstacles.append(CylinderObstacle(x, y, radius, height))
            else:
                # The first block guarantees that every block-bearing default
                # scene contains a 15-30 m structure. Remaining sides follow a
                # 60/28/12 percent small/medium/large long-tail distribution.
                size_x = sample_block_side(force_large=block_index == 0)
                size_y = sample_block_side(force_large=block_index == 0)
                block_index += 1
                obstacles.append(
                    BoxObstacle(
                        x,
                        y,
                        size_x,
                        size_y,
                        height,
                        float(rng.uniform(-math.pi, math.pi)),
                    )
                )
        return cls(config, obstacles)

    @classmethod
    def _yopo_real_forest(
        cls, config: GroundTruthConfig, seed: int, tree_asset_path: Path
    ) -> "GroundTruthScene":
        if not tree_asset_path.is_file():
            raise FileNotFoundError(
                "YOPO-Simple tree asset not found; pass --yopo-tree-ply: "
                f"{tree_asset_path}"
            )
        try:
            import open3d as o3d
        except ImportError as error:
            raise RuntimeError(
                "yopo_real_forest requires open3d to read the original binary tree PLY"
            ) from error

        source_cloud = o3d.io.read_point_cloud(str(tree_asset_path))
        if source_cloud.is_empty():
            raise ValueError(f"YOPO-Simple tree asset is empty: {tree_asset_path}")
        # The upstream generator voxel-filters the assembled forest. Filtering
        # the reusable asset first keeps memory bounded while retaining its
        # trunk, branches, and crown geometry.
        source_cloud = source_cloud.voxel_down_sample(config.point_resolution_m)
        source = np.asarray(source_cloud.points, dtype=np.float32)
        if source.ndim != 2 or source.shape[1:] != (3,) or not np.isfinite(source).all():
            raise ValueError(f"invalid YOPO-Simple tree asset: {tree_asset_path}")

        tree_spacing_m = 4.0
        rows = int(config.map_size_x_m / tree_spacing_m)
        columns = int(config.map_size_y_m / tree_spacing_m)
        position_rng = np.random.default_rng(seed)
        transform_rng = np.random.default_rng(seed)
        positions = [
            (
                row * tree_spacing_m + float(position_rng.uniform(0.0, tree_spacing_m))
                - 0.5 * config.map_size_x_m,
                column * tree_spacing_m + float(position_rng.uniform(0.0, tree_spacing_m))
                - 0.5 * config.map_size_y_m,
            )
            for row in range(rows)
            for column in range(columns)
        ]
        instances: list[np.ndarray] = []
        scales: list[float] = []
        for x, y in positions:
            scale = float(transform_rng.uniform(0.5, 1.0))
            roll = float(transform_rng.uniform(0.0, math.radians(10.0)))
            pitch = float(transform_rng.uniform(0.0, math.radians(10.0)))
            yaw = float(transform_rng.uniform(0.0, 2.0 * math.pi))
            cx, sx = math.cos(roll), math.sin(roll)
            cy, sy = math.cos(pitch), math.sin(pitch)
            cz, sz = math.cos(yaw), math.sin(yaw)
            rotation = np.array(
                [
                    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                    [-sy, cy * sx, cy * cx],
                ],
                dtype=np.float32,
            )
            transformed = (source * scale) @ rotation.T
            transformed[:, 0] += x
            transformed[:, 1] += y
            instances.append(transformed.astype(np.float32, copy=False))
            scales.append(scale)
        points = np.concatenate(instances, axis=0)
        digest = hashlib.sha256(tree_asset_path.read_bytes()).hexdigest()
        return cls(
            config,
            (),
            point_obstacles_world=points,
            route_blocker_centers_xy=np.asarray(positions, dtype=np.float32),
            scene_metadata={
                "tree_asset": str(tree_asset_path),
                "tree_asset_sha256": digest,
                "tree_asset_points": int(len(source)),
                "tree_instances": len(instances),
                "tree_spacing_m": tree_spacing_m,
                "tree_scale_min": float(min(scales)),
                "tree_scale_max": float(max(scales)),
            },
        )

    def _grid_centers(self) -> tuple[np.ndarray, np.ndarray]:
        x = self.x_min + (np.arange(self.nx, dtype=np.float32) + 0.5) * self.config.grid_resolution_m
        y = self.y_min + (np.arange(self.ny, dtype=np.float32) + 0.5) * self.config.grid_resolution_m
        return np.meshgrid(x, y, indexing="ij")

    def _build_occupancy(self) -> np.ndarray:
        x, y = self._grid_centers()
        occupied = np.zeros((self.nx, self.ny), dtype=bool)
        if self.point_obstacles_world is not None:
            points = self.point_obstacles_world
            vertical_band = np.abs(points[:, 2] - self.config.altitude_m) <= max(
                self.config.robot_radius_m, self.config.point_resolution_m
            )
            section = points[vertical_band]
            indices_x = np.floor(
                (section[:, 0] - self.x_min) / self.config.grid_resolution_m
            ).astype(np.int32)
            indices_y = np.floor(
                (section[:, 1] - self.y_min) / self.config.grid_resolution_m
            ).astype(np.int32)
            inside = (
                (indices_x >= 0)
                & (indices_x < self.nx)
                & (indices_y >= 0)
                & (indices_y < self.ny)
            )
            occupied[indices_x[inside], indices_y[inside]] = True
        for obstacle in self.obstacles:
            if isinstance(obstacle, CylinderObstacle):
                occupied |= ((x - obstacle.center_x_m) ** 2 + (y - obstacle.center_y_m) ** 2
                             <= obstacle.radius_m ** 2)
            else:
                cosine = math.cos(obstacle.yaw_rad)
                sine = math.sin(obstacle.yaw_rad)
                dx = x - obstacle.center_x_m
                dy = y - obstacle.center_y_m
                local_x = cosine * dx + sine * dy
                local_y = -sine * dx + cosine * dy
                occupied |= ((np.abs(local_x) <= 0.5 * obstacle.size_x_m)
                             & (np.abs(local_y) <= 0.5 * obstacle.size_y_m))
        occupied[[0, -1], :] = True
        occupied[:, [0, -1]] = True
        return occupied

    def _build_clearance_field(self) -> np.ndarray:
        x, y = self._grid_centers()
        x_max = self.x_min + self.config.map_size_x_m
        y_max = self.y_min + self.config.map_size_y_m
        clearance = np.minimum.reduce(
            (x - self.x_min, x_max - x, y - self.y_min, y_max - y)
        ).astype(np.float32)
        if self.point_obstacles_world is not None:
            query = np.column_stack(
                (
                    x.ravel(),
                    y.ravel(),
                    np.full(x.size, self.config.altitude_m, dtype=np.float32),
                )
            )
            point_clearance, _ = self._point_obstacle_tree.query(query, k=1, workers=-1)
            clearance = np.minimum(
                clearance, np.asarray(point_clearance, dtype=np.float32).reshape(x.shape)
            )
        for obstacle in self.obstacles:
            if isinstance(obstacle, CylinderObstacle):
                obstacle_clearance = np.maximum(
                    0.0,
                    np.hypot(x - obstacle.center_x_m, y - obstacle.center_y_m)
                    - obstacle.radius_m,
                )
            else:
                cosine = math.cos(obstacle.yaw_rad)
                sine = math.sin(obstacle.yaw_rad)
                dx = x - obstacle.center_x_m
                dy = y - obstacle.center_y_m
                local_x = np.abs(cosine * dx + sine * dy) - 0.5 * obstacle.size_x_m
                local_y = np.abs(-sine * dx + cosine * dy) - 0.5 * obstacle.size_y_m
                obstacle_clearance = np.hypot(
                    np.maximum(local_x, 0.0), np.maximum(local_y, 0.0)
                )
                obstacle_clearance[(local_x <= 0.0) & (local_y <= 0.0)] = 0.0
            clearance = np.minimum(clearance, obstacle_clearance.astype(np.float32))
        return clearance.astype(np.float32)

    def _build_navigation_graph(self) -> csr_matrix:
        node_ids = np.arange(self.nx * self.ny, dtype=np.int32).reshape(self.nx, self.ny)
        source_parts: list[np.ndarray] = []
        target_parts: list[np.ndarray] = []
        weight_parts: list[np.ndarray] = []
        free = ~self.planning_occupancy
        for dx, dy, scale in ((1, 0, 1.0), (0, 1, 1.0), (1, 1, math.sqrt(2.0)),
                              (1, -1, math.sqrt(2.0))):
            x0 = slice(0, self.nx - dx)
            x1 = slice(dx, self.nx)
            if dy >= 0:
                y0 = slice(0, self.ny - dy if dy else self.ny)
                y1 = slice(dy, self.ny)
            else:
                y0 = slice(-dy, self.ny)
                y1 = slice(0, self.ny + dy)
            valid = free[x0, y0] & free[x1, y1]
            if dx and dy:
                valid &= free[x1, y0] & free[x0, y1]
            source = node_ids[x0, y0][valid]
            target = node_ids[x1, y1][valid]
            source_parts.extend((source, target))
            target_parts.extend((target, source))
            weight = np.full(len(source), scale * self.config.grid_resolution_m, dtype=np.float32)
            weight_parts.extend((weight, weight))
        source = np.concatenate(source_parts)
        target = np.concatenate(target_parts)
        weights = np.concatenate(weight_parts)
        return csr_matrix((weights, (source, target)), shape=(self.nx * self.ny,) * 2)

    def cell_to_world(self, cell: tuple[int, int]) -> np.ndarray:
        return np.array(
            [
                self.x_min + (cell[0] + 0.5) * self.config.grid_resolution_m,
                self.y_min + (cell[1] + 0.5) * self.config.grid_resolution_m,
                self.config.altitude_m,
            ],
            dtype=np.float32,
        )

    def world_to_cell(self, point: Sequence[float]) -> tuple[int, int]:
        value = np.asarray(point, dtype=np.float32)
        return (
            int(np.clip(math.floor((float(value[0]) - self.x_min) / self.config.grid_resolution_m),
                        0, self.nx - 1)),
            int(np.clip(math.floor((float(value[1]) - self.y_min) / self.config.grid_resolution_m),
                        0, self.ny - 1)),
        )

    def clearance_at_world(self, points_world: np.ndarray) -> np.ndarray:
        points = np.asarray(points_world, dtype=np.float32)
        single = points.ndim == 1
        points = points.reshape(-1, 3)
        coordinates = np.vstack(
            (
                (points[:, 0] - self.x_min) / self.config.grid_resolution_m - 0.5,
                (points[:, 1] - self.y_min) / self.config.grid_resolution_m - 0.5,
            )
        )
        values = map_coordinates(
            self.clearance_m,
            coordinates,
            order=1,
            mode="constant",
            cval=0.0,
        ).astype(np.float32)
        return values[0] if single else values

    def clearance_gradient_at_world(self, points_world: np.ndarray) -> np.ndarray:
        points = np.asarray(points_world, dtype=np.float32)
        single = points.ndim == 1
        points = points.reshape(-1, 3)
        coordinates = np.vstack(
            (
                (points[:, 0] - self.x_min) / self.config.grid_resolution_m - 0.5,
                (points[:, 1] - self.y_min) / self.config.grid_resolution_m - 0.5,
            )
        )
        gx = map_coordinates(
            self.clearance_gradient_x, coordinates, order=1, mode="constant", cval=0.0
        )
        gy = map_coordinates(
            self.clearance_gradient_y, coordinates, order=1, mode="constant", cval=0.0
        )
        result = np.column_stack((gx, gy, np.zeros_like(gx))).astype(np.float32)
        return result[0] if single else result

    def segment_min_clearance(
        self, start: np.ndarray, end: np.ndarray, *, max_step_m: float | None = None
    ) -> float:
        start = np.asarray(start, dtype=np.float32)
        end = np.asarray(end, dtype=np.float32)
        length = float(np.linalg.norm(end - start))
        step = max_step_m or min(0.1, 0.5 * self.config.grid_resolution_m)
        count = max(2, int(math.ceil(length / step)) + 1)
        samples = np.linspace(start, end, count, dtype=np.float32)
        return float(np.min(self.clearance_at_world(samples)))

    def nearest_planning_free_cell(self, point: Sequence[float]) -> tuple[int, int]:
        requested = np.asarray(self.world_to_cell(point), dtype=np.float32)
        _, index = self.free_cell_tree.query(requested, k=1)
        cell = self.free_cells[int(index)]
        return int(cell[0]), int(cell[1])

    def is_segment_free(
        self,
        start: np.ndarray,
        end: np.ndarray,
        *,
        planning_margin: bool = True,
        minimum_safe_radius_m: float | None = None,
        route_anchors_world: tuple[np.ndarray, np.ndarray] | None = None,
    ) -> bool:
        config = self.config
        base_required = config.robot_radius_m + config.safety_margin_m
        base_required += config.planning_extra_margin_m if planning_margin else 0.15
        required = base_required
        if minimum_safe_radius_m is not None:
            required = max(
                required,
                config.robot_radius_m
                + config.safety_margin_m
                + float(minimum_safe_radius_m),
            )
        if route_anchors_world is None or required <= base_required + 1.0e-6:
            return self.segment_min_clearance(start, end) + 1.0e-5 >= required
        start = np.asarray(start, dtype=np.float32)
        end = np.asarray(end, dtype=np.float32)
        length = float(np.linalg.norm(end - start))
        count = max(
            2,
            int(math.ceil(length / min(0.1, 0.5 * config.grid_resolution_m))) + 1,
        )
        samples = np.linspace(start, end, count, dtype=np.float32)
        first_anchor = np.asarray(route_anchors_world[0], dtype=np.float32)
        last_anchor = np.asarray(route_anchors_world[1], dtype=np.float32)
        anchor_distance = np.minimum(
            np.linalg.norm(samples - first_anchor, axis=1),
            np.linalg.norm(samples - last_anchor, axis=1),
        )
        ramp = np.clip(anchor_distance / config.clearance_anchor_relaxation_m, 0.0, 1.0)
        profile = base_required + ramp * (required - base_required)
        return bool(np.all(self.clearance_at_world(samples) + 1.0e-5 >= profile))

    def _astar_with_clearance(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        *,
        minimum_safe_radius_m: float,
        clearance_cost_weight: float = 0.0,
    ) -> tuple[list[tuple[int, int]], float] | None:
        config = self.config
        if self.planning_occupancy[start] or self.planning_occupancy[goal]:
            return None
        base_required = (
            config.robot_radius_m + config.safety_margin_m + config.planning_extra_margin_m
        )
        target_required = max(
            base_required,
            config.robot_radius_m + config.safety_margin_m + minimum_safe_radius_m,
        )
        grid_x, grid_y = np.indices((self.nx, self.ny), dtype=np.float32)
        anchor_distance = np.minimum(
            np.hypot(grid_x - start[0], grid_y - start[1]),
            np.hypot(grid_x - goal[0], grid_y - goal[1]),
        ) * config.grid_resolution_m
        ramp = np.clip(anchor_distance / config.clearance_anchor_relaxation_m, 0.0, 1.0)
        required = base_required + ramp * (target_required - base_required)
        available = self.clearance_m + 1.0e-5 >= required
        available[start] = True
        available[goal] = True

        neighbors = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        queue: list[tuple[float, float, float, tuple[int, int]]] = [(0.0, 0.0, 0.0, start)]
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        cost = {start: 0.0}
        path_length = {start: 0.0}
        closed: set[tuple[int, int]] = set()
        while queue:
            _, current_cost, current_length, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == goal:
                path = [current]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                return list(reversed(path)), current_length
            closed.add(current)
            for dx, dy, step_scale in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < self.nx and 0 <= nxt[1] < self.ny):
                    continue
                if not available[nxt]:
                    continue
                if dx and dy and (
                    not available[current[0] + dx, current[1]]
                    or not available[current[0], current[1] + dy]
                ):
                    continue
                step_length = step_scale * config.grid_resolution_m
                safe_radius = max(
                    0.0,
                    float(self.clearance_m[nxt])
                    - config.robot_radius_m
                    - config.safety_margin_m,
                )
                risk_factor = config.widest_clearance_target_m / max(
                    config.widest_clearance_target_m + safe_radius, 1.0e-6
                )
                edge_cost = step_length * (
                    1.0 + clearance_cost_weight * risk_factor * risk_factor
                )
                candidate_cost = current_cost + edge_cost
                if candidate_cost >= cost.get(nxt, math.inf):
                    continue
                candidate_length = current_length + step_length
                cost[nxt] = candidate_cost
                path_length[nxt] = candidate_length
                parent[nxt] = current
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1]) \
                    * config.grid_resolution_m
                heapq.heappush(
                    queue,
                    (candidate_cost + heuristic, candidate_cost, candidate_length, nxt),
                )
        return None

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
        result = self._astar_with_clearance(
            start,
            goal,
            minimum_safe_radius_m=self.config.planning_extra_margin_m,
        )
        return None if result is None else result[0]

    def widest_shortest_path(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        *,
        shortest_length_m: float | None = None,
        shortest_cells: Sequence[tuple[int, int]] | None = None,
    ) -> RouteSearchResult | None:
        config = self.config
        if shortest_cells is None:
            shortest = self._astar_with_clearance(
                start,
                goal,
                minimum_safe_radius_m=config.planning_extra_margin_m,
            )
            if shortest is None:
                return None
            base_cells, measured_shortest_length = shortest
        else:
            base_cells = list(shortest_cells)
            measured_shortest_length = self._path_cells_length(base_cells)
        if shortest_length_m is None or not math.isfinite(shortest_length_m):
            shortest_length_m = measured_shortest_length
        else:
            shortest_length_m = max(float(shortest_length_m), measured_shortest_length)
        maximum_length = config.widest_detour_ratio * shortest_length_m

        base = config.planning_extra_margin_m
        levels = np.arange(
            base,
            config.widest_clearance_target_m + 0.5 * config.widest_clearance_step_m,
            config.widest_clearance_step_m,
            dtype=np.float32,
        )
        selected_cells = base_cells
        selected_length = measured_shortest_length
        selected_threshold = base
        low = 0
        high = len(levels) - 1
        while low <= high:
            middle = (low + high) // 2
            level = float(levels[middle])
            candidate = self._astar_with_clearance(
                start, goal, minimum_safe_radius_m=level
            )
            if candidate is not None and candidate[1] <= maximum_length + 1.0e-5:
                selected_cells, selected_length = candidate
                selected_threshold = level
                low = middle + 1
            else:
                high = middle - 1

        risk_optimized = self._astar_with_clearance(
            start,
            goal,
            minimum_safe_radius_m=selected_threshold,
            clearance_cost_weight=config.clearance_cost_weight,
        )
        if risk_optimized is not None and risk_optimized[1] <= maximum_length + 1.0e-5:
            selected_cells, selected_length = risk_optimized

        points = np.stack([self.cell_to_world(cell) for cell in selected_cells])
        safe_radius = self.clearance_at_world(points) \
            - config.robot_radius_m - config.safety_margin_m
        cumulative = np.concatenate(
            ([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
        )
        core = (cumulative >= config.clearance_anchor_relaxation_m) \
            & (cumulative <= cumulative[-1] - config.clearance_anchor_relaxation_m)
        measured = safe_radius[core] if np.any(core) else safe_radius
        return RouteSearchResult(
            cells=tuple(selected_cells),
            shortest_length_m=float(shortest_length_m),
            path_length_m=float(selected_length),
            minimum_safe_radius_m=float(np.min(measured)),
            safe_radius_p05_m=float(np.percentile(measured, 5)),
            clearance_threshold_m=selected_threshold,
        )

    def _path_cells_length(self, cells: Sequence[tuple[int, int]]) -> float:
        if len(cells) < 2:
            return 0.0
        values = np.asarray(cells, dtype=np.float32)
        return float(
            np.linalg.norm(np.diff(values, axis=0), axis=1).sum()
            * self.config.grid_resolution_m
        )

    def shortest_path_tree(
        self, start: tuple[int, int]
    ) -> tuple[np.ndarray, np.ndarray, int]:
        start_id = int(np.ravel_multi_index(start, (self.nx, self.ny)))
        distances, predecessors = dijkstra(
            self.navigation_graph,
            directed=True,
            indices=start_id,
            return_predecessors=True,
            limit=1.5 * self.config.route_max_length_m,
        )
        return (
            np.asarray(distances, dtype=np.float32).reshape(self.nx, self.ny),
            np.asarray(predecessors, dtype=np.int32),
            start_id,
        )

    def path_from_tree(
        self,
        goal: tuple[int, int],
        predecessors: np.ndarray,
        start_id: int,
    ) -> list[tuple[int, int]] | None:
        current = int(np.ravel_multi_index(goal, (self.nx, self.ny)))
        path_ids = [current]
        while current != start_id:
            current = int(predecessors[current])
            if current < 0:
                return None
            path_ids.append(current)
        return [tuple(int(value) for value in np.unravel_index(node, (self.nx, self.ny)))
                for node in reversed(path_ids)]

    def smooth_grid_path(
        self,
        cells: Sequence[tuple[int, int]],
        *,
        minimum_safe_radius_m: float | None = None,
    ) -> np.ndarray | None:
        raw = np.stack([self.cell_to_world(cell) for cell in cells])
        route_anchors = (raw[0], raw[-1])
        simplified = [raw[0]]
        index = 0
        while index < len(raw) - 1:
            candidate = len(raw) - 1
            while candidate > index + 1 and not self.is_segment_free(
                raw[index],
                raw[candidate],
                minimum_safe_radius_m=minimum_safe_radius_m,
                route_anchors_world=route_anchors,
            ):
                candidate -= 1
            simplified.append(raw[candidate])
            index = candidate
        path = np.asarray(simplified, dtype=np.float32)
        for _ in range(5):
            if len(path) <= 2:
                break
            refined = [path[0]]
            for left, right in zip(path[:-1], path[1:]):
                refined.extend((0.75 * left + 0.25 * right, 0.25 * left + 0.75 * right))
            refined.append(path[-1])
            candidate_path = np.asarray(refined, dtype=np.float32)
            if not all(
                self.is_segment_free(
                    left,
                    right,
                    planning_margin=True,
                    minimum_safe_radius_m=minimum_safe_radius_m,
                    route_anchors_world=route_anchors,
                )
                for left, right in zip(candidate_path[:-1], candidate_path[1:])
            ):
                break
            path = candidate_path
        for left, right in zip(path[:-1], path[1:]):
            if not self.is_segment_free(
                left,
                right,
                planning_margin=True,
                minimum_safe_radius_m=minimum_safe_radius_m,
                route_anchors_world=route_anchors,
            ):
                return None
        if len(path) >= 3:
            segment_length = np.linalg.norm(np.diff(path, axis=0), axis=1)
            cumulative = np.concatenate(([0.0], np.cumsum(segment_length)))
            if cumulative[-1] > 1.0e-5:
                sample_count = max(3, int(math.ceil(float(cumulative[-1]) / 0.1)) + 1)
                sample_distance = np.linspace(0.0, float(cumulative[-1]), sample_count)
                spline = CubicSpline(cumulative, path, axis=0, bc_type="natural")
                candidate = spline(sample_distance).astype(np.float32)
                candidate[:, 2] = self.config.altitude_m
                if all(
                    self.is_segment_free(
                        left,
                        right,
                        planning_margin=True,
                        minimum_safe_radius_m=minimum_safe_radius_m,
                        route_anchors_world=route_anchors,
                    )
                    for left, right in zip(candidate[:-1], candidate[1:])
                ):
                    path = candidate
        return path

    def _centerline_metrics(
        self, points_world: np.ndarray
    ) -> tuple[float, float, float]:
        config = self.config
        points = np.asarray(points_world, dtype=np.float32)
        clearance = self.clearance_at_world(points)
        safe_radius = clearance - config.robot_radius_m - config.safety_margin_m
        cumulative, length = self._polyline_arclength(points)
        core = (cumulative >= config.clearance_anchor_relaxation_m) & (
            cumulative <= length - config.clearance_anchor_relaxation_m
        )
        measured = safe_radius[core] if np.any(core) else safe_radius
        segment_length = np.linalg.norm(np.diff(points, axis=0), axis=1)
        segment_radius = np.minimum(safe_radius[:-1], safe_radius[1:])
        ratio = config.widest_clearance_target_m / np.maximum(
            config.widest_clearance_target_m + segment_radius, 1.0e-6
        )
        risk = float(np.sum(segment_length * ratio * ratio))
        return float(np.min(measured)), float(np.percentile(measured, 5)), risk

    @staticmethod
    def _polyline_arclength(points_world: np.ndarray) -> tuple[np.ndarray, float]:
        segment_length = np.linalg.norm(np.diff(points_world, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment_length)))
        return cumulative, float(cumulative[-1])

    def refine_witness_centerline(
        self,
        path_points_world: np.ndarray,
        *,
        minimum_safe_radius_m: float | None = None,
    ) -> CenterlineRefinementResult:
        config = self.config
        points, _ = resample_polyline(
            path_points_world, max_step_m=config.centerline_resample_step_m
        )
        points[:, 2] = config.altitude_m
        original = points.copy()
        before_minimum, before_p05, before_risk = self._centerline_metrics(points)
        current_minimum, current_p05, current_risk = before_minimum, before_p05, before_risk
        initial_curvature = maximum_polyline_curvature(points)
        iterations = 0
        # Keep every update physically executable.  The search threshold is a
        # bottleneck target, rather than a hard constraint here: if the input
        # path already contains a narrower real passage, gradient refinement
        # must be allowed to push it outward and improve it incrementally.
        target_safe_radius = (
            config.planning_extra_margin_m
            if minimum_safe_radius_m is None
            else max(config.planning_extra_margin_m, float(minimum_safe_radius_m))
        )
        required = config.robot_radius_m + config.safety_margin_m + config.planning_extra_margin_m
        for iteration in range(config.centerline_iterations):
            gradient = self.clearance_gradient_at_world(points)
            norm = np.linalg.norm(gradient[:, :2], axis=1, keepdims=True)
            direction = gradient[:, :2] / np.maximum(norm, 1.0e-6)
            for _ in range(3):
                direction[1:-1] = (
                    0.25 * direction[:-2]
                    + 0.5 * direction[1:-1]
                    + 0.25 * direction[2:]
                )
            candidate = points.copy()
            candidate[1:-1, :2] += config.centerline_step_m * direction[1:-1]
            displacement = candidate[:, :2] - original[:, :2]
            distance = np.linalg.norm(displacement, axis=1, keepdims=True)
            candidate[:, :2] = original[:, :2] + displacement * np.minimum(
                1.0, config.centerline_max_deviation_m / np.maximum(distance, 1.0e-6)
            )
            candidate[0] = original[0]
            candidate[-1] = original[-1]
            candidate[:, 2] = config.altitude_m
            maximum_allowed_curvature = max(1.6, 1.05 * initial_curvature)
            if maximum_polyline_curvature(candidate) > maximum_allowed_curvature:
                break
            if any(
                self.segment_min_clearance(left, right) + 1.0e-5 < required
                for left, right in zip(candidate[:-1], candidate[1:])
            ):
                break
            candidate_minimum, candidate_p05, candidate_risk = self._centerline_metrics(candidate)
            if current_minimum + 1.0e-4 < target_safe_radius:
                # While below the selected corridor width, prioritize the
                # actual minimum so an isolated bubble waist is widened.
                improves_bottleneck = candidate_minimum > current_minimum + 1.0e-4 \
                    and candidate_p05 + 1.0e-4 >= current_p05
            else:
                improves_bottleneck = candidate_p05 > current_p05 + 1.0e-4 \
                    and candidate_minimum + 1.0e-4 >= current_minimum
            improves_risk = candidate_risk < current_risk - 1.0e-4 \
                and candidate_minimum + 1.0e-4 >= current_minimum \
                and candidate_p05 + 1.0e-4 >= current_p05
            if not (improves_bottleneck or improves_risk):
                break
            points = candidate
            current_minimum = candidate_minimum
            current_p05 = candidate_p05
            current_risk = candidate_risk
            iterations = iteration + 1
        return CenterlineRefinementResult(
            points_world=points,
            minimum_safe_radius_before_m=before_minimum,
            minimum_safe_radius_after_m=current_minimum,
            safe_radius_p05_before_m=before_p05,
            safe_radius_p05_after_m=current_p05,
            clearance_risk_before=before_risk,
            clearance_risk_after=current_risk,
            iterations=iterations,
        )

    def obstacle_point_cloud(self) -> np.ndarray:
        step = self.config.point_resolution_m
        points: list[np.ndarray] = []
        if self.point_obstacles_world is not None:
            points.append(self.point_obstacles_world)
        for obstacle in self.obstacles:
            z = np.arange(0.0, obstacle.height_m + 0.5 * step, step, dtype=np.float32)
            if isinstance(obstacle, CylinderObstacle):
                angle_count = max(12, int(math.ceil(2.0 * math.pi * obstacle.radius_m / step)))
                angles = np.linspace(0.0, 2.0 * math.pi, angle_count, endpoint=False, dtype=np.float32)
                angle_grid, z_grid = np.meshgrid(angles, z, indexing="ij")
                points.append(np.stack((obstacle.center_x_m + obstacle.radius_m * np.cos(angle_grid),
                                        obstacle.center_y_m + obstacle.radius_m * np.sin(angle_grid),
                                        z_grid), axis=-1).reshape(-1, 3))
            else:
                perimeter: list[tuple[float, float]] = []
                for value in np.arange(-0.5 * obstacle.size_x_m, 0.5 * obstacle.size_x_m + step, step):
                    perimeter.extend(((float(value), -0.5 * obstacle.size_y_m),
                                      (float(value), 0.5 * obstacle.size_y_m)))
                for value in np.arange(-0.5 * obstacle.size_y_m, 0.5 * obstacle.size_y_m + step, step):
                    perimeter.extend(((-0.5 * obstacle.size_x_m, float(value)),
                                      (0.5 * obstacle.size_x_m, float(value))))
                xy = np.asarray(perimeter, dtype=np.float32)
                cosine, sine = math.cos(obstacle.yaw_rad), math.sin(obstacle.yaw_rad)
                rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=np.float32)
                xy = xy @ rotation.T + np.array([obstacle.center_x_m, obstacle.center_y_m])
                xy_grid, z_grid = np.meshgrid(np.arange(len(xy)), z, indexing="ij")
                points.append(np.column_stack((xy[xy_grid.ravel()], z_grid.ravel())))
        ground_x = np.arange(self.x_min, -self.x_min + 0.5 * step, max(step, 0.3), dtype=np.float32)
        ground_y = np.arange(self.y_min, -self.y_min + 0.5 * step, max(step, 0.3), dtype=np.float32)
        gx, gy = np.meshgrid(ground_x, ground_y, indexing="ij")
        points.append(np.stack((gx.ravel(), gy.ravel(), np.zeros(gx.size, dtype=np.float32)), axis=1))
        return np.concatenate(points, axis=0).astype(np.float32)

    def _render_point_obstacle_depth(
        self, origin: np.ndarray, rotation_world_body: np.ndarray
    ) -> np.ndarray:
        config = self.config
        assert self.point_obstacles_world is not None
        assert self._point_obstacle_tree is not None
        horizontal = math.tan(math.radians(config.horizontal_fov_deg) * 0.5)
        vertical = math.tan(math.radians(config.vertical_fov_deg) * 0.5)
        query_radius = config.max_depth_m * math.sqrt(1.0 + horizontal**2 + vertical**2)
        nearby_indices = self._point_obstacle_tree.query_ball_point(origin, query_radius)
        depth = np.full((config.image_height, config.image_width), config.max_depth_m, dtype=np.float32)
        if not nearby_indices:
            return depth
        relative_world = self.point_obstacles_world[np.asarray(nearby_indices, dtype=np.int64)] - origin
        relative_body = relative_world @ rotation_world_body
        forward = relative_body[:, 0]
        visible = (forward > 0.05) & (forward <= config.max_depth_m)
        if not np.any(visible):
            return depth
        relative_body = relative_body[visible]
        forward = forward[visible]
        fx = 0.5 * (config.image_width - 1) / horizontal
        fy = 0.5 * (config.image_height - 1) / vertical
        columns = np.rint(0.5 * (config.image_width - 1) - fx * relative_body[:, 1] / forward)
        rows = np.rint(0.5 * (config.image_height - 1) - fy * relative_body[:, 2] / forward)
        columns = columns.astype(np.int32)
        rows = rows.astype(np.int32)
        inside = (
            (columns >= 0)
            & (columns < config.image_width)
            & (rows >= 0)
            & (rows < config.image_height)
        )
        flat = depth.reshape(-1)
        pixels = rows[inside] * config.image_width + columns[inside]
        np.minimum.at(flat, pixels, forward[inside])
        return depth

    def render_depth(self, position_world: np.ndarray, yaw_rad: float) -> np.ndarray:
        config = self.config
        columns = np.arange(config.image_width, dtype=np.float32)
        rows = np.arange(config.image_height, dtype=np.float32)
        row_grid, column_grid = np.meshgrid(rows, columns, indexing="ij")
        fx = 0.5 * (config.image_width - 1) / math.tan(math.radians(config.horizontal_fov_deg) / 2.0)
        fy = 0.5 * (config.image_height - 1) / math.tan(math.radians(config.vertical_fov_deg) / 2.0)
        directions_body = np.stack(
            (
                np.ones_like(column_grid),
                -(column_grid - 0.5 * (config.image_width - 1)) / fx,
                -(row_grid - 0.5 * (config.image_height - 1)) / fy,
            ),
            axis=-1,
        ).reshape(-1, 3)
        cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
        rotation = np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
                            dtype=np.float32)
        directions = directions_body @ rotation.T
        origin = np.asarray(position_world, dtype=np.float32)
        depth = np.full(len(directions), config.max_depth_m, dtype=np.float32)
        down = directions[:, 2] < -1.0e-6
        ground_t = np.where(down, -origin[2] / np.minimum(directions[:, 2], -1.0e-6), math.inf)
        depth = np.minimum(depth, np.where(ground_t > 0.0, ground_t, math.inf))
        if self.point_obstacles_world is not None:
            point_depth = self._render_point_obstacle_depth(origin, rotation)
            depth = np.minimum(depth, point_depth.reshape(-1))
        for obstacle in self.obstacles:
            if isinstance(obstacle, CylinderObstacle):
                ox = origin[0] - obstacle.center_x_m
                oy = origin[1] - obstacle.center_y_m
                a = directions[:, 0] ** 2 + directions[:, 1] ** 2
                b = 2.0 * (ox * directions[:, 0] + oy * directions[:, 1])
                c = ox * ox + oy * oy - obstacle.radius_m ** 2
                discriminant = b * b - 4.0 * a * c
                valid = discriminant >= 0.0
                root = np.sqrt(np.maximum(discriminant, 0.0))
                for candidate in ((-b - root) / np.maximum(2.0 * a, 1.0e-8),
                                  (-b + root) / np.maximum(2.0 * a, 1.0e-8)):
                    z = origin[2] + candidate * directions[:, 2]
                    hit = valid & (candidate > 0.0) & (z >= 0.0) & (z <= obstacle.height_m)
                    depth = np.minimum(depth, np.where(hit, candidate, math.inf))
            else:
                cosine, sine = math.cos(obstacle.yaw_rad), math.sin(obstacle.yaw_rad)
                offset = origin[:2] - np.array([obstacle.center_x_m, obstacle.center_y_m])
                local_origin = np.array([cosine * offset[0] + sine * offset[1],
                                         -sine * offset[0] + cosine * offset[1], origin[2]])
                local_direction = np.column_stack(
                    (cosine * directions[:, 0] + sine * directions[:, 1],
                     -sine * directions[:, 0] + cosine * directions[:, 1], directions[:, 2])
                )
                lower = np.array([-0.5 * obstacle.size_x_m, -0.5 * obstacle.size_y_m, 0.0])
                upper = np.array([0.5 * obstacle.size_x_m, 0.5 * obstacle.size_y_m,
                                  obstacle.height_m])
                safe_direction = np.where(np.abs(local_direction) < 1.0e-8, 1.0e-8, local_direction)
                first = (lower - local_origin) / safe_direction
                second = (upper - local_origin) / safe_direction
                near = np.max(np.minimum(first, second), axis=1)
                far = np.min(np.maximum(first, second), axis=1)
                hit = (far >= np.maximum(near, 0.0)) & (near > 0.0)
                depth = np.minimum(depth, np.where(hit, near, math.inf))
        return np.clip(depth.reshape(config.image_height, config.image_width), 0.05,
                       config.max_depth_m).astype(np.float32)


def _yaw_to_ned_pose(position_enu: np.ndarray, yaw_enu: float) -> PoseSample:
    position_ned = enu_to_ned(position_enu)
    yaw_ned = 0.5 * math.pi - yaw_enu
    return PoseSample(
        tuple(float(value) for value in position_ned),
        (math.cos(0.5 * yaw_ned), 0.0, 0.0, math.sin(0.5 * yaw_ned)),
    )


def _rotation_from_yaw(yaw: float) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
                    dtype=np.float32)


def _path_length(points: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _obstacle_center_and_reach(obstacle: Obstacle) -> tuple[np.ndarray, float]:
    center = np.array([obstacle.center_x_m, obstacle.center_y_m], dtype=np.float32)
    if isinstance(obstacle, CylinderObstacle):
        return center, obstacle.radius_m
    return center, 0.5 * math.hypot(obstacle.size_x_m, obstacle.size_y_m)


def _angle_difference(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


def _draw_preview(scene: GroundTruthScene, route: np.ndarray, start: np.ndarray, output: Path) -> None:
    image = np.where(scene.raw_occupancy.T[::-1], 35, 245).astype(np.uint8)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    pixels = np.stack(
        ((route[:, 0] - scene.x_min) / scene.config.grid_resolution_m,
         scene.ny - 1 - (route[:, 1] - scene.y_min) / scene.config.grid_resolution_m), axis=1
    ).round().astype(np.int32)
    cv2.polylines(image, [pixels], False, (40, 60, 230), 2, cv2.LINE_AA)
    start_pixel = tuple(pixels[0])
    end_pixel = tuple(pixels[-1])
    cv2.circle(image, start_pixel, 3, (40, 180, 40), -1)
    cv2.circle(image, end_pixel, 3, (230, 120, 30), -1)
    scale = max(1, int(round(800 / max(image.shape))))
    image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise IOError(f"failed to write route preview: {output}")


def _candidate_route(
    scene: GroundTruthScene,
    start_cell: tuple[int, int],
    yaw: float,
    rng: np.random.Generator,
    obstacle_points: np.ndarray,
    quality_gate: RouteQualityGate,
    obstacle_tree: cKDTree,
    rejection_stats: dict[str, int],
    path_tree: tuple[np.ndarray, np.ndarray, int],
    *,
    require_detour: bool = False,
) -> CandidateRoute | None:
    config = scene.config
    start = scene.cell_to_world(start_cell)
    component = int(scene.free_labels[start_cell])
    component_cells = scene.free_cells[
        scene.free_labels[scene.free_cells[:, 0], scene.free_cells[:, 1]] == component
    ]
    goal_xy = np.column_stack(
        (
            scene.x_min + (component_cells[:, 0] + 0.5) * config.grid_resolution_m,
            scene.y_min + (component_cells[:, 1] + 0.5) * config.grid_resolution_m,
        )
    )
    delta = goal_xy - start[:2]
    distance = np.linalg.norm(delta, axis=1)
    heading = np.arctan2(delta[:, 1], delta[:, 0])
    heading_error = np.abs(np.arctan2(np.sin(heading - yaw), np.cos(heading - yaw)))
    shortest_distances, predecessors, start_id = path_tree
    path_distance = shortest_distances[component_cells[:, 0], component_cells[:, 1]]
    eligible = (
        (distance >= config.route_min_length_m)
        & (distance <= min(config.route_max_length_m, 18.0))
        & (heading_error <= math.radians(42.0))
        & np.isfinite(path_distance)
        & (path_distance >= config.route_min_length_m)
        & (path_distance <= 1.2 * config.route_max_length_m)
    )
    candidates = component_cells[eligible]
    if len(candidates) == 0:
        rejection_stats["no_reachable_goal"] = rejection_stats.get("no_reachable_goal", 0) + 1
        return None
    candidates = candidates[rng.permutation(len(candidates))]
    for goal_values in candidates[: config.maximum_route_attempts]:
        goal_cell = int(goal_values[0]), int(goal_values[1])
        goal_world = scene.cell_to_world(goal_cell)
        if require_detour and scene.is_segment_free(start, goal_world, planning_margin=False):
            rejection_stats["not_a_detour"] = rejection_stats.get("not_a_detour", 0) + 1
            continue
        shortest_cells = scene.path_from_tree(goal_cell, predecessors, start_id)
        if shortest_cells is None:
            rejection_stats["astar_failed"] = rejection_stats.get("astar_failed", 0) + 1
            continue
        search = scene.widest_shortest_path(
            start_cell,
            goal_cell,
            shortest_length_m=float(shortest_distances[goal_cell]),
            shortest_cells=shortest_cells,
        )
        if search is None:
            rejection_stats["astar_failed"] = rejection_stats.get("astar_failed", 0) + 1
            continue
        smoothed = scene.smooth_grid_path(
            search.cells, minimum_safe_radius_m=search.clearance_threshold_m
        )
        if smoothed is None:
            rejection_stats["smoothing_failed"] = rejection_stats.get("smoothing_failed", 0) + 1
            continue
        refinement = scene.refine_witness_centerline(
            smoothed, minimum_safe_radius_m=search.clearance_threshold_m
        )
        refined = refinement.points_world
        length = _path_length(refined)
        if not config.route_min_length_m <= length <= 1.5 * config.route_max_length_m:
            rejection_stats["route_length"] = rejection_stats.get("route_length", 0) + 1
            continue
        points, clearance, radius = build_witness_corridor(
            refined,
            obstacle_points,
            robot_radius_m=config.robot_radius_m,
            safety_margin_m=config.safety_margin_m,
            max_step_m=0.1,
            obstacle_tree=obstacle_tree,
        )
        result = quality_gate.evaluate(
            path_points_world=points,
            path_clearance_m=clearance,
            path_bubble_radius_m=radius,
            start_world=start,
            frontier_world=points[-1],
            start_rotation_world_body=_rotation_from_yaw(yaw),
        )
        if result.valid:
            return CandidateRoute(points, clearance, radius, search, refinement, result)
        for flag in type(result.flags):
            if flag and result.flags & flag:
                key = f"quality_{flag.name.lower()}"
                rejection_stats[key] = rejection_stats.get(key, 0) + 1
    rejection_stats["attempt_limit"] = rejection_stats.get("attempt_limit", 0) + 1
    return None


def generate_ground_truth_dataset(
    output: Path,
    *,
    scene_count: int = 2,
    frames_per_scene: int = 500,
    seed: int = 0,
    config: GroundTruthConfig = GroundTruthConfig(),
    preview_routes: int = 100,
    overwrite: bool = False,
    dataset_role: str = "train",
    scene_styles: Sequence[str] | None = None,
    yopo_tree_ply: Path | None = None,
) -> Path:
    output = Path(output)
    if scene_count <= 0 or frames_per_scene <= 0:
        raise ValueError("scene_count and frames_per_scene must be positive")
    if dataset_role not in {"train", "offline_test"}:
        raise ValueError("dataset_role must be train or offline_test")
    if scene_styles is not None:
        if len(scene_styles) != scene_count:
            raise ValueError("scene_styles must contain one style per scene")
        invalid_styles = set(scene_styles) - SCENE_STYLES
        if invalid_styles:
            raise ValueError(f"invalid scene styles: {sorted(invalid_styles)}")
    if output.exists() and any(output.iterdir()):
        if not overwrite:
            raise FileExistsError(f"output directory is not empty: {output}")
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)
    report: dict[str, object] = {
        "generator": "ScaleNav ground-truth YOPO-style dataset",
        "dataset_role": dataset_role,
        "seed": seed,
        "config": asdict(config),
        "scenes": [],
    }
    preview_index = 0
    route_lengths: list[float] = []
    route_clearances: list[float] = []
    route_safe_radius_p05: list[float] = []
    route_search_detour_ratios: list[float] = []
    centerline_gains: list[float] = []
    route_neck_lengths: list[float] = []
    route_continuous_clearances: list[float] = []
    route_overlap_margins: list[float] = []
    detour_route_count = 0
    rejection_stats: dict[str, int] = {}
    quality_gate = RouteQualityGate(
        RouteQualityConfig(
            robot_radius_m=config.robot_radius_m,
            safety_margin_m=config.safety_margin_m,
            minimum_execution_length_m=2.0,
            maximum_heading_deg=51.0,
            maximum_curvature_rad_m=1.6,
        )
    )
    capture = CaptureConfig(
        horizontal_fov_deg=config.horizontal_fov_deg,
        vertical_fov_deg=config.vertical_fov_deg,
        max_depth_m=config.max_depth_m,
        settle_time_s=0.0,
    )
    for scene_index in range(scene_count):
        scene_seed = seed + scene_index * 100003
        rng = np.random.default_rng(scene_seed)
        style = scene_styles[scene_index] if scene_styles is not None else config.scene_style
        if scene_styles is None and style == "alternating":
            style = "blocks" if scene_index % 2 == 0 else "mixed"
        truth = GroundTruthScene.random(
            config, scene_seed, style=style, yopo_tree_ply=yopo_tree_ply
        )
        obstacle_points = truth.obstacle_point_cloud()
        obstacle_tree = cKDTree(obstacle_points)
        source_ply = output / f".scene_{scene_index:04d}_ned.ply"
        write_point_cloud_ply(source_ply, enu_to_ned(obstacle_points))
        scene_dir = output / f"Scene_{scene_index:04d}"
        writer = SceneWriter(scene_dir, capture, sampling_seed=scene_seed)
        records: list[RouteRecord] = []
        frame_attempts = 0
        route_failures = 0
        valid_cells = np.argwhere(
            (~truth.planning_occupancy) & (truth.clearance_m >= config.safe_pose_clearance_m)
        )
        if len(valid_cells) == 0:
            raise RuntimeError(f"scene {scene_index} has no safe pose cells")
        for frame_index in range(frames_per_scene):
            accepted_routes: list[CandidateRoute] | None = None
            while accepted_routes is None:
                frame_attempts += 1
                if frame_attempts > config.maximum_frame_attempts * frames_per_scene:
                    raise RuntimeError(
                        f"scene {scene_index}: could not produce {frames_per_scene} route-conditioned frames"
                    )
                cell_values = valid_cells[int(rng.integers(0, len(valid_cells)))]
                start_cell = (int(cell_values[0]), int(cell_values[1]))
                start = truth.cell_to_world(start_cell)
                blocker_headings = []
                for center in truth.route_blocker_centers_xy:
                    delta = center - start[:2]
                    distance = float(np.linalg.norm(delta))
                    if 3.0 < distance < config.route_max_length_m - 3.0:
                        blocker_headings.append(math.atan2(float(delta[1]), float(delta[0])))
                if not blocker_headings:
                    continue
                yaw = blocker_headings[int(rng.integers(0, len(blocker_headings)))]
                yaw += float(rng.uniform(-math.radians(18.0), math.radians(18.0)))
                path_tree = truth.shortest_path_tree(start_cell)
                candidates: list[CandidateRoute] = []
                for variant in range(config.routes_per_frame):
                    route = _candidate_route(
                        truth,
                        start_cell,
                        yaw,
                        rng,
                        obstacle_points,
                        quality_gate,
                        obstacle_tree,
                        rejection_stats,
                        path_tree,
                        require_detour=variant == 0,
                    )
                    if route is None:
                        route_failures += 1
                        break
                    candidates.append(route)
                if len(candidates) == config.routes_per_frame:
                    accepted_routes = candidates
            depth = truth.render_depth(start, yaw)
            normalized = np.clip(depth / config.max_depth_m, 0.0, 1.0)
            rgb = cv2.applyColorMap((255.0 * (1.0 - normalized)).astype(np.uint8),
                                    cv2.COLORMAP_VIRIDIS)
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
            writer.write_frame(
                frame_index,
                rgb,
                depth,
                _yaw_to_ned_pose(start, yaw),
                scene_seed * 1_000_000 + frame_index,
                "ground_truth_frontier",
            )
            for variant, candidate in enumerate(accepted_routes):
                points = candidate.points_world
                clearance = candidate.clearance_m
                radius = candidate.safe_radius_m
                topo_indices = np.linspace(0, len(points) - 1, min(8, len(points)), dtype=np.int64)
                frontier = points[-1].copy()
                _, local_subgoal_distance = local_subgoal_on_witness(
                    points, config.local_subgoal_distance_m
                )
                route_seed = scene_seed * 1_000_000 + frame_index * config.routes_per_frame + variant
                records.append(
                    RouteRecord(
                        frame_index=frame_index,
                        mission_goal_world=frontier.copy(),
                        frontier_goal_world=frontier,
                        path_points_world=points,
                        path_clearance_m=clearance,
                        path_bubble_radius_m=radius,
                        topo_centers_world=points[topo_indices],
                        topo_bubble_radius_m=radius[topo_indices],
                        topo_persistent_id=np.arange(len(topo_indices), dtype=np.uint64)
                        + np.uint64(route_seed * 16),
                        route_valid=True,
                        route_quality_flags=0,
                        route_quality_weight=candidate.quality.weight,
                        route_seed=route_seed,
                        route_search_detour_ratio=candidate.search.detour_ratio,
                        route_centerline_gain_m=candidate.refinement.gain_m,
                        local_subgoal_distance_m=local_subgoal_distance,
                    )
                )
                route_lengths.append(_path_length(points))
                route_clearances.append(float(np.min(clearance)))
                route_safe_radius_p05.append(float(np.percentile(radius, 5)))
                route_search_detour_ratios.append(candidate.search.detour_ratio)
                centerline_gains.append(candidate.refinement.gain_m)
                route_neck_lengths.append(candidate.quality.neck_length_m)
                route_continuous_clearances.append(
                    candidate.quality.continuous_minimum_clearance_m
                )
                route_overlap_margins.append(candidate.quality.bubble_overlap_margin_m)
                if not truth.is_segment_free(points[0], points[-1], planning_margin=False):
                    detour_route_count += 1
                if preview_index < preview_routes:
                    _draw_preview(
                        truth,
                        points,
                        start,
                        output / "route_previews" / f"route_{preview_index:05d}.png",
                    )
                    preview_index += 1
            completed = frame_index + 1
            if completed % 50 == 0 or completed == frames_per_scene:
                print(
                    f"{scene_dir.name} [{style}]: {completed}/{frames_per_scene} frames, "
                    f"{len(records)} routes",
                    flush=True,
                )
        writer.finalize(source_ply)
        source_ply.unlink(missing_ok=True)
        save_route_table(scene_dir / "routes.npz", pack_route_records(records))
        grid_shape = [truth.nx, truth.ny,
                      int(math.ceil((config.map_height_m + 6.2) / config.grid_resolution_m))]
        scene_report = {
            "scene": scene_dir.name,
            "seed": scene_seed,
            "style": style,
            "frames": frames_per_scene,
            "routes": len(records),
            "frame_attempts": frame_attempts,
            "route_search_failures": route_failures,
            "obstacle_points": len(obstacle_points),
            "estimated_esdf_shape": grid_shape,
            "estimated_esdf_float32_mib": float(np.prod(grid_shape) * 4 / 2**20),
            **truth.scene_metadata,
        }
        block_sizes = np.asarray(
            [[obstacle.size_x_m, obstacle.size_y_m]
             for obstacle in truth.obstacles if isinstance(obstacle, BoxObstacle)],
            dtype=np.float32,
        )
        if len(block_sizes):
            scene_report["block_size_m"] = {
                "count": int(len(block_sizes)),
                "side_min": float(np.min(block_sizes)),
                "side_mean": float(np.mean(block_sizes)),
                "side_max": float(np.max(block_sizes)),
                "large_side_count": int(np.count_nonzero(block_sizes >= 15.0)),
            }
        report["scenes"].append(scene_report)  # type: ignore[union-attr]
    report["route_count"] = len(route_lengths)
    report["detour_route_count"] = detour_route_count
    report["detour_route_ratio"] = detour_route_count / len(route_lengths)
    report["route_rejection_counts"] = rejection_stats
    report["preview_count"] = preview_index
    report["route_length_m"] = {
        "min": min(route_lengths), "mean": float(np.mean(route_lengths)), "max": max(route_lengths)
    }
    report["minimum_clearance_m"] = {
        "min": min(route_clearances), "mean": float(np.mean(route_clearances))
    }
    report["safe_radius_p05_m"] = {
        "min": min(route_safe_radius_p05),
        "mean": float(np.mean(route_safe_radius_p05)),
    }
    report["search_detour_ratio"] = {
        "min": min(route_search_detour_ratios),
        "mean": float(np.mean(route_search_detour_ratios)),
        "max": max(route_search_detour_ratios),
    }
    report["centerline_refinement_gain_m"] = {
        "min": min(centerline_gains),
        "mean": float(np.mean(centerline_gains)),
        "max": max(centerline_gains),
        "improved_route_count": int(np.count_nonzero(np.asarray(centerline_gains) > 1.0e-4)),
    }
    report["neck_length_below_target_m"] = {
        "min": min(route_neck_lengths),
        "mean": float(np.mean(route_neck_lengths)),
        "max": max(route_neck_lengths),
    }
    report["continuous_minimum_clearance_m"] = {
        "min": min(route_continuous_clearances),
        "mean": float(np.mean(route_continuous_clearances)),
    }
    report["bubble_overlap_margin_m"] = {
        "min": min(route_overlap_margins),
        "mean": float(np.mean(route_overlap_margins)),
    }
    (output / "generation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate YOPO-style scenes with ground-truth A* witness corridors"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenes", type=int, default=2)
    parser.add_argument("--frames", type=int, default=500)
    parser.add_argument("--routes-per-frame", type=int, default=3)
    parser.add_argument("--obstacles", type=int, default=40)
    parser.add_argument(
        "--widest-detour-ratio",
        type=float,
        default=1.12,
        help="maximum route length divided by the shortest route length",
    )
    parser.add_argument(
        "--widest-clearance-target",
        type=float,
        default=1.2,
        help="target safe radius for widest-shortest search, in meters",
    )
    parser.add_argument(
        "--scene-style",
        choices=tuple(sorted(SCENE_STYLES)),
        default="alternating",
    )
    parser.add_argument(
        "--scene-styles",
        type=str,
        help="comma-separated style per scene, e.g. yopo_forest,yopo_real_forest,blocks",
    )
    parser.add_argument(
        "--yopo-tree-ply",
        type=Path,
        default=DEFAULT_YOPO_TREE_ASSET,
        help="original YOPO-Simple tree.ply used by yopo_real_forest",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--preview-routes", type=int, default=100)
    parser.add_argument("--dataset-role", choices=("train", "offline_test"), default="train")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    config = GroundTruthConfig(
        routes_per_frame=args.routes_per_frame,
        obstacle_count=args.obstacles,
        scene_style=args.scene_style,
        widest_detour_ratio=args.widest_detour_ratio,
        widest_clearance_target_m=args.widest_clearance_target,
    )
    print(
        generate_ground_truth_dataset(
            args.output,
            scene_count=args.scenes,
            frames_per_scene=args.frames,
            seed=args.seed,
            config=config,
            preview_routes=args.preview_routes,
            overwrite=args.overwrite,
            dataset_role=args.dataset_role,
            scene_styles=(
                tuple(value.strip() for value in args.scene_styles.split(","))
                if args.scene_styles else None
            ),
            yopo_tree_ply=args.yopo_tree_ply,
        )
    )


if __name__ == "__main__":
    main()
