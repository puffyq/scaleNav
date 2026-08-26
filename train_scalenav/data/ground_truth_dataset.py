from __future__ import annotations

import argparse
import heapq
import json
import math
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

from .coordinates import enu_to_ned
from .route_contract import (
    RouteQualityConfig,
    RouteQualityGate,
    RouteRecord,
    build_witness_corridor,
    pack_route_records,
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


Obstacle = CylinderObstacle | BoxObstacle


@dataclass(frozen=True)
class GroundTruthConfig:
    map_size_x_m: float = 40.0
    map_size_y_m: float = 40.0
    map_height_m: float = 5.0
    altitude_m: float = 1.6
    grid_resolution_m: float = 0.2
    point_resolution_m: float = 0.2
    obstacle_count: int = 40
    scene_style: str = "alternating"
    forest_fraction: float = 0.55
    tree_radius_min_m: float = 0.3
    tree_radius_max_m: float = 0.75
    wall_length_min_m: float = 2.0
    wall_length_max_m: float = 5.0
    wall_thickness_m: float = 0.35
    block_size_min_m: float = 2.5
    block_size_max_m: float = 6.5
    robot_radius_m: float = 0.3
    safety_margin_m: float = 0.2
    planning_extra_margin_m: float = 0.35
    safe_pose_clearance_m: float = 0.8
    route_min_length_m: float = 7.0
    route_max_length_m: float = 24.0
    routes_per_frame: int = 3
    image_width: int = 160
    image_height: int = 96
    horizontal_fov_deg: float = 90.0
    vertical_fov_deg: float = 60.0
    max_depth_m: float = 20.0
    maximum_frame_attempts: int = 250
    maximum_route_attempts: int = 120

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
            self.max_depth_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("ground-truth scene dimensions and distances must be positive")
        if self.altitude_m >= self.map_height_m:
            raise ValueError("altitude must be below map height")
        if self.route_min_length_m >= self.route_max_length_m:
            raise ValueError("route_min_length_m must be smaller than route_max_length_m")
        if not 0.0 <= self.forest_fraction <= 1.0:
            raise ValueError("forest_fraction must be in [0, 1]")
        if self.scene_style not in {"alternating", "blocks", "mixed", "forest"}:
            raise ValueError("scene_style must be alternating, blocks, mixed, or forest")
        if not 0.0 < self.block_size_min_m <= self.block_size_max_m:
            raise ValueError("invalid block size range")
        if self.obstacle_count < 0 or self.routes_per_frame <= 0:
            raise ValueError("obstacle_count and routes_per_frame are invalid")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image dimensions must be positive")


class GroundTruthScene:
    """Analytic obstacles, a fixed-height planning grid, and a shared renderer."""

    def __init__(self, config: GroundTruthConfig, obstacles: Sequence[Obstacle]) -> None:
        self.config = config
        self.obstacles = tuple(obstacles)
        self.x_min = -0.5 * config.map_size_x_m
        self.y_min = -0.5 * config.map_size_y_m
        self.nx = int(math.ceil(config.map_size_x_m / config.grid_resolution_m))
        self.ny = int(math.ceil(config.map_size_y_m / config.grid_resolution_m))
        self.raw_occupancy = self._build_occupancy()
        self.clearance_m = distance_transform_edt(~self.raw_occupancy) * config.grid_resolution_m
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

    @classmethod
    def random(
        cls, config: GroundTruthConfig, seed: int, *, style: str | None = None
    ) -> "GroundTruthScene":
        rng = np.random.default_rng(seed)
        obstacles: list[Obstacle] = []
        boundary_margin = 1.5
        selected_style = config.scene_style if style is None else style
        if selected_style == "alternating":
            selected_style = "mixed"
        obstacle_count = config.obstacle_count
        if selected_style == "blocks":
            obstacle_count = max(8, int(round(0.45 * obstacle_count)))
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
                size_x = float(rng.uniform(config.block_size_min_m, config.block_size_max_m))
                size_y = float(rng.uniform(config.block_size_min_m, config.block_size_max_m))
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

    def _grid_centers(self) -> tuple[np.ndarray, np.ndarray]:
        x = self.x_min + (np.arange(self.nx, dtype=np.float32) + 0.5) * self.config.grid_resolution_m
        y = self.y_min + (np.arange(self.ny, dtype=np.float32) + 0.5) * self.config.grid_resolution_m
        return np.meshgrid(x, y, indexing="ij")

    def _build_occupancy(self) -> np.ndarray:
        x, y = self._grid_centers()
        occupied = np.zeros((self.nx, self.ny), dtype=bool)
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

    def nearest_planning_free_cell(self, point: Sequence[float]) -> tuple[int, int]:
        requested = np.asarray(self.world_to_cell(point), dtype=np.float32)
        _, index = self.free_cell_tree.query(requested, k=1)
        cell = self.free_cells[int(index)]
        return int(cell[0]), int(cell[1])

    def is_segment_free(
        self, start: np.ndarray, end: np.ndarray, *, planning_margin: bool = True
    ) -> bool:
        length = float(np.linalg.norm(end[:2] - start[:2]))
        count = max(2, int(math.ceil(length / (0.45 * self.config.grid_resolution_m))) + 1)
        samples = np.linspace(start[:2], end[:2], count, dtype=np.float32)
        indices_x = np.floor((samples[:, 0] - self.x_min) / self.config.grid_resolution_m)
        indices_y = np.floor((samples[:, 1] - self.y_min) / self.config.grid_resolution_m)
        indices_x = np.clip(indices_x.astype(np.int32), 0, self.nx - 1)
        indices_y = np.clip(indices_y.astype(np.int32), 0, self.ny - 1)
        occupancy = self.planning_occupancy if planning_margin else self.smoothing_occupancy
        return not bool(np.any(occupancy[indices_x, indices_y]))

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
        if self.planning_occupancy[start] or self.planning_occupancy[goal]:
            return None
        neighbors = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        cost = {start: 0.0}
        closed: set[tuple[int, int]] = set()
        while queue:
            _, current_cost, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == goal:
                path = [current]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                return list(reversed(path))
            closed.add(current)
            for dx, dy, step_cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < self.nx and 0 <= nxt[1] < self.ny):
                    continue
                if self.planning_occupancy[nxt]:
                    continue
                if dx and dy and (self.planning_occupancy[current[0] + dx, current[1]]
                                  or self.planning_occupancy[current[0], current[1] + dy]):
                    continue
                candidate = current_cost + step_cost
                if candidate >= cost.get(nxt, math.inf):
                    continue
                cost[nxt] = candidate
                parent[nxt] = current
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(queue, (candidate + heuristic, candidate, nxt))
        return None

    def smooth_grid_path(self, cells: Sequence[tuple[int, int]]) -> np.ndarray | None:
        raw = np.stack([self.cell_to_world(cell) for cell in cells])
        simplified = [raw[0]]
        index = 0
        while index < len(raw) - 1:
            candidate = len(raw) - 1
            while candidate > index + 1 and not self.is_segment_free(raw[index], raw[candidate]):
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
                self.is_segment_free(left, right, planning_margin=False)
                for left, right in zip(candidate_path[:-1], candidate_path[1:])
            ):
                break
            path = candidate_path
        for left, right in zip(path[:-1], path[1:]):
            if not self.is_segment_free(left, right, planning_margin=False):
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
                    self.is_segment_free(left, right, planning_margin=False)
                    for left, right in zip(candidate[:-1], candidate[1:])
                ):
                    path = candidate
        return path

    def obstacle_point_cloud(self) -> np.ndarray:
        step = self.config.point_resolution_m
        points: list[np.ndarray] = []
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
    *,
    require_detour: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
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
    eligible = (
        (distance >= config.route_min_length_m)
        & (distance <= config.route_max_length_m)
        & (heading_error <= math.radians(42.0))
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
        cells = scene.astar(start_cell, goal_cell)
        if cells is None:
            rejection_stats["astar_failed"] = rejection_stats.get("astar_failed", 0) + 1
            continue
        smoothed = scene.smooth_grid_path(cells)
        if smoothed is None:
            rejection_stats["smoothing_failed"] = rejection_stats.get("smoothing_failed", 0) + 1
            continue
        length = _path_length(smoothed)
        if not config.route_min_length_m <= length <= 1.5 * config.route_max_length_m:
            rejection_stats["route_length"] = rejection_stats.get("route_length", 0) + 1
            continue
        points, clearance, radius = build_witness_corridor(
            smoothed,
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
            return points, clearance, radius
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
) -> Path:
    output = Path(output)
    if scene_count <= 0 or frames_per_scene <= 0:
        raise ValueError("scene_count and frames_per_scene must be positive")
    if output.exists() and any(output.iterdir()):
        if not overwrite:
            raise FileExistsError(f"output directory is not empty: {output}")
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)
    report: dict[str, object] = {
        "generator": "ScaleNav ground-truth YOPO-style dataset",
        "seed": seed,
        "config": asdict(config),
        "scenes": [],
    }
    preview_index = 0
    route_lengths: list[float] = []
    route_clearances: list[float] = []
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
        style = config.scene_style
        if style == "alternating":
            style = "blocks" if scene_index % 2 == 0 else "mixed"
        truth = GroundTruthScene.random(config, scene_seed, style=style)
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
            accepted_routes: list[tuple[np.ndarray, np.ndarray, np.ndarray]] | None = None
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
                for obstacle in truth.obstacles:
                    center, _ = _obstacle_center_and_reach(obstacle)
                    delta = center - start[:2]
                    distance = float(np.linalg.norm(delta))
                    if 3.0 < distance < config.route_max_length_m - 3.0:
                        blocker_headings.append(math.atan2(float(delta[1]), float(delta[0])))
                if not blocker_headings:
                    continue
                yaw = blocker_headings[int(rng.integers(0, len(blocker_headings)))]
                yaw += float(rng.uniform(-math.radians(18.0), math.radians(18.0)))
                candidates: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
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
            for variant, (points, clearance, radius) in enumerate(accepted_routes):
                topo_indices = np.linspace(0, len(points) - 1, min(8, len(points)), dtype=np.int64)
                frontier = points[-1].copy()
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
                        route_quality_weight=1.0,
                        route_seed=route_seed,
                    )
                )
                route_lengths.append(_path_length(points))
                route_clearances.append(float(np.min(clearance)))
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
        "--scene-style",
        choices=("alternating", "blocks", "mixed", "forest"),
        default="alternating",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--preview-routes", type=int, default=100)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    config = GroundTruthConfig(
        routes_per_frame=args.routes_per_frame,
        obstacle_count=args.obstacles,
        scene_style=args.scene_style,
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
        )
    )


if __name__ == "__main__":
    main()
