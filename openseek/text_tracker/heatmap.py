from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np
import torch
from torch.nn import functional as F


def _validate_goal(goal_body: np.ndarray) -> np.ndarray:
    goal = np.asarray(goal_body, dtype=np.float32)
    if goal.shape != (3,):
        raise ValueError(f"goal_body must have shape (3,), got {goal.shape}")
    if not np.isfinite(goal).all():
        raise ValueError("goal_body must be finite")
    return goal


def goal_body_to_heatmap(
    goal_body: np.ndarray,
    *,
    width: int = 160,
    height: int = 32,
    horizontal_fov_deg: float = 90.0,
    vertical_fov_deg: float = 60.0,
    horizontal_only: bool = True,
    sigma_deg: float = 7.5,
    amplitude: float = 1.0,
    distance_scale: float | None = None,
) -> np.ndarray:
    """Project a 3-D body-frame goal into a front-camera heatmap.

    Body axes are x-forward, y-left, z-up. Goals outside the camera frustum are
    placed on the nearest image edge so a front-facing policy can turn toward
    them. When ``distance_scale`` is set, peak amplitude is multiplied by the
    remaining-distance ratio clipped to [0, 1].
    """
    goal = _validate_goal(goal_body)
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("horizontal_fov_deg must be between 0 and 180")
    if not horizontal_only and not 0.0 < vertical_fov_deg < 180.0:
        raise ValueError("vertical_fov_deg must be between 0 and 180")
    if sigma_deg <= 0.0:
        raise ValueError("sigma_deg must be positive")
    if distance_scale is not None and distance_scale <= 0.0:
        raise ValueError("distance_scale must be positive")

    distance = float(np.linalg.norm(goal))
    if distance_scale is not None:
        amplitude *= np.clip(distance / distance_scale, 0.0, 1.0)
    if distance < 1e-6 or abs(amplitude) < 1e-8:
        return np.zeros((height, width), dtype=np.float32)

    forward, left, up = (float(value) for value in goal)
    azimuth = np.arctan2(left, forward)
    elevation = np.arctan2(up, np.hypot(forward, left))
    half_horizontal = np.deg2rad(horizontal_fov_deg) * 0.5
    clipped_azimuth = np.clip(azimuth, -half_horizontal, half_horizontal)
    fx = (width - 1) * 0.5 / np.tan(half_horizontal)
    center_x = (width - 1) * 0.5 - fx * np.tan(clipped_azimuth)

    sigma_x = max(
        1.0,
        fx * np.tan(np.deg2rad(sigma_deg)),
    )
    x = np.arange(width, dtype=np.float32)
    horizontal = np.exp(-0.5 * ((x - center_x) / sigma_x) ** 2)

    if horizontal_only:
        heatmap = np.broadcast_to(horizontal, (height, width)).copy()
    else:
        half_vertical = np.deg2rad(vertical_fov_deg) * 0.5
        clipped_elevation = np.clip(elevation, -half_vertical, half_vertical)
        fy = (height - 1) * 0.5 / np.tan(half_vertical)
        center_y = (height - 1) * 0.5 - fy * np.tan(clipped_elevation)
        sigma_y = max(1.0, fy * np.tan(np.deg2rad(sigma_deg)))
        y = np.arange(height, dtype=np.float32)
        vertical = np.exp(-0.5 * ((y - center_y) / sigma_y) ** 2)
        heatmap = vertical[:, None] * horizontal[None, :]

    return np.clip(amplitude * heatmap, -1.0, 1.0).astype(np.float32)


def goal_world_to_heatmap(
    goal_world: np.ndarray,
    position_world: np.ndarray,
    rotation_world_body: np.ndarray,
    **kwargs,
) -> np.ndarray:
    goal = np.asarray(goal_world, dtype=np.float32)
    position = np.asarray(position_world, dtype=np.float32)
    rotation = np.asarray(rotation_world_body, dtype=np.float32)
    if goal.shape != (3,) or position.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("goal/position must be 3-D and rotation must be 3x3")
    goal_body = rotation.T @ (goal - position)
    return goal_body_to_heatmap(goal_body, **kwargs)


def resize_heatmap(heatmap: np.ndarray, width: int, height: int) -> np.ndarray:
    values = np.nan_to_num(
        np.asarray(heatmap, dtype=np.float32), nan=0.0, posinf=1.0, neginf=-1.0
    )
    resized = cv2.resize(values, (width, height), interpolation=cv2.INTER_LINEAR)
    return np.clip(resized, -1.0, 1.0).astype(np.float32)


def pool_heatmap_to_primitives(
    heatmap: torch.Tensor,
    vertical_num: int,
    horizon_num: int,
    top_fraction: float = 0.1,
) -> torch.Tensor:
    """Reduce a signed image heatmap to one absolute-scale value per primitive."""
    if heatmap.ndim != 3:
        raise ValueError(f"heatmap must be [B,H,W], got {tuple(heatmap.shape)}")
    batch, height, width = heatmap.shape
    if height % vertical_num or width % horizon_num:
        raise ValueError(
            f"heatmap {height}x{width} is not divisible by "
            f"primitive grid {vertical_num}x{horizon_num}"
        )
    cells = heatmap.reshape(
        batch,
        vertical_num,
        height // vertical_num,
        horizon_num,
        width // horizon_num,
    )
    cells = cells.permute(0, 1, 3, 2, 4).flatten(start_dim=3)
    count = max(1, int(cells.shape[-1] * top_fraction))
    indices = cells.abs().topk(count, dim=-1).indices
    return cells.gather(-1, indices).mean(dim=-1)


def sample_heatmap_at_body_directions(
    heatmap: torch.Tensor,
    directions: torch.Tensor,
    *,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    horizontal_only: bool,
) -> torch.Tensor:
    """Differentiably sample a front-camera heatmap using body-frame vectors."""
    if heatmap.ndim != 3 or directions.ndim != 4 or directions.shape[-1] != 3:
        raise ValueError("expected heatmap [B,H,W] and directions [B,V,H,3]")
    forward = directions[..., 0]
    left = directions[..., 1]
    up = directions[..., 2]
    horizontal_sq = forward.square() + left.square()
    direction_sq = horizontal_sq + up.square()
    direction_valid = direction_sq >= 1e-6
    horizontal_valid = horizontal_sq >= 1e-6

    # atan2(0, 0) has a finite forward value but NaN gradients. A zero-length
    # endpoint has no meaningful direction, so sample forward and mask it out.
    safe_forward = torch.where(
        horizontal_valid, forward, torch.ones_like(forward)
    )
    safe_left = torch.where(horizontal_valid, left, torch.zeros_like(left))
    azimuth = torch.atan2(safe_left, safe_forward)
    half_horizontal = torch.deg2rad(
        heatmap.new_tensor(horizontal_fov_deg * 0.5)
    )
    grid_x = -torch.tan(azimuth) / torch.tan(half_horizontal)

    source = heatmap.unsqueeze(1)
    if horizontal_only:
        indices = heatmap.abs().argmax(dim=1, keepdim=True)
        source = heatmap.gather(1, indices).unsqueeze(1)
        grid_y = torch.zeros_like(grid_x)
    else:
        safe_horizontal = torch.sqrt(horizontal_sq.clamp_min(1e-6))
        elevation = torch.atan2(up, safe_horizontal)
        half_vertical = torch.deg2rad(
            heatmap.new_tensor(vertical_fov_deg * 0.5)
        )
        grid_y = -torch.tan(elevation) / torch.tan(half_vertical)

    grid = torch.stack([grid_x, grid_y], dim=-1)
    sampled = F.grid_sample(
        source,
        grid,
        mode="bilinear",
        padding_mode="zeros",
        align_corners=True,
    )
    return torch.where(direction_valid, sampled[:, 0], torch.zeros_like(sampled[:, 0]))


def pearl_similarity(heatmap: np.ndarray, top_fraction: float = 0.0005) -> float:
    """Compute an absolute PEARL confidence without per-frame normalization."""
    values = np.nan_to_num(np.asarray(heatmap, dtype=np.float32), nan=0.0)
    values = np.clip(values, 0.0, 1.0).reshape(-1)
    count = min(values.size, max(16, int(values.size * top_fraction)))
    partition = np.partition(values, values.size - count)
    return float(partition[-count:].mean())


def heatmap_peak_body_goal(
    heatmap: np.ndarray,
    *,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    distance: float,
) -> np.ndarray:
    """Convert the strongest heatmap pixel to a 3-D body-FLU goal vector."""
    values = np.asarray(heatmap, dtype=np.float32)
    if values.ndim != 2 or values.size == 0 or not np.isfinite(values).all():
        raise ValueError("heatmap must be a finite non-empty 2-D array")
    if distance <= 0.0:
        raise ValueError("distance must be positive")
    height, width = values.shape
    row, column = np.unravel_index(int(values.argmax()), values.shape)
    center_x = (width - 1) * 0.5
    center_y = (height - 1) * 0.5
    fx = center_x / np.tan(np.deg2rad(horizontal_fov_deg) * 0.5)
    fy = center_y / np.tan(np.deg2rad(vertical_fov_deg) * 0.5)
    ray = np.array(
        [1.0, (center_x - float(column)) / fx, (center_y - float(row)) / fy],
        dtype=np.float32,
    )
    return ray * (float(distance) / float(np.linalg.norm(ray)))


class GuidanceMode(str, Enum):
    SEARCH = "search"
    APPROACH = "approach"


@dataclass(frozen=True)
class GuidanceSelection:
    mode: GuidanceMode
    confidence: float
    heatmap: np.ndarray


class HeatmapModeSelector:
    """Hysteretic switch between a numeric search goal and a PEARL map."""

    def __init__(
        self,
        *,
        width: int = 160,
        height: int = 32,
        horizontal_fov_deg: float = 90.0,
        vertical_fov_deg: float = 60.0,
        horizontal_only: bool = True,
        goal_distance_scale: float = 10.0,
        enter_threshold: float = 0.08,
        exit_threshold: float = 0.05,
        enter_frames: int = 3,
        exit_frames: int = 5,
    ) -> None:
        if not 0.0 <= exit_threshold < enter_threshold <= 1.0:
            raise ValueError("thresholds must satisfy 0 <= exit < enter <= 1")
        if goal_distance_scale <= 0.0:
            raise ValueError("goal_distance_scale must be positive")
        if enter_frames <= 0 or exit_frames <= 0:
            raise ValueError("frame counts must be positive")
        self.width = width
        self.height = height
        self.horizontal_fov_deg = horizontal_fov_deg
        self.vertical_fov_deg = vertical_fov_deg
        self.horizontal_only = horizontal_only
        self.goal_distance_scale = goal_distance_scale
        self.enter_threshold = enter_threshold
        self.exit_threshold = exit_threshold
        self.enter_frames = enter_frames
        self.exit_frames = exit_frames
        self.mode = GuidanceMode.SEARCH
        self._enter_count = 0
        self._exit_count = 0

    def update(
        self,
        goal_body: np.ndarray,
        pearl_heatmap: np.ndarray | None,
    ) -> GuidanceSelection:
        confidence = 0.0 if pearl_heatmap is None else pearl_similarity(pearl_heatmap)
        if self.mode is GuidanceMode.SEARCH:
            self._enter_count = self._enter_count + 1 if confidence >= self.enter_threshold else 0
            if self._enter_count >= self.enter_frames:
                self.mode = GuidanceMode.APPROACH
                self._enter_count = 0
        else:
            self._exit_count = self._exit_count + 1 if confidence < self.exit_threshold else 0
            if self._exit_count >= self.exit_frames:
                self.mode = GuidanceMode.SEARCH
                self._exit_count = 0

        if self.mode is GuidanceMode.APPROACH and pearl_heatmap is not None:
            heatmap = resize_heatmap(pearl_heatmap, self.width, self.height)
        else:
            heatmap = goal_body_to_heatmap(
                goal_body,
                width=self.width,
                height=self.height,
                horizontal_fov_deg=self.horizontal_fov_deg,
                vertical_fov_deg=self.vertical_fov_deg,
                horizontal_only=self.horizontal_only,
                distance_scale=self.goal_distance_scale,
            )
        return GuidanceSelection(self.mode, confidence, heatmap)
