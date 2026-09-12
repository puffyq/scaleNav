"""Double Sphere camera projection and perspective remapping utilities."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


RECORDED_DEPTH_SCALE = 0.07812003
RECORDED_DEPTH_OFFSET = 0.0166666667
RECORDED_MAX_DISTANCE_M = 50.0


@dataclass(frozen=True)
class DoubleSphereIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    xi: float
    alpha: float
    width: int
    height: int

    def validate(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy, self.xi, self.alpha)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Double Sphere intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0 or self.width <= 0 or self.height <= 0:
            raise ValueError("Double Sphere focal lengths and image size must be positive")
        if not 0.0 < self.alpha < 1.0:
            raise ValueError("Double Sphere alpha must be in (0, 1)")


def perspective_intrinsics(
    width: int, height: int, horizontal_fov_deg: float, vertical_fov_deg: float
) -> tuple[float, float, float, float]:
    if width <= 0 or height <= 0:
        raise ValueError("perspective output size must be positive")
    if not 0.0 < horizontal_fov_deg < 180.0 or not 0.0 < vertical_fov_deg < 180.0:
        raise ValueError("perspective FOV must be in (0, 180) degrees")
    fx = 0.5 * width / math.tan(math.radians(horizontal_fov_deg) * 0.5)
    fy = 0.5 * height / math.tan(math.radians(vertical_fov_deg) * 0.5)
    return fx, fy, (width - 1) * 0.5, (height - 1) * 0.5


def make_perspective_to_ds_map(
    source: DoubleSphereIntrinsics,
    output_width: int,
    output_height: int,
    horizontal_fov_deg: float = 90.0,
    vertical_fov_deg: float = 73.7398,
) -> tuple[np.ndarray, np.ndarray, tuple[float, float, float, float]]:
    """Map a pinhole output pixel to the calibrated Double Sphere image.

    Formula follows Usenko et al., "The Double Sphere Camera Model" (3DV 2018).
    Rays use ROS optical axes: +x right, +y down, +z forward.
    """
    source.validate()
    fx, fy, cx, cy = perspective_intrinsics(
        output_width, output_height, horizontal_fov_deg, vertical_fov_deg
    )
    u, v = np.meshgrid(
        np.arange(output_width, dtype=np.float64),
        np.arange(output_height, dtype=np.float64),
    )
    x = (u - cx) / fx
    y = (v - cy) / fy
    z = np.ones_like(x)
    d1 = np.sqrt(x * x + y * y + z * z)
    z_xi = source.xi * d1 + z
    d2 = np.sqrt(x * x + y * y + z_xi * z_xi)
    denominator = source.alpha * d2 + (1.0 - source.alpha) * z_xi
    valid = np.isfinite(denominator) & (denominator > 1e-9)
    map_x = np.full_like(x, -1.0, dtype=np.float32)
    map_y = np.full_like(y, -1.0, dtype=np.float32)
    map_x[valid] = (source.fx * x[valid] / denominator[valid] + source.cx).astype(
        np.float32
    )
    map_y[valid] = (source.fy * y[valid] / denominator[valid] + source.cy).astype(
        np.float32
    )
    valid &= (
        (map_x >= 0.0)
        & (map_x <= source.width - 1)
        & (map_y >= 0.0)
        & (map_y <= source.height - 1)
    )
    map_x[~valid] = -1.0
    map_y[~valid] = -1.0
    return map_x, map_y, (fx, fy, cx, cy)


def remap_image(
    image: np.ndarray, map_x: np.ndarray, map_y: np.ndarray, *, depth: bool = False
) -> np.ndarray:
    interpolation = cv2.INTER_NEAREST if depth else cv2.INTER_LINEAR
    return cv2.remap(
        image,
        map_x,
        map_y,
        interpolation,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )


def decode_recorded_depth(
    values: np.ndarray,
    invalid_mask: np.ndarray | None = None,
    max_distance_m: float = RECORDED_MAX_DISTANCE_M,
) -> np.ndarray:
    """Decode the recorded uint8 inverse-depth network output into metres.

    A zero network output is a valid far-plane value (approximately 50 m).
    Only pixels explicitly selected by ``invalid_mask`` are replaced by the
    configured maximum distance.
    """
    if values.dtype != np.uint8:
        raise ValueError("recorded depth must be uint8")
    if max_distance_m <= 0.0 or not math.isfinite(max_distance_m):
        raise ValueError("max_distance_m must be finite and positive")
    depth = (
        1.0
        / (
            values.astype(np.float32) / 255.0 * RECORDED_DEPTH_SCALE
            + RECORDED_DEPTH_OFFSET
        )
        - 10.0
    ).astype(np.float32)
    if invalid_mask is not None:
        if invalid_mask.shape != values.shape:
            raise ValueError("invalid mask shape does not match depth image")
        depth[np.asarray(invalid_mask, dtype=bool)] = np.float32(max_distance_m)
    return depth


def double_sphere_unproject_grid(
    intrinsics: DoubleSphereIntrinsics,
    *,
    minimum_elevation_deg: float = -20.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the original code's direction table and geometric valid mask.

    Directions first follow the calibrated Double Sphere inverse projection.
    The image Y component is then negated, matching ``creatSinglekernel()``.
    ``theta`` is the angle from the optical axis and ``phi`` is elevation.
    """
    intrinsics.validate()
    u, v = np.meshgrid(
        np.arange(intrinsics.width, dtype=np.float64),
        np.arange(intrinsics.height, dtype=np.float64),
    )
    mx = (u - intrinsics.cx) / intrinsics.fx
    my = (v - intrinsics.cy) / intrinsics.fy
    r2 = mx * mx + my * my
    inner = 1.0 - (2.0 * intrinsics.alpha - 1.0) * r2
    valid = np.isfinite(inner) & (inner >= 0.0)
    safe_inner = np.maximum(inner, 0.0)
    denominator = (
        intrinsics.alpha * np.sqrt(safe_inner) + 1.0 - intrinsics.alpha
    )
    valid &= np.isfinite(denominator) & (np.abs(denominator) > 1e-12)
    mz = np.divide(
        1.0 - intrinsics.alpha * intrinsics.alpha * r2,
        denominator,
        out=np.zeros_like(r2),
        where=valid,
    )
    second_inner = mz * mz + (1.0 - intrinsics.xi * intrinsics.xi) * r2
    valid &= np.isfinite(second_inner) & (second_inner >= 0.0)
    ray_denominator = mz * mz + r2
    valid &= np.isfinite(ray_denominator) & (ray_denominator > 1e-12)
    s = np.divide(
        mz * intrinsics.xi + np.sqrt(np.maximum(second_inner, 0.0)),
        ray_denominator,
        out=np.zeros_like(r2),
        where=valid,
    )
    ray_x = s * mx
    ray_y = -(s * my)
    ray_z = s * mz - intrinsics.xi
    rays = np.stack((ray_x, ray_y, ray_z), axis=-1)
    norm = np.linalg.norm(rays, axis=-1)
    valid &= np.isfinite(rays).all(axis=-1) & (norm > 1e-12)
    rays = np.divide(
        rays,
        norm[..., None],
        out=np.zeros_like(rays),
        where=(norm > 1e-12)[..., None],
    )
    theta = np.arctan2(np.hypot(rays[..., 0], rays[..., 1]), rays[..., 2])
    phi = np.arctan2(rays[..., 1], np.hypot(rays[..., 0], rays[..., 2]))
    valid &= (theta <= math.pi * 0.5) & (
        phi >= math.radians(minimum_elevation_deg)
    )
    rays[~valid] = 0.0
    return rays.astype(np.float32), valid


def depth_to_original_camera_points(
    values: np.ndarray,
    intrinsics: DoubleSphereIntrinsics,
    *,
    rays: np.ndarray | None = None,
    ray_valid: np.ndarray | None = None,
    invalid_mask: np.ndarray | None = None,
    max_distance_m: float = RECORDED_MAX_DISTANCE_M,
    include_far_plane: bool = False,
    stride: int = 1,
) -> np.ndarray:
    """Convert recorded depth to the original camera coordinate ordering.

    The returned order exactly follows the source implementation:
    ``(d*ray.x, d*ray.z, d*ray.y)``.
    """
    if values.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("depth image dimensions do not match intrinsics")
    if stride <= 0:
        raise ValueError("stride must be positive")
    if rays is None or ray_valid is None:
        rays, ray_valid = double_sphere_unproject_grid(intrinsics)
    if rays.shape != values.shape + (3,) or ray_valid.shape != values.shape:
        raise ValueError("ray table dimensions do not match depth image")
    depth = decode_recorded_depth(values, invalid_mask, max_distance_m)
    valid = ray_valid & np.isfinite(depth) & (depth > 0.0)
    if include_far_plane:
        valid &= depth <= max_distance_m + 1e-3
    else:
        valid &= depth < max_distance_m - 1e-3
    if stride > 1:
        sample = np.zeros_like(valid)
        sample[::stride, ::stride] = True
        valid &= sample
    selected_rays = rays[valid]
    selected_depth = depth[valid, None]
    return np.column_stack(
        (
            selected_depth[:, 0] * selected_rays[:, 0],
            selected_depth[:, 0] * selected_rays[:, 2],
            selected_depth[:, 0] * selected_rays[:, 1],
        )
    ).astype(np.float32)


def recorded_depth_to_perspective(
    values: np.ndarray,
    intrinsics: DoubleSphereIntrinsics,
    output_width: int,
    output_height: int,
    horizontal_fov_deg: float = 90.0,
    vertical_fov_deg: float = 73.7398,
    *,
    rays: np.ndarray | None = None,
    ray_valid: np.ndarray | None = None,
    max_depth_m: float = 20.0,
) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """Resample DS range into a dense pinhole optical-Z depth image.

    The source point-cloud elevation filter is intentionally not used here:
    it belongs to ``creatSinglekernel()``, while YOPO expects its complete
    perspective image. Each target pinhole ray is mapped into the DS image,
    then range is converted to optical Z by dividing by the ray norm.
    """
    if values.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("depth image dimensions do not match intrinsics")
    map_x, map_y, pinhole = make_perspective_to_ds_map(
        intrinsics, output_width, output_height,
        horizontal_fov_deg, vertical_fov_deg,
    )
    sampled = remap_image(values, map_x, map_y, depth=True)
    ranges = decode_recorded_depth(sampled)
    fx, fy, cx, cy = pinhole
    u, v = np.meshgrid(
        np.arange(output_width, dtype=np.float32),
        np.arange(output_height, dtype=np.float32),
    )
    ray_norm = np.sqrt(
        ((u - cx) / fx) ** 2 + ((v - cy) / fy) ** 2 + 1.0
    )
    planar = ranges / ray_norm
    outside = (map_x < 0.0) | (map_y < 0.0)
    planar[outside] = max_depth_m
    return np.clip(planar, 0.0, max_depth_m).astype(np.float32), pinhole
