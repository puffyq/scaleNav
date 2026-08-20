from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Protocol, Sequence

import numpy as np

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")
import cv2


class SnapshotClient(Protocol):
    def simSetVehiclePose(self, pose: Any, ignore_collision: bool, vehicle_name: str = "") -> None:
        ...

    def simGetVehiclePose(self, vehicle_name: str = "") -> Any:
        ...

    def simGetImages(self, requests: Sequence[Any], vehicle_name: str = "") -> Sequence[Any]:
        ...


@dataclass(frozen=True)
class CaptureConfig:
    camera_name: str = "0"
    vehicle_name: str = "drone_1"
    color_order: str = "bgr"
    horizontal_fov_deg: float = 90.0
    vertical_fov_deg: float = 60.0
    max_depth_m: float = 20.0
    settle_time_s: float = 0.03

    def __post_init__(self) -> None:
        if not self.camera_name:
            raise ValueError("camera_name must not be empty")
        if self.color_order not in {"rgb", "bgr"}:
            raise ValueError("color_order must be 'rgb' or 'bgr'")
        if self.max_depth_m <= 0.0 or not math.isfinite(self.max_depth_m):
            raise ValueError("max_depth_m must be finite and positive")
        if not 0.0 < self.horizontal_fov_deg < 180.0:
            raise ValueError("horizontal_fov_deg must be between 0 and 180")
        if not 0.0 < self.vertical_fov_deg < 180.0:
            raise ValueError("vertical_fov_deg must be between 0 and 180")
        if self.settle_time_s < 0.0 or not math.isfinite(self.settle_time_s):
            raise ValueError("settle_time_s must be finite and non-negative")


@dataclass(frozen=True)
class PoseSample:
    """AirSim local-NED pose. Quaternion order is w, x, y, z."""

    position_ned: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        position = np.asarray(self.position_ned, dtype=np.float64)
        orientation = np.asarray(self.orientation_wxyz, dtype=np.float64)
        if position.shape != (3,) or orientation.shape != (4,):
            raise ValueError("pose must contain 3 position and 4 orientation values")
        if not np.isfinite(position).all() or not np.isfinite(orientation).all():
            raise ValueError("pose values must be finite")
        norm = float(np.linalg.norm(orientation))
        if norm < 1e-8:
            raise ValueError("orientation quaternion must not be zero")

    @property
    def yaw_deg(self) -> float:
        w, x, y, z = self.orientation_wxyz
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return math.degrees(yaw)


class PoseSampler:
    """Deterministic random snapshot poses in a rectangular local-NED area."""

    def __init__(
        self,
        x_range_m: tuple[float, float],
        y_range_m: tuple[float, float],
        altitude_m: float = 1.6,
        seed: int = 0,
    ) -> None:
        if x_range_m[0] >= x_range_m[1] or y_range_m[0] >= y_range_m[1]:
            raise ValueError("pose sampling ranges must be increasing")
        if altitude_m <= 0.0 or not math.isfinite(altitude_m):
            raise ValueError("altitude_m must be finite and positive")
        self.x_range_m = x_range_m
        self.y_range_m = y_range_m
        self.altitude_m = altitude_m
        self.seed = seed

    def sample(self, count: int) -> list[PoseSample]:
        if count < 0:
            raise ValueError("count must be non-negative")
        rng = np.random.default_rng(self.seed)
        result: list[PoseSample] = []
        for _ in range(count):
            x = float(rng.uniform(*self.x_range_m))
            y = float(rng.uniform(*self.y_range_m))
            yaw = float(rng.uniform(-math.pi, math.pi))
            result.append(
                PoseSample(
                    (x, y, -self.altitude_m),
                    (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)),
                )
            )
        return result


def _extract_image(response: Any, *, depth: bool, color_order: str = "rgb") -> np.ndarray:
    if hasattr(response, "image_data_float"):
        pixels = response.image_data_float if depth else response.image_data_uint8
        width = int(response.width)
        height = int(response.height)
        image_type = int(response.image_type)
    elif isinstance(response, dict):
        pixels = response["image_data_float" if depth else "image_data_uint8"]
        width = int(response["width"])
        height = int(response["height"])
        image_type = int(response["image_type"])
    else:
        raise TypeError("unsupported AirSim image response")

    expected_type = 1 if depth else 0
    if image_type != expected_type or width <= 0 or height <= 0:
        raise ValueError("AirSim returned an invalid RGB-D response")
    if isinstance(pixels, (bytes, bytearray, memoryview)):
        array = np.frombuffer(pixels, dtype=np.float32 if depth else np.uint8)
    else:
        array = np.asarray(pixels)
    if depth:
        array = array.astype(np.float32, copy=False).reshape(height, width)
    else:
        array = array.astype(np.uint8, copy=False).reshape(height, width, 3)
        if color_order == "bgr":
            array = array[:, :, ::-1]
    return np.ascontiguousarray(array)


def _pose_values(pose: Any) -> PoseSample:
    if hasattr(pose, "position"):
        position = pose.position
        orientation = pose.orientation
        return PoseSample(
            (float(position.x_val), float(position.y_val), float(position.z_val)),
            (
                float(orientation.w_val),
                float(orientation.x_val),
                float(orientation.y_val),
                float(orientation.z_val),
            ),
        )
    if isinstance(pose, dict):
        return PoseSample(
            tuple(float(value) for value in pose["position_ned"]),
            tuple(float(value) for value in pose["orientation_wxyz"]),
        )
    raise TypeError("unsupported AirSim pose")


def _toml_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def _rotation_from_quaternion_wxyz(quaternion: Sequence[float]) -> np.ndarray:
    """Return the active body-to-world rotation for a WXYZ quaternion."""
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1.0e-8:
        raise ValueError("orientation quaternion must not be zero")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def depth_planar_to_world_ned(
    depth_m: np.ndarray,
    pose: PoseSample,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    max_depth_m: float,
    *,
    stride: int = 4,
    max_points: int = 1500,
) -> np.ndarray:
    """Convert sampled AirSim DepthPlanar pixels into world-NED points."""
    if stride <= 0 or max_points <= 0:
        raise ValueError("stride and max_points must be positive")
    depth = np.asarray(depth_m, dtype=np.float32)[::stride, ::stride]
    height, width = depth.shape
    if width < 2 or height < 2:
        return np.empty((0, 3), dtype=np.float32)
    full_height, full_width = np.asarray(depth_m).shape
    fx = (full_width - 1) * 0.5 / np.tan(np.deg2rad(horizontal_fov_deg) * 0.5)
    fy = (full_height - 1) * 0.5 / np.tan(np.deg2rad(vertical_fov_deg) * 0.5)
    columns = np.arange(0, full_width, stride, dtype=np.float32)
    rows = np.arange(0, full_height, stride, dtype=np.float32)
    rows, columns = np.meshgrid(rows, columns, indexing="ij")
    valid = np.isfinite(depth) & (depth > 0.05) & (depth <= max_depth_m)
    camera_forward = depth
    camera_right = (columns - (full_width - 1) * 0.5) * camera_forward / fx
    camera_down = (rows - (full_height - 1) * 0.5) * camera_forward / fy
    # AirSim camera axes are forward/right/down, identical to vehicle FRD.
    body_frd = np.stack((camera_forward, camera_right, camera_down), axis=-1)[valid]
    rotation = _rotation_from_quaternion_wxyz(pose.orientation_wxyz)
    world = body_frd @ rotation.T + np.asarray(pose.position_ned, dtype=np.float32)
    if world.shape[0] > max_points:
        indices = np.linspace(0, world.shape[0] - 1, max_points, dtype=np.int64)
        world = world[indices]
    return world.astype(np.float32, copy=False)


class SceneWriter:
    """Writes the compact scene format consumed by the OpenSeek text dataset."""

    def __init__(
        self,
        scene_dir: Path,
        config: CaptureConfig,
        *,
        overwrite: bool = False,
        include_depth_obstacles: bool = False,
    ) -> None:
        self.scene_dir = Path(scene_dir)
        self.texture_dir = self.scene_dir / "Textures"
        if self.scene_dir.exists() and any(self.scene_dir.iterdir()) and not overwrite:
            raise FileExistsError(f"scene directory is not empty: {self.scene_dir}")
        self.texture_dir.mkdir(parents=True, exist_ok=True)
        self.config = config
        self.include_depth_obstacles = include_depth_obstacles
        self.depth_obstacle_points: list[np.ndarray] = []
        self.records: list[dict[str, Any]] = []

    def write_frame(
        self,
        index: int,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        pose: PoseSample,
        timestamp_ns: int,
        target_prompt: str,
    ) -> None:
        if index != len(self.records):
            raise ValueError("frame indices must be contiguous and start at zero")
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError("rgb must be uint8 HxWx3")
        if depth_m.ndim != 2 or depth_m.shape != rgb.shape[:2]:
            raise ValueError("depth must be HxW and match RGB dimensions")
        if not np.isfinite(depth_m).all():
            raise ValueError("depth contains non-finite values")
        depth = np.clip(depth_m.astype(np.float32), 0.0, self.config.max_depth_m)
        if self.include_depth_obstacles:
            self.depth_obstacle_points.append(
                depth_planar_to_world_ned(
                    depth,
                    pose,
                    self.config.horizontal_fov_deg,
                    self.config.vertical_fov_deg,
                    self.config.max_depth_m,
                )
            )
        rgb_name = f"rgb_{index:06d}.png"
        depth_name = f"depth_{index:06d}.exr"
        if not cv2.imwrite(str(self.texture_dir / rgb_name), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise IOError(f"failed to write {rgb_name}")
        if not cv2.imwrite(str(self.texture_dir / depth_name), depth):
            raise IOError(f"failed to write {depth_name}")
        self.records.append(
            {
                "frameIndex": index,
                "timestampNs": int(timestamp_ns),
                "rgbFileName": rgb_name,
                "depthFileName": depth_name,
                "posStart": [float(value) for value in pose.position_ned],
                "orientationWxyz": [float(value) for value in pose.orientation_wxyz],
                "yawStart": float(pose.yaw_deg),
                "targetPrompt": target_prompt,
                "depthMaxMeters": float(self.config.max_depth_m),
            }
        )

    def finalize(self, obstacle_ply: Path, *, person_positions: Path | None = None) -> Path:
        obstacle_ply = Path(obstacle_ply)
        if not obstacle_ply.is_file():
            raise FileNotFoundError(f"obstacle point cloud not found: {obstacle_ply}")
        destination = self.scene_dir / "tree.ply"
        source = obstacle_ply
        temporary_paths: list[Path] = []
        if self.depth_obstacle_points:
            temporary = self.scene_dir / ".tree_with_depth.ply"
            depth_points = np.concatenate(self.depth_obstacle_points, axis=0)
            static_points = read_ascii_point_cloud_ply(source)
            write_point_cloud_ply(temporary, np.concatenate((static_points, depth_points)))
            source = temporary
            temporary_paths.append(temporary)
        if person_positions is not None:
            # Merge into a temporary file in the scene directory, then replace
            # the destination in one operation. This also handles an existing
            # tree.ply as the source without truncating it first.
            temporary = self.scene_dir / ".tree_with_people.ply"
            merge_person_collision_point_cloud(source, person_positions, temporary)
            source = temporary
            temporary_paths.append(temporary)
        if source.resolve() != destination.resolve():
            shutil.copyfile(source, destination)
        for temporary in temporary_paths:
            temporary.unlink(missing_ok=True)
        lines = [
            f"depthCameraHorizontalFOV = {self.config.horizontal_fov_deg}",
            f"depthCameraVerticalFOV = {self.config.vertical_fov_deg}",
            f"depthMaxMeters = {self.config.max_depth_m}",
            "",
        ]
        for record in self.records:
            lines.extend(
                [
                    "[[dataArray]]",
                    f"frameIndex = {record['frameIndex']}",
                    f"timestampNs = {record['timestampNs']}",
                    f"rgbFileName = {_toml_string(record['rgbFileName'])}",
                    f"depthFileName = {_toml_string(record['depthFileName'])}",
                    f"posStart = {record['posStart']}",
                    f"orientationWxyz = {record['orientationWxyz']}",
                    f"yawStart = {record['yawStart']}",
                    f"targetPrompt = {_toml_string(record['targetPrompt'])}",
                    f"depthMaxMeters = {record['depthMaxMeters']}",
                    "",
                ]
            )
        data_path = self.scene_dir / "data.toml"
        data_path.write_text("\n".join(lines), encoding="utf-8")
        return data_path


class AirSimSnapshotCollector:
    def __init__(self, client: SnapshotClient, config: CaptureConfig) -> None:
        self.client = client
        self.config = config

    def collect_scene(
        self,
        scene_dir: Path,
        poses: Iterable[PoseSample],
        obstacle_ply: Path,
        *,
        target_prompt: str = "person",
        person_positions: Path | None = None,
        overwrite: bool = False,
    ) -> Path:
        writer = SceneWriter(
            scene_dir,
            self.config,
            overwrite=overwrite,
            include_depth_obstacles=True,
        )
        requests = self._image_requests()
        for index, requested_pose in enumerate(poses):
            self.client.simSetVehiclePose(
                self._to_airsim_pose(requested_pose), True, self.config.vehicle_name
            )
            if self.config.settle_time_s:
                time.sleep(self.config.settle_time_s)
            responses = self.client.simGetImages(requests, self.config.vehicle_name)
            if len(responses) != 2:
                raise RuntimeError("AirSim returned an incomplete RGB-D response")
            rgb = _extract_image(
                responses[0], depth=False, color_order=self.config.color_order
            )
            depth = _extract_image(responses[1], depth=True)
            actual_pose = _pose_values(self.client.simGetVehiclePose(self.config.vehicle_name))
            timestamp_ns = int(getattr(responses[0], "time_stamp", time.time_ns()))
            writer.write_frame(index, rgb, depth, actual_pose, timestamp_ns, target_prompt)
        return writer.finalize(obstacle_ply, person_positions=person_positions)

    def _image_requests(self) -> list[Any]:
        try:
            from colosseum import ImageRequest, ImageType

            return [
                ImageRequest(self.config.camera_name, ImageType.Scene, False, False),
                ImageRequest(self.config.camera_name, ImageType.DepthPlanar, True, False),
            ]
        except ImportError:
            # The test client and a low-level adapter can use the same wire shape.
            return [
                [self.config.camera_name, 0, False, False],
                [self.config.camera_name, 1, True, False],
            ]

    @staticmethod
    def _to_airsim_pose(pose: PoseSample) -> Any:
        try:
            from colosseum import Pose, Quaternionr, Vector3r

            x, y, z = pose.position_ned
            w, qx, qy, qz = pose.orientation_wxyz
            return Pose(Vector3r(x, y, z), Quaternionr(qx, qy, qz, w))
        except ImportError:
            return {
                "position_ned": list(pose.position_ned),
                "orientation_wxyz": list(pose.orientation_wxyz),
            }


class SceneValidationError(ValueError):
    pass


def _load_toml(path: Path) -> dict[str, Any]:
    try:
        import rtoml

        return dict(rtoml.load(path))
    except ImportError as error:
        raise RuntimeError("rtoml is required to validate collected scenes") from error


def validate_scene(scene_dir: Path, *, require_semantic: bool = False) -> int:
    scene_dir = Path(scene_dir)
    data_path = scene_dir / "data.toml"
    texture_dir = scene_dir / "Textures"
    obstacle_path = scene_dir / "tree.ply"
    if not data_path.is_file() or not texture_dir.is_dir() or not obstacle_path.is_file():
        raise SceneValidationError(f"incomplete scene: {scene_dir}")
    document = _load_toml(data_path)
    records = document.get("dataArray", [])
    if not records:
        raise SceneValidationError(f"scene has no frames: {scene_dir}")
    for index, record in enumerate(records):
        rgb_path = texture_dir / str(record.get("rgbFileName", ""))
        depth_path = texture_dir / str(record.get("depthFileName", ""))
        if not rgb_path.is_file() or not depth_path.is_file():
            raise SceneValidationError(f"missing frame files at index {index}")
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(depth_path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
        if rgb is None or depth is None or depth.ndim != 2:
            raise SceneValidationError(f"unreadable RGB-D frame at index {index}")
        if rgb.shape[:2] != depth.shape[:2]:
            raise SceneValidationError(f"RGB-D shape mismatch at index {index}")
        if require_semantic:
            semantic_name = f"semantic_pearl_{Path(str(record.get('rgbFileName', ''))).stem.removeprefix('rgb_')}.npy"
            semantic_path = texture_dir / semantic_name
            if not semantic_path.is_file():
                raise SceneValidationError(f"missing semantic heatmap at index {index}")
            try:
                semantic = np.load(semantic_path)
            except (OSError, ValueError) as error:
                raise SceneValidationError(
                    f"unreadable semantic heatmap at index {index}"
                ) from error
            if semantic.ndim != 2 or semantic.shape != rgb.shape[:2] or not np.isfinite(semantic).all():
                raise SceneValidationError(f"invalid semantic heatmap at index {index}")
        if len(record.get("posStart", [])) != 3 or len(record.get("orientationWxyz", [])) != 4:
            raise SceneValidationError(f"invalid pose metadata at index {index}")
    return len(records)


def validate_dataset(data_root: Path, *, require_semantic: bool = False) -> dict[str, int]:
    data_root = Path(data_root)
    scene_dirs = sorted(data_root.glob("Scene_*"))
    if not scene_dirs:
        raise SceneValidationError(f"no Scene_* directories under {data_root}")
    report = {
        scene.name: validate_scene(scene, require_semantic=require_semantic)
        for scene in scene_dirs
    }
    return report


def write_point_cloud_ply(path: Path, points: np.ndarray) -> Path:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("points must be a finite Nx3 array")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as file:
        file.write("ply\nformat ascii 1.0\n")
        file.write(f"element vertex {len(points)}\n")
        file.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for point in points:
            file.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    return path


def read_ascii_point_cloud_ply(path: Path) -> np.ndarray:
    """Read the XYZ vertex section of an ASCII PLY point cloud.

    The collector writes ASCII PLY deliberately so that scene files remain
    portable and easy to inspect. Binary PLY files are rejected with a clear
    error instead of being silently misparsed.
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"point cloud not found: {path}")
    try:
        lines = path.read_text(encoding="ascii").splitlines()
    except UnicodeDecodeError as error:
        raise ValueError(f"only ASCII PLY point clouds are supported: {path}") from error
    if not lines or lines[0].strip() != "ply":
        raise ValueError(f"not a PLY file: {path}")
    if not any(line.strip() == "format ascii 1.0" for line in lines[:20]):
        raise ValueError(f"only ASCII PLY point clouds are supported: {path}")
    vertex_count = None
    header_end = None
    for index, line in enumerate(lines):
        fields = line.split()
        if len(fields) == 3 and fields[0] == "element" and fields[1] == "vertex":
            vertex_count = int(fields[2])
        if line.strip() == "end_header":
            header_end = index
            break
    if vertex_count is None or header_end is None:
        raise ValueError(f"PLY header has no vertex count: {path}")
    body = lines[header_end + 1 : header_end + 1 + vertex_count]
    if len(body) != vertex_count:
        raise ValueError(f"PLY vertex section is truncated: {path}")
    points = []
    for line in body:
        fields = line.split()
        if len(fields) < 3:
            raise ValueError(f"invalid PLY vertex in {path}: {line!r}")
        points.append((float(fields[0]), float(fields[1]), float(fields[2])))
    array = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if not np.isfinite(array).all():
        raise ValueError(f"PLY contains non-finite vertices: {path}")
    return array


@dataclass(frozen=True)
class GeneratedPerson:
    """Generated-person foot position in AirSim local-NED meters."""

    position_ned: tuple[float, float, float]
    radius_m: float = 0.35
    height_m: float = 1.8

    def __post_init__(self) -> None:
        position = np.asarray(self.position_ned, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("person position must contain three finite values")
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("person radius_m must be finite and positive")
        if not math.isfinite(self.height_m) or self.height_m <= 0.0:
            raise ValueError("person height_m must be finite and positive")


def load_generated_people(path: Path) -> list[GeneratedPerson]:
    """Load UE PersonSpawner JSON and convert world centimeters to NED meters."""
    path = Path(path)
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid generated-person JSON: {path}") from error
    defaults = document if isinstance(document, dict) else {}
    entries = defaults.get("people") if isinstance(document, dict) else document
    if not isinstance(entries, list):
        raise ValueError("generated-person JSON must contain a 'people' array")
    result: list[GeneratedPerson] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("each generated person must be an object")
        position_cm = entry.get("positionCm")
        if not isinstance(position_cm, list) or len(position_cm) != 3:
            raise ValueError("each person requires positionCm=[x,y,z]")
        radius = float(entry.get("radiusMeters", defaults.get("radiusMeters", 0.35)))
        height = float(entry.get("heightMeters", defaults.get("heightMeters", 1.8)))
        x_cm, y_cm, z_cm = (float(value) for value in position_cm)
        result.append(
            GeneratedPerson((x_cm * 0.01, y_cm * 0.01, -z_cm * 0.01), radius, height)
        )
    return result


def generated_person_collision_points(
    people: Sequence[GeneratedPerson], *, radial_samples: int = 8, vertical_samples: int = 5
) -> np.ndarray:
    """Approximate each person by a capsule-like point cloud in NED meters."""
    if radial_samples < 4 or vertical_samples < 2:
        raise ValueError("radial_samples must be >= 4 and vertical_samples >= 2")
    points: list[tuple[float, float, float]] = []
    for person in people:
        x0, y0, z0 = person.position_ned
        for vertical in np.linspace(0.0, person.height_m, vertical_samples):
            z = z0 - float(vertical)
            for radial in range(radial_samples):
                angle = 2.0 * math.pi * radial / radial_samples
                points.append((x0 + person.radius_m * math.cos(angle),
                               y0 + person.radius_m * math.sin(angle), z))
        points.append((x0, y0, z0))
        points.append((x0, y0, z0 - person.height_m))
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)


def merge_person_collision_point_cloud(
    obstacle_ply: Path,
    person_positions: Path,
    output_ply: Path,
    *,
    radial_samples: int = 8,
    vertical_samples: int = 5,
) -> Path:
    """Append generated-person collision points to a static ASCII PLY."""
    static_points = read_ascii_point_cloud_ply(obstacle_ply)
    people = load_generated_people(person_positions)
    person_points = generated_person_collision_points(
        people, radial_samples=radial_samples, vertical_samples=vertical_samples
    )
    return write_point_cloud_ply(output_ply, np.concatenate((static_points, person_points)))


def export_static_mesh_point_cloud(
    client: Any,
    path: Path,
    *,
    max_points: int = 200_000,
    max_abs_coordinate_cm: float = 5.0e3,
    z_range_m: tuple[float, float] = (-8.0, 3.0),
) -> Path:
    """Export Colosseum static-mesh vertices in AirSim local-NED meters."""
    if max_points <= 0:
        raise ValueError("max_points must be positive")
    if max_abs_coordinate_cm <= 0.0 or not math.isfinite(max_abs_coordinate_cm):
        raise ValueError("max_abs_coordinate_cm must be finite and positive")
    if (
        len(z_range_m) != 2
        or z_range_m[0] >= z_range_m[1]
        or not all(math.isfinite(value) for value in z_range_m)
    ):
        raise ValueError("z_range_m must be an increasing pair of finite values")
    meshes = client.simGetMeshPositionVertexBuffers()
    chunks: list[np.ndarray] = []
    for mesh in meshes:
        vertices = getattr(mesh, "vertices", None)
        if vertices is None and isinstance(mesh, dict):
            vertices = mesh.get("vertices")
        if vertices is None:
            continue
        array = np.asarray(vertices, dtype=np.float32)
        if array.size == 0:
            continue
        if array.size % 3:
            raise ValueError("mesh vertex buffer length must be divisible by three")
        vertices_cm = array.reshape(-1, 3)
        valid = np.isfinite(vertices_cm).all(axis=1)
        valid &= np.abs(vertices_cm).max(axis=1) <= max_abs_coordinate_cm
        # Empty render-buffer entries are common for instanced/foliage meshes.
        # They otherwise create a dense fake obstacle at the NED origin.
        valid &= np.linalg.norm(vertices_cm, axis=1) > 1.0e-4
        if valid.any():
            chunks.append(vertices_cm[valid])
    if not chunks:
        raise RuntimeError("AirSim did not return any static mesh vertices")
    points_ue_cm = np.concatenate(chunks, axis=0)
    if len(points_ue_cm) > max_points:
        indices = np.linspace(0, len(points_ue_cm) - 1, max_points, dtype=np.int64)
        points_ue_cm = points_ue_cm[indices]
    points_ned_m = points_ue_cm * np.array([0.01, 0.01, -0.01], dtype=np.float32)
    points_ned_m = points_ned_m[
        (points_ned_m[:, 2] >= z_range_m[0]) & (points_ned_m[:, 2] <= z_range_m[1])
    ]
    if len(points_ned_m) == 0:
        raise RuntimeError("static mesh export has no vertices in z_range_m")
    return write_point_cloud_ply(path, points_ned_m)


def _load_airsim(root: Path) -> Any:
    sys.path.insert(0, str(root))
    from colosseum import MultirotorClient

    return MultirotorClient()


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect YOPO-Simple RGB-D scene snapshots from AirSim.")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--obstacle-ply", type=Path)
    parser.add_argument("--export-static-meshes", action="store_true")
    parser.add_argument(
        "--person-positions",
        type=Path,
        help="UE PersonSpawner generated_people.json; its collision points are added to tree.ply",
    )
    parser.add_argument("--airsim-root", required=True, type=Path)
    parser.add_argument("--scene-id", default="0001")
    parser.add_argument("--count", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--x-min", type=float, default=-30.0)
    parser.add_argument("--x-max", type=float, default=30.0)
    parser.add_argument("--y-min", type=float, default=-30.0)
    parser.add_argument("--y-max", type=float, default=30.0)
    parser.add_argument("--altitude", type=float, default=1.6)
    parser.add_argument("--prompt", default="person")
    parser.add_argument("--color-order", choices=("rgb", "bgr"), default="bgr")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if bool(args.obstacle_ply) == args.export_static_meshes:
        parser.error("pass exactly one of --obstacle-ply or --export-static-meshes")
    client = _load_airsim(args.airsim_root)
    client.confirmConnection()
    config = CaptureConfig(color_order=args.color_order)
    poses = PoseSampler(
        (args.x_min, args.x_max), (args.y_min, args.y_max), args.altitude, args.seed
    ).sample(args.count)
    collector = AirSimSnapshotCollector(client, config)
    output = args.output / f"Scene_{args.scene_id}"
    if output.exists() and any(output.iterdir()) and not args.overwrite:
        raise FileExistsError(f"scene directory is not empty: {output}")
    obstacle_ply = args.obstacle_ply
    temporary_obstacle: Path | None = None
    if args.export_static_meshes:
        args.output.mkdir(parents=True, exist_ok=True)
        temporary_obstacle = args.output / f".Scene_{args.scene_id}.static.ply"
        obstacle_ply = export_static_mesh_point_cloud(client, temporary_obstacle)
    try:
        collector.collect_scene(
            output,
            poses,
            obstacle_ply,
            target_prompt=args.prompt,
            person_positions=args.person_positions,
            overwrite=args.overwrite,
        )
    finally:
        if temporary_obstacle is not None:
            temporary_obstacle.unlink(missing_ok=True)
    print(f"collected {args.count} frames to {output}")


if __name__ == "__main__":
    main()
