from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from torch.utils.data import Dataset

from config.config import cfg
from data.coordinates import world_to_body_flu
from data.route_contract import (
    RouteQualityFlag,
    RouteTable,
    load_route_table,
    sample_route_bubbles,
)


os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")


@dataclass(frozen=True)
class SceneData:
    path: Path
    frames: dict[int, dict[str, Any]]
    routes: RouteTable
    map_id: int


def _load_toml(path: Path) -> dict[str, Any]:
    try:
        import tomllib
    except ImportError:
        try:
            import tomli as tomllib
        except ImportError:
            try:
                import rtoml
            except ImportError as error:
                raise RuntimeError(
                    "tomllib, tomli, or rtoml is required to read ScaleNav scenes"
                ) from error
            return dict(rtoml.load(path))
    with path.open("rb") as stream:
        return dict(tomllib.load(stream))


def _rotation_wxyz(quaternion: Sequence[float]) -> np.ndarray:
    values = np.asarray(quaternion, dtype=np.float32)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("orientationWxyz must contain four finite values")
    w, x, y, z = values
    return Rotation.from_quat([x, y, z, w]).as_matrix().astype(np.float32)


class YOPODataset(Dataset):
    """Route-conditioned ScaleNav dataset with separate model/loss routes."""

    def __init__(
        self,
        mode: str = "train",
        *,
        data_root: str | Path | None = None,
        validation_ratio: float = 0.1,
        seed: int = 0,
    ) -> None:
        super().__init__()
        if mode not in {"train", "valid", "all"}:
            raise ValueError("mode must be 'train', 'valid', or 'all'")
        if not 0.0 <= validation_ratio < 1.0:
            raise ValueError("validation_ratio must be in [0, 1)")
        base_dir = Path(__file__).resolve().parent.parent
        self.data_root = Path(data_root) if data_root is not None else base_dir / cfg["dataset_path"]
        self.height = int(cfg["image_height"])
        self.width = int(cfg["image_width"])
        self.vel_max = float(cfg["vel_max_train"])
        self.acc_max = float(cfg["acc_max_train"])
        self.vx_lognorm_mean = float(np.log(1.0 - cfg["vx_mean_unit"]))
        self.vx_lognorm_sigma = float(np.log(cfg["vx_std_unit"]))
        self.v_mean = np.asarray(
            [cfg["vx_mean_unit"], cfg["vy_mean_unit"], cfg["vz_mean_unit"]], dtype=np.float32
        )
        self.v_std = np.asarray(
            [cfg["vx_std_unit"], cfg["vy_std_unit"], cfg["vz_std_unit"]], dtype=np.float32
        )
        self.a_mean = np.asarray(
            [cfg["ax_mean_unit"], cfg["ay_mean_unit"], cfg["az_mean_unit"]], dtype=np.float32
        )
        self.a_std = np.asarray(
            [cfg["ax_std_unit"], cfg["ay_std_unit"], cfg["az_std_unit"]], dtype=np.float32
        )
        self.anchors = np.asarray(cfg["route_anchor_distances_m"], dtype=np.float32)
        if len(self.anchors) != int(cfg["route_bubble_count"]):
            raise ValueError("route_anchor_distances_m must match route_bubble_count")
        self.clearance_clip_m = float(cfg["route_clearance_clip_m"])
        self.mode = mode
        self.seed = int(seed)
        self.split_strategy = "all"
        self.scenes = self._load_scenes()
        self.obstacle_paths = [scene.path / "tree.ply" for scene in self.scenes]
        all_samples = self._valid_samples()
        self.samples = self._split_samples(all_samples, mode, validation_ratio)
        if not self.samples:
            raise ValueError(f"no {mode} route samples under {self.data_root}")

    def _load_scenes(self) -> list[SceneData]:
        scene_paths = sorted(path for path in self.data_root.glob("Scene_*") if path.is_dir())
        if not scene_paths:
            raise FileNotFoundError(f"no Scene_* directories under {self.data_root}")
        scenes: list[SceneData] = []
        for map_id, scene_path in enumerate(scene_paths):
            document = _load_toml(scene_path / "data.toml")
            if document.get("worldFrame") != "world_enu" or document.get("bodyFrame") != "body_flu":
                raise ValueError(f"{scene_path} is not world_enu/body_flu")
            records = document.get("dataArray", [])
            frames = {int(record["frameIndex"]): record for record in records}
            if len(frames) != len(records):
                raise ValueError(f"duplicate frameIndex in {scene_path / 'data.toml'}")
            routes = load_route_table(scene_path / "routes.npz", frame_count=len(records))
            scenes.append(SceneData(scene_path, frames, routes, map_id))
        return scenes

    def _valid_samples(self) -> list[tuple[int, int]]:
        samples: list[tuple[int, int]] = []
        for scene_index, scene in enumerate(self.scenes):
            valid = scene.routes.arrays["route_valid"].astype(bool)
            flags = scene.routes.arrays["route_quality_flags"]
            for route_index in np.flatnonzero(valid & (flags == int(RouteQualityFlag.NONE))):
                samples.append((scene_index, int(route_index)))
        return samples

    def _split_samples(
        self,
        samples: list[tuple[int, int]],
        mode: str,
        validation_ratio: float,
    ) -> list[tuple[int, int]]:
        if mode == "all" or validation_ratio == 0.0:
            return samples
        scene_ids = sorted({scene_index for scene_index, _ in samples})
        self.split_strategy = "frame_group_holdout"
        validation_frames: set[tuple[int, int]] = set()
        rng = np.random.default_rng(self.seed)
        for scene_index in scene_ids:
            route_indices = [route for scene, route in samples if scene == scene_index]
            frame_indices = sorted(
                {
                    int(self.scenes[scene_index].routes.arrays["frame_index"][route])
                    for route in route_indices
                }
            )
            if len(frame_indices) < 2:
                continue
            validation_count = min(
                len(frame_indices) - 1,
                max(1, int(round(len(frame_indices) * validation_ratio))),
            )
            selected_frames = rng.permutation(frame_indices)[:validation_count]
            validation_frames.update((scene_index, int(frame)) for frame in selected_frames)

        def is_valid(sample: tuple[int, int]) -> bool:
            scene_index, route_index = sample
            frame_index = int(
                self.scenes[scene_index].routes.arrays["frame_index"][route_index]
            )
            return (scene_index, frame_index) in validation_frames

        if not validation_frames:
            validation_count = max(1, int(round(len(samples) * validation_ratio)))
            validation_samples = set(samples[-validation_count:])

            def is_valid(sample: tuple[int, int]) -> bool:
                return sample in validation_samples

        selected = [sample for sample in samples if is_valid(sample) == (mode == "valid")]
        return selected or samples

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, item: int) -> dict[str, torch.Tensor]:
        scene_index, route_index = self.samples[item]
        scene = self.scenes[scene_index]
        arrays = scene.routes.arrays
        frame_index = int(arrays["frame_index"][route_index])
        frame = scene.frames[frame_index]
        depth = self._read_depth(scene.path / "Textures" / str(frame["depthFileName"]), frame)
        position = np.asarray(frame["posStart"], dtype=np.float32)
        rotation = _rotation_wxyz(frame["orientationWxyz"])
        path_world, _, path_radius = scene.routes.path(route_index)
        initial_segments = np.diff(path_world, axis=0)
        initial_lengths = np.linalg.norm(initial_segments, axis=1)
        nonzero = np.flatnonzero(initial_lengths > 1.0e-5)
        if len(nonzero) == 0:
            raise ValueError(f"route {route_index} has no initial direction")
        tangent_world = initial_segments[nonzero[0]] / initial_lengths[nonzero[0]]
        tangent_body = rotation.T @ tangent_world
        motion_rng = (
            np.random
            if self.mode == "train"
            else np.random.default_rng(
                np.random.SeedSequence((self.seed, scene_index, route_index))
            )
        )
        motion = self._random_motion(motion_rng, forward_direction=tangent_body)
        goal_field = (
            "local_subgoal_world"
            if "local_subgoal_world" in arrays
            else "frontier_goal_world"
        )
        frontier_world = arrays[goal_field][route_index].astype(np.float32, copy=True)
        frontier_body = world_to_body_flu(frontier_world, position, rotation).astype(np.float32)

        centers_world, conservative_radius, sample_distances = sample_route_bubbles(
            path_world, path_radius, self.anchors
        )
        centers_body = world_to_body_flu(centers_world, position, rotation)
        normalized_centers = centers_body / float(cfg["goal_length"])
        normalized_radius = np.clip(conservative_radius, 0.0, self.clearance_clip_m) / self.clearance_clip_m
        route_bubbles = np.concatenate(
            (normalized_centers, normalized_radius[:, None]), axis=1
        ).astype(np.float32)
        return {
            "depth": torch.from_numpy(depth),
            "position_world": torch.from_numpy(position),
            "rotation_world_body": torch.from_numpy(rotation),
            "motion_body": torch.from_numpy(motion),
            "frontier_body": torch.from_numpy(frontier_body),
            "frontier_world": torch.from_numpy(frontier_world),
            "route_bubbles": torch.from_numpy(route_bubbles),
            "route_points_world": torch.from_numpy(centers_world.astype(np.float32)),
            "route_radii_world": torch.from_numpy(conservative_radius.astype(np.float32)),
            "map_id": torch.tensor(scene.map_id, dtype=torch.long),
        }

    def _read_depth(self, path: Path, frame: dict[str, Any]) -> np.ndarray:
        depth = cv2.imread(str(path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
        if depth is None or depth.ndim != 2:
            raise ValueError(f"unable to read depth image: {path}")
        maximum = float(frame.get("depthMaxMeters", 20.0))
        if maximum <= 0.0 or not np.isfinite(maximum):
            raise ValueError(f"invalid depthMaxMeters for {path}")
        depth = cv2.resize(
            depth.astype(np.float32), (self.width, self.height), interpolation=cv2.INTER_NEAREST
        )
        depth = np.nan_to_num(depth, nan=maximum, posinf=maximum, neginf=0.0)
        return np.expand_dims(np.clip(depth, 0.0, maximum) / maximum, axis=0).astype(np.float32)

    def _random_motion(
        self,
        rng: Any = np.random,
        *,
        forward_direction: np.ndarray | None = None,
    ) -> np.ndarray:
        while True:
            velocity = self.vel_max * (self.v_mean + self.v_std * rng.standard_normal(3))
            forward = self.vel_max * rng.lognormal(
                mean=self.vx_lognorm_mean, sigma=self.vx_lognorm_sigma
            )
            velocity[0] = -forward + 1.2 * self.vel_max
            if np.linalg.norm(velocity) < 1.2 * self.vel_max:
                break
        if forward_direction is not None:
            tangent = np.asarray(forward_direction, dtype=np.float32)
            tangent /= max(float(np.linalg.norm(tangent)), 1.0e-6)
            parallel = float(np.dot(velocity, tangent))
            velocity = velocity + (max(abs(parallel), 0.5) - parallel) * tangent
        while True:
            acceleration = self.acc_max * (
                self.a_mean + self.a_std * rng.standard_normal(3)
            )
            if np.linalg.norm(acceleration) < 1.2 * self.acc_max:
                break
        return np.concatenate((velocity, acceleration)).astype(np.float32)
