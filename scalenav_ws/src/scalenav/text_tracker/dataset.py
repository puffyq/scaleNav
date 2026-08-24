from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import cv2
import numpy as np
import rtoml
import torch
from scipy.spatial.transform import Rotation
from torch.utils.data import Dataset

from config.config import cfg

from .heatmap import heatmap_peak_body_goal, pearl_similarity


# Snapshot poses and obstacle clouds are AirSim local-NED (body FRD). The
# YOPO state contract is body-FLU, so convert right/down to left/up at the
# world-to-body boundary while keeping the world cloud in NED coordinates.
_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0]).astype(np.float32)


class TextYopoDataset(Dataset):
    """Depth frames with numeric-search or PEARL-approach heatmaps."""

    def __init__(
        self,
        data_root: str,
        image_size: tuple[int, int] | None = None,
        velocity_max: float = 6.0,
        seed: int = 0,
        approach_probability: float = 1.0,
        pearl_enter_threshold: float = 0.08,
        heatmap_sigma_deg: float = 7.5,
        fixed_search_goal_body: tuple[float, float, float] | None = None,
    ) -> None:
        if not 0.0 <= approach_probability <= 1.0:
            raise ValueError("approach_probability must be between 0 and 1")
        if not 0.0 <= pearl_enter_threshold <= 1.0:
            raise ValueError("pearl_enter_threshold must be between 0 and 1")
        self.data_root = Path(data_root)
        self.image_size = image_size or (
            int(cfg["image_width"]),
            int(cfg["image_height"]),
        )
        self.velocity_max = velocity_max
        self.seed = seed
        self.approach_probability = approach_probability
        self.pearl_enter_threshold = pearl_enter_threshold
        self.heatmap_sigma_deg = heatmap_sigma_deg
        self.fixed_search_goal_body = None
        if fixed_search_goal_body is not None:
            fixed_goal = np.asarray(fixed_search_goal_body, dtype=np.float32)
            if fixed_goal.shape != (3,) or not np.isfinite(fixed_goal).all():
                raise ValueError("fixed_search_goal_body must contain three finite values")
            self.fixed_search_goal_body = fixed_goal
        self.horizontal_only = int(cfg["vertical_num"]) == 1
        self.records: list[dict] = []
        self.scene_obstacles: list[str] = []
        self.semantic_record_count = 0
        self.visible_record_count = 0
        self._load_records()

    def _load_records(self) -> None:
        for scene_dir in sorted(self.data_root.glob("Scene_*")):
            toml_path = scene_dir / "data.toml"
            obstacle_path = scene_dir / "tree.ply"
            if not toml_path.is_file() or not obstacle_path.is_file():
                continue

            texture_dir = scene_dir / "Textures"
            if not texture_dir.is_dir():
                continue
            with toml_path.open("r", encoding="utf-8") as file:
                document = rtoml.load(file)
            horizontal_fov = float(
                document.get("depthCameraHorizontalFOV", cfg["horizon_camera_fov"])
            )

            scene_records = []
            for item in document.get("dataArray", []):
                depth_name = item.get("depthFileName")
                if not depth_name:
                    continue
                index = Path(depth_name).stem.removeprefix("depth_")
                semantic_candidate = texture_dir / f"semantic_pearl_{index}.npy"
                semantic_path = (
                    semantic_candidate if semantic_candidate.is_file() else None
                )
                depth_path = texture_dir / depth_name
                if not depth_path.is_file():
                    continue
                scene_records.append(
                    {
                        "depth_path": str(depth_path),
                        "semantic_path": (
                            str(semantic_path) if semantic_path is not None else None
                        ),
                        "horizontal_fov": horizontal_fov,
                        "metadata": item,
                        "target_visible": False,
                        "pearl_confidence": 0.0,
                    }
                )
                if semantic_path is not None:
                    self.semantic_record_count += 1
                    pearl_map = np.load(semantic_path).astype(np.float32)
                    confidence = pearl_similarity(pearl_map)
                    explicit_visible = item.get("targetVisible")
                    visible = (
                        bool(explicit_visible)
                        if explicit_visible is not None
                        else confidence >= self.pearl_enter_threshold
                    )
                    scene_records[-1]["pearl_confidence"] = confidence
                    scene_records[-1]["target_visible"] = visible
                    self.visible_record_count += int(visible)

            if scene_records:
                scene_id = len(self.scene_obstacles)
                self.scene_obstacles.append(str(obstacle_path))
                for record in scene_records:
                    record["scene_id"] = scene_id
                self.records.extend(scene_records)

        if not self.records:
            raise FileNotFoundError(
                f"No depth samples with data.toml and tree.ply found under {self.data_root}"
            )

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        record = self.records[index]
        metadata = record["metadata"]

        depth = cv2.imread(
            record["depth_path"], cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH
        )
        if depth is None:
            raise FileNotFoundError(record["depth_path"])
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        depth_max_m = float(record["metadata"].get("depthMaxMeters", 1.0))
        if not np.isfinite(depth_max_m) or depth_max_m <= 0.0:
            raise ValueError("depthMaxMeters must be finite and positive")
        if depth_max_m != 1.0:
            depth = depth.astype(np.float32) / depth_max_m
        depth = cv2.resize(depth, self.image_size, interpolation=cv2.INTER_AREA)
        depth = np.nan_to_num(depth, nan=1.0, posinf=1.0, neginf=0.0)
        depth = np.clip(depth.astype(np.float32), 0.0, 1.0)

        rng = np.random.default_rng(self.seed + index * 104729)
        velocity = np.array(
            [
                rng.uniform(0.0, self.velocity_max),
                np.clip(rng.normal(0.0, 0.15 * self.velocity_max),
                        -self.velocity_max, self.velocity_max),
                0.0,
            ],
            dtype=np.float32,
        )
        acceleration = np.array(
            [
                np.clip(rng.normal(0.0, 0.15 * self.velocity_max),
                        -self.velocity_max, self.velocity_max),
                np.clip(rng.normal(0.0, 0.15 * self.velocity_max),
                        -self.velocity_max, self.velocity_max),
                0.0,
            ],
            dtype=np.float32,
        )

        goal_distance_scale = float(cfg["goal_length"])
        pitch = np.deg2rad(rng.normal(0.0, float(cfg["goal_pitch_std"])))
        yaw = np.deg2rad(rng.normal(0.0, float(cfg["goal_yaw_std"])))
        goal_body = goal_distance_scale * np.array(
            [
                np.cos(yaw) * np.cos(pitch),
                np.sin(yaw) * np.cos(pitch),
                np.sin(pitch),
            ],
            dtype=np.float32,
        )
        nearby = rng.random()
        if nearby < 0.1:
            goal_body *= nearby * 10.0

        target_visible = (
            bool(record["target_visible"])
            and record["semantic_path"] is not None
            and rng.random() < self.approach_probability
        )
        if target_visible:
            pearl_map = np.load(record["semantic_path"]).astype(np.float32)
            goal_body = heatmap_peak_body_goal(
                pearl_map,
                horizontal_fov_deg=float(record["horizontal_fov"]),
                vertical_fov_deg=float(cfg["vertical_camera_fov"]),
                distance=goal_distance_scale,
            )
        elif self.fixed_search_goal_body is not None:
            goal_body = self.fixed_search_goal_body.copy()
        semantic = np.zeros_like(depth, dtype=np.float32)
        if record["semantic_path"] is not None:
            semantic = np.load(record["semantic_path"]).astype(np.float32)
            if semantic.shape != depth.shape:
                semantic = cv2.resize(
                    semantic, self.image_size, interpolation=cv2.INTER_LINEAR
                )
            semantic = np.nan_to_num(
                semantic, nan=0.0, posinf=1.0, neginf=0.0
            ).astype(np.float32, copy=False)
        image = torch.from_numpy(np.stack([depth, semantic], axis=0))

        position_raw = metadata.get("posStart", [0.0, 0.0, 0.0])
        if len(position_raw) != 3:
            raise ValueError("posStart must contain NED x, y, and z")
        position = np.asarray(position_raw, dtype=np.float32)
        yaw = np.deg2rad(float(metadata.get("yawStart", 0.0)))
        rotation_ned_frd = Rotation.from_euler("z", yaw).as_matrix().astype(np.float32)
        rotation = rotation_ned_frd @ _FRD_TO_FLU
        obs = np.concatenate([velocity, acceleration, goal_body]).astype(np.float32)
        return {
            "image": image,
            "position": torch.from_numpy(position),
            "rotation": torch.from_numpy(rotation),
            "obs": torch.from_numpy(obs),
            "scene_id": torch.tensor(record["scene_id"], dtype=torch.long),
            "approach": torch.tensor(float(target_visible), dtype=torch.float32),
            "target_visible": torch.tensor(float(target_visible), dtype=torch.float32),
            "pearl_confidence": torch.tensor(
                float(record["pearl_confidence"]), dtype=torch.float32
            ),
            "numeric_goal": torch.from_numpy(goal_body),
        }
