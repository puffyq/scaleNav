"""Frozen YOPO-Simple 3x5 baseline used by the paired offline benchmark."""

from __future__ import annotations

import math

import torch
from scipy.spatial.transform import Rotation
from torch import nn

from policy.models.backbone import YopoBackbone
from policy.models.head import YopoHead


class YopoSimpleBaseline(nn.Module):
    """Original YOPO-Simple network and trajectory decoding contract.

    These constants come from the upstream config committed with
    ``saved/YOPO_1/epoch50.pth``.  Keeping them here prevents unrelated local
    edits to the upstream YAML from changing benchmark results.
    """

    image_height = 96
    image_width = 160
    vertical_num = 3
    horizon_num = 5
    trajectory_count = 15
    vel_max = 6.0
    acc_max = 6.0
    goal_length = 10.0
    radio_range = 5.0
    segment_time = 10.0 / 6.0
    horizon_camera_fov_deg = 90.0
    vertical_camera_fov_deg = 60.0
    horizon_anchor_fov_deg = 30.0
    vertical_anchor_fov_deg = 30.0

    def __init__(self) -> None:
        super().__init__()
        self.image_backbone = YopoBackbone(64)
        self.state_backbone = nn.Sequential()
        self.yopo_head = YopoHead(64 + 9, 10)

        horizontal_step = math.radians(self.horizon_camera_fov_deg) / self.horizon_num
        vertical_step = math.radians(self.vertical_camera_fov_deg) / self.vertical_num
        angles: list[tuple[float, float]] = []
        rotations: list[torch.Tensor] = []
        for vertical in range(self.vertical_num):
            for horizontal in range(self.horizon_num):
                yaw = -horizontal_step * (self.horizon_num - 1) / 2 + horizontal * horizontal_step
                pitch = -vertical_step * (self.vertical_num - 1) / 2 + vertical * vertical_step
                angles.append((yaw, pitch))
                matrix = Rotation.from_euler("ZYX", [yaw, -pitch, 0.0]).as_matrix()
                rotations.append(torch.as_tensor(matrix, dtype=torch.float32))
        self.register_buffer("lattice_angles", torch.tensor(angles, dtype=torch.float32))
        self.register_buffer("lattice_rotations", torch.stack(rotations))

    def _prepare_observation(self, motion_body: torch.Tensor, goal_body: torch.Tensor) -> torch.Tensor:
        observation = torch.cat((motion_body, goal_body), dim=1).clone()
        observation[:, :3] /= self.vel_max
        observation[:, 3:6] /= self.acc_max
        goal_norm = observation[:, 6:9].norm(dim=1, keepdim=True)
        observation[:, 6:9] /= goal_norm.clamp(min=self.goal_length)

        batch = observation.shape[0]
        vectors = observation.view(batch, 3, 3)
        rotations = self.lattice_rotations.flip(0)
        transformed = torch.matmul(
            vectors[:, None].expand(batch, self.trajectory_count, 3, 3),
            rotations[None].expand(batch, self.trajectory_count, 3, 3),
        )
        return transformed.reshape(batch, self.trajectory_count, 9).permute(0, 2, 1).reshape(
            batch, 9, self.vertical_num, self.horizon_num
        )

    def _decode(self, prediction: torch.Tensor) -> torch.Tensor:
        batch = prediction.shape[0]
        prediction = prediction.permute(0, 2, 3, 1).reshape(batch, self.trajectory_count, 9)
        yaw = self.lattice_angles[:, 0].flip(0)[None]
        pitch = self.lattice_angles[:, 1].flip(0)[None]
        rotations = self.lattice_rotations.flip(0)[None]
        delta_yaw = prediction[:, :, 0] * (0.5 * math.radians(self.horizon_anchor_fov_deg))
        delta_pitch = prediction[:, :, 1] * (0.5 * math.radians(self.vertical_anchor_fov_deg))
        radius = (prediction[:, :, 2] + 1.0) * self.radio_range
        cosine_pitch = torch.cos(pitch + delta_pitch)
        position = torch.stack(
            (
                cosine_pitch * torch.cos(yaw + delta_yaw) * radius,
                cosine_pitch * torch.sin(yaw + delta_yaw) * radius,
                torch.sin(pitch + delta_pitch) * radius,
            ),
            dim=-1,
        )
        velocity = torch.matmul(
            rotations, (prediction[:, :, 3:6] * self.vel_max).unsqueeze(-1)
        ).squeeze(-1)
        acceleration = torch.matmul(
            rotations, (prediction[:, :, 6:9] * self.acc_max).unsqueeze(-1)
        ).squeeze(-1)
        endstate = torch.cat((position, velocity, acceleration), dim=-1)
        return endstate.permute(0, 2, 1).reshape(
            batch, 9, self.vertical_num, self.horizon_num
        )

    def forward(
        self,
        depth: torch.Tensor,
        motion_body: torch.Tensor,
        goal_body: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        observation = self._prepare_observation(motion_body, goal_body)
        features = torch.cat((observation, self.image_backbone(depth)), dim=1)
        output = self.yopo_head(features)
        return self._decode(torch.tanh(output[:, :9])), torch.nn.functional.softplus(output[:, 9])
