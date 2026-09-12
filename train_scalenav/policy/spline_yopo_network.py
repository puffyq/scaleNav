from __future__ import annotations

import math

import torch
from scipy.spatial.transform import Rotation
from torch import nn

from config.config import cfg
from policy.models.backbone import YopoBackbone
from policy.models.head import YopoHead


class SplineYopoNetwork(nn.Module):
    FEATURE_ORDER = "yopo_simple_plus_primitive_frame_route_bubbles_spline_pva_v2"
    LEGACY_FEATURE_ORDER = "yopo_simple_plus_primitive_frame_route_bubbles_spline_v1"

    def __init__(
        self,
        *,
        control_point_count: int = 12,
        hidden_state: int = 64,
    ) -> None:
        super().__init__()
        self.control_point_count = int(control_point_count)
        self.free_control_point_count = self.control_point_count - 3
        self.route_bubble_count = int(cfg["route_bubble_count"])
        self.vertical_num = int(cfg["vertical_num"])
        self.horizon_num = int(cfg["horizon_num"])
        self.traj_num = self.vertical_num * self.horizon_num * int(cfg["radio_num"])
        self.vel_max = float(cfg["vel_max_train"])
        self.acc_max = float(cfg["acc_max_train"])
        self.goal_length = float(cfg["goal_length"])
        output_dim = 3 * self.free_control_point_count + 1

        self.image_backbone = YopoBackbone(hidden_state)
        self.yopo_head = YopoHead(
            hidden_state + 9 + self.route_bubble_count * 4, output_dim
        )
        horizontal_step = math.radians(float(cfg["horizon_camera_fov"])) / self.horizon_num
        vertical_step = math.radians(float(cfg["vertical_camera_fov"])) / self.vertical_num
        angles, rotations = [], []
        for vertical in range(self.vertical_num):
            for horizontal in range(self.horizon_num):
                yaw = -horizontal_step * (self.horizon_num - 1) / 2 + horizontal * horizontal_step
                pitch = -vertical_step * (self.vertical_num - 1) / 2 + vertical * vertical_step
                angles.append((yaw, pitch))
                rotations.append(
                    torch.as_tensor(
                        Rotation.from_euler("ZYX", [yaw, -pitch, 0.0]).as_matrix(),
                        dtype=torch.float32,
                    )
                )
        self.register_buffer("lattice_angles", torch.tensor(angles, dtype=torch.float32))
        self.register_buffer("lattice_rotations", torch.stack(rotations))
        progress = torch.linspace(
            3.0 / (self.control_point_count - 1),
            1.0,
            self.free_control_point_count,
        ) * self.goal_length
        baseline = torch.zeros((self.free_control_point_count, 3))
        baseline[:, 0] = progress
        self.register_buffer("control_baseline", baseline)
        self.register_buffer("control_residual_scale", torch.tensor((3.0, 4.0, 2.5)))

    def _prepare_observation(
        self, motion_body: torch.Tensor, goal_body: torch.Tensor
    ) -> torch.Tensor:
        observation = torch.cat((motion_body, goal_body), dim=1).clone()
        observation[:, :3] /= self.vel_max
        observation[:, 3:6] /= self.acc_max
        goal_norm = observation[:, 6:9].norm(dim=1, keepdim=True)
        observation[:, 6:9] /= goal_norm.clamp(min=self.goal_length)
        vectors = observation.view(observation.shape[0], 3, 3)
        rotations = self.lattice_rotations.flip(0)
        transformed = torch.matmul(vectors[:, None], rotations[None])
        return transformed.reshape(observation.shape[0], self.traj_num, 9).permute(
            0, 2, 1
        ).reshape(observation.shape[0], 9, self.vertical_num, self.horizon_num)

    def _prepare_route(self, route_bubbles: torch.Tensor) -> torch.Tensor:
        centers = route_bubbles[:, :, :3]
        radii = route_bubbles[:, :, 3:4]
        rotations = self.lattice_rotations.flip(0)
        centers_local = torch.matmul(centers[:, None], rotations[None])
        radii_local = radii[:, None].expand(-1, self.traj_num, -1, -1)
        features = torch.cat((centers_local, radii_local), dim=-1)
        return features.reshape(features.shape[0], self.traj_num, -1).permute(
            0, 2, 1
        ).reshape(
            features.shape[0],
            self.route_bubble_count * 4,
            self.vertical_num,
            self.horizon_num,
        )

    def _decode_controls(self, prediction: torch.Tensor) -> torch.Tensor:
        batch = prediction.shape[0]
        local = prediction.permute(0, 2, 3, 1).reshape(
            batch, self.traj_num, self.free_control_point_count, 3
        )
        local = self.control_baseline[None, None] + torch.tanh(local) * self.control_residual_scale
        rotations = self.lattice_rotations.flip(0)
        body = torch.matmul(rotations[None, :, None], local.unsqueeze(-1)).squeeze(-1)
        return body

    def forward(
        self,
        depth: torch.Tensor,
        motion_body: torch.Tensor,
        frontier_body: torch.Tensor,
        route_bubbles: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        expected = (motion_body.shape[0], self.route_bubble_count, 4)
        if tuple(route_bubbles.shape) != expected:
            raise ValueError(f"route_bubbles must have shape {expected}")
        features = torch.cat(
            (
                self._prepare_observation(motion_body, frontier_body),
                self.image_backbone(depth),
                self._prepare_route(route_bubbles),
            ),
            dim=1,
        )
        output = self.yopo_head(features)
        controls = self._decode_controls(output[:, :-1])
        score = torch.nn.functional.softplus(output[:, -1])
        return controls, score

    def load_yopo_simple_state_dict(self, state_dict: dict[str, torch.Tensor]) -> None:
        source = state_dict.get("model_state_dict", state_dict)
        current = self.state_dict()
        for name, value in source.items():
            if name not in current:
                continue
            if name == "yopo_head.model.0.weight" and value.shape[1] == 73:
                current[name][:, :73].copy_(value)
            elif name == "yopo_head.model.4.weight" and value.shape[0] == 10:
                current[name][-1:].copy_(value[9:10])
            elif name == "yopo_head.model.4.bias" and value.shape[0] == 10:
                current[name][-1:].copy_(value[9:10])
            elif current[name].shape == value.shape:
                current[name].copy_(value)
        self.load_state_dict(current)

    def load_spline_checkpoint(self, checkpoint: dict) -> None:
        feature_order = checkpoint.get("feature_order")
        if feature_order != self.FEATURE_ORDER:
            raise ValueError(f"unsupported spline feature order: {feature_order!r}")
        if int(checkpoint.get("control_point_count", -1)) != self.control_point_count:
            raise ValueError("spline control-point count mismatch")
        self.load_state_dict(checkpoint["model_state_dict"], strict=True)

    def load_training_checkpoint(self, checkpoint: dict) -> None:
        feature_order = checkpoint.get("feature_order") if isinstance(checkpoint, dict) else None
        if feature_order == self.FEATURE_ORDER:
            self.load_spline_checkpoint(checkpoint)
            return
        if feature_order != self.LEGACY_FEATURE_ORDER:
            self.load_yopo_simple_state_dict(checkpoint)
            return

        if int(checkpoint.get("control_point_count", -1)) != self.control_point_count:
            raise ValueError("legacy spline control-point count mismatch")
        source = checkpoint["model_state_dict"]
        current = self.state_dict()
        final_weight = "yopo_head.model.4.weight"
        final_bias = "yopo_head.model.4.bias"
        for name, value in source.items():
            if name in current and current[name].shape == value.shape:
                current[name].copy_(value)
        # v1 emitted C2..C(N-1),score. P/V/A fixes C2, so retain C3..C(N-1),score.
        current[final_weight][:-1].copy_(source[final_weight][3:-1])
        current[final_weight][-1].copy_(source[final_weight][-1])
        current[final_bias][:-1].copy_(source[final_bias][3:-1])
        current[final_bias][-1].copy_(source[final_bias][-1])
        self.load_state_dict(current)
