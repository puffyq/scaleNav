"""YOPO-Simple with one additional route-bubble input."""
from __future__ import annotations
import math
import torch
from scipy.spatial.transform import Rotation
from torch import nn
from config.config import cfg
from policy.models.backbone import YopoBackbone
from policy.models.head import YopoHead


class YopoNetwork(nn.Module):
    FEATURE_ORDER = "yopo_simple_plus_route_bubbles_v1"

    def __init__(self, output_dim: int = 10, hidden_state: int = 64) -> None:
        super().__init__()
        self.route_bubble_count = int(cfg["route_bubble_count"])
        self.vertical_num = int(cfg["vertical_num"])
        self.horizon_num = int(cfg["horizon_num"])
        self.traj_num = self.vertical_num * self.horizon_num * int(cfg["radio_num"])
        self.vel_max = float(cfg["vel_max_train"])
        self.acc_max = float(cfg["acc_max_train"])
        self.goal_length = float(cfg["goal_length"])
        self.radio_range = float(cfg["radio_range"])
        self.yaw_diff = 0.5 * math.radians(float(cfg["horizon_anchor_fov"]))
        self.pitch_diff = 0.5 * math.radians(float(cfg["vertical_anchor_fov"]))
        self.image_backbone = YopoBackbone(hidden_state)
        self.state_backbone = nn.Sequential()
        self.yopo_head = YopoHead(hidden_state + 9 + self.route_bubble_count * 4, output_dim)
        with torch.no_grad():
            self.yopo_head.model[0].weight[:, hidden_state + 9:].zero_()
        horizontal_step = math.radians(float(cfg["horizon_camera_fov"])) / self.horizon_num
        vertical_step = math.radians(float(cfg["vertical_camera_fov"])) / self.vertical_num
        angles, rotations = [], []
        for vertical in range(self.vertical_num):
            for horizontal in range(self.horizon_num):
                yaw = -horizontal_step * (self.horizon_num - 1) / 2 + horizontal * horizontal_step
                pitch = -vertical_step * (self.vertical_num - 1) / 2 + vertical * vertical_step
                angles.append((yaw, pitch))
                rotations.append(torch.as_tensor(Rotation.from_euler("ZYX", [yaw, -pitch, 0.0]).as_matrix(), dtype=torch.float32))
        self.register_buffer("lattice_angles", torch.tensor(angles, dtype=torch.float32))
        self.register_buffer("lattice_rotations", torch.stack(rotations))

    def _prepare_observation(self, motion_body, goal_body):
        observation = torch.cat((motion_body, goal_body), dim=1).clone()
        observation[:, :3] /= self.vel_max
        observation[:, 3:6] /= self.acc_max
        goal_norm = observation[:, 6:9].norm(dim=1, keepdim=True)
        observation[:, 6:9] /= goal_norm.clamp(min=self.goal_length)
        vectors = observation.view(observation.shape[0], 3, 3)
        rotations = self.lattice_rotations.flip(0)
        transformed = torch.matmul(vectors[:, None].expand(-1, self.traj_num, -1, -1), rotations[None].expand(observation.shape[0], -1, -1, -1))
        return transformed.reshape(observation.shape[0], self.traj_num, 9).permute(0, 2, 1).reshape(observation.shape[0], 9, self.vertical_num, self.horizon_num)

    def _decode(self, prediction):
        batch = prediction.shape[0]
        prediction = prediction.permute(0, 2, 3, 1).reshape(batch, self.traj_num, 9)
        yaw = self.lattice_angles[:, 0].flip(0)[None]
        pitch = self.lattice_angles[:, 1].flip(0)[None]
        rotations = self.lattice_rotations.flip(0)[None]
        delta_yaw = prediction[:, :, 0] * self.yaw_diff
        delta_pitch = prediction[:, :, 1] * self.pitch_diff
        radius = (prediction[:, :, 2] + 1.0) * self.radio_range
        cosine_pitch = torch.cos(pitch + delta_pitch)
        position = torch.stack((cosine_pitch * torch.cos(yaw + delta_yaw) * radius, cosine_pitch * torch.sin(yaw + delta_yaw) * radius, torch.sin(pitch + delta_pitch) * radius), dim=-1)
        velocity = torch.matmul(rotations, (prediction[:, :, 3:6] * self.vel_max).unsqueeze(-1)).squeeze(-1)
        acceleration = torch.matmul(rotations, (prediction[:, :, 6:9] * self.acc_max).unsqueeze(-1)).squeeze(-1)
        return torch.cat((position, velocity, acceleration), dim=-1).permute(0, 2, 1).reshape(batch, 9, self.vertical_num, self.horizon_num)

    def forward(self, depth, motion_body, frontier_body, route_bubbles):
        if motion_body.ndim != 2 or motion_body.shape[1] != 6:
            raise ValueError("motion_body must have shape [B, 6]")
        if frontier_body.shape != (motion_body.shape[0], 3):
            raise ValueError("frontier_body must have shape [B, 3]")
        expected = (motion_body.shape[0], self.route_bubble_count, 4)
        if tuple(route_bubbles.shape) != expected:
            raise ValueError(f"route_bubbles must have shape {expected}")
        route_feature = route_bubbles.reshape(route_bubbles.shape[0], -1, 1, 1).expand(-1, -1, self.vertical_num, self.horizon_num)
        output = self.yopo_head(torch.cat((self._prepare_observation(motion_body, frontier_body), self.image_backbone(depth), route_feature), dim=1))
        return self._decode(torch.tanh(output[:, :9])), torch.nn.functional.softplus(output[:, 9])

    def load_yopo_simple_state_dict(self, state_dict):
        current = self.state_dict()
        for name, value in state_dict.items():
            if name not in current:
                continue
            if name == "yopo_head.model.0.weight" and value.shape[1] == 73:
                current[name][:, :73].copy_(value)
            elif current[name].shape == value.shape:
                current[name].copy_(value)
        self.load_state_dict(current)

    def load_route_checkpoint(self, checkpoint):
        feature_order = checkpoint.get("feature_order") if isinstance(checkpoint, dict) else None
        if feature_order not in (None, self.FEATURE_ORDER):
            raise ValueError(f"unsupported Route-YOPO feature order: {feature_order!r}")
        state_dict = checkpoint.get("model_state_dict", checkpoint)
        self.load_state_dict(state_dict, strict=True)
        return str(feature_order or self.FEATURE_ORDER)
