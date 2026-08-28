"""Route-conditioned YOPO network used by the offline trainer."""

from __future__ import annotations

import torch
from torch import nn

from config.config import cfg
from policy.models.backbone import YopoBackbone
from policy.models.head import YopoHead
from policy.state_transform import StateTransform


class YopoNetwork(nn.Module):
    def __init__(self, output_dim: int = 10, hidden_state: int = 64) -> None:
        super().__init__()
        self.route_bubble_count = int(cfg["route_bubble_count"])
        self.state_transform = StateTransform()
        observation_dim = 9 + self.route_bubble_count * 5
        self.image_backbone = YopoBackbone(hidden_state)
        self.yopo_head = YopoHead(hidden_state + observation_dim, output_dim)
        with torch.no_grad():
            first_layer = self.yopo_head.model[0]
            route_start = hidden_state + 9
            nn.init.normal_(first_layer.weight[:, route_start:], mean=0.0, std=1.0e-3)

    def forward(
        self,
        depth: torch.Tensor,
        motion_body: torch.Tensor,
        frontier_body: torch.Tensor,
        route_bubbles: torch.Tensor,
        route_mask: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if motion_body.ndim != 2 or motion_body.shape[1] != 6:
            raise ValueError("motion_body must have shape [B, 6]")
        if frontier_body.shape != (motion_body.shape[0], 3):
            raise ValueError("frontier_body must have shape [B, 3]")
        if route_bubbles.shape != (motion_body.shape[0], self.route_bubble_count, 4):
            raise ValueError(
                f"route_bubbles must have shape [B, {self.route_bubble_count}, 4]"
            )
        observation = torch.cat((motion_body, frontier_body), dim=1).clone()
        observation = self.state_transform.normalize_obs(observation)
        observation_features = self.state_transform.prepare_input(observation)
        route_features = self.state_transform.prepare_route_input(route_bubbles, route_mask)
        depth_features = self.image_backbone(depth)
        if depth_features.shape[-2:] != observation_features.shape[-2:]:
            raise ValueError(
                "depth feature grid does not match the configured primitive grid: "
                f"{depth_features.shape[-2:]} vs {observation_features.shape[-2:]}"
            )
        # Preserve the original YOPO-Simple head contract: observation
        # channels first, depth channels second.  Route channels are appended
        # after the legacy 73 channels so old checkpoints remain compatible.
        features = torch.cat((observation_features, depth_features, route_features), dim=1)
        output = self.yopo_head(features)
        endstate_prediction = torch.tanh(output[:, :9])
        score = torch.nn.functional.softplus(output[:, 9])
        endstate = self.state_transform.pred_to_endstate(endstate_prediction)
        return endstate, score

    def load_yopo_simple_state_dict(self, state_dict: dict[str, torch.Tensor]) -> None:
        """Load the old backbone and compatible head slices into the larger V1 head."""
        current = self.state_dict()
        with torch.no_grad():
            for name, value in state_dict.items():
                if name not in current:
                    continue
                target = current[name]
                if target.shape == value.shape:
                    target.copy_(value)
                    continue
                if name == "yopo_head.model.0.weight" and value.ndim == 4:
                    # YOPO-Simple concatenates observation(9) then depth(64).
                    # Route channels are appended after these legacy channels.
                    if value.shape[1] == 73 and target.shape[1] >= 73:
                        target[:, :73].copy_(value)
                    else:
                        channels = min(value.shape[1], target.shape[1])
                        target[:, :channels].copy_(value[:, :channels])
        self.load_state_dict(current)
