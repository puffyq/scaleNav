from __future__ import annotations

import torch
from torch import nn
from torch.nn import functional as F

from config.config import cfg
from loss.loss_function import YOPOLoss
from policy.state_transform import rotate_body2world, state_body2world, transform_body2world


def progress_weighted_heatmap_value(
    heatmap_value: torch.Tensor,
    directions: torch.Tensor,
    distance_scale: float,
) -> torch.Tensor:
    """Preserve signed heatmap values while requiring trajectory progress."""
    if distance_scale <= 0.0:
        raise ValueError("distance_scale must be positive")
    distance = torch.linalg.vector_norm(directions, dim=-1)
    progress = (distance / distance_scale).clamp(0.0, 1.0)
    return heatmap_value.clamp(-1.0, 1.0) * progress


class TextYopoGuidanceLoss(nn.Module):
    """YOPO physical costs plus a signed heatmap cost for primitive scores."""

    def __init__(
        self,
        obstacle_paths: list[str],
        semantic_weight: float = 1.2,
        device: torch.device | None = None,
    ) -> None:
        super().__init__()
        self.yopo_loss = YOPOLoss(obstacle_paths, include_goal=True, device=device)

    def forward(
        self,
        prediction: tuple[torch.Tensor, torch.Tensor],
        batch: dict[str, torch.Tensor],
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        endstate, score = prediction
        score_target, components = self.trajectory_costs(endstate, batch)

        trajectory_loss = score_target.mean()
        score_error = F.smooth_l1_loss(
            score, score_target.detach(), reduction="none"
        )
        score_loss = score_error.mean()
        total = trajectory_loss + score_loss

        flat_score = score.flatten(1)
        flat_target = score_target.flatten(1)
        flat_goal = components["goal"].flatten(1)
        flat_distance = components["endpoint_distance"].flatten(1)
        selected = flat_score.detach().argmin(dim=1, keepdim=True)
        oracle = flat_target.detach().argmin(dim=1, keepdim=True)
        selected_cost = flat_target.gather(1, selected).squeeze(1)
        oracle_cost = flat_target.gather(1, oracle).squeeze(1)
        return total, {
            "trajectory": score_target.flatten(1).mean(1).detach(),
            "score": score_error.flatten(1).mean(1).detach(),
            "smooth": components["smooth"].flatten(1).mean(1).detach(),
            "safety": components["safety"].flatten(1).mean(1).detach(),
            "acceleration": components["acceleration"].flatten(1).mean(1).detach(),
            "goal": flat_goal.mean(1).detach(),
            "selected_goal_cost": flat_goal.gather(1, selected).squeeze(1).detach(),
            "oracle_goal_cost": flat_goal.gather(1, oracle).squeeze(1).detach(),
            "selected_total_cost": selected_cost.detach(),
            "oracle_total_cost": oracle_cost.detach(),
            "selected_endpoint_distance": flat_distance.gather(1, selected).squeeze(1).detach(),
            "oracle_endpoint_distance": flat_distance.gather(1, oracle).squeeze(1).detach(),
            "selection_regret": (selected_cost - oracle_cost).detach(),
            "selection_top1": selected.eq(oracle).squeeze(1).float(),
        }

    def trajectory_costs(
        self,
        endstate: torch.Tensor,
        batch: dict[str, torch.Tensor],
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Return the true cost and components for every predicted candidate."""
        batch_size = endstate.shape[0]
        vertical_num = endstate.shape[2]
        horizon_num = endstate.shape[3]
        trajectory_count = vertical_num * horizon_num

        position = batch["position"]
        rotation = batch["rotation"]
        obs = batch["obs"]
        velocity_world = rotate_body2world(rotation, obs[:, :3])
        acceleration_world = rotate_body2world(rotation, obs[:, 3:6])
        start_state_world = torch.stack(
            [position, velocity_world, acceleration_world], dim=1
        )

        endstate_flat = endstate.permute(0, 2, 3, 1).reshape(
            batch_size * trajectory_count, 9
        )
        repeated_position = position.repeat_interleave(trajectory_count, dim=0)
        repeated_rotation = rotation.repeat_interleave(trajectory_count, dim=0)
        repeated_start = start_state_world.repeat_interleave(
            trajectory_count, dim=0
        )
        end_position, end_velocity, end_acceleration = state_body2world(
            repeated_position,
            repeated_rotation,
            endstate_flat[:, :3],
            endstate_flat[:, 3:6],
            endstate_flat[:, 6:9],
        )
        end_state_world = torch.stack(
            [end_position, end_velocity, end_acceleration], dim=1
        )

        goal_world = transform_body2world(
            rotation, position, batch["numeric_goal"]
        ).repeat_interleave(trajectory_count, dim=0)

        smooth, safety, goal, acceleration = self.yopo_loss(
            repeated_start,
            end_state_world,
            goal_world,
            batch["scene_id"],
        )
        physical_cost = smooth + safety + goal + acceleration
        end_directions = endstate[:, :3].permute(0, 2, 3, 1)
        shape = (batch_size, vertical_num, horizon_num)
        smooth = smooth.reshape(shape)
        safety = safety.reshape(shape)
        goal = goal.reshape(shape)
        acceleration = acceleration.reshape(shape)
        physical_cost = physical_cost.reshape(shape)
        score_target = physical_cost
        return score_target, {
            "smooth": smooth,
            "safety": safety,
            "goal": goal,
            "acceleration": acceleration,
            "endpoint_distance": torch.linalg.vector_norm(end_directions, dim=-1),
            "total": score_target,
        }
