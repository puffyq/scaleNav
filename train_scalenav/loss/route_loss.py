from __future__ import annotations

import torch
import torch.nn as nn

from config.config import cfg


class RouteLoss(nn.Module):
    def __init__(self, coefficient_map: torch.Tensor, eval_points: int = 30) -> None:
        super().__init__()
        self.register_buffer("coefficient_map", coefficient_map.detach().clone())
        self.segment_time = float(cfg["sgm_time"])
        self.eval_points = int(eval_points)
        self.corridor_width_cap_m = float(cfg["route_corridor_width_cap_m"])
        self.progress_target_m = float(cfg["goal_length"])

    def forward(
        self,
        fixed_derivatives: torch.Tensor,
        decision_derivatives: torch.Tensor,
        route_points: torch.Tensor,
        route_radii: torch.Tensor,
        route_mask: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        if route_points.ndim != 3 or route_points.shape[-1] != 3:
            raise ValueError("route_points must have shape [B, M, 3]")
        if route_radii.shape != route_points.shape[:2] or route_mask.shape != route_points.shape[:2]:
            raise ValueError("route radii/mask must have shape [B, M]")
        coefficients = self._coefficients(fixed_derivatives, decision_derivatives)
        times = torch.linspace(
            self.segment_time / self.eval_points,
            self.segment_time,
            self.eval_points,
            device=coefficients.device,
            dtype=coefficients.dtype,
        )
        positions = self._positions(coefficients, times)
        end_velocity = decision_derivatives[:, :, 1]

        segment_start = route_points[:, :-1]
        segment_vector = route_points[:, 1:] - segment_start
        segment_length = segment_vector.norm(dim=-1)
        segment_valid = (route_mask[:, :-1] > 0.5) & (route_mask[:, 1:] > 0.5)
        segment_valid &= segment_length > 1.0e-5
        active = segment_valid.any(dim=1)

        difference = positions[:, :, None, :] - segment_start[:, None, :, :]
        denominator = segment_vector.square().sum(dim=-1).clamp_min(1.0e-8)
        alpha = (
            difference * segment_vector[:, None, :, :]
        ).sum(dim=-1) / denominator[:, None, :]
        alpha = alpha.clamp(0.0, 1.0)
        closest = segment_start[:, None, :, :] + alpha[..., None] * segment_vector[:, None, :, :]
        distance_squared = (positions[:, :, None, :] - closest).square().sum(dim=-1)
        distance_squared = distance_squared.masked_fill(~segment_valid[:, None, :], 1.0e8)
        nearest_distance_squared, nearest_index = distance_squared.min(dim=-1)
        nearest_alpha = torch.gather(alpha, 2, nearest_index[..., None]).squeeze(-1)

        radius_start = route_radii[:, :-1]
        radius_delta = route_radii[:, 1:] - radius_start
        nearest_radius_start = torch.gather(
            radius_start[:, None, :].expand(-1, self.eval_points, -1),
            2,
            nearest_index[..., None],
        ).squeeze(-1)
        nearest_radius_delta = torch.gather(
            radius_delta[:, None, :].expand(-1, self.eval_points, -1),
            2,
            nearest_index[..., None],
        ).squeeze(-1)
        nearest_radius = nearest_radius_start + nearest_alpha * nearest_radius_delta
        nearest_radius = nearest_radius.clamp(min=0.0, max=self.corridor_width_cap_m)
        corridor = torch.relu(nearest_distance_squared.clamp_min(0.0).sqrt() - nearest_radius)
        corridor_cost = corridor.square().mean(dim=1)

        cumulative = torch.cat(
            (torch.zeros_like(segment_length[:, :1]), torch.cumsum(segment_length, dim=1)), dim=1
        )
        end_nearest = nearest_index[:, -1]
        end_alpha = nearest_alpha[:, -1]
        nearest_segment_length = torch.gather(segment_length, 1, end_nearest[:, None]).squeeze(1)
        progress = torch.gather(cumulative[:, :-1], 1, end_nearest[:, None]).squeeze(1)
        progress = progress + end_alpha * nearest_segment_length
        route_length = (segment_length * segment_valid.to(segment_length.dtype)).sum(dim=1)
        target = torch.minimum(route_length, torch.full_like(route_length, self.progress_target_m))
        progress_cost = (torch.relu(target - progress) / target.clamp_min(1.0)).square()

        unit_tangent = segment_vector / segment_length[..., None].clamp_min(1.0e-6)
        tangent = torch.gather(
            unit_tangent,
            1,
            end_nearest[:, None, None].expand(-1, 1, 3),
        ).squeeze(1)
        velocity_direction = end_velocity / end_velocity.norm(dim=-1, keepdim=True).clamp_min(1.0e-3)
        tangent_cost = 1.0 - (velocity_direction * tangent).sum(dim=-1).clamp(-1.0, 1.0)

        active_weight = active.to(corridor_cost.dtype)
        return (
            corridor_cost * active_weight,
            progress_cost * active_weight,
            tangent_cost * active_weight,
        )

    def _coefficients(
        self, fixed_derivatives: torch.Tensor, decision_derivatives: torch.Tensor
    ) -> torch.Tensor:
        boundary = torch.cat((fixed_derivatives, decision_derivatives), dim=2)
        # coefficient_map maps [p0,v0,a0,p1,v1,a1] for one axis.
        return torch.einsum("ij,baj->bai", self.coefficient_map, boundary)

    @staticmethod
    def _positions(coefficients: torch.Tensor, times: torch.Tensor) -> torch.Tensor:
        powers = torch.stack(
            (torch.ones_like(times), times, times**2, times**3, times**4, times**5), dim=1
        )
        return torch.einsum("bai,ti->bta", coefficients, powers)
