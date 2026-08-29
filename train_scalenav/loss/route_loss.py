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
        self.corridor_peak_weight = float(cfg["route_corridor_peak_weight"])
        self.corridor_scale_m = float(cfg["route_corridor_scale_m"])
        self.centerline_weight = float(cfg["wcenterline"])
        self.progress_target_m = float(cfg["goal_length"])
        self.progress_floor_m = float(cfg["route_progress_floor_m"])
        self.progress_error_scale_m = float(cfg["route_progress_error_scale_m"])

    def forward(
        self,
        fixed_derivatives: torch.Tensor,
        decision_derivatives: torch.Tensor,
        route_points: torch.Tensor,
        route_radii: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        if route_points.ndim != 3 or route_points.shape[-1] != 3:
            raise ValueError("route_points must have shape [B, M, 3]")
        if route_radii.shape != route_points.shape[:2]:
            raise ValueError("route radii must have shape [B, M]")
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
        segment_valid = segment_length > 1.0e-5
        active = segment_valid.any(dim=1)
        cumulative = torch.cat(
            (torch.zeros_like(segment_length[:, :1]), torch.cumsum(segment_length, dim=1)), dim=1
        )

        # Treat the union of witness bubbles as a signed distance field. The
        # exponential has the ESDF obstacle-cost shape with the sign reversed:
        # minimizing it pulls a primitive into the safe bubble and keeps a
        # smooth gradient after it has entered the bubble.
        bubble_difference = positions[:, :, None, :] - route_points[:, None, :, :]
        safe_radii = route_radii.clamp(min=0.0, max=self.corridor_width_cap_m)
        bubble_signed_distance = bubble_difference.norm(dim=-1) - safe_radii[:, None, :]
        union_signed_distance = -bubble_signed_distance.min(dim=-1).values
        field_argument = (union_signed_distance / max(self.corridor_scale_m, 1.0e-3)).clamp(-8.0, 8.0)
        corridor_field = torch.exp(-field_argument)
        corridor_cost = corridor_field.mean(dim=1)
        corridor_cost = corridor_cost + self.corridor_peak_weight * corridor_field.max(dim=1).values

        # Keep the original progress/tangent diagnostics for compatibility;
        # the caller fuses `corridor_cost` into the ESDF safety term.
        difference = positions[:, :, None, :] - segment_start[:, None, :, :]
        denominator = segment_vector.square().sum(dim=-1).clamp_min(1.0e-8)
        alpha = (
            difference * segment_vector[:, None, :, :]
        ).sum(dim=-1) / denominator[:, None, :]
        alpha = alpha.clamp(0.0, 1.0)
        closest = segment_start[:, None, :, :] + alpha[..., None] * segment_vector[:, None, :, :]
        distance_squared = (positions[:, :, None, :] - closest).square().sum(dim=-1)
        distance_squared = distance_squared.masked_fill(~segment_valid[:, None, :], 1.0e8)
        nearest_distance = distance_squared.min(dim=-1).values.sqrt()
        # Geometric cross-track error, independent of the polynomial's time
        # parameterization.  The synchronized path MSE below is useful for
        # ordered progress, but this term is what keeps curved trajectories
        # physically close to the witness centerline.
        centerline_cost = nearest_distance.square().mean(dim=1)
        nearest_index = distance_squared.argmin(dim=-1)
        nearest_alpha = torch.gather(alpha, 2, nearest_index[..., None]).squeeze(-1)

        end_nearest = nearest_index[:, -1]
        end_alpha = nearest_alpha[:, -1]
        nearest_segment_length = torch.gather(segment_length, 1, end_nearest[:, None]).squeeze(1)
        progress = torch.gather(cumulative[:, :-1], 1, end_nearest[:, None]).squeeze(1)
        progress = progress + end_alpha * nearest_segment_length
        route_length = (segment_length * segment_valid.to(segment_length.dtype)).sum(dim=1)
        target = torch.minimum(route_length, torch.full_like(route_length, self.progress_target_m))
        # Project the local 10 m subgoal onto the witness by arclength. The
        # existing YOPO evaluation grid supplies the fixed 30 uniformly timed
        # samples; mapping time linearly to witness arclength gives the same
        # ordered geometric target without changing the model's point count.
        desired_progress = target[:, None] * (times[None, :] / self.segment_time)
        ordered_segment = (cumulative[:, None, 1:] <= desired_progress[:, :, None]).sum(dim=-1)
        ordered_segment = ordered_segment.clamp(max=segment_length.shape[1] - 1)
        gather_vec = ordered_segment[..., None, None].expand(-1, -1, 1, 3)
        ordered_start = torch.gather(
            segment_start[:, None].expand(-1, self.eval_points, -1, -1), 2, gather_vec
        ).squeeze(2)
        ordered_vector = torch.gather(
            segment_vector[:, None].expand(-1, self.eval_points, -1, -1), 2, gather_vec
        ).squeeze(2)
        ordered_length = torch.gather(
            segment_length[:, None].expand(-1, self.eval_points, -1),
            2, ordered_segment[..., None],
        ).squeeze(2).clamp_min(1.0e-6)
        ordered_base = torch.gather(
            cumulative[:, None, :-1].expand(-1, self.eval_points, -1),
            2, ordered_segment[..., None],
        ).squeeze(2)
        ordered_alpha = ((desired_progress - ordered_base) / ordered_length).clamp(0.0, 1.0)
        ordered_reference = ordered_start + ordered_alpha[..., None] * ordered_vector
        path_mse = (positions - ordered_reference).square().sum(dim=-1).mean(dim=1)
        # Use a physical deficit scale instead of normalizing by the 10 m
        # target. A 2 m shortfall must remain visible next to ESDF costs.
        progress_cost = (
            torch.relu(target - progress) / max(self.progress_error_scale_m, 1.0e-3)
        ).square()
        progress_floor = torch.minimum(
            route_length, torch.full_like(route_length, self.progress_floor_m)
        )
        progress_floor_cost = torch.relu(progress_floor - progress).square()

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
            centerline_cost * active_weight,
            progress_cost * active_weight,
            progress_floor_cost * active_weight,
            tangent_cost * active_weight,
            path_mse * active_weight,
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
