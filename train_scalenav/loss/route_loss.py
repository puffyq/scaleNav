"""Route corridor loss for polynomial YOPO candidates."""
from __future__ import annotations

import torch
from torch import nn

from config.config import cfg


class RouteLoss(nn.Module):
    """Evaluate polynomial candidates against an ordered Bubble corridor.

    The route is a feasible region, not a timed expert trajectory.  When
    radii are supplied, :meth:`forward` returns only the squared violation of
    the Bubble union.  Legacy angle and centerline helpers remain available
    for diagnostics, but are not part of the training objective.
    """

    def __init__(
        self,
        coefficient_map: torch.Tensor,
        eval_points: int | None = None,
        bubble_radius_weight: float | None = None,
    ) -> None:
        super().__init__()
        self.register_buffer("coefficient_map", coefficient_map.detach().clone())
        self.segment_time = float(cfg["sgm_time"])
        self.eval_points = int(eval_points or cfg["safety_eval_points"])
        self.goal_length = float(cfg["goal_length"])
        self.corridor_peak_weight = float(cfg["route_corridor_peak_weight"])
        self.corridor_radius_cap_m = float(cfg["route_corridor_radius_cap_m"])

    def forward(
        self,
        fixed_derivatives: torch.Tensor,
        decision_derivatives: torch.Tensor,
        route_points: torch.Tensor,
        route_radii: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Return corridor barrier when radii are provided.

        Calling without radii preserves the historical angle diagnostic and
        should not be used by the trainer.
        """
        positions = self._sample_positions(fixed_derivatives, decision_derivatives)
        if route_radii is None:
            return self.angle_loss_positions(positions, fixed_derivatives[:, :, 0], route_points)
        return self.corridor_barrier_positions(positions, route_points, route_radii)

    def angle_loss(
        self,
        fixed_derivatives: torch.Tensor,
        decision_derivatives: torch.Tensor,
        route_points: torch.Tensor,
    ) -> torch.Tensor:
        positions = self._sample_positions(fixed_derivatives, decision_derivatives)
        return self.angle_loss_positions(positions, fixed_derivatives[:, :, 0], route_points)

    @staticmethod
    def angle_loss_positions(
        positions: torch.Tensor,
        start_position: torch.Tensor,
        route_points: torch.Tensor,
    ) -> torch.Tensor:
        if positions.ndim != 3 or positions.shape[-1] != 3:
            raise ValueError("positions must have shape [B, T, 3]")
        if start_position.shape != (positions.shape[0], 3):
            raise ValueError("start_position must have shape [B, 3]")
        if route_points.ndim != 3 or route_points.shape[-1] != 3:
            raise ValueError("route_points must have shape [B, M, 3]")
        guide = route_points[:, -1] - start_position
        predicted = positions[:, -1] - start_position
        guide_norm = torch.linalg.vector_norm(guide, dim=-1)
        predicted_norm = torch.linalg.vector_norm(predicted, dim=-1)
        valid = (guide_norm > 1.0e-6) & (predicted_norm > 1.0e-6)
        guide_unit = guide / guide_norm[:, None].clamp_min(1.0e-6)
        predicted_unit = predicted / predicted_norm[:, None].clamp_min(1.0e-6)
        cosine = (guide_unit * predicted_unit).sum(dim=-1).clamp(-1.0, 1.0)
        cross_norm = torch.linalg.vector_norm(
            torch.linalg.cross(guide_unit, predicted_unit, dim=-1), dim=-1
        )
        angle = torch.atan2(cross_norm, cosine)
        near_parallel = cross_norm <= 1.0e-6
        angle = torch.where(
            near_parallel,
            torch.where(
                cosine >= 0.0, torch.zeros_like(angle), angle.new_full(angle.shape, torch.pi)
            ),
            angle,
        )
        return angle * valid.to(positions.dtype)

    def corridor_barrier(
        self,
        fixed_derivatives: torch.Tensor,
        decision_derivatives: torch.Tensor,
        route_points: torch.Tensor,
        route_radii: torch.Tensor,
    ) -> torch.Tensor:
        positions = self._sample_positions(fixed_derivatives, decision_derivatives)
        return self.corridor_barrier_positions(positions, route_points, route_radii)

    def corridor_barrier_positions(
        self,
        positions: torch.Tensor,
        route_points: torch.Tensor,
        route_radii: torch.Tensor,
    ) -> torch.Tensor:
        if positions.ndim != 3 or positions.shape[-1] != 3:
            raise ValueError("positions must have shape [B, T, 3]")
        if route_points.ndim != 3 or route_points.shape[-1] != 3:
            raise ValueError("route_points must have shape [B, M, 3]")
        if route_points.shape[0] != positions.shape[0]:
            raise ValueError("route_points batch must match positions")
        if route_radii.shape != route_points.shape[:2]:
            raise ValueError("route_radii must have shape [B, M]")
        if not torch.isfinite(positions).all() or not torch.isfinite(route_points).all():
            raise ValueError("corridor inputs must be finite")
        if not torch.isfinite(route_radii).all():
            raise ValueError("route radii must be finite")
        radii = route_radii.clamp(min=0.0, max=self.corridor_radius_cap_m)
        signed_distance = torch.linalg.vector_norm(
            positions[:, :, None, :] - route_points[:, None, :, :], dim=-1
        ) - radii[:, None, :]
        violation = torch.relu(signed_distance.min(dim=-1).values)
        return violation.square().mean(dim=1) + self.corridor_peak_weight * violation.square().amax(dim=1)

    def forward_positions_components(
        self,
        positions: torch.Tensor,
        start_position: torch.Tensor,
        route_points: torch.Tensor,
        route_radii: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Return legacy angle diagnostic and optional corridor barrier."""
        angle = self.angle_loss_positions(positions, start_position, route_points)
        if route_radii is None:
            return angle, torch.zeros_like(angle)
        return angle, self.corridor_barrier_positions(positions, route_points, route_radii)

    def centerline_mse(
        self,
        fixed_derivatives: torch.Tensor,
        decision_derivatives: torch.Tensor,
        route_points: torch.Tensor,
    ) -> torch.Tensor:
        """Legacy ordered centerline diagnostic; not used for training."""
        positions = self._sample_positions(fixed_derivatives, decision_derivatives)
        start_position = fixed_derivatives[:, :, 0]
        fractions = torch.linspace(
            1.0 / positions.shape[1], 1.0, positions.shape[1],
            device=positions.device, dtype=positions.dtype,
        )
        route = torch.cat((start_position[:, None, :], route_points), dim=1)
        reference = self._sample_by_arclength(route, fractions, maximum_length=self.goal_length)
        return (positions - reference).square().sum(dim=-1).mean(dim=1)

    def _sample_positions(
        self,
        fixed_derivatives: torch.Tensor,
        decision_derivatives: torch.Tensor,
    ) -> torch.Tensor:
        boundary = torch.cat((fixed_derivatives, decision_derivatives), dim=2)
        coefficients = torch.einsum("ij,baj->bai", self.coefficient_map, boundary)
        times = torch.linspace(
            self.segment_time / self.eval_points, self.segment_time, self.eval_points,
            device=coefficients.device, dtype=coefficients.dtype,
        )
        powers = torch.stack(
            (torch.ones_like(times), times, times**2, times**3, times**4, times**5), dim=1
        )
        return torch.einsum("bai,ti->bta", coefficients, powers)

    @staticmethod
    def _sample_by_arclength(
        polyline: torch.Tensor,
        fractions: torch.Tensor,
        *,
        maximum_length: float | None = None,
    ) -> torch.Tensor:
        lower, upper, alpha, stationary = RouteLoss._arclength_interpolation(
            polyline, fractions, maximum_length=maximum_length
        )
        width = polyline.shape[-1]
        start = torch.gather(polyline, 1, lower[..., None].expand(-1, -1, width))
        end = torch.gather(polyline, 1, upper[..., None].expand(-1, -1, width))
        sampled = start + alpha[..., None] * (end - start)
        return torch.where(stationary[..., None], polyline[:, :1], sampled)

    @staticmethod
    def _arclength_interpolation(
        polyline: torch.Tensor,
        fractions: torch.Tensor,
        *,
        maximum_length: float | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        segment_length = torch.linalg.vector_norm(polyline[:, 1:] - polyline[:, :-1], dim=-1)
        cumulative = torch.cat(
            (torch.zeros_like(segment_length[:, :1]), segment_length.cumsum(dim=1)), dim=1
        )
        total = cumulative[:, -1:]
        if maximum_length is not None:
            total = total.clamp(max=float(maximum_length))
        target = total * fractions[None, :]
        upper = torch.searchsorted(cumulative.contiguous(), target.contiguous(), right=True)
        upper = upper.clamp(min=1, max=polyline.shape[1] - 1)
        lower = upper - 1
        lower_distance = torch.gather(cumulative, 1, lower)
        upper_distance = torch.gather(cumulative, 1, upper)
        alpha = (target - lower_distance) / (upper_distance - lower_distance).clamp_min(1.0e-8)
        stationary = cumulative[:, -1:] <= 1.0e-8
        return lower, upper, alpha, stationary
