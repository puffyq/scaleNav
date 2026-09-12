from __future__ import annotations

import numpy as np
import torch
from scipy.interpolate import BSpline
from torch import nn


class ClampedCubicSpline(nn.Module):
    """Differentiable clamped cubic B-spline with fixed initial P/V/A."""

    def __init__(
        self,
        *,
        control_point_count: int,
        duration: float,
        sample_count: int = 30,
    ) -> None:
        super().__init__()
        if control_point_count < 6:
            raise ValueError("a cubic trajectory requires at least six control points")
        if duration <= 0.0 or sample_count < 2:
            raise ValueError("duration and sample_count must be positive")
        self.control_point_count = int(control_point_count)
        self.free_control_point_count = self.control_point_count - 3
        self.duration = float(duration)
        self.sample_count = int(sample_count)

        position, velocity, acceleration, jerk = self.basis(
            self.sample_count, include_start=True
        )
        self.register_buffer("position_basis", torch.from_numpy(position).float())
        self.register_buffer("velocity_basis", torch.from_numpy(velocity).float())
        self.register_buffer("acceleration_basis", torch.from_numpy(acceleration).float())
        self.register_buffer("jerk_basis", torch.from_numpy(jerk).float())

        spline = self._scipy_spline()
        constraint = np.stack(
            (
                spline(0.0),
                spline.derivative(1)(0.0) / self.duration,
                spline.derivative(2)(0.0) / self.duration**2,
            )
        )
        initial_block = constraint[:, :3]
        if np.max(np.abs(constraint[:, 3:])) > 1.0e-10:
            raise RuntimeError("unexpected clamped-spline boundary support")
        self.register_buffer(
            "initial_control_map",
            torch.from_numpy(np.linalg.inv(initial_block)).float(),
        )

    def _scipy_spline(self) -> BSpline:
        degree = 3
        span_count = self.control_point_count - degree
        internal = np.arange(1, span_count, dtype=np.float64) / span_count
        knots = np.concatenate(
            (np.zeros(degree + 1), internal, np.ones(degree + 1))
        )
        return BSpline(knots, np.eye(self.control_point_count), degree)

    def basis(
        self, sample_count: int, *, include_start: bool
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        start = 0.0 if include_start else 1.0 / sample_count
        normalized_time = np.linspace(start, 1.0, sample_count, dtype=np.float64)
        spline = self._scipy_spline()
        return (
            spline(normalized_time),
            spline.derivative(1)(normalized_time) / self.duration,
            spline.derivative(2)(normalized_time) / self.duration**2,
            spline.derivative(3)(normalized_time) / self.duration**3,
        )

    def assemble_controls(
        self,
        start_position: torch.Tensor,
        start_velocity: torch.Tensor,
        start_acceleration: torch.Tensor,
        free_controls: torch.Tensor,
    ) -> torch.Tensor:
        expected = (*start_position.shape[:-1], self.free_control_point_count, 3)
        if not (
            start_position.shape == start_velocity.shape == start_acceleration.shape
            and start_position.shape[-1] == 3
        ):
            raise ValueError("initial position, velocity and acceleration must match [..., 3]")
        if tuple(free_controls.shape) != expected:
            raise ValueError(f"free controls must have shape {expected}")
        boundary = torch.stack(
            (start_position, start_velocity, start_acceleration), dim=-2
        )
        initial = torch.einsum("ij,...jc->...ic", self.initial_control_map, boundary)
        return torch.cat((initial, free_controls), dim=-2)

    @staticmethod
    def sample_with_basis(
        controls: torch.Tensor, basis: torch.Tensor
    ) -> torch.Tensor:
        return torch.einsum("tk,...kc->...tc", basis, controls)

    def forward(
        self, controls: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        return (
            self.sample_with_basis(controls, self.position_basis),
            self.sample_with_basis(controls, self.velocity_basis),
            self.sample_with_basis(controls, self.acceleration_basis),
            self.sample_with_basis(controls, self.jerk_basis),
        )
