"""Differentiable triple-integrator MPC constrained by ordered route bubbles.

The route is deliberately not a YOPO input here.  YOPO supplies one terminal
state proposal; the route supplies stagewise bubble constraints to the MPC.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class OrderedBubbleMPCConfig:
    horizon_steps: int = 12
    horizon_time_s: float = 10.0 / 6.0
    max_velocity_mps: float = 8.0
    max_acceleration_mps2: float = 8.0
    max_jerk_mps3: float = 24.0
    center_weight: float = 0.35
    velocity_weight: float = 0.025
    acceleration_weight: float = 0.02
    jerk_weight: float = 0.002
    terminal_position_weight: float = 10.0
    terminal_velocity_weight: float = 0.4
    terminal_acceleration_weight: float = 0.08
    bubble_slack_linear_weight: float = 2.0e3
    bubble_slack_quadratic_weight: float = 2.0e4
    minimum_radius_m: float = 0.05
    maximum_accepted_bubble_violation_m: float = 0.03

    @property
    def dt(self) -> float:
        return self.horizon_time_s / self.horizon_steps


def _polyline_arclength(points: np.ndarray) -> tuple[np.ndarray, float]:
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths, dtype=np.float64)))
    return cumulative, float(cumulative[-1])


def sample_stage_bubbles(
    path_points: np.ndarray,
    safe_radii: np.ndarray,
    *,
    horizon_steps: int,
    travel_distance_m: float,
    horizon_time_s: float | None = None,
    initial_speed_mps: float | None = None,
    terminal_speed_mps: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Resample an ordered safe corridor to dynamically timed MPC stages.

    When endpoint speeds and horizon time are supplied, stage progress follows
    a fifth-order boundary-value polynomial with zero endpoint acceleration.
    This avoids making a low-speed vehicle chase a fictitious constant-speed
    bubble schedule.
    """
    points = np.asarray(path_points, dtype=np.float64)
    radii = np.asarray(safe_radii, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 2:
        raise ValueError("path_points must contain at least two 3D points")
    if radii.shape != (len(points),):
        raise ValueError("safe_radii must contain one radius per path point")
    if not np.isfinite(points).all() or not np.isfinite(radii).all():
        raise ValueError("path points and radii must be finite")
    if horizon_steps < 1:
        raise ValueError("horizon_steps must be positive")
    if travel_distance_m <= 0.0 or not math.isfinite(travel_distance_m):
        raise ValueError("travel_distance_m must be finite and positive")

    cumulative, path_length = _polyline_arclength(points)
    if path_length <= 1.0e-6:
        raise ValueError("path must contain a non-zero segment")
    requested_distance = min(float(travel_distance_m), path_length)
    use_dynamic_timing = all(
        value is not None
        for value in (horizon_time_s, initial_speed_mps, terminal_speed_mps)
    )
    if use_dynamic_timing:
        duration = float(horizon_time_s)
        if duration <= 0.0 or not math.isfinite(duration):
            raise ValueError("horizon_time_s must be finite and positive")
        start_speed = max(0.0, float(initial_speed_mps))
        end_speed = max(0.0, float(terminal_speed_mps))
        boundary_matrix = np.array(
            [
                [duration**3, duration**4, duration**5],
                [3.0 * duration**2, 4.0 * duration**3, 5.0 * duration**4],
                [6.0 * duration, 12.0 * duration**2, 20.0 * duration**3],
            ],
            dtype=np.float64,
        )
        rhs = np.array(
            [requested_distance - start_speed * duration, end_speed - start_speed, 0.0],
            dtype=np.float64,
        )
        high_order = np.linalg.solve(boundary_matrix, rhs)
        coefficients = np.concatenate(([0.0, start_speed, 0.0], high_order))
        times = np.linspace(0.0, duration, horizon_steps + 1, dtype=np.float64)
        stage_distance = sum(coefficients[power] * times**power for power in range(6))
        stage_distance = np.maximum.accumulate(np.clip(stage_distance, 0.0, requested_distance))
        stage_distance[-1] = requested_distance
    else:
        stage_distance = np.linspace(
            0.0, requested_distance, horizon_steps + 1, dtype=np.float64
        )
    centers = np.column_stack(
        [np.interp(stage_distance, cumulative, points[:, axis]) for axis in range(3)]
    )
    stage_radii = np.interp(stage_distance, cumulative, radii)
    return centers, stage_radii


def project_path_progress(point: np.ndarray, path_points: np.ndarray) -> float:
    """Return arc-length progress of the closest point on a polyline."""
    point_array = np.asarray(point, dtype=np.float64)
    points = np.asarray(path_points, dtype=np.float64)
    if point_array.shape != (3,) or points.ndim != 2 or points.shape[1:] != (3,):
        raise ValueError("point/path shapes are invalid")
    if len(points) < 2 or not np.isfinite(point_array).all() or not np.isfinite(points).all():
        raise ValueError("point and path must be finite with at least two path points")
    segments = points[1:] - points[:-1]
    squared_length = np.maximum(np.sum(segments * segments, axis=1), 1.0e-12)
    alpha = np.clip(
        np.sum((point_array[None] - points[:-1]) * segments, axis=1) / squared_length,
        0.0,
        1.0,
    )
    closest = points[:-1] + alpha[:, None] * segments
    distances = np.linalg.norm(closest - point_array[None], axis=1)
    cumulative, _ = _polyline_arclength(points)
    # At a stale route prefix, several segments can be almost equally far
    # from a YOPO endpoint. Selecting the first one turns a valid forward
    # target into zero progress. Prefer the furthest progress inside a small
    # distance tie band; this is still a closest-point projection, with a
    # deterministic forward tie break.
    minimum_distance = float(np.min(distances))
    candidates = np.flatnonzero(distances <= minimum_distance + 0.3)
    segment_index = int(
        max(
            candidates,
            key=lambda index: float(
                cumulative[index] + alpha[index] * math.sqrt(float(squared_length[index]))
            ),
        )
    )
    return float(
        cumulative[segment_index]
        + alpha[segment_index] * math.sqrt(float(squared_length[segment_index]))
    )


def resolve_target_progress(
    point: np.ndarray,
    path_points: np.ndarray,
    *,
    reachable_progress_m: float,
    near_start_m: float = 1.0,
) -> tuple[float, bool]:
    """Return a useful forward target when the endpoint is off the route.

    A terminal proposal that is far from the route start but projects to the
    start is geometrically off-route, not a request to hold at the first
    bubble. In that case constrain the MPC to the reachable route prefix and
    report that the projection was adjusted.
    """
    progress = project_path_progress(point, path_points)
    path = np.asarray(path_points, dtype=np.float64)
    distance_to_start = float(np.linalg.norm(np.asarray(point, dtype=np.float64) - path[0]))
    adjusted = progress <= 1.0e-6 and distance_to_start > float(near_start_m)
    if adjusted:
        progress = max(0.0, float(reachable_progress_m))
    return progress, adjusted


def maximum_reachable_progress(
    *,
    horizon_time_s: float,
    initial_speed_mps: float,
    max_velocity_mps: float,
    max_acceleration_mps2: float,
) -> float:
    """Distance reachable under the one-dimensional speed/acceleration limits."""
    duration = float(horizon_time_s)
    speed = float(np.clip(initial_speed_mps, 0.0, max_velocity_mps))
    acceleration_time = max(0.0, (max_velocity_mps - speed) / max_acceleration_mps2)
    accelerating_time = min(duration, acceleration_time)
    return float(
        speed * accelerating_time
        + 0.5 * max_acceleration_mps2 * accelerating_time**2
        + max_velocity_mps * max(duration - acceleration_time, 0.0)
    )


def sample_reachable_stage_bubbles(
    path_points: np.ndarray,
    safe_radii: np.ndarray,
    *,
    horizon_steps: int,
    horizon_time_s: float,
    initial_speed_mps: float,
    max_velocity_mps: float,
    max_acceleration_mps2: float,
    target_progress_m: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample bubbles with a dynamically reachable stage-progress schedule.

    The unscaled schedule accelerates from the current forward speed to the
    velocity limit.  Scaling it down to a nearer target can only reduce its
    speed and acceleration, so the resulting timing remains feasible for the
    one-dimensional progress dynamics.
    """
    points = np.asarray(path_points, dtype=np.float64)
    radii = np.asarray(safe_radii, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 2:
        raise ValueError("path_points must contain at least two 3D points")
    if radii.shape != (len(points),) or not np.isfinite(points).all() or not np.isfinite(radii).all():
        raise ValueError("path points and radii are invalid")
    values = (horizon_time_s, max_velocity_mps, max_acceleration_mps2)
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError("horizon, velocity and acceleration limits must be positive")
    if horizon_steps < 1 or not math.isfinite(initial_speed_mps):
        raise ValueError("horizon_steps and initial_speed_mps are invalid")
    cumulative, path_length = _polyline_arclength(points)
    start_speed = float(np.clip(initial_speed_mps, 0.0, max_velocity_mps))
    acceleration_time = max(0.0, (max_velocity_mps - start_speed) / max_acceleration_mps2)
    times = np.linspace(0.0, horizon_time_s, horizon_steps + 1, dtype=np.float64)
    accelerating_time = np.minimum(times, acceleration_time)
    stage_distance = (
        start_speed * accelerating_time
        + 0.5 * max_acceleration_mps2 * accelerating_time**2
        + max_velocity_mps * np.maximum(times - acceleration_time, 0.0)
    )
    reachable_distance = float(stage_distance[-1])
    requested_distance = path_length
    if target_progress_m is not None:
        if not math.isfinite(target_progress_m) or target_progress_m < 0.0:
            raise ValueError("target_progress_m must be finite and non-negative")
        requested_distance = min(requested_distance, float(target_progress_m))
    requested_distance = min(requested_distance, reachable_distance)
    if reachable_distance > 1.0e-9:
        stage_distance *= requested_distance / reachable_distance
    else:
        stage_distance.fill(0.0)
    centers = np.column_stack(
        [np.interp(stage_distance, cumulative, points[:, axis]) for axis in range(3)]
    )
    stage_radii = np.interp(stage_distance, cumulative, radii)
    return centers, stage_radii, stage_distance


def build_ordered_bubble_ocp(
    config: OrderedBubbleMPCConfig,
    *,
    model_name: str = "ordered_bubble_yopo",
) -> tuple[Any, Any]:
    """Create an acados OCP and leap-c parameter manager."""
    try:
        import casadi as ca
        from acados_template import ACADOS_INFTY, AcadosOcp
        from leap_c.parameters import AcadosParameterManager
    except ImportError as error:
        raise RuntimeError(
            "ordered-bubble MPC requires the leap-c Python environment; "
            "activate /mnt/code/lab/yopo/leap-c/.venv and set ACADOS_SOURCE_DIR"
        ) from error

    manager = AcadosParameterManager(N_horizon=config.horizon_steps)
    bubble_center = manager.register_parameter(
        "bubble_center", np.zeros(3, dtype=np.float64), differentiable=False
    )
    bubble_radius = manager.register_parameter(
        "bubble_radius", np.ones(1, dtype=np.float64), differentiable=False
    )
    terminal_reference = manager.register_parameter(
        "terminal_reference", np.zeros(9, dtype=np.float64), differentiable=True
    )

    ocp = AcadosOcp()
    ocp.model.name = model_name
    position = ca.SX.sym("position", 3)
    velocity = ca.SX.sym("velocity", 3)
    acceleration = ca.SX.sym("acceleration", 3)
    jerk = ca.SX.sym("jerk", 3)
    state = ca.vertcat(position, velocity, acceleration)
    ocp.model.x = state
    ocp.model.u = jerk

    dt = config.dt
    ocp.model.disc_dyn_expr = ca.vertcat(
        position + dt * velocity + 0.5 * dt**2 * acceleration + (dt**3 / 6.0) * jerk,
        velocity + dt * acceleration + 0.5 * dt**2 * jerk,
        acceleration + dt * jerk,
    )

    center_error = position - bubble_center
    stage_cost = (
        config.center_weight * ca.sumsqr(center_error)
        + config.velocity_weight * ca.sumsqr(velocity)
        + config.acceleration_weight * ca.sumsqr(acceleration)
        + config.jerk_weight * ca.sumsqr(jerk)
    )
    terminal_error = state - terminal_reference
    terminal_cost = (
        config.terminal_position_weight * ca.sumsqr(terminal_error[0:3])
        + config.terminal_velocity_weight * ca.sumsqr(terminal_error[3:6])
        + config.terminal_acceleration_weight * ca.sumsqr(terminal_error[6:9])
    )
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost = stage_cost
    ocp.model.cost_expr_ext_cost_e = terminal_cost

    # Positive inside the assigned bubble.  Only the lower side is softened.
    inside_margin = bubble_radius[0] ** 2 - ca.sumsqr(center_error)
    ocp.model.con_h_expr = inside_margin
    ocp.constraints.lh = np.zeros(1)
    ocp.constraints.uh = np.full(1, ACADOS_INFTY)
    ocp.constraints.idxsh = np.array([0])
    ocp.cost.zl = np.full(1, config.bubble_slack_linear_weight)
    ocp.cost.zu = np.full(1, config.bubble_slack_linear_weight)
    ocp.cost.Zl = np.full(1, config.bubble_slack_quadratic_weight)
    ocp.cost.Zu = np.full(1, config.bubble_slack_quadratic_weight)

    ocp.model.con_h_expr_e = inside_margin
    ocp.constraints.lh_e = np.zeros(1)
    ocp.constraints.uh_e = np.full(1, ACADOS_INFTY)
    ocp.constraints.idxsh_e = np.array([0])
    ocp.cost.zl_e = np.full(1, config.bubble_slack_linear_weight)
    ocp.cost.zu_e = np.full(1, config.bubble_slack_linear_weight)
    ocp.cost.Zl_e = np.full(1, config.bubble_slack_quadratic_weight)
    ocp.cost.Zu_e = np.full(1, config.bubble_slack_quadratic_weight)

    ocp.constraints.x0 = np.zeros(9)
    ocp.constraints.idxbu = np.arange(3)
    ocp.constraints.lbu = np.full(3, -config.max_jerk_mps3)
    ocp.constraints.ubu = np.full(3, config.max_jerk_mps3)
    ocp.constraints.idxbx = np.arange(3, 9)
    ocp.constraints.lbx = np.concatenate(
        (
            np.full(3, -config.max_velocity_mps),
            np.full(3, -config.max_acceleration_mps2),
        )
    )
    ocp.constraints.ubx = -ocp.constraints.lbx
    ocp.constraints.idxbx_e = np.arange(3, 9)
    ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
    ocp.constraints.ubx_e = ocp.constraints.ubx.copy()

    options = ocp.solver_options
    options.N_horizon = config.horizon_steps
    options.tf = config.horizon_time_s
    options.integrator_type = "DISCRETE"
    options.nlp_solver_type = "SQP"
    options.nlp_solver_max_iter = 60
    options.hessian_approx = "EXACT"
    options.regularize_method = "CONVEXIFY"
    options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    options.qp_solver_cond_N = config.horizon_steps
    options.qp_solver_iter_max = 100
    options.tol = 1.0e-6

    return ocp, manager


class OrderedBubbleMPC:
    """Torch-facing leap-c layer for YOPO terminal references."""

    def __init__(
        self,
        config: OrderedBubbleMPCConfig = OrderedBubbleMPCConfig(),
        *,
        batch_size: int = 1,
        export_directory: str | Path | None = None,
        model_name: str = "ordered_bubble_yopo",
        verbose: bool = False,
    ) -> None:
        try:
            import torch
            from leap_c.torch import AcadosDiffMpcTorch
        except ImportError as error:
            raise RuntimeError(
                "OrderedBubbleMPC must run inside /mnt/code/lab/yopo/leap-c/.venv"
            ) from error

        self.config = config
        ocp, manager = build_ordered_bubble_ocp(config, model_name=model_name)
        self.layer = AcadosDiffMpcTorch(
            ocp,
            manager,
            dtype=torch.float64,
            n_batch_init=batch_size,
            num_threads_batch_solver=1,
            export_directory=None if export_directory is None else str(export_directory),
            verbose=verbose,
        )

    def __call__(
        self,
        initial_state: Any,
        terminal_reference: Any,
        bubble_centers: Any,
        bubble_radii: Any,
        *,
        context: Any | None = None,
    ) -> tuple[Any, Any, Any, Any, Any]:
        import torch

        dtype = torch.float64
        x0 = torch.as_tensor(initial_state, dtype=dtype)
        reference = torch.as_tensor(terminal_reference, dtype=dtype)
        centers = torch.as_tensor(bubble_centers, dtype=dtype)
        radii = torch.as_tensor(bubble_radii, dtype=dtype)
        if x0.ndim == 1:
            x0 = x0.unsqueeze(0)
        if reference.ndim == 1:
            reference = reference.unsqueeze(0)
        if centers.ndim == 2:
            centers = centers.unsqueeze(0)
        if radii.ndim == 1:
            radii = radii.unsqueeze(0)
        expected_stages = self.config.horizon_steps + 1
        if x0.shape[1:] != (9,) or reference.shape[1:] != (9,):
            raise ValueError("initial_state and terminal_reference must have shape (B, 9)")
        if centers.shape[1:] != (expected_stages, 3):
            raise ValueError(f"bubble_centers must have shape (B, {expected_stages}, 3)")
        if radii.shape[1:] != (expected_stages,):
            raise ValueError(f"bubble_radii must have shape (B, {expected_stages})")
        if not (x0.shape[0] == reference.shape[0] == centers.shape[0] == radii.shape[0]):
            raise ValueError("all MPC inputs must have the same batch size")
        radii = radii.clamp_min(self.config.minimum_radius_m).unsqueeze(-1)
        return self.layer(
            x0=x0,
            params={
                "terminal_reference": reference,
                "bubble_center": centers,
                "bubble_radius": radii,
            },
            ctx=context,
        )


def maximum_bubble_violation(
    positions: np.ndarray, centers: np.ndarray, radii: np.ndarray
) -> float:
    distance = np.linalg.norm(np.asarray(positions) - np.asarray(centers), axis=-1)
    return float(np.maximum(distance - np.asarray(radii), 0.0).max(initial=0.0))
