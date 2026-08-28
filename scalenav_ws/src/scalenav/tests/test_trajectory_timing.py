from __future__ import annotations

import numpy as np
import pytest

from policy.poly_solver import Poly5Solver
from trajectory_timing import (
    load_maximum_trajectory_speed,
    sample_time_scaled_trajectory,
    trajectory_peak_speed,
    trajectory_time_scale,
)


def make_trajectory(duration_s: float = 1.0):
    return tuple(
        Poly5Solver(start, velocity, 0.0, end, velocity, 0.0, duration_s)
        for start, end, velocity in (
            (0.0, 12.0, 0.0),
            (0.0, 2.0, 0.0),
            (1.6, 1.6, 0.0),
        )
    )


def test_time_scaling_limits_continuous_trajectory_speed():
    polynomials = make_trajectory()
    maximum_speed = 6.0
    peak_speed = trajectory_peak_speed(polynomials, 1.0)
    scale = trajectory_time_scale(peak_speed, maximum_speed)

    assert scale > 1.0
    sampled_speeds = []
    for elapsed in np.linspace(0.0, scale, 1001):
        _, velocity, _, _ = sample_time_scaled_trajectory(
            polynomials, elapsed, 1.0, scale
        )
        sampled_speeds.append(np.linalg.norm(velocity))
    assert max(sampled_speeds) <= maximum_speed + 1e-9


def test_time_scaling_keeps_the_same_spatial_path_and_scales_derivatives():
    polynomials = make_trajectory()
    scale = 2.5
    polynomial_time = 0.4
    position, velocity, acceleration, sampled_time = sample_time_scaled_trajectory(
        polynomials, polynomial_time * scale, 1.0, scale
    )

    expected_position = np.array(
        [polynomial.get_position(polynomial_time) for polynomial in polynomials]
    )
    expected_velocity = np.array(
        [polynomial.get_velocity(polynomial_time) for polynomial in polynomials]
    )
    expected_acceleration = np.array(
        [polynomial.get_acceleration(polynomial_time) for polynomial in polynomials]
    )
    assert sampled_time == pytest.approx(polynomial_time)
    np.testing.assert_allclose(position, expected_position)
    np.testing.assert_allclose(velocity, expected_velocity / scale)
    np.testing.assert_allclose(acceleration, expected_acceleration / scale**2)


def test_config_load_and_cli_override(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text(
        "scalenav_online_planner:\n"
        "  ros__parameters:\n"
        "    maximum_trajectory_speed_mps: 7.5\n",
        encoding="utf-8",
    )
    assert load_maximum_trajectory_speed(config) == pytest.approx(7.5)
    assert load_maximum_trajectory_speed(config, override_mps=4.0) == pytest.approx(4.0)


@pytest.mark.parametrize("value", [0.0, -1.0, float("inf"), float("nan")])
def test_invalid_speed_is_rejected(value):
    with pytest.raises(ValueError):
        load_maximum_trajectory_speed(None, override_mps=value)
