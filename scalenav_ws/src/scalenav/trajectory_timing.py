from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml


def load_maximum_trajectory_speed(
    config_file: str | Path | None,
    override_mps: float | None = None,
    default_mps: float = 6.0,
) -> float:
    """Load the YOPO execution-speed limit, with an optional CLI override."""
    value = override_mps
    if value is None and config_file is not None:
        path = Path(config_file).expanduser()
        if not path.is_file():
            raise ValueError(f"config file not found: {path}")
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        try:
            value = document["scalenav_online_planner"]["ros__parameters"][
                "maximum_trajectory_speed_mps"
            ]
        except (KeyError, TypeError) as error:
            raise ValueError(
                "config is missing scalenav_online_planner.ros__parameters."
                "maximum_trajectory_speed_mps"
            ) from error
    if value is None:
        value = default_mps
    try:
        speed = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("maximum trajectory speed must be numeric") from error
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("maximum trajectory speed must be finite and positive")
    return speed


def trajectory_peak_speed(polynomials: Sequence, duration_s: float) -> float:
    """Return the continuous-time peak norm of a three-axis Poly5 trajectory."""
    if duration_s <= 0.0 or not math.isfinite(duration_s):
        raise ValueError("trajectory duration must be finite and positive")
    if len(polynomials) != 3:
        raise ValueError("a trajectory must contain three axis polynomials")

    # d(||v||^2)/dt = 2 v dot a is degree seven. Its real roots plus the
    # interval endpoints contain the exact continuous-time speed maximum.
    derivative = np.zeros(8, dtype=np.float64)
    for polynomial in polynomials:
        coefficients = np.asarray(polynomial.A, dtype=np.float64)
        velocity = np.arange(1, 6, dtype=np.float64) * coefficients[1:]
        acceleration = np.arange(1, 5, dtype=np.float64) * velocity[1:]
        product = np.polynomial.polynomial.polymul(velocity, acceleration)
        derivative[: product.size] += 2.0 * product

    candidates = [0.0, float(duration_s)]
    if np.any(np.abs(derivative) > 1e-12):
        for root in np.polynomial.polynomial.polyroots(np.trim_zeros(derivative, "b")):
            if abs(float(root.imag)) <= 1e-7:
                root_time = float(root.real)
                if 0.0 < root_time < duration_s:
                    candidates.append(root_time)

    return max(
        float(
            np.linalg.norm(
                [polynomial.get_velocity(sample_time) for polynomial in polynomials]
            )
        )
        for sample_time in candidates
    )


def trajectory_time_scale(peak_speed_mps: float, maximum_speed_mps: float) -> float:
    if not math.isfinite(peak_speed_mps) or peak_speed_mps < 0.0:
        raise ValueError("peak trajectory speed must be finite and non-negative")
    if not math.isfinite(maximum_speed_mps) or maximum_speed_mps <= 0.0:
        raise ValueError("maximum trajectory speed must be finite and positive")
    if peak_speed_mps <= maximum_speed_mps:
        return 1.0
    return (peak_speed_mps / maximum_speed_mps) * (1.0 + 1e-12)


def sample_time_scaled_trajectory(
    polynomials: Sequence,
    elapsed_s: float,
    base_duration_s: float,
    time_scale: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Sample P(t/scale), including correctly scaled derivatives."""
    if not math.isfinite(time_scale) or time_scale < 1.0:
        raise ValueError("trajectory time scale must be finite and at least one")
    polynomial_time = min(max(float(elapsed_s) / time_scale, 0.0), base_duration_s)
    position = np.array(
        [polynomial.get_position(polynomial_time) for polynomial in polynomials],
        dtype=np.float64,
    )
    velocity = np.array(
        [polynomial.get_velocity(polynomial_time) for polynomial in polynomials],
        dtype=np.float64,
    ) / time_scale
    acceleration = np.array(
        [polynomial.get_acceleration(polynomial_time) for polynomial in polynomials],
        dtype=np.float64,
    ) / (time_scale * time_scale)
    return position, velocity, acceleration, polynomial_time


def sample_fixed_period_trajectory(
    polynomials: Sequence,
    current_time_s: float,
    control_period_s: float,
    duration_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Advance and sample a trajectory on YOPO-Simple's discrete control clock."""
    values = (current_time_s, control_period_s, duration_s)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("trajectory control times must be finite")
    if current_time_s < 0.0 or control_period_s <= 0.0 or duration_s <= 0.0:
        raise ValueError(
            "current trajectory time must be non-negative and periods positive"
        )
    sample_time = min(current_time_s + control_period_s, duration_s)
    return sample_time_scaled_trajectory(
        polynomials, sample_time, duration_s, 1.0
    )
