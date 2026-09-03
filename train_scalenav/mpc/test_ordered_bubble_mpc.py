"""Executable smoke and regression tests for ordered-bubble diff-MPC."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import pytest
import torch

from mpc.ordered_bubble_ocp import (
    OrderedBubbleMPC,
    OrderedBubbleMPCConfig,
    maximum_bubble_violation,
    sample_reachable_stage_bubbles,
    sample_stage_bubbles,
)


def _corridor(kind: str, count: int = 101) -> tuple[np.ndarray, np.ndarray]:
    distance = np.linspace(0.0, 10.0, count)
    if kind == "straight":
        points = np.column_stack((distance, np.zeros(count), np.zeros(count)))
        radii = np.full(count, 1.0)
    elif kind == "curve":
        angle = distance / 10.0 * (math.pi / 3.0)
        points = np.column_stack(
            (8.0 * np.sin(angle), 8.0 * (1.0 - np.cos(angle)), np.zeros(count))
        )
        radii = np.full(count, 0.85)
        radii[(distance > 4.0) & (distance < 6.0)] = 0.55
    else:
        raise ValueError(kind)
    return points, radii


def _run_case(
    mpc: OrderedBubbleMPC,
    kind: str,
    terminal_position: np.ndarray,
) -> dict[str, float | int | list[float]]:
    path, radii = _corridor(kind)
    centers, stage_radii = sample_stage_bubbles(
        path,
        radii,
        horizon_steps=mpc.config.horizon_steps,
        travel_distance_m=10.0,
        horizon_time_s=mpc.config.horizon_time_s,
        initial_speed_mps=3.0,
        terminal_speed_mps=5.5,
    )
    initial = np.zeros(9)
    initial[3] = 3.0
    terminal = np.zeros(9)
    terminal[:3] = terminal_position
    terminal[3:6] = np.array([5.5, 0.0, 0.0])
    start = time.perf_counter()
    ctx, _, states, _, value = mpc(initial, terminal, centers, stage_radii)
    latency_ms = (time.perf_counter() - start) * 1.0e3
    trajectory = states.detach().cpu().numpy()[0]
    return {
        "status": int(ctx.status[0]),
        "latencyMs": latency_ms,
        "value": float(value.detach().cpu().numpy()[0, 0]),
        "maximumBubbleViolationM": maximum_bubble_violation(
            trajectory[:, :3], centers, stage_radii
        ),
        "terminalPosition": trajectory[-1, :3].round(6).tolist(),
        "terminalReference": terminal_position.round(6).tolist(),
    }


def run_regression(output: Path | None = None) -> dict[str, object]:
    config = OrderedBubbleMPCConfig()
    mpc = OrderedBubbleMPC(config, batch_size=1, model_name="ordered_bubble_regression")
    straight_path, _ = _corridor("straight")
    straight = _run_case(mpc, "straight", straight_path[-1])
    curve_path, _ = _corridor("curve")
    curved = _run_case(mpc, "curve", curve_path[-1])
    corrected = _run_case(mpc, "straight", np.array([10.0, 4.0, 0.0]))

    centers, radii = sample_stage_bubbles(
        straight_path,
        np.full(len(straight_path), 3.0),
        horizon_steps=config.horizon_steps,
        travel_distance_m=10.0,
        horizon_time_s=config.horizon_time_s,
        initial_speed_mps=3.0,
        terminal_speed_mps=5.0,
    )
    initial = torch.zeros((1, 9), dtype=torch.float64)
    initial[:, 3] = 3.0
    terminal = torch.zeros((1, 9), dtype=torch.float64, requires_grad=True)
    terminal.data[:, 0] = 8.0
    terminal.data[:, 3] = 5.0
    ctx, _, states, _, value = mpc(initial, terminal, centers, radii)
    trajectory_loss = (states[:, -1, 0] - 7.0).square().sum()
    trajectory_loss.backward(retain_graph=True)
    trajectory_gradient = terminal.grad.detach().clone()
    terminal.grad.zero_()
    value.sum().backward()
    value_gradient = terminal.grad.detach().clone()

    timings = []
    warm_context = ctx
    for _ in range(20):
        start = time.perf_counter()
        warm_context, _, _, _, _ = mpc(
            initial, terminal.detach(), centers, radii, context=warm_context
        )
        timings.append((time.perf_counter() - start) * 1.0e3)

    report: dict[str, object] = {
        "configuration": {
            "horizonSteps": config.horizon_steps,
            "horizonTimeS": config.horizon_time_s,
            "dtS": config.dt,
        },
        "straight": straight,
        "curvedNarrowWaist": curved,
        "outsideProposalCorrection": corrected,
        "gradient": {
            "status": int(ctx.status[0]),
            "trajectoryGradientNorm": float(torch.linalg.vector_norm(trajectory_gradient)),
            "valueGradientNorm": float(torch.linalg.vector_norm(value_gradient)),
            "trajectoryGradientFinite": bool(torch.isfinite(trajectory_gradient).all()),
            "valueGradientFinite": bool(torch.isfinite(value_gradient).all()),
        },
        "warmLatencyMs": {
            "mean": float(np.mean(timings)),
            "p50": float(np.percentile(timings, 50)),
            "p95": float(np.percentile(timings, 95)),
            "maximum": float(np.max(timings)),
        },
    }
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def test_regression() -> None:
    report = run_regression()
    for key in ("straight", "curvedNarrowWaist", "outsideProposalCorrection"):
        assert report[key]["status"] == 0
    assert report["straight"]["maximumBubbleViolationM"] < 2.0e-3
    assert report["curvedNarrowWaist"]["maximumBubbleViolationM"] < 2.0e-2
    corrected_y = report["outsideProposalCorrection"]["terminalPosition"][1]
    assert abs(corrected_y) < 1.05
    assert report["gradient"]["trajectoryGradientFinite"]
    assert report["gradient"]["valueGradientFinite"]
    assert report["gradient"]["trajectoryGradientNorm"] > 1.0e-8
    assert report["gradient"]["valueGradientNorm"] > 1.0e-8


def test_reachable_bubble_schedule_respects_motion_envelope() -> None:
    path, radii = _corridor("straight")
    centers, _, progress = sample_reachable_stage_bubbles(
        path,
        radii,
        horizon_steps=12,
        horizon_time_s=10.0 / 6.0,
        initial_speed_mps=0.0,
        max_velocity_mps=2.0,
        max_acceleration_mps2=6.0,
        target_progress_m=8.0,
    )
    # Accelerate for 1/3 s, then cruise: 0.5*6*(1/3)^2 + 2*(4/3) = 3 m.
    assert progress[-1] == pytest.approx(3.0, abs=1.0e-8)
    assert np.all(np.diff(progress) >= 0.0)
    np.testing.assert_allclose(centers[:, 0], progress)


if __name__ == "__main__":
    result = run_regression(
        Path(__file__).resolve().parents[1] / "tmp" / "mpc_001" / "prototype_report.json"
    )
    print(json.dumps(result, indent=2))
