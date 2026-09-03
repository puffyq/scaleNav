import math

import torch

from config.config import cfg
from loss.route_loss import RouteLoss


def _coefficient_map() -> torch.Tensor:
    duration = float(cfg["sgm_time"])
    boundary_matrix = torch.zeros((6, 6))
    for derivative in range(3):
        boundary_matrix[2 * derivative, derivative] = math.factorial(derivative)
        for power in range(derivative, 6):
            boundary_matrix[2 * derivative + 1, power] = (
                math.factorial(power)
                / math.factorial(power - derivative)
                * duration ** (power - derivative)
            )
    selector = torch.zeros((6, 6))
    selector[[0, 2, 4, 1, 3, 5], [0, 1, 2, 3, 4, 5]] = 1
    return torch.inverse(boundary_matrix) @ selector


def _route(y: float) -> torch.Tensor:
    x = torch.tensor(cfg["route_anchor_distances_m"], dtype=torch.float32)
    return torch.stack((x, torch.full_like(x, y), torch.zeros_like(x)), dim=1)[None]


def test_route_loss_is_one_scalar_per_trajectory_and_backpropagates():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    decision = torch.tensor(
        [[[10.0, 6.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]],
        requires_grad=True,
    )
    route_angle = loss(fixed, decision, _route(3.0))
    assert route_angle.shape == (1,)
    route_angle.mean().backward()
    assert decision.grad is not None
    assert torch.isfinite(decision.grad).all()


def test_route_angle_matches_the_guide_prediction_heading_difference():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    decision = torch.tensor(
        [[[10.0, 6.0, 0.0], [10.0, 0.0, 0.0], [0.0, 0.0, 0.0]]],
        requires_grad=True,
    )
    route_angle = loss(fixed, decision, _route(0.0))
    assert torch.allclose(route_angle, torch.tensor([torch.pi / 4]), atol=1.0e-5)
    route_angle.backward()
    assert decision.grad is not None
    assert torch.isfinite(decision.grad).all()
    assert decision.grad[:, 1, 0].abs().item() > 1.0e-6


def test_route_angle_is_independent_of_prediction_length():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    short = torch.tensor(
        [[[4.0, 4.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]]
    )
    long = torch.tensor(
        [[[7.0, 4.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]]
    )
    short_cost = loss(fixed, short, _route(0.0))
    long_cost = loss(fixed, long, _route(0.0))
    torch.testing.assert_close(short_cost, long_cost)
    assert short_cost.item() == 0.0


def test_route_angle_is_independent_of_polynomial_timing():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    fixed[:, 0, 1] = 6.0
    aligned = torch.tensor(
        [[[10.0, 6.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]]
    )
    wrong_progress = torch.tensor(
        [[[10.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]]
    )
    aligned_cost = loss(fixed, aligned, _route(0.0))
    wrong_progress_cost = loss(fixed, wrong_progress, _route(0.0))
    assert aligned_cost.item() == 0.0
    torch.testing.assert_close(wrong_progress_cost, aligned_cost)


def test_centerline_mse_is_ordered_and_differentiable():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    decision = torch.tensor(
        [[[10.0, 6.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]],
        requires_grad=True,
    )
    aligned = loss.centerline_mse(fixed, decision, _route(0.0))
    offset = loss.centerline_mse(fixed, decision, _route(2.0))
    assert aligned.shape == (1,)
    assert offset.item() > aligned.item()
    offset.mean().backward()
    assert decision.grad is not None
    assert torch.isfinite(decision.grad).all()


def test_bubble_radius_only_penalizes_points_outside_corridor():
    loss = RouteLoss(_coefficient_map(), bubble_radius_weight=1.0)
    start = torch.zeros((1, 3))
    positions = torch.stack(
        (
            torch.linspace(1.0, 10.0, 10),
            torch.full((10,), 0.75),
            torch.zeros(10),
        ),
        dim=1,
    )[None]
    # The configured anchors contain a 2 m gap (6 -> 8 m); use a radius
    # large enough to keep that union continuous for the wide-corridor case.
    wide = torch.full((1, 12), 1.5)
    narrow = torch.full((1, 12), 0.5)
    _, wide_cost = loss.forward_positions_components(
        positions, start, _route(0.0), route_radii=wide
    )
    _, narrow_cost = loss.forward_positions_components(
        positions, start, _route(0.0), route_radii=narrow
    )
    assert wide_cost.item() == 0.0
    assert narrow_cost.item() > 0.0
