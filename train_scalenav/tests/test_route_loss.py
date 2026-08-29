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


def _route(y: float) -> tuple[torch.Tensor, torch.Tensor]:
    x = torch.linspace(0.0, 10.0, 41)
    points = torch.stack((x, torch.full_like(x, y), torch.zeros_like(x)), dim=1)[None]
    radius = torch.full((1, 41), 0.75)
    return points, radius


def test_route_loss_penalizes_leaving_corridor_and_backpropagates():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    decision = torch.tensor(
        [[[6.0, 4.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]],
        requires_grad=True,
    )
    on_route = loss(fixed, decision, *_route(0.0))
    off_route = loss(fixed, decision, *_route(3.0))
    assert off_route[0].item() > on_route[0].item() + 1.0
    total = sum(value.mean() for value in off_route)
    total.backward()
    assert decision.grad is not None
    assert torch.isfinite(decision.grad).all()


def test_bubble_field_has_gradient_inside_safe_volume():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    decision = torch.tensor(
        [[[6.0, 4.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]],
        requires_grad=True,
    )
    x = torch.linspace(0.0, 10.0, 41)
    route = torch.stack((x, torch.full_like(x, 0.3), torch.zeros_like(x)), dim=1)[None]
    radius = torch.full((1, 41), 0.75)
    corridor = loss(fixed, decision, route, radius)[0]
    corridor.backward()
    assert corridor.item() < 2.0
    assert decision.grad is not None
    assert torch.isfinite(decision.grad).all()
    assert decision.grad[:, 0, 1].abs().item() > 1.0e-6


def test_route_progress_floor_rejects_short_endpoint():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    short = torch.tensor(
        [[[4.0, 4.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]]
    )
    long = torch.tensor(
        [[[7.0, 4.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]]
    )
    short_cost = loss(fixed, short, *_route(0.0))[3]
    long_cost = loss(fixed, long, *_route(0.0))[3]
    # The configured floor is 6.8 m (rather than the historical 6.4 m), so
    # keep the assertion tied to a positive, substantial penalty.
    assert short_cost.item() > 7.0
    assert long_cost.item() < 1.0e-5
