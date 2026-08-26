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


def _route(y: float) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    x = torch.linspace(0.0, 10.0, 41)
    points = torch.stack((x, torch.full_like(x, y), torch.zeros_like(x)), dim=1)[None]
    radius = torch.full((1, 41), 0.75)
    mask = torch.ones((1, 41))
    return points, radius, mask


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


def test_route_dropout_disables_all_route_costs():
    loss = RouteLoss(_coefficient_map())
    fixed = torch.zeros((1, 3, 3))
    decision = torch.zeros((1, 3, 3))
    points, radius, mask = _route(2.0)
    costs = loss(fixed, decision, points, radius, torch.zeros_like(mask))
    for cost in costs:
        torch.testing.assert_close(cost, torch.zeros_like(cost))
