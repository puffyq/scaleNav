import numpy as np
import torch

from config.config import cfg
from policy.spline_trajectory import ClampedCubicSpline
from policy.spline_yopo_network import SplineYopoNetwork
from policy.yopo_dataset import YOPODataset


def test_spline_preserves_initial_position_velocity_and_acceleration():
    spline = ClampedCubicSpline(
        control_point_count=12,
        duration=float(cfg["sgm_time"]),
        sample_count=30,
    )
    start_position = torch.tensor([[1.0, 2.0, 3.0]])
    start_velocity = torch.tensor([[4.0, -2.0, 1.0]])
    start_acceleration = torch.tensor([[0.5, -0.25, 0.75]])
    free = torch.randn(1, 9, 3)
    controls = spline.assemble_controls(
        start_position, start_velocity, start_acceleration, free
    )
    position, velocity, acceleration, _ = spline.basis(2, include_start=True)
    position = torch.from_numpy(position).float()
    velocity = torch.from_numpy(velocity).float()
    acceleration = torch.from_numpy(acceleration).float()
    sampled_position = spline.sample_with_basis(controls, position)
    sampled_velocity = spline.sample_with_basis(controls, velocity)
    sampled_acceleration = spline.sample_with_basis(controls, acceleration)
    assert torch.allclose(sampled_position[:, 0], start_position, atol=1.0e-5)
    assert torch.allclose(sampled_velocity[:, 0], start_velocity, atol=1.0e-4)
    assert torch.allclose(
        sampled_acceleration[:, 0], start_acceleration, atol=1.0e-3
    )


def test_spline_network_output_contract_and_route_is_primitive_local():
    network = SplineYopoNetwork(control_point_count=12).eval()
    batch = 2
    depth = torch.zeros(batch, 1, int(cfg["image_height"]), int(cfg["image_width"]))
    motion = torch.zeros(batch, 6)
    frontier = torch.tensor([[10.0, 0.0, 0.0]]).expand(batch, -1)
    route = torch.zeros(batch, int(cfg["route_bubble_count"]), 4)
    route[:, :, 0] = 1.0
    prepared = network._prepare_route(route)
    assert not torch.allclose(prepared[:, :, 0, 0], prepared[:, :, 1, 2])
    controls, score = network(depth, motion, frontier, route)
    assert controls.shape == (batch, int(cfg["traj_num"]), 9, 3)
    assert score.shape == (batch, int(cfg["vertical_num"]), int(cfg["horizon_num"]))


def test_route_conditioned_motion_never_starts_backwards():
    dataset = object.__new__(YOPODataset)
    dataset.vel_max = float(cfg["vel_max_train"])
    dataset.acc_max = float(cfg["acc_max_train"])
    dataset.vx_lognorm_mean = float(torch.log(torch.tensor(1.0 - cfg["vx_mean_unit"])))
    dataset.vx_lognorm_sigma = float(torch.log(torch.tensor(cfg["vx_std_unit"])))
    dataset.v_mean = np.asarray(
        [cfg["vx_mean_unit"], cfg["vy_mean_unit"], cfg["vz_mean_unit"]]
    )
    dataset.v_std = np.asarray(
        [cfg["vx_std_unit"], cfg["vy_std_unit"], cfg["vz_std_unit"]]
    )
    dataset.a_mean = np.asarray(
        [cfg["ax_mean_unit"], cfg["ay_mean_unit"], cfg["az_mean_unit"]]
    )
    dataset.a_std = np.asarray(
        [cfg["ax_std_unit"], cfg["ay_std_unit"], cfg["az_std_unit"]]
    )
    generator = np.random.default_rng(7)
    tangent = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    for _ in range(100):
        motion = dataset._random_motion(generator, forward_direction=tangent)
        assert float(motion[:3] @ tangent) >= 0.5 - 1.0e-5
