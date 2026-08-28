"""Strict 3-D contract tests against the frozen YOPO-Simple baseline.

These tests intentionally exercise all three spatial axes.  A top-down plot
can hide a sign error in pitch, a row/column lattice permutation, or a
transposed body/world rotation, so each test compares the complete vectors.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from evaluate_yopo import _sample_trajectory
from policy.state_transform import StateTransform, state_body2world
from policy.yopo_simple_baseline import YopoSimpleBaseline
from policy.yopo_network import YopoNetwork


def _upstream_poly5_solver():
    """Load the upstream standalone solver without shadowing local policy modules."""
    path = Path("/mnt/code/lab/yopo/YOPO-Simple/YOPO/policy/poly_solver.py")
    spec = importlib.util.spec_from_file_location("upstream_poly_solver", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load upstream solver: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Poly5Solver


def test_lattice_3x5_angles_and_rotations_match_frozen_baseline():
    """The image-grid flip and 3x5 flatten order must be identical."""
    transform = StateTransform()
    baseline = YopoSimpleBaseline()

    # LatticePrimitive stores the canonical bottom-to-top/right-to-left order;
    # both implementations apply ``flip(0)`` only when mapping to image-grid
    # primitive indices.  Compare the canonical buffers here.
    local_angles = torch.stack(transform.lattice_primitive.getAngleLattice(), dim=1).cpu()
    local_rotations = transform.lattice_primitive.getRotation().cpu()

    torch.testing.assert_close(local_angles, baseline.lattice_angles, rtol=0.0, atol=1.0e-6)
    torch.testing.assert_close(
        local_rotations, baseline.lattice_rotations, rtol=0.0, atol=1.0e-6
    )
    assert tuple(local_angles.shape) == (15, 2)
    assert tuple(local_rotations.shape) == (15, 3, 3)


def test_random_3d_prediction_decoding_matches_frozen_baseline():
    """Compare p, v and a for every primitive, including nonzero pitch."""
    transform = StateTransform()
    baseline = YopoSimpleBaseline()
    device = transform.lattice_primitive.getRotation().device
    generator = torch.Generator(device=device).manual_seed(20260828)
    raw = torch.randn((4, 9, 3, 5), generator=generator, device=device)

    local = transform.pred_to_endstate(torch.tanh(raw)).cpu()
    reference = baseline._decode(torch.tanh(raw.cpu())).detach().cpu()
    torch.testing.assert_close(local, reference, rtol=1.0e-5, atol=1.0e-5)

    # Explicitly check the vertical channel is exercised and not accidentally
    # flattened away by a 2-D projection.
    assert float(local[:, 2].abs().max()) > 1.0e-3
    assert float(local[:, 5:9].abs().max()) > 1.0e-3


def test_zero_prediction_has_explicit_3x5_yaw_pitch_layout():
    """Pin output row/column orientation independently of either implementation."""
    transform = StateTransform()
    device = transform.lattice_primitive.getRotation().device
    decoded = transform.pred_to_endstate(torch.zeros((1, 9, 3, 5), device=device))
    positions = decoded[0, :3].permute(1, 2, 0).cpu().numpy()
    radii = np.linalg.norm(positions, axis=2)
    pitch_deg = np.degrees(np.arcsin(positions[:, :, 2] / radii))
    yaw_deg = np.degrees(np.arctan2(positions[:, :, 1], positions[:, :, 0]))
    np.testing.assert_allclose(radii, 5.0, atol=1.0e-5)
    np.testing.assert_allclose(pitch_deg, [[20] * 5, [0] * 5, [-20] * 5], atol=1.0e-4)
    np.testing.assert_allclose(
        yaw_deg,
        [[36, 18, 0, -18, -36], [36, 18, 0, -18, -36], [36, 18, 0, -18, -36]],
        atol=1.0e-4,
    )


def test_observation_3d_primitive_transform_matches_frozen_baseline():
    """Body-frame v/a/goal preprocessing must preserve all axes and order."""
    transform = StateTransform()
    baseline = YopoSimpleBaseline()
    device = transform.lattice_primitive.getRotation().device
    motion = torch.tensor(
        [[2.3, -1.1, 0.7, -0.4, 0.8, 1.2], [-1.2, 0.5, -0.9, 0.2, -0.6, 0.4]],
        dtype=torch.float32,
        device=device,
    )
    goal = torch.tensor(
        [[8.0, -3.0, 2.5], [-4.0, 6.0, -2.0]], dtype=torch.float32, device=device
    )
    observation = torch.cat((motion, goal), dim=1)
    local_input = transform.prepare_input(transform.normalize_obs(observation.clone())).cpu()
    reference_input = baseline._prepare_observation(motion.cpu(), goal.cpu()).cpu()
    torch.testing.assert_close(local_input, reference_input, rtol=1.0e-5, atol=1.0e-5)


def test_poly5_sampler_matches_upstream_solver_in_xyz():
    """The offline trajectory sampler must agree with YOPO's Poly5Solver."""
    start = np.array(
        [[1.2, -2.0, 0.4], [0.7, -0.3, 0.2], [0.1, 0.4, -0.2]], dtype=np.float32
    )
    end = np.array(
        [[7.5, 3.1, 4.2], [-0.2, 1.4, -0.8], [0.4, -0.7, 0.9]], dtype=np.float32
    )
    count = 37
    sampled = _sample_trajectory(start, end, count=count)
    times = np.linspace(0.0, 10.0 / 6.0, count)
    poly5_solver = _upstream_poly5_solver()
    reference = np.stack(
        [
            [poly5_solver(start[0, axis], start[1, axis], start[2, axis], end[0, axis], end[1, axis], end[2, axis], 10.0 / 6.0).get_position(t) for t in times]
            for axis in range(3)
        ],
        axis=1,
    )
    np.testing.assert_allclose(sampled, reference, rtol=1.0e-5, atol=2.0e-5)
    assert sampled.shape == (count, 3)
    assert np.ptp(sampled[:, 2]) > 0.1


def test_body_world_transform_matches_manual_3d_rotation():
    """Position, velocity and acceleration all use R_wb (no transpose)."""
    rotation = torch.as_tensor(
        Rotation.from_euler("ZYX", [37.0, -19.0, 23.0], degrees=True).as_matrix(),
        dtype=torch.float32,
    ).unsqueeze(0)
    translation = torch.tensor([[4.0, -2.5, 1.7]], dtype=torch.float32)
    pos_body = torch.tensor([[1.5, -0.8, 2.2]], dtype=torch.float32)
    vel_body = torch.tensor([[-0.4, 2.1, 0.9]], dtype=torch.float32)
    acc_body = torch.tensor([[0.6, -1.3, 0.2]], dtype=torch.float32)

    pos_world, vel_world, acc_world = state_body2world(
        translation, rotation, pos_body, vel_body, acc_body
    )
    matrix = rotation[0].numpy()
    np.testing.assert_allclose(pos_world.numpy()[0], matrix @ pos_body.numpy()[0] + translation.numpy()[0], atol=1.0e-6)
    np.testing.assert_allclose(vel_world.numpy()[0], matrix @ vel_body.numpy()[0], atol=1.0e-6)
    np.testing.assert_allclose(acc_world.numpy()[0], matrix @ acc_body.numpy()[0], atol=1.0e-6)


def test_legacy_route_head_channel_migration_preserves_3d_output():
    """Corrected concat plus migrated weights must equal the old head function."""
    model = YopoNetwork().cpu().eval()
    state = model.state_dict()
    legacy = dict(state)
    generator = torch.Generator().manual_seed(8428)
    weight = torch.randn(state["yopo_head.model.0.weight"].shape, generator=generator)
    legacy["yopo_head.model.0.weight"] = weight
    migrated = model.migrate_legacy_route_state_dict(legacy)

    depth = torch.randn((2, 64, 3, 5), generator=generator)
    observation = torch.randn((2, 9, 3, 5), generator=generator)
    route = torch.randn((2, weight.shape[1] - 73, 3, 5), generator=generator)
    old_features = torch.cat((depth, observation, route), dim=1)
    new_features = torch.cat((observation, depth, route), dim=1)
    old_output = torch.nn.functional.conv2d(
        old_features, weight, state["yopo_head.model.0.bias"]
    )
    new_output = torch.nn.functional.conv2d(
        new_features, migrated["yopo_head.model.0.weight"], state["yopo_head.model.0.bias"]
    )
    torch.testing.assert_close(old_output, new_output, rtol=0.0, atol=2.0e-5)
