from __future__ import annotations

from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import Odometry

from graph.depth_query import DepthSafeVolumeQuery
from demo_route_yopo_log_replay import select_depth_record
from route_yopo_control_core import (
    LocalRouteId,
    RouteMode,
    build_route_features,
    conservative_depth_reduce,
    decide_route_mode,
    project_endstates_to_altitude,
    quaternion_xyzw_to_matrix,
    route_signature,
    sample_poly5_candidate_states,
    sample_poly5_candidates,
    select_first_certified,
    validate_depth_trajectory,
)
from route_yopo_control_ros2 import RouteYopoController


def test_route_mode_uses_explicit_fallback_states():
    assert decide_route_mode(
        frontier_fresh=True,
        route_fresh=True,
        route_coherent=True,
        route_valid=True,
    ).mode == RouteMode.ROUTE
    assert decide_route_mode(
        frontier_fresh=True,
        route_fresh=False,
        route_coherent=False,
        route_valid=False,
    ).mode == RouteMode.FRONTIER_ONLY
    assert decide_route_mode(
        frontier_fresh=False,
        route_fresh=True,
        route_coherent=True,
        route_valid=True,
    ).mode == RouteMode.SAFETY_HOLD


def test_adapter_route_id_is_stable_and_monotonic():
    frontier = np.array([10.0, 0.0, 1.5])
    first_path = np.array([[0.0, 0.0, 1.5], [10.0, 0.0, 1.5]])
    second_path = np.array([[0.0, 0.0, 1.5], [8.0, 2.0, 1.5], [10.0, 0.0, 1.5]])
    tracker = LocalRouteId()
    first = route_signature(frontier, first_path)
    second = route_signature(frontier, second_path)
    assert tracker.observe(first) == 1
    assert tracker.observe(first) == 1
    assert tracker.observe(second) == 2
    assert tracker.observe(first) == 3


def test_route_feature_normalization_matches_training_contract():
    centers = np.array([[1.0, 0.0, 0.0], [0.0, 4.0, 0.0]], dtype=np.float32)
    radii = np.array([1.5, 6.0], dtype=np.float32)
    mask = np.array([1.0, 0.0], dtype=np.float32)
    distances = np.array([1.0, 4.0], dtype=np.float32)
    features, output_mask = build_route_features(
        centers,
        radii,
        mask,
        distances,
        np.zeros(3),
        np.eye(3),
        radius_clip_m=3.0,
    )
    np.testing.assert_allclose(features[0], [1.0, 0.0, 0.0, 0.5])
    np.testing.assert_allclose(features[1], np.zeros(4))
    np.testing.assert_array_equal(output_mask, mask)


def test_quaternion_rotation_preserves_full_xyz_axes():
    angle = np.pi / 2.0
    rotation = quaternion_xyzw_to_matrix([0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0)])
    np.testing.assert_allclose(rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1.0e-7)
    np.testing.assert_allclose(rotation @ [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], atol=1.0e-7)


def test_log_replay_selects_depth_nearest_requested_goal_offset():
    records = [
        {"kind": "goal", "stamp_ns": 1_000_000_000},
        {"kind": "depth", "stamp_ns": 2_000_000_000, "file": "depth/depth_1.pgm"},
        {"kind": "depth", "stamp_ns": 4_100_000_000, "file": "depth/depth_2.pgm"},
    ]
    selected = select_depth_record(
        records, depth_file=None, seconds_after_goal=3.0
    )
    assert selected["file"] == "depth/depth_2.pgm"
    explicit = select_depth_record(
        records, depth_file="depth_1.pgm", seconds_after_goal=99.0
    )
    assert explicit["file"] == "depth/depth_1.pgm"


def test_log_replay_fixed_altitude_projection_preserves_world_xy():
    angle = np.pi / 3.0
    rotation = quaternion_xyzw_to_matrix(
        [0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0)]
    )
    position = np.array([2.0, -3.0, 1.6])
    states = np.array([[4.0, 1.0, 2.0, 0.5, -0.2, 1.0, 0.3, 0.4, -0.7]])
    original_endpoint = position + rotation @ states[0, :3]
    projected = project_endstates_to_altitude(states, position, rotation, 1.6)
    projected_endpoint = position + rotation @ projected[0, :3]
    projected_velocity = rotation @ projected[0, 3:6]
    projected_acceleration = rotation @ projected[0, 6:9]
    np.testing.assert_allclose(projected_endpoint[:2], original_endpoint[:2], atol=1.0e-6)
    assert projected_endpoint[2] == pytest.approx(1.6)
    assert projected_velocity[2] == pytest.approx(0.0, abs=1.0e-7)
    assert projected_acceleration[2] == pytest.approx(0.0, abs=1.0e-7)


def test_poly5_candidates_have_101_xyz_samples_and_exact_boundaries():
    start_position = np.array([1.0, -2.0, 1.5])
    start_velocity = np.array([0.2, -0.1, 0.3])
    start_acceleration = np.array([0.4, 0.0, -0.2])
    endstate = np.array([[4.0, 2.0, 1.0, -0.3, 0.5, 0.2, 0.1, -0.4, 0.6]])
    trajectories = sample_poly5_candidates(
        start_position,
        start_velocity,
        start_acceleration,
        endstate,
        np.eye(3),
        segment_time_s=10.0 / 6.0,
        sample_count=101,
    )
    assert trajectories.shape == (1, 101, 3)
    np.testing.assert_allclose(trajectories[0, 0], start_position, atol=1.0e-6)
    np.testing.assert_allclose(trajectories[0, -1], start_position + endstate[0, :3], atol=2.0e-5)
    assert np.ptp(trajectories[0, :, 2]) > 0.1


def test_poly5_control_derivatives_match_start_and_end_boundaries():
    start_position = np.array([0.0, 0.0, 1.5])
    start_velocity = np.array([0.4, -0.2, 0.1])
    start_acceleration = np.array([0.3, 0.1, -0.2])
    endstate = np.array([[5.0, 1.0, 0.5, 0.2, 0.4, -0.1, -0.3, 0.2, 0.6]])
    positions, velocities, accelerations = sample_poly5_candidate_states(
        start_position,
        start_velocity,
        start_acceleration,
        endstate,
        np.eye(3),
        segment_time_s=10.0 / 6.0,
        sample_count=101,
    )
    np.testing.assert_allclose(positions[0, 0], start_position, atol=1.0e-6)
    np.testing.assert_allclose(velocities[0, 0], start_velocity, atol=1.0e-6)
    np.testing.assert_allclose(accelerations[0, 0], start_acceleration, atol=1.0e-5)
    np.testing.assert_allclose(velocities[0, -1], endstate[0, 3:6], atol=2.0e-5)
    np.testing.assert_allclose(accelerations[0, -1], endstate[0, 6:9], atol=5.0e-5)


def test_score_collision_uses_next_certified_candidate():
    trajectories = np.zeros((3, 101, 3), dtype=np.float32)
    trajectories[:, :, 2] = 1.5
    selected = select_first_certified(
        np.array([0.1, 0.2, 0.3]),
        trajectories,
        ["INVALID", "CERTIFIED", "CERTIFIED"],
        minimum_altitude_m=0.25,
    )
    assert selected == 1


def test_depth_gate_skips_blocked_lowest_score_for_open_candidate():
    depth = np.full((96, 160), 20.0, dtype=np.float32)
    depth[:, 76:84] = 3.0
    forward = np.linspace(0.0, 5.0, 101)
    blocked = np.column_stack((forward, np.zeros_like(forward), np.ones_like(forward)))
    open_path = np.column_stack(
        (forward, np.minimum(0.5 * forward, 1.0), np.ones_like(forward))
    )
    trajectories = np.stack((blocked, open_path))
    query = DepthSafeVolumeQuery(
        depth,
        robot_radius_m=0.3,
        safety_margin_m=0.2,
        max_unknown_fraction=0.2,
    )
    safety = [
        validate_depth_trajectory(
            query,
            trajectory,
            np.array([0.0, 0.0, 1.0]),
            np.eye(3),
            minimum_altitude_m=0.25,
        )
        for trajectory in trajectories
    ]

    selected = select_first_certified(
        np.array([0.1, 0.2]),
        trajectories,
        [item["state"] for item in safety],
        minimum_altitude_m=0.25,
    )

    assert [item["state"] for item in safety] == ["INVALID", "CERTIFIED"]
    assert selected == 1


def test_all_unsafe_or_nonfinite_candidates_produce_no_selection():
    trajectories = np.zeros((3, 101, 3), dtype=np.float32)
    trajectories[:, :, 2] = 1.5
    trajectories[2, 50, 0] = np.nan
    selected = select_first_certified(
        np.array([0.1, 0.2, 0.05]),
        trajectories,
        ["INVALID", "UNVALIDATED", "CERTIFIED"],
        minimum_altitude_m=0.25,
    )
    assert selected is None


def test_dense_depth_gate_certifies_free_path_and_rejects_obstacle():
    adapter = RouteYopoController.__new__(RouteYopoController)
    adapter.args = SimpleNamespace(minimum_altitude=0.25)
    trajectory = np.column_stack(
        (np.linspace(0.0, 5.0, 101), np.zeros(101), np.ones(101))
    )
    position = np.array([0.0, 0.0, 1.0])
    free_query = DepthSafeVolumeQuery(
        np.full((96, 160), 20.0, dtype=np.float32),
        robot_radius_m=0.3,
        safety_margin_m=0.2,
        max_unknown_fraction=0.2,
    )
    blocked_query = DepthSafeVolumeQuery(
        np.full((96, 160), 2.0, dtype=np.float32),
        robot_radius_m=0.3,
        safety_margin_m=0.2,
        max_unknown_fraction=0.2,
    )
    assert adapter._validate_trajectory(free_query, trajectory, position, np.eye(3))["state"] == "CERTIFIED"
    assert adapter._validate_trajectory(blocked_query, trajectory, position, np.eye(3))["state"] == "INVALID"


def test_depth_gate_rejects_route_altitude_band_violation_before_depth_query():
    trajectory = np.array([[0.0, 0.0, 1.6], [1.0, 0.0, 1.91]])
    result = validate_depth_trajectory(
        SimpleNamespace(),
        trajectory,
        np.zeros(3),
        np.eye(3),
        minimum_altitude_m=0.25,
        route_altitude_m=1.6,
        route_altitude_tolerance_m=0.25,
    )
    assert result["state"] == "ROUTE_ALTITUDE"
    assert result["maximum_altitude_error_m"] == pytest.approx(0.31)


def test_route_contract_accepts_only_fixed_altitude_path():
    adapter = RouteYopoController.__new__(RouteYopoController)
    adapter.args = SimpleNamespace(
        route_planarity_tolerance=0.05,
        route_start_tolerance=1.5,
        route_terminal_tolerance=2.0,
        minimum_route_length=0.5,
        robot_radius=0.3,
        safety_margin=0.2,
    )
    position = np.array([0.0, 0.0, 1.6])
    frontier = np.array([5.0, 0.0, 1.6])
    fixed = np.array([[0.0, 0.0, 1.6], [5.0, 0.0, 1.6]])
    result = adapter._validate_route(fixed, frontier, position, 1.0)
    assert result[0]
    assert result[1] == "valid_fixed_altitude_route"
    assert result[4] == pytest.approx(1.6)

    nonplanar = fixed.copy()
    nonplanar[1, 2] = 1.8
    result = adapter._validate_route(nonplanar, frontier, position, 1.0)
    assert not result[0]
    assert result[1] == "path_not_fixed_altitude"


def test_safety_depth_reduction_keeps_nearest_obstacle_and_unknown():
    adapter = RouteYopoController.__new__(RouteYopoController)
    adapter.image_height = 96
    adapter.image_width = 160
    source = np.full((192, 320), 20.0, dtype=np.float32)
    source[10:12, 20:22] = 2.0
    source[20, 40] = np.nan
    reduced = adapter._safety_depth(source)
    assert reduced.shape == (96, 160)
    assert reduced[5, 10] == 2.0
    assert np.isnan(reduced[10, 20])

    shared = conservative_depth_reduce(source, 96, 160)
    np.testing.assert_array_equal(shared, reduced)


def test_control_node_owns_the_existing_trajectory_topic_without_importing_old_planner():
    source = Path(__file__).resolve().parents[1] / "route_yopo_control_ros2.py"
    text = source.read_text(encoding="utf-8")
    assert "/scalenav/trajectory_point" in text
    assert "colosseum_interfaces" not in text


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def error(self, _message):
        pass

    def warning(self, _message):
        pass


def _control_fixture(trajectory):
    controller = SimpleNamespace()
    controller.args = SimpleNamespace(
        control_topic="/scalenav/trajectory_point",
        odom_timeout=0.5,
        control_timeout=0.5,
        max_yaw_rate=1.5,
        minimum_altitude=0.25,
    )
    controller.lock = threading.RLock()
    odom = Odometry()
    odom.pose.pose.position.z = 1.5
    odom.pose.pose.orientation.w = 1.0
    now = time.monotonic()
    controller.odom_record = (odom, now)
    controller.last_depth_monotonic = now
    controller.control_trajectory = trajectory
    controller.segment_time_s = 10.0 / 6.0
    controller.planned_yaw = 0.0
    controller.control_conflict = False
    controller.control_executing = trajectory is not None
    controller.control_armed = True
    controller.last_status = {"mode": "ROUTE", "control_state": "ACTIVE"}
    controller.command_pub = _CapturePublisher()
    controller.count_publishers = lambda _topic: 1
    controller.get_logger = lambda: _Logger()
    controller.route_ids = LocalRouteId()
    controller._hold_status = lambda reason, **extra: RouteYopoController._hold_status(
        controller, reason, **extra
    )
    return controller


def test_control_timer_publishes_certified_trajectory_state():
    positions = np.column_stack(
        (np.linspace(0.0, 5.0, 101), np.zeros(101), np.full(101, 1.5))
    )
    velocities = np.tile([3.0, 0.0, 0.0], (101, 1))
    accelerations = np.zeros((101, 3))
    controller = _control_fixture(
        (positions, velocities, accelerations, time.monotonic())
    )
    RouteYopoController.publish_control(controller)
    assert len(controller.command_pub.messages) == 1
    command = controller.command_pub.messages[0]
    assert command.transforms[0].translation.z == 1.5
    assert command.velocities[0].linear.x == 3.0


def test_control_timer_publishes_position_hold_without_safe_trajectory():
    controller = _control_fixture(None)
    RouteYopoController.publish_control(controller)
    assert len(controller.command_pub.messages) == 1
    command = controller.command_pub.messages[0]
    assert command.transforms[0].translation.z == 1.5
    assert command.velocities[0].linear.x == 0.0
    assert command.velocities[0].linear.y == 0.0
    assert command.velocities[0].linear.z == 0.0


def test_control_timer_does_not_override_preflight_before_first_frontier():
    controller = _control_fixture(None)
    controller.control_armed = False
    RouteYopoController.publish_control(controller)
    assert controller.command_pub.messages == []


def test_control_timer_rejects_second_control_publisher():
    controller = _control_fixture(None)
    controller.count_publishers = lambda _topic: 2
    RouteYopoController.publish_control(controller)
    assert controller.command_pub.messages == []
    assert controller.control_conflict
    assert controller.last_status["reason"] == "control_publisher_conflict"
