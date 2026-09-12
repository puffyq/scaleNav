from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry

from online_planner_ros2 import OnlinePlanner


def make_planner() -> OnlinePlanner:
    planner = OnlinePlanner.__new__(OnlinePlanner)
    planner.lock = threading.Lock()
    planner.args = SimpleNamespace(
        plan_from_reference=True,
        original_goal_input=True,
        minimum_trajectory_altitude=0.15,
        altitude_margin=0.10,
        control=True,
    )
    planner.desired_position_world = np.array([2.0, 3.0, 1.6])
    planner.desired_velocity_world = np.array([1.0, 0.5, 0.0])
    planner.desired_acceleration_world = np.array([0.2, 0.1, 0.0])
    planner.velocity_world = np.zeros(3)
    planner.acceleration_world = np.zeros(3)
    planner.segment_time = 10.0 / 6.0
    planner.maximum_trajectory_speed_mps = 6.0
    planner.fixed_altitude = False
    planner.goal_world = None
    planner.planned_yaw = 0.0
    planner.frame_index = 1
    planner.polynomials = None
    planner.control_period_s = 0.02
    planner.trajectory_control_time = 0.75
    planner.publish_path = lambda _polynomials: None
    planner.emit_event = lambda _event, _payload: None
    return planner


def test_reference_planning_uses_latest_desired_state_without_odom_reset():
    planner = make_planner()
    odom = Odometry()
    odom.pose.pose.position.x = 100.0
    odom.pose.pose.position.y = -100.0

    position, velocity, acceleration = planner.reference_state(odom, 123.0)

    np.testing.assert_allclose(position, planner.desired_position_world)
    np.testing.assert_allclose(velocity, planner.desired_velocity_world)
    np.testing.assert_allclose(acceleration, planner.desired_acceleration_world)


def test_selected_poly5_executes_without_runtime_time_scaling():
    planner = make_planner()
    odom = Odometry()
    odom.pose.pose.orientation.w = 1.0
    state = np.array([10.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    planner.update_trajectory(
        odom,
        np.eye(3),
        state,
        0,
        np.zeros(1),
        planner.desired_position_world,
        planner.desired_velocity_world,
        planner.desired_acceleration_world,
    )

    assert planner.trajectory_time_scale == 1.0
    assert planner.trajectory_duration == planner.segment_time
    assert planner.trajectory_control_time == 0.0
    np.testing.assert_allclose(
        [polynomial.get_velocity(0.0) for polynomial in planner.polynomials],
        planner.desired_velocity_world,
    )


def test_trajectory_install_refreshes_stale_reference_under_lock():
    planner = make_planner()
    odom = Odometry()
    odom.pose.pose.orientation.w = 1.0
    state = np.array([10.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    stale_position = np.array([-10.0, -10.0, 1.6])
    stale_velocity = np.zeros(3)
    stale_acceleration = np.zeros(3)

    planner.update_trajectory(
        odom,
        np.eye(3),
        state,
        0,
        np.zeros(1),
        stale_position,
        stale_velocity,
        stale_acceleration,
    )

    np.testing.assert_allclose(
        [polynomial.get_position(0.0) for polynomial in planner.polynomials],
        planner.desired_position_world,
    )
    np.testing.assert_allclose(
        [polynomial.get_velocity(0.0) for polynomial in planner.polynomials],
        planner.desired_velocity_world,
    )
    assert planner.trajectory_control_time == 0.0
