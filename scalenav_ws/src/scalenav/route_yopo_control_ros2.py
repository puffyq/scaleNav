#!/usr/bin/env python3
"""Original YOPO-Simple with route-constrained MPC post-processing."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Point, PoseStamped, Transform, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from graph.depth_query import DepthSafeVolumeQuery
from route_yopo_control_core import (
    LocalRouteId,
    RouteMode,
    build_route_features,
    clip_goal_to_camera_fov,
    enforce_route_progress,
    conservative_depth_reduce,
    decide_route_mode,
    quaternion_xyzw_to_matrix,
    project_endstates_to_altitude,
    reanchor_route_path,
    trim_route_for_motion,
    route_signature,
    route_timestamps_coherent,
    sample_poly5_candidate_states,
    select_first_certified,
    validate_depth_trajectory,
    validate_route_corridor,
    world_to_body_flu,
)
from yopo_inference_scaling import YopoInferenceScaling


def _stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _point(values: np.ndarray) -> Point:
    message = Point()
    message.x, message.y, message.z = (float(value) for value in values)
    return message


def _quaternion_yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RouteYopoController(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("scalenav_route_yopo_controller")
        self.args = args
        self.lock = threading.RLock()
        self.inference_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()
        self.last_plan_monotonic = 0.0
        self.route_ids = LocalRouteId()
        self.odom_record: tuple[Odometry, float] | None = None
        self.path_record: tuple[np.ndarray, float, float] | None = None
        self.frontier_record: tuple[np.ndarray, float, float] | None = None
        self.bubble_record: tuple[np.ndarray, np.ndarray, float, float] | None = None
        self.clearance_record: tuple[float, float, float] | None = None
        self.previous_velocity_world: np.ndarray | None = None
        self.previous_velocity_time: float | None = None
        self.acceleration_world = np.zeros(3, dtype=np.float32)
        self.last_depth_monotonic = 0.0
        self.control_trajectory: (
            tuple[np.ndarray, np.ndarray, np.ndarray, float, float] | None
        ) = None
        self.last_trajectory_replace_monotonic = 0.0
        self.planned_yaw = 0.0
        self.control_conflict = False
        self.control_executing = False
        self.control_armed = False
        self.mpc = None
        self.mpc_context = None
        self.mpc_enabled = bool(getattr(args, "ordered_bubble_mpc", False))
        self.mpc_last_status: list[int] = []
        self.inference_times_ms: deque[float] = deque(maxlen=4096)
        self.last_status: dict[str, Any] = self._hold_status("waiting_for_inputs")

        train_root = Path(args.train_root).expanduser().resolve()
        # The executable lives beside the legacy online ``config/data/policy``
        # packages, so Python puts that directory at sys.path[0]. Route-YOPO
        # must resolve all three packages from the training tree as one unit.
        train_path = str(train_root)
        sys.path[:] = [entry for entry in sys.path if entry != train_path]
        sys.path.insert(0, train_path)
        from config.config import cfg
        from data.route_contract import sample_route_bubbles
        self.cfg = cfg
        self.sample_route_bubbles = sample_route_bubbles
        self.anchors = np.asarray(cfg["route_anchor_distances_m"], dtype=np.float32)
        self.route_radius_clip_m = float(cfg["route_clearance_clip_m"])
        self.training_segment_time_s = float(cfg["sgm_time"])
        self.image_width = int(cfg["image_width"])
        self.image_height = int(cfg["image_height"])
        self.device = self._resolve_device(args.device)

        checkpoint_path = Path(args.model).expanduser().resolve()
        self.model = torch.jit.load(checkpoint_path, map_location=self.device).eval()
        self.feature_order = "yopo_simple_original_v1"
        self.yopo_scaling = YopoInferenceScaling(
            training_speed_mps=6.0,
            training_acceleration_mps2=6.0,
            inference_speed_mps=float(args.maximum_speed),
            base_segment_time_s=self.training_segment_time_s,
        )
        self.segment_time_s = self.yopo_scaling.segment_time_s
        if self.mpc_enabled:
            try:
                from mpc.ordered_bubble_ocp import (
                    OrderedBubbleMPC,
                    OrderedBubbleMPCConfig,
                )

                self.mpc = OrderedBubbleMPC(
                    OrderedBubbleMPCConfig(
                        horizon_steps=max(
                            12,
                            int(
                                round(
                                    12
                                    * self.segment_time_s
                                    / self.training_segment_time_s
                                )
                            ),
                        ),
                        horizon_time_s=self.segment_time_s,
                        max_velocity_mps=float(args.maximum_speed),
                        max_acceleration_mps2=float(args.acceleration_limit),
                        max_jerk_mps3=40.0,
                    ),
                    batch_size=1,
                    model_name="route_yopo_ordered_bubble_online",
                )
                self.get_logger().info("ordered-bubble MPC enabled")
            except Exception as error:
                self.mpc = None
                self.mpc_enabled = False
                self.get_logger().error("ordered-bubble MPC disabled: %s" % error)
        self._warm_up()

        self.path_pub = self.create_publisher(
            PathMsg, "/scalenav/route_yopo/planned_path", 10
        )
        self.candidate_pub = self.create_publisher(
            MarkerArray, "/scalenav/route_yopo/candidates", 10
        )
        self.status_pub = self.create_publisher(
            String, "/scalenav/route_yopo/status", 10
        )
        self.route_pub = self.create_publisher(
            String, "/scalenav/route_yopo/route_condition", 10
        )
        self.mpc_path_pub = self.create_publisher(
            PathMsg, args.mpc_path_topic, 10
        )
        self.mpc_bubble_pub = self.create_publisher(
            MarkerArray, args.mpc_bubble_topic, 10
        )
        self.command_pub = self.create_publisher(
            MultiDOFJointTrajectoryPoint, args.control_topic, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            args.odom_topic,
            self.on_odometry,
            20,
            callback_group=self.callback_group,
        )
        self.depth_sub = self.create_subscription(
            Image,
            args.depth_topic,
            self.on_depth,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.path_sub = self.create_subscription(
            PathMsg,
            args.path_topic,
            self.on_path,
            10,
            callback_group=self.callback_group,
        )
        self.graph_sub = self.create_subscription(
            MarkerArray,
            args.graph_topic,
            self.on_graph,
            10,
            callback_group=self.callback_group,
        )
        self.bubble_sub = self.create_subscription(
            MarkerArray,
            args.bubble_topic,
            self.on_bubbles,
            10,
            callback_group=self.callback_group,
        )
        self.clearance_sub = self.create_subscription(
            Vector3Stamped,
            args.clearance_topic,
            self.on_clearance,
            10,
            callback_group=self.callback_group,
        )
        self.status_timer = self.create_timer(
            2.0, self.publish_latest_status, callback_group=self.callback_group
        )
        self.control_timer = self.create_timer(
            0.02, self.publish_control, callback_group=self.callback_group
        )
        self.get_logger().info(
            "Route-YOPO controller ready: model=%s device=%s feature_order=%s "
            "control_output=%s@50Hz route_source=compat_non_atomic"
            % (checkpoint_path, self.device, self.feature_order, args.control_topic)
        )

    def _resolve_device(self, requested: str) -> torch.device:
        if requested == "auto":
            return torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("--device cuda requested but CUDA is unavailable")
        return torch.device(requested)

    @torch.inference_mode()
    def _warm_up(self) -> None:
        depth = torch.ones(
            (1, 1, self.image_height, self.image_width), device=self.device
        )
        observation = torch.zeros((1, 9), device=self.device)
        observation[:, 6] = 10.0
        model_output, score = self.model(depth, observation)
        if tuple(model_output.shape) != (1, 9, 3, 5):
            raise ValueError(
                "YOPO-Simple output contract mismatch: "
                f"{tuple(model_output.shape)}, {tuple(score.shape)}"
            )
        if not torch.isfinite(model_output).all() or not torch.isfinite(score).all():
            raise FloatingPointError("Route-YOPO warm-up produced non-finite output")

    def on_odometry(self, message: Odometry) -> None:
        now = time.monotonic()
        velocity = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ],
            dtype=np.float32,
        )
        orientation = message.pose.pose.orientation
        rotation = quaternion_xyzw_to_matrix(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        if self.args.odom_twist_frame == "body":
            velocity = (rotation @ velocity).astype(np.float32)
        with self.lock:
            first_odometry = self.odom_record is None
            if self.previous_velocity_world is not None and self.previous_velocity_time is not None:
                elapsed = now - self.previous_velocity_time
                if 0.002 <= elapsed <= 0.2:
                    measured = np.clip(
                        (velocity - self.previous_velocity_world) / elapsed,
                        -float(self.args.acceleration_limit),
                        float(self.args.acceleration_limit),
                    )
                    self.acceleration_world = (
                        0.85 * self.acceleration_world + 0.15 * measured
                    ).astype(np.float32)
            self.previous_velocity_world = velocity
            self.previous_velocity_time = now
            self.odom_record = (message, now)
            if first_odometry:
                self.planned_yaw = _quaternion_yaw(message.pose.pose.orientation)

    def on_path(self, message: PathMsg) -> None:
        points = np.asarray(
            [
                [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                for pose in message.poses
            ],
            dtype=np.float32,
        )
        if points.size == 0:
            points = np.empty((0, 3), dtype=np.float32)
        with self.lock:
            self.path_record = (points, time.monotonic(), _stamp_seconds(message.header.stamp))

    def on_graph(self, message: MarkerArray) -> None:
        found: tuple[np.ndarray, float] | None = None
        deleted = False
        for marker in message.markers:
            if marker.ns != "scalenav_frontier_goal":
                continue
            if marker.action == Marker.ADD:
                values = np.array(
                    [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
                    dtype=np.float32,
                )
                if np.isfinite(values).all():
                    found = (values, _stamp_seconds(marker.header.stamp))
            elif marker.action in (Marker.DELETE, Marker.DELETEALL):
                deleted = True
        with self.lock:
            if found is not None:
                self.frontier_record = (found[0], time.monotonic(), found[1])
                self.control_armed = True
            elif deleted:
                self.frontier_record = None

    def on_bubbles(self, message: MarkerArray) -> None:
        bubble_centers: list[np.ndarray] = []
        bubble_radii: list[float] = []
        source_stamp = 0.0
        for marker in message.markers:
            if marker.ns == "scalenav_route_bubble_radius" and marker.action == Marker.ADD:
                center = np.array(
                    [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
                    dtype=np.float32,
                )
                radius = 0.5 * float(marker.scale.x)
                if np.isfinite(center).all() and np.isfinite(radius) and radius > 0.0:
                    bubble_centers.append(center)
                    bubble_radii.append(radius)
                    source_stamp = _stamp_seconds(marker.header.stamp)
        with self.lock:
            if bubble_centers:
                self.bubble_record = (
                    np.asarray(bubble_centers, dtype=np.float32),
                    np.asarray(bubble_radii, dtype=np.float32),
                    time.monotonic(),
                    source_stamp,
                )

    def _route_local_radii(
        self, route_path: np.ndarray, fallback: float, route_stamp: float
    ) -> np.ndarray:
        """Match route vertices to the latest raw topology bubble radii."""
        with self.lock:
            record = self.bubble_record
        if (
            record is None
            or time.monotonic() - record[2] > self.args.route_timeout
            or abs(float(route_stamp) - record[3]) > self.args.compat_stamp_slop
        ):
            return np.full(len(route_path), fallback, dtype=np.float32)
        centers, radii, _, _ = record
        distances = np.linalg.norm(route_path[:, None, :] - centers[None, :, :], axis=2)
        nearest = np.argmin(distances, axis=1)
        nearest_distance = distances[np.arange(len(route_path)), nearest]
        # A route node is expected to be a topology bubble center. Reject
        # unrelated bubbles instead of assigning a distant radius.
        matched = nearest_distance <= 1.0
        output = np.full(len(route_path), fallback, dtype=np.float32)
        # ScaleNav publishes the topology bubble radius as the route geometry
        # to constrain.  Keep that original radius intact; vehicle-size
        # handling belongs to the independent depth collision certification.
        output[matched] = radii[nearest[matched]]
        return np.clip(output, 0.05, self.route_radius_clip_m)

    def on_clearance(self, message: Vector3Stamped) -> None:
        with self.lock:
            self.clearance_record = (
                float(message.vector.y),
                time.monotonic(),
                _stamp_seconds(message.header.stamp),
            )

    @staticmethod
    def decode_depth(message: Image) -> np.ndarray:
        if message.encoding == "32FC1":
            dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
            item_size = 4
            scale = 1.0
        elif message.encoding == "16UC1":
            dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
            item_size = 2
            scale = 0.001
        else:
            raise ValueError(f"unsupported depth encoding: {message.encoding!r}")
        row_values = message.step // item_size
        values = np.frombuffer(message.data, dtype=dtype)
        expected = row_values * message.height
        if values.size < expected:
            raise ValueError("depth payload is shorter than height*step")
        return (
            values[:expected].reshape(message.height, row_values)[:, : message.width]
            .astype(np.float32)
            * scale
        )

    def _model_depth(self, raw_depth_m: np.ndarray) -> np.ndarray:
        resized = cv2.resize(
            raw_depth_m,
            (self.image_width, self.image_height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(np.float32)
        normalized = np.minimum(resized, self.args.max_depth) / self.args.max_depth
        invalid = ~np.isfinite(normalized) | (
            normalized < self.args.minimum_depth / self.args.max_depth
        )
        return cv2.inpaint(
            np.uint8(np.nan_to_num(normalized, nan=0.0, posinf=1.0, neginf=0.0) * 255.0),
            np.uint8(invalid),
            1,
            cv2.INPAINT_NS,
        ).astype(np.float32) / 255.0

    def _safety_depth(self, raw_depth_m: np.ndarray) -> np.ndarray:
        return conservative_depth_reduce(
            raw_depth_m, self.image_height, self.image_width
        )

    def on_depth(self, message: Image) -> None:
        now = time.monotonic()
        # Sensor freshness is independent of the planning rate. In particular,
        # a low replan rate must not make the 50 Hz controller treat a healthy
        # depth stream as stale between inference ticks.
        with self.lock:
            self.last_depth_monotonic = now
        if now - self.last_plan_monotonic < 1.0 / self.args.update_rate:
            return
        if not self.inference_lock.acquire(blocking=False):
            return
        self.last_plan_monotonic = now
        try:
            self._plan(message, now)
        except Exception as error:
            self.get_logger().error(f"Route-YOPO control tick failed: {error}")
            self._publish_hold("adapter_exception", detail=str(error))
        finally:
            self.inference_lock.release()

    @torch.inference_mode()
    def _plan(self, depth_message: Image, now: float) -> None:
        raw_depth = self.decode_depth(depth_message)
        with self.lock:
            odom_record = self.odom_record
            path_record = getattr(self, "path_record", None)
            frontier_record = self.frontier_record
            clearance_record = self.clearance_record
            velocity_world = (
                None if self.previous_velocity_world is None else self.previous_velocity_world.copy()
            )
            acceleration_world = self.acceleration_world.copy()

        if odom_record is None or now - odom_record[1] > self.args.odom_timeout:
            self._publish_hold("odometry_missing_or_stale")
            return
        odom = odom_record[0]
        pose = odom.pose.pose
        position_world = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
        )
        rotation = quaternion_xyzw_to_matrix(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )
        if velocity_world is None:
            velocity_world = np.zeros(3, dtype=np.float32)

        frontier_fresh = (
            frontier_record is not None and now - frontier_record[1] <= self.args.route_timeout
        )
        route_fresh = (
            path_record is not None
            and clearance_record is not None
            and now - path_record[1] <= self.args.route_timeout
            and now - clearance_record[1] <= self.args.route_timeout
        )
        route_coherent = False
        if frontier_fresh and route_fresh:
            route_coherent = route_timestamps_coherent(
                path_record[2],
                clearance_record[2],
                frontier_record[2],
                stamp_slop_s=self.args.compat_stamp_slop,
            )

        route_valid = False
        route_reason = "route_not_available"
        route_terminal_error_m = math.nan
        route_path = None
        safe_radius_m = math.nan
        route_altitude_m = math.nan
        if frontier_fresh and route_fresh:
            route_path = path_record[0].astype(np.float64, copy=True)
            frontier_world = frontier_record[0].astype(np.float64, copy=True)
            path_clearance_m = float(clearance_record[0])
            route_terminal_error_m = float(np.linalg.norm(route_path[-1] - frontier_world)) if len(route_path) else math.nan
            (
                route_valid,
                route_reason,
                route_path,
                safe_radius_m,
                route_altitude_m,
            ) = self._validate_route(route_path, frontier_world, position_world, path_clearance_m)
        elif frontier_fresh:
            frontier_world = frontier_record[0].astype(np.float64, copy=True)
        else:
            frontier_world = None

        decision = decide_route_mode(
            frontier_fresh=frontier_fresh,
            route_fresh=route_fresh,
            route_coherent=route_coherent,
            route_valid=route_valid,
        )
        if decision.mode == RouteMode.SAFETY_HOLD or frontier_world is None:
            self._publish_hold(decision.reason)
            return

        count = len(self.anchors)
        route_id = self.route_ids.value
        centers_world = np.repeat(position_world[None], count, axis=0).astype(np.float32)
        sampled_radii = np.zeros(count, dtype=np.float32)
        route_features = np.zeros((count, 4), dtype=np.float32)
        sample_distances = self.anchors.copy()
        if decision.mode == RouteMode.ROUTE:
            route_path = trim_route_for_motion(route_path, position_world, velocity_world)
            feature_radius = max(safe_radius_m, 0.0)
            point_radii = self._route_local_radii(
                route_path, feature_radius, float(path_record[2])
            )
            centers_world, sampled_radii, sample_distances = (
                self.sample_route_bubbles(route_path, point_radii, self.anchors)
            )
            route_features = build_route_features(
                centers_world,
                sampled_radii,
                sample_distances,
                position_world,
                rotation,
                radius_clip_m=self.route_radius_clip_m,
                normalization_distance_m=10.0,
            )
            route_id = self.route_ids.observe(route_signature(frontier_world, route_path))

        frontier_body = clip_goal_to_camera_fov(
            world_to_body_flu(frontier_world, position_world, rotation),
            horizontal_fov_deg=self.args.source_horizontal_fov,
            vertical_fov_deg=self.args.source_vertical_fov,
        ).astype(np.float32)
        motion_body = np.concatenate(
            (rotation.T @ velocity_world, rotation.T @ acceleration_world)
        ).astype(np.float32)
        model_depth = self._model_depth(raw_depth)
        depth_tensor = torch.from_numpy(model_depth[None, None]).to(
            self.device, non_blocking=True
        )
        observation = np.concatenate((motion_body, frontier_body), axis=0).astype(np.float32)
        observation_tensor = torch.from_numpy(observation[None]).to(
            self.device, non_blocking=True
        )
        observation_tensor = self.yopo_scaling.model_input(observation_tensor)

        started = time.perf_counter()
        model_output, score = self.model(depth_tensor, observation_tensor)
        model_output = self.yopo_scaling.physical_endstate(model_output)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        inference_ms = (time.perf_counter() - started) * 1000.0
        self.inference_times_ms.append(inference_ms)
        if not torch.isfinite(model_output).all() or not torch.isfinite(score).all():
            self._publish_hold("model_output_non_finite")
            return

        scores = score[0].reshape(-1).detach().cpu().numpy()
        score_selected = int(np.argmin(scores))
        endstates_body = (
            model_output[0].permute(1, 2, 0).reshape(-1, 9).detach().cpu().numpy()
        )
        raw_endpoint_world = position_world[None] + endstates_body[:, :3] @ rotation.T
        if decision.mode in (RouteMode.ROUTE, RouteMode.FRONTIER_ONLY):
            endstates_body = enforce_route_progress(
                endstates_body,
                frontier_body,
                minimum_forward_m=2.0,
                maximum_forward_m=8.0,
            )
            constrained_altitude_m = (
                route_altitude_m
                if decision.mode == RouteMode.ROUTE
                else float(frontier_world[2])
            )
            endstates_body = project_endstates_to_altitude(
                endstates_body,
                position_world,
                rotation,
                constrained_altitude_m,
            )
        trajectories, trajectory_velocities, trajectory_accelerations = sample_poly5_candidate_states(
            position_world,
            velocity_world,
            acceleration_world,
            endstates_body,
            rotation,
            segment_time_s=self.segment_time_s,
            sample_count=self.args.safety_samples,
        )
        trajectory_durations = np.full(
            len(trajectories), self.segment_time_s, dtype=np.float32
        )
        # YOPO performs proposal selection.  MPC receives only the top-scored
        # terminal state; it is a constrained trajectory generator, not a
        # second selector over the whole 3x5 lattice.
        mpc_status: list[int] = [-2] * len(trajectories)
        mpc_bubble_violation_m: list[float | None] = [None] * len(trajectories)
        mpc_visual_trajectory: np.ndarray | None = None
        mpc_visual_bubble_centers: np.ndarray | None = None
        mpc_visual_bubble_radii: np.ndarray | None = None
        if self.mpc is not None and decision.mode == RouteMode.ROUTE and safe_radius_m > 0.0:
            try:
                from mpc.ordered_bubble_ocp import (
                    maximum_bubble_violation,
                    maximum_reachable_progress,
                    resolve_target_progress,
                    sample_reachable_stage_bubbles,
                )

                route_segments = np.linalg.norm(np.diff(route_path, axis=0), axis=1)
                initial_world = np.concatenate(
                    (position_world, velocity_world, acceleration_world)
                ).astype(np.float64)
                terminal_world_all = np.concatenate(
                    (
                        position_world[None] + endstates_body[:, :3] @ rotation.T,
                        endstates_body[:, 3:6] @ rotation.T,
                        endstates_body[:, 6:9] @ rotation.T,
                    ),
                    axis=1,
                ).astype(np.float64)
                terminal_world = terminal_world_all[score_selected : score_selected + 1]
                first_segment_index = int(np.flatnonzero(route_segments > 1.0e-6)[0])
                route_tangent = (
                    route_path[first_segment_index + 1] - route_path[first_segment_index]
                ) / route_segments[first_segment_index]
                initial_forward_speed = max(0.0, float(np.dot(velocity_world, route_tangent)))
                reachable_progress = maximum_reachable_progress(
                    horizon_time_s=self.mpc.config.horizon_time_s,
                    initial_speed_mps=initial_forward_speed,
                    max_velocity_mps=self.mpc.config.max_velocity_mps,
                    max_acceleration_mps2=self.mpc.config.max_acceleration_mps2,
                )
                target_progress, _ = resolve_target_progress(
                    terminal_world[0, :3], route_path,
                    reachable_progress_m=reachable_progress,
                )
                stage_centers, stage_radii, _ = sample_reachable_stage_bubbles(
                    route_path,
                    point_radii.astype(np.float64),
                    horizon_steps=self.mpc.config.horizon_steps,
                    horizon_time_s=self.mpc.config.horizon_time_s,
                    initial_speed_mps=initial_forward_speed,
                    max_velocity_mps=self.mpc.config.max_velocity_mps,
                    max_acceleration_mps2=self.mpc.config.max_acceleration_mps2,
                    target_progress_m=target_progress,
                )
                mpc_context, _, mpc_states, _, _ = self.mpc(
                    initial_world[None],
                    terminal_world,
                    stage_centers[None],
                    stage_radii[None],
                    context=self.mpc_context,
                )
                self.mpc_context = mpc_context
                top1_status = int(np.asarray(mpc_context.status).reshape(-1)[0])
                mpc_status[score_selected] = top1_status
                mpc_nodes = mpc_states.detach().cpu().numpy()
                # Keep the exact candidate and ordered bubbles used by MPC
                # for RViz, including a soft-constraint solution that is later
                # rejected by the explicit bubble-violation check.
                mpc_visual_trajectory = mpc_nodes[0, :, :3].copy()
                mpc_visual_bubble_centers = stage_centers.copy()
                mpc_visual_bubble_radii = stage_radii.copy()
                top1_violation = maximum_bubble_violation(
                    mpc_nodes[0, :, :3], stage_centers, stage_radii
                )
                mpc_bubble_violation_m[score_selected] = top1_violation
                if (
                    top1_status == 0
                    and top1_violation
                    > self.mpc.config.maximum_accepted_bubble_violation_m
                ):
                    # acados status 0 only means the softened NLP converged.
                    # It does not mean the safety set was actually respected.
                    top1_status = -3
                    mpc_status[score_selected] = top1_status
                node_times = np.linspace(0.0, self.segment_time_s, mpc_nodes.shape[1])
                sample_times = np.linspace(0.0, self.segment_time_s, self.args.safety_samples)
                if top1_status == 0:
                    for component in range(3):
                        trajectories[score_selected, :, component] = np.interp(
                            sample_times, node_times, mpc_nodes[0, :, component]
                        )
                        trajectory_velocities[score_selected, :, component] = np.interp(
                            sample_times, node_times, mpc_nodes[0, :, 3 + component]
                        )
                        trajectory_accelerations[score_selected, :, component] = np.interp(
                            sample_times, node_times, mpc_nodes[0, :, 6 + component]
                        )
            except Exception as error:
                self.mpc_context = None
                mpc_status[score_selected] = -1
                self.get_logger().warning("ordered-bubble MPC solve failed: %s" % error)
        self.mpc_last_status = mpc_status
        safety_depth = self._safety_depth(raw_depth)
        query = DepthSafeVolumeQuery(
            safety_depth,
            horizontal_fov_deg=self.args.source_horizontal_fov,
            vertical_fov_deg=self.args.source_vertical_fov,
            robot_radius_m=self.args.robot_radius,
            safety_margin_m=self.args.safety_margin,
            sample_step_m=self.args.safety_sample_step,
            far_depth_m=self.args.max_depth,
            # Ordered bubbles constrain the part of an MPC trajectory that
            # leaves the forward camera FOV. Allow a small additional unknown
            # fraction only for that mode; non-MPC trajectories keep the
            # stricter online depth gate.
            max_unknown_fraction=max(
                self.args.max_unknown_fraction,
                0.25 if self.mpc is not None and decision.mode == RouteMode.ROUTE else 0.0,
            ),
        )
        safety = [
            self._validate_trajectory(
                query,
                trajectory,
                position_world,
                rotation,
                route_altitude_m=constrained_altitude_m,
                route_path_world=route_path if decision.mode == RouteMode.ROUTE and safe_radius_m > 0.0 else None,
                route_safe_radius_m=safe_radius_m if decision.mode == RouteMode.ROUTE and safe_radius_m > 0.0 else None,
            )
            for trajectory in trajectories
        ]
        for index, item in enumerate(safety):
            speed_peak = float(np.linalg.norm(trajectory_velocities[index], axis=1).max())
            acceleration_peak = float(np.linalg.norm(trajectory_accelerations[index], axis=1).max())
            jerk_peak = (
                float(
                    np.linalg.norm(
                        np.diff(trajectory_accelerations[index], axis=0), axis=1
                    ).max()
                )
                / max(
                    float(trajectory_durations[index])
                    / max(len(trajectory_accelerations[index]) - 1, 1),
                    1.0e-6,
                )
                if len(trajectory_accelerations[index]) > 1
                else 0.0
            )
            item.update(
                maximum_speed_mps=speed_peak,
                maximum_acceleration_mps2=acceleration_peak,
                maximum_jerk_mps3=jerk_peak,
            )
        safety_states = [item["state"] for item in safety]
        top1_mpc_required = self.mpc is not None and decision.mode == RouteMode.ROUTE
        if top1_mpc_required:
            selected = (
                score_selected
                if mpc_status[score_selected] == 0
                and safety_states[score_selected] == "CERTIFIED"
                else None
            )
        else:
            selected = select_first_certified(
                scores,
                trajectories,
                safety_states,
                minimum_altitude_m=self.args.minimum_altitude,
            )
        certified_count = safety_states.count("CERTIFIED")
        invalid_count = safety_states.count("INVALID")
        unvalidated_count = safety_states.count("UNVALIDATED")
        altitude_count = sum(
            state in ("ALTITUDE", "ROUTE_ALTITUDE") for state in safety_states
        )
        selected_end = (
            trajectories[selected, -1]
            if selected is not None
            else np.full(3, np.nan, dtype=np.float32)
        )
        selected_safety = safety[selected] if selected is not None else {}
        selected_clearance = selected_safety.get("minimum_clearance_m")
        selected_known_fraction = selected_safety.get("known_fraction")
        maximum_known_fraction = max(
            (float(item.get("known_fraction", 0.0) or 0.0) for item in safety),
            default=0.0,
        )
        self.get_logger().info(
            "[Route-YOPO safety] mode=%s route_reason=%s route_id=%d "
            "selected=%s score_selected=%d "
            "states=certified:%d invalid:%d unvalidated:%d altitude:%d "
            "selected_end=(%.2f,%.2f,%.2f) selected_min_clearance=%s "
            "selected_known_fraction=%s max_known_fraction=%.3f"
            % (
                decision.mode.value,
                decision.reason,
                route_id,
                "none" if selected is None else str(selected),
                score_selected,
                certified_count,
                invalid_count,
                unvalidated_count,
                altitude_count,
                float(selected_end[0]),
                float(selected_end[1]),
                float(selected_end[2]),
                "none" if selected_clearance is None else f"{float(selected_clearance):.3f}",
                "none"
                if selected_known_fraction is None
                else f"{float(selected_known_fraction):.3f}",
                maximum_known_fraction,
            )
        )
        output_mode = decision.mode if selected is not None else RouteMode.SAFETY_HOLD
        output_reason = decision.reason if selected is not None else "no_certified_candidate"

        route_contract = {
            "contract": "RouteCondition-compatible diagnostic v1",
            "source": "scalenav_compat_non_atomic",
            "atomic": False,
            "route_id": route_id,
            "mode": decision.mode.value,
            "frontier_world": frontier_world.tolist(),
            "centers_world": centers_world.tolist(),
            "safe_radii_m": sampled_radii.tolist(),
            "anchor_or_feature_distances_m": sample_distances.tolist(),
            "radius_source": "scalenav_route_topology_bubble_or_raw_clearance",
            "radius_vehicle_subtraction_m": 0.0,
            "path_safe_radius_m": safe_radius_m if math.isfinite(safe_radius_m) else None,
            "route_validation": route_reason,
            "route_terminal_error_m": route_terminal_error_m,
            "fixed_altitude_m": constrained_altitude_m,
        }
        status = {
            "controller": "route_yopo",
            "control_output": self.args.control_topic,
            "control_rate_hz": 50.0,
            "control_state": "ACTIVE" if selected is not None else "HOLD",
            "control_conflict": False,
            "mode": output_mode.value,
            "input_mode": decision.mode.value,
            "reason": output_reason,
            "route_id": route_id,
            "route_id_source": "adapter_local_monotonic",
            "route_source": "scalenav_compat_non_atomic",
            "feature_order": self.feature_order,
            "yopo_route_input": False,
            "route_validation": route_reason,
            "route_terminal_error_m": route_terminal_error_m,
            "fixed_altitude_m": constrained_altitude_m,
            "training_trajectory_duration_s": self.training_segment_time_s,
            "trajectory_duration_s": (
                self.segment_time_s
                if selected is None
                else float(trajectory_durations[selected])
            ),
            "selected_primitive": selected,
            "selected_known_fraction": selected_known_fraction,
            "selected_depth_known_fraction": selected_safety.get("depth_known_fraction"),
            "selected_depth_known_sample_fraction": selected_safety.get(
                "depth_known_sample_fraction"
            ),
            "selected_corridor_certified": selected_safety.get("corridor_certified", False),
            "selected_combined_certified": selected_safety.get("combined_certified", False),
            "selected_validation_source": selected_safety.get("validation_source"),
            "route_corridor_min_known_fraction": float(
                getattr(self.args, "route_corridor_min_known_fraction", 0.75)
            ),
            "selection_policy": "yopo_top1_then_mpc_certification",
            "trajectory_replaced": False,
            "score_selected_primitive": score_selected,
            "score_selected_certified": selected == score_selected,
            "selected_score": None if selected is None else float(scores[selected]),
            "candidate_scores": scores.tolist(),
            "raw_candidate_endpoint_z_min_max_m": [
                float(np.min(raw_endpoint_world[:, 2])),
                float(np.max(raw_endpoint_world[:, 2])),
            ],
            "constrained_candidate_endpoint_z_min_max_m": [
                float(np.min(trajectories[:, -1, 2])),
                float(np.max(trajectories[:, -1, 2])),
            ],
            "candidate_safety": safety,
            "inference_ms": inference_ms,
            "inference_p95_ms": float(np.percentile(self.inference_times_ms, 95.0)),
            "safety_samples_per_primitive": self.args.safety_samples,
            "safety_depth_shape_hw": list(safety_depth.shape),
            "mpc_enabled": self.mpc is not None,
            "mpc_submitted_primitive": score_selected if top1_mpc_required else None,
            "mpc_not_submitted_status_code": -2,
            "mpc_bubble_violation_rejected_status_code": -3,
            "mpc_status": mpc_status,
            "mpc_bubble_violation_m": mpc_bubble_violation_m,
            "trajectory_source": (
                "ordered_bubble_mpc"
                if top1_mpc_required and mpc_status[score_selected] == 0
                else "poly5"
                if not top1_mpc_required
                else "none"
            ),
        }
        self._publish_json(self.route_pub, route_contract)
        self._publish_mpc_visualization(
            mpc_visual_trajectory,
            mpc_visual_bubble_centers,
            mpc_visual_bubble_radii,
            depth_message,
        )
        if selected is None:
            with self.lock:
                # Safety certification is evaluated on an asynchronous depth
                # frame. A single frame can temporarily reject every new
                # candidate even though the currently executing trajectory was
                # certified moments earlier. Clearing it here makes the
                # controller brake to zero and restart on the next good frame,
                # which causes the observed stop-and-go motion. Keep the last
                # certified trajectory until it expires; stale sensors and
                # route/frontier failures still enter _publish_hold() above.
                if invalid_count:
                    # Never continue an old command after the newest depth
                    # frame explicitly found an occupied swept volume.
                    self.control_trajectory = None
                    active = False
                else:
                    active = (
                        self.control_trajectory is not None
                        and now - self.control_trajectory[3]
                        < min(
                            self.control_trajectory[4],
                            self.args.minimum_trajectory_hold,
                        )
                        and now - self.last_depth_monotonic <= self.args.control_timeout
                    )
                self.control_executing = active
                status["trajectory_preserved"] = active
            self._publish_json(self.status_pub, status)
            return
        with self.lock:
            self.last_status = status
        self._publish_candidates(trajectories, safety_states, selected, depth_message)
        with self.lock:
            replace = (
                self.control_trajectory is None
                or now - self.last_trajectory_replace_monotonic
                >= self.args.minimum_trajectory_hold
                or now - self.control_trajectory[3] >= self.control_trajectory[4]
            )
            if replace:
                self.control_trajectory = (
                    trajectories[selected].copy(),
                    trajectory_velocities[selected].copy(),
                    trajectory_accelerations[selected].copy(),
                    now,
                    float(trajectory_durations[selected]),
                )
                self.last_trajectory_replace_monotonic = now
            self.control_executing = True
            status["trajectory_replaced"] = replace
            self.last_status = status
        self._publish_json(self.status_pub, status)
        self._publish_path(trajectories[selected], depth_message)

    def _validate_route(
        self,
        path: np.ndarray,
        frontier: np.ndarray,
        position: np.ndarray,
        path_clearance_m: float,
    ) -> tuple[bool, str, np.ndarray, float, float]:
        if path.ndim != 2 or path.shape[1:] != (3,) or len(path) < 2:
            return False, "path_empty", path, math.nan, math.nan
        if not np.isfinite(path).all() or not np.isfinite(path_clearance_m):
            return False, "path_non_finite", path, math.nan, math.nan
        route_altitude = float(np.median(path[:, 2]))
        if float(np.ptp(path[:, 2])) > getattr(self.args, "route_planarity_tolerance", 0.05):
            return False, "path_not_fixed_altitude", path, math.nan, math.nan
        start_error = float(np.linalg.norm(path[0] - position))
        route_reason = "valid_route"
        if start_error > self.args.route_start_tolerance:
            anchored, nearest_error = reanchor_route_path(
                path,
                position,
                maximum_distance_m=float(
                getattr(self.args, "route_reanchor_tolerance", 5.0)
                ),
            )
            if anchored is None:
                return False, "path_start_discontinuous", path, math.nan, math.nan
            path = anchored
            start_error = nearest_error
        elif start_error > 1.0e-3:
            path = np.concatenate((position[None], path), axis=0)
        # Path and frontier arrive on separate topics and can be one graph tick
        # apart. Path geometry is authoritative; endpoint skew is retained in
        # status.route_terminal_error_m instead of forcing an unsafe fallback.
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        if length < self.args.minimum_route_length:
            return False, "path_too_short", path, math.nan, math.nan
        # The route constraint uses the original ScaleNav clearance directly.
        # Do not shrink it by the vehicle radius or an additional margin here.
        safe_radius = path_clearance_m
        if safe_radius <= 0.0:
            # A non-positive source clearance cannot define a route bubble.
            return False, "route_corridor_too_narrow", path, 0.0, route_altitude
        return True, "valid_fixed_altitude_route", path, safe_radius, route_altitude

    def _validate_trajectory(
        self,
        query: DepthSafeVolumeQuery,
        trajectory_world: np.ndarray,
        position_world: np.ndarray,
        rotation_body_to_world: np.ndarray,
        *,
        route_altitude_m: float | None = None,
        route_path_world: np.ndarray | None = None,
        route_safe_radius_m: float | None = None,
    ) -> dict[str, Any]:
        corridor = None
        if route_path_world is not None or route_safe_radius_m is not None:
            if route_path_world is None or route_safe_radius_m is None:
                raise ValueError("route path and safe radius must be provided together")
            corridor = validate_route_corridor(
                trajectory_world,
                route_path_world,
                route_safe_radius_m,
                max(
                    getattr(self.args, "route_corridor_tracking_tolerance", 0.0),
                    0.25 if getattr(self, "mpc", None) is not None else 0.0,
                ),
            )
        result = validate_depth_trajectory(
            query,
            trajectory_world,
            position_world
            + rotation_body_to_world
            @ np.asarray(
                getattr(self.args, "camera_translation_flu", (0.0, 0.0, 0.0)),
                dtype=np.float64,
            ),
            rotation_body_to_world,
            minimum_altitude_m=self.args.minimum_altitude,
            route_altitude_m=route_altitude_m,
            route_altitude_tolerance_m=getattr(
                self.args, "route_altitude_tolerance", 0.25
            ),
        )
        # A route corridor can cover points outside the camera FOV, but it
        # cannot replace all sensor evidence.  In particular, a trajectory
        # with known_fraction == 0 must never become publishable merely
        # because its geometry lies on the planned route.
        depth_known_fraction = float(result.get("known_fraction", 0.0) or 0.0)
        depth_known_sample_fraction = float(
            result.get("known_sample_fraction", depth_known_fraction) or 0.0
        )
        minimum_known_fraction = float(
            getattr(self.args, "route_corridor_min_known_fraction", 0.75)
        )
        corridor_certified = corridor is not None and corridor["state"] == "CERTIFIED"
        result = dict(result)
        result["depth_known_fraction"] = depth_known_fraction
        result["depth_known_sample_fraction"] = depth_known_sample_fraction
        result["corridor_certified"] = bool(corridor_certified)
        result["combined_certified"] = result["state"] == "CERTIFIED"
        result["validation_source"] = "depth"
        if (
            result["state"] == "UNVALIDATED"
            and corridor_certified
            and depth_known_sample_fraction > 0.0
            and depth_known_sample_fraction >= minimum_known_fraction
        ):
            result["state"] = "CERTIFIED"
            result["combined_certified"] = True
            result["validation_source"] = "depth_plus_route_corridor"
        if corridor is not None:
            result.update(corridor=corridor, corridor_state=corridor["state"])
        return result

    def _publish_path(self, trajectory: np.ndarray, source: Image) -> None:
        message = PathMsg()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self.args.world_frame
        for values in trajectory:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position = _point(values)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_mpc_visualization(
        self,
        trajectory_world: np.ndarray | None,
        bubble_centers_world: np.ndarray | None,
        bubble_radii_m: np.ndarray | None,
        source: Image,
    ) -> None:
        """Publish the MPC solution and the exact ordered bubbles it used."""
        path = PathMsg()
        path.header.stamp = source.header.stamp
        path.header.frame_id = self.args.world_frame
        if trajectory_world is not None:
            points = np.asarray(trajectory_world, dtype=np.float64)
            if points.ndim == 2 and points.shape[1:] == (3,) and np.isfinite(points).all():
                for values in points:
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position = _point(values)
                    pose.pose.orientation.w = 1.0
                    path.poses.append(pose)
        self.mpc_path_pub.publish(path)

        markers = MarkerArray()
        clear = Marker()
        clear.header = path.header
        clear.ns = "route_yopo_mpc_bubbles"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        centers = np.asarray(
            [] if bubble_centers_world is None else bubble_centers_world,
            dtype=np.float64,
        )
        radii = np.asarray(
            [] if bubble_radii_m is None else bubble_radii_m,
            dtype=np.float64,
        ).reshape(-1)
        if (
            centers.ndim == 2
            and centers.shape[1:] == (3,)
            and len(centers) == len(radii)
            and np.isfinite(centers).all()
            and np.isfinite(radii).all()
        ):
            order = Marker()
            order.header = path.header
            order.ns = "route_yopo_mpc_bubbles"
            order.id = 0
            order.type = Marker.LINE_STRIP
            order.action = Marker.ADD
            order.pose.orientation.w = 1.0
            order.scale.x = 0.045
            order.color.r = 1.0
            order.color.g = 0.78
            order.color.b = 0.10
            order.color.a = 0.75
            order.points = [_point(center) for center, radius in zip(centers, radii) if radius > 0.0]
            if len(order.points) >= 2:
                markers.markers.append(order)
            for index, (center, radius) in enumerate(zip(centers, radii)):
                if radius <= 0.0:
                    continue
                marker = Marker()
                marker.header = path.header
                marker.ns = "route_yopo_mpc_bubbles"
                marker.id = index + 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.pose.position = _point(center)
                diameter = float(2.0 * radius)
                marker.scale.x = diameter
                marker.scale.y = diameter
                marker.scale.z = diameter
                marker.color.r = 1.0
                marker.color.g = 0.78
                marker.color.b = 0.10
                marker.color.a = 0.22
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 0
                markers.markers.append(marker)
        self.mpc_bubble_pub.publish(markers)

    def _publish_candidates(
        self,
        trajectories: np.ndarray,
        safety_states: list[str],
        selected: int | None,
        source: Image,
    ) -> None:
        output = MarkerArray()
        clear = Marker()
        clear.header.stamp = source.header.stamp
        clear.header.frame_id = self.args.world_frame
        clear.ns = "route_yopo_control"
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        colors = {
            "CERTIFIED": (0.25, 0.75, 0.95, 0.55),
            "UNVALIDATED": (0.95, 0.70, 0.10, 0.60),
            "INVALID": (0.90, 0.15, 0.12, 0.60),
            "ALTITUDE": (0.72, 0.20, 0.75, 0.60),
            "NON_FINITE": (0.35, 0.35, 0.35, 0.50),
        }
        for index, trajectory in enumerate(trajectories):
            if not np.isfinite(trajectory).all():
                continue
            marker = Marker()
            marker.header.stamp = source.header.stamp
            marker.header.frame_id = self.args.world_frame
            marker.ns = "route_yopo_control/candidates"
            marker.id = index + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12 if index == selected else 0.045
            rgba = (0.10, 0.95, 0.35, 0.95) if index == selected else colors[safety_states[index]]
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
            marker.points = [_point(values) for values in trajectory]
            output.markers.append(marker)
        self.candidate_pub.publish(output)

    @staticmethod
    def _publish_json(publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
        publisher.publish(message)

    def _hold_status(self, reason: str, **extra: Any) -> dict[str, Any]:
        status = {
            "controller": "route_yopo",
            "control_output": getattr(self.args, "control_topic", "/scalenav/trajectory_point")
            if hasattr(self, "args")
            else "/scalenav/trajectory_point",
            "control_state": "HOLD",
            "mode": RouteMode.SAFETY_HOLD.value,
            "reason": reason,
            "route_source": "scalenav_compat_non_atomic",
            "route_id": self.route_ids.value if hasattr(self, "route_ids") else 0,
        }
        status.update(extra)
        return status

    def _publish_hold(self, reason: str, **extra: Any) -> None:
        status = self._hold_status(reason, **extra)
        with self.lock:
            self.control_trajectory = None
            self.control_executing = False
            self.last_status = status
        if hasattr(self, "status_pub"):
            self._publish_json(self.status_pub, status)

    def publish_control(self) -> None:
        now = time.monotonic()
        publisher_count = self.count_publishers(self.args.control_topic)
        if publisher_count > 1:
            with self.lock:
                self.control_trajectory = None
                self.control_executing = False
                first_conflict = not self.control_conflict
                self.control_conflict = True
                self.last_status = self._hold_status(
                    "control_publisher_conflict",
                    publisher_count=publisher_count,
                )
            if first_conflict:
                self.get_logger().error(
                    f"control disabled: {publisher_count} publishers on {self.args.control_topic}"
                )
            return

        with self.lock:
            recovered = self.control_conflict
            self.control_conflict = False
            odom_record = self.odom_record
            trajectory = self.control_trajectory
            path_record = getattr(self, "path_record", None)
            last_depth = self.last_depth_monotonic
            planned_yaw = self.planned_yaw
            control_armed = self.control_armed
            previous_velocity_world = getattr(self, "previous_velocity_world", None)
            measured_velocity_world = (
                np.zeros(3, dtype=np.float64)
                if previous_velocity_world is None
                else previous_velocity_world.astype(np.float64, copy=True)
            )
        if recovered:
            self.get_logger().warning(
                "control publisher conflict cleared; waiting for a newly certified trajectory"
            )
        if odom_record is None or now - odom_record[1] > self.args.odom_timeout:
            return
        if not control_armed:
            return

        odom = odom_record[0]
        measured_position = np.array(
            [
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        trajectory_fresh = (
            trajectory is not None
            and last_depth > 0.0
            and now - last_depth <= self.args.control_timeout
            and now - trajectory[3] <= self.segment_time_s
        )
        if trajectory_fresh:
            if len(trajectory) == 4:
                positions, velocities, accelerations, started = trajectory
                duration_s = self.segment_time_s
            else:
                positions, velocities, accelerations, started, duration_s = trajectory
            progress = np.clip(
                (now - started) / max(duration_s, 1.0e-6) * (len(positions) - 1),
                0.0,
                len(positions) - 1,
            )
            lower = int(math.floor(progress))
            upper = min(lower + 1, len(positions) - 1)
            weight = progress - lower
            desired_position = (1.0 - weight) * positions[lower] + weight * positions[upper]
            desired_velocity = (1.0 - weight) * velocities[lower] + weight * velocities[upper]
            desired_acceleration = (
                (1.0 - weight) * accelerations[lower] + weight * accelerations[upper]
            )
            # Keep the command anchored at the measured pose. This prevents a
            # far-ahead polynomial sample from creating a large stale position
            # error while preserving the planned velocity direction.
            desired_position = measured_position.copy()
            desired_position[2] = positions[lower, 2] * (1.0 - weight) + positions[upper, 2] * weight
            velocity_norm = float(np.linalg.norm(desired_velocity))
            maximum_speed = getattr(self.args, "maximum_speed", 3.0)
            command_acceleration_limit = getattr(
                self.args, "command_acceleration_limit", 2.5
            )
            if velocity_norm > maximum_speed:
                desired_velocity *= maximum_speed / velocity_norm
            acceleration_norm = float(np.linalg.norm(desired_acceleration))
            if acceleration_norm > command_acceleration_limit:
                desired_acceleration *= command_acceleration_limit / acceleration_norm
            # Horizontal tilt temporarily changes the vertical thrust. Close a
            # small explicit altitude loop so that this coupling cannot turn a
            # fixed-height route into a climb or descent.
            altitude_error = float(desired_position[2] - measured_position[2])
            desired_acceleration[2] += (
                getattr(self.args, "altitude_kp", 4.0) * altitude_error
                - getattr(self.args, "altitude_kv", 2.5) * measured_velocity_world[2]
            )
            acceleration_norm = float(np.linalg.norm(desired_acceleration))
            if acceleration_norm > command_acceleration_limit:
                desired_acceleration *= command_acceleration_limit / acceleration_norm
            speed_xy = float(np.linalg.norm(desired_velocity[:2]))
            target_yaw = (
                math.atan2(float(desired_velocity[1]), float(desired_velocity[0]))
                if speed_xy > 0.2
                else planned_yaw
            )
            maximum_change = self.args.max_yaw_rate * 0.02
            yaw_change = float(
                np.clip(_wrap_angle(target_yaw - planned_yaw), -maximum_change, maximum_change)
            )
            command_yaw = _wrap_angle(planned_yaw + yaw_change)
            yaw_rate = yaw_change / 0.02
            with self.lock:
                self.control_executing = True
        else:
            if measured_position[2] < self.args.minimum_altitude:
                return
            desired_position = measured_position
            desired_velocity = np.zeros(3, dtype=np.float64)
            desired_acceleration = np.zeros(3, dtype=np.float64)
            command_yaw = _quaternion_yaw(odom.pose.pose.orientation)
            yaw_rate = 0.0
            # A safety hold should stop translation, but it should not leave
            # the camera pointed permanently away from the route. Rotate at
            # the normal yaw-rate limit toward the next route segment so the
            # next depth frame can reacquire the corridor and resume motion.
            if (
                path_record is not None
                and now - path_record[1] <= getattr(self.args, "route_timeout", 1.0)
                and len(path_record[0]) >= 2
            ):
                route = np.asarray(path_record[0], dtype=np.float64)
                nearest = int(
                    np.argmin(np.linalg.norm(route[:, :2] - measured_position[None, :2], axis=1))
                )
                target = route[min(nearest + 1, len(route) - 1)] - measured_position
                if float(np.linalg.norm(target[:2])) > 0.5:
                    target_yaw = math.atan2(float(target[1]), float(target[0]))
                    maximum_change = self.args.max_yaw_rate * 0.02
                    yaw_change = float(
                        np.clip(
                            _wrap_angle(target_yaw - planned_yaw),
                            -maximum_change,
                            maximum_change,
                        )
                    )
                    command_yaw = _wrap_angle(planned_yaw + yaw_change)
                    yaw_rate = yaw_change / 0.02
            with self.lock:
                if self.control_executing:
                    status = dict(self.last_status)
                    status["mode"] = RouteMode.SAFETY_HOLD.value
                    status["control_state"] = "HOLD"
                    status["reason"] = "trajectory_expired_or_sensor_stale"
                    self.last_status = status
                self.control_executing = False

        if not all(
            np.isfinite(values).all()
            for values in (desired_position, desired_velocity, desired_acceleration)
        ):
            with self.lock:
                self.control_trajectory = None
            return

        command = MultiDOFJointTrajectoryPoint()
        transform = Transform()
        transform.translation.x = float(desired_position[0])
        transform.translation.y = float(desired_position[1])
        transform.translation.z = float(desired_position[2])
        transform.rotation.z = math.sin(0.5 * command_yaw)
        transform.rotation.w = math.cos(0.5 * command_yaw)
        velocity = Twist()
        acceleration = Twist()
        for axis, field in enumerate(("x", "y", "z")):
            setattr(velocity.linear, field, float(desired_velocity[axis]))
            setattr(acceleration.linear, field, float(desired_acceleration[axis]))
        velocity.angular.z = float(yaw_rate)
        command.transforms.append(transform)
        command.velocities.append(velocity)
        command.accelerations.append(acceleration)
        self.command_pub.publish(command)
        with self.lock:
            self.planned_yaw = command_yaw

    def publish_latest_status(self) -> None:
        with self.lock:
            status = dict(self.last_status)
        self._publish_json(self.status_pub, status)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(
        description="Route-conditioned YOPO online controller"
    )
    parser.add_argument(
        "--model",
        default=str(root / "scalenav_ws/src/models/original_yopo_simple/model.pt"),
    )
    parser.add_argument("--train-root", default=str(root / "train_scalenav"))
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--odom-topic", default="/sim/odom")
    parser.add_argument("--depth-topic", default="/camera/depth/image")
    parser.add_argument("--path-topic", default="/scalenav/path")
    parser.add_argument("--graph-topic", default="/scalenav/graph")
    parser.add_argument("--bubble-topic", default="/scalenav/bubbles")
    parser.add_argument("--clearance-topic", default="/scalenav/clearance")
    parser.add_argument("--world-frame", default="world_enu")
    parser.add_argument("--control-topic", default="/scalenav/trajectory_point")
    parser.add_argument(
        "--mpc-path-topic",
        default="/scalenav/route_yopo/mpc_path",
        help="RViz nav_msgs/Path topic for the current ordered-bubble MPC solution",
    )
    parser.add_argument(
        "--mpc-bubble-topic",
        default="/scalenav/route_yopo/mpc_bubbles",
        help="RViz visualization_msgs/MarkerArray topic for MPC ordered bubbles",
    )
    parser.add_argument("--control-timeout", type=float, default=0.5)
    parser.add_argument("--max-yaw-rate", type=float, default=1.5)
    parser.add_argument(
        "--maximum-speed",
        type=float,
        default=6.0,
        help="controller and MPC speed limit in m/s (default: 6.0)",
    )
    parser.add_argument("--odom-twist-frame", choices=("body", "world"), default="body")
    parser.add_argument(
        "--ordered-bubble-mpc",
        action="store_true",
        help="run leap-c/acados MPC with ordered route bubbles after YOPO",
    )
    parser.add_argument("--update-rate", type=float, default=1.0)
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--route-timeout", type=float, default=1.0)
    parser.add_argument("--compat-stamp-slop", type=float, default=0.20)
    parser.add_argument("--route-start-tolerance", type=float, default=1.5)
    # Route updates can lag the 50 Hz controller by several metres while the
    # vehicle is moving. Allow trimming that lagging prefix without accepting
    # an unrelated route from far away.
    parser.add_argument("--route-reanchor-tolerance", type=float, default=10.0)
    parser.add_argument("--route-terminal-tolerance", type=float, default=2.0)
    parser.add_argument("--route-planarity-tolerance", type=float, default=0.05)
    parser.add_argument("--route-altitude-tolerance", type=float, default=0.25)
    parser.add_argument("--route-corridor-tracking-tolerance", type=float, default=0.1)
    parser.add_argument(
        "--route-corridor-min-known-fraction",
        type=float,
        default=0.75,
        help="minimum depth coverage required before a route corridor can supplement unknown FOV samples",
    )
    parser.add_argument("--minimum-route-length", type=float, default=0.5)
    parser.add_argument("--robot-radius", type=float, default=0.3)
    parser.add_argument("--safety-margin", type=float, default=0.2)
    parser.add_argument("--minimum-altitude", type=float, default=0.25)
    parser.add_argument("--acceleration-limit", type=float, default=6.0)
    parser.add_argument("--command-acceleration-limit", type=float, default=2.5)
    parser.add_argument("--altitude-kp", type=float, default=4.0)
    parser.add_argument("--altitude-kv", type=float, default=2.5)
    parser.add_argument("--minimum-trajectory-hold", type=float, default=1.0)
    parser.add_argument("--safety-samples", type=int, default=101)
    parser.add_argument("--safety-sample-step", type=float, default=0.20)
    parser.add_argument("--max-unknown-fraction", type=float, default=0.20)
    parser.add_argument("--minimum-depth", type=float, default=0.04)
    parser.add_argument("--max-depth", type=float, default=20.0)
    parser.add_argument("--source-horizontal-fov", type=float, default=90.0)
    parser.add_argument("--source-vertical-fov", type=float, default=73.7398)
    parser.add_argument(
        "--camera-translation-flu",
        type=float,
        nargs=3,
        default=(0.5, 0.0, -0.1),
    )
    args = parser.parse_args()
    if (
        args.update_rate <= 0.0
        or args.safety_samples < 101
        or args.acceleration_limit <= 0.0
        or args.maximum_speed <= 0.0
        or args.command_acceleration_limit <= 0.0
        or args.altitude_kp <= 0.0
        or args.altitude_kv <= 0.0
        or args.minimum_trajectory_hold < 0.0
        or args.route_planarity_tolerance <= 0.0
        or args.route_altitude_tolerance <= 0.0
        or args.route_corridor_tracking_tolerance < 0.0
        or args.route_corridor_tracking_tolerance > args.safety_margin
        or not 0.0 < args.route_corridor_min_known_fraction <= 1.0
        or not all(math.isfinite(value) for value in args.camera_translation_flu)
    ):
        parser.error("update rate must be positive and safety samples must be at least 101")
    return args


def main() -> None:
    args = parse_args()
    model = Path(args.model).expanduser()
    if not model.is_file():
        raise FileNotFoundError(f"Route-YOPO checkpoint not found: {model}")
    rclpy.init()
    node = RouteYopoController(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Process-group shutdown can invalidate the rclpy context while the
        # executor is constructing its next wait set. Suppress only that
        # shutdown race; real runtime exceptions still propagate.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
