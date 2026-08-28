#!/usr/bin/env python3
"""Route-conditioned YOPO online controller with a mandatory safety gate."""

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
    conservative_depth_reduce,
    decide_route_mode,
    project_endstates_to_altitude,
    quaternion_xyzw_to_matrix,
    route_signature,
    sample_poly5_candidate_states,
    select_first_certified,
    validate_depth_trajectory,
    world_to_body_flu,
)


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
        self.clearance_record: tuple[float, float, float] | None = None
        self.previous_velocity_world: np.ndarray | None = None
        self.previous_velocity_time: float | None = None
        self.acceleration_world = np.zeros(3, dtype=np.float32)
        self.last_depth_monotonic = 0.0
        self.control_trajectory: tuple[np.ndarray, np.ndarray, np.ndarray, float] | None = None
        self.planned_yaw = 0.0
        self.control_conflict = False
        self.control_executing = False
        self.control_armed = False
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
        from policy.yopo_network import YopoNetwork

        self.cfg = cfg
        self.sample_route_bubbles = sample_route_bubbles
        self.anchors = np.asarray(cfg["route_anchor_distances_m"], dtype=np.float32)
        self.route_radius_clip_m = float(cfg["route_clearance_clip_m"])
        self.segment_time_s = float(cfg["sgm_time"])
        self.image_width = int(cfg["image_width"])
        self.image_height = int(cfg["image_height"])
        self.device = self._resolve_device(args.device)

        checkpoint_path = Path(args.model).expanduser().resolve()
        checkpoint = torch.load(
            checkpoint_path, map_location=self.device, weights_only=False
        )
        checkpoint_anchors = checkpoint.get("route_anchor_distances_m")
        if checkpoint_anchors is not None and not np.array_equal(
            np.asarray(checkpoint_anchors, dtype=np.float32), self.anchors
        ):
            raise ValueError("checkpoint route anchors do not match online configuration")
        if int(checkpoint.get("route_bubble_count", len(self.anchors))) != len(self.anchors):
            raise ValueError("checkpoint route bubble count does not match online configuration")
        self.model = YopoNetwork().to(self.device).eval()
        self.feature_order = self.model.load_route_checkpoint(checkpoint)
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
        count = len(self.anchors)
        depth = torch.ones(
            (1, 1, self.image_height, self.image_width), device=self.device
        )
        motion = torch.zeros((1, 6), device=self.device)
        frontier = torch.tensor([[10.0, 0.0, 0.0]], device=self.device)
        route = torch.zeros((1, count, 4), device=self.device)
        mask = torch.zeros((1, count), device=self.device)
        endstate, score = self.model(depth, motion, frontier, route, mask)
        if tuple(endstate.shape) != (1, 9, 3, 5) or tuple(score.shape) != (1, 3, 5):
            raise ValueError(
                f"Route-YOPO output contract mismatch: {tuple(endstate.shape)}, {tuple(score.shape)}"
            )
        if not torch.isfinite(endstate).all() or not torch.isfinite(score).all():
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
            self.last_depth_monotonic = now
            odom_record = self.odom_record
            path_record = self.path_record
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
            source_stamps = [frontier_record[2], path_record[2], clearance_record[2]]
            positive_stamps = [stamp for stamp in source_stamps if stamp > 0.0]
            route_coherent = len(positive_stamps) == 3 and (
                max(positive_stamps) - min(positive_stamps) <= self.args.compat_stamp_slop
            )

        route_valid = False
        route_reason = "route_not_available"
        route_path = None
        safe_radius_m = math.nan
        route_altitude_m = math.nan
        if frontier_fresh and route_fresh:
            route_path = path_record[0].astype(np.float64, copy=True)
            frontier_world = frontier_record[0].astype(np.float64, copy=True)
            path_clearance_m = float(clearance_record[0])
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
        route_mask = np.zeros(count, dtype=np.float32)
        route_features = np.zeros((count, 4), dtype=np.float32)
        sample_distances = self.anchors.copy()
        if decision.mode == RouteMode.ROUTE:
            point_radii = np.full(len(route_path), safe_radius_m, dtype=np.float32)
            centers_world, sampled_radii, route_mask, sample_distances = (
                self.sample_route_bubbles(route_path, point_radii, self.anchors)
            )
            route_features, route_mask = build_route_features(
                centers_world,
                sampled_radii,
                route_mask,
                sample_distances,
                position_world,
                rotation,
                radius_clip_m=self.route_radius_clip_m,
            )
            route_id = self.route_ids.observe(route_signature(frontier_world, route_path))

        frontier_body = world_to_body_flu(frontier_world, position_world, rotation).astype(
            np.float32
        )
        motion_body = np.concatenate(
            (rotation.T @ velocity_world, rotation.T @ acceleration_world)
        ).astype(np.float32)
        model_depth = self._model_depth(raw_depth)
        depth_tensor = torch.from_numpy(model_depth[None, None]).to(
            self.device, non_blocking=True
        )
        motion_tensor = torch.from_numpy(motion_body[None]).to(self.device, non_blocking=True)
        frontier_tensor = torch.from_numpy(frontier_body[None]).to(
            self.device, non_blocking=True
        )
        route_tensor = torch.from_numpy(route_features[None]).to(
            self.device, non_blocking=True
        )
        mask_tensor = torch.from_numpy(route_mask[None]).to(self.device, non_blocking=True)

        started = time.perf_counter()
        endstate, score = self.model(
            depth_tensor, motion_tensor, frontier_tensor, route_tensor, mask_tensor
        )
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        inference_ms = (time.perf_counter() - started) * 1000.0
        self.inference_times_ms.append(inference_ms)
        if not torch.isfinite(endstate).all() or not torch.isfinite(score).all():
            self._publish_hold("model_output_non_finite")
            return

        endstates_body = (
            endstate[0].permute(1, 2, 0).reshape(-1, 9).detach().cpu().numpy()
        )
        scores = score[0].reshape(-1).detach().cpu().numpy()
        raw_endpoint_world = position_world[None] + endstates_body[:, :3] @ rotation.T
        if decision.mode == RouteMode.ROUTE:
            endstates_body = project_endstates_to_altitude(
                endstates_body,
                position_world,
                rotation,
                route_altitude_m,
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
        safety_depth = self._safety_depth(raw_depth)
        query = DepthSafeVolumeQuery(
            safety_depth,
            horizontal_fov_deg=self.args.source_horizontal_fov,
            vertical_fov_deg=self.args.source_vertical_fov,
            robot_radius_m=self.args.robot_radius,
            safety_margin_m=self.args.safety_margin,
            sample_step_m=self.args.safety_sample_step,
            far_depth_m=self.args.max_depth,
            max_unknown_fraction=self.args.max_unknown_fraction,
        )
        safety = [
            self._validate_trajectory(
                query,
                trajectory,
                position_world,
                rotation,
                route_altitude_m=route_altitude_m
                if decision.mode == RouteMode.ROUTE
                else None,
            )
            for trajectory in trajectories
        ]
        safety_states = [item["state"] for item in safety]
        selected = select_first_certified(
            scores,
            trajectories,
            safety_states,
            minimum_altitude_m=self.args.minimum_altitude,
        )
        score_selected = int(np.argmin(scores))
        output_mode = decision.mode if selected is not None else RouteMode.SAFETY_HOLD
        output_reason = decision.reason if selected is not None else "no_certified_primitive"

        route_contract = {
            "contract": "RouteCondition-compatible diagnostic v1",
            "source": "scalenav_compat_non_atomic",
            "atomic": False,
            "route_id": route_id,
            "mode": decision.mode.value,
            "frontier_world": frontier_world.tolist(),
            "centers_world": centers_world.tolist(),
            "safe_radii_m": sampled_radii.tolist(),
            "mask": route_mask.tolist(),
            "anchor_or_feature_distances_m": sample_distances.tolist(),
            "radius_source": "scalenav_path_min_clearance_broadcast",
            "path_safe_radius_m": safe_radius_m if math.isfinite(safe_radius_m) else None,
            "fixed_altitude_m": route_altitude_m
            if math.isfinite(route_altitude_m)
            else None,
            "route_validation": route_reason,
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
            "selected_primitive": selected,
            "score_selected_primitive": score_selected,
            "score_selected_certified": safety_states[score_selected] == "CERTIFIED",
            "selected_score": None if selected is None else float(scores[selected]),
            "candidate_scores": scores.tolist(),
            "raw_candidate_endpoint_z_min_max_m": [
                float(np.min(raw_endpoint_world[:, 2])),
                float(np.max(raw_endpoint_world[:, 2])),
            ],
            "fixed_altitude_m": route_altitude_m
            if math.isfinite(route_altitude_m)
            else None,
            "candidate_safety": safety,
            "inference_ms": inference_ms,
            "inference_p95_ms": float(np.percentile(self.inference_times_ms, 95.0)),
            "safety_samples_per_primitive": self.args.safety_samples,
            "safety_depth_shape_hw": list(safety_depth.shape),
        }
        self._publish_json(self.route_pub, route_contract)
        self._publish_json(self.status_pub, status)
        with self.lock:
            if selected is None:
                self.control_trajectory = None
                self.control_executing = False
            self.last_status = status
        self._publish_candidates(trajectories, safety_states, selected, depth_message)
        if selected is not None:
            with self.lock:
                self.control_trajectory = (
                    trajectories[selected].copy(),
                    trajectory_velocities[selected].copy(),
                    trajectory_accelerations[selected].copy(),
                    time.monotonic(),
                )
                self.control_executing = True
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
        altitude_span = float(np.ptp(path[:, 2]))
        if altitude_span > self.args.route_planarity_tolerance:
            return False, "path_not_fixed_altitude", path, math.nan, math.nan
        start_error = float(np.linalg.norm(path[0] - position))
        if start_error > self.args.route_start_tolerance:
            return False, "path_start_discontinuous", path, math.nan, math.nan
        if start_error > 1.0e-3:
            path = np.concatenate((position[None], path), axis=0)
        if float(np.linalg.norm(path[-1] - frontier)) > self.args.route_terminal_tolerance:
            return False, "path_terminal_mismatch", path, math.nan, math.nan
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        if length < self.args.minimum_route_length:
            return False, "path_too_short", path, math.nan, math.nan
        safe_radius = path_clearance_m - self.args.robot_radius - self.args.safety_margin
        if safe_radius <= 0.0:
            return False, "path_safe_space_insufficient", path, safe_radius, math.nan
        return True, "valid_fixed_altitude_route", path, safe_radius, route_altitude

    def _validate_trajectory(
        self,
        query: DepthSafeVolumeQuery,
        trajectory_world: np.ndarray,
        position_world: np.ndarray,
        rotation_body_to_world: np.ndarray,
        *,
        route_altitude_m: float | None = None,
    ) -> dict[str, Any]:
        return validate_depth_trajectory(
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
            route_altitude_tolerance_m=self.args.route_altitude_tolerance
            if route_altitude_m is not None
            else None,
        )

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
            "ROUTE_ALTITUDE": (0.85, 0.35, 0.75, 0.65),
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
            last_depth = self.last_depth_monotonic
            planned_yaw = self.planned_yaw
            control_armed = self.control_armed
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
            positions, velocities, accelerations, started = trajectory
            progress = np.clip(
                (now - started) / self.segment_time_s * (len(positions) - 1),
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
        default=str(root / "train_scalenav/saved_corrected/YOPO_5/best.pth"),
    )
    parser.add_argument("--train-root", default=str(root / "train_scalenav"))
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--odom-topic", default="/sim/odom")
    parser.add_argument("--depth-topic", default="/camera/depth/image")
    parser.add_argument("--path-topic", default="/scalenav/path")
    parser.add_argument("--graph-topic", default="/scalenav/graph")
    parser.add_argument("--clearance-topic", default="/scalenav/clearance")
    parser.add_argument("--world-frame", default="world_enu")
    parser.add_argument("--control-topic", default="/scalenav/trajectory_point")
    parser.add_argument("--control-timeout", type=float, default=0.5)
    parser.add_argument("--max-yaw-rate", type=float, default=1.5)
    parser.add_argument("--odom-twist-frame", choices=("body", "world"), default="body")
    parser.add_argument("--update-rate", type=float, default=5.0)
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--route-timeout", type=float, default=1.0)
    parser.add_argument("--compat-stamp-slop", type=float, default=0.20)
    parser.add_argument("--route-start-tolerance", type=float, default=1.5)
    parser.add_argument("--route-terminal-tolerance", type=float, default=2.0)
    parser.add_argument("--route-planarity-tolerance", type=float, default=0.05)
    parser.add_argument("--route-altitude-tolerance", type=float, default=0.25)
    parser.add_argument("--minimum-route-length", type=float, default=0.5)
    parser.add_argument("--robot-radius", type=float, default=0.3)
    parser.add_argument("--safety-margin", type=float, default=0.2)
    parser.add_argument("--minimum-altitude", type=float, default=0.25)
    parser.add_argument("--acceleration-limit", type=float, default=6.0)
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
        or args.route_planarity_tolerance <= 0.0
        or args.route_altitude_tolerance <= 0.0
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
