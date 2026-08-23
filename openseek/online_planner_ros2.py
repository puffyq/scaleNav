#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Point, PoseStamped, Transform, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, PointCloud2, PointField
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from config.config import cfg
from policy.poly_solver import Poly5Solver, calculate_yaw
from planner_continuity import (
    accept_local_goal,
    is_final_subgoal,
    mission_goal_for_local_goal,
    mission_arrived,
    project_goal_to_fixed_altitude,
)
from text_tracker.heatmap import goal_body_to_heatmap
from graph import (
    DepthSafeVolumeQuery,
    GraphConfig,
    SparseDepthGraph,
    STATE_RGBA,
    build_graph_visualization,
)


def quaternion_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_rotation(q) -> np.ndarray:
    """Return the body-to-world rotation for a ROS xyzw quaternion."""
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    message = Quaternion()
    message.z = math.sin(0.5 * yaw)
    message.w = math.cos(0.5 * yaw)
    return message


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class OnlinePlanner(Node):
    @staticmethod
    def format_vec(values: np.ndarray | list[float] | tuple[float, ...], precision: int = 2) -> str:
        return "(" + ",".join(f"{float(value):.{precision}f}" for value in values) + ")"

    @staticmethod
    def vec(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
        return [float(value) for value in values]

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("openseek_online_planner")
        self.args = args
        self.device = torch.device(args.device)
        self.model = torch.jit.load(args.model, map_location=self.device).eval()
        self.image_width = args.model_image_width or int(cfg["image_width"])
        self.image_height = args.model_image_height or int(cfg["image_height"])
        self.model_vertical_num = args.model_vertical_num or int(cfg["vertical_num"])
        self.fixed_altitude = bool(args.fixed_altitude)
        self.direct_goal_distance = float(args.direct_goal_distance)
        self.max_depth = 20.0
        self.minimum_depth = 0.04
        self.segment_time = 2.0 * float(cfg["radio_range"]) / float(cfg["vel_max_train"])
        self.lock = threading.Lock()
        self.flight_lock = threading.RLock()
        self.callback_group = ReentrantCallbackGroup()
        self.inference_lock = threading.Lock()
        self.odom: Odometry | None = None
        self.last_depth_time = 0.0
        self.previous_velocity_world: np.ndarray | None = None
        self.velocity_world = np.zeros(3, dtype=np.float32)
        self.previous_velocity_stamp: float | None = None
        self.acceleration_world = np.zeros(3, dtype=np.float32)
        self.flight_samples = deque(maxlen=50000)
        self.flight_path_length_m = 0.0
        self.flight_duration_s = 0.0
        self.flight_speed_integral = 0.0
        self.flight_max_speed_mps = 0.0
        self.flight_max_acceleration_mps2 = 0.0
        self.flight_max_jerk_mps3 = 0.0
        self.flight_jerk_squared_integral = 0.0
        self.flight_last_acceleration = np.zeros(3, dtype=np.float64)
        self.flight_have_acceleration = False
        self.flight_last_report = 0.0
        self.flight_visualization_max_points = 2000
        # Match YOPO-Simple's desire_pos / desire_vel / desire_acc state.
        self.desired_position_world: np.ndarray | None = None
        self.desired_velocity_world: np.ndarray | None = None
        self.desired_acceleration_world = np.zeros(3, dtype=np.float64)
        self.polynomials: tuple[Poly5Solver, Poly5Solver, Poly5Solver] | None = None
        self.trajectory_started = 0.0
        self.planned_yaw = 0.0
        self.inference_count = 0
        self.inference_total_ms = 0.0
        self.last_log_time = 0.0
        self.trajectory_valid_for_control = False
        self.last_tracking_error = 0.0
        self.control_topic = args.control_topic
        self.goal_world: np.ndarray | None = None
        self.mission_goal_world: np.ndarray | None = None
        self.mission_complete = False
        self.graph = (
            SparseDepthGraph(
                GraphConfig(candidate_distance_m=args.graph_candidate_distance)
            )
            if args.graph_visualization
            else None
        )
        project_root = Path(__file__).resolve().parents[1]
        self.event_log_dir = Path(
            args.event_log_dir or os.environ.get("OPENSEEK_EVENT_LOG_DIR", project_root / "log_event")
        ).expanduser().resolve()
        self.event_log_dir.mkdir(parents=True, exist_ok=True)
        run_id = time.strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}"
        self.event_log_path = self.event_log_dir / f"openseek_events_{run_id}.jsonl"
        self.depth_log_dir = self.event_log_dir / f"openseek_depth_{run_id}"
        self.depth_log_dir.mkdir(parents=True, exist_ok=True)
        self.save_depth_png_enabled = bool(args.save_depth_png)
        self.frame_index = 0
        self.event_log = logging.getLogger(f"openseek.events.{os.getpid()}")
        self.event_log.setLevel(logging.INFO)
        self.event_log.propagate = False
        if not self.event_log.handlers:
            handler = logging.FileHandler(self.event_log_path, mode="w", encoding="utf-8")
            handler.setFormatter(logging.Formatter("%(message)s"))
            self.event_log.addHandler(handler)

        self.command_pub = None
        self.colosseum_command_pub = None
        self.colosseum_command_type = None
        if self.control_topic:
            from colosseum_interfaces.msg import VelCmd

            self.colosseum_command_type = VelCmd
            self.colosseum_command_pub = self.create_publisher(VelCmd, self.control_topic, 10)
        else:
            self.command_pub = self.create_publisher(
                MultiDOFJointTrajectoryPoint, "/openseek/trajectory_point", 10
            )
        self.path_pub = self.create_publisher(PathMsg, "/openseek/planned_path", 10)
        self.visual_odom_pub = self.create_publisher(
            Odometry, "/openseek/odom", 20
        )
        self.flight_pub = self.create_publisher(MarkerArray, "/openseek/flight", 1)
        goal_qos = QoSProfile(depth=1)
        goal_qos.reliability = ReliabilityPolicy.RELIABLE
        goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_visual_pub = self.create_publisher(
            PoseStamped, "/openseek/goal", goal_qos
        )
        self.graph_marker_pub = (
            self.create_publisher(MarkerArray, args.graph_marker_topic, 10)
            if self.graph is not None
            else None
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/sim/odom", self.on_odometry, 20,
            callback_group=self.callback_group,
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image", self.on_depth, qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2, args.lidar_topic, self.on_lidar, 10,
            callback_group=self.callback_group,
        )
        if args.original_goal_input:
            self.goal_sub = self.create_subscription(
                PoseStamped, args.goal_topic, self.on_goal, 10,
                callback_group=self.callback_group,
            )
        self.mission_goal_sub = None
        if args.mission_goal_topic:
            self.mission_goal_sub = self.create_subscription(
                PoseStamped, args.mission_goal_topic, self.on_mission_goal, 10,
                callback_group=self.callback_group,
            )
        self.control_timer = self.create_timer(
            0.02, self.publish_control, callback_group=self.callback_group)
        self.status_timer = self.create_timer(
            2.0, self.report_status, callback_group=self.callback_group)
        self.flight_timer = self.create_timer(
            0.5, self.publish_flight_telemetry, callback_group=self.callback_group)

        self.warm_up()
        self.emit_event(
            "startup",
            {
                "model": args.model,
                "device": str(self.device),
                "control": bool(args.control),
                "event_log": str(self.event_log_path),
                "depth_log_dir": str(self.depth_log_dir),
                "depth_png_enabled": self.save_depth_png_enabled,
                "depth_png_encoding": "uint16 millimeters; divide by 1000 for meters",
                "source_depth_contract": "32FC1 meters or 16UC1 millimeters",
                "network_depth_contract": "clip(depth_m, 20) / 20, uint8 inpaint, then / 255",
                "network_depth_range": [0.0, 1.0],
                "max_depth_m": self.max_depth,
                "minimum_depth_m": self.minimum_depth,
                "model_image_shape_hw": [self.image_height, self.image_width],
                "model_horizontal_fov_deg": args.model_horizontal_fov,
                "model_vertical_fov_deg": args.model_vertical_fov,
                "segment_time_s": self.segment_time,
                "odom_twist_frame": args.odom_twist_frame,
                "plan_from_reference": bool(args.plan_from_reference),
                "reference_reset_position_error_m": args.reference_reset_position_error,
                "reference_reset_velocity_error_m_s": args.reference_reset_velocity_error,
                "minimum_trajectory_altitude_m": args.minimum_trajectory_altitude,
                "trajectory_altitude_margin_m": args.altitude_margin,
                "model_motion_contract": "body FLU [velocity_m_s, acceleration_m_s2, goal_m]",
                "model_output_contract": "body FLU [position_m, velocity_m_s, acceleration_m_s2]",
                "lidar_topic": args.lidar_topic,
                "graph_visualization": bool(args.graph_visualization),
                "graph_marker_topic": args.graph_marker_topic,
                "graph_candidate_distance_m": args.graph_candidate_distance,
                "graph_robot_radius_m": args.graph_robot_radius,
            },
        )

    def emit_event(self, event: str, payload: dict) -> None:
        record = {
            "event": event,
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
        }
        record.update(payload)
        self.event_log.info(json.dumps(record, separators=(",", ":"), ensure_ascii=True))

    def write_depth_png(self, path: Path, depth_meters: np.ndarray) -> None:
        depth_mm = np.clip(np.rint(depth_meters * 1000.0), 0.0, 65535.0).astype(np.uint16)
        cv2.imwrite(str(path), depth_mm)

    def on_lidar(self, message: PointCloud2) -> None:
        fields = {field.name: field for field in message.fields}
        xyz_fields = [fields.get(axis) for axis in ("x", "y", "z")]
        if any(field is None for field in xyz_fields):
            self.emit_event(
                "lidar_error",
                {"reason": "PointCloud2 has no x/y/z fields", "frame_id": message.header.frame_id},
            )
            return
        if any(field.datatype != PointField.FLOAT32 or field.count != 1 for field in xyz_fields):
            self.emit_event(
                "lidar_error",
                {"reason": "x/y/z fields are not scalar float32", "frame_id": message.header.frame_id},
            )
            return

        point_count = int(message.width * message.height)
        if point_count == 0 or message.point_step <= 0:
            points = np.empty((0, 3), dtype=np.float32)
        else:
            byte_order = ">" if message.is_bigendian else "<"
            cloud = np.frombuffer(message.data, dtype=np.uint8)
            required = point_count * int(message.point_step)
            if cloud.size < required:
                self.emit_event(
                    "lidar_error",
                    {
                        "reason": "PointCloud2 payload is shorter than width*height*point_step",
                        "frame_id": message.header.frame_id,
                    },
                )
                return
            records = cloud[:required].reshape(point_count, int(message.point_step))
            coordinates = []
            for field in xyz_fields:
                coordinate = records[:, field.offset : field.offset + 4].copy().view(
                    np.dtype(f"{byte_order}f4")
                ).reshape(-1)
                coordinates.append(coordinate)
            points = np.stack(coordinates, axis=1).astype(np.float32, copy=False)
            points = points[np.isfinite(points).all(axis=1)]

        # settings.json requests SensorLocalFrame, whose raw Colosseum axes
        # are FRD. The bridge's ENU lidar transform currently fails before it
        # touches these points, so convert the local sensor basis to ROS FLU.
        if points.size:
            points[:, 1] *= -1.0
            points[:, 2] *= -1.0

        ranges = np.linalg.norm(points, axis=1) if points.size else np.empty(0, dtype=np.float32)
        stamp = message.header.stamp
        self.emit_event(
            "lidar",
            {
                "stamp": stamp.sec + stamp.nanosec * 1e-9,
                "frame_id": message.header.frame_id,
                "point_count": int(points.shape[0]),
                "range_m_min_p05_median_p95_max": None
                if not ranges.size
                else self.vec(np.percentile(ranges, [0.0, 5.0, 50.0, 95.0, 100.0])),
                "points_xyz_m": points.tolist(),
            },
        )

    def warm_up(self) -> None:
        image_channels = 1 if self.args.original_goal_input else 2
        image = torch.zeros(
            1, image_channels, self.image_height, self.image_width, device=self.device
        )
        image[:, 0] = 1.0
        observation_size = 9 if self.args.original_goal_input else 6
        motion = torch.zeros(1, observation_size, device=self.device)
        if self.args.original_goal_input:
            motion[:, 6] = self.args.search_distance
        with torch.inference_mode():
            endstate, score = self.model(image, motion)
        if not torch.isfinite(endstate).all() or not torch.isfinite(score).all():
            raise FloatingPointError("model warm-up produced non-finite output")
        if self.args.original_goal_input:
            expected_image = (1, 1, self.image_height, self.image_width)
            expected_state = (1, 9)
            expected_output = (1, 9, self.model_vertical_num, 5)
            if tuple(image.shape) != expected_image or tuple(motion.shape) != expected_state:
                raise ValueError(
                    f"YOPO-Simple input contract mismatch: image={tuple(image.shape)} "
                    f"state={tuple(motion.shape)}, expected image={expected_image} "
                    f"state={expected_state}"
                )
            if tuple(endstate.shape) != expected_output or tuple(score.shape) != (
                1, self.model_vertical_num, 5
            ):
                raise ValueError(
                    f"YOPO-Simple output contract mismatch: endstate={tuple(endstate.shape)} "
                    f"score={tuple(score.shape)}, expected endstate={expected_output}"
                )

    def on_odometry(self, message: Odometry) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        message_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        sample_stamp = message_stamp if message_stamp > 0.0 else now
        # The Colosseum bridge publishes ENU numeric values but sets the pose
        # frame to the vehicle's spawn frame. Relabel an unchanged copy for
        # visualization so RViz does not apply the spawn transform a second
        # time. Planning continues to use the original numeric values below.
        visual_odom = Odometry()
        visual_odom.header.stamp = message.header.stamp
        visual_odom.header.frame_id = self.args.world_frame
        visual_odom.child_frame_id = message.child_frame_id
        visual_odom.pose = message.pose
        visual_odom.twist = message.twist
        self.visual_odom_pub.publish(visual_odom)

        velocity_input = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ],
            dtype=np.float32,
        )
        if self.args.odom_twist_frame == "body":
            velocity_world = (
                quaternion_rotation(message.pose.pose.orientation) @ velocity_input
            ).astype(np.float32)
        else:
            velocity_world = velocity_input
        with self.flight_lock:
            self.velocity_world = velocity_world
            if self.previous_velocity_world is not None and self.previous_velocity_stamp is not None:
                dt = sample_stamp - self.previous_velocity_stamp
                if 0.002 <= dt <= 0.2:
                    measured = (velocity_world - self.previous_velocity_world) / dt
                    measured = np.clip(measured, -6.0, 6.0)
                    self.acceleration_world = 0.85 * self.acceleration_world + 0.15 * measured
            self.previous_velocity_world = velocity_world
            self.previous_velocity_stamp = sample_stamp
            self.update_flight_statistics(
                np.array([
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    message.pose.pose.position.z,
                ], dtype=np.float64), velocity_world.astype(np.float64), sample_stamp)
        with self.lock:
            self.odom = message
            if self.polynomials is None or not self.trajectory_valid_for_control:
                self.desired_position_world = np.array(
                    [
                        message.pose.pose.position.x,
                        message.pose.pose.position.y,
                        message.pose.pose.position.z,
                    ],
                    dtype=np.float64,
                )
                self.desired_velocity_world = velocity_world.astype(
                    np.float64, copy=True
                )
                self.desired_acceleration_world = np.zeros(3, dtype=np.float64)
                self.planned_yaw = quaternion_yaw(message.pose.pose.orientation)
        yaw_deg = math.degrees(quaternion_yaw(message.pose.pose.orientation))
        stamp = message.header.stamp
        self.emit_event(
            "odom",
            {
                "stamp": stamp.sec + stamp.nanosec * 1e-9,
                "frame_id": message.header.frame_id,
                "child_frame_id": message.child_frame_id,
                "position_world": self.vec(
                    [
                        message.pose.pose.position.x,
                        message.pose.pose.position.y,
                        message.pose.pose.position.z,
                    ]
                ),
                "velocity_world": self.vec(velocity_world),
                "acceleration_world": self.vec(self.acceleration_world),
                "orientation_xyzw": self.vec(
                    [
                        message.pose.pose.orientation.x,
                        message.pose.pose.orientation.y,
                        message.pose.pose.orientation.z,
                        message.pose.pose.orientation.w,
                    ]
                ),
                "yaw_deg": yaw_deg,
            },
        )

    def update_flight_statistics(
        self, position: np.ndarray, velocity: np.ndarray, sample_time: float
    ) -> None:
        if not np.isfinite(position).all() or not np.isfinite(velocity).all():
            return
        with self.flight_lock:
            if self.flight_samples:
                previous_time, previous_position, previous_velocity = self.flight_samples[-1]
                dt = sample_time - previous_time
                if 1e-3 < dt < 2.0:
                    self.flight_path_length_m += float(np.linalg.norm(position - previous_position))
                    self.flight_duration_s += dt
                    self.flight_speed_integral += float(np.linalg.norm(velocity)) * dt
                    raw_acceleration = (velocity - previous_velocity) / dt
                    acceleration = 0.2 * raw_acceleration + 0.8 * self.flight_last_acceleration
                    if self.flight_have_acceleration:
                        jerk = (acceleration - self.flight_last_acceleration) / dt
                        jerk_norm = float(np.linalg.norm(jerk))
                        self.flight_max_jerk_mps3 = max(self.flight_max_jerk_mps3, jerk_norm)
                        self.flight_jerk_squared_integral += jerk_norm * jerk_norm * dt
                    self.flight_last_acceleration = acceleration
                    self.flight_have_acceleration = True
                    self.flight_max_acceleration_mps2 = max(
                        self.flight_max_acceleration_mps2, float(np.linalg.norm(acceleration)))
                    self.flight_max_speed_mps = max(
                        self.flight_max_speed_mps, float(np.linalg.norm(velocity)))
            self.flight_samples.append((sample_time, position.copy(), velocity.copy()))

    @staticmethod
    def speed_color(speed: float, maximum: float) -> tuple[float, float, float]:
        t = min(1.0, max(0.0, speed / max(0.1, maximum)))
        anchors = ((0.05, 0.20, 0.95), (0.05, 0.85, 0.35),
                   (1.00, 0.85, 0.05), (0.95, 0.05, 0.03))
        scaled = t * 3.0
        index = min(2, int(scaled))
        local = scaled - index
        return tuple(anchors[index][axis] * (1.0 - local) +
                     anchors[index + 1][axis] * local for axis in range(3))

    def publish_flight_telemetry(self) -> None:
        with self.flight_lock:
            samples = list(self.flight_samples)
            latest_sample = samples[-1] if samples else None
            current_velocity = self.velocity_world.copy()
            current_odom = self.odom
        if len(samples) > self.flight_visualization_max_points:
            step = max(1, math.ceil(
                (len(samples) - 1) / (self.flight_visualization_max_points - 1)))
            samples = samples[::step]
            if latest_sample is not None and samples[-1][0] != latest_sample[0]:
                samples.append(latest_sample)

        # Build and serialize outside flight_lock. Odom callbacks must keep
        # running even when RViz/DDS is slow to accept a large MarkerArray.
        marker_array = MarkerArray()
        trajectory = Marker()
        trajectory.header.frame_id = self.args.world_frame
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.ns = "openseek_flight_trajectory"
        trajectory.id = 0
        trajectory.type = Marker.LINE_LIST
        trajectory.action = Marker.ADD
        trajectory.scale.x = 0.10
        for previous, current in zip(samples[:-1], samples[1:]):
            speed = 0.5 * (float(np.linalg.norm(previous[2])) + float(np.linalg.norm(current[2])))
            r, g, b = self.speed_color(speed, self.args.trajectory_speed_color_max_mps)
            trajectory.points.extend([self.point_msg(previous[1]), self.point_msg(current[1])])
            color = self.color_msg(r, g, b)
            trajectory.colors.extend([color, color])
        marker_array.markers.append(trajectory)
        vehicle = Marker()
        vehicle.header = trajectory.header
        vehicle.ns = "openseek_flight_vehicle"
        vehicle.id = 1
        vehicle.type = Marker.ARROW
        vehicle.action = Marker.ADD
        if current_odom is not None:
            vehicle.pose = current_odom.pose.pose
            r, g, b = self.speed_color(
                float(np.linalg.norm(current_velocity)),
                self.args.trajectory_speed_color_max_mps)
            vehicle.color = self.color_msg(r, g, b)
        vehicle.scale.x = 1.25
        vehicle.scale.y = 0.30
        vehicle.scale.z = 0.30
        marker_array.markers.append(vehicle)
        self.flight_pub.publish(marker_array)
        now = time.monotonic()
        if now - self.flight_last_report >= 5.0:
            self.flight_last_report = now
            self.write_flight_statistics(False)

    @staticmethod
    def point_msg(values: np.ndarray) -> Point:
        point = Point()
        point.x, point.y, point.z = (float(values[0]), float(values[1]), float(values[2]))
        return point

    @staticmethod
    def color_msg(r: float, g: float, b: float):
        from std_msgs.msg import ColorRGBA
        color = ColorRGBA()
        color.r, color.g, color.b, color.a = float(r), float(g), float(b), 1.0
        return color

    def write_flight_statistics(self, final: bool) -> None:
        with self.flight_lock:
            duration = self.flight_duration_s
            average = self.flight_speed_integral / duration if duration > 1e-6 else 0.0
            rms_jerk = math.sqrt(self.flight_jerk_squared_integral / duration) if duration > 1e-6 else 0.0
            path = self.event_log_dir / "yopo_flight_statistics.csv"
            path.parent.mkdir(parents=True, exist_ok=True)
            new_file = not path.exists() or path.stat().st_size == 0
            with path.open("a", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                if new_file:
                    writer.writerow(["wall_time", "source", "final", "path_m", "duration_s",
                                     "current_speed_mps", "average_speed_mps", "max_speed_mps",
                                     "max_acceleration_mps2", "jerk_rms_mps3", "max_jerk_mps3"])
                writer.writerow([time.time(), "yopo", int(final), self.flight_path_length_m,
                                 duration, float(np.linalg.norm(self.velocity_world)), average,
                                 self.flight_max_speed_mps, self.flight_max_acceleration_mps2,
                                 rms_jerk, self.flight_max_jerk_mps3])

    def on_goal(self, message: PoseStamped) -> None:
        source_frame = message.header.frame_id or self.args.world_frame
        if source_frame != self.args.world_frame:
            self.get_logger().error(
                f"goal frame {source_frame!r} is unsupported; expected "
                f"{self.args.world_frame!r}"
            )
            return
        goal = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float32,
        )
        if not np.isfinite(goal).all():
            self.get_logger().error("rejected non-finite /goal_pose")
            return
        with self.lock:
            accepted_goal, changed, trajectory_valid = accept_local_goal(
                self.goal_world, goal, self.trajectory_valid_for_control)
            if not changed:
                return
            self.goal_world = accepted_goal
            # Without a separate EPIC mission topic, /goal_pose is the final
            # task goal rather than a rolling local waypoint.  Keeping this
            # unset disables both near-goal braking and arrival hold, causing
            # YOPO to keep selecting forward primitives around an already-reached goal.
            self.mission_goal_world = mission_goal_for_local_goal(
                accepted_goal,
                self.mission_goal_world,
                has_separate_mission_goal=bool(self.args.mission_goal_topic),
            )
            if not self.args.mission_goal_topic:
                self.mission_complete = False
            # EPIC advances the local waypoint continuously. Keep executing
            # the current verified trajectory until the next inference swaps
            # in its replacement; invalidating here creates a 5 Hz stop/start.
            self.trajectory_valid_for_control = trajectory_valid
        goal_message = PoseStamped()
        goal_message.header.stamp = self.get_clock().now().to_msg()
        goal_message.header.frame_id = self.args.world_frame
        goal_message.pose.position.x = float(goal[0])
        goal_message.pose.position.y = float(goal[1])
        goal_message.pose.position.z = float(goal[2])
        goal_message.pose.orientation.w = 1.0
        self.goal_visual_pub.publish(goal_message)
        if self.args.original_goal_input and self.model_vertical_num == 1:
            self.get_logger().info(
                f"new map goal=({goal[0]:.2f},{goal[1]:.2f}); "
                "fixed-height model ignores goal z"
            )
        else:
            self.get_logger().info(
                f"new map goal=({goal[0]:.2f},{goal[1]:.2f},{goal[2]:.2f})"
            )

    def on_mission_goal(self, message: PoseStamped) -> None:
        source_frame = message.header.frame_id or self.args.world_frame
        if source_frame != self.args.world_frame:
            self.get_logger().error(
                f"mission goal frame {source_frame!r} is unsupported; expected "
                f"{self.args.world_frame!r}"
            )
            return
        goal = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float32,
        )
        if not np.isfinite(goal).all():
            self.get_logger().error("rejected non-finite mission goal")
            return
        with self.lock:
            # EPIC projects the task goal onto the current fixed-height graph
            # layer. Mirror that contract here so a command such as z=0.05
            # still matches EPIC's final rolling waypoint at flight altitude.
            fixed_altitude = (
                float(self.odom.pose.pose.position.z)
                if self.fixed_altitude and self.odom is not None
                else None
            )
            goal = project_goal_to_fixed_altitude(goal, fixed_altitude)
            if (
                self.mission_goal_world is not None
                and np.linalg.norm(self.mission_goal_world - goal) <= 1e-3
            ):
                return
            self.mission_goal_world = goal
            self.mission_complete = False
        self.get_logger().info(
            f"new mission goal=({goal[0]:.2f},{goal[1]:.2f},{goal[2]:.2f})"
        )

    @staticmethod
    def decode_depth(message: Image) -> np.ndarray:
        if message.encoding == "32FC1":
            dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
            row_values = message.step // 4
            values = np.frombuffer(message.data, dtype=dtype)
        elif message.encoding == "16UC1":
            dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
            row_values = message.step // 2
            values = np.frombuffer(message.data, dtype=dtype).astype(np.float32) / 1000.0
        else:
            raise ValueError(f"unsupported depth encoding: {message.encoding!r}")
        expected = row_values * message.height
        if values.size < expected:
            raise ValueError("depth payload is shorter than height*step")
        return values[:expected].reshape(message.height, row_values)[:, : message.width]

    def on_depth(self, message: Image) -> None:
        if not self.inference_lock.acquire(blocking=False):
            return
        try:
            self.on_depth_locked(message)
        finally:
            self.inference_lock.release()

    @torch.inference_mode()
    def on_depth_locked(self, message: Image) -> None:
        receive_time = time.monotonic()
        self.last_depth_time = receive_time
        with self.lock:
            odom = self.odom
            goal_world = None if self.goal_world is None else self.goal_world.copy()
            mission_goal_world = (
                None
                if self.mission_goal_world is None
                else self.mission_goal_world.copy()
            )
            mission_complete = self.mission_complete
            desired_position_world = (
                None
                if self.desired_position_world is None
                else self.desired_position_world.copy()
            )
        if odom is None:
            return
        if desired_position_world is None:
            return
        if self.args.original_goal_input and goal_world is None:
            return
        # A completed mission remains under the 50 Hz hold command installed
        # on arrival.  Depth callbacks still refresh last_depth_time above so
        # the controller's sensor watchdog remains active.
        if mission_complete:
            return
        try:
            depth = self.decode_depth(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        # Match YOPO-Simple/test_yopo_ros.py exactly: preserve the complete
        # camera image, resize with nearest-neighbor only when dimensions do
        # not match, then clip/normalize and inpaint invalid pixels.
        interpolation = cv2.INTER_NEAREST
        raw_depth_meters = cv2.resize(
            depth, (self.image_width, self.image_height), interpolation=interpolation
        ).astype(np.float32)
        raw_valid = raw_depth_meters[
            np.isfinite(raw_depth_meters) & (raw_depth_meters >= self.minimum_depth)
        ]
        raw_depth_stats = None
        if raw_valid.size:
            raw_depth_stats = (
                float(np.percentile(raw_valid, 5.0)),
                float(np.median(raw_valid)),
                float(raw_depth_meters[raw_depth_meters.shape[0] // 2, raw_depth_meters.shape[1] // 2]),
            )

        depth = np.minimum(raw_depth_meters, self.max_depth) / self.max_depth
        invalid = np.isnan(depth) | (depth < self.minimum_depth / self.max_depth)
        depth = cv2.inpaint(
            np.uint8(depth * 255.0), np.uint8(invalid), 1, cv2.INPAINT_NS
        ).astype(np.float32) / 255.0
        depth_meters = depth * self.max_depth
        depth_stats = (
            float(np.percentile(depth_meters, 5.0)),
            float(np.median(depth_meters)),
            float(depth_meters[depth.shape[0] // 2, depth.shape[1] // 2]),
        )
        stamp = message.header.stamp
        self.emit_event(
            "depth",
            {
                "stamp": stamp.sec + stamp.nanosec * 1e-9,
                "frame_id": message.header.frame_id,
                "encoding": message.encoding,
                "source_shape_hw": [int(message.height), int(message.width)],
                "model_shape_hw": [int(self.image_height), int(self.image_width)],
                "raw_depth_m_p05_median_center": None
                if raw_depth_stats is None
                else self.vec(raw_depth_stats),
                "model_depth_m_p05_median_center": self.vec(depth_stats),
                "depth_png_encoding": "uint16 millimeters; divide by 1000 for meters",
            },
        )

        if self.args.original_goal_input:
            # The exported YOPO-Simple runtime consumes Depth plus a 9-D
            # [velocity, acceleration, goal] observation. It ignores the
            # OpenSeek heatmap channel, so keep this interface single-channel.
            image_array = depth[np.newaxis, np.newaxis]
        else:
            heatmap = goal_body_to_heatmap(
                np.array([self.args.search_distance, 0.0, 0.0], dtype=np.float32),
                width=self.image_width,
                height=self.image_height,
                horizontal_fov_deg=90.0,
                vertical_fov_deg=73.7398,
                horizontal_only=True,
                sigma_deg=self.args.heatmap_sigma,
                distance_scale=float(cfg["goal_length"]),
            )
            image_array = np.stack([depth, heatmap], axis=0)[None]
        image = torch.from_numpy(image_array).to(self.device, non_blocking=True)

        rotation = quaternion_rotation(odom.pose.pose.orientation)
        if self.graph is not None and self.graph_marker_pub is not None:
            self.update_graph_visualization(
                raw_depth_meters,
                odom,
                rotation,
                goal_world,
            )
        reference_position, reference_velocity, reference_acceleration = (
            self.reference_state(odom, receive_time)
        )
        body_velocity = (rotation.T @ reference_velocity).astype(np.float32)
        acceleration_body = (rotation.T @ reference_acceleration).astype(np.float32)
        observation = np.concatenate([body_velocity, acceleration_body])
        if self.args.original_goal_input:
            # Reference continuation uses the previous desired position, as
            # YOPO-Simple does. Direct replanning must use measured odometry;
            # otherwise tracking error rotates the goal behind the vehicle and
            # can make the controller circle around a stale reference.
            if self.args.plan_from_reference:
                goal_origin_world = reference_position
            else:
                goal_origin_world = np.array(
                    [
                        odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z,
                    ],
                    dtype=np.float64,
                )
            goal_delta_world = goal_world - goal_origin_world
            if self.fixed_altitude or self.model_vertical_num == 1:
                goal_delta_world[2] = 0.0
            goal_distance = float(np.linalg.norm(goal_delta_world))
            goal_body = (rotation.T @ goal_delta_world).astype(np.float32)
            observation = np.concatenate([observation, goal_body])
            final_subgoal = is_final_subgoal(
                goal_world,
                mission_goal_world,
                self.args.final_subgoal_tolerance,
            )
        else:
            goal_body = None
            goal_delta_world = None
            goal_distance = None
            final_subgoal = False

        # Intermediate EPIC waypoints remain moving YOPO targets.  Only brake
        # when EPIC's rolling target has reached the actual mission goal.
        if (
            self.args.original_goal_input
            and goal_world is not None
            and goal_distance is not None
            and final_subgoal
            and goal_distance <= self.direct_goal_distance
        ):
            direct_state = np.zeros(9, dtype=np.float32)
            direct_state[:3] = (rotation.T @ goal_delta_world).astype(np.float32)
            self.frame_index += 1
            self.emit_event(
                "model",
                {
                    "frame_index": self.frame_index,
                    "raw_depth_png": None,
                    "model_depth_png": None,
                    "depth_png_encoding": "uint16 millimeters; divide by 1000 for meters",
                    "inference_ms": 0.0,
                    "image_shape": [1, 1, self.image_height, self.image_width],
                    "motion": self.vec(observation),
                    "reference_position_world": self.vec(reference_position),
                    "reference_velocity_world": self.vec(reference_velocity),
                    "reference_acceleration_world": self.vec(reference_acceleration),
                    "body_velocity": self.vec(body_velocity),
                    "body_acceleration": self.vec(acceleration_body),
                    "goal_delta_world": self.vec(goal_delta_world),
                    "goal_body": self.vec(goal_body),
                    "goal_world": self.vec(goal_world),
                    "goal_distance": goal_distance,
                    "selected": -1,
                    "score_shape": [1, 1, 1],
                    "endstate_shape": [1, 9, 1, 1],
                    "selected_score": 0.0,
                    "selected_state_body": self.vec(direct_state),
                },
            )
            self.update_trajectory(
                odom, rotation, direct_state, -1, np.zeros(1, dtype=np.float32),
                reference_position, reference_velocity, reference_acceleration,
            )
            measured_position = np.array(
                [
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                ],
                dtype=np.float32,
            )
            arrived = mission_arrived(
                measured_position,
                self.velocity_world,
                mission_goal_world,
                self.args.mission_goal_tolerance,
                self.args.mission_stop_speed,
            )
            if arrived:
                with self.lock:
                    self.mission_complete = True
                self.get_logger().info(
                    "mission complete: holding final goal "
                    f"({mission_goal_world[0]:.2f},{mission_goal_world[1]:.2f},"
                    f"{mission_goal_world[2]:.2f})"
                )
            self.inference_count += 1
            return
        motion = torch.from_numpy(observation[None]).to(
            self.device, non_blocking=True
        )

        started = time.perf_counter()
        endstate, score = self.model(image, motion)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        inference_ms = (time.perf_counter() - started) * 1000.0
        if not torch.isfinite(endstate).all() or not torch.isfinite(score).all():
            self.get_logger().error("model produced non-finite output; trajectory rejected")
            return

        scores = score[0].reshape(-1)
        selected = int(torch.argmin(scores).item())
        states = endstate[0].permute(1, 2, 0).reshape(-1, 9)
        scores_np = scores.detach().cpu().numpy()
        selected_state = states[selected].detach().cpu().numpy()
        self.frame_index += 1
        raw_depth_png = None
        model_depth_png = None
        if self.save_depth_png_enabled:
            raw_depth_png = self.depth_log_dir / f"frame_{self.frame_index:06d}_raw_depth_mm.png"
            model_depth_png = self.depth_log_dir / f"frame_{self.frame_index:06d}_model_depth_mm.png"
            self.write_depth_png(raw_depth_png, np.nan_to_num(raw_depth_meters, nan=0.0, posinf=self.max_depth, neginf=0.0))
            self.write_depth_png(model_depth_png, depth_meters)
        self.emit_event(
            "model",
            {
                "frame_index": self.frame_index,
                "raw_depth_png": None if raw_depth_png is None else str(raw_depth_png),
                "model_depth_png": None if model_depth_png is None else str(model_depth_png),
                "depth_png_encoding": "uint16 millimeters; divide by 1000 for meters",
                "inference_ms": inference_ms,
                "image_shape": list(image.shape),
                "depth_tensor_min_max_mean": self.vec(
                    [float(depth.min()), float(depth.max()), float(depth.mean())]
                ),
                "motion": self.vec(observation),
                "reference_position_world": self.vec(reference_position),
                "reference_velocity_world": self.vec(reference_velocity),
                "reference_acceleration_world": self.vec(reference_acceleration),
                "body_velocity": self.vec(body_velocity),
                "body_acceleration": self.vec(acceleration_body),
                "goal_delta_world": None if goal_delta_world is None else self.vec(goal_delta_world),
                "goal_body": None if goal_body is None else self.vec(goal_body),
                "goal_world": None if goal_world is None else self.vec(goal_world),
                "goal_distance": goal_distance,
                "selected": selected,
                "score_shape": list(score.shape),
                "endstate_shape": list(endstate.shape),
                "selected_score": float(scores_np[selected]),
                "selected_state_body": self.vec(selected_state),
                "candidate_scores": self.vec(scores.detach().cpu().numpy()),
                "candidate_states_body": [
                    self.vec(state) for state in states.detach().cpu().numpy()
                ],
            },
        )
        self.update_trajectory(
            odom,
            rotation,
            selected_state,
            selected,
            scores_np,
            reference_position,
            reference_velocity,
            reference_acceleration,
        )
        self.inference_count += 1
        self.inference_total_ms += inference_ms

    @staticmethod
    def _point_message(values: np.ndarray | list[float] | tuple[float, ...]) -> Point:
        point = Point()
        point.x, point.y, point.z = (float(value) for value in values)
        return point

    @staticmethod
    def _set_marker_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba

    def _marker(
        self,
        stamp,
        marker_id: int,
        marker_type: int,
        rgba: tuple[float, float, float, float],
        *,
        scale: float,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.args.world_frame
        marker.ns = "openseek_graph"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = float(scale)
        self._set_marker_color(marker, rgba)
        return marker

    def update_graph_visualization(
        self,
        depth_meters: np.ndarray,
        odom: Odometry,
        rotation_body_to_world: np.ndarray,
        goal_world: np.ndarray | None,
    ) -> None:
        if self.graph is None or self.graph_marker_pub is None:
            return
        position = np.array(
            [
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        if goal_world is None:
            goal = position + rotation_body_to_world @ np.array(
                [self.args.search_distance, 0.0, 0.0], dtype=np.float64
            )
        else:
            goal = np.asarray(goal_world, dtype=np.float64)
        try:
            query = DepthSafeVolumeQuery(
                depth_meters,
                horizontal_fov_deg=self.args.model_horizontal_fov,
                vertical_fov_deg=self.args.model_vertical_fov,
                robot_radius_m=self.args.graph_robot_radius,
            )
            update = self.graph.update(
                position_world=position,
                rotation_body_to_world=rotation_body_to_world,
                goal_world=goal,
                depth_query=query,
            )
            snapshot = build_graph_visualization(self.graph, update, goal)
        except (ValueError, FloatingPointError) as error:
            self.get_logger().warning(f"Graph visualization update skipped: {error}")
            return

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.args.world_frame
        clear.ns = "openseek_graph"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 1
        for state, rgba in STATE_RGBA.items():
            edges = snapshot.edge_segments[state]
            if edges:
                marker = self._marker(
                    stamp,
                    marker_id,
                    Marker.LINE_LIST,
                    rgba,
                    scale=0.075 if state == "CERTIFIED" else 0.055,
                )
                marker_id += 1
                for start, end in edges:
                    marker.points.extend(
                        [self._point_message(start), self._point_message(end)]
                    )
                markers.markers.append(marker)
            nodes = snapshot.node_points[state]
            if nodes:
                marker = self._marker(
                    stamp,
                    marker_id,
                    Marker.SPHERE_LIST,
                    rgba,
                    scale=0.34,
                )
                marker_id += 1
                marker.points.extend(self._point_message(node) for node in nodes)
                markers.markers.append(marker)

        for name, path, rgba, scale in (
            ("optimistic_path", snapshot.optimistic_path, STATE_RGBA["UNVALIDATED"], 0.10),
            ("certified_path", snapshot.certified_path, STATE_RGBA["CERTIFIED"], 0.14),
        ):
            if len(path) < 2:
                continue
            marker = self._marker(stamp, marker_id, Marker.LINE_STRIP, rgba, scale=scale)
            marker_id += 1
            marker.ns = f"openseek_graph/{name}"
            marker.points.extend(self._point_message(point) for point in path)
            markers.markers.append(marker)

        for label, point, rgba, marker_type, scale in (
            ("current", snapshot.current, (0.10, 0.42, 0.75, 1.0), Marker.SPHERE, 0.55),
            ("goal", snapshot.goal, (1.0, 1.0, 1.0, 1.0), Marker.CUBE, 0.60),
        ):
            marker = self._marker(stamp, marker_id, marker_type, rgba, scale=scale)
            marker_id += 1
            marker.ns = f"openseek_graph/{label}"
            marker.pose.position = self._point_message(point)
            markers.markers.append(marker)
        if snapshot.waypoint is not None:
            marker = self._marker(
                stamp,
                marker_id,
                Marker.CUBE,
                (0.0, 0.75, 0.80, 1.0),
                scale=0.52,
            )
            marker.ns = "openseek_graph/waypoint"
            marker.pose.position = self._point_message(snapshot.waypoint)
            markers.markers.append(marker)
        self.graph_marker_pub.publish(markers)
        self.emit_event(
            "graph",
            {
                "node_count": len(self.graph.nodes),
                "edge_count": len(self.graph.edges),
                "state_counts": update.state_counts,
                "certified_path": list(update.certified_path),
                "optimistic_path": list(update.optimistic_path),
                "certified_waypoint_body": None
                if update.certified_waypoint_world is None
                else self.vec(rotation_body_to_world.T @ (update.certified_waypoint_world - position)),
            },
        )

    def reference_state(
        self, odom: Odometry, sample_clock: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self.lock:
            polynomials = self.polynomials
            started = self.trajectory_started
            valid = self.trajectory_valid_for_control
        if self.args.plan_from_reference and polynomials is not None and valid:
            sample_time = min(max(sample_clock - started, 0.0), self.segment_time)
            position = np.array(
                [polynomial.get_position(sample_time) for polynomial in polynomials],
                dtype=np.float64,
            )
            velocity = np.array(
                [polynomial.get_velocity(sample_time) for polynomial in polynomials],
                dtype=np.float64,
            )
            acceleration = np.array(
                [polynomial.get_acceleration(sample_time) for polynomial in polynomials],
                dtype=np.float64,
            )
            measured_position = np.array(
                [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z],
                dtype=np.float64,
            )
            measured_velocity = self.velocity_world.astype(np.float64, copy=True)
            position_error = float(np.linalg.norm(position - measured_position))
            velocity_error = float(np.linalg.norm(velocity - measured_velocity))
            self.last_tracking_error = position_error
            if (
                position_error <= self.args.reference_reset_position_error
                and velocity_error <= self.args.reference_reset_velocity_error
            ):
                return position, velocity, acceleration
            self.get_logger().warning(
                "reference reset: position_error=%.2f m velocity_error=%.2f m/s"
                % (position_error, velocity_error)
            )

        position = np.array(
            [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z],
            dtype=np.float64,
        )
        velocity = self.velocity_world.astype(np.float64, copy=True)
        # Direct replanning starts from the measured state on all derivatives.
        # Mixing measured P/V with the previous trajectory's desired A creates
        # a false boundary condition and bends every newly generated segment.
        return position, velocity, self.acceleration_world.astype(np.float64, copy=True)

    def update_trajectory(
        self,
        odom: Odometry,
        rotation_world_body: np.ndarray,
        state_body: np.ndarray,
        selected: int,
        scores: np.ndarray,
        start_position: np.ndarray,
        start_velocity: np.ndarray,
        start_acceleration: np.ndarray,
    ) -> None:
        position = start_position.astype(np.float64, copy=True)
        yaw = quaternion_yaw(odom.pose.pose.orientation)
        if self.args.original_goal_input:
            rotation = rotation_world_body
        else:
            cosine, sine = math.cos(yaw), math.sin(yaw)
            rotation = np.array(
                [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
        end_position = position + rotation @ state_body[0:3]
        end_velocity = rotation @ state_body[3:6]
        end_acceleration = rotation @ state_body[6:9]
        if self.fixed_altitude:
            with self.lock:
                fixed_goal = None if self.goal_world is None else self.goal_world.copy()
            if fixed_goal is not None and np.isfinite(fixed_goal[2]):
                end_position[2] = float(fixed_goal[2])
                end_velocity[2] = 0.0
                end_acceleration[2] = 0.0
        polynomials = tuple(
            Poly5Solver(
                position[axis], start_velocity[axis], start_acceleration[axis],
                end_position[axis], end_velocity[axis], end_acceleration[axis],
                self.segment_time,
            )
            for axis in range(3)
        )
        endpoint_distance = float(np.linalg.norm(state_body[0:3]))
        # YOPO-Simple's original test node does not reject a model-selected
        # endpoint by distance. Keep the model output and let the downstream
        # controller handle the command; only reject non-finite trajectories.
        finite_trajectory = bool(
            np.isfinite(end_position).all()
            and np.isfinite(end_velocity).all()
            and np.isfinite(end_acceleration).all()
        )
        minimum_sampled_altitude = float("-inf")
        if finite_trajectory:
            minimum_sampled_altitude = min(
                float(polynomials[2].get_position(sample_time))
                for sample_time in np.linspace(0.0, self.segment_time, 41)
            )
        altitude_floor = self.args.minimum_trajectory_altitude + self.args.altitude_margin
        trajectory_valid = finite_trajectory and minimum_sampled_altitude >= altitude_floor
        if trajectory_valid:
            with self.lock:
                self.polynomials = polynomials
                self.trajectory_started = time.monotonic()
                self.trajectory_valid_for_control = True
                planned_yaw = self.planned_yaw
            self.publish_path(polynomials)
        else:
            with self.lock:
                # Keep executing the previous verified segment. Invalid model
                # output must not turn a 50 Hz controller into stop-and-go.
                if self.polynomials is None:
                    self.trajectory_valid_for_control = False
                planned_yaw = self.planned_yaw
            if finite_trajectory:
                self.get_logger().warning(
                    "trajectory rejected: min_z=%.3f m floor=%.3f m"
                    % (minimum_sampled_altitude, altitude_floor)
                )

        self.emit_event(
            "trajectory",
            {
                "frame_index": self.frame_index,
                "selected": selected,
                "candidate_count": int(scores.size),
                "selected_score": float(scores[selected]),
                "selected_state_body": self.vec(state_body),
                "endpoint_distance_body": endpoint_distance,
                "start_position_world": self.vec(position),
                "start_velocity_world": self.vec(start_velocity),
                "start_acceleration_world": self.vec(start_acceleration),
                "end_position_world": self.vec(end_position),
                "end_velocity_world": self.vec(end_velocity),
                "end_acceleration_world": self.vec(end_acceleration),
                "minimum_sampled_altitude": minimum_sampled_altitude,
                "altitude_floor": altitude_floor,
                "planned_yaw_deg": math.degrees(planned_yaw),
                "valid": trajectory_valid,
                "control_enabled": bool(self.args.control and trajectory_valid),
            },
        )

    def publish_path(self, polynomials: tuple[Poly5Solver, ...]) -> None:
        message = PathMsg()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.world_frame
        for sample_time in np.linspace(0.0, self.segment_time, 41):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(polynomials[0].get_position(sample_time))
            pose.pose.position.y = float(polynomials[1].get_position(sample_time))
            pose.pose.position.z = float(polynomials[2].get_position(sample_time))
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def publish_control(self) -> None:
        if not self.args.control:
            return
        with self.lock:
            polynomials = self.polynomials
            trajectory_started = self.trajectory_started
            trajectory_valid = self.trajectory_valid_for_control
            goal_world = None if self.goal_world is None else self.goal_world.copy()
            last_yaw = self.planned_yaw
        if (
            polynomials is None
            or not trajectory_valid
            or time.monotonic() - self.last_depth_time > self.args.depth_timeout
        ):
            return
        sample_time = min(time.monotonic() - trajectory_started, self.segment_time)
        desired_position = np.array(
            [polynomial.get_position(sample_time) for polynomial in polynomials],
            dtype=np.float64,
        )
        desired_velocity = np.array(
            [polynomial.get_velocity(sample_time) for polynomial in polynomials],
            dtype=np.float64,
        )
        desired_acceleration = np.array(
            [polynomial.get_acceleration(sample_time) for polynomial in polynomials],
            dtype=np.float64,
        )
        goal_direction = (
            desired_velocity if goal_world is None else goal_world - desired_position
        )
        planned_yaw, yaw_rate = calculate_yaw(
            desired_velocity, goal_direction, last_yaw, 0.02
        )
        command = MultiDOFJointTrajectoryPoint()
        transform = Transform()
        transform.translation.x = float(desired_position[0])
        transform.translation.y = float(desired_position[1])
        transform.translation.z = float(desired_position[2])
        transform.rotation = yaw_quaternion(planned_yaw)
        velocity = Twist()
        acceleration = Twist()
        for axis, field in enumerate(("x", "y", "z")):
            setattr(velocity.linear, field, float(desired_velocity[axis]))
            setattr(acceleration.linear, field, float(desired_acceleration[axis]))
        velocity.angular.z = float(yaw_rate)
        with self.lock:
            self.desired_position_world = desired_position
            self.desired_velocity_world = desired_velocity
            self.desired_acceleration_world = desired_acceleration
            self.planned_yaw = planned_yaw
        command.transforms.append(transform)
        command.velocities.append(velocity)
        command.accelerations.append(acceleration)

        # These values also describe the trajectory-message output mode, where
        # no direct yaw-rate command is published.
        with self.lock:
            odom = self.odom
        current_yaw = (
            planned_yaw
            if odom is None
            else quaternion_yaw(odom.pose.pose.orientation)
        )
        yaw_error = wrap_angle(planned_yaw - current_yaw)
        if self.colosseum_command_pub is not None:
            vel_cmd = self.colosseum_command_type()
            # This publisher is the world-frame ENU Colosseum topic. The
            # bridge performs the final ENU -> NED conversion.
            vel_cmd.twist.linear.x = velocity.linear.x
            vel_cmd.twist.linear.y = velocity.linear.y
            vel_cmd.twist.linear.z = velocity.linear.z
            vel_cmd.twist.angular.z = yaw_rate
            self.colosseum_command_pub.publish(vel_cmd)
            published_topic = self.control_topic
            published_type = "colosseum_interfaces/VelCmd"
        else:
            self.command_pub.publish(command)
            published_topic = "/openseek/trajectory_point"
            published_type = "trajectory_msgs/MultiDOFJointTrajectoryPoint"
        self.emit_event(
            "control",
            {
                "frame_index": self.frame_index,
                "topic": published_topic,
                "type": published_type,
                "sample_time": sample_time,
                "position_world": self.vec(
                    [transform.translation.x, transform.translation.y, transform.translation.z]
                ),
                "velocity_world": self.vec(
                    [velocity.linear.x, velocity.linear.y, velocity.linear.z]
                ),
                "acceleration_world": self.vec(
                    [acceleration.linear.x, acceleration.linear.y, acceleration.linear.z]
                ),
                "yaw_deg": math.degrees(planned_yaw),
                "current_yaw_deg": math.degrees(current_yaw),
                "yaw_error_deg": math.degrees(yaw_error),
                "yaw_rate_rad_s": yaw_rate,
            },
        )

    def report_status(self) -> None:
        if self.odom is None:
            self.get_logger().warning("waiting for /sim/odom")
        if self.args.original_goal_input and self.goal_world is None:
            self.get_logger().warning(f"waiting for {self.args.goal_topic}")
        if self.args.mission_goal_topic and self.mission_goal_world is None:
            self.get_logger().warning(f"waiting for {self.args.mission_goal_topic}")
        if self.last_depth_time == 0.0:
            self.get_logger().warning("waiting for /camera/depth/image")
        elif time.monotonic() - self.last_depth_time > self.args.depth_timeout:
            self.get_logger().error("depth stream timed out; control output stopped")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OpenSeek ROS2 online local planner")
    parser.add_argument("--model", required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--control", action="store_true")
    parser.add_argument(
        "--control-topic",
        default=os.environ.get("COLOSSEUM_CONTROL_TOPIC", ""),
        help="Publish colosseum_interfaces/VelCmd to this topic instead of the OpenSeek trajectory message.",
    )
    parser.add_argument("--search-distance", type=float, default=10.0)
    parser.add_argument("--heatmap-sigma", type=float, default=7.5)
    parser.add_argument("--depth-timeout", type=float, default=0.5)
    parser.add_argument(
        "--odom-twist-frame", choices=("body", "world"), default="world",
        help="Coordinate frame used by Odometry.twist.linear.",
    )
    parser.add_argument("--reference-reset-position-error", type=float, default=0.75)
    parser.add_argument("--reference-reset-velocity-error", type=float, default=1.5)
    parser.add_argument("--minimum-trajectory-altitude", type=float, default=0.15)
    parser.add_argument("--altitude-margin", type=float, default=0.10)
    parser.add_argument(
        "--original-goal-input",
        action="store_true",
        help="Pass Depth + velocity/acceleration + map-frame /goal_pose to the model.",
    )
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument(
        "--mission-goal-topic",
        default="",
        help="Final task goal topic; distinct from EPIC's rolling local goal.",
    )
    parser.add_argument("--mission-goal-tolerance", type=float, default=0.5)
    parser.add_argument("--mission-stop-speed", type=float, default=0.3)
    parser.add_argument("--final-subgoal-tolerance", type=float, default=0.25)
    parser.add_argument("--lidar-topic", default="/lidar/front/points")
    parser.add_argument("--world-frame", default="world_enu")
    parser.add_argument("--max-yaw-rate", type=float, default=1.5)
    parser.add_argument("--goal-tolerance", type=float, default=2.0)
    parser.add_argument("--model-image-width", type=int)
    parser.add_argument("--model-image-height", type=int)
    parser.add_argument("--model-vertical-num", type=int)
    parser.add_argument(
        "--fixed-altitude", action="store_true",
        help="Keep generated YOPO trajectories on the EPIC waypoint altitude.",
    )
    parser.add_argument(
        "--direct-goal-distance", type=float, default=3.5,
        help="Use a short deterministic trajectory below this waypoint distance.",
    )
    parser.add_argument("--event-log-dir")
    parser.add_argument(
        "--trajectory-speed-color-max-mps", type=float,
        default=float(os.environ.get("EPIC_TRAJECTORY_SPEED_COLOR_MAX_MPS", "8.0")),
    )
    parser.add_argument("--save-depth-png", action="store_true")
    parser.add_argument("--source-vertical-fov", type=float, default=73.7398)
    parser.add_argument("--model-vertical-fov", type=float, default=60.0)
    parser.add_argument("--model-horizontal-fov", type=float, default=90.0)
    parser.add_argument("--z-depth-to-ray-distance", action="store_true")
    parser.add_argument("--plan-from-reference", action="store_true")
    parser.add_argument(
        "--graph-visualization",
        action="store_true",
        help="Build the depth-native sparse Graph and publish its live MarkerArray.",
    )
    parser.add_argument("--graph-marker-topic", default="/openseek/graph_markers")
    parser.add_argument("--graph-candidate-distance", type=float, default=5.0)
    parser.add_argument("--graph-robot-radius", type=float, default=0.6)
    args = parser.parse_args()
    if not Path(args.model).is_file():
        parser.error(f"model not found: {args.model}")
    if args.search_distance <= 0.0 or args.heatmap_sigma <= 0.0:
        parser.error("search distance and heatmap sigma must be positive")
    if args.goal_tolerance <= 0.0:
        parser.error("goal tolerance must be positive")
    if (
        args.mission_goal_tolerance <= 0.0
        or args.mission_stop_speed <= 0.0
        or args.final_subgoal_tolerance <= 0.0
    ):
        parser.error("mission completion thresholds must be positive")
    if args.direct_goal_distance <= args.goal_tolerance:
        parser.error("direct goal distance must exceed goal tolerance")
    if args.model_image_width is not None and args.model_image_width <= 0:
        parser.error("model image width must be positive")
    if args.model_image_height is not None and args.model_image_height <= 0:
        parser.error("model image height must be positive")
    if args.model_vertical_num is not None and args.model_vertical_num <= 0:
        parser.error("model vertical primitive count must be positive")
    if not 0.0 < args.model_vertical_fov <= args.source_vertical_fov < 180.0:
        parser.error("vertical FOV values are invalid")
    if not 0.0 < args.model_horizontal_fov < 180.0:
        parser.error("horizontal model FOV is invalid")
    if args.max_yaw_rate <= 0.0:
        parser.error("yaw rate must be positive")
    if args.trajectory_speed_color_max_mps <= 0.0:
        parser.error("trajectory speed color maximum must be positive")
    if args.reference_reset_position_error <= 0.0 or args.reference_reset_velocity_error <= 0.0:
        parser.error("reference reset thresholds must be positive")
    if args.minimum_trajectory_altitude < 0.0 or args.altitude_margin < 0.0:
        parser.error("trajectory altitude constraints must be non-negative")
    if args.graph_candidate_distance <= 0.0 or args.graph_robot_radius <= 0.0:
        parser.error("graph candidate distance and robot radius must be positive")
    if args.device == "cuda" and not torch.cuda.is_available():
        args.device = "cpu"
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OnlinePlanner(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.write_flight_statistics(True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
