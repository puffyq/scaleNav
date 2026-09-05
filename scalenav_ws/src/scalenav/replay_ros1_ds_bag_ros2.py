#!/usr/bin/env python3
"""Replay the 0903 ROS1 Double Sphere RGB/depth/odometry bag into ROS2."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import signal
import time

import cv2
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from scalenav.ds_camera import (
    DoubleSphereIntrinsics,
    RECORDED_MAX_DISTANCE_M,
    depth_to_original_camera_points,
    double_sphere_unproject_grid,
    make_perspective_to_ds_map,
    recorded_depth_to_perspective,
    remap_image,
)


RGB_INTRINSICS = DoubleSphereIntrinsics(
    fx=687.41007288084666,
    fy=684.4057338973347,
    cx=856.96673000481906,
    cy=848.44830446562298,
    xi=0.39966203005896744,
    alpha=0.81972596411025866,
    width=1728,
    height=1728,
)
DEPTH_INTRINSICS = DoubleSphereIntrinsics(
    fx=90.80628858336394,
    fy=90.933561368389,
    cx=253.42890566638994,
    cy=259.24566303155916,
    xi=-0.31058127136562713,
    alpha=0.56406562076283007,
    width=512,
    height=512,
)


def stamp_from_ns(message_stamp, stamp_ns: int) -> None:
    message_stamp.sec = int(stamp_ns // 1_000_000_000)
    message_stamp.nanosec = int(stamp_ns % 1_000_000_000)


class DsBagPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ros1_ds_bag_replay")
        self.args = args
        self.rgb_pub = self.create_publisher(Image, "/camera/color/image", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/depth/points", qos_profile_sensor_data
        )
        self.free_ray_pub = self.create_publisher(
            PointCloud2, "/depth/free_rays", qos_profile_sensor_data
        )
        self.odom_pub = self.create_publisher(Odometry, "/sim/odom", 20)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    @staticmethod
    def image_message(array: np.ndarray, encoding: str, stamp_ns: int) -> Image:
        array = np.ascontiguousarray(array)
        message = Image()
        stamp_from_ns(message.header.stamp, stamp_ns)
        message.header.frame_id = "camera_optical_frame"
        message.height, message.width = array.shape[:2]
        message.encoding = encoding
        message.is_bigendian = False
        message.step = int(array.strides[0])
        message.data = array.tobytes()
        return message

    @staticmethod
    def camera_info_message(
        width: int,
        height: int,
        pinhole: tuple[float, float, float, float],
        stamp_ns: int,
    ) -> CameraInfo:
        fx, fy, cx, cy = pinhole
        message = CameraInfo()
        stamp_from_ns(message.header.stamp, stamp_ns)
        message.header.frame_id = "camera_optical_frame"
        message.width = width
        message.height = height
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    @staticmethod
    def pointcloud_message(points: np.ndarray, stamp_ns: int) -> PointCloud2:
        points = np.ascontiguousarray(points, dtype="<f4")
        message = PointCloud2()
        stamp_from_ns(message.header.stamp, stamp_ns)
        message.header.frame_id = "base_link"
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = points.tobytes()
        message.is_dense = bool(np.isfinite(points).all())
        return message

    def odom_message(self, source, stamp_ns: int, origin: np.ndarray) -> Odometry:
        result = Odometry()
        stamp_from_ns(result.header.stamp, stamp_ns)
        result.header.frame_id = self.args.world_frame
        result.child_frame_id = "base_link"
        p = source.pose.pose.position
        q = source.pose.pose.orientation
        xyz = np.array([p.x, p.y, p.z], dtype=np.float64) - origin
        if self.args.fixed_altitude is not None:
            xyz[2] = self.args.fixed_altitude
        result.pose.pose.position.x = float(xyz[0])
        result.pose.pose.position.y = float(xyz[1])
        result.pose.pose.position.z = float(xyz[2])
        result.pose.pose.orientation.x = float(q.x)
        result.pose.pose.orientation.y = float(q.y)
        result.pose.pose.orientation.z = float(q.z)
        result.pose.pose.orientation.w = float(q.w)
        result.pose.covariance = list(source.pose.covariance)
        linear = source.twist.twist.linear
        angular = source.twist.twist.angular
        result.twist.twist.linear.x = float(linear.x)
        result.twist.twist.linear.y = float(linear.y)
        result.twist.twist.linear.z = 0.0 if self.args.fixed_altitude is not None else float(linear.z)
        result.twist.twist.angular.x = float(angular.x)
        result.twist.twist.angular.y = float(angular.y)
        result.twist.twist.angular.z = float(angular.z)
        result.twist.covariance = list(source.twist.covariance)
        return result

    def publish_goal(self, goal: np.ndarray, stamp_ns: int) -> None:
        message = PoseStamped()
        stamp_from_ns(message.header.stamp, stamp_ns)
        message.header.frame_id = self.args.world_frame
        message.pose.position.x = float(goal[0])
        message.pose.position.y = float(goal[1])
        message.pose.position.z = float(goal[2])
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start", type=float, default=0.0, help="bag offset in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="0 replays to the end")
    parser.add_argument("--rgb-width", type=int, default=640)
    parser.add_argument("--rgb-height", type=int, default=384)
    parser.add_argument("--depth-width", type=int, default=160)
    parser.add_argument("--depth-height", type=int, default=96)
    parser.add_argument("--horizontal-fov", type=float, default=90.0)
    parser.add_argument("--vertical-fov", type=float, default=73.7398)
    parser.add_argument("--model-max-depth-m", type=float, default=20.0)
    parser.add_argument("--sensor-max-distance-m", type=float, default=RECORDED_MAX_DISTANCE_M)
    parser.add_argument("--cloud-stride", type=int, default=1)
    parser.add_argument("--free-ray-stride", type=int, default=4)
    parser.add_argument("--fixed-altitude", type=float, default=1.6)
    parser.add_argument("--preserve-odom-z", action="store_true")
    parser.add_argument("--world-frame", default="world_enu")
    parser.add_argument("--goal", type=float, nargs=3)
    parser.add_argument("--goal-from-final-odom", action="store_true")
    parser.add_argument("--odom-csv", type=Path)
    parser.add_argument("--preview-dir", type=Path)
    args = parser.parse_args()
    if args.preserve_odom_z:
        args.fixed_altitude = None
    if not args.bag.is_file():
        parser.error(f"bag not found: {args.bag}")
    if args.rate <= 0.0 or args.start < 0.0 or args.duration < 0.0:
        parser.error("rate must be positive; start and duration must be non-negative")
    if args.model_max_depth_m <= 0.0 or args.sensor_max_distance_m <= 0.0:
        parser.error("depth limits must be positive")
    if args.cloud_stride <= 0 or args.free_ray_stride <= 0:
        parser.error("point-cloud strides must be positive")
    return args


def main() -> None:
    args = parse_args()
    typestore = get_typestore(Stores.ROS1_NOETIC)
    rgb_map = make_perspective_to_ds_map(
        RGB_INTRINSICS, args.rgb_width, args.rgb_height,
        args.horizontal_fov, args.vertical_fov,
    )
    depth_rays, depth_ray_valid = double_sphere_unproject_grid(DEPTH_INTRINSICS)
    depth_pinhole = recorded_depth_to_perspective(
        np.zeros((DEPTH_INTRINSICS.height, DEPTH_INTRINSICS.width), dtype=np.uint8),
        DEPTH_INTRINSICS, args.depth_width, args.depth_height,
        args.horizontal_fov, args.vertical_fov,
        rays=depth_rays, ray_valid=depth_ray_valid,
        max_depth_m=args.model_max_depth_m,
    )[1]
    stop = False

    def request_stop(*_unused) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init()
    node = DsBagPublisher(args)
    csv_file = None
    writer = None
    if args.odom_csv:
        args.odom_csv.parent.mkdir(parents=True, exist_ok=True)
        csv_file = args.odom_csv.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(("time_s", "raw_x", "raw_y", "raw_z", "x", "y", "z", "speed_mps"))
    try:
        with Reader(args.bag) as reader:
            connections = {connection.topic: connection for connection in reader.connections}
            required = (
                "/omni_record/fisheye_img3",
                "/omni_record/depth_visual",
                "/omni_record/odom",
            )
            missing = [topic for topic in required if topic not in connections]
            if missing:
                raise RuntimeError(f"bag is missing topics: {missing}")
            odom_connection = connections["/omni_record/odom"]
            final_raw = None
            odom_sources = {}
            for connection, timestamp, raw in reader.messages(connections=[odom_connection]):
                source = typestore.deserialize_ros1(raw, connection.msgtype)
                odom_sources[int(timestamp)] = source
                if args.goal_from_final_odom:
                    p = source.pose.pose.position
                    final_raw = np.array([p.x, p.y, p.z], dtype=np.float64)
            first_odom_raw = None
            start_ns = reader.start_time + int(args.start * 1e9)
            stop_ns = None if args.duration == 0.0 else start_ns + int(args.duration * 1e9)
            wall_start = time.monotonic()
            bag_start = None
            goal = None if args.goal is None else np.asarray(args.goal, dtype=np.float64)
            goal_sent_at = 0.0
            counts = {topic: 0 for topic in required}
            published_odom_stamps = set()
            if args.preview_dir:
                args.preview_dir.mkdir(parents=True, exist_ok=True)

            def publish_odom(source, stamp_ns: int) -> None:
                nonlocal first_odom_raw, goal
                if stamp_ns in published_odom_stamps:
                    return
                p = source.pose.pose.position
                raw_position = np.array([p.x, p.y, p.z], dtype=np.float64)
                if first_odom_raw is None:
                    first_odom_raw = raw_position.copy()
                    if final_raw is not None:
                        goal = final_raw - first_odom_raw
                        if args.fixed_altitude is not None:
                            goal[2] = args.fixed_altitude
                        else:
                            # Keep absolute altitude while normalizing only the
                            # horizontal origin. This preserves P_out=R*P+t.
                            goal[2] = final_raw[2]
                origin = first_odom_raw.copy()
                if args.fixed_altitude is None:
                    origin[2] = 0.0
                published = node.odom_message(source, stamp_ns, origin)
                node.odom_pub.publish(published)
                published_odom_stamps.add(stamp_ns)
                counts[required[2]] += 1
                if writer is not None:
                    v = source.twist.twist.linear
                    writer.writerow((
                        (stamp_ns - bag_start) / 1e9,
                        *raw_position,
                        published.pose.pose.position.x,
                        published.pose.pose.position.y,
                        published.pose.pose.position.z,
                        math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
                    ))
            node.get_logger().info(
                "depth_visual decoding: d=1/(q/255*0.07812003+0.0166666667)-10; "
                "q=0 is %.1f m far-plane evidence" % args.sensor_max_distance_m
            )
            selected = [connections[topic] for topic in required]
            for connection, timestamp, raw in reader.messages(
                connections=selected, start=start_ns, stop=stop_ns
            ):
                if stop or not rclpy.ok():
                    break
                if bag_start is None:
                    bag_start = timestamp
                due = wall_start + (timestamp - bag_start) / 1e9 / args.rate
                while not stop and time.monotonic() < due:
                    rclpy.spin_once(node, timeout_sec=min(0.01, due - time.monotonic()))
                if stop or not rclpy.ok():
                    break
                source = typestore.deserialize_ros1(raw, connection.msgtype)
                stamp_ns = int(timestamp)
                # The bag stores RGB, depth, then odometry at the same stamp.
                # Publish that synchronized odometry first so point-cloud
                # projection never falls back to the previous 7 Hz pose.
                if connection.topic != required[2] and stamp_ns in odom_sources:
                    publish_odom(odom_sources[stamp_ns], stamp_ns)
                if connection.topic == required[0]:
                    image = np.frombuffer(source.data, dtype=np.uint8).reshape(
                        source.height, source.step // 3, 3
                    )[:, : source.width]
                    rectified = remap_image(image, rgb_map[0], rgb_map[1])
                    if args.preview_dir and counts[connection.topic] == 0:
                        cv2.imwrite(str(args.preview_dir / "rgb_raw.jpg"), image)
                        cv2.imwrite(str(args.preview_dir / "rgb_remap.jpg"), rectified)
                    node.rgb_pub.publish(node.image_message(rectified, "bgr8", stamp_ns))
                elif connection.topic == required[1]:
                    visual = np.frombuffer(source.data, dtype=np.uint8).reshape(
                        source.height, source.step
                    )[:, : source.width]
                    metric, _ = recorded_depth_to_perspective(
                        visual, DEPTH_INTRINSICS,
                        args.depth_width, args.depth_height,
                        args.horizontal_fov, args.vertical_fov,
                        rays=depth_rays, ray_valid=depth_ray_valid,
                        max_depth_m=args.model_max_depth_m,
                    )
                    cloud = depth_to_original_camera_points(
                        visual, DEPTH_INTRINSICS,
                        rays=depth_rays, ray_valid=depth_ray_valid,
                        max_distance_m=args.sensor_max_distance_m,
                        include_far_plane=True,
                        stride=args.cloud_stride,
                    )
                    far_valid = depth_ray_valid & (visual == 0)
                    if args.free_ray_stride > 1:
                        sampled = np.zeros_like(far_valid)
                        sampled[::args.free_ray_stride, ::args.free_ray_stride] = True
                        far_valid &= sampled
                    far_rays = depth_rays[far_valid]
                    far_distance = np.float32(args.sensor_max_distance_m)
                    free_rays = np.column_stack((
                        far_distance * far_rays[:, 0],
                        far_distance * far_rays[:, 2],
                        far_distance * far_rays[:, 1],
                    )).astype(np.float32)
                    if args.preview_dir and counts[connection.topic] == 0:
                        cv2.imwrite(str(args.preview_dir / "depth_raw.png"), visual)
                        cv2.imwrite(
                            str(args.preview_dir / "depth_perspective.png"),
                            np.uint8(np.clip(metric / args.model_max_depth_m, 0.0, 1.0) * 255.0),
                        )
                    node.depth_pub.publish(node.image_message(metric, "32FC1", stamp_ns))
                    node.info_pub.publish(node.camera_info_message(
                        args.depth_width, args.depth_height, depth_pinhole, stamp_ns
                    ))
                    node.cloud_pub.publish(node.pointcloud_message(cloud, stamp_ns))
                    node.free_ray_pub.publish(node.pointcloud_message(free_rays, stamp_ns))
                else:
                    publish_odom(source, stamp_ns)
                if connection.topic != required[2]:
                    counts[connection.topic] += 1
                now = time.monotonic()
                if goal is not None and first_odom_raw is not None and now - goal_sent_at > 0.5:
                    node.publish_goal(goal, stamp_ns)
                    goal_sent_at = now
                rclpy.spin_once(node, timeout_sec=0.0)
            node.get_logger().info(f"replay complete: {counts}")
    finally:
        if csv_file is not None:
            csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
