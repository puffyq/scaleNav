#!/usr/bin/env python3
from __future__ import annotations

import argparse

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


def quaternion_rotation(q) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = np.asarray([x, y, z, w], dtype=np.float64) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class ColosseumTfCompat(Node):
    def __init__(self, vehicle_frame: str) -> None:
        super().__init__("colosseum_tf_compat")
        self.vehicle_frame = vehicle_frame
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.odom_subscription = self.create_subscription(
            Odometry, "/sim/odom", self.publish_base_link, 10
        )
        self.timer = self.create_timer(0.1, self.publish_compatibility_frame)

    def publish_base_link(self, odom: Odometry) -> None:
        message = TransformStamped()
        message.header.stamp = odom.header.stamp
        message.header.frame_id = "world_enu"
        message.child_frame_id = "base_link"
        message.transform.translation.x = odom.pose.pose.position.x
        message.transform.translation.y = odom.pose.pose.position.y
        message.transform.translation.z = odom.pose.pose.position.z
        message.transform.rotation = odom.pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(message)

    def publish_compatibility_frame(self) -> None:
        try:
            transform = self.buffer.lookup_transform(
                "world_enu", self.vehicle_frame, Time()
            ).transform
        except TransformException:
            return

        rotation_enu_vehicle = quaternion_rotation(transform.rotation)
        translation_enu_vehicle = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            dtype=np.float64,
        )
        rotation_ned_enu = rotation_enu_vehicle.T
        translation_ned_enu = -rotation_ned_enu @ translation_enu_vehicle

        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world_ned"
        message.child_frame_id = "world_enu"
        message.transform.translation.x = float(translation_ned_enu[0])
        message.transform.translation.y = float(translation_ned_enu[1])
        message.transform.translation.z = float(translation_ned_enu[2])
        message.transform.rotation.x = -float(transform.rotation.x)
        message.transform.rotation.y = -float(transform.rotation.y)
        message.transform.rotation.z = -float(transform.rotation.z)
        message.transform.rotation.w = float(transform.rotation.w)
        self.broadcaster.sendTransform(message)
        self.timer.cancel()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle-frame", default="drone_1")
    args = parser.parse_args()
    rclpy.init()
    node = ColosseumTfCompat(args.vehicle_frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
