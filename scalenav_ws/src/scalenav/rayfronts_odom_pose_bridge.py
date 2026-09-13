#!/usr/bin/env python3
"""Bridge AirSim odometry to the PoseStamped input expected by RayFronts."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("rayfronts_odom_pose_bridge")
        self.publisher = self.create_publisher(PoseStamped, "/rayfronts/pose", 10)
        self.subscription = self.create_subscription(
            Odometry, "/sim/odom", self.on_odom, 20)

    def on_odom(self, message: Odometry) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = pose.header.frame_id or "world_enu"
        pose.pose = message.pose.pose
        self.publisher.publish(pose)


def main() -> None:
    rclpy.init()
    node = OdomPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
