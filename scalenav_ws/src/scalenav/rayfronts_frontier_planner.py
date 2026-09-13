#!/usr/bin/env python3
"""Direct RayFronts frontier planner.

This node deliberately does not import or launch TopoGraph. It consumes the
RayFronts ``frontiers`` point cloud and publishes the nearest useful frontier
as a local goal for the existing YOPO executor. RayFronts remains the mapping
backend; this small planner is an isolated experimental path.
"""

from __future__ import annotations

import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class RayFrontierPlanner(Node):
    def __init__(self) -> None:
        super().__init__("rayfronts_frontier_planner")
        self.lock = threading.RLock()
        self.position = np.zeros(3, dtype=np.float32)
        self.goal = np.array([0.0, 140.0, 1.6], dtype=np.float32)
        self.frontiers = np.empty((0, 3), dtype=np.float32)
        self.have_odom = False
        self.have_goal = False
        self.have_frontiers = False
        self.local_goal_distance = float(self.declare_parameter("local_goal_distance", 12.0).value)
        self.layer_z = float(self.declare_parameter("layer_z", 1.6).value)
        self.prompt = str(self.declare_parameter("prompt", "blocks, walls, box").value)
        self.sub_odom = self.create_subscription(Odometry, "/sim/odom", self.on_odom, 20)
        self.sub_goal = self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.sub_frontiers = self.create_subscription(
            PointCloud2, "/rayfronts/msg_serv/frontiers", self.on_frontiers,
            10)
        self.pub_query = self.create_publisher(String, "/rayfronts/msg_serv/new_text_query", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/scalenav/local_goal", 10)
        self.timer = self.create_timer(0.1, self.publish_goal)
        self.query_timer = self.create_timer(1.0, self.publish_query)

    def publish_query(self) -> None:
        message = String()
        message.data = self.prompt
        self.pub_query.publish(message)

    def on_odom(self, message: Odometry) -> None:
        with self.lock:
            self.position[:] = [message.pose.pose.position.x,
                                message.pose.pose.position.y,
                                message.pose.pose.position.z]
            self.have_odom = True

    def on_goal(self, message: PoseStamped) -> None:
        with self.lock:
            self.goal[:] = [message.pose.position.x, message.pose.position.y,
                            message.pose.position.z]
            self.have_goal = bool(np.isfinite(self.goal).all())

    def on_frontiers(self, message: PointCloud2) -> None:
        try:
            points = np.asarray([
                (float(x), float(y), float(z))
                for x, y, z in point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True)
            ], dtype=np.float32)
        except (RuntimeError, TypeError, ValueError):
            return
        with self.lock:
            self.frontiers = points if points.size else np.empty((0, 3), dtype=np.float32)
            self.have_frontiers = bool(len(self.frontiers))

    def publish_goal(self) -> None:
        with self.lock:
            if not self.have_odom or not self.have_goal:
                return
            position = self.position.copy()
            goal = self.goal.copy()
            frontiers = self.frontiers.copy()
        if len(frontiers):
            delta = frontiers[:, :2] - position[:2]
            distance = np.linalg.norm(delta, axis=1)
            forward = (goal[:2] - position[:2])
            forward_norm = np.linalg.norm(forward)
            if forward_norm > 1e-3:
                alignment = (delta @ (forward / forward_norm))
            else:
                alignment = np.zeros(len(frontiers), dtype=np.float32)
            valid = (distance > 1.0) & (alignment > -1.0)
            if np.any(valid):
                indices = np.flatnonzero(valid)
                score = distance[indices] - 2.0 * alignment[indices]
                point = frontiers[indices[int(np.argmin(score))]].copy()
                point[2] = self.layer_z
                direction = point[:2] - position[:2]
                norm = np.linalg.norm(direction)
                if norm > self.local_goal_distance:
                    point[:2] = position[:2] + direction * (self.local_goal_distance / norm)
            else:
                point = goal.copy()
        else:
            point = goal.copy()
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world_enu"
        message.pose.position.x = float(point[0])
        message.pose.position.y = float(point[1])
        message.pose.position.z = float(self.layer_z)
        message.pose.orientation.w = 1.0
        self.pub_goal.publish(message)


def main() -> None:
    rclpy.init()
    node = RayFrontierPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
