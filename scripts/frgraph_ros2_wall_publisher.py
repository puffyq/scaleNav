#!/usr/bin/env python3
"""Offline ROS2 input and output check for the FRGraph wall case."""

import json
import os
import struct
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray


class WallCase(Node):
    def __init__(self):
        super().__init__("frgraph_ros2_wall_case")
        self.points_pub = self.create_publisher(PointCloud2, "/frgraph/points", 10)
        self.odom_pub = self.create_publisher(Odometry, "/sim/odom", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.create_subscription(MarkerArray, "/frgraph/graph", self.on_graph, 10)
        self.create_subscription(MarkerArray, "/frgraph/free_space", self.on_free_space, 10)
        self.create_subscription(MarkerArray, "/epic/graph", self.on_epic_graph, 10)
        self.create_subscription(MarkerArray, "/epic/bubbles", self.on_epic_bubbles, 10)
        self.create_subscription(NavPath, "/epic/path", self.on_epic_path, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.started = time.monotonic()
        self.sent_goal = False
        self.graph = None
        self.free_space = None
        self.epic_graph = None
        self.epic_bubbles = None
        self.epic_path = None
        self.result = None
        self.hold = os.environ.get("WALL_HOLD", "0") == "1"
        self.require_epic = os.environ.get("REQUIRE_EPIC_WALL", "0") == "1"

    def on_graph(self, message):
        self.graph = message

    def on_free_space(self, message):
        self.free_space = message

    def on_epic_graph(self, message):
        self.epic_graph = message

    def on_epic_bubbles(self, message):
        self.epic_bubbles = message

    def on_epic_path(self, message):
        self.epic_path = message

    def tick(self):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        if not self.sent_goal and time.monotonic() - self.started > 0.8:
            goal = PoseStamped()
            goal.header.stamp = stamp
            goal.header.frame_id = "odom"
            goal.pose.position.x = 20.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.sent_goal = True

        width, height = 160, 96
        fx = fy = 0.5 * width
        cx, cy = 0.5 * (width - 1), 0.5 * (height - 1)
        points = []
        side_mode = os.environ.get("WALL_SIDES", "unknown").strip().lower()
        for v in range(height):
            for u in range(width):
                # The wall occupies the camera's central 44 columns.  In the
                # default mode, side pixels are unknown and therefore omitted
                # from the point cloud.  ``WALL_SIDES=far`` is an explicit
                # comparison case that publishes 19.5 m returns at the sides.
                if 58 <= u < 102:
                    depth = 3.0
                elif side_mode == "far":
                    depth = 19.5
                else:
                    continue
                y = -(u - cx) * depth / fx
                z = -(v - cy) * depth / fy
                points.append((depth, y, z))

        message = PointCloud2()
        message.header.stamp = stamp
        # Odom and base_link are identical in this stationary offline case.
        # Publishing in odom lets RViz display the cloud without a TF source.
        message.header.frame_id = "odom"
        message.height = 1
        message.width = len(points)
        message.is_bigendian = False
        message.is_dense = True
        message.point_step = 16
        message.row_step = message.point_step * message.width
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        # PointCloud2Modifier normally adds a 4-byte padding field for xyz.
        message.data = b"".join(struct.pack("<fffxxxx", *point) for point in points)
        self.points_pub.publish(message)

        epic_ready = (
            not self.require_epic or
            (self.epic_graph is not None and self.epic_bubbles is not None and
             self.epic_path is not None)
        )
        elapsed = time.monotonic() - self.started
        if self.result is None and epic_ready and elapsed > 4.5:
            graph_markers = self.graph.markers if self.graph else []
            free_markers = self.free_space.markers if self.free_space else []
            frontier = next((m for m in graph_markers if m.ns == "frgraph_frontier_nodes"), None)
            goal = next((m for m in graph_markers if m.ns == "frgraph_global_goal"), None)
            edges = next((m for m in graph_markers if m.ns == "frgraph_edges"), None)
            frontier_points = [
                [float(p.x), float(p.y), float(p.z)]
                for p in (frontier.points if frontier else [])
            ]
            edge_points = [
                [float(p.x), float(p.y), float(p.z)]
                for p in (edges.points if edges else [])
            ]
            primary_edge = edge_points[:2] if len(edge_points) >= 2 else None

            # Ground-truth wall bounds generated above.  Evaluate each root
            # ray where it crosses x=3 m; a valid bypass must clear the wall
            # laterally or vertically at that plane.
            wall_half_y = max(abs(-(u - cx) * 3.0 / fx) for u in (58, 101))
            wall_half_z = max(abs(-(v - cy) * 3.0 / fy) for v in (0, height - 1))

            def clears_wall(point):
                x, y, z = point
                if x <= 3.0:
                    return False
                scale = 3.0 / x
                return abs(y * scale) > wall_half_y or abs(z * scale) > wall_half_z

            safe_frontiers = [point for point in frontier_points if clears_wall(point)]
            namespaces = {m.ns for m in graph_markers}
            checks = {
                "graph_received": self.graph is not None,
                "free_space_received": self.free_space is not None and bool(free_markers),
                "has_start_node": "frgraph_start_node" in namespaces,
                "has_goal_node": "frgraph_goal_node" in namespaces,
                "has_optimistic_path": "frgraph_optimistic_path" in namespaces,
                "has_forward_left_bypass": any(p[0] > 3.0 and p[1] > 0.0 and clears_wall(p)
                                               for p in frontier_points),
                "has_forward_right_bypass": any(p[0] > 3.0 and p[1] < 0.0 and clears_wall(p)
                                                for p in frontier_points),
                "primary_edge_bypasses_wall": bool(primary_edge and clears_wall(primary_edge[1])),
            }

            epic_graph_markers = self.epic_graph.markers if self.epic_graph else []
            epic_bubble_markers = self.epic_bubbles.markers if self.epic_bubbles else []
            epic_by_ns = {marker.ns: marker for marker in epic_graph_markers}
            epic_path_points = np.asarray([
                [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                for pose in (self.epic_path.poses if self.epic_path else [])
            ], dtype=np.float64)
            known_surface = np.asarray(points, dtype=np.float64)

            def segment_hits_known_surface(start, end, radius=0.20):
                segment = end - start
                squared_length = float(np.dot(segment, segment))
                if squared_length < 1e-12:
                    return bool(np.any(np.linalg.norm(known_surface - start, axis=1) < radius))
                alpha = np.clip(
                    ((known_surface - start) @ segment) / squared_length,
                    0.0,
                    1.0,
                )
                closest = start + alpha[:, None] * segment
                return bool(np.any(np.linalg.norm(known_surface - closest, axis=1) < radius))

            epic_segment_collisions = [
                segment_hits_known_surface(start, end)
                for start, end in zip(epic_path_points, epic_path_points[1:])
            ]
            epic_checks = {
                "real_bubbles_nonzero": any(
                    marker.ns == "epic_real_bubbles" for marker in epic_bubble_markers
                ),
                "artificial_goal_nodes_absent": (
                    "epic_goal_directed_nodes" not in epic_by_ns or
                    not epic_by_ns["epic_goal_directed_nodes"].points
                ),
                "skeleton_nodes_nonzero": (
                    "epic_skeleton_nodes" in epic_by_ns and
                    bool(epic_by_ns["epic_skeleton_nodes"].points)
                ),
                "edge_witness_paths_nonzero": (
                    "epic_edge_witness_paths" in epic_by_ns and
                    bool(epic_by_ns["epic_edge_witness_paths"].points)
                ),
                "selected_witness_path_nonempty": len(epic_path_points) >= 2,
                "selected_witness_path_avoids_wall": not any(epic_segment_collisions),
            }
            if self.require_epic:
                checks.update({f"epic_{key}": value for key, value in epic_checks.items()})
            self.result = {
                "graph_received": self.graph is not None,
                "graph_markers": len(graph_markers),
                "free_space_received": self.free_space is not None,
                "free_space_markers": len(free_markers),
                "graph_namespaces": sorted(namespaces),
                "frontier_count": len(frontier_points),
                "safe_frontier_count": len(safe_frontiers),
                "global_goal": (
                    [float(goal.pose.position.x), float(goal.pose.position.y), float(goal.pose.position.z)]
                    if goal else None
                ),
                "wall_sides": side_mode,
                "point_count": len(points),
                "wall_bounds_at_x_3m": {
                    "half_width_y_m": wall_half_y,
                    "half_height_z_m": wall_half_z,
                },
                "primary_edge": primary_edge,
                "epic_real_bubbles": sum(
                    1 for marker in epic_bubble_markers if marker.ns == "epic_real_bubbles"
                ),
                "epic_skeleton_nodes": (
                    len(epic_by_ns["epic_skeleton_nodes"].points)
                    if "epic_skeleton_nodes" in epic_by_ns else 0
                ),
                "epic_edge_witness_segments": (
                    len(epic_by_ns["epic_edge_witness_paths"].points) // 2
                    if "epic_edge_witness_paths" in epic_by_ns else 0
                ),
                "epic_path": epic_path_points.tolist(),
                "epic_path_segment_known_collisions": epic_segment_collisions,
                "epic_checks": epic_checks,
                "checks": checks,
                "passed": all(checks.values()),
            }
            if not self.hold:
                self.timer.cancel()
        elif self.result is None and elapsed > 30.0:
            self.result = {"passed": False, "error": "timed out waiting for graph outputs"}
            if not self.hold:
                self.timer.cancel()


def main():
    rclpy.init()
    node = WallCase()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        print(json.dumps(node.result or {}, indent=2), flush=True)
        passed = bool(node.result and node.result.get("passed"))
        while node.hold and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
