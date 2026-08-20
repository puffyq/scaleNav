#!/usr/bin/env python3
"""Replay one captured AirSim RGB-D frame through the ROS2 FRGraph chain."""

from __future__ import annotations

import json
import os
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


def _list_value(block: str, key: str) -> list[float]:
    match = re.search(rf"{re.escape(key)}\s*=\s*\[([^]]+)\]", block)
    if not match:
        raise ValueError(f"missing {key} in data.toml")
    return [float(value.strip()) for value in match.group(1).split(",")]


def load_frame(data_root: Path, scene: str, frame_index: int):
    scene_dir = data_root / scene
    text = (scene_dir / "data.toml").read_text(encoding="utf-8")
    blocks = [block for block in text.split("[[dataArray]]") if block.strip()]
    selected = None
    for block in blocks:
        match = re.search(r"frameIndex\s*=\s*(\d+)", block)
        if match and int(match.group(1)) == frame_index:
            selected = block
            break
    if selected is None:
        raise ValueError(f"frame {frame_index} not found in {scene_dir / 'data.toml'}")

    depth_name = re.search(r'depthFileName\s*=\s*"([^"]+)"', selected)
    if not depth_name:
        raise ValueError("missing depthFileName")
    depth_path = scene_dir / "Textures" / depth_name.group(1)
    depth = cv2.imread(str(depth_path), cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    if depth is None or depth.ndim != 2:
        raise ValueError(f"failed to read DepthPlanar: {depth_path}")
    depth = np.ascontiguousarray(depth.astype(np.float32))

    position = np.asarray(_list_value(selected, "posStart"), dtype=np.float64)
    orientation_wxyz = np.asarray(
        _list_value(selected, "orientationWxyz"), dtype=np.float64
    )
    horizontal_fov = float(re.search(
        r"depthCameraHorizontalFOV\s*=\s*([0-9.]+)", text
    ).group(1))
    vertical_fov = float(re.search(
        r"depthCameraVerticalFOV\s*=\s*([0-9.]+)", text
    ).group(1))
    return depth, position, orientation_wxyz, horizontal_fov, vertical_fov, depth_path


def quaternion_wxyz_to_matrix(values: np.ndarray) -> np.ndarray:
    w, x, y, z = values / np.linalg.norm(values)
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_to_quaternion_wxyz(matrix: np.ndarray) -> np.ndarray:
    # Numerically stable enough for the yaw-only AirSim captures used here.
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = 2 * np.sqrt(trace + 1)
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2 * np.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = 2 * np.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = 2 * np.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return np.asarray([w, x, y, z], dtype=np.float64)


class AirSimFrame(Node):
    def __init__(self, depth, position, orientation_wxyz, hfov, vfov, goal_distance):
        super().__init__("frgraph_airsim_frame_replay")
        self.depth = depth
        # Captured AirSim metadata is world NED / body FRD. ROS odom and RViz
        # use world ENU / body FLU, so convert once at this replay boundary.
        ned_to_enu = np.asarray([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ], dtype=np.float64)
        self.position = ned_to_enu @ position
        rotation_ned_frd = quaternion_wxyz_to_matrix(orientation_wxyz)
        self.rotation_world_flu = (
            ned_to_enu @ rotation_ned_frd @ np.diag([1.0, -1.0, -1.0])
        )
        self.orientation_world_flu = matrix_to_quaternion_wxyz(
            self.rotation_world_flu
        )
        self.initial_position = self.position.copy()
        self.hfov = hfov
        self.vfov = vfov
        self.goal = self.position + self.rotation_world_flu @ np.asarray(
            [goal_distance, 0.0, 0.0], dtype=np.float64
        )
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/sim/odom", qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PointCloud2, "/frgraph/points", self.on_pointcloud, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, "/frgraph/graph", self.on_graph, 10)
        self.create_subscription(MarkerArray, "/frgraph/free_space", self.on_free_space, 10)
        self.create_subscription(MarkerArray, "/epic/graph", self.on_epic_graph, 10)
        self.create_subscription(MarkerArray, "/epic/bubbles", self.on_epic_bubbles, 10)
        self.create_subscription(NavPath, "/epic/path", self.on_epic_path, 10)
        self.create_subscription(PoseStamped, "/epic/yopo_goal", self.on_epic_next_goal, 10)
        self.started = time.monotonic()
        self.depth_first = None
        self.goal_sent = None
        self.pointcloud_first = None
        self.graph_first = None
        self.free_space_first = None
        self.epic_graph_first = None
        self.epic_bubbles_first = None
        self.epic_path_first = None
        self.epic_next_goal_first = None
        self.pointcloud_count = 0
        self.graph = None
        self.free_space = None
        self.epic_graph = None
        self.epic_bubbles = None
        self.epic_path = None
        self.epic_next_goal = None
        self.result = None
        self.require_epic = os.environ.get("REQUIRE_EPIC", "0") == "1"
        self.replay_motion_mps = float(os.environ.get("EPIC_REPLAY_MOTION_MPS", "0"))
        self.result_delay_s = float(os.environ.get("EPIC_REPLAY_RESULT_DELAY_S", "1.5"))
        self.timer = self.create_timer(0.05, self.tick)

    def on_pointcloud(self, _message):
        self.pointcloud_count += 1
        if self.pointcloud_first is None:
            self.pointcloud_first = time.monotonic()

    def on_graph(self, message):
        if self.goal_sent is not None and self.graph_first is None:
            self.graph_first = time.monotonic()
        self.graph = message

    def on_free_space(self, message):
        if self.goal_sent is not None and self.free_space_first is None:
            self.free_space_first = time.monotonic()
        self.free_space = message

    def on_epic_graph(self, message):
        if self.goal_sent is not None and self.epic_graph_first is None:
            self.epic_graph_first = time.monotonic()
        self.epic_graph = message

    def on_epic_bubbles(self, message):
        if self.goal_sent is not None and self.epic_bubbles_first is None:
            self.epic_bubbles_first = time.monotonic()
        self.epic_bubbles = message

    def on_epic_path(self, message):
        if self.goal_sent is not None and self.epic_path_first is None:
            self.epic_path_first = time.monotonic()
        self.epic_path = message

    def on_epic_next_goal(self, message):
        if self.goal_sent is not None and self.epic_next_goal_first is None:
            self.epic_next_goal_first = time.monotonic()
        self.epic_next_goal = message

    def tick(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        if self.replay_motion_mps != 0.0:
            elapsed = max(0.0, now - self.started - 0.5)
            forward_world = self.rotation_world_flu @ np.asarray(
                [1.0, 0.0, 0.0], dtype=np.float64
            )
            self.position = (
                self.initial_position + self.replay_motion_mps * elapsed * forward_world
            )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = self.orientation_world_flu
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(self.position[0])
        transform.transform.translation.y = float(self.position[1])
        transform.transform.translation.z = float(self.position[2])
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        height, width = self.depth.shape
        depth_message = Image()
        depth_message.header.stamp = stamp
        depth_message.header.frame_id = "camera_optical"
        depth_message.height = height
        depth_message.width = width
        depth_message.encoding = "32FC1"
        depth_message.is_bigendian = False
        depth_message.step = width * 4
        depth_message.data = self.depth.astype("<f4", copy=False).tobytes()
        info = CameraInfo()
        info.header = depth_message.header
        info.height = height
        info.width = width
        info.k[0] = 0.5 * width / np.tan(np.deg2rad(self.hfov) * 0.5)
        info.k[4] = 0.5 * height / np.tan(np.deg2rad(self.vfov) * 0.5)
        info.k[2] = 0.5 * (width - 1)
        info.k[5] = 0.5 * (height - 1)
        info.k[8] = 1.0
        self.info_pub.publish(info)

        self.depth_pub.publish(depth_message)
        if self.depth_first is None:
            self.depth_first = now

        if self.goal_sent is None and now - self.started > 0.35:
            goal = PoseStamped()
            goal.header.stamp = stamp
            goal.header.frame_id = "odom"
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.goal
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = now

        epic_ready = (
            not self.require_epic or
            (self.epic_graph is not None and self.epic_bubbles is not None and
             self.epic_path is not None and self.epic_next_goal is not None)
        )
        if (self.result is None and self.graph_first is not None and epic_ready and
                now - self.graph_first > self.result_delay_s):
            self.result = self.make_result()
            if os.environ.get("FRGRAPH_HOLD", "0") != "1":
                self.timer.cancel()
        elif self.result is None and now - self.started > 30.0:
            self.result = self.make_result()
            if os.environ.get("FRGRAPH_HOLD", "0") != "1":
                self.timer.cancel()

    @staticmethod
    def elapsed_ms(start, end):
        return None if start is None or end is None else (end - start) * 1000.0

    def make_result(self):
        markers = self.graph.markers if self.graph else []
        free_markers = self.free_space.markers if self.free_space else []
        epic_graph_markers = self.epic_graph.markers if self.epic_graph else []
        epic_bubble_markers = self.epic_bubbles.markers if self.epic_bubbles else []
        epic_path_points = []
        if self.epic_path:
            epic_path_points = [
                [float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)]
                for p in self.epic_path.poses
            ]
        epic_markers_by_ns = {
            marker.ns: marker for marker in epic_graph_markers
        }
        epic_skeleton_nodes = (
            len(epic_markers_by_ns["epic_skeleton_nodes"].points)
            if "epic_skeleton_nodes" in epic_markers_by_ns else 0
        )
        epic_goal_directed_nodes = (
            len(epic_markers_by_ns["epic_goal_directed_nodes"].points)
            if "epic_goal_directed_nodes" in epic_markers_by_ns else 0
        )
        epic_edge_witness_segments = 0
        if "epic_edge_witness_paths" in epic_markers_by_ns:
            epic_edge_witness_segments = len(
                epic_markers_by_ns["epic_edge_witness_paths"].points
            ) // 2
        epic_real_bubbles = sum(
            1 for marker in epic_bubble_markers
            if marker.ns == "epic_real_bubbles"
        )
        epic_next_goal = None
        if self.epic_next_goal is not None:
            epic_next_goal = np.asarray([
                self.epic_next_goal.pose.position.x,
                self.epic_next_goal.pose.position.y,
                self.epic_next_goal.pose.position.z,
            ], dtype=np.float64)
        epic_next_goal_ahead = bool(
            epic_next_goal is not None and
            np.linalg.norm(epic_next_goal - self.position) > 0.1
        )
        path_marker = next(
            (marker for marker in markers if marker.ns == "frgraph_optimistic_path"),
            None,
        )
        path = [
            np.asarray([point.x, point.y, point.z], dtype=np.float64)
            for point in (path_marker.points if path_marker else [])
        ]

        def path_hits_depth(start, end, robot_radius=0.20):
            if start is None or end is None:
                return None
            fx = 0.5 * self.depth.shape[1] / np.tan(np.deg2rad(self.hfov) * 0.5)
            fy = 0.5 * self.depth.shape[0] / np.tan(np.deg2rad(self.vfov) * 0.5)
            cx = 0.5 * (self.depth.shape[1] - 1)
            cy = 0.5 * (self.depth.shape[0] - 1)
            for alpha in np.linspace(0.0, 1.0, 160):
                world = start * (1.0 - alpha) + end * alpha
                body = self.rotation_world_flu.T @ (world - self.position)
                if body[0] <= 0.05:
                    continue
                u = int(round(cx - fx * body[1] / body[0]))
                v = int(round(cy - fy * body[2] / body[0]))
                if not (1 <= u < self.depth.shape[1] - 1 and 1 <= v < self.depth.shape[0] - 1):
                    continue
                local = self.depth[v - 1:v + 2, u - 1:u + 2]
                finite = local[np.isfinite(local) & (local > 0.05)]
                if finite.size and body[0] >= float(finite.min()) - robot_radius:
                    return True
            return False

        height, width = self.depth.shape
        fx = 0.5 * width / np.tan(np.deg2rad(self.hfov) * 0.5)
        fy = 0.5 * height / np.tan(np.deg2rad(self.vfov) * 0.5)
        cx = 0.5 * (width - 1)
        cy = 0.5 * (height - 1)
        pixel_v, pixel_u = np.indices(self.depth.shape, dtype=np.float64)
        valid_surface = (
            np.isfinite(self.depth) & (self.depth > 0.05) &
            (self.depth < 20.0 - 1e-4)
        )
        surface_depth = self.depth[valid_surface].astype(np.float64)
        surface_body = np.column_stack([
            surface_depth,
            -(pixel_u[valid_surface] - cx) * surface_depth / fx,
            -(pixel_v[valid_surface] - cy) * surface_depth / fy,
        ])
        surface_world = self.position + surface_body @ self.rotation_world_flu.T

        def path_hits_known_surface(start, end, robot_radius=0.20):
            if start is None or end is None or surface_world.size == 0:
                return None
            segment = end - start
            squared_length = float(np.dot(segment, segment))
            if squared_length < 1e-12:
                return bool(np.any(np.linalg.norm(surface_world - start, axis=1) < robot_radius))
            alpha = np.clip(
                ((surface_world - start) @ segment) / squared_length,
                0.0,
                1.0,
            )
            closest = start + alpha[:, None] * segment
            distances = np.linalg.norm(surface_world - closest, axis=1)
            return bool(np.any(distances < robot_radius))

        direct_collision = path_hits_depth(self.position, self.goal)
        primary_collision = None
        post_waypoint_collision = None
        if len(path) >= 2:
            primary_collision = path_hits_depth(path[0], path[1])
            post_waypoint_collision = path_hits_depth(path[1], self.goal)
        epic_segment_collisions = []
        for start, end in zip(epic_path_points, epic_path_points[1:]):
            epic_segment_collisions.append(path_hits_known_surface(
                np.asarray(start, dtype=np.float64),
                np.asarray(end, dtype=np.float64),
            ))
        epic_known_collision = any(value is True for value in epic_segment_collisions)
        epic_path_fixed_height = (
            len(epic_path_points) >= 2 and
            max(point[2] for point in epic_path_points) -
            min(point[2] for point in epic_path_points) < 1e-3
        )
        epic_checks = {
            "real_bubbles_nonzero": epic_real_bubbles > 0,
            "skeleton_nodes_nonzero": epic_skeleton_nodes > 0,
            "artificial_goal_nodes_absent": epic_goal_directed_nodes == 0,
            "edge_witness_paths_nonzero": epic_edge_witness_segments > 0,
            "selected_witness_path_nonempty": len(epic_path_points) >= 2,
            "selected_witness_path_avoids_known_depth": not epic_known_collision,
            "selected_witness_path_fixed_height": epic_path_fixed_height,
            "next_yopo_goal_received": epic_next_goal is not None,
            "next_yopo_goal_ahead": epic_next_goal_ahead,
        }
        return {
            "source": "AirSim captured RGB-D",
            "graph_received": self.graph is not None,
            "free_space_received": self.free_space is not None,
            "graph_markers": len(markers),
            "free_space_markers": len(free_markers),
            "epic_graph_received": self.epic_graph is not None,
            "epic_bubbles_received": self.epic_bubbles is not None,
            "epic_path_received": self.epic_path is not None,
            "epic_next_goal_received": epic_next_goal is not None,
            "epic_next_goal": None if epic_next_goal is None else epic_next_goal.tolist(),
            "epic_graph_markers": len(epic_graph_markers),
            "epic_bubble_markers": len(epic_bubble_markers),
            "epic_real_bubbles": epic_real_bubbles,
            "epic_skeleton_nodes": epic_skeleton_nodes,
            "epic_goal_directed_nodes": epic_goal_directed_nodes,
            "epic_edge_witness_segments": epic_edge_witness_segments,
            "epic_graph_namespaces": sorted({marker.ns for marker in epic_graph_markers}),
            "epic_path": epic_path_points,
            "epic_path_segment_known_collisions": epic_segment_collisions,
            "epic_checks": epic_checks,
            "pointcloud_messages": self.pointcloud_count,
            "goal_odom": self.goal.tolist(),
            "graph_namespaces": sorted({marker.ns for marker in markers}),
            "optimistic_path": [point.tolist() for point in path],
            "depth_collision_check": {
                "direct_start_to_goal": direct_collision,
                "start_to_primary_waypoint": primary_collision,
                "primary_waypoint_to_goal": post_waypoint_collision,
            },
            "timing_ms": {
                "depth_to_pointcloud": self.elapsed_ms(self.depth_first, self.pointcloud_first),
                "depth_to_graph": self.elapsed_ms(self.depth_first, self.graph_first),
                "depth_to_free_space": self.elapsed_ms(self.depth_first, self.free_space_first),
                "goal_to_graph": self.elapsed_ms(self.goal_sent, self.graph_first),
                "goal_to_epic_graph": self.elapsed_ms(self.goal_sent, self.epic_graph_first),
                "goal_to_epic_path": self.elapsed_ms(self.goal_sent, self.epic_path_first),
                "goal_to_epic_next_goal": self.elapsed_ms(self.goal_sent, self.epic_next_goal_first),
            },
            "passed": self.graph is not None and self.free_space is not None and
                      (not self.require_epic or
                       (self.epic_graph is not None and self.epic_bubbles is not None and
                        self.epic_path is not None and all(epic_checks.values()))),
        }


def main():
    data_root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("data/Map2GraphData")
    scene = sys.argv[2] if len(sys.argv) > 2 else "Scene_0002"
    frame = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    goal_distance = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
    load_start = time.monotonic()
    depth, position, orientation, hfov, vfov, depth_path = load_frame(data_root, scene, frame)
    load_ms = (time.monotonic() - load_start) * 1000.0

    rclpy.init()
    node = AirSimFrame(depth, position, orientation, hfov, vfov, goal_distance)
    passed = False
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = node.result or {}
        result["depth_file"] = str(depth_path)
        result["image_shape"] = [int(depth.shape[1]), int(depth.shape[0])]
        result["load_ms"] = load_ms
        print(json.dumps(result, indent=2), flush=True)
        passed = bool(result.get("passed"))
        if os.environ.get("FRGRAPH_HOLD", "0") == "1":
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
