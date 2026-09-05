#!/usr/bin/env python3
"""Collect replay-time graph and trajectory topics into one compact JSON file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray


def xyz(point) -> list[float]:
    return [float(point.x), float(point.y), float(point.z)]


def quaternion_matrix(quaternion) -> np.ndarray:
    x, y, z, w = (float(quaternion.x), float(quaternion.y), float(quaternion.z), float(quaternion.w))
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [[1.0 - yy - zz, xy - wz, xz + wy],
         [xy + wz, 1.0 - xx - zz, yz - wx],
         [xz - wy, yz + wx, 1.0 - xx - yy]], dtype=np.float64
    )


class ReplayCollector(Node):
    def __init__(self, output: Path, stop_file: Path | None = None) -> None:
        super().__init__("replay_graph_collector")
        self.output = output
        self.odom: list[list[float]] = []
        self.pointcloud: list[list[float]] = []
        self.last_rgb: np.ndarray | None = None
        self.last_rgb_stamp_ns = 0
        self.last_pearl: np.ndarray | None = None
        self.last_pearl_stamp_ns = 0
        self.gcn_column: int | None = None
        self.gcn_column_stamp_ns = 0
        self.poses: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}
        self.paths: dict[str, list[list[float]]] = {}
        self.graph = {"nodes": [], "edges": []}
        self.planning_snapshot: dict = {}
        self.counts = {"odom": 0, "depth_cloud": 0, "graph": 0, "astar": 0, "yopo": 0, "mpc": 0}
        self.saved = False
        self.stop_file = stop_file
        self.create_subscription(Odometry, "/sim/odom", self.on_odom, 50)
        self.create_subscription(
            PointCloud2, "/depth/points", self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Image, "/camera/color/image", self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/scalenav/text_heatmap", self.on_pearl, qos_profile_sensor_data)
        self.create_subscription(Int32, "/scalenav/gcn_frontier_column", self.on_gcn_column, 10)
        self.create_subscription(MarkerArray, "/scalenav/graph", self.on_graph, 10)
        for topic, key in (
            ("/scalenav/path", "astar"),
            ("/scalenav/route_yopo/planned_path", "yopo"),
            ("/scalenav/route_yopo/mpc_path", "mpc"),
        ):
            self.create_subscription(
                PathMessage, topic, lambda message, name=key: self.on_path(name, message), 10
            )
        if self.stop_file is not None:
            self.create_timer(0.1, self.check_stop_file)

    def check_stop_file(self) -> None:
        if self.stop_file is not None and self.stop_file.exists():
            self.save()
            rclpy.shutdown()

    def on_odom(self, message: Odometry) -> None:
        position = np.asarray(xyz(message.pose.pose.position), dtype=np.float64)
        self.odom.append(position.tolist())
        stamp = message.header.stamp
        self.poses[(stamp.sec, stamp.nanosec)] = (
            position, quaternion_matrix(message.pose.pose.orientation)
        )
        if len(self.poses) > 2048:
            del self.poses[next(iter(self.poses))]
        self.counts["odom"] += 1

    @staticmethod
    def _image_array(message: Image) -> np.ndarray | None:
        channels = 1 if message.encoding in ("mono8", "32FC1", "64FC1") else 3
        dtype = np.float32 if message.encoding == "32FC1" else (
            np.float64 if message.encoding == "64FC1" else np.uint8
        )
        itemsize = np.dtype(dtype).itemsize
        width_bytes = int(message.width) * channels * itemsize
        if message.height <= 0 or message.width <= 0 or message.step < width_bytes:
            return None
        try:
            raw = np.frombuffer(message.data, dtype=dtype)
            row_items = int(message.step) // itemsize
            array = raw.reshape(int(message.height), row_items)[:, : int(message.width) * channels]
            if channels == 1:
                return array.reshape(int(message.height), int(message.width)).copy()
            return array.reshape(int(message.height), int(message.width), channels).copy()
        except (ValueError, TypeError):
            return None

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def on_rgb(self, message: Image) -> None:
        image = self._image_array(message)
        if image is None:
            return
        if message.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.last_rgb = image
        self.last_rgb_stamp_ns = self._stamp_ns(message)

    def on_pearl(self, message: Image) -> None:
        image = self._image_array(message)
        if image is None:
            return
        if image.ndim == 3 and message.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.last_pearl = image
        self.last_pearl_stamp_ns = self._stamp_ns(message)

    def on_gcn_column(self, message: Int32) -> None:
        column = int(message.data)
        if 0 <= column < 5:
            self.gcn_column = column
            self.gcn_column_stamp_ns = self.get_clock().now().nanoseconds

    def on_cloud(self, message: PointCloud2) -> None:
        stamp = message.header.stamp
        pose = self.poses.get((stamp.sec, stamp.nanosec))
        if pose is None or message.point_step <= 0 or message.width * message.height == 0:
            return
        offsets = {field.name: field.offset for field in message.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            return
        count = int(message.width * message.height)
        dtype = ">f4" if message.is_bigendian else "<f4"
        try:
            coordinates = np.column_stack([
                np.ndarray(
                    (count,), dtype=dtype, buffer=message.data,
                    offset=offsets[name], strides=(message.point_step,),
                )
                for name in ("x", "y", "z")
            ]).astype(np.float64)
        except (TypeError, ValueError):
            return
        coordinates = coordinates[np.isfinite(coordinates).all(axis=1)]
        if not len(coordinates):
            return
        if len(coordinates) > 600:
            selection = np.linspace(0, len(coordinates) - 1, 600, dtype=np.int64)
            coordinates = coordinates[selection]
        position, rotation = pose
        world = coordinates @ rotation.T + position
        self.pointcloud.extend(world.tolist())
        self.counts["depth_cloud"] += 1

    def on_path(self, name: str, message: PathMessage) -> None:
        points = [xyz(pose.pose.position) for pose in message.poses]
        if points:
            self.paths[name] = points
        self.counts[name] += 1

    def on_graph(self, message: MarkerArray) -> None:
        candidate_nodes: list[list[float]] = []
        candidate_edges: list[list[list[float]]] = []
        candidate_semantic_edges: list[list[list[float]]] = []
        selected_path: list[list[float]] = []
        semantic_points: list[dict] = []
        current_semantic_points: list[dict] = []
        frontier_goal = None
        local_goal = None
        mission_goal = None
        vehicle = None
        vehicle_orientation = None
        stamp_ns = 0
        for marker in message.markers:
            namespace = marker.ns
            stamp_ns = max(
                stamp_ns,
                int(marker.header.stamp.sec) * 1_000_000_000
                + int(marker.header.stamp.nanosec),
            )
            if marker.action != marker.ADD:
                continue
            if namespace == "scalenav_skeleton_nodes":
                candidate_nodes.extend(xyz(point) for point in marker.points)
            elif namespace == "scalenav_skeleton_edges" and marker.type == marker.LINE_LIST:
                points = [xyz(point) for point in marker.points]
                candidate_edges.extend([points[index], points[index + 1]] for index in range(0, len(points) - 1, 2))
            elif namespace == "scalenav_semantic_links" and marker.type == marker.LINE_LIST:
                points = [xyz(point) for point in marker.points]
                candidate_semantic_edges.extend(
                    [points[index], points[index + 1]]
                    for index in range(0, len(points) - 1, 2)
                )
            elif namespace == "scalenav_astar_topology_path":
                selected_path = [xyz(point) for point in marker.points]
            elif namespace == "scalenav_semantic_points":
                fallback = [float(marker.color.r), float(marker.color.g),
                            float(marker.color.b), float(marker.color.a)]
                for index, point in enumerate(marker.points):
                    color = marker.colors[index] if index < len(marker.colors) else None
                    rgba = fallback if color is None else [
                        float(color.r), float(color.g), float(color.b), float(color.a)
                    ]
                    semantic_points.append({"position": xyz(point), "color": rgba})
            elif namespace == "scalenav_current_semantic_points":
                fallback = [float(marker.color.r), float(marker.color.g),
                            float(marker.color.b), float(marker.color.a)]
                for index, point in enumerate(marker.points):
                    color = marker.colors[index] if index < len(marker.colors) else None
                    rgba = fallback if color is None else [
                        float(color.r), float(color.g), float(color.b), float(color.a)
                    ]
                    current_semantic_points.append({"position": xyz(point), "color": rgba})
            elif namespace == "scalenav_frontier_goal":
                frontier_goal = xyz(marker.pose.position)
            elif namespace == "scalenav_local_goal":
                local_goal = xyz(marker.pose.position)
            elif namespace == "scalenav_global_goal":
                mission_goal = xyz(marker.pose.position)
            elif namespace == "scalenav_vehicle_pose":
                vehicle = xyz(marker.pose.position)
                vehicle_orientation = [
                    float(marker.pose.orientation.x),
                    float(marker.pose.orientation.y),
                    float(marker.pose.orientation.z),
                    float(marker.pose.orientation.w),
                ]
        for value_name, value in (
            ("frontier_goal", frontier_goal), ("local_goal", local_goal),
            ("mission_goal", mission_goal), ("vehicle", vehicle),
        ):
            if value is not None and not all(math.isfinite(item) for item in value):
                if value_name == "frontier_goal": frontier_goal = None
                elif value_name == "local_goal": local_goal = None
                elif value_name == "mission_goal": mission_goal = None
                else: vehicle = None
        if len(candidate_nodes) >= len(self.graph["nodes"]):
            self.graph = {"nodes": candidate_nodes, "edges": candidate_edges}
        candidate_snapshot = {
            "stamp_ns": stamp_ns,
            "nodes": candidate_nodes,
            "edges": candidate_edges,
            "semantic_edges": candidate_semantic_edges,
            "selected_path": selected_path,
            "semantic_points": semantic_points,
            "current_semantic_points": current_semantic_points,
            "frontier_goal": frontier_goal,
            "local_goal": local_goal,
            "mission_goal": mission_goal,
            "vehicle": vehicle,
            "vehicle_orientation": vehicle_orientation,
        }
        candidate_complete = bool(semantic_points) and frontier_goal is not None
        current_complete = bool(self.planning_snapshot.get("semantic_points")) and \
            self.planning_snapshot.get("frontier_goal") is not None
        candidate_richness = (frontier_goal is not None, bool(semantic_points), len(candidate_nodes))
        current_richness = (
            self.planning_snapshot.get("frontier_goal") is not None,
            bool(self.planning_snapshot.get("semantic_points")),
            len(self.planning_snapshot.get("nodes", [])),
        )
        if candidate_complete or (not current_complete and candidate_richness >= current_richness):
            self.planning_snapshot = candidate_snapshot
        self.counts["graph"] += 1

    def save(self) -> None:
        if self.saved:
            return
        self.output.parent.mkdir(parents=True, exist_ok=True)
        media = {
            "rgb_capture": self.last_rgb is not None,
            "rgb_stamp_ns": self.last_rgb_stamp_ns,
            "pearl_heatmap": self.last_pearl is not None,
            "pearl_stamp_ns": self.last_pearl_stamp_ns,
            "gcn_column": self.gcn_column,
            "gcn_column_stamp_ns": self.gcn_column_stamp_ns,
        }
        if self.last_rgb is not None:
            cv2.imwrite(str(self.output.parent / "rgb_capture.jpg"), self.last_rgb)
        if self.last_pearl is not None:
            pearl_path = self.output.parent / "pearl_heatmap.png"
            pearl_color = None
            if self.last_pearl.dtype.kind == "f":
                np.save(self.output.parent / "pearl_heatmap_raw.npy", self.last_pearl)
                values = np.nan_to_num(self.last_pearl, nan=0.0, posinf=1.0, neginf=0.0)
                low, high = float(values.min()), float(values.max())
                normalized = (values - low) / max(high - low, 1e-6)
                pearl_color = cv2.applyColorMap(
                    np.uint8(np.clip(normalized, 0.0, 1.0) * 255.0), cv2.COLORMAP_TURBO
                )
                cv2.imwrite(str(pearl_path), pearl_color)
            else:
                pearl_color = self.last_pearl
                cv2.imwrite(str(pearl_path), pearl_color)
            if self.last_rgb is not None and pearl_color is not None:
                rgb = self.last_rgb
                if rgb.ndim == 2:
                    rgb = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)
                if rgb.shape[:2] != pearl_color.shape[:2]:
                    pearl_color = cv2.resize(pearl_color, (rgb.shape[1], rgb.shape[0]))
                cv2.imwrite(
                    str(self.output.parent / "pearl_overlay.jpg"),
                    cv2.addWeighted(rgb, 0.52, pearl_color, 0.48, 0.0),
                )
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(
                {"odom": self.odom, "pointcloud": self.pointcloud, "paths": self.paths,
                 "graph": self.graph, "planning_snapshot": self.planning_snapshot,
                 "counts": self.counts, "media": media},
                ensure_ascii=True,
            ),
            encoding="utf-8",
        )
        temporary.replace(self.output)
        self.saved = True
        self.get_logger().info(f"saved replay collection to {self.output}: {self.counts}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stop-file", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = ReplayCollector(args.output, args.stop_file)

    def stop(*_unused) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
