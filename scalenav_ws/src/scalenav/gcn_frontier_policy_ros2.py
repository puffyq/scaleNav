#!/usr/bin/env python3
"""Online five-direction GCN frontier selector for the ScaleNav graph.

The node publishes only a body-relative column (0..4).  ScaleNav still runs
its original topology A*, route processing, local-goal selection and YOPO
execution after applying that direction to frontier ranking.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
from pathlib import Path

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from train import build_model


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GcnFrontierPolicy(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("scalenav_gcn_frontier_policy")
        self.args = args
        self.device = torch.device(args.device)
        checkpoint = torch.load(args.model, map_location=self.device, weights_only=False)
        self.model = build_model(int(checkpoint["input_dim"]), checkpoint.get("architecture", "legacy"))
        self.model.load_state_dict(checkpoint["model"])
        self.model.to(self.device).eval()
        self.lock = threading.RLock()
        self.callback_group = ReentrantCallbackGroup()
        self.odom = None
        self.goal = np.array([0.0, 140.0, 1.6], dtype=np.float32)
        self.have_goal = False
        self.graph_markers: MarkerArray | None = None
        self.semantic_ranking: dict[int, dict[str, float]] = {}
        self.hidden = None
        self.previous = -1
        self.last_selected = -1
        self.publish_count = 0
        self.last_status_time = 0.0

        self.column_pub = self.create_publisher(Int32, args.output_topic, 10)
        self.marker_pub = self.create_publisher(Marker, args.marker_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, args.odom_topic, self.on_odom, 20,
            callback_group=self.callback_group)
        self.graph_sub = self.create_subscription(
            MarkerArray, args.graph_topic, self.on_graph, 10,
            callback_group=self.callback_group)
        self.timing_sub = self.create_subscription(
            String, args.timing_topic, self.on_timing, 20,
            callback_group=self.callback_group)
        self.goal_sub = self.create_subscription(
            PoseStamped, args.mission_goal_topic, self.on_goal, 10,
            callback_group=self.callback_group)
        self.timer = self.create_timer(args.publish_period, self.tick,
                                       callback_group=self.callback_group)
        self.get_logger().info(
            f"GCN frontier selector ready: model={args.model} "
            f"device={self.device} output={args.output_topic}"
        )

    def on_odom(self, message: Odometry) -> None:
        with self.lock:
            self.odom = message

    def on_goal(self, message: PoseStamped) -> None:
        frame = message.header.frame_id or self.args.world_frame
        if frame != self.args.world_frame:
            self.get_logger().warning(f"ignoring mission goal frame {frame}")
            return
        with self.lock:
            self.goal = np.array([message.pose.position.x, message.pose.position.y,
                                  message.pose.position.z], dtype=np.float32)
            self.have_goal = bool(np.isfinite(self.goal).all())

    def on_graph(self, message: MarkerArray) -> None:
        with self.lock:
            self.graph_markers = message

    def on_timing(self, message: String) -> None:
        """Cache the planner's latest semantic five-column ranking."""
        try:
            payload = json.loads(message.data)
            ranking = payload.get("semantic_frontier_ranking", [])
            parsed = {}
            for item in ranking:
                column = int(item.get("column", -1))
                if 0 <= column < 5:
                    risk = max(float(item.get("risk", 1.0)), 0.0)
                    parsed[column] = {"score": 1.0 / (1.0 + risk), "confidence": 1.0}
            with self.lock:
                self.semantic_ranking = parsed
        except (ValueError, TypeError, json.JSONDecodeError):
            return

    @staticmethod
    def nearest_edges(nodes: np.ndarray, marker: Marker | None) -> set[tuple[int, int]]:
        edges: set[tuple[int, int]] = set()
        if marker is None:
            return edges
        for i in range(0, len(marker.points) - 1, 2):
            left = np.array([marker.points[i].x, marker.points[i].y], dtype=np.float32)
            right = np.array([marker.points[i + 1].x, marker.points[i + 1].y], dtype=np.float32)
            a = int(np.argmin(np.linalg.norm(nodes - left[None], axis=1)))
            b = int(np.argmin(np.linalg.norm(nodes - right[None], axis=1)))
            if a != b:
                edges.add((a, b)); edges.add((b, a))
        return edges

    def make_sample(self, markers: MarkerArray, position: np.ndarray, yaw: float,
                    goal: np.ndarray, safe: list[bool]):
        node_marker = next((m for m in markers.markers
                            if m.ns == "scalenav_skeleton_nodes" and m.action == Marker.ADD), None)
        edge_marker = next((m for m in markers.markers
                            if m.ns == "scalenav_skeleton_edges" and m.action == Marker.ADD), None)
        if node_marker is None or not node_marker.points:
            return None
        base = np.asarray([[p.x, p.y] for p in node_marker.points], dtype=np.float32)
        if len(base) < 1:
            return None
        edges = self.nearest_edges(base, edge_marker)
        degree = np.zeros(len(base), dtype=np.float32)
        for a, _ in edges:
            degree[a] += 1.0
        nodes = base.tolist()
        frontier = []
        for column in range(5):
            angle = yaw + (column - 2) * math.radians(20.0)
            point = position[:2] + self.args.candidate_distance * np.array(
                [math.cos(angle), math.sin(angle)], dtype=np.float32)
            frontier.append(len(nodes))
            nodes.append(point.tolist())
        nodes_np = np.asarray(nodes, dtype=np.float32)
        for index in frontier:
            nearest = int(np.argmin(np.linalg.norm(base - nodes_np[index][None], axis=1)))
            edges.add((nearest, index)); edges.add((index, nearest))
        # Recompute degree after adding the five virtual candidate links. This
        # matches the training graph, where candidate edges are present before
        # node features are constructed.
        degree = np.zeros(len(nodes_np), dtype=np.float32)
        for a, _ in edges:
            degree[a] += 1.0
        columns = np.full(len(nodes), -1, dtype=np.int64)
        for column, index in enumerate(frontier):
            columns[index] = column
        odom_index = int(np.argmin(np.linalg.norm(base - position[None, :2], axis=1)))
        x = np.zeros((len(nodes), 20), dtype=np.float32)
        x[:, 0:2] = nodes_np / np.array([25.0, 80.0], dtype=np.float32)
        x[:, 2] = float(node_marker.scale.x) / 2.0 / 3.0
        x[:, 3] = np.minimum(degree, 8.0) / 8.0
        x[frontier, 2] = 1.0 / 3.0
        # build_log_graph uses risk=1.0/confidence=1.0 when a frame has no
        # semantic frontier ranking. Keep the online feature distribution
        # identical until the first PEARL ranking arrives.
        x[frontier, 4] = 0.5
        x[frontier, 5] = 1.0
        with self.lock:
            ranking = dict(self.semantic_ranking)
        for column, index in enumerate(frontier):
            item = ranking.get(column)
            if item is not None:
                x[index, 4] = float(item["score"])
                x[index, 5] = float(item["confidence"])
        x[:, 6] = (columns >= 0).astype(np.float32)
        x[:, 7] = np.maximum(columns, 0) / 4.0
        x[:, 8] = np.linalg.norm(nodes_np, axis=1) / 80.0
        x[:, 9] = np.linalg.norm(nodes_np - goal[None, :2], axis=1) / 80.0
        x[odom_index, 10] = 1.0
        x[:, 11] = (columns == 2).astype(np.float32)
        x[:, 12] = math.sin(yaw); x[:, 13] = math.cos(yaw)
        delta = nodes_np - position[None, :2]
        c, s = math.cos(yaw), math.sin(yaw)
        body_x = c * delta[:, 0] + s * delta[:, 1]
        body_y = -s * delta[:, 0] + c * delta[:, 1]
        goal_delta = goal[:2] - position[:2]
        goal_body = np.array([c * goal_delta[0] + s * goal_delta[1],
                              -s * goal_delta[0] + c * goal_delta[1]], dtype=np.float32)
        x[:, 14] = body_x / 80.0; x[:, 15] = body_y / 80.0
        x[:, 16] = np.linalg.norm(delta, axis=1) / 80.0
        x[:, 17] = np.arctan2(body_y, body_x) / math.pi
        x[:, 18] = goal_body[0] / 140.0; x[:, 19] = goal_body[1] / 140.0
        directed = list(edges)
        edge_index = torch.tensor(directed, dtype=torch.long).t().contiguous()
        weights = [1.0 / max(float(np.linalg.norm(nodes_np[a] - nodes_np[b])), 1e-3)
                   for a, b in directed]
        return {
            "x": torch.from_numpy(x).to(self.device),
            "edge_index": edge_index.to(self.device),
            "edge_weight": torch.tensor(weights, dtype=torch.float32, device=self.device),
            "frontier_index": torch.tensor(frontier, dtype=torch.long, device=self.device),
            "frontier_columns": torch.arange(5, dtype=torch.long, device=self.device),
            "safe_columns": torch.tensor(safe, dtype=torch.bool, device=self.device),
        }, nodes_np, frontier

    def tick(self) -> None:
        with self.lock:
            odom, markers, goal = self.odom, self.graph_markers, self.goal.copy()
        if odom is None or markers is None or not self.have_goal:
            return
        position = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y], dtype=np.float32)
        yaw = yaw_from_quaternion(odom.pose.pose.orientation)
        candidates = []
        for column in range(5):
            angle = yaw + (column - 2) * math.radians(20.0)
            candidates.append(position + self.args.candidate_distance *
                             np.array([math.cos(angle), math.sin(angle)], dtype=np.float32))
        # Reachability and collision safety remain the downstream A*'s job.
        # The classifier only selects one of the five relative directions.
        safe = [True] * 5
        sample_data = self.make_sample(markers, position, yaw, goal, safe)
        if sample_data is None:
            return
        sample, _, _ = sample_data
        with torch.inference_mode():
            logits, hidden = self.model(sample, self.hidden, self.previous)
        selected = int(logits.argmax().item())
        self.hidden = hidden.detach() if hidden is not None else None
        self.previous = selected
        self.last_selected = selected
        self.column_pub.publish(Int32(data=selected))
        self.publish_count += 1
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        if now_seconds - self.last_status_time >= 2.0:
            self.get_logger().info(
                f"GCN inference active: count={self.publish_count} column={selected}")
            self.last_status_time = now_seconds
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.args.world_frame
        marker.ns = "scalenav_gcn_selected"; marker.id = 0
        marker.type = Marker.ARROW; marker.action = Marker.ADD; marker.scale.x = 2.0
        marker.scale.y = 0.15; marker.scale.z = 0.15
        marker.color.r = 0.73; marker.color.b = 1.0; marker.color.a = 1.0
        marker.pose.position.x = float(position[0]); marker.pose.position.y = float(position[1]); marker.pose.position.z = float(odom.pose.pose.position.z)
        marker.pose.orientation.z = math.sin(0.5 * (yaw + (selected - 2) * math.radians(20.0)))
        marker.pose.orientation.w = math.cos(0.5 * (yaw + (selected - 2) * math.radians(20.0)))
        self.marker_pub.publish(marker)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=str(root / "train_gcn/frontier_gcn_map2_35m.pt"))
    parser.add_argument("--device", default="cuda", choices=("cuda", "cpu"))
    parser.add_argument("--odom-topic", default="/sim/odom")
    parser.add_argument("--graph-topic", default="/scalenav/graph")
    parser.add_argument("--timing-topic", default="/scalenav/timing")
    parser.add_argument("--mission-goal-topic", default="/goal_pose")
    parser.add_argument("--output-topic", default="/scalenav/gcn_frontier_column")
    parser.add_argument("--marker-topic", default="/scalenav/gcn_selected")
    parser.add_argument("--world-frame", default="world_enu")
    parser.add_argument("--candidate-distance", type=float, default=10.0)
    parser.add_argument("--publish-period", type=float, default=0.2)
    args = parser.parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise SystemExit("CUDA requested but unavailable")
    if not Path(args.model).is_file():
        raise SystemExit(f"model not found: {args.model}")
    return args


def main() -> None:
    args = parse_args(); rclpy.init(); node = GcnFrontierPolicy(args)
    executor = MultiThreadedExecutor(num_threads=3); executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
