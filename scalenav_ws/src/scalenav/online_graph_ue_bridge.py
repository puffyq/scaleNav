#!/usr/bin/env python3
from __future__ import annotations

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

import colosseum
from colosseum import Vector3r

from graph.visualization import enu_to_ned


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward OpenSeek graph MarkerArray to AirSim/Colosseum UE debug drawing."
    )
    parser.add_argument("--marker-topic", default="/scalenav/graph_markers")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--rpc-host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="")
    return parser.parse_args()


class UnrealGraphBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("scalenav_graph_ue_bridge")
        if args.rate <= 0.0:
            raise ValueError("--rate must be positive")
        self.client = colosseum.VehicleClient(ip=args.rpc_host, port=args.rpc_port)
        self.client.confirmConnection()
        self.vehicle_name = args.vehicle_name
        self.draw_duration = max(0.4, 2.5 / args.rate)
        self._lock = threading.Lock()
        self._latest: MarkerArray | None = None
        self._last_error_time = 0.0
        self.subscription = self.create_subscription(
            MarkerArray,
            args.marker_topic,
            self.on_markers,
            10,
        )
        self.timer = self.create_timer(1.0 / args.rate, self.draw_latest)
        self.get_logger().info(
            f"UE Graph bridge ready: topic={args.marker_topic}, rate={args.rate:g} Hz, "
            f"RPC={args.rpc_host}:{args.rpc_port}"
        )

    def on_markers(self, message: MarkerArray) -> None:
        with self._lock:
            self._latest = message

    @staticmethod
    def _color(marker: Marker) -> list[float]:
        return [
            float(np.clip(marker.color.r, 0.0, 1.0)),
            float(np.clip(marker.color.g, 0.0, 1.0)),
            float(np.clip(marker.color.b, 0.0, 1.0)),
            float(np.clip(marker.color.a, 0.0, 1.0)),
        ]

    @staticmethod
    def _ned_point(point: object, marker: Marker) -> Vector3r:
        # The planner publishes world ENU positions. Marker pose is only used
        # for single-point markers; graph lists carry absolute points.
        enu = np.array(
            [
                float(point.x) + float(marker.pose.position.x),
                float(point.y) + float(marker.pose.position.y),
                float(point.z) + float(marker.pose.position.z),
            ],
            dtype=np.float64,
        )
        ned = enu_to_ned(enu)
        return Vector3r(float(ned[0]), float(ned[1]), float(ned[2]))

    def draw_latest(self) -> None:
        with self._lock:
            message = self._latest
            self._latest = None
        if message is None:
            return
        try:
            for marker in message.markers:
                self._draw_marker(marker)
        except Exception as error:  # RPC failures must not kill ROS callbacks.
            now = time.monotonic()
            if now - self._last_error_time > 2.0:
                self.get_logger().error(f"UE Graph draw failed: {error}")
                self._last_error_time = now

    def _draw_marker(self, marker: Marker) -> None:
        if marker.action in (Marker.DELETE, Marker.DELETEALL):
            return
        color = self._color(marker)
        if color[3] <= 0.0:
            return
        points = [self._ned_point(point, marker) for point in marker.points]
        if marker.type == Marker.LINE_LIST:
            if len(points) >= 2:
                self.client.simPlotLineList(
                    points,
                    color_rgba=color,
                    thickness=max(1.0, float(marker.scale.x) * 45.0),
                    duration=self.draw_duration,
                    is_persistent=False,
                )
            return
        if marker.type == Marker.LINE_STRIP:
            if len(points) >= 2:
                self.client.simPlotLineStrip(
                    points,
                    color_rgba=color,
                    thickness=max(1.0, float(marker.scale.x) * 45.0),
                    duration=self.draw_duration,
                    is_persistent=False,
                )
            return
        if marker.type in (Marker.POINTS, Marker.SPHERE_LIST, Marker.CUBE_LIST):
            if points:
                self.client.simPlotPoints(
                    points,
                    color_rgba=color,
                    size=max(8.0, float(marker.scale.x) * 35.0),
                    duration=self.draw_duration,
                    is_persistent=False,
                )
            return
        if marker.type in (Marker.SPHERE, Marker.CUBE, Marker.CYLINDER):
            point = self._ned_point(marker.pose.position, Marker())
            self.client.simPlotPoints(
                [point],
                color_rgba=color,
                size=max(10.0, float(marker.scale.x) * 35.0),
                duration=self.draw_duration,
                is_persistent=False,
            )


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = UnrealGraphBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
