#!/usr/bin/env python3
import argparse
import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RgbdStreamCheck(Node):
    def __init__(self, *, exit_when_ready: bool, timeout_sec: Optional[float]) -> None:
        super().__init__("openseek_rgbd_stream_check")
        self.exit_when_ready = exit_when_ready
        self.timeout_sec = timeout_sec
        self.depth_count = 0
        self.rgb_count = 0
        self.started = time.monotonic()
        self.last_report = 0.0
        self.last_wait_report = self.started
        self.camera_info_reported = False
        self.ready_reported = False
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image", self.on_depth, 10
        )
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image", self.on_rgb, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self.on_camera_info, 10
        )
        self.status_timer = self.create_timer(3.0, self.report_wait_status)
        if self.timeout_sec is not None:
            self.timeout_timer = self.create_timer(0.2, self.check_timeout)

    def maybe_finish(self) -> None:
        if self.ready_reported:
            return
        if self.depth_count == 0 or self.rgb_count == 0 or not self.camera_info_reported:
            return
        self.ready_reported = True
        elapsed = time.monotonic() - self.started
        self.get_logger().info(
            f"RGBD stream ready after {elapsed:.1f}s: "
            f"rgb_frames={self.rgb_count}, depth_frames={self.depth_count}"
        )
        if self.exit_when_ready:
            raise SystemExit(0)

    def check_timeout(self) -> None:
        if self.timeout_sec is None or self.ready_reported:
            return
        elapsed = time.monotonic() - self.started
        if elapsed < self.timeout_sec:
            return
        missing = []
        if self.rgb_count == 0:
            missing.append("RGB")
        if self.depth_count == 0:
            missing.append("depth")
        if not self.camera_info_reported:
            missing.append("CameraInfo")
        if not missing:
            missing.append("stream readiness")
        self.get_logger().error(
            f"Timed out after {elapsed:.1f}s waiting for " + ", ".join(missing)
        )
        raise SystemExit(1)

    def report_wait_status(self) -> None:
        missing_messages = []
        if self.depth_count == 0:
            missing_messages.append("depth")
        if self.rgb_count == 0:
            missing_messages.append("RGB")
        if not self.camera_info_reported:
            missing_messages.append("CameraInfo")
        if not missing_messages:
            return

        depth_publishers = self.count_publishers("/camera/depth/image")
        rgb_publishers = self.count_publishers("/camera/color/image")
        info_publishers = self.count_publishers("/camera/depth/camera_info")
        publisher_counts = (
            f"publishers: depth={depth_publishers}, RGB={rgb_publishers}, "
            f"CameraInfo={info_publishers}"
        )
        if depth_publishers == rgb_publishers == info_publishers == 0:
            self.get_logger().warning(
                "No Colosseum camera publishers found (" + publisher_counts + "). "
                "Start 05_open_blocks_v2.sh, enter Play, then start 06_start_colosseum_ros2.sh."
            )
        else:
            self.get_logger().warning(
                "Waiting for " + ", ".join(missing_messages) + " frames; " +
                publisher_counts + ". If publishers exist but counts stay zero, "
                "check that Unity is in Play and ForwardRGBD is active."
            )

    def on_rgb(self, message: Image) -> None:
        self.rgb_count += 1
        if self.rgb_count == 1:
            self.get_logger().info(
                f"RGB OK: encoding={message.encoding!r}, bytes={len(message.data)}"
            )
        self.maybe_finish()

    def on_camera_info(self, message: CameraInfo) -> None:
        if self.camera_info_reported:
            return
        self.camera_info_reported = True
        horizontal_fov = math.degrees(2.0 * math.atan2(message.width, 2.0 * message.k[0]))
        vertical_fov = math.degrees(2.0 * math.atan2(message.height, 2.0 * message.k[4]))
        self.get_logger().info(
            f"CameraInfo OK: {message.width}x{message.height}, "
            f"horizontal_fov={horizontal_fov:.2f} deg, vertical_fov={vertical_fov:.2f} deg"
        )
        self.maybe_finish()

    def on_depth(self, message: Image) -> None:
        self.depth_count += 1
        now = time.monotonic()
        if now - self.last_report < 1.0:
            return
        self.last_report = now

        if message.encoding == "16UC1":
            self.get_logger().error(
                "Depth encoding is '16UC1' (millimeters); Unity prefab is still using "
                "the old serializer setting. Re-run OpenSeek > Configure UAV Simulation "
                "or set the depth ImageMsgSerializer encoding to 32FC1."
            )
            return
        if message.encoding != "32FC1":
            self.get_logger().error(
                f"Depth encoding is {message.encoding!r}; expected '32FC1' in meters"
            )
            return
        endian = ">" if message.is_bigendian else "<"
        row_floats = message.step // 4
        values = np.frombuffer(message.data, dtype=np.dtype(endian + "f4"))
        if values.size < row_floats * message.height:
            self.get_logger().error("Depth payload is shorter than height*step")
            return
        depth = values[: row_floats * message.height].reshape(message.height, row_floats)
        depth = depth[:, : message.width]
        valid = depth[np.isfinite(depth) & (depth > 0.0)]
        center = float(depth[message.height // 2, message.width // 2])
        elapsed = max(now - self.started, 1e-6)
        if valid.size == 0:
            self.get_logger().error("Depth frame has no finite positive values")
            return
        p05, median, p95 = np.percentile(valid, [5, 50, 95])
        out_of_range = int(np.count_nonzero(valid > 20.01))
        self.get_logger().info(
            f"Depth {message.width}x{message.height} 32FC1 meters: "
            f"center={center:.3f}, p05={p05:.3f}, median={median:.3f}, "
            f"p95={p95:.3f}, min={valid.min():.3f}, max={valid.max():.3f}, "
            f">20m={out_of_range}, receive_rate={self.depth_count / elapsed:.1f} Hz"
        )
        self.maybe_finish()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check Colosseum RGBD stream health")
    parser.add_argument(
        "--wait-until-ready",
        action="store_true",
        help="Exit 0 after RGB, depth, and CameraInfo are all received.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Optional timeout in seconds while waiting for readiness.",
    )
    args = parser.parse_args()
    if args.timeout is not None and args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RgbdStreamCheck(
        exit_when_ready=args.wait_until_ready,
        timeout_sec=args.timeout,
    )
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        raise exc
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
