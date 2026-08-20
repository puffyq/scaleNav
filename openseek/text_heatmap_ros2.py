#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import sys
import threading
import time
from pathlib import Path

import rclpy
import numpy as np
import torch
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

from text_tracker.pearl_adapter import PEARLHeatmapEncoder
from text_tracker.ros_heatmap import (
    LatestValue,
    colorize_heatmap,
    decode_color_image,
    encode_float_image,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROMPT = "tree"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a text-conditioned PEARL probability heatmap."
    )
    parser.add_argument("--prompt", default=DEFAULT_PROMPT)
    parser.add_argument("--input-topic", default="/camera/color/image")
    parser.add_argument("--output-topic", default="/openseek/text_heatmap")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--update-rate", type=float, default=1.0)
    parser.add_argument(
        "--pearl-root", default=str(PROJECT_ROOT / "third_party" / "PEARL")
    )
    parser.add_argument("--checkpoint", default="ViT-B/16")
    parser.add_argument("--short-side", type=int, default=336)
    parser.add_argument("--crop-size", type=int, default=224)
    parser.add_argument("--stride", type=int, default=112)
    parser.add_argument("--no-propagation", action="store_true")
    arguments = sys.argv if argv is None else argv
    return parser.parse_args(remove_ros_args(args=arguments)[1:])


class TextHeatmapNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("openseek_text_heatmap")
        self.declare_parameter("prompt", args.prompt)
        self.declare_parameter("input_topic", args.input_topic)
        self.declare_parameter("output_topic", args.output_topic)
        self.declare_parameter("device", args.device)
        self.declare_parameter("update_rate", args.update_rate)

        self.prompt = self.get_parameter("prompt").value.strip()
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        update_rate = float(self.get_parameter("update_rate").value)
        if not self.prompt:
            raise ValueError("prompt cannot be empty")
        if update_rate <= 0.0:
            raise ValueError("update_rate must be greater than zero")
        self.minimum_interval = 1.0 / update_rate

        device_name = str(self.get_parameter("device").value)
        if device_name.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warning(
                f"CUDA is unavailable; falling back from {device_name!r} to CPU"
            )
            device_name = "cpu"
        self.device = torch.device(device_name)

        self.get_logger().info(
            f"loading PEARL checkpoint={args.checkpoint!r} on {self.device}"
        )
        self.encoder = PEARLHeatmapEncoder(
            args.pearl_root,
            checkpoint=args.checkpoint,
            device=self.device,
            short_side=args.short_side,
            crop_size=args.crop_size,
            stride=args.stride,
            use_propagation=not args.no_propagation,
        )
        self.encoder.prepare_prompt(self.prompt)

        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, output_topic, output_qos)
        self.raw_publisher = self.create_publisher(
            Image, f"{output_topic}_raw", output_qos
        )
        self.frames: LatestValue[tuple[object, object]] = LatestValue()
        self.subscription = self.create_subscription(
            Image, input_topic, self.on_image, qos_profile_sensor_data
        )
        self.dropped_frames = 0
        self.published_frames = 0
        self._stop = threading.Event()
        self._worker = threading.Thread(
            target=self._run_worker, name="pearl-heatmap", daemon=True
        )
        self._worker.start()
        self.get_logger().info(
            f"ready: prompt={self.prompt!r}, input={input_topic}, "
            f"output={output_topic}, max_rate={update_rate:g} Hz"
        )

    def on_image(self, message: Image) -> None:
        try:
            rgb = decode_color_image(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        if self.frames.put((rgb, copy.deepcopy(message.header))):
            self.dropped_frames += 1

    def _run_worker(self) -> None:
        next_start = 0.0
        while not self._stop.is_set():
            remaining = next_start - time.monotonic()
            if remaining > 0.0:
                self._stop.wait(min(remaining, 0.1))
                continue
            item = self.frames.get(timeout=0.1)
            if item is None:
                continue

            rgb, header = item
            started = time.monotonic()
            next_start = started + self.minimum_interval
            try:
                heatmap = self.encoder.encode_rgb(rgb, self.prompt)
                payload = encode_float_image(heatmap)
                raw_message = Image()
                raw_message.header = header
                raw_message.height = payload.height
                raw_message.width = payload.width
                raw_message.encoding = "32FC1"
                raw_message.is_bigendian = payload.is_bigendian
                raw_message.step = payload.step
                raw_message.data = payload.data
                self.raw_publisher.publish(raw_message)

                color = colorize_heatmap(heatmap)
                color_message = Image()
                color_message.header = header
                color_message.height = color.shape[0]
                color_message.width = color.shape[1]
                color_message.encoding = "bgr8"
                color_message.is_bigendian = False
                color_message.step = color.shape[1] * 3
                color_message.data = color.tobytes()
                self.publisher.publish(color_message)
                self.published_frames += 1
                elapsed_ms = (time.monotonic() - started) * 1000.0
                probability_min, probability_median, probability_max = np.percentile(
                    heatmap, (0.0, 50.0, 100.0)
                )
                self.get_logger().info(
                    f"published PEARL heatmap #{self.published_frames}: "
                    f"{payload.width}x{payload.height}, {elapsed_ms:.0f} ms, "
                    f"prob={probability_min:.3f}/{probability_median:.3f}/"
                    f"{probability_max:.3f} min/median/max, "
                    f"dropped={self.dropped_frames}"
                )
            except Exception as error:
                self.get_logger().error(f"PEARL inference failed: {error}")

    def stop(self) -> None:
        self._stop.set()
        self.frames.close()
        self._worker.join(timeout=5.0)


def main(argv: list[str] | None = None) -> None:
    arguments = sys.argv if argv is None else argv
    args = parse_args(arguments)
    rclpy.init(args=arguments)
    node: TextHeatmapNode | None = None
    try:
        node = TextHeatmapNode(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
