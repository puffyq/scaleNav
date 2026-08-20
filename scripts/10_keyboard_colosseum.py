#!/usr/bin/env python3
"""Terminal keyboard teleop for the official Colosseum ROS2 wrapper."""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty

import rclpy
from colosseum_interfaces.msg import VelCmd
from rclpy.node import Node


class KeyboardTeleop(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("openseek_colosseum_keyboard")
        self.args = args
        self.publisher = self.create_publisher(VelCmd, args.topic, 10)
        self.command = VelCmd()
        self.command_expiry = 0.0
        self.timer = self.create_timer(1.0 / args.rate, self.publish_command)

    def set_key(self, key: str) -> bool:
        command = VelCmd()
        if key == "w":
            command.twist.linear.x = self.args.speed
        elif key == "s":
            command.twist.linear.x = -self.args.speed
        elif key == "a":
            command.twist.linear.y = self.args.speed
        elif key == "d":
            command.twist.linear.y = -self.args.speed
        elif key == "r":
            command.twist.linear.z = self.args.vertical_speed
        elif key == "f":
            command.twist.linear.z = -self.args.vertical_speed
        elif key == "q":
            command.twist.angular.z = math.radians(self.args.yaw_rate)
        elif key == "e":
            command.twist.angular.z = -math.radians(self.args.yaw_rate)
        elif key in (" ", "x"):
            self.command = VelCmd()
            self.command_expiry = 0.0
            return key != "x"
        else:
            return True
        self.command = command
        self.command_expiry = time.monotonic() + self.args.hold_time
        return True

    def publish_command(self) -> None:
        if time.monotonic() > self.command_expiry:
            self.command = VelCmd()
        self.publisher.publish(self.command)

    def stop(self) -> None:
        self.command = VelCmd()
        self.command_expiry = 0.0
        for _ in range(3):
            self.publisher.publish(self.command)
            rclpy.spin_once(self, timeout_sec=0.01)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic",
        default="/colosseum_node/drone_1/vel_cmd_body_frame",
        help="Colosseum VelCmd topic",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="forward/side speed in m/s")
    parser.add_argument("--vertical-speed", type=float, default=0.6, help="vertical speed in m/s")
    parser.add_argument("--yaw-rate", type=float, default=45.0, help="yaw rate in degrees/s")
    parser.add_argument("--hold-time", type=float, default=0.25, help="seconds a key command remains active")
    parser.add_argument("--rate", type=float, default=20.0, help="publish rate in Hz")
    args = parser.parse_args()
    if args.speed <= 0 or args.vertical_speed <= 0 or args.yaw_rate <= 0 or args.hold_time <= 0 or args.rate <= 0:
        parser.error("speed, vertical-speed, yaw-rate, hold-time, and rate must be positive")
    return args


def main() -> None:
    args = parse_args()
    if not sys.stdin.isatty():
        raise SystemExit("键盘控制需要在交互式终端运行。")
    rclpy.init()
    node = KeyboardTeleop(args)
    old_settings = termios.tcgetattr(sys.stdin)
    print("Colosseum 键盘控制已启动")
    print("W/S 前后  A/D 左右  R/F 上下  Q/E yaw  空格悬停  X 退出")
    print(f"topic={args.topic}, speed={args.speed:.2f} m/s, yaw={args.yaw_rate:.1f} deg/s")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 1.0 / args.rate)
            if readable:
                key = sys.stdin.read(1).lower()
                if not node.set_key(key):
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("键盘控制已停止，已发送零速度。")


if __name__ == "__main__":
    main()
