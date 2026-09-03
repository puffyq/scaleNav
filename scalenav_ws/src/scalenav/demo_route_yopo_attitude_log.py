#!/usr/bin/env python3
"""Inspect Route-YOPO attitude and command limits from a scalenav log."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def euler_from_xyzw(q):
    x, y, z, w = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    odom_angles = []
    altitudes = []
    command_speeds = []
    command_accelerations = []
    for line in (args.session / "index.jsonl").open(encoding="utf-8"):
        record = json.loads(line)
        data = record.get("data", {})
        if record.get("kind") == "odom":
            angles = euler_from_xyzw(data["orientation"])
            odom_angles.append(tuple(math.degrees(value) for value in angles))
            altitudes.append(float(data["position"][2]))
        elif record.get("kind") == "control":
            velocity = data.get("velocity", data.get("velocity_world", [0.0, 0.0, 0.0]))
            acceleration = data.get("acceleration", data.get("acceleration_world", [0.0, 0.0, 0.0]))
            command_speeds.append(math.sqrt(sum(float(value) ** 2 for value in velocity)))
            command_accelerations.append(math.sqrt(sum(float(value) ** 2 for value in acceleration)))
    if not odom_angles:
        raise SystemExit("no odom records found")
    print(f"session: {args.session}")
    print(f"odom samples: {len(odom_angles)}")
    print("max_abs_roll_deg: %.2f" % max(abs(value[0]) for value in odom_angles))
    print("max_abs_pitch_deg: %.2f" % max(abs(value[1]) for value in odom_angles))
    print("max_abs_yaw_deg: %.2f" % max(abs(value[2]) for value in odom_angles))
    print("altitude_min_max_m: %.3f %.3f" % (min(altitudes), max(altitudes)))
    if command_speeds:
        print("command_speed_max_mps: %.3f" % max(command_speeds))
        print("command_acceleration_max_mps2: %.3f" % max(command_accelerations))


if __name__ == "__main__":
    main()
