#!/usr/bin/env python3
"""Offline Route-YOPO command/dynamics replay without ROS or AirSim."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def norm(v):
    return math.sqrt(sum(float(x) * float(x) for x in v))


def vec(value, default=(0.0, 0.0, 0.0)):
    value = value if value is not None else default
    return [float(value[i]) for i in range(3)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--position-gain", type=float, default=4.0)
    parser.add_argument("--velocity-gain", type=float, default=3.0)
    parser.add_argument("--gravity", type=float, default=9.81)
    parser.add_argument("--maximum-acceleration", type=float, default=12.0)
    args = parser.parse_args()

    records = []
    initial_position = [0.0, 0.0, 1.6]
    for line in (args.session / "index.jsonl").open(encoding="utf-8"):
        record = json.loads(line)
        if record.get("kind") == "odom" and not records:
            initial_position = vec(record.get("data", {}).get("position"), initial_position)
        if record.get("kind") == "control":
            data = record.get("data", {})
            position = data.get("position_world", data.get("position"))
            velocity = data.get("velocity_world", data.get("velocity"))
            acceleration = data.get("acceleration_world", data.get("acceleration"))
            if position is None or velocity is None:
                continue
            records.append((int(record.get("stamp_ns", 0)), vec(position), vec(velocity), vec(acceleration)))
    if not records:
        raise SystemExit("no control records found")

    state_position = initial_position[:]
    state_velocity = [0.0, 0.0, 0.0]
    max_tilt = 0.0
    max_speed = 0.0
    min_z = state_position[2]
    max_z = state_position[2]
    for index, (stamp, target_position, target_velocity, feedforward) in enumerate(records):
        if index == 0:
            dt = 0.01
        else:
            dt = max(0.001, min(0.1, (stamp - records[index - 1][0]) * 1.0e-9))
        command = [
            feedforward[axis]
            + args.position_gain * (target_position[axis] - state_position[axis])
            + args.velocity_gain * (target_velocity[axis] - state_velocity[axis])
            for axis in range(3)
        ]
        command_norm = norm(command)
        if command_norm > args.maximum_acceleration:
            command = [value * args.maximum_acceleration / command_norm for value in command]
        tilt = math.degrees(math.atan2(norm(command[:2]), max(0.1, args.gravity + command[2])))
        max_tilt = max(max_tilt, tilt)
        state_velocity = [state_velocity[axis] + command[axis] * dt for axis in range(3)]
        state_position = [state_position[axis] + state_velocity[axis] * dt for axis in range(3)]
        speed = norm(state_velocity)
        max_speed = max(max_speed, speed)
        min_z = min(min_z, state_position[2])
        max_z = max(max_z, state_position[2])

    print(f"session: {args.session}")
    print(f"control_samples: {len(records)}")
    print(f"predicted_max_tilt_deg: {max_tilt:.2f}")
    print(f"predicted_speed_max_mps: {max_speed:.2f}")
    print(f"predicted_altitude_min_max_m: {min_z:.3f} {max_z:.3f}")
    print(f"last_position_m: {' '.join(f'{value:.3f}' for value in state_position)}")


if __name__ == "__main__":
    main()
