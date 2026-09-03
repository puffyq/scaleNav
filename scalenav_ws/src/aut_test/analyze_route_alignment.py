#!/usr/bin/env python3
"""Align ScaleNav route decisions with vehicle and published-path geometry."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def point_segment_distance(point, start, end):
    delta = [end[i] - start[i] for i in range(3)]
    offset = [point[i] - start[i] for i in range(3)]
    length_sq = sum(value * value for value in delta)
    if length_sq <= 1e-12:
        return math.dist(point, start)
    ratio = max(0.0, min(1.0, sum(offset[i] * delta[i] for i in range(3)) / length_sq))
    projection = [start[i] + ratio * delta[i] for i in range(3)]
    return math.dist(point, projection)


def path_distance(point, poses):
    if not poses:
        return math.nan
    if len(poses) == 1:
        return math.dist(point, poses[0])
    return min(
        point_segment_distance(point, poses[index - 1], poses[index])
        for index in range(1, len(poses))
    )


def load_path(session, event):
    relative = event.get("file")
    if not relative:
        return []
    with (session / relative).open(encoding="utf-8") as stream:
        return json.load(stream).get("poses", [])


def summarize(session):
    latest = {}
    with (session / "index.jsonl").open(encoding="utf-8") as stream:
        events = [json.loads(line) for line in stream]
    print(
        "event,seq,x,y,path_front_m,path_nearest_m,local_goal_m,vehicle_clearance_m,"
        "reason,blocked,repair_attempted,repair_found,reselection,route_lateral_m,"
        "incumbent_goal_m,candidate_goal_m,"
        "incumbent_risk,candidate_risk,incumbent_loss,candidate_loss"
    )
    for event in events:
        kind = event["kind"]
        if kind in {"odom", "path", "local_goal", "clearance"}:
            latest[kind] = event
        is_switch = kind == "timing" and event["data"].get("module") == "planner" and (
            event["data"].get("searched") or event["data"].get("candidate_accepted")
        )
        if not is_switch and kind != "collision":
            continue
        odom = latest.get("odom", {}).get("data", {})
        position = odom.get("position")
        poses = load_path(session, latest.get("path", {})) if latest.get("path") else []
        local_goal = latest.get("local_goal", {}).get("data", {}).get("position")
        clearance = latest.get("clearance", {}).get("data", {}).get("vehicle_m", math.nan)
        if position:
            front_distance = math.dist(position, poses[0]) if poses else math.nan
            nearest_distance = path_distance(position, poses)
            goal_distance = math.dist(position, local_goal) if local_goal else math.nan
            x, y = position[:2]
        else:
            x = y = front_distance = nearest_distance = goal_distance = math.nan
        data = event["data"]
        reason = data.get("switch_reason", "") if kind == "timing" else (
            f"active={data.get('active')}"
        )
        def value(name):
            item = data.get(name, math.nan) if kind == "timing" else math.nan
            return item if isinstance(item, (int, float)) else math.nan

        print(
            f"{kind},{event['seq']},{x:.3f},{y:.3f},{front_distance:.3f},"
            f"{nearest_distance:.3f},{goal_distance:.3f},{clearance:.3f},{reason},"
            f"{value('accepted_route_blocked'):.0f},"
            f"{value('route_repair_attempted'):.0f},{value('route_repair_found'):.0f},"
            f"{value('frontier_reselection_attempted'):.0f},"
            f"{value('route_lateral_error'):.3f},"
            f"{value('incumbent_goal_distance'):.3f},{value('candidate_goal_distance'):.3f},"
            f"{value('incumbent_risk'):.3f},{value('candidate_risk'):.3f},"
            f"{value('incumbent_loss'):.3f},{value('candidate_loss'):.3f}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    summarize(args.session.resolve())


if __name__ == "__main__":
    main()
