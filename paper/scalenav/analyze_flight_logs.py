#!/usr/bin/env python3
"""Compute reproducible closed-loop navigation metrics from ScaleNav logs.

The primary metrics follow the evaluation style used by EGO-Planner and SUPER:
goal completion, flight time, flown distance, average speed, path efficiency,
kinematics, and planning latency.  ScaleNav log v2 supplies simulator collision
and per-cycle planner timing; older logs can use --metadata-csv and ROS text logs.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence


NS_PER_S = 1_000_000_000
UPDATE_RE = re.compile(r"\[EPIC timing\]\[update\].*?astar=([0-9.]+) ms.*?publish=([0-9.]+) ms.*?total=([0-9.]+) ms")
BACKGROUND_RE = re.compile(r"\[EPIC timing\]\[background [^]]+\].*?total=([0-9.]+) ms")
CLOUD_RE = re.compile(r"\[EPIC timing\]\[cloud\].*?total=([0-9.]+) ms")
ROUTE_SWITCH_RE = re.compile(r"\[EPIC route switch\] reason=([A-Z_]+)")
ROS_TIME_RE = re.compile(r"^\[[A-Z]+\] \[([0-9]+(?:\.[0-9]+)?)\]")


@dataclass(frozen=True)
class Odom:
    seq: int
    stamp_ns: int
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]


@dataclass(frozen=True)
class TimedRecord:
    seq: int
    stamp_ns: int
    data: dict[str, Any]


@dataclass(frozen=True)
class GoalEvent:
    seq: int
    position: tuple[float, float, float]


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def distance(left: Sequence[float], right: Sequence[float]) -> float:
    return norm(tuple(a - b for a, b in zip(left, right)))


def percentile(values: Sequence[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def describe(values: Iterable[float]) -> dict[str, float | int | None]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"count": 0, "mean": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "p50": percentile(finite, 0.50),
        "p95": percentile(finite, 0.95),
        "p99": percentile(finite, 0.99),
        "max": max(finite),
    }


def t_critical_975(sample_count: int) -> float:
    """Two-sided 95% Student-t critical value without a scipy dependency."""
    table = {
        1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
        6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
        11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131,
        16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
        21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060,
        26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
    }
    degrees_of_freedom = sample_count - 1
    if degrees_of_freedom <= 30:
        return table[max(1, degrees_of_freedom)]
    if degrees_of_freedom <= 60:
        return 2.000
    if degrees_of_freedom <= 120:
        return 1.980
    return 1.960


def parse_bool(value: str | None) -> bool | None:
    if value is None or not value.strip():
        return None
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "y"}:
        return True
    if normalized in {"0", "false", "no", "n"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def load_metadata(path: Path | None) -> dict[str, dict[str, Any]]:
    if path is None:
        return {}
    result: dict[str, dict[str, Any]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            key = (row.get("session") or "").strip()
            if not key:
                raise ValueError("metadata CSV requires a non-empty 'session' column")
            parsed: dict[str, Any] = dict(row)
            for field in ("collision", "kinematic_violation", "timeout"):
                parsed[field] = parse_bool(row.get(field))
            result[key] = parsed
    return result


def in_intervals(stamp_ns: int, intervals: Sequence[tuple[int, int]]) -> bool:
    return any(start <= stamp_ns <= end for start, end in intervals)


def integrate_path(samples: Sequence[Odom], start: int, end: int) -> float:
    return sum(
        distance(samples[index - 1].position, samples[index].position)
        for index in range(start + 1, end + 1)
    )


def dynamics(samples: Sequence[Odom], ranges: Sequence[tuple[int, int]]) -> dict[str, float | None]:
    """Replicate EPIC's alpha=0.2 filtered odometry acceleration/jerk estimate."""
    speed_integral = 0.0
    duration = 0.0
    jerk_sq_integral = 0.0
    jerk_duration = 0.0
    max_speed = 0.0
    max_acceleration = 0.0
    max_jerk = 0.0
    for start, end in ranges:
        filtered_acceleration = (0.0, 0.0, 0.0)
        have_acceleration = False
        for index in range(start + 1, end + 1):
            previous, current = samples[index - 1], samples[index]
            dt = (current.stamp_ns - previous.stamp_ns) / NS_PER_S
            if not 1e-3 < dt < 2.0:
                continue
            speed = norm(current.velocity)
            max_speed = max(max_speed, speed)
            speed_integral += speed * dt
            duration += dt
            raw = tuple((current.velocity[axis] - previous.velocity[axis]) / dt for axis in range(3))
            acceleration = tuple(0.2 * raw[axis] + 0.8 * filtered_acceleration[axis] for axis in range(3))
            max_acceleration = max(max_acceleration, norm(acceleration))
            if have_acceleration:
                jerk = tuple((acceleration[axis] - filtered_acceleration[axis]) / dt for axis in range(3))
                jerk_norm = norm(jerk)
                max_jerk = max(max_jerk, jerk_norm)
                jerk_sq_integral += jerk_norm * jerk_norm * dt
                jerk_duration += dt
            filtered_acceleration = acceleration
            have_acceleration = True
    return {
        "time_weighted_speed_mps": speed_integral / duration if duration else None,
        "max_speed_mps": max_speed if duration else None,
        "estimated_max_acceleration_mps2": max_acceleration if duration else None,
        "estimated_jerk_rms_mps3": math.sqrt(jerk_sq_integral / jerk_duration) if jerk_duration else None,
        "estimated_max_jerk_mps3": max_jerk if jerk_duration else None,
    }


def publish_stats(records: Sequence[TimedRecord], intervals: Sequence[tuple[int, int]], active_s: float) -> dict[str, Any]:
    stamps = sorted(record.stamp_ns for record in records if record.stamp_ns > 0 and in_intervals(record.stamp_ns, intervals))
    gaps = [(right - left) / NS_PER_S for left, right in zip(stamps, stamps[1:]) if right >= left]
    return {
        "count": len(stamps),
        "rate_hz": len(stamps) / active_s if active_s > 0 else None,
        "gap_p99_s": percentile(gaps, 0.99),
        "gap_max_s": max(gaps) if gaps else None,
    }


def find_ros_log(manifest: dict[str, Any], explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit
    target_ms = int(manifest.get("created_unix_ns", 0)) / 1_000_000
    candidates: list[tuple[float, Path]] = []
    for path in (Path.home() / ".ros" / "log").glob("epic_graph_node_*.log"):
        match = re.search(r"_([0-9]+)\.log$", path.name)
        if match:
            candidates.append((abs(int(match.group(1)) - target_ms), path))
    if not candidates:
        return None
    delta_ms, path = min(candidates)
    return path if delta_ms <= 120_000 else None


def parse_ros_metrics(path: Path | None, intervals: Sequence[tuple[int, int]]) -> dict[str, Any]:
    if path is None or not path.is_file():
        return {"source": None, "planner_update_ms": {}, "graph_update_ms": {}, "cloud_update_ms": {}, "route_switches": None}
    update: dict[str, list[float]] = defaultdict(list)
    background: list[float] = []
    cloud: list[float] = []
    switches: Counter[str] = Counter()
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            time_match = ROS_TIME_RE.match(line)
            if not time_match or not in_intervals(int(float(time_match.group(1)) * NS_PER_S), intervals):
                continue
            if match := UPDATE_RE.search(line):
                for key, value in zip(("astar", "publish", "total"), match.groups()):
                    update[key].append(float(value))
            if match := BACKGROUND_RE.search(line):
                background.append(float(match.group(1)))
            if match := CLOUD_RE.search(line):
                cloud.append(float(match.group(1)))
            if match := ROUTE_SWITCH_RE.search(line):
                switches[match.group(1)] += 1
    return {
        "source": str(path),
        "planner_update_ms": {key: describe(values) for key, values in update.items()},
        "graph_update_ms": describe(background),
        "cloud_update_ms": describe(cloud),
        "route_switches": {"count": sum(switches.values()), "by_reason": dict(sorted(switches.items()))},
    }


def parse_structured_runtime(
    records: Sequence[TimedRecord], intervals: Sequence[tuple[int, int]]
) -> dict[str, Any] | None:
    selected = [record for record in records if in_intervals(record.stamp_ns, intervals)]
    if not selected:
        return None
    planner: dict[str, list[float]] = defaultdict(list)
    background: list[float] = []
    cloud: list[float] = []
    switches: Counter[str] = Counter()
    for record in selected:
        module = str(record.data.get("module", ""))
        if module == "planner":
            for key in ("odom", "astar", "publish", "total"):
                value = record.data.get(f"{key}_ms")
                if isinstance(value, (int, float)):
                    planner[key].append(float(value))
            reason = str(record.data.get("switch_reason", "NONE"))
            if record.data.get("candidate_accepted") and record.data.get("found") and reason != "NONE":
                switches[reason] += 1
        elif module == "background":
            value = record.data.get("total_ms")
            if isinstance(value, (int, float)):
                background.append(float(value))
        elif module == "cloud":
            value = record.data.get("total_ms")
            if isinstance(value, (int, float)):
                cloud.append(float(value))
    return {
        "source": "scalenav_log.v2:/epic/timing",
        "planner_update_ms": {key: describe(values) for key, values in planner.items()},
        "graph_update_ms": describe(background),
        "cloud_update_ms": describe(cloud),
        "route_switches": {"count": sum(switches.values()), "by_reason": dict(sorted(switches.items()))},
    }


def select_metadata(metadata: dict[str, dict[str, Any]], session: Path) -> dict[str, Any]:
    for key in (session.name, str(session), str(session.resolve())):
        if key in metadata:
            return metadata[key]
    return {}


def analyze_session(
    session: Path,
    args: argparse.Namespace,
    metadata: dict[str, dict[str, Any]],
    explicit_ros_log: Path | None,
) -> dict[str, Any]:
    manifest_path = session / "manifest.json"
    index_path = session / "index.jsonl"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    schema = manifest.get("schema")
    if schema not in {"scalenav_log.v1", "scalenav_log.v2"}:
        raise ValueError(f"{session}: unsupported schema {manifest.get('schema')!r}")

    odom: list[Odom] = []
    goals: list[GoalEvent] = []
    timed: dict[str, list[TimedRecord]] = defaultdict(list)
    counts: Counter[str] = Counter()
    last_goal: tuple[float, float, float] | None = None
    invalid_json_lines = 0
    with index_path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                invalid_json_lines += 1
                continue
            kind = str(record.get("kind", ""))
            counts[kind] += 1
            seq = int(record.get("seq", 0))
            stamp_ns = int(record.get("stamp_ns", 0))
            data = record.get("data") or {}
            if kind == "odom":
                position = tuple(float(value) for value in data["position"])
                velocity = tuple(float(value) for value in data["velocity"])
                odom.append(Odom(seq, stamp_ns, position, velocity))
            elif kind == "goal":
                position = tuple(float(value) for value in data["position"])
                if last_goal is None or distance(position, last_goal) > args.goal_change_epsilon:
                    goals.append(GoalEvent(seq, position))
                    last_goal = position
            if stamp_ns > 0:
                timed[kind].append(TimedRecord(seq, stamp_ns, data))

    if len(odom) < 2:
        raise ValueError(f"{session}: fewer than two odometry samples")
    odom.sort(key=lambda sample: sample.seq)
    seqs = [sample.seq for sample in odom]
    timestamp_regressions = sum(right.stamp_ns <= left.stamp_ns for left, right in zip(odom, odom[1:]))
    odom_gaps = [
        (right.stamp_ns - left.stamp_ns) / NS_PER_S
        for left, right in zip(odom, odom[1:])
        if right.stamp_ns > left.stamp_ns
    ]

    mapped_goals: list[tuple[GoalEvent, int]] = []
    for goal in goals:
        index = min(bisect.bisect_left(seqs, goal.seq), len(odom) - 1)
        mapped_goals.append((goal, index))

    latched_collisions = sorted(
        (
            record for record in timed["collision"]
            if bool(record.data.get("latched", record.data.get("active", False)))
        ),
        key=lambda record: record.seq,
    )

    legs: list[dict[str, Any]] = []
    ranges: list[tuple[int, int]] = []
    for goal_index, (goal, start_index) in enumerate(mapped_goals):
        initial_distance = distance(odom[start_index].position, goal.position)
        if initial_distance <= args.position_tolerance:
            continue
        search_end = (
            max(start_index, mapped_goals[goal_index + 1][1] - 1)
            if goal_index + 1 < len(mapped_goals)
            else len(odom) - 1
        )
        terminal_collision = next(
            (
                record for record in latched_collisions
                if odom[start_index].seq <= record.seq <= odom[search_end].seq
            ),
            None,
        )
        if terminal_collision is not None:
            search_end = min(
                search_end,
                max(start_index + 1, bisect.bisect_left(seqs, terminal_collision.seq)),
            )
        spatial_index: int | None = None
        settled_index: int | None = None
        for index in range(start_index, search_end + 1):
            position_error = distance(odom[index].position, goal.position)
            if spatial_index is None and position_error <= args.position_tolerance:
                spatial_index = index
            if position_error <= args.position_tolerance and norm(odom[index].velocity) <= args.speed_tolerance:
                settled_index = index
                break
        end_index = settled_index if settled_index is not None else search_end
        if end_index <= start_index:
            continue
        path_m = integrate_path(odom, start_index, end_index)
        duration_s = (odom[end_index].stamp_ns - odom[start_index].stamp_ns) / NS_PER_S
        final_error = distance(odom[end_index].position, goal.position)
        progress_m = min(initial_distance, max(0.0, initial_distance - final_error))
        legs.append({
            "leg": len(legs) + 1,
            "goal": list(goal.position),
            "goal_seq": goal.seq,
            "start_stamp_ns": odom[start_index].stamp_ns,
            "end_stamp_ns": odom[end_index].stamp_ns,
            "reached_position": spatial_index is not None,
            "reached_and_stopped": settled_index is not None,
            "straight_line_m": initial_distance,
            "net_progress_m": progress_m,
            "path_m": path_m,
            "duration_s": duration_s,
            "average_speed_mps": path_m / duration_s if duration_s > 0 else None,
            "path_efficiency_pct": 100.0 * progress_m / path_m if path_m > 0 else None,
            "final_goal_error_m": final_error,
            "terminal_reason": "collision" if terminal_collision is not None else (
                "goal" if settled_index is not None else "log_end"
            ),
        })
        ranges.append((start_index, end_index))
        if args.mission_mode == "first":
            break

    if not legs:
        raise ValueError(f"{session}: no non-trivial goal leg found")
    intervals = [(odom[start].stamp_ns, odom[end].stamp_ns) for start, end in ranges]
    total_path = sum(leg["path_m"] for leg in legs)
    total_duration = sum(leg["duration_s"] for leg in legs)
    straight_line = sum(leg["straight_line_m"] for leg in legs)
    progress = sum(leg["net_progress_m"] for leg in legs)
    goal_complete = all(leg["reached_and_stopped"] for leg in legs)
    trial_meta = select_metadata(metadata, session)
    collision = trial_meta.get("collision")
    collision_source = "metadata" if collision is not None else None
    if collision is None and schema == "scalenav_log.v2":
        collision_history = [
            record for record in timed["collision"]
            if record.stamp_ns <= intervals[-1][1]
        ]
        if collision_history:
            collision = any(bool(record.data.get("latched")) for record in collision_history)
            collision_source = "scalenav_log.v2:/sim/collision"
    kinematic_violation = trial_meta.get("kinematic_violation")
    timeout = trial_meta.get("timeout")
    success = None
    safe = None
    if collision is not None and kinematic_violation is not None:
        safe = not collision and not kinematic_violation
    if safe is not None and timeout is not None:
        success = goal_complete and safe and not timeout

    clearance_records = [record for record in timed["clearance"] if in_intervals(record.stamp_ns, intervals)]
    vehicle_clearance = [
        number
        for record in clearance_records
        if (number := finite_number(record.data.get("vehicle_m"))) is not None
    ]
    global_witness_clearance = [
        number
        for record in clearance_records
        if (
            number := finite_number(
                record.data.get("global_witness_min_m", record.data.get("path_min_m"))
            )
        ) is not None
    ]

    structured_runtime = parse_structured_runtime(timed["timing"], intervals)
    ros_log = None if structured_runtime is not None else find_ros_log(manifest, explicit_ros_log)
    runtime_metrics = structured_runtime or parse_ros_metrics(ros_log, intervals)
    warnings: list[str] = []
    if collision is None:
        warnings.append("collision is unknown; provide --metadata-csv, and do not infer it from clearance")
    if kinematic_violation is None:
        warnings.append("kinematic validity is unknown; derived odometry dynamics are diagnostic only")
    if structured_runtime is None and ros_log is None:
        warnings.append("EPIC ROS log was not found; planning latency and route-switch count are unavailable")
    elif structured_runtime is None:
        timing_count = int(nested(runtime_metrics, "planner_update_ms.total.count") or 0)
        path_count = counts["path"]
        if timing_count and path_count > timing_count * 2:
            warnings.append(
                f"planner timing is throttled ({timing_count}/{path_count} cycles logged); "
                "reported latency percentiles apply only to logged samples"
            )
    if invalid_json_lines:
        warnings.append(f"ignored {invalid_json_lines} malformed JSONL lines")
    if timestamp_regressions:
        warnings.append(f"found {timestamp_regressions} non-increasing odometry timestamps")
    if odom_gaps and max(odom_gaps) > 0.1:
        warnings.append(f"maximum odometry gap is {max(odom_gaps):.3f} s")

    return {
        "session": session.name,
        "session_path": str(session.resolve()),
        "labels": {key: trial_meta.get(key) for key in ("method", "scenario", "seed")},
        "outcome": {
            "goal_sequence_complete": goal_complete,
            "safe": safe,
            "success": success,
            "collision": collision,
            "collision_source": collision_source,
            "kinematic_violation": kinematic_violation,
            "timeout": timeout,
        },
        "mission": {
            "mode": args.mission_mode,
            "leg_count": len(legs),
            "active_duration_s": total_duration,
            "wall_duration_s": (intervals[-1][1] - intervals[0][0]) / NS_PER_S,
            "straight_line_m": straight_line,
            "net_progress_m": progress,
            "path_m": total_path,
            "average_speed_mps": total_path / total_duration if total_duration > 0 else None,
            "path_efficiency_pct": 100.0 * min(1.0, progress / total_path) if total_path > 0 else None,
            "goal_spl_pct": 100.0 * min(1.0, straight_line / total_path) if goal_complete and total_path > 0 else 0.0,
            "spl_pct": (100.0 * min(1.0, straight_line / total_path) if success and total_path > 0 else 0.0) if success is not None else None,
        },
        "legs": legs,
        "dynamics": dynamics(odom, ranges),
        "clearance": {
            "sample_count": len(vehicle_clearance),
            "vehicle_min_m": min(vehicle_clearance) if vehicle_clearance else None,
            "vehicle_p05_m": percentile(vehicle_clearance, 0.05),
            "vehicle_mean_m": statistics.fmean(vehicle_clearance) if vehicle_clearance else None,
            # Diagnostic only: YOPO executes and locally avoids obstacles
            # around this global witness; this is not executed-flight clearance.
            "global_witness_min_clearance_m": (
                min(global_witness_clearance) if global_witness_clearance else None
            ),
        },
        "rates": {
            kind: publish_stats(timed[kind], intervals, total_duration)
            for kind in ("local_goal", "path", "graph", "semantic", "rgb", "depth")
        },
        "runtime": runtime_metrics,
        "data_quality": {
            "record_counts": dict(sorted(counts.items())),
            "odom_timestamp_regressions": timestamp_regressions,
            "odom_gap_p99_s": percentile(odom_gaps, 0.99),
            "odom_gap_max_s": max(odom_gaps) if odom_gaps else None,
            "zero_stamp_goal_count": counts["goal"] - len(timed["goal"]),
            "zero_stamp_control_count": counts["control"] - len(timed["control"]),
        },
        "warnings": warnings,
    }


def nested(result: dict[str, Any], path: str) -> Any:
    value: Any = result
    for component in path.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(component)
    return value


AGGREGATE_FIELDS = {
    "completion_time_s": "mission.active_duration_s",
    "path_m": "mission.path_m",
    "average_speed_mps": "mission.average_speed_mps",
    "path_efficiency_pct": "mission.path_efficiency_pct",
    "goal_spl_pct": "mission.goal_spl_pct",
    "spl_pct": "mission.spl_pct",
    "minimum_clearance_m": "clearance.vehicle_min_m",
    "max_speed_mps": "dynamics.max_speed_mps",
    "planner_total_mean_ms": "runtime.planner_update_ms.total.mean",
    "planner_total_p99_ms": "runtime.planner_update_ms.total.p99",
}


def aggregate(results: Sequence[dict[str, Any]]) -> dict[str, Any]:
    metrics: dict[str, Any] = {}
    for label, path in AGGREGATE_FIELDS.items():
        values = [float(value) for result in results if (value := nested(result, path)) is not None]
        if not values:
            metrics[label] = {"n": 0, "mean": None, "std": None, "ci95": None}
            continue
        std = statistics.stdev(values) if len(values) > 1 else None
        ci = t_critical_975(len(values)) * std / math.sqrt(len(values)) if std is not None else None
        metrics[label] = {"n": len(values), "mean": statistics.fmean(values), "std": std, "ci95": ci}
    goal_successes = sum(bool(nested(result, "outcome.goal_sequence_complete")) for result in results)
    labeled_success = [nested(result, "outcome.success") for result in results]
    labeled_success = [value for value in labeled_success if value is not None]
    return {
        "trial_count": len(results),
        "goal_completion_rate_pct": 100.0 * goal_successes / len(results),
        "success_rate_pct": 100.0 * sum(bool(value) for value in labeled_success) / len(labeled_success) if labeled_success else None,
        "success_label_count": len(labeled_success),
        "metrics": metrics,
    }


def aggregate_by_method(results: Sequence[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for result in results:
        groups[str(nested(result, "labels.method") or "unlabeled")].append(result)
    return {method: aggregate(group) for method, group in sorted(groups.items())}


def fmt(value: Any, digits: int = 2) -> str:
    if value is None:
        return "--"
    if isinstance(value, bool):
        return "yes" if value else "no"
    return f"{float(value):.{digits}f}"


def markdown(results: Sequence[dict[str, Any]], summary: dict[str, Any]) -> str:
    lines = [
        "# Closed-loop flight metrics",
        "",
        "| Session | Goal complete | Success | Time (s) | Path (m) | Avg. speed (m/s) | Geom. eff. (%) | SPL (%) | Min clear. (m) | Logged plan P99 (ms) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        lines.append(
            "| {session} | {goal} | {success} | {time} | {path} | {speed} | {eff} | {spl} | {clearance} | {plan} |".format(
                session=result["session"],
                goal=fmt(nested(result, "outcome.goal_sequence_complete")),
                success=fmt(nested(result, "outcome.success")),
                time=fmt(nested(result, "mission.active_duration_s")),
                path=fmt(nested(result, "mission.path_m")),
                speed=fmt(nested(result, "mission.average_speed_mps")),
                eff=fmt(nested(result, "mission.path_efficiency_pct")),
                spl=fmt(nested(result, "mission.spl_pct")),
                clearance=fmt(nested(result, "clearance.vehicle_min_m")),
                plan=fmt(nested(result, "runtime.planner_update_ms.total.p99")),
            )
        )
    lines.extend(["", "## Aggregate", "", f"Trials: {summary['trial_count']}; goal completion: {fmt(summary['goal_completion_rate_pct'])}%."])
    if summary["success_rate_pct"] is None:
        lines.append("Success rate is unavailable until collision, kinematic-validity, and timeout labels are supplied.")
    else:
        lines.append(f"Labeled success rate: {fmt(summary['success_rate_pct'])}% ({summary['success_label_count']} trials).")
    lines.extend(["", "| Metric | n | Mean | Std. | 95% CI half-width |", "|---|---:|---:|---:|---:|"])
    for label, stats in summary["metrics"].items():
        lines.append(f"| {label} | {stats['n']} | {fmt(stats['mean'], 3)} | {fmt(stats['std'], 3)} | {fmt(stats['ci95'], 3)} |")
    warnings = [(result["session"], warning) for result in results for warning in result["warnings"]]
    if warnings:
        lines.extend(["", "## Audit warnings", ""])
        lines.extend(f"- `{session}`: {warning}" for session, warning in warnings)
    return "\n".join(lines) + "\n"


def flat_row(result: dict[str, Any]) -> dict[str, Any]:
    return {
        "session": result["session"],
        "method": nested(result, "labels.method"),
        "scenario": nested(result, "labels.scenario"),
        "seed": nested(result, "labels.seed"),
        "goal_complete": nested(result, "outcome.goal_sequence_complete"),
        "success": nested(result, "outcome.success"),
        "collision": nested(result, "outcome.collision"),
        "completion_time_s": nested(result, "mission.active_duration_s"),
        "path_m": nested(result, "mission.path_m"),
        "average_speed_mps": nested(result, "mission.average_speed_mps"),
        "path_efficiency_pct": nested(result, "mission.path_efficiency_pct"),
        "goal_spl_pct": nested(result, "mission.goal_spl_pct"),
        "minimum_clearance_m": nested(result, "clearance.vehicle_min_m"),
        "max_speed_mps": nested(result, "dynamics.max_speed_mps"),
        "estimated_max_acceleration_mps2": nested(result, "dynamics.estimated_max_acceleration_mps2"),
        "estimated_jerk_rms_mps3": nested(result, "dynamics.estimated_jerk_rms_mps3"),
        "route_switches": nested(result, "runtime.route_switches.count"),
        "planner_total_mean_ms": nested(result, "runtime.planner_update_ms.total.mean"),
        "planner_total_p99_ms": nested(result, "runtime.planner_update_ms.total.p99"),
    }


def write_outputs(
    output_dir: Path,
    results: Sequence[dict[str, Any]],
    summary: dict[str, Any],
    by_method: dict[str, Any],
    report: str,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "flight_metrics.json").write_text(
        json.dumps(
            {"schema": "scalenav_flight_metrics.v1", "trials": results, "aggregate": summary, "by_method": by_method},
            indent=2,
        ) + "\n",
        encoding="utf-8",
    )
    rows = [flat_row(result) for result in results]
    with (output_dir / "flight_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "flight_metrics.md").write_text(report, encoding="utf-8")


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", nargs="+", type=Path, help="ScaleNav session directories")
    parser.add_argument("--output-dir", type=Path, help="write JSON, CSV, and Markdown reports here")
    parser.add_argument("--metadata-csv", type=Path, help="optional labels: session,method,scenario,seed,collision,kinematic_violation,timeout")
    parser.add_argument("--ros-log", type=Path, help="EPIC ROS log; valid only with one session (otherwise auto-detected)")
    parser.add_argument("--position-tolerance", type=float, default=0.5, help="goal radius in meters (default: 0.5)")
    parser.add_argument("--speed-tolerance", type=float, default=0.3, help="arrival speed in m/s (default: 0.3)")
    parser.add_argument("--goal-change-epsilon", type=float, default=0.05, help="deduplicate repeated goal messages (default: 0.05 m)")
    parser.add_argument(
        "--mission-mode", choices=("first", "all"), default="first",
        help="analyze the first non-trivial one-way mission (default) or all goal legs",
    )
    args = parser.parse_args(argv)
    if args.ros_log is not None and len(args.sessions) != 1:
        parser.error("--ros-log can only be used with one session")
    if args.position_tolerance <= 0 or args.speed_tolerance < 0 or args.goal_change_epsilon <= 0:
        parser.error("tolerances must be positive (speed tolerance may be zero)")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        metadata = load_metadata(args.metadata_csv)
        results = [analyze_session(session, args, metadata, args.ros_log) for session in args.sessions]
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    summary = aggregate(results)
    by_method = aggregate_by_method(results)
    report = markdown(results, summary)
    if args.output_dir:
        write_outputs(args.output_dir, results, summary, by_method, report)
    print(report, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
