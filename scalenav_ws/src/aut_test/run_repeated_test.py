#!/usr/bin/env python3
"""Run isolated, repeated ScaleNav missions against AirSim."""

from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
WS = SCRIPT_DIR.parent.parent
PROJECT_ROOT = WS.parent
START_SCRIPT = WS / "scripts" / "start.sh"
ROUTE_YOPO_SCRIPT = WS / "scripts" / "start_route_yopo.sh"
BASELINE_ROOT = PROJECT_ROOT / "bc" / "third_party" / "compare"
BASELINE_SCRIPTS = {
    "ego": BASELINE_ROOT / "run_ego_map2.sh",
    "super": BASELINE_ROOT / "run_super_map2.sh",
}
AIRSIM_PYTHON = (
    WS
    / "src"
    / "controller_airsim"
    / "src"
    / "airsim_renderer"
)

STOP_REQUESTED = False


def request_stop(signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True
    print(f"\nreceived signal {signum}; stopping after cleanup", flush=True)


def timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def finite_nonnegative(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("must be a finite non-negative number")
    return parsed


def finite_positive(value: str) -> float:
    parsed = finite_nonnegative(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Reset AirSim and ScaleNav, fly from (0,0,1.6) to "
            "(0,140,1.6), then repeat with an isolated log session."
        )
    )
    parser.add_argument(
        "--count",
        type=int,
        default=10,
        help="number of trials; 0 repeats until Ctrl-C (default: 10)",
    )
    parser.add_argument(
        "--stack",
        choices=("scalenav", "route_yopo", "ego", "super"),
        default="scalenav",
        help="planner stack to launch for each isolated trial (default: scalenav)",
    )
    parser.add_argument(
        "--timeout",
        type=finite_positive,
        default=90.0,
        help="mission timeout after publishing the goal in seconds (default: 90)",
    )
    parser.add_argument(
        "--startup-timeout",
        type=finite_positive,
        default=60.0,
        help="maximum stack and ROS readiness wait in seconds (default: 60)",
    )
    parser.add_argument(
        "--cooldown",
        type=finite_nonnegative,
        default=3.0,
        help="pause between trials in seconds (default: 3)",
    )
    parser.add_argument("--goal-x", type=float, default=0.0)
    parser.add_argument("--goal-y", type=float, default=140.0)
    parser.add_argument("--goal-z", type=float, default=1.6)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-z", type=float, default=1.6)
    parser.add_argument(
        "--start-tolerance",
        type=finite_positive,
        default=0.5,
        help="allowed error after /scalenav/reset_sim (default: 0.5 m)",
    )
    parser.add_argument(
        "--position-tolerance",
        type=finite_positive,
        default=0.5,
        help="goal position tolerance (default: 0.5 m)",
    )
    parser.add_argument(
        "--speed-tolerance",
        type=finite_nonnegative,
        default=0.3,
        help="goal settling speed tolerance (default: 0.3 m/s)",
    )
    parser.add_argument(
        "--airsim-host", default="127.0.0.1", help="AirSim RPC host"
    )
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument(
        "--airsim-timeout",
        type=finite_positive,
        default=5.0,
        help="AirSim RPC timeout in seconds (default: 5)",
    )
    parser.add_argument(
        "--reset-settle",
        type=finite_nonnegative,
        default=2.0,
        help="wait after AirSim reset before starting ScaleNav (default: 2)",
    )
    parser.add_argument(
        "--log-root",
        type=Path,
        default=PROJECT_ROOT / "log_scalenav",
        help="ScaleNav session log root",
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=SCRIPT_DIR / "results",
        help="automated-test result root",
    )
    parser.add_argument(
        "--no-semantic",
        action="store_true",
        help="pass --no-semantic to ScaleNav start.sh",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and print the test configuration without resetting or flying",
    )
    args = parser.parse_args()
    if args.count < 0:
        parser.error("--count must be zero or greater")
    for name in ("goal_x", "goal_y", "goal_z", "start_x", "start_y", "start_z"):
        if not math.isfinite(getattr(args, name)):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    if not 1 <= args.airsim_port <= 65535:
        parser.error("--airsim-port must be between 1 and 65535")
    return args


def acquire_lock() -> Any:
    lock_path = Path("/tmp/scalenav_aut_test.lock")
    lock_file = lock_path.open("w", encoding="ascii")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError(
            f"another automated ScaleNav test holds {lock_path}"
        ) from None
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file


def process_arguments(pid: int) -> list[str]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return []
    return [
        item.decode(errors="replace")
        for item in raw.split(b"\0")
        if item
    ]


def conflicting_processes() -> list[tuple[int, str]]:
    executable_names = {
        "uav_sim_node",
        "airsim_renderer_node",
        "depth_planar_to_pointcloud_node",
        "scalenav_log_node",
        "text_heatmap_ros2.py",
        "online_planner_ros2.py",
        "scalenav_graph_node",
    }
    own = {os.getpid(), os.getppid()}
    conflicts: list[tuple[int, str]] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in own:
            continue
        arguments = process_arguments(pid)
        if any(Path(argument).name in executable_names for argument in arguments):
            conflicts.append((pid, " ".join(arguments)))
    return sorted(conflicts)


def validate_environment(args: argparse.Namespace) -> None:
    if args.stack == "scalenav":
        launcher = START_SCRIPT
    elif args.stack == "route_yopo":
        launcher = ROUTE_YOPO_SCRIPT
    else:
        launcher = BASELINE_SCRIPTS[args.stack]
    if not launcher.is_file():
        raise RuntimeError(f"missing launcher: {launcher}")
    if not (WS / "install" / "setup.bash").is_file():
        raise RuntimeError(f"workspace is not built: {WS / 'install/setup.bash'}")
    if not AIRSIM_PYTHON.is_dir():
        raise RuntimeError(f"missing AirSim RPC client: {AIRSIM_PYTHON}")
    if "AMENT_PREFIX_PATH" not in os.environ:
        raise RuntimeError("ROS environment is not loaded; use run_0_140.sh")
    try:
        import msgpack  # noqa: F401
        import rclpy  # noqa: F401
    except ImportError as error:
        raise RuntimeError(
            f"ROS/AirSim Python dependency is unavailable in {sys.executable}: {error}"
        ) from error
    conflicts = conflicting_processes()
    if conflicts:
        details = "\n".join(f"  pid={pid} {cmd}" for pid, cmd in conflicts)
        raise RuntimeError(
            "another controller/ScaleNav stack is already running; stop it first:\n"
            + details
        )
    args.log_root.mkdir(parents=True, exist_ok=True)
    args.results_root.mkdir(parents=True, exist_ok=True)


def reset_airsim(args: argparse.Namespace) -> None:
    sys.path.insert(0, str(AIRSIM_PYTHON))
    try:
        from airsim_renderer.rpc import MessagePackRpcClient
    finally:
        sys.path.pop(0)
    client = MessagePackRpcClient(
        args.airsim_host, args.airsim_port, args.airsim_timeout
    )
    try:
        client.call("reset")
    finally:
        client.close()


def session_directories(log_root: Path) -> set[Path]:
    return {path.resolve() for path in log_root.glob("session_*") if path.is_dir()}


def newest_new_session(log_root: Path, previous: set[Path]) -> str:
    candidates = [
        path.resolve()
        for path in log_root.glob("session_*")
        if path.is_dir() and path.resolve() not in previous
    ]
    if not candidates:
        return ""
    return str(max(candidates, key=lambda path: path.stat().st_mtime_ns))


def start_stack(
    args: argparse.Namespace, console_path: Path
) -> tuple[subprocess.Popen[str], Any]:
    if args.stack in {"scalenav", "route_yopo"}:
        launcher = START_SCRIPT if args.stack == "scalenav" else ROUTE_YOPO_SCRIPT
        command = [str(launcher)]
        if args.no_semantic:
            command.append("--no-semantic")
    else:
        command = [
            "bash",
            str(BASELINE_SCRIPTS[args.stack]),
            f"output_dir:={args.log_root.resolve()}",
        ]
    environment = os.environ.copy()
    environment["SCALENAV_LOG_DIR"] = str(args.log_root.resolve())
    console = console_path.open("w", encoding="utf-8", buffering=1)
    process = subprocess.Popen(
        command,
        cwd=PROJECT_ROOT,
        env=environment,
        stdout=console,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return process, console


def stop_stack(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        print(f"warning: stack process {process.pid} did not exit", file=sys.stderr)


class MissionMonitor:
    def __init__(self, args: argparse.Namespace, stack: subprocess.Popen[str]):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.PoseStamped = PoseStamped
        self.Trigger = Trigger
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("scalenav_automated_test")
        self.stack = stack
        self.latest_odom: Any | None = None
        self.odom_received_at = 0.0
        self.collision: bool | None = None
        self.collision_received_at = 0.0
        self.goal_topic = (
            "/goal_pose"
            if args.stack in {"scalenav", "route_yopo"}
            else "/move_base_simple/goal"
        )
        # Every benchmark launch has one planner and one structured logger on
        # the goal topic. Waiting for both prevents the logger from accepting a
        # goal before the planner subscription has finished starting.
        self.minimum_goal_subscribers = 2
        self.goal_pub = self.node.create_publisher(PoseStamped, self.goal_topic, 10)
        self.reset_client = self.node.create_client(Trigger, "/scalenav/reset_sim")
        self.node.create_subscription(Odometry, "/sim/odom", self.on_odom, 1)
        self.node.create_subscription(Bool, "/sim/collision", self.on_collision, 1)

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def on_odom(self, message: Any) -> None:
        self.latest_odom = message
        self.odom_received_at = time.monotonic()

    def on_collision(self, message: Any) -> None:
        self.collision = bool(message.data)
        self.collision_received_at = time.monotonic()

    def spin_once(self, timeout: float = 0.1) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def stack_error(self) -> str | None:
        status = self.stack.poll()
        return None if status is None else f"ScaleNav stack exited with status {status}"

    def wait_until(self, predicate: Any, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not STOP_REQUESTED:
            stack_error = self.stack_error()
            if stack_error:
                raise RuntimeError(stack_error)
            self.spin_once(min(0.1, max(0.0, deadline - time.monotonic())))
            if predicate():
                return
        if STOP_REQUESTED:
            raise InterruptedError("test interrupted")
        raise TimeoutError(f"timed out waiting for {description}")

    def call_sim_reset(self, timeout: float) -> None:
        request = self.Trigger.Request()
        future = self.reset_client.call_async(request)
        self.wait_until(lambda: future.done(), timeout, "reset response")
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"/scalenav/reset_sim failed: {message}")

    def prepare(self, args: argparse.Namespace) -> None:
        self.wait_until(
            lambda: self.reset_client.service_is_ready(),
            args.startup_timeout,
            "/scalenav/reset_sim service",
        )
        self.wait_until(
            lambda: self.latest_odom is not None and self.collision is False,
            args.startup_timeout,
            "fresh /sim/odom and collision=false",
        )
        self.wait_until(
            lambda: self.goal_pub.get_subscription_count()
            >= self.minimum_goal_subscribers,
            args.startup_timeout,
            f"{self.minimum_goal_subscribers} {self.goal_topic} subscriber(s)",
        )
        self.call_sim_reset(args.startup_timeout)

        reset_time = time.monotonic()
        self.wait_until(
            lambda: self.odom_received_at >= reset_time,
            args.startup_timeout,
            "post-reset odometry",
        )
        position, _ = self.odom_state()
        start_error = math.dist(
            position, (args.start_x, args.start_y, args.start_z)
        )
        if start_error > args.start_tolerance:
            raise RuntimeError(
                "simulator reset position is "
                f"{position}, {start_error:.3f} m from expected start "
                f"({args.start_x}, {args.start_y}, {args.start_z})"
            )
        self.wait_until(
            lambda: self.collision is False,
            args.startup_timeout,
            "collision=false after simulator reset",
        )
        # Clear any emergency stop latched during startup immediately before goal.
        self.call_sim_reset(args.startup_timeout)
        self.wait_for_stable_start(args)

    def odom_state(self) -> tuple[tuple[float, float, float], float]:
        message = self.latest_odom
        if message is None:
            return ((math.nan, math.nan, math.nan), math.nan)
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return ((position.x, position.y, position.z), speed)

    def odom_yaw(self) -> float:
        message = self.latest_odom
        if message is None:
            return math.nan
        orientation = message.pose.pose.orientation
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )

    def wait_for_stable_start(self, args: argparse.Namespace) -> None:
        deadline = time.monotonic() + args.startup_timeout
        stable_since: float | None = None
        target_yaw = math.atan2(args.goal_y - args.start_y, args.goal_x - args.start_x)
        while time.monotonic() < deadline and not STOP_REQUESTED:
            stack_error = self.stack_error()
            if stack_error:
                raise RuntimeError(stack_error)
            self.spin_once(0.05)
            position, speed = self.odom_state()
            position_error = math.dist(
                position, (args.start_x, args.start_y, args.start_z)
            )
            yaw_error = abs(
                math.atan2(
                    math.sin(self.odom_yaw() - target_yaw),
                    math.cos(self.odom_yaw() - target_yaw),
                )
            )
            stable = position_error <= 0.10 and speed <= 0.10 and yaw_error <= 0.10
            now = time.monotonic()
            if stable:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= 1.0:
                    return
            else:
                stable_since = None
        if STOP_REQUESTED:
            raise InterruptedError("test interrupted")
        raise TimeoutError(
            "timed out waiting for a stable start pose, speed, and mission heading"
        )

    def publish_goal(self, args: argparse.Namespace) -> None:
        goal = self.PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = "world_enu"
        goal.pose.position.x = args.goal_x
        goal.pose.position.y = args.goal_y
        goal.pose.position.z = args.goal_z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.spin_once(0.1)

    def run(self, args: argparse.Namespace) -> dict[str, Any]:
        self.prepare(args)
        if self.collision is not False:
            raise RuntimeError("collision is not clear immediately before goal publish")
        self.publish_goal(args)
        mission_start = time.monotonic()
        previous_position: tuple[float, float, float] | None = None
        path_m = 0.0
        max_speed = 0.0
        samples = 0
        outcome = "timeout"
        detail = f"mission exceeded {args.timeout:.1f} s"

        while not STOP_REQUESTED:
            elapsed = time.monotonic() - mission_start
            if elapsed >= args.timeout:
                break
            stack_error = self.stack_error()
            if stack_error:
                outcome, detail = "stack_exited", stack_error
                break
            self.spin_once(min(0.05, args.timeout - elapsed))
            if self.latest_odom is None:
                continue
            position, speed = self.odom_state()
            if previous_position is not None:
                step = math.dist(position, previous_position)
                if math.isfinite(step) and step < 2.0:
                    path_m += step
            previous_position = position
            max_speed = max(max_speed, speed)
            samples += 1
            if self.collision is True:
                outcome, detail = "collision", "/sim/collision became true"
                break
            error = math.dist(
                position, (args.goal_x, args.goal_y, args.goal_z)
            )
            if error <= args.position_tolerance and speed <= args.speed_tolerance:
                outcome = "success"
                detail = "goal position and settling-speed tolerances satisfied"
                break

        if STOP_REQUESTED:
            outcome, detail = "interrupted", "test interrupted by signal"
        duration = time.monotonic() - mission_start
        position, speed = self.odom_state()
        goal_error = math.dist(position, (args.goal_x, args.goal_y, args.goal_z))
        return {
            "outcome": outcome,
            "detail": detail,
            "duration_s": duration,
            "path_m": path_m,
            "average_speed_mps": path_m / duration if duration > 0.0 else 0.0,
            "max_speed_mps": max_speed,
            "final_x": position[0],
            "final_y": position[1],
            "final_z": position[2],
            "final_error_m": goal_error,
            "final_speed_mps": speed,
            "odom_samples": samples,
            "collision": self.collision is True,
        }


CSV_FIELDS = (
    "trial",
    "started_at",
    "ended_at",
    "outcome",
    "duration_s",
    "path_m",
    "average_speed_mps",
    "max_speed_mps",
    "final_x",
    "final_y",
    "final_z",
    "final_error_m",
    "final_speed_mps",
    "collision",
    "session_dir",
    "console_log",
    "detail",
)


def write_result(
    run_dir: Path, summary_path: Path, trial: int, result: dict[str, Any]
) -> None:
    json_path = run_dir / f"trial_{trial:04d}.json"
    temporary = json_path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(result, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    temporary.replace(json_path)
    new_file = not summary_path.exists()
    with summary_path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow(result)
        stream.flush()


def run_trial(
    args: argparse.Namespace, run_dir: Path, summary_path: Path, trial: int
) -> dict[str, Any]:
    started_at = timestamp()
    console_path = run_dir / f"trial_{trial:04d}_stack.log"
    known_sessions = session_directories(args.log_root)
    stack: subprocess.Popen[str] | None = None
    console = None
    monitor: MissionMonitor | None = None
    print(f"[{started_at}] trial {trial}: resetting AirSim", flush=True)
    result: dict[str, Any]
    try:
        conflicts = conflicting_processes()
        if conflicts:
            raise RuntimeError(
                "controller processes remained before AirSim reset: "
                + ", ".join(str(pid) for pid, _ in conflicts)
            )
        reset_airsim(args)
        deadline = time.monotonic() + args.reset_settle
        while time.monotonic() < deadline and not STOP_REQUESTED:
            time.sleep(min(0.1, deadline - time.monotonic()))
        if STOP_REQUESTED:
            raise InterruptedError("test interrupted")
        stack, console = start_stack(args, console_path)
        print(
            f"trial {trial}: {args.stack} pid={stack.pid}; waiting for readiness",
            flush=True,
        )
        monitor = MissionMonitor(args, stack)
        result = monitor.run(args)
        # Let the structured logger consume the terminal collision/goal state
        # before terminating the launch process.
        terminal_deadline = time.monotonic() + 0.5
        while time.monotonic() < terminal_deadline:
            monitor.spin_once(min(0.05, terminal_deadline - time.monotonic()))
    except InterruptedError as error:
        result = {"outcome": "interrupted", "detail": str(error)}
    except Exception as error:
        result = {
            "outcome": "startup_failed" if monitor is None else "failed",
            "detail": f"{type(error).__name__}: {error}",
        }
    finally:
        if monitor is not None:
            monitor.close()
        stop_stack(stack)
        if console is not None:
            console.close()

    result.update(
        {
            "trial": trial,
            "started_at": started_at,
            "ended_at": timestamp(),
            "goal": [args.goal_x, args.goal_y, args.goal_z],
            "expected_start": [args.start_x, args.start_y, args.start_z],
            "position_tolerance_m": args.position_tolerance,
            "speed_tolerance_mps": args.speed_tolerance,
            "timeout_s": args.timeout,
            "session_dir": newest_new_session(args.log_root, known_sessions),
            "console_log": str(console_path),
        }
    )
    write_result(run_dir, summary_path, trial, result)
    print(
        f"trial {trial}: {result['outcome']} "
        f"duration={result.get('duration_s', float('nan')):.3f}s "
        f"error={result.get('final_error_m', float('nan')):.3f}m "
        f"session={result['session_dir'] or 'missing'}",
        flush=True,
    )
    if result.get("detail"):
        print(f"trial {trial}: {result['detail']}", flush=True)
    return result


def configuration(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "stack": args.stack,
        "count": args.count,
        "timeout_s": args.timeout,
        "startup_timeout_s": args.startup_timeout,
        "cooldown_s": args.cooldown,
        "start": [args.start_x, args.start_y, args.start_z],
        "goal": [args.goal_x, args.goal_y, args.goal_z],
        "start_tolerance_m": args.start_tolerance,
        "position_tolerance_m": args.position_tolerance,
        "speed_tolerance_mps": args.speed_tolerance,
        "airsim": f"{args.airsim_host}:{args.airsim_port}",
        "semantic": not args.no_semantic,
        "log_root": str(args.log_root.resolve()),
        "results_root": str(args.results_root.resolve()),
    }


def main() -> int:
    args = parse_args()
    lock_file = acquire_lock()
    validate_environment(args)
    config = configuration(args)
    print(json.dumps(config, indent=2, ensure_ascii=True), flush=True)
    if args.dry_run:
        print("dry-run passed; AirSim and ROS were not changed")
        return 0

    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.results_root.resolve() / f"run_{run_stamp}_{os.getpid()}"
    run_dir.mkdir(parents=True)
    (run_dir / "config.json").write_text(
        json.dumps(config, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    summary_path = run_dir / "summary.csv"
    print(f"results: {run_dir}", flush=True)

    trial = 1
    while not STOP_REQUESTED and (args.count == 0 or trial <= args.count):
        run_trial(args, run_dir, summary_path, trial)
        if STOP_REQUESTED or (args.count != 0 and trial >= args.count):
            break
        deadline = time.monotonic() + args.cooldown
        while time.monotonic() < deadline and not STOP_REQUESTED:
            time.sleep(min(0.1, deadline - time.monotonic()))
        trial += 1
    # Keep the descriptor referenced until every trial and cleanup has completed.
    _ = lock_file
    print(f"test stopped; summary={summary_path}", flush=True)
    return 130 if STOP_REQUESTED else 0


if __name__ == "__main__":
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1) from None
