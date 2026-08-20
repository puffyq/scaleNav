#!/usr/bin/env python3
"""Read-only HTTP viewer for OpenSeek JSONL event logs."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import mimetypes
import re
import threading
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np


EVENT_RE = re.compile(br'"event":"([^"]+)"')
WALL_TIME_RE = re.compile(br'"wall_time":([-+0-9.eE]+)')
FRAME_RE = re.compile(br'"frame_index":([0-9]+)')
SCORE_RE = re.compile(br'"selected_score":([-+0-9.eE]+)')


def extract(pattern: re.Pattern[bytes], line: bytes, default=None):
    match = pattern.search(line)
    return default if match is None else match.group(1)


def read_json_at(path: Path, offset: int | None) -> dict | None:
    if offset is None:
        return None
    with path.open("rb") as stream:
        stream.seek(offset)
        line = stream.readline()
    try:
        return json.loads(line)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None


@dataclass
class FrameEntry:
    frame_index: int
    wall_time: float
    model_offset: int
    depth_offset: int | None
    odom_offset: int | None
    lidar_offset: int | None
    selected_score: float | None
    trajectory_offset: int | None = None
    control_offsets: list[int] = field(default_factory=list)


class SessionIndex:
    def __init__(self, path: Path, log_root: Path) -> None:
        self.path = path
        self.log_root = log_root
        self.id = path.stem.removeprefix("openseek_events_")
        stat = path.stat()
        self.size = stat.st_size
        self.mtime_ns = stat.st_mtime_ns
        self.startup: dict = {}
        self.frames: list[FrameEntry] = []
        self.frames_by_number: dict[int, FrameEntry] = {}
        self.odom_times: list[float] = []
        self.odom_offsets: list[int] = []
        self._scan()

    def stale(self) -> bool:
        stat = self.path.stat()
        return stat.st_size != self.size or stat.st_mtime_ns != self.mtime_ns

    def _scan(self) -> None:
        last_depth = None
        last_odom = None
        last_lidar = None
        with self.path.open("rb") as stream:
            while True:
                offset = stream.tell()
                line = stream.readline()
                if not line:
                    break
                event_raw = extract(EVENT_RE, line)
                if event_raw is None:
                    continue
                event = event_raw.decode("ascii", errors="ignore")
                wall_raw = extract(WALL_TIME_RE, line, b"0")
                try:
                    wall_time = float(wall_raw)
                except ValueError:
                    wall_time = 0.0

                if event == "startup" and not self.startup:
                    self.startup = read_json_at(self.path, offset) or {}
                elif event == "depth":
                    last_depth = offset
                elif event == "odom":
                    last_odom = offset
                    self.odom_times.append(wall_time)
                    self.odom_offsets.append(offset)
                elif event == "lidar":
                    last_lidar = offset
                elif event == "model":
                    frame_raw = extract(FRAME_RE, line)
                    if frame_raw is None:
                        continue
                    frame_index = int(frame_raw)
                    score_raw = extract(SCORE_RE, line)
                    score = None if score_raw is None else float(score_raw)
                    entry = FrameEntry(
                        frame_index=frame_index,
                        wall_time=wall_time,
                        model_offset=offset,
                        depth_offset=last_depth,
                        odom_offset=last_odom,
                        lidar_offset=last_lidar,
                        selected_score=score,
                    )
                    self.frames.append(entry)
                    self.frames_by_number[frame_index] = entry
                elif event in ("trajectory", "control"):
                    frame_raw = extract(FRAME_RE, line)
                    if frame_raw is None:
                        continue
                    entry = self.frames_by_number.get(int(frame_raw))
                    if entry is None:
                        continue
                    if event == "trajectory":
                        entry.trajectory_offset = offset
                    else:
                        entry.control_offsets.append(offset)

    def summary(self) -> dict:
        return {
            "id": self.id,
            "file": self.path.name,
            "size_bytes": self.size,
            "frame_count": len(self.frames),
            "first_wall_time": None if not self.frames else self.frames[0].wall_time,
            "last_wall_time": None if not self.frames else self.frames[-1].wall_time,
            "frames": [
                {
                    "frame_index": frame.frame_index,
                    "wall_time": frame.wall_time,
                    "selected_score": frame.selected_score,
                }
                for frame in self.frames
            ],
            "startup": self.startup,
        }

    def frame_payload(self, frame_number: int) -> dict:
        entry = self.frames_by_number.get(frame_number)
        if entry is None:
            raise KeyError(f"frame {frame_number} does not exist")
        model = read_json_at(self.path, entry.model_offset) or {}
        depth = read_json_at(self.path, entry.depth_offset)
        odom = read_json_at(self.path, entry.odom_offset)
        lidar = read_json_at(self.path, entry.lidar_offset)
        trajectory = read_json_at(self.path, entry.trajectory_offset)
        controls = [
            record
            for record in (read_json_at(self.path, offset) for offset in entry.control_offsets)
            if record is not None
        ]

        segment_time = float(self.startup.get("segment_time_s", 1.0))
        window_start = entry.wall_time - 0.75
        window_end = entry.wall_time + max(segment_time, 0.5)
        first = bisect.bisect_left(self.odom_times, window_start)
        last = bisect.bisect_right(self.odom_times, window_end)
        context_offsets = self.odom_offsets[first:last]
        if len(context_offsets) > 300:
            stride = math.ceil(len(context_offsets) / 300)
            context_offsets = context_offsets[::stride]
        odom_context = [
            record
            for record in (read_json_at(self.path, offset) for offset in context_offsets)
            if record is not None
        ]

        return {
            "session_id": self.id,
            "startup": self.startup,
            "frame": {
                "frame_index": entry.frame_index,
                "wall_time": entry.wall_time,
            },
            "depth": depth,
            "model": model,
            "trajectory": trajectory,
            "controls": controls,
            "odom": odom,
            "odom_context": odom_context,
            "lidar": lidar,
            "planned_path_world": sample_planned_path(trajectory, segment_time),
        }

    def safe_depth_path(self, frame_number: int, kind: str) -> Path:
        entry = self.frames_by_number.get(frame_number)
        if entry is None:
            raise KeyError(f"frame {frame_number} does not exist")
        model = read_json_at(self.path, entry.model_offset) or {}
        key = "raw_depth_png" if kind == "raw" else "model_depth_png"
        value = model.get(key)
        if not value:
            raise FileNotFoundError(f"{key} was not recorded")
        path = Path(value).expanduser().resolve()
        try:
            path.relative_to(self.log_root)
        except ValueError as error:
            raise PermissionError("depth image is outside log root") from error
        if not path.is_file():
            raise FileNotFoundError(path)
        return path


def sample_planned_path(trajectory: dict | None, duration: float) -> list[list[float]]:
    if not trajectory or not trajectory.get("valid") or duration <= 0.0:
        return []
    keys = (
        "start_position_world",
        "start_velocity_world",
        "start_acceleration_world",
        "end_position_world",
        "end_velocity_world",
        "end_acceleration_world",
    )
    if any(key not in trajectory for key in keys):
        return []
    p0, v0, acc0, p1, v1, acc1 = (
        np.asarray(trajectory[key], dtype=np.float64) for key in keys
    )
    t = duration
    matrix = np.array(
        [[t**3, t**4, t**5], [3 * t**2, 4 * t**3, 5 * t**4], [6 * t, 12 * t**2, 20 * t**3]],
        dtype=np.float64,
    )
    fixed = np.stack((p0, v0, 0.5 * acc0), axis=0)
    rhs = np.stack(
        (
            p1 - (fixed[0] + fixed[1] * t + fixed[2] * t**2),
            v1 - (fixed[1] + 2 * fixed[2] * t),
            acc1 - 2 * fixed[2],
        ),
        axis=0,
    )
    high = np.linalg.solve(matrix, rhs)
    coefficients = np.concatenate((fixed, high), axis=0)
    samples = []
    for sample_time in np.linspace(0.0, t, 41):
        powers = np.array([sample_time**order for order in range(6)])
        samples.append((powers @ coefficients).tolist())
    return samples


class IndexStore:
    def __init__(self, log_root: Path) -> None:
        self.log_root = log_root.resolve()
        self.lock = threading.Lock()
        self.cache: dict[str, SessionIndex] = {}

    def discover(self) -> list[Path]:
        return sorted(
            self.log_root.glob("openseek_events_*.jsonl"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )

    def get(self, session_id: str) -> SessionIndex:
        candidates = {
            path.stem.removeprefix("openseek_events_"): path for path in self.discover()
        }
        path = candidates.get(session_id)
        if path is None:
            raise KeyError(f"session {session_id!r} does not exist")
        with self.lock:
            index = self.cache.get(session_id)
            if index is None or index.stale():
                index = SessionIndex(path, self.log_root)
                self.cache[session_id] = index
            return index

    def summaries(self) -> list[dict]:
        summaries = []
        for path in self.discover():
            session_id = path.stem.removeprefix("openseek_events_")
            stat = path.stat()
            index = self.cache.get(session_id)
            summaries.append(
                {
                    "id": session_id,
                    "file": path.name,
                    "size_bytes": stat.st_size,
                    "frame_count": None if index is None or index.stale() else len(index.frames),
                    "first_wall_time": None,
                    "last_wall_time": None,
                    "startup": {} if index is None or index.stale() else index.startup,
                }
            )
        return summaries


class ViewerHandler(BaseHTTPRequestHandler):
    server_version = "OpenSeekLogViewer/1.0"

    @property
    def app(self):
        return self.server.app

    def log_message(self, message: str, *args) -> None:
        print(f"{self.address_string()} - {message % args}")

    def send_json(self, value, status=HTTPStatus.OK) -> None:
        body = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def query_one(self, query: dict[str, list[str]], name: str) -> str:
        values = query.get(name)
        if not values or not values[0]:
            raise ValueError(f"missing query parameter: {name}")
        return values[0]

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        try:
            if parsed.path == "/api/sessions":
                self.send_json({"sessions": self.app.store.summaries()})
            elif parsed.path == "/api/session":
                session = self.app.store.get(self.query_one(query, "id"))
                self.send_json(session.summary())
            elif parsed.path == "/api/frame":
                session = self.app.store.get(self.query_one(query, "id"))
                frame = int(self.query_one(query, "frame"))
                self.send_json(session.frame_payload(frame))
            elif parsed.path == "/api/depth-image":
                self.serve_depth_image(query)
            elif parsed.path == "/api/depth-points":
                self.serve_depth_points(query)
            elif parsed.path == "/" or parsed.path.startswith("/static/"):
                self.serve_static(parsed.path)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except (KeyError, FileNotFoundError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.NOT_FOUND)
        except (ValueError, PermissionError) as error:
            self.send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
        except BrokenPipeError:
            pass

    def depth_array(self, query) -> tuple[np.ndarray, SessionIndex, int, str]:
        session = self.app.store.get(self.query_one(query, "id"))
        frame = int(self.query_one(query, "frame"))
        kind = query.get("kind", ["model"])[0]
        if kind not in ("raw", "model"):
            raise ValueError("kind must be raw or model")
        path = session.safe_depth_path(frame, kind)
        depth_mm = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if depth_mm is None or depth_mm.ndim != 2:
            raise ValueError(f"could not decode depth PNG: {path.name}")
        return depth_mm.astype(np.float32) / 1000.0, session, frame, kind

    def serve_depth_image(self, query) -> None:
        depth_m, _, _, _ = self.depth_array(query)
        max_m = float(query.get("max_m", ["20"])[0])
        if max_m <= 0.0:
            raise ValueError("max_m must be positive")
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        normalized = np.clip(depth_m / max_m, 0.0, 1.0)
        intensity = np.uint8(np.rint((1.0 - normalized) * 255.0))
        colored = cv2.applyColorMap(intensity, cv2.COLORMAP_TURBO)
        colored[~valid] = 0
        ok, encoded = cv2.imencode(".png", colored)
        if not ok:
            raise ValueError("failed to encode depth preview")
        self.send_bytes(encoded.tobytes(), "image/png")

    def serve_depth_points(self, query) -> None:
        depth_m, session, frame, kind = self.depth_array(query)
        stride = max(1, min(16, int(query.get("stride", ["4"])[0])))
        max_m = float(query.get("max_m", ["20"])[0])
        height, width = depth_m.shape
        startup = session.startup
        hfov = float(startup.get("model_horizontal_fov_deg", 90.0))
        focal = width / (2.0 * math.tan(math.radians(hfov) / 2.0))
        rows, columns = np.mgrid[0:height:stride, 0:width:stride]
        z_forward = depth_m[::stride, ::stride]
        valid = np.isfinite(z_forward) & (z_forward > 0.0) & (z_forward <= max_m)
        x_body = z_forward
        y_body = -(columns + 0.5 - width / 2.0) * z_forward / focal
        z_body = -(rows + 0.5 - height / 2.0) * z_forward / focal
        points = np.stack((x_body, y_body, z_body), axis=-1)[valid]
        distances = z_forward[valid]
        self.send_json(
            {
                "session_id": session.id,
                "frame_index": frame,
                "kind": kind,
                "unit": "meter",
                "depth_semantics": "camera forward-axis z-depth",
                "frame": "body_flu_approx_camera_origin",
                "points": points.tolist(),
                "depth_m": distances.tolist(),
            }
        )

    def serve_static(self, request_path: str) -> None:
        relative = "index.html" if request_path == "/" else request_path.removeprefix("/static/")
        path = (self.app.static_root / relative).resolve()
        try:
            path.relative_to(self.app.static_root)
        except ValueError as error:
            raise PermissionError("invalid static path") from error
        if not path.is_file():
            raise FileNotFoundError(path.name)
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self.send_bytes(path.read_bytes(), content_type)


class ViewerServer(ThreadingHTTPServer):
    daemon_threads = True


class App:
    def __init__(self, log_root: Path, static_root: Path) -> None:
        self.store = IndexStore(log_root)
        self.static_root = static_root.resolve()


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="OpenSeek event log HTML viewer")
    parser.add_argument("--log-dir", type=Path, default=project_root / "log_event")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    log_root = args.log_dir.expanduser().resolve()
    if not log_root.is_dir():
        raise SystemExit(f"log directory does not exist: {log_root}")
    static_root = Path(__file__).resolve().parent / "static"
    app = App(log_root, static_root)
    server = ViewerServer((args.host, args.port), ViewerHandler)
    server.app = app
    print(f"OpenSeek log viewer: http://{args.host}:{server.server_port}")
    print(f"Log directory: {log_root}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
