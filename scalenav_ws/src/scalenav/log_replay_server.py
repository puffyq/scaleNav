#!/usr/bin/env python3
"""Serve an interactive logged-frame Route-YOPO replay and comparison page."""

from __future__ import annotations

import argparse
import json
import mimetypes
import sys
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse



ROOT = Path(__file__).resolve().parents[3]
WEB = Path(__file__).resolve().parent / "log_replay.html"


def _records(session: Path) -> list[dict]:
    with (session / "index.jsonl").open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _safe_session(root: Path, name: str) -> Path:
    if not name.startswith("session_") or Path(name).name != name:
        raise ValueError("invalid session")
    path = (root / name).resolve()
    if path.parent != root.resolve() or not (path / "index.jsonl").is_file():
        raise FileNotFoundError(name)
    return path


def _depth(path: Path) -> np.ndarray:
    import numpy as np
    import cv2
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim != 2:
        raise ValueError(f"cannot decode depth: {path.name}")
    return image.astype(np.float32) * (0.001 if image.dtype == np.uint16 else 1.0)


class Inferencer:
    def __init__(self, checkpoint: Path, train_root: Path, device: str) -> None:
        import torch
        self.torch = torch
        sys.path.insert(0, str(train_root))
        from config.config import cfg
        from data.route_contract import sample_route_bubbles
        from policy.yopo_network import YopoNetwork
        from route_yopo_control_core import (
            quaternion_xyzw_to_matrix,
            sample_poly5_candidate_states,
            validate_depth_trajectory,
        )

        self.cfg = cfg
        self.sample_route_bubbles = sample_route_bubbles
        self.quaternion = quaternion_xyzw_to_matrix
        self.sample_states = sample_poly5_candidate_states
        self.validate = validate_depth_trajectory
        self.device = torch.device("cuda" if device == "auto" and torch.cuda.is_available() else device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable")
        self.model = YopoNetwork().to(self.device).eval()
        loaded = torch.load(checkpoint, map_location=self.device, weights_only=False)
        self.feature_order = self.model.load_route_checkpoint(loaded)
        from graph.depth_query import DepthSafeVolumeQuery
        self.DepthSafeVolumeQuery = DepthSafeVolumeQuery

    def run(self, session: Path, records: list[dict], depth_record: dict) -> dict:
        import numpy as np
        stamp = int(depth_record["stamp_ns"])
        def latest(kind: str):
            values = [r for r in records if r.get("kind") == kind and int(r.get("stamp_ns", -1)) <= stamp]
            return max(values, key=lambda r: int(r["stamp_ns"])) if values else None

        odom = latest("odom")
        if odom is None:
            raise ValueError("no odometry before depth frame")
        od = odom["data"]
        position = np.asarray(od["position"], dtype=np.float64)
        rotation = self.quaternion(od["orientation"])
        velocity = (rotation @ np.asarray(od.get("velocity", [0, 0, 0]), dtype=np.float64)).astype(np.float32)
        previous = [r for r in records if r.get("kind") == "odom" and int(r.get("stamp_ns", -1)) < int(odom["stamp_ns"])]
        acceleration = np.zeros(3, dtype=np.float32)
        if previous:
            prev = previous[-1]
            dt = (int(odom["stamp_ns"]) - int(prev["stamp_ns"])) * 1e-9
            if 1e-3 < dt < 1.0:
                prev_v = rotation @ np.asarray(prev["data"].get("velocity", [0, 0, 0]), dtype=np.float64)
                acceleration = np.clip((velocity - prev_v) / dt, -20.0, 20.0).astype(np.float32)

        path_record = latest("path")
        path = []
        if path_record and path_record.get("file"):
            payload = json.loads((session / path_record["file"]).read_text(encoding="utf-8"))
            path = payload.get("poses", payload.get("points", []))
        goal_record = latest("local_goal") or latest("goal")
        if goal_record is None:
            raise ValueError("no goal before depth frame")
        goal = np.asarray(goal_record["data"]["position"], dtype=np.float64)
        depth = _depth(session / depth_record["file"])
        query = self.DepthSafeVolumeQuery(depth, horizontal_fov_deg=90.0, vertical_fov_deg=73.7398,
                                          robot_radius_m=0.3, safety_margin_m=0.2, sample_step_m=0.2,
                                          max_unknown_fraction=0.2)
        prepared = np.minimum(np.nan_to_num(depth, nan=20.0), 20.0) / 20.0
        motion = np.concatenate((rotation.T @ velocity, rotation.T @ acceleration)).astype(np.float32)
        goal_body = rotation.T @ (goal - position)
        route_count = int(self.cfg["route_bubble_count"])
        anchors = np.asarray(self.cfg["route_anchor_distances_m"], dtype=np.float32)
        route_features = np.zeros((route_count, 4), dtype=np.float32)
        if len(path) >= 2:
            path_np = np.asarray(path, dtype=np.float32)
            radii = np.full(len(path_np), 1.0, dtype=np.float32)
            centers, sampled_radii, distances = self.sample_route_bubbles(path_np, radii, anchors)
            from route_yopo_control_core import build_route_features
            route_features = build_route_features(centers, sampled_radii, distances, position, rotation,
                                                   radius_clip_m=float(self.cfg["route_clearance_clip_m"]))
        torch = self.torch
        with torch.inference_mode():
            endstate, score = self.model(torch.from_numpy(prepared[None, None]).to(self.device),
                                         torch.from_numpy(motion[None]).to(self.device),
                                         torch.from_numpy(goal_body.astype(np.float32)[None]).to(self.device),
                                         torch.from_numpy(route_features[None]).to(self.device))
        endstates = endstate[0].permute(1, 2, 0).reshape(-1, 9).cpu().numpy()
        trajectories, _, _ = self.sample_states(position, velocity, acceleration, endstates, rotation,
                                                segment_time_s=float(self.cfg["sgm_time"]), sample_count=61)
        safety = [self.validate(query, t, position, rotation, minimum_altitude_m=0.25) for t in trajectories]
        scores = score[0].reshape(-1).cpu().numpy()
        selected = next((i for i in np.argsort(scores) if safety[int(i)]["state"] == "CERTIFIED"), None)
        return {"depth_stamp_ns": stamp, "path": path, "goal": goal.tolist(),
                "candidates": [{"primitive": int(i), "score": float(scores[i]),
                                 "state": safety[i]["state"], "clearance_m": safety[i].get("minimum_clearance_m"),
                                 "trajectory": trajectories[i].round(4).tolist()} for i in range(len(scores))],
                "selected": None if selected is None else int(selected), "feature_order": self.feature_order}


class Handler(BaseHTTPRequestHandler):
    root: Path
    inferencer: Inferencer | None
    cache: dict[tuple[str, int], dict]

    def _json(self, value, status=200):
        body = json.dumps(value, ensure_ascii=True, separators=(",", ":")).encode()
        self.send_response(status); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path); query = parse_qs(parsed.query)
        try:
            if parsed.path == "/api/sessions":
                sessions = []
                for p in sorted(self.root.glob("session_*"), reverse=True):
                    if p.is_dir() and (p / "index.jsonl").is_file(): sessions.append({"name": p.name, "bytes": sum(x.stat().st_size for x in p.rglob("*") if x.is_file())})
                return self._json(sessions)
            if parsed.path == "/api/records":
                session = _safe_session(self.root, query.get("session", [""])[0]); return self._json(_records(session))
            if parsed.path == "/api/infer":
                name = query.get("session", [""])[0]; index = int(query.get("index", ["0"])[0]); session = _safe_session(self.root, name); records = _records(session)
                depths = [r for r in records if r.get("kind") == "depth" and r.get("file")]
                depth = depths[index]
                key = (name, index)
                if key not in self.cache:
                    if self.inferencer is None: raise RuntimeError("inference is disabled; provide --model")
                    self.cache[key] = self.inferencer.run(session, records, depth)
                return self._json(self.cache[key])
            if parsed.path in ("/", "/index.html"):
                body = WEB.read_bytes(); self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body); return
            self.send_error(404)
        except Exception as exc:
            self._json({"error": str(exc)}, 400)

    def log_message(self, *_): pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT / "log_scalenav")
    parser.add_argument("--model", type=Path, help="Route-YOPO checkpoint; omit to browse logs without inference")
    parser.add_argument("--train-root", type=Path, default=ROOT / "train_scalenav")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--host", default="127.0.0.1"); parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    Handler.root = args.root.expanduser().resolve(); Handler.cache = {}
    Handler.inferencer = Inferencer(args.model.resolve(), args.train_root.resolve(), args.device) if args.model else None
    print(f"ScaleNav log replay: http://{args.host}:{args.port}")
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__": main()
