#!/usr/bin/env python3
"""Repeatable GPU/CPU latency benchmark for the Route-YOPO control tick."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import torch

from graph.depth_query import DepthSafeVolumeQuery
from route_yopo_control_core import sample_poly5_candidates
from route_yopo_control_ros2 import RouteYopoController


def _percentiles(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=np.float64)
    return {
        "p50_ms": float(np.percentile(array, 50.0)),
        "p95_ms": float(np.percentile(array, 95.0)),
        "max_ms": float(np.max(array)),
        "mean_ms": float(np.mean(array)),
    }


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        default=str(root / "train_scalenav/saved_route_centerline_w01_train_large_001/YOPO_0/epoch12.pth"),
    )
    parser.add_argument("--train-root", default=str(root / "train_scalenav"))
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--model-iterations", type=int, default=1000)
    parser.add_argument("--full-iterations", type=int, default=100)
    args = parser.parse_args()
    if min(args.warmup, args.model_iterations, args.full_iterations) <= 0:
        parser.error("iteration counts must be positive")
    return args


def main() -> None:
    args = parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    device = torch.device(args.device)
    train_path = str(Path(args.train_root).expanduser().resolve())
    sys.path[:] = [entry for entry in sys.path if entry != train_path]
    sys.path.insert(0, train_path)
    from config.config import cfg
    from policy.yopo_network import YopoNetwork

    checkpoint_path = Path(args.model).expanduser().resolve()
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model = YopoNetwork().to(device).eval()
    feature_order = model.load_route_checkpoint(checkpoint)
    height, width = int(cfg["image_height"]), int(cfg["image_width"])
    bubble_count = int(cfg["route_bubble_count"])
    depth = torch.ones((1, 1, height, width), device=device)
    motion = torch.zeros((1, 6), device=device)
    frontier = torch.tensor([[10.0, 0.0, 0.0]], device=device)
    route = torch.zeros((1, bubble_count, 4), device=device)
    route[:, :, 0] = 1.0
    route[:, :, 3] = 0.5

    @torch.inference_mode()
    def infer(
        depth_input: torch.Tensor = depth,
        motion_input: torch.Tensor = motion,
        frontier_input: torch.Tensor = frontier,
        route_input: torch.Tensor = route,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        return model(depth_input, motion_input, frontier_input, route_input)

    for _ in range(args.warmup):
        infer()
    if device.type == "cuda":
        torch.cuda.synchronize(device)
        torch.cuda.reset_peak_memory_stats(device)

    model_times: list[float] = []
    for _ in range(args.model_iterations):
        started = time.perf_counter()
        infer()
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        model_times.append((time.perf_counter() - started) * 1000.0)

    adapter = RouteYopoController.__new__(RouteYopoController)
    adapter.args = SimpleNamespace(
        minimum_altitude=0.25, minimum_depth=0.04, max_depth=20.0
    )
    adapter.image_width = width
    adapter.image_height = height
    raw_depth = np.full((height, width), 20.0, dtype=np.float32)
    query = DepthSafeVolumeQuery(
        raw_depth,
        horizontal_fov_deg=90.0,
        vertical_fov_deg=73.7398,
        robot_radius_m=0.3,
        safety_margin_m=0.2,
        sample_step_m=0.2,
        far_depth_m=20.0,
        max_unknown_fraction=0.2,
    )
    position = np.array([0.0, 0.0, 1.6], dtype=np.float64)
    zero = np.zeros(3, dtype=np.float64)
    rotation = np.eye(3, dtype=np.float64)
    full_times: list[float] = []
    certified_counts: list[int] = []
    for _ in range(args.full_iterations):
        started = time.perf_counter()
        model_depth = adapter._model_depth(raw_depth)
        depth_tick = torch.from_numpy(model_depth[None, None]).to(device)
        motion_tick = torch.from_numpy(np.zeros((1, 6), dtype=np.float32)).to(device)
        frontier_tick = torch.from_numpy(np.array([[10.0, 0.0, 0.0]], dtype=np.float32)).to(device)
        route_tick = route.detach().clone()
        endstate, _ = infer(depth_tick, motion_tick, frontier_tick, route_tick)
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        endstates = endstate[0].permute(1, 2, 0).reshape(-1, 9).cpu().numpy()
        trajectories = sample_poly5_candidates(
            position,
            zero,
            zero,
            endstates,
            rotation,
            segment_time_s=float(cfg["sgm_time"]),
            sample_count=101,
        )
        safety = [
            adapter._validate_trajectory(query, trajectory, position, rotation)
            for trajectory in trajectories
        ]
        certified_counts.append(sum(item["state"] == "CERTIFIED" for item in safety))
        full_times.append((time.perf_counter() - started) * 1000.0)

    report = {
        "checkpoint": str(checkpoint_path),
        "feature_order": feature_order,
        "device": str(device),
        "gpu_name": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "model_iterations": args.model_iterations,
        "full_control_tick_iterations": args.full_iterations,
        "model_inference": _percentiles(model_times),
        "full_control_tick": _percentiles(full_times),
        "peak_gpu_memory_mib": None
        if device.type != "cuda"
        else float(torch.cuda.max_memory_allocated(device) / (1024.0 * 1024.0)),
        "certified_candidate_count_min_max": [
            int(min(certified_counts)),
            int(max(certified_counts)),
        ],
        "benchmark_input": "synthetic 96x160 far-depth frame, zero motion, straight route",
        "control_output": "/scalenav/trajectory_point@50Hz",
    }
    print(json.dumps(report, indent=2, ensure_ascii=True))


if __name__ == "__main__":
    main()
