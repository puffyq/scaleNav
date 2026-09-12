from __future__ import annotations

import argparse
from pathlib import Path

import torch

from config.config import cfg
from policy.spline_yopo_trainer import SplineYopoTrainer
from train_yopo import configure_random_seed


def main() -> None:
    parser = argparse.ArgumentParser(description="Train cubic B-spline Route-YOPO")
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=24)
    parser.add_argument("--learning-rate", type=float, default=1e-5)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--seed", type=int, default=602042)
    parser.add_argument("--control-points", type=int, default=12)
    parser.add_argument("--smoothness-weight", type=float, default=10.0)
    parser.add_argument("--collision-weight", type=float, default=1.0)
    parser.add_argument("--acceleration-weight", type=float, default=1.0)
    parser.add_argument("--frontier-weight", type=float, default=0.15)
    parser.add_argument("--route-weight", type=float, default=1.0)
    parser.add_argument("--bubble-radius-weight", type=float, default=0.0)
    parser.add_argument("--route-only", action="store_true")
    parser.add_argument("--score-only", action="store_true")
    parser.add_argument("--save-interval", type=int, default=3)
    args = parser.parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is unavailable")
    configure_random_seed(args.seed)
    cfg["ws"] = float(args.smoothness_weight)
    cfg["wc"] = float(args.collision_weight)
    cfg["wa"] = float(args.acceleration_weight)
    cfg["wg"] = float(args.frontier_weight)
    cfg["wp"] = float(args.route_weight)
    trainer = SplineYopoTrainer(
        data_root=args.data,
        output_root=args.output,
        checkpoint_path=args.checkpoint,
        learning_rate=args.learning_rate,
        batch_size=args.batch_size,
        num_workers=args.workers,
        device=args.device,
        random_seed=args.seed,
        control_point_count=args.control_points,
        route_only=args.route_only,
        score_only=args.score_only,
        bubble_radius_weight=args.bubble_radius_weight,
    )
    trainer.train(args.epochs, save_interval=args.save_interval)


if __name__ == "__main__":
    main()
