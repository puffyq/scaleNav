from __future__ import annotations

import argparse
import os
import random
from pathlib import Path

import numpy as np
import torch

from config.config import cfg
from policy.yopo_trainer import YopoTrainer


def configure_random_seed(seed: int) -> None:
    random.seed(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train route-conditioned ScaleNav YOPO")
    parser.add_argument("--data", type=Path, default=Path(__file__).resolve().parent / "dataset")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "saved")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument(
        "--finetune",
        action="store_true",
        help="load model weights but reset optimizer, epoch, and best validation cost",
    )
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=1.5e-4)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", choices=("cpu", "cuda"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--freeze-backbone-epochs", type=int, default=3)
    parser.add_argument("--save-interval", type=int, default=5)
    parser.add_argument("--max-train-batches", type=int)
    parser.add_argument("--max-val-batches", type=int)
    parser.add_argument("--safety-weight", type=float)
    parser.add_argument(
        "--goal-weight",
        type=float,
        help="guidance/progress loss weight",
    )
    parser.add_argument(
        "--safety-attraction-weight",
        type=float,
        help="guide-line attraction weight inside SafetyLoss",
    )
    parser.add_argument(
        "--route-weight",
        type=float,
        help="deprecated angle loss weight; only zero is accepted",
    )
    parser.add_argument(
        "--centerline-weight",
        type=float,
        help="deprecated centerline MSE weight; only zero is accepted",
    )
    parser.add_argument(
        "--corridor-weight",
        type=float,
        help="weight of the ordered Bubble corridor barrier",
    )
    parser.add_argument(
        "--score-only", action="store_true",
        help="freeze trajectory generation and train only the final score channel",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    if args.safety_weight is not None:
        if args.safety_weight < 0.0:
            raise ValueError("--safety-weight must be non-negative")
        cfg["wc"] = float(args.safety_weight)
    if args.goal_weight is not None:
        if args.goal_weight < 0.0:
            raise ValueError("--goal-weight must be non-negative")
        cfg["wg"] = float(args.goal_weight)
    if args.route_weight is not None:
        if args.route_weight != 0.0:
            raise ValueError("--route-weight is deprecated; use --corridor-weight")
        cfg["wp"] = 0.0
    if args.centerline_weight is not None:
        if args.centerline_weight != 0.0:
            raise ValueError("--centerline-weight is deprecated and disabled")
        cfg["wcenterline"] = 0.0
    if args.corridor_weight is not None:
        if args.corridor_weight < 0.0:
            raise ValueError("--corridor-weight must be non-negative")
        cfg["wcorridor"] = float(args.corridor_weight)
    if args.safety_attraction_weight is not None:
        if args.safety_attraction_weight != 0.0:
            raise ValueError("--safety-attraction-weight is disabled")
        cfg["safety_route_attraction_weight"] = 0.0
    configure_random_seed(args.seed)
    trainer = YopoTrainer(
        data_root=args.data,
        learning_rate=args.learning_rate,
        batch_size=args.batch_size,
        num_workers=args.workers,
        tensorboard_path=args.output,
        checkpoint_path=args.checkpoint,
        resume_training_state=not args.finetune,
        device=args.device,
        random_seed=args.seed,
        score_only=args.score_only,
    )
    trainer.train(
        args.epochs,
        freeze_backbone_epochs=args.freeze_backbone_epochs,
        save_interval=args.save_interval,
        max_train_batches=args.max_train_batches,
        max_val_batches=args.max_val_batches,
    )


if __name__ == "__main__":
    main()
