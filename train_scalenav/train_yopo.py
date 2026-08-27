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
    parser.add_argument("--route-dropout", type=float)
    parser.add_argument(
        "--progress-weight",
        type=float,
        help="override wprogress for this run; stored in the checkpoint",
    )
    parser.add_argument("--progress-floor-m", type=float)
    parser.add_argument("--progress-floor-weight", type=float)
    parser.add_argument("--safety-weight", type=float)
    parser.add_argument("--safety-peak-weight", type=float)
    parser.add_argument("--safety-collision-margin-weight", type=float)
    parser.add_argument("--safety-ranking-weight", type=float)
    parser.add_argument("--safety-ranking-target-margin", type=float)
    parser.add_argument("--score-ranking-weight", type=float)
    parser.add_argument(
        "--bubble-weight",
        type=float,
        help="override wp, the ESDF-like attraction weight into witness bubbles",
    )
    parser.add_argument(
        "--path-mse-weight",
        type=float,
        help="override wpath_mse, the ordered witness centerline guidance weight",
    )
    parser.add_argument("--centerline-weight", type=float)
    parser.add_argument(
        "--score-only", action="store_true",
        help="freeze trajectory generation and train only the final score channel",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    if args.progress_weight is not None:
        if args.progress_weight < 0.0:
            raise ValueError("--progress-weight must be non-negative")
        cfg["wprogress"] = float(args.progress_weight)
    if args.progress_floor_m is not None:
        if args.progress_floor_m <= 0.0:
            raise ValueError("--progress-floor-m must be positive")
        cfg["route_progress_floor_m"] = float(args.progress_floor_m)
    if args.progress_floor_weight is not None:
        if args.progress_floor_weight < 0.0:
            raise ValueError("--progress-floor-weight must be non-negative")
        cfg["wprogress_floor"] = float(args.progress_floor_weight)
    if args.safety_weight is not None:
        if args.safety_weight < 0.0:
            raise ValueError("--safety-weight must be non-negative")
        cfg["wc"] = float(args.safety_weight)
    if args.safety_peak_weight is not None:
        if args.safety_peak_weight < 0.0:
            raise ValueError("--safety-peak-weight must be non-negative")
        cfg["safety_peak_weight"] = float(args.safety_peak_weight)
    if args.safety_collision_margin_weight is not None:
        if args.safety_collision_margin_weight < 0.0:
            raise ValueError("--safety-collision-margin-weight must be non-negative")
        cfg["safety_collision_margin_weight"] = float(args.safety_collision_margin_weight)
    if args.safety_ranking_weight is not None:
        if args.safety_ranking_weight < 0.0:
            raise ValueError("--safety-ranking-weight must be non-negative")
        cfg["safety_ranking_weight"] = float(args.safety_ranking_weight)
    if args.safety_ranking_target_margin is not None:
        if args.safety_ranking_target_margin < 0.0:
            raise ValueError("--safety-ranking-target-margin must be non-negative")
        cfg["safety_ranking_target_margin"] = float(args.safety_ranking_target_margin)
    if args.score_ranking_weight is not None:
        if args.score_ranking_weight < 0.0:
            raise ValueError("--score-ranking-weight must be non-negative")
        cfg["score_ranking_weight"] = float(args.score_ranking_weight)
    if args.bubble_weight is not None:
        if args.bubble_weight < 0.0:
            raise ValueError("--bubble-weight must be non-negative")
        cfg["wp"] = float(args.bubble_weight)
    if args.path_mse_weight is not None:
        if args.path_mse_weight < 0.0:
            raise ValueError("--path-mse-weight must be non-negative")
        cfg["wpath_mse"] = float(args.path_mse_weight)
    if args.centerline_weight is not None:
        if args.centerline_weight < 0.0:
            raise ValueError("--centerline-weight must be non-negative")
        cfg["wcenterline"] = float(args.centerline_weight)
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
        route_dropout_probability=args.route_dropout,
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
