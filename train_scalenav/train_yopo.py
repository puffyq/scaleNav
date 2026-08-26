from __future__ import annotations

import argparse
import os
import random
from pathlib import Path

import numpy as np
import torch

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
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda requested but CUDA is unavailable")
    configure_random_seed(args.seed)
    trainer = YopoTrainer(
        data_root=args.data,
        learning_rate=args.learning_rate,
        batch_size=args.batch_size,
        num_workers=args.workers,
        tensorboard_path=args.output,
        checkpoint_path=args.checkpoint,
        device=args.device,
        route_dropout_probability=args.route_dropout,
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
