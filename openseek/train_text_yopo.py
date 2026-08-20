import argparse

from text_tracker.dataset import TextYopoDataset
from text_tracker.trainer import TextYopoTrainer


def main() -> None:
    parser = argparse.ArgumentParser(description="Train YOPO with retained 3-D goals selected by PEARL visibility.")
    parser.add_argument("--train-data", required=True)
    parser.add_argument("--test-data", required=True)
    parser.add_argument("--output", default="saved/TextYOPO_dual_heatmap")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--semantic-weight", type=float, default=1.2)
    parser.add_argument(
        "--approach-probability",
        type=float,
        default=1.0,
        help="Probability of using a visible PEARL peak as the 3-D goal (default: always).",
    )
    parser.add_argument("--pearl-enter-threshold", type=float, default=0.08)
    parser.add_argument("--heatmap-sigma-deg", type=float, default=7.5)
    parser.add_argument("--pretrained-backbone")
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument(
        "--precision",
        choices=("fp32", "amp"),
        default="fp32",
        help="Training precision. fp32 is the stable default; amp is faster on CUDA.",
    )
    parser.add_argument(
        "--eval-every",
        type=int,
        default=5,
        help="Evaluate and write Test metrics every N epochs; 0 means final only.",
    )
    parser.add_argument(
        "--checkpoint-every",
        type=int,
        default=5,
        help="Save TorchScript and state checkpoints every N epochs; 0 disables.",
    )
    parser.add_argument(
        "--tensorboard-dir",
        help="TensorBoard log directory (default: <output>/tensorboard).",
    )
    args = parser.parse_args()

    train_dataset = TextYopoDataset(
        args.train_data,
        seed=0,
        approach_probability=args.approach_probability,
        pearl_enter_threshold=args.pearl_enter_threshold,
        heatmap_sigma_deg=args.heatmap_sigma_deg,
    )
    test_dataset = TextYopoDataset(
        args.test_data,
        seed=100000,
        approach_probability=args.approach_probability,
        pearl_enter_threshold=args.pearl_enter_threshold,
        heatmap_sigma_deg=args.heatmap_sigma_deg,
    )
    print(
        f"Training samples: {len(train_dataset)} frames, "
        f"visible 3-D goals: {train_dataset.visible_record_count}, "
        f"random goals: {len(train_dataset) - train_dataset.visible_record_count}"
    )
    print(
        f"Testing samples: {len(test_dataset)} frames, "
        f"visible 3-D goals: {test_dataset.visible_record_count}, "
        f"random goals: {len(test_dataset) - test_dataset.visible_record_count}"
    )
    trainer = TextYopoTrainer(
        train_dataset,
        test_dataset,
        args.output,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        semantic_weight=args.semantic_weight,
        pretrained_backbone=args.pretrained_backbone,
        num_workers=args.workers,
        tensorboard_dir=args.tensorboard_dir,
        precision=args.precision,
    )
    try:
        metrics = trainer.train(
            args.epochs,
            eval_every=args.eval_every,
            checkpoint_every=args.checkpoint_every,
        )
        print("Final test " + trainer.format_metrics(metrics))
    finally:
        trainer.close()


if __name__ == "__main__":
    main()
