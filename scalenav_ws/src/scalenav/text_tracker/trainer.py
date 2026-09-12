from __future__ import annotations

from pathlib import Path

import torch
from torch.amp import GradScaler, autocast
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter

from config.config import cfg

from .dataset import TextYopoDataset
from .loss import TextYopoGuidanceLoss
from .network import TextYopoNetwork, export_text_yopo_torchscript


def move_batch(batch: dict[str, torch.Tensor], device: torch.device) -> dict[str, torch.Tensor]:
    return {key: value.to(device, non_blocking=True) for key, value in batch.items()}


class TextYopoTrainer:
    DISPLAY_METRICS = (
        "total",
        "trajectory",
        "score",
        "selection_regret",
        "selection_top1",
        "selected_total_cost",
        "oracle_total_cost",
        "selected_goal_cost",
        "oracle_goal_cost",
        "selected_endpoint_distance",
        "oracle_endpoint_distance",
        "random_selected_goal_cost",
        "visible_selected_goal_cost",
    )
    METRIC_GROUPS = {
        "total": "Loss",
        "trajectory": "Loss",
        "score": "Loss",
        "smooth": "Cost",
        "safety": "Cost",
        "acceleration": "Cost",
        "goal": "Cost",
        "selection_regret": "Selection",
        "selection_top1": "Selection",
        "selected_total_cost": "Selection",
        "oracle_total_cost": "Selection",
        "selected_goal_cost": "Goal",
        "oracle_goal_cost": "Goal",
        "selected_endpoint_distance": "Trajectory",
        "oracle_endpoint_distance": "Trajectory",
        "random_selected_goal_cost": "GoalByMode",
        "visible_selected_goal_cost": "GoalByMode",
    }

    def __init__(
        self,
        train_dataset: TextYopoDataset,
        test_dataset: TextYopoDataset,
        output_dir: str,
        batch_size: int = 32,
        learning_rate: float = 1e-3,
        semantic_weight: float = 1.2,
        pretrained_backbone: str | None = None,
        num_workers: int = 0,
        tensorboard_dir: str | None = None,
        precision: str = "fp32",
    ) -> None:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if precision not in {"fp32", "amp"}:
            raise ValueError("precision must be 'fp32' or 'amp'")
        if precision == "amp" and self.device.type != "cuda":
            raise RuntimeError("AMP precision requires a CUDA device")
        self.precision = precision
        self.amp_enabled = precision == "amp"
        self.output_dir = Path(output_dir)
        self.tensorboard_dir = (
            Path(tensorboard_dir)
            if tensorboard_dir is not None
            else self.output_dir / "tensorboard"
        )
        self.tensorboard = SummaryWriter(log_dir=str(self.tensorboard_dir))
        self._logged_inputs: set[str] = set()
        self.completed_epochs = 0
        self.best_selected_total_cost = float("inf")
        self.train_loader = DataLoader(
            train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers,
            pin_memory=self.device.type == "cuda", drop_last=False
        )
        self.test_loader = DataLoader(
            test_dataset, batch_size=batch_size, shuffle=False, num_workers=num_workers,
            pin_memory=self.device.type == "cuda", drop_last=False
        )
        self.model = TextYopoNetwork().to(self.device)
        if pretrained_backbone:
            self._load_pretrained_backbone(pretrained_backbone)
        self.loss = TextYopoGuidanceLoss(
            train_dataset.scene_obstacles, semantic_weight=semantic_weight
        ).to(self.device)
        self.test_loss = TextYopoGuidanceLoss(
            test_dataset.scene_obstacles, semantic_weight=semantic_weight
        ).to(self.device)
        self.optimizer = torch.optim.AdamW(self.model.parameters(), lr=learning_rate)
        self.scaler = GradScaler(enabled=self.amp_enabled)
        print(f"TensorBoard log: {self.tensorboard_dir}")
        print(f"Training precision: {self.precision}")

    def _load_pretrained_backbone(self, checkpoint_path: str) -> None:
        source = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        target = self.model.state_dict()
        loaded = 0
        for key, value in source.items():
            if not key.startswith("image_backbone.") or key not in target:
                continue
            if key == "image_backbone.cnn.conv1.weight":
                target[key][:, :1].copy_(value)
                target[key][:, 1:].zero_()
                loaded += 1
            elif target[key].shape == value.shape:
                target[key].copy_(value)
                loaded += 1
        self.model.load_state_dict(target)
        print(f"Loaded {loaded} image-backbone tensors from {checkpoint_path}")

    def train(
        self,
        epochs: int,
        eval_every: int = 5,
        checkpoint_every: int = 5,
    ) -> dict[str, float]:
        if eval_every < 0 or checkpoint_every < 0:
            raise ValueError("eval_every and checkpoint_every must be non-negative")
        test_metrics: dict[str, float] = {}
        for epoch in range(epochs):
            self.model.train()
            totals: dict[str, float] = {}
            counts: dict[str, int] = {}
            amp_overflow_batches = 0
            consecutive_amp_overflows = 0
            for batch_index, batch in enumerate(self.train_loader, start=1):
                batch = move_batch(batch, self.device)
                self._log_inputs("Train", batch)
                self.optimizer.zero_grad(set_to_none=True)
                with autocast(self.device.type, enabled=self.amp_enabled):
                    prediction = self.model(batch["image"], batch["obs"])
                    total, details = self.loss(prediction, batch)
                if not torch.isfinite(total):
                    raise FloatingPointError(
                        f"Non-finite loss at epoch {epoch + 1}, "
                        f"batch {batch_index}; training stopped"
                    )
                batch_size = batch["image"].shape[0]
                self._accumulate(totals, counts, "total", total.detach(), batch_size)
                self._accumulate_details(totals, counts, details, batch["approach"])
                self.scaler.scale(total).backward()
                self.scaler.unscale_(self.optimizer)
                grad_norm = torch.nn.utils.clip_grad_norm_(
                    self.model.parameters(), 1.0
                )
                if not torch.isfinite(grad_norm):
                    self.optimizer.zero_grad(set_to_none=True)
                    if self.amp_enabled:
                        amp_overflow_batches += 1
                        consecutive_amp_overflows += 1
                        self.scaler.update()
                        if consecutive_amp_overflows >= 5:
                            raise FloatingPointError(
                                "Five consecutive AMP gradient overflows occurred; "
                                "use --precision fp32"
                            )
                        continue
                    raise FloatingPointError(
                        f"Non-finite FP32 gradient at epoch {epoch + 1}, "
                        f"batch {batch_index}; training stopped"
                    )
                if self.amp_enabled:
                    self.scaler.step(self.optimizer)
                    self.scaler.update()
                else:
                    self.optimizer.step()
                consecutive_amp_overflows = 0
            if not counts:
                raise FloatingPointError(
                    f"Every batch in epoch {epoch + 1} was non-finite; training stopped"
                )
            metrics = self._averages(totals, counts)
            epoch_number = epoch + 1
            self.completed_epochs = epoch_number
            self._write_metrics("Train", metrics, epoch_number)
            self.tensorboard.add_scalar(
                "Train/Optimizer/learning_rate",
                self.optimizer.param_groups[0]["lr"],
                epoch_number,
            )
            self.tensorboard.add_scalar(
                "Train/Optimizer/amp_overflow_batches",
                amp_overflow_batches,
                epoch_number,
            )
            if self.amp_enabled:
                self.tensorboard.add_scalar(
                    "Train/Optimizer/amp_scale",
                    self.scaler.get_scale(),
                    epoch_number,
                )
            self.tensorboard.flush()
            summary = self.format_metrics(metrics)
            print(f"Epoch {epoch_number:03d}/{epochs:03d} {summary}")
            should_evaluate = epoch_number == epochs or (
                eval_every > 0 and epoch_number % eval_every == 0
            )
            if should_evaluate:
                test_metrics = self.evaluate(step=epoch_number)
                print(
                    f"Test  {epoch_number:03d}/{epochs:03d} "
                    f"{self.format_metrics(test_metrics)}"
                )
                selected_total_cost = test_metrics.get("selected_total_cost")
                if (
                    selected_total_cost is not None
                    and selected_total_cost < self.best_selected_total_cost
                ):
                    self.best_selected_total_cost = selected_total_cost
                    self.save_best_checkpoint(epoch_number, test_metrics)
            if checkpoint_every > 0 and epoch_number % checkpoint_every == 0:
                self.save_checkpoint(epoch_number)
        self.save()
        return test_metrics

    @torch.inference_mode()
    def evaluate(self, step: int | None = None) -> dict[str, float]:
        self.model.eval()
        totals: dict[str, float] = {}
        counts: dict[str, int] = {}
        for batch in self.test_loader:
            batch = move_batch(batch, self.device)
            self._log_inputs("Test", batch)
            prediction = self.model(batch["image"], batch["obs"])
            total, details = self.test_loss(prediction, batch)
            batch_size = batch["image"].shape[0]
            self._accumulate(totals, counts, "total", total, batch_size)
            self._accumulate_details(totals, counts, details, batch["approach"])
        metrics = self._averages(totals, counts)
        self._write_metrics(
            "Test", metrics, self.completed_epochs if step is None else step
        )
        self.tensorboard.flush()
        return metrics

    def _log_inputs(self, split: str, batch: dict[str, torch.Tensor]) -> None:
        if split in self._logged_inputs:
            return
        images = batch["image"][:8].detach().float().cpu()
        self.tensorboard.add_images(f"{split}/Input/depth", images[:, :1], 0)
        self.tensorboard.add_images(
            f"{split}/Input/pearl_raw", images[:, 1:2].clamp(0.0, 1.0), 0
        )
        self._logged_inputs.add(split)

    def _write_metrics(
        self, split: str, metrics: dict[str, float], step: int
    ) -> None:
        for name, value in metrics.items():
            group = self.METRIC_GROUPS.get(name, "Other")
            self.tensorboard.add_scalar(f"{split}/{group}/{name}", value, step)

    @staticmethod
    def _accumulate(
        totals: dict[str, float],
        counts: dict[str, int],
        name: str,
        value: torch.Tensor,
        count: int,
    ) -> None:
        values = value.float()
        if values.numel() == 1:
            increment = values.item() * count
        elif values.numel() == count:
            increment = values.sum().item()
        else:
            raise ValueError(
                f"Metric {name} has {values.numel()} values for {count} samples"
            )
        totals[name] = totals.get(name, 0.0) + increment
        counts[name] = counts.get(name, 0) + count

    def _accumulate_details(
        self,
        totals: dict[str, float],
        counts: dict[str, int],
        details: dict[str, torch.Tensor],
        approach: torch.Tensor,
    ) -> None:
        batch_size = int(approach.numel())
        for name, value in details.items():
            self._accumulate(totals, counts, name, value, batch_size)

        for label, mask in (
            ("random", approach < 0.5),
            ("visible", approach >= 0.5),
        ):
            count = int(mask.sum().item())
            if count:
                self._accumulate(
                    totals,
                    counts,
                    f"{label}_selected_goal_cost",
                    details["selected_goal_cost"][mask],
                    count,
                )

    @staticmethod
    def _averages(
        totals: dict[str, float], counts: dict[str, int]
    ) -> dict[str, float]:
        return {name: total / counts[name] for name, total in totals.items()}

    @classmethod
    def format_metrics(cls, metrics: dict[str, float]) -> str:
        return " ".join(
            f"{name}={metrics[name]:.4f}"
            for name in cls.DISPLAY_METRICS
            if name in metrics
        )

    def _trace_model(self, script_path: Path) -> None:
        export_text_yopo_torchscript(
            self.model,
            script_path,
            image_height=int(cfg["image_height"]),
            image_width=int(cfg["image_width"]),
        )

    def _assert_model_finite(self) -> None:
        invalid = [
            name
            for name, value in self.model.state_dict().items()
            if not torch.isfinite(value).all()
        ]
        if invalid:
            preview = ", ".join(invalid[:5])
            raise FloatingPointError(
                f"Refusing to save a non-finite model; invalid tensors: {preview}"
            )

    def save_checkpoint(self, epoch: int) -> None:
        self._assert_model_finite()
        checkpoint_dir = self.output_dir / "checkpoints"
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        state_path = checkpoint_dir / f"epoch_{epoch:03d}.pth"
        script_path = checkpoint_dir / f"epoch_{epoch:03d}.pt"
        torch.save(self.model.state_dict(), state_path)
        self._trace_model(script_path)
        print(f"Saved checkpoint {state_path}")
        print(f"Saved checkpoint {script_path}")

    def save_best_checkpoint(
        self, epoch: int, metrics: dict[str, float]
    ) -> None:
        self._assert_model_finite()
        best_dir = self.output_dir / "best"
        best_dir.mkdir(parents=True, exist_ok=True)
        state_path = best_dir / "text_yopo_state.pth"
        script_path = best_dir / "text_yopo.pt"
        metrics_path = best_dir / "metrics.txt"
        torch.save(self.model.state_dict(), state_path)
        self._trace_model(script_path)
        metrics_path.write_text(
            "\n".join(
                [f"epoch={epoch}"]
                + [f"{name}={value:.8f}" for name, value in sorted(metrics.items())]
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            f"Saved best checkpoint {script_path} "
            f"(selected_total_cost={metrics['selected_total_cost']:.4f})"
        )

    def save(self) -> None:
        self._assert_model_finite()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        state_path = self.output_dir / "text_yopo_state.pth"
        script_path = self.output_dir / "text_yopo.pt"
        torch.save(self.model.state_dict(), state_path)
        self._trace_model(script_path)
        print(f"Saved {state_path}")
        print(f"Saved {script_path}")

    def close(self) -> None:
        self.tensorboard.close()
