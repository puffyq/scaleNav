from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import torch
from torch.nn import functional as F
from torch.utils.data import DataLoader
from torch.utils.tensorboard.writer import SummaryWriter

from config.config import cfg
from loss.loss_function import YOPOLoss
from policy.state_transform import (
    project_world_endstate_to_altitude,
    rotate_body2world,
    state_body2world,
)
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork


class YopoTrainer:
    def __init__(
        self,
        *,
        data_root: str | Path | None = None,
        learning_rate: float = 1.5e-4,
        batch_size: int = 16,
        num_workers: int = 4,
        tensorboard_path: str | Path | None = None,
        checkpoint_path: str | Path | None = None,
        resume_training_state: bool = True,
        device: str | torch.device | None = None,
        random_seed: int | None = None,
        score_only: bool = False,
    ) -> None:
        self.batch_size = int(batch_size)
        self.learning_rate = float(learning_rate)
        self.num_workers = int(num_workers)
        self.random_seed = None if random_seed is None else int(random_seed)
        self.score_only = bool(score_only)
        self.max_grad_norm = 0.1
        self.device = torch.device(
            device or ("cuda" if torch.cuda.is_available() else "cpu")
        )
        output_root = Path(tensorboard_path or Path(__file__).resolve().parent.parent / "saved")
        output_root.mkdir(parents=True, exist_ok=True)
        self.output_path = self.get_next_log_path(output_root)
        self.tensorboard_log = SummaryWriter(log_dir=self.output_path)
        self.traj_num = int(cfg["traj_num"])

        self.train_dataset = YOPODataset(
            mode="train",
            data_root=data_root,
            seed=self.random_seed or 0,
        )
        self.valid_dataset = YOPODataset(
            mode="valid",
            data_root=data_root,
            seed=self.random_seed or 0,
        )
        loader_options = {
            "batch_size": self.batch_size,
            "num_workers": int(num_workers),
            "pin_memory": self.device.type == "cuda",
        }
        self.train_dataloader = DataLoader(
            self.train_dataset, shuffle=True, drop_last=False, **loader_options
        )
        self.val_dataloader = DataLoader(
            self.valid_dataset, shuffle=False, drop_last=False, **loader_options
        )

        self.policy = YopoNetwork().to(self.device)
        self.best_validation_cost = float("inf")
        self.epoch_i = -1
        self._checkpoint_optimizer_state = None
        self.checkpoint_path = (
            None if checkpoint_path is None else str(Path(checkpoint_path).resolve())
        )
        self.resume_training_state = bool(resume_training_state)
        if checkpoint_path:
            self.load_checkpoint(
                Path(checkpoint_path), resume_training_state=resume_training_state
            )
        self.yopo_loss = YOPOLoss(
            obstacle_paths=self.train_dataset.obstacle_paths,
            device=self.device,
        ).to(self.device)
        optimizer_options = {"lr": self.learning_rate}
        if self.device.type == "cuda":
            optimizer_options["fused"] = True
        self.optimizer = torch.optim.AdamW(self.policy.parameters(), **optimizer_options)
        if self._checkpoint_optimizer_state is not None:
            self.optimizer.load_state_dict(self._checkpoint_optimizer_state)
        self._write_run_metadata()

    def train(
        self,
        epochs: int,
        *,
        freeze_backbone_epochs: int = 0,
        save_interval: int | None = None,
        max_train_batches: int | None = None,
        max_val_batches: int | None = None,
    ) -> None:
        self._update_run_metadata(
            {
                "epochs": int(epochs),
                "freeze_backbone_epochs": int(freeze_backbone_epochs),
                "save_interval": save_interval,
                "max_train_batches": max_train_batches,
                "max_val_batches": max_val_batches,
            }
        )
        for epoch in range(int(epochs)):
            self.epoch_i = epoch
            self._set_backbone_trainable(epoch >= freeze_backbone_epochs)
            train_metrics = self.run_epoch(
                self.train_dataloader,
                training=True,
                max_batches=max_train_batches,
            )
            validation_metrics = self.run_epoch(
                self.val_dataloader,
                training=False,
                max_batches=max_val_batches,
            )
            self._log_metrics("Train", train_metrics, epoch)
            self._log_metrics("Validation", validation_metrics, epoch)
            validation_cost = validation_metrics["selected_total_cost"]
            if validation_cost < self.best_validation_cost:
                self.best_validation_cost = validation_cost
                self.save_checkpoint(self.output_path / "best.pth")
            if save_interval and (epoch + 1) % save_interval == 0:
                self.save_checkpoint(self.output_path / f"epoch{epoch + 1}.pth")
            print(
                f"epoch={epoch + 1} train={train_metrics['total_loss']:.5f} "
                f"valid={validation_metrics['total_loss']:.5f} "
                f"selected={validation_cost:.5f} "
                f"regret={validation_metrics['selection_regret']:.5f}"
            )
        self.save_checkpoint(self.output_path / "last.pth")
        self.tensorboard_log.flush()

    def run_epoch(
        self,
        dataloader: DataLoader,
        *,
        training: bool,
        max_batches: int | None = None,
    ) -> dict[str, float]:
        self.policy.train(training)
        metrics: list[dict[str, float]] = []
        context = torch.enable_grad() if training else torch.inference_mode()
        with context:
            for batch_index, batch in enumerate(dataloader):
                if max_batches is not None and batch_index >= max_batches:
                    break
                batch = {name: value.to(self.device, non_blocking=True) for name, value in batch.items()}
                if training:
                    self.optimizer.zero_grad(set_to_none=True)
                total_loss, batch_metrics = self.forward_and_compute_loss(batch)
                if training:
                    total_loss.backward()
                    torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                    self.optimizer.step()
                metrics.append(batch_metrics)
        if not metrics:
            raise RuntimeError("epoch did not process any batches")
        return {
            name: float(np.mean([entry[name] for entry in metrics]))
            for name in metrics[0]
        }

    def forward_and_compute_loss(
        self, batch: dict[str, torch.Tensor]
    ) -> tuple[torch.Tensor, dict[str, float]]:
        endstate, score = self.policy(
            batch["depth"],
            batch["motion_body"],
            batch["frontier_body"],
            batch["route_bubbles"],
        )
        batch_size = batch["depth"].shape[0]
        position = batch["position_world"]
        rotation = batch["rotation_world_body"]
        motion = batch["motion_body"]
        start_velocity_world = rotate_body2world(rotation, motion[:, :3])
        start_acceleration_world = rotate_body2world(rotation, motion[:, 3:6])
        start_state_world = torch.stack(
            (position, start_velocity_world, start_acceleration_world), dim=1
        )

        endstate_flat = endstate.permute(0, 2, 3, 1).reshape(batch_size * self.traj_num, 9)
        score_flat = score.reshape(batch_size * self.traj_num)
        position_expanded = position.repeat_interleave(self.traj_num, dim=0)
        rotation_expanded = rotation.repeat_interleave(self.traj_num, dim=0)
        end_position_world, end_velocity_world, end_acceleration_world = state_body2world(
            position_expanded,
            rotation_expanded,
            endstate_flat[:, :3],
            endstate_flat[:, 3:6],
            endstate_flat[:, 6:9],
        )
        frontier_expanded = batch["frontier_world"].repeat_interleave(
            self.traj_num, dim=0
        )
        route_active = (
            (batch["dense_route_world"][:, 1:] - batch["dense_route_world"][:, :-1])
            .norm(dim=-1)
            .gt(1.0e-5)
            .any(dim=1)
        )
        route_active_expanded = route_active.repeat_interleave(self.traj_num)
        route_compatible = self.policy.state_transform.route_primitive_compatibility(
            batch["frontier_body"]
        ).reshape(batch_size * self.traj_num)
        raw_route_altitude_error = torch.abs(
            end_position_world[:, 2] - frontier_expanded[:, 2]
        )
        end_position_world, end_velocity_world, end_acceleration_world = (
            project_world_endstate_to_altitude(
                end_position_world,
                end_velocity_world,
                end_acceleration_world,
                frontier_expanded[:, 2],
                route_active_expanded,
            )
        )
        end_state_world = torch.stack(
            (end_position_world, end_velocity_world, end_acceleration_world), dim=1
        )
        start_state_expanded = start_state_world.repeat_interleave(self.traj_num, dim=0)
        route_points_expanded = batch["dense_route_world"].repeat_interleave(
            self.traj_num, dim=0
        )
        route_radii_expanded = batch["dense_route_radius"].repeat_interleave(
            self.traj_num, dim=0
        )
        costs = self.yopo_loss(
            start_state_expanded,
            end_state_world,
            frontier_expanded,
            batch["map_id"],
            route_points_expanded,
            route_radii_expanded,
        )
        # Fuse the bubble signed-distance field into the original ESDF safety
        # term. The original YOPO terms remain intact; ordered witness MSE is
        # the only additional route supervision. Bubble violation is retained
        # as a diagnostic rather than a separate total-loss component.
        route_weight = batch["route_quality_weight"].repeat_interleave(self.traj_num)
        route_supervision_weight = route_weight * route_compatible.to(route_weight.dtype)
        for name in (
            "path_corridor",
            "path_mse",
            "path_progress",
            "path_progress_floor",
            "path_centerline",
            "path_tangent",
        ):
            costs[name] = costs[name] * route_supervision_weight
        costs["safety"] = costs["safety"] + costs["path_corridor"]
        total_names = (
            "smooth", "safety", "frontier", "acceleration",
            "path_progress", "path_mse", "path_centerline", "path_tangent",
        )
        total_cost = torch.stack(tuple(costs[name] for name in total_names), dim=0).sum(dim=0)
        trajectory_loss = total_cost.mean()
        incompatible = route_active_expanded & ~route_compatible
        score_target_cost = total_cost.detach() + incompatible.to(total_cost.dtype) * float(
            cfg["route_incompatible_score_cost"]
        )
        score_loss = F.smooth_l1_loss(score_flat, score_target_cost)
        score_ranking_loss = self._score_ranking_loss(
            score_flat.view(batch_size, self.traj_num),
            score_target_cost.view(batch_size, self.traj_num),
        )
        # Candidate generation remains governed by the original YOPO score
        # regression.  Keep the old ranking metric for opt-in ablations, but
        # compute no ranking gradient in the default configuration.
        if float(cfg["safety_ranking_weight"]) > 0.0:
            safety_score_loss = self._safety_binary_ranking_loss(
                score_flat.view(batch_size, self.traj_num),
                costs["safety_barrier"].detach().view(batch_size, self.traj_num),
            )
        else:
            safety_score_loss = torch.zeros((), device=total_cost.device)
        total_loss = (
            trajectory_loss + score_loss
            + float(cfg["score_ranking_weight"]) * score_ranking_loss
            + float(cfg["safety_ranking_weight"]) * safety_score_loss
        )

        total_by_sample = score_target_cost.view(batch_size, self.traj_num)
        physical_by_sample = total_cost.view(batch_size, self.traj_num)
        score_by_sample = score_flat.view(batch_size, self.traj_num)
        selected_index = score_by_sample.argmin(dim=1)
        oracle_cost, oracle_index = total_by_sample.min(dim=1)
        selected_cost = torch.gather(total_by_sample, 1, selected_index[:, None]).squeeze(1)
        selected_physical_cost = torch.gather(
            physical_by_sample, 1, selected_index[:, None]
        ).squeeze(1)
        metrics = {
            "total_loss": float(total_loss.detach()),
            "trajectory_loss": float(trajectory_loss.detach()),
            "score_loss": float(score_loss.detach()),
            "score_ranking_loss": float(score_ranking_loss.detach()),
            "safety_score_loss": float(safety_score_loss.detach()),
            "selected_total_cost": float(selected_cost.mean().detach()),
            "oracle_total_cost": float(oracle_cost.mean().detach()),
            "selection_regret": float((selected_cost - oracle_cost).mean().detach()),
            "selected_physical_cost": float(selected_physical_cost.mean().detach()),
            "top1": float((selected_index == oracle_index).float().mean().detach()),
            "route_compatible_primitives": float(
                route_compatible.view(batch_size, self.traj_num).float().sum(dim=1).mean()
            ),
            "raw_route_altitude_error": float(
                raw_route_altitude_error[route_active_expanded].mean().detach()
            )
            if torch.any(route_active_expanded)
            else 0.0,
        }
        metrics.update({name: float(value.mean().detach()) for name, value in costs.items()})
        return total_loss, metrics

    @staticmethod
    def _safety_binary_ranking_loss(predicted: torch.Tensor, barrier: torch.Tensor) -> torch.Tensor:
        """Rank every collision candidate after every clearly safe candidate.

        Unlike ranking raw barrier magnitudes, this does not make the score
        head prefer the shortest low-clearance trajectory among safe options.
        The original detached total-cost regression remains responsible for
        progress and smoothness ordering.
        """
        unsafe = barrier > float(cfg["safety_ranking_target_margin"])
        margin = float(cfg["score_ranking_margin"])
        pair_losses: list[torch.Tensor] = []
        for left in range(predicted.shape[1]):
            for right in range(predicted.shape[1]):
                if left == right:
                    continue
                valid = unsafe[:, left] & ~unsafe[:, right]
                if torch.any(valid):
                    # score[unsafe] >= score[safe] + margin
                    pair_losses.append(
                        torch.relu(
                            margin - (predicted[valid, left] - predicted[valid, right])
                        )
                    )
        if not pair_losses:
            return predicted.new_zeros(())
        return torch.cat(pair_losses).mean()

    @staticmethod
    def _score_ranking_loss(predicted: torch.Tensor, target: torch.Tensor, *, target_margin: float | None = None) -> torch.Tensor:
        """Require candidate score ordering to agree with differentiable costs."""
        margin = float(cfg["score_ranking_margin"])
        target_margin = float(cfg["score_ranking_target_margin"]) if target_margin is None else float(target_margin)
        pair_losses: list[torch.Tensor] = []
        for left in range(predicted.shape[1]):
            for right in range(left + 1, predicted.shape[1]):
                delta = target[:, right] - target[:, left]
                valid = delta.abs() > target_margin
                if not torch.any(valid):
                    continue
                direction = torch.sign(delta[valid])
                predicted_delta = predicted[:, right] - predicted[:, left]
                pair_losses.append(torch.relu(margin - direction * predicted_delta[valid]))
        if not pair_losses:
            return predicted.new_zeros(())
        return torch.cat(pair_losses).mean()

    def _set_backbone_trainable(self, trainable: bool) -> None:
        if self.score_only:
            for parameter in self.policy.parameters():
                parameter.requires_grad_(False)
            final = self.policy.yopo_head.model[-1]
            final.weight.requires_grad_(True)
            final.bias.requires_grad_(True)
            return
        for parameter in self.policy.image_backbone.parameters():
            parameter.requires_grad_(trainable)

    def load_checkpoint(
        self, path: Path, *, resume_training_state: bool = True
    ) -> None:
        checkpoint = torch.load(path, map_location=self.device, weights_only=False)
        state_dict = checkpoint.get("model_state_dict", checkpoint) if isinstance(checkpoint, dict) else checkpoint
        try:
            if isinstance(checkpoint, dict) and "route_bubble_count" in checkpoint:
                applied_order = self.policy.load_route_checkpoint(checkpoint)
                print(f"feature order: {applied_order}")
            else:
                self.policy.load_state_dict(state_dict)
        except RuntimeError:
            self.policy.load_yopo_simple_state_dict(state_dict)
        if (
            resume_training_state
            and isinstance(checkpoint, dict)
            and "optimizer_state_dict" in checkpoint
        ):
            self._checkpoint_optimizer_state = checkpoint["optimizer_state_dict"]
            self.epoch_i = int(checkpoint.get("epoch", -1))
            self.best_validation_cost = float(
                checkpoint.get("best_validation_cost", float("inf"))
            )
        print(f"loaded checkpoint: {path}")

    def save_checkpoint(self, path: Path) -> None:
        torch.save(
            {
                "model_state_dict": self.policy.state_dict(),
                "optimizer_state_dict": self.optimizer.state_dict(),
                "epoch": self.epoch_i,
                "best_validation_cost": self.best_validation_cost,
                "route_dataset_version": int(cfg["route_dataset_version"]),
                "feature_order": self.policy.FEATURE_ORDER,
                "route_bubble_count": int(cfg["route_bubble_count"]),
                "route_anchor_distances_m": list(cfg["route_anchor_distances_m"]),
                "local_subgoal_distance_m": float(cfg["local_subgoal_distance_m"]),
                "fixed_route_altitude_projection": True,
                "route_incompatible_score_cost": float(
                    cfg["route_incompatible_score_cost"]
                ),
                "loss_weights": {
                    name: float(cfg[name])
                    for name in (
                        "ws", "wc", "wa", "wg", "wp", "wpath_mse", "wprogress",
                        "wprogress_floor", "wtangent", "wcenterline", "score_ranking_weight",
                        "score_ranking_margin", "score_ranking_target_margin",
                        "safety_ranking_weight", "safety_ranking_target_margin",
                        "route_progress_error_scale_m", "route_corridor_scale_m",
                    )
                },
                "peak_weights": {
                    "safety": float(cfg["safety_peak_weight"]),
                    "path_corridor": float(cfg["route_corridor_peak_weight"]),
                },
                "safety_collision_margin_weight": float(cfg["safety_collision_margin_weight"]),
                "safety_eval_points": int(cfg["safety_eval_points"]),
                "active_loss_terms": [
                    "smooth", "safety", "frontier", "acceleration",
                    "path_progress", "score_regression", "path_mse",
                    "path_centerline", "path_tangent",
                    "fixed_route_altitude_projection",
                    *(
                        ["score_ranking"]
                        if float(cfg["score_ranking_weight"]) > 0.0
                        else []
                    ),
                    *(
                        ["safety_ranking"]
                        if float(cfg["safety_ranking_weight"]) > 0.0
                        else []
                    ),
                ],
                "score_only": self.score_only,
            },
            path,
        )

    def _write_run_metadata(self) -> None:
        metadata = {
            "data_root": str(self.train_dataset.data_root),
            "split_strategy": self.train_dataset.split_strategy,
            "train_samples": len(self.train_dataset),
            "validation_samples": len(self.valid_dataset),
            "device": str(self.device),
            "batch_size": self.batch_size,
            "learning_rate": self.learning_rate,
            "num_workers": self.num_workers,
            "random_seed": self.random_seed,
            "route_dataset_version": int(cfg["route_dataset_version"]),
            "local_subgoal_distance_m": float(cfg["local_subgoal_distance_m"]),
            "fixed_route_altitude_projection": True,
            "route_incompatible_score_cost": float(
                cfg["route_incompatible_score_cost"]
            ),
            "loss_weights": {
                name: float(cfg[name])
                for name in (
                    "ws", "wc", "wa", "wg", "wp", "wpath_mse", "wprogress",
                    "wprogress_floor", "wtangent", "wcenterline", "score_ranking_weight",
                    "score_ranking_margin", "score_ranking_target_margin",
                    "safety_ranking_weight", "safety_ranking_target_margin",
                    "route_progress_error_scale_m", "route_corridor_scale_m",
                )
            },
            "checkpoint": self.checkpoint_path,
            "resume_training_state": self.resume_training_state,
            "peak_weights": {
                "safety": float(cfg["safety_peak_weight"]),
                "path_corridor": float(cfg["route_corridor_peak_weight"]),
            },
            "safety_collision_margin_weight": float(cfg["safety_collision_margin_weight"]),
            "safety_eval_points": int(cfg["safety_eval_points"]),
            "active_loss_terms": [
                "smooth", "safety", "frontier", "acceleration",
                "path_progress", "score_regression", "path_mse",
                "path_centerline", "path_tangent",
                "fixed_route_altitude_projection",
                *(
                    ["score_ranking"]
                    if float(cfg["score_ranking_weight"]) > 0.0
                    else []
                ),
                *(
                    ["safety_ranking"]
                    if float(cfg["safety_ranking_weight"]) > 0.0
                    else []
                ),
            ],
            "score_only": self.score_only,
        }
        (self.output_path / "run.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )

    def _update_run_metadata(self, values: dict[str, object]) -> None:
        path = self.output_path / "run.json"
        metadata = json.loads(path.read_text(encoding="utf-8"))
        metadata.update(values)
        path.write_text(json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8")

    def _log_metrics(self, prefix: str, metrics: dict[str, float], epoch: int) -> None:
        for name, value in metrics.items():
            self.tensorboard_log.add_scalar(f"{prefix}/{name}", value, epoch)

    @staticmethod
    def get_next_log_path(base_path: Path) -> Path:
        numbers = []
        for path in base_path.glob("YOPO_*"):
            if path.is_dir() and path.name.removeprefix("YOPO_").isdigit():
                numbers.append(int(path.name.removeprefix("YOPO_")))
        output = base_path / f"YOPO_{max(numbers, default=-1) + 1}"
        output.mkdir(parents=False, exist_ok=False)
        return output
