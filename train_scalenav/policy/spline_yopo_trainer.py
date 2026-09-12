from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch
from torch.nn import functional as F
from torch.utils.data import DataLoader
from torch.utils.tensorboard.writer import SummaryWriter

from config.config import cfg
from loss.guidance_loss import GuidanceLoss
from loss.route_loss import RouteLoss
from loss.safety_loss import SafetyLoss
from loss.loss_function import YOPOLoss
from policy.spline_trajectory import ClampedCubicSpline
from policy.spline_yopo_network import SplineYopoNetwork
from policy.state_transform import rotate_body2world
from policy.yopo_dataset import YOPODataset


class SplineYopoTrainer:
    def __init__(
        self,
        *,
        data_root: str | Path,
        output_root: str | Path,
        checkpoint_path: str | Path,
        learning_rate: float,
        batch_size: int,
        num_workers: int,
        device: str,
        random_seed: int,
        control_point_count: int = 12,
        route_only: bool = False,
        score_only: bool = False,
        bubble_radius_weight: float = 0.0,
    ) -> None:
        self.device = torch.device(device)
        self.batch_size = int(batch_size)
        self.traj_num = int(cfg["traj_num"])
        self.control_point_count = int(control_point_count)
        self.route_only = bool(route_only)
        self.score_only = bool(score_only)
        self.bubble_radius_weight = float(bubble_radius_weight)
        self.max_grad_norm = 0.1
        self.train_dataset = YOPODataset("train", data_root=data_root, seed=random_seed)
        self.valid_dataset = YOPODataset("valid", data_root=data_root, seed=random_seed)
        options = {
            "batch_size": self.batch_size,
            "num_workers": int(num_workers),
            "pin_memory": self.device.type == "cuda",
        }
        self.train_loader = DataLoader(
            self.train_dataset, shuffle=True, drop_last=False, **options
        )
        self.valid_loader = DataLoader(
            self.valid_dataset, shuffle=False, drop_last=False, **options
        )
        self.policy = SplineYopoNetwork(
            control_point_count=self.control_point_count
        ).to(self.device)
        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        self.policy.load_training_checkpoint(checkpoint)

        coefficient_map = YOPOLoss.qp_generation(self._coefficient_source())[2]
        self.route_loss = RouteLoss(
            coefficient_map.to(self.device),
            bubble_radius_weight=self.bubble_radius_weight,
        ).to(self.device)
        self.safety_loss = None
        self.guidance_loss = None
        if not self.route_only:
            self.safety_loss = SafetyLoss(
                coefficient_map.to(self.device), self.train_dataset.obstacle_paths
            ).to(self.device)
            self.guidance_loss = GuidanceLoss().to(self.device)
        self.spline = ClampedCubicSpline(
            control_point_count=self.control_point_count,
            duration=float(cfg["sgm_time"]),
            sample_count=30,
        ).to(self.device)
        optimizer_parameters = list(self.policy.parameters())
        if self.score_only:
            for parameter in self.policy.parameters():
                parameter.requires_grad_(False)
            final = self.policy.yopo_head.model[4]
            final.weight.requires_grad_(True)
            final.bias.requires_grad_(True)
            weight_mask = torch.zeros_like(final.weight)
            bias_mask = torch.zeros_like(final.bias)
            weight_mask[-1] = 1.0
            bias_mask[-1] = 1.0
            final.weight.register_hook(lambda gradient: gradient * weight_mask)
            final.bias.register_hook(lambda gradient: gradient * bias_mask)
            optimizer_parameters = [final.weight, final.bias]
        self.optimizer = torch.optim.AdamW(
            optimizer_parameters,
            lr=float(learning_rate),
            fused=self.device.type == "cuda",
        )
        output_root = Path(output_root)
        output_root.mkdir(parents=True, exist_ok=True)
        self.output_path = self._next_output(output_root)
        self.writer = SummaryWriter(log_dir=self.output_path)
        self.best_validation_cost = float("inf")
        self._write_metadata(
            checkpoint_path=checkpoint_path,
            learning_rate=learning_rate,
            num_workers=num_workers,
            random_seed=random_seed,
            route_only=self.route_only,
            score_only=self.score_only,
            bubble_radius_weight=self.bubble_radius_weight,
        )

    @staticmethod
    def _coefficient_source():
        source = object.__new__(YOPOLoss)
        source.sgm_time = float(cfg["sgm_time"])
        return source

    @staticmethod
    def _next_output(root: Path) -> Path:
        indices = [
            int(path.name.removeprefix("YOPO_"))
            for path in root.glob("YOPO_*")
            if path.is_dir() and path.name.removeprefix("YOPO_").isdigit()
        ]
        path = root / f"YOPO_{max(indices, default=-1) + 1}"
        path.mkdir()
        return path

    def _write_metadata(self, **values) -> None:
        metadata = {
            "representation": "clamped_cubic_b_spline",
            "control_point_count": self.control_point_count,
            "fixed_initial_derivatives": ["position", "velocity", "acceleration"],
            "train_samples": len(self.train_dataset),
            "validation_samples": len(self.valid_dataset),
            "batch_size": self.batch_size,
            "loss_weights": {
                key: float(cfg[key]) for key in ("ws", "wc", "wa", "wg", "wp")
            },
            "score_target_normalizer": max(float(cfg["wp"]), 1.0),
            **{key: str(value) if isinstance(value, Path) else value for key, value in values.items()},
        }
        (self.output_path / "run.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )

    def _sample_trajectories(
        self, batch: dict[str, torch.Tensor]
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        free_body, score = self.policy(
            batch["depth"],
            batch["motion_body"],
            batch["frontier_body"],
            batch["route_bubbles"],
        )
        batch_size = free_body.shape[0]
        start_position_body = torch.zeros(
            (batch_size, self.traj_num, 3), device=self.device, dtype=free_body.dtype
        )
        start_velocity_body = batch["motion_body"][:, None, :3].expand(
            -1, self.traj_num, -1
        )
        start_acceleration_body = batch["motion_body"][:, None, 3:6].expand(
            -1, self.traj_num, -1
        )
        controls_body = self.spline.assemble_controls(
            start_position_body,
            start_velocity_body,
            start_acceleration_body,
            free_body,
        )
        position_body, velocity_body, acceleration_body, jerk_body = self.spline(
            controls_body
        )
        rotation = batch["rotation_world_body"][:, None, None]
        position_world = batch["position_world"][:, None, None] + torch.matmul(
            rotation, position_body.unsqueeze(-1)
        ).squeeze(-1)
        velocity_world = torch.matmul(rotation, velocity_body.unsqueeze(-1)).squeeze(-1)
        acceleration_world = torch.matmul(rotation, acceleration_body.unsqueeze(-1)).squeeze(-1)
        jerk_world = torch.matmul(rotation, jerk_body.unsqueeze(-1)).squeeze(-1)
        return position_world, velocity_world, acceleration_world, jerk_world, score

    def forward_and_loss(
        self, batch: dict[str, torch.Tensor]
    ) -> tuple[torch.Tensor, dict[str, float]]:
        positions, velocities, accelerations, jerks, score = self._sample_trajectories(batch)
        batch_size, trajectory_count, sample_count, _ = positions.shape
        flat_count = batch_size * trajectory_count
        flat_positions = positions.reshape(flat_count, sample_count, 3)
        start_position = batch["position_world"].repeat_interleave(trajectory_count, dim=0)
        route_points = batch["route_points_world"].repeat_interleave(trajectory_count, dim=0)
        route_radii = batch["route_radii_world"].repeat_interleave(
            trajectory_count, dim=0
        )
        centerline_cost, bubble_cost = self.route_loss.forward_positions_components(
            flat_positions,
            start_position,
            route_points,
            route_radii=route_radii,
        )
        centerline_cost = centerline_cost.reshape(batch_size, trajectory_count)
        bubble_cost = bubble_cost.reshape(batch_size, trajectory_count)
        route_cost = centerline_cost + self.bubble_radius_weight * bubble_cost

        smooth_cost = jerks.square().sum(dim=-1).mean(dim=-1) * float(cfg["sgm_time"])
        acceleration_cost = accelerations.square().sum(dim=-1).mean(dim=-1) * float(cfg["sgm_time"])
        safety_cost = torch.zeros_like(route_cost)
        frontier_cost = torch.zeros_like(route_cost)
        base_cost = torch.zeros_like(route_cost)
        if not self.route_only:
            assert self.safety_loss is not None and self.guidance_loss is not None
            distance_cost, _ = self.safety_loss.get_distance_cost(
                positions.reshape(batch_size, trajectory_count * sample_count, 3),
                batch["map_id"],
            )
            safety_cost = distance_cost.reshape(
                batch_size, trajectory_count, sample_count
            ).mean(2)
            start_velocity = rotate_body2world(
                batch["rotation_world_body"], batch["motion_body"][:, :3]
            ).repeat_interleave(trajectory_count, dim=0)
            start_acceleration = rotate_body2world(
                batch["rotation_world_body"], batch["motion_body"][:, 3:]
            ).repeat_interleave(trajectory_count, dim=0)
            fixed = torch.stack((start_position, start_velocity, start_acceleration), dim=2)
            decision = torch.stack(
                (
                    positions[:, :, -1].reshape(flat_count, 3),
                    velocities[:, :, -1].reshape(flat_count, 3),
                    accelerations[:, :, -1].reshape(flat_count, 3),
                ),
                dim=2,
            )
            frontier = batch["frontier_world"].repeat_interleave(
                trajectory_count, dim=0
            )
            frontier_cost = self.guidance_loss(fixed, decision, frontier).reshape(
                batch_size, trajectory_count
            )
            smooth_weight = float(cfg["ws"]) / float(cfg["vel_max_train"]) ** 5
            acceleration_weight = float(cfg["wa"]) / float(cfg["vel_max_train"]) ** 3
            base_cost = (
                smooth_weight * smooth_cost
                + float(cfg["wc"]) * safety_cost
                + float(cfg["wg"]) * frontier_cost
                + acceleration_weight * acceleration_cost
            )
        route_weight = float(cfg["wp"])
        trajectory_loss = base_cost.mean() + route_weight * route_cost.min(dim=1).values.mean()
        score_target_normalizer = max(route_weight, 1.0)
        score_target = (
            (base_cost + route_weight * route_cost) / score_target_normalizer
        ).detach()
        score_loss = F.smooth_l1_loss(score.reshape_as(score_target), score_target)
        total_loss = trajectory_loss + score_loss
        selected = score.reshape_as(score_target).argmin(dim=1)
        selected_route = torch.gather(route_cost, 1, selected[:, None]).squeeze(1)
        oracle_route = route_cost.min(dim=1).values
        metrics = {
            "total_loss": float(total_loss.detach()),
            "trajectory_loss": float(trajectory_loss.detach()),
            "score_loss": float(score_loss.detach()),
            "route_fit_selected": float(selected_route.mean().detach()),
            "route_fit_oracle": float(oracle_route.mean().detach()),
            "route_centerline": float(centerline_cost.mean().detach()),
            "route_bubble_outside": float(bubble_cost.mean().detach()),
            "safety": float(safety_cost.mean().detach()),
            "smooth": float(smooth_cost.mean().detach()),
            "acceleration": float(acceleration_cost.mean().detach()),
            "frontier": float(frontier_cost.mean().detach()),
            "weighted_smooth": float((smooth_weight * smooth_cost).mean().detach())
            if not self.route_only
            else 0.0,
            "weighted_safety": float(
                (float(cfg["wc"]) * safety_cost).mean().detach()
            ),
            "weighted_acceleration": float(
                (acceleration_weight * acceleration_cost).mean().detach()
            )
            if not self.route_only
            else 0.0,
            "weighted_frontier": float(
                (float(cfg["wg"]) * frontier_cost).mean().detach()
            ),
            "weighted_route_oracle": float(
                (route_weight * oracle_route).mean().detach()
            ),
        }
        return total_loss, metrics

    def run_epoch(self, loader: DataLoader, *, training: bool) -> dict[str, float]:
        self.policy.train(training)
        values = []
        context = torch.enable_grad() if training else torch.inference_mode()
        with context:
            for batch in loader:
                batch = {key: value.to(self.device, non_blocking=True) for key, value in batch.items()}
                if training:
                    self.optimizer.zero_grad(set_to_none=True)
                loss, metrics = self.forward_and_loss(batch)
                if training:
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                    self.optimizer.step()
                values.append(metrics)
        return {key: float(np.mean([item[key] for item in values])) for key in values[0]}

    def train(self, epochs: int, *, save_interval: int = 3) -> None:
        for epoch in range(int(epochs)):
            train_metrics = self.run_epoch(self.train_loader, training=True)
            valid_metrics = self.run_epoch(self.valid_loader, training=False)
            for name, value in train_metrics.items():
                self.writer.add_scalar(f"Train/{name}", value, epoch)
            for name, value in valid_metrics.items():
                self.writer.add_scalar(f"Validation/{name}", value, epoch)
            validation_cost = valid_metrics["route_fit_selected"]
            if validation_cost < self.best_validation_cost:
                self.best_validation_cost = validation_cost
                self.save(self.output_path / "best.pth", epoch)
            if (epoch + 1) % int(save_interval) == 0:
                self.save(self.output_path / f"epoch{epoch + 1}.pth", epoch)
            print(
                f"epoch={epoch + 1} train={train_metrics['total_loss']:.5f} "
                f"valid={valid_metrics['total_loss']:.5f} "
                f"route_selected={validation_cost:.5f} "
                f"route_oracle={valid_metrics['route_fit_oracle']:.5f}",
                flush=True,
            )
        self.save(self.output_path / "last.pth", int(epochs) - 1)

    def save(self, path: Path, epoch: int) -> None:
        torch.save(
            {
                "model_state_dict": self.policy.state_dict(),
                "optimizer_state_dict": self.optimizer.state_dict(),
                "epoch": int(epoch),
                "feature_order": self.policy.FEATURE_ORDER,
                "control_point_count": self.control_point_count,
                "loss_weights": {
                    key: float(cfg[key]) for key in ("ws", "wc", "wa", "wg", "wp")
                },
                "score_target_normalizer": max(float(cfg["wp"]), 1.0),
                "bubble_radius_weight": self.bubble_radius_weight,
            },
            path,
        )
