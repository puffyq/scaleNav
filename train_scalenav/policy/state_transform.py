import torch
import numpy as np
from config.config import cfg
from policy.primitive import LatticePrimitive


class StateTransform:
    def __init__(self):
        self.lattice_primitive = LatticePrimitive.get_instance()
        self.goal_length = cfg['goal_length']

    def pred_to_endstate(self, endstate_pred: torch.Tensor) -> torch.Tensor:
        """
            Transform the predicted state to the body frame (Original prediction → Primitive frame → Body frame).
            endstate_pred: [batch; px py pz vx vy vz ax ay az; primitive_v; primitive_h]
            :return [batch; px py pz vx vy vz ax ay az; primitive_v; primitive_h] in body frame
        """
        B, V, H = endstate_pred.shape[0], endstate_pred.shape[2], endstate_pred.shape[3]

        # [B, 9, 3, 5] -> [B, 3, 5, 9] -> [B, 15, 9]
        endstate_pred = endstate_pred.permute(0, 2, 3, 1).reshape(B, V * H, 9)

        # 获取 lattice angle 和 rotation (.flip: 由于lattice和grid的顺序相反)
        yaw, pitch = self.lattice_primitive.getAngleLattice()  # [15]
        yaw = yaw.to(device=endstate_pred.device, dtype=endstate_pred.dtype)
        pitch = pitch.to(device=endstate_pred.device, dtype=endstate_pred.dtype)
        yaw = yaw.flip(0)[None, :].expand(B, -1)  # [B, 15]
        pitch = pitch.flip(0)[None, :].expand(B, -1)  # [B, 15]
        Rbp = self.lattice_primitive.getRotation().to(
            device=endstate_pred.device, dtype=endstate_pred.dtype
        ).flip(0)  # [15, 3, 3]
        Rbp = Rbp[None, :, :, :].expand(B, -1, -1, -1)  # [B, 15, 3, 3]

        delta_yaw = endstate_pred[:, :, 0] * self.lattice_primitive.yaw_diff  # [B, 15]
        delta_pitch = endstate_pred[:, :, 1] * self.lattice_primitive.pitch_diff
        radio = (endstate_pred[:, :, 2] + 1.0) * self.lattice_primitive.radio_range

        cos_pitch = torch.cos(pitch + delta_pitch)
        endstate_x = cos_pitch * torch.cos(yaw + delta_yaw) * radio
        endstate_y = cos_pitch * torch.sin(yaw + delta_yaw) * radio
        endstate_z = torch.sin(pitch + delta_pitch) * radio
        endstate_p = torch.stack([endstate_x, endstate_y, endstate_z], dim=-1)  # [B, 15, 3]

        # vel / acc
        endstate_vp = endstate_pred[:, :, 3:6] * self.lattice_primitive.vel_max  # [B, 15, 3]
        endstate_ap = endstate_pred[:, :, 6:9] * self.lattice_primitive.acc_max  # [B, 15, 3]

        # v/a 变换到 body frame
        endstate_vb = torch.matmul(Rbp, endstate_vp.unsqueeze(-1)).squeeze(-1)  # [B, 15, 3]
        endstate_ab = torch.matmul(Rbp, endstate_ap.unsqueeze(-1)).squeeze(-1)

        endstate = torch.cat([endstate_p, endstate_vb, endstate_ab], dim=-1)  # [B, 15, 9]

        endstate = endstate.permute(0, 2, 1).reshape(B, 9, V, H)  # [B, 9, 3, 5]
        return endstate

    def route_primitive_compatibility(self, frontier_body: torch.Tensor) -> torch.Tensor:
        """Return the image-grid primitives whose angular sector covers the Route goal."""
        if frontier_body.ndim != 2 or frontier_body.shape[1] != 3:
            raise ValueError("frontier_body must have shape [B, 3]")
        horizontal = torch.linalg.vector_norm(frontier_body[:, :2], dim=1)
        yaw_goal = torch.atan2(frontier_body[:, 1], frontier_body[:, 0])
        pitch_goal = torch.atan2(frontier_body[:, 2], horizontal)
        yaw, pitch = self.lattice_primitive.getAngleLattice()
        yaw = yaw.to(frontier_body).flip(0)[None, :]
        pitch = pitch.to(frontier_body).flip(0)[None, :]
        yaw_error = torch.atan2(
            torch.sin(yaw_goal[:, None] - yaw),
            torch.cos(yaw_goal[:, None] - yaw),
        ).abs()
        pitch_error = (pitch_goal[:, None] - pitch).abs()
        compatible = (yaw_error <= self.lattice_primitive.yaw_diff) & (
            pitch_error <= self.lattice_primitive.pitch_diff
        )
        # Keep supervision defined for a goal just outside the configured FOV.
        missing = ~compatible.any(dim=1)
        if torch.any(missing):
            normalized_error = (
                yaw_error / max(float(self.lattice_primitive.yaw_diff), 1.0e-6)
                + pitch_error / max(float(self.lattice_primitive.pitch_diff), 1.0e-6)
            )
            nearest = normalized_error.argmin(dim=1)
            compatible = compatible.clone()
            compatible[missing, nearest[missing]] = True
        return compatible.view(
            frontier_body.shape[0],
            self.lattice_primitive.vertical_num,
            self.lattice_primitive.horizon_num,
        )

    def pred_to_endstate_cpu(self, endstate_pred: np.ndarray, lattice_id: torch.Tensor) -> np.ndarray:
        """
            Used during test:
            Numpy version of pred_to_endstate() on CPU (used in test, x10 times faster than torch on CUDA)
            :return [B; px py pz vx vy vz ax ay az] in body frame
        """
        delta_yaw = endstate_pred[:, 0] * self.lattice_primitive.yaw_diff
        delta_pitch = endstate_pred[:, 1] * self.lattice_primitive.pitch_diff
        radio = (endstate_pred[:, 2] + 1.0) * self.lattice_primitive.radio_range

        yaw, pitch = self.lattice_primitive.getAngleLattice(lattice_id)
        yaw, pitch = yaw.cpu().numpy(), pitch.cpu().numpy()
        endstate_x = np.cos(pitch + delta_pitch) * np.cos(yaw + delta_yaw) * radio
        endstate_y = np.cos(pitch + delta_pitch) * np.sin(yaw + delta_yaw) * radio
        endstate_z = np.sin(pitch + delta_pitch) * radio
        endstate_p = np.stack((endstate_x, endstate_y, endstate_z), axis=1)

        endstate_vp = endstate_pred[:, 3:6] * self.lattice_primitive.vel_max
        endstate_ap = endstate_pred[:, 6:9] * self.lattice_primitive.acc_max

        Rpb = self.lattice_primitive.getRotation(lattice_id).cpu().numpy()
        endstate_vb = np.matmul(Rpb, endstate_vp[:, :, np.newaxis]).squeeze(-1)
        endstate_ab = np.matmul(Rpb, endstate_ap[:, :, np.newaxis]).squeeze(-1)

        return np.concatenate((endstate_p, endstate_vb, endstate_ab), axis=1)


    def prepare_input(self, obs):
        """
            Transform the observation to the primitive frame (Body frame → Primitive frame → Body frame).
            obs contains one or more 3-D body-frame vectors. Original YOPO uses
            velocity, acceleration, and goal; heatmap-guided YOPO uses only
            velocity and acceleration.
        """
        B, N = obs.shape[0], self.lattice_primitive.traj_num
        if obs.shape[1] % 3:
            raise ValueError("observation dimension must be divisible by 3")
        vector_count = obs.shape[1] // 3

        # 获取所有 Rbp 并倒序排列 (由于lattice和grid的顺序相反)
        Rbp_all = self.lattice_primitive.getRotation().to(
            device=obs.device, dtype=obs.dtype
        ).flip(0)  # shape: [N, 3, 3]

        obs = obs.view(B, vector_count, 3)

        obs_exp = obs[:, None, :, :].expand(B, N, vector_count, 3)
        Rbp_exp = Rbp_all[None, :, :, :].expand(B, N, 3, 3)

        # 执行批量坐标变换
        transformed = torch.matmul(obs_exp, Rbp_exp)

        feature_count = vector_count * 3
        transformed_flat = transformed.view(B, N, feature_count)
        out = transformed_flat.permute(0, 2, 1).contiguous()
        out = out.view(
            B,
            feature_count,
            self.lattice_primitive.vertical_num,
            self.lattice_primitive.horizon_num,
        )
        return out

    def prepare_motion_input(self, motion):
        """Transform [velocity, acceleration] for heatmap-guided YOPO."""
        batch = motion.shape[0]
        trajectory_count = self.lattice_primitive.traj_num
        rotations = self.lattice_primitive.getRotation().to(
            device=motion.device, dtype=motion.dtype
        ).flip(0)
        vectors = motion.view(batch, 2, 3)
        vectors = vectors[:, None].expand(batch, trajectory_count, 2, 3)
        rotations = rotations[None].expand(batch, trajectory_count, 3, 3)
        transformed = torch.matmul(vectors, rotations)
        transformed = transformed.view(batch, trajectory_count, 6)
        transformed = transformed.permute(0, 2, 1).contiguous()
        return transformed.view(
            batch,
            6,
            self.lattice_primitive.vertical_num,
            self.lattice_primitive.horizon_num,
        )

    def prepare_route_input(self, route_bubbles):
        """Transform K normalized body-FLU corridor bubbles per primitive."""
        if route_bubbles.ndim != 3 or route_bubbles.shape[-1] != 4:
            raise ValueError("route_bubbles must have shape [B, K, 4]")
        batch, bubble_count, _ = route_bubbles.shape
        trajectory_count = self.lattice_primitive.traj_num
        rotations = self.lattice_primitive.getRotation().to(
            device=route_bubbles.device, dtype=route_bubbles.dtype
        ).flip(0)
        centers = route_bubbles[:, None, :, :3].expand(
            batch, trajectory_count, bubble_count, 3
        )
        rotations = rotations[None, :, :, :].expand(
            batch, trajectory_count, 3, 3
        )
        transformed = torch.matmul(centers, rotations)
        radius = route_bubbles[:, None, :, 3:4].expand(
            batch, trajectory_count, bubble_count, 1
        )
        features = torch.cat((transformed, radius), dim=-1)
        features = features.reshape(batch, trajectory_count, bubble_count * 4)
        features = features.permute(0, 2, 1).contiguous()
        return features.view(
            batch,
            bubble_count * 4,
            self.lattice_primitive.vertical_num,
            self.lattice_primitive.horizon_num,
        )

    def unnormalize_obs(self, vel_acc):
        vel_acc[:, 0:3] = vel_acc[:, 0:3] * self.lattice_primitive.vel_max
        vel_acc[:, 3:6] = vel_acc[:, 3:6] * self.lattice_primitive.acc_max
        return vel_acc

    def normalize_motion(self, motion):
        motion[:, 0:3] = motion[:, 0:3] / self.lattice_primitive.vel_max
        motion[:, 3:6] = motion[:, 3:6] / self.lattice_primitive.acc_max
        return motion

    def normalize_obs(self, vel_acc_goal):
        if vel_acc_goal.shape[1] < 6:
            raise ValueError("observation must contain velocity and acceleration")
        vel_acc_goal[:, 0:3] = vel_acc_goal[:, 0:3] / self.lattice_primitive.vel_max
        vel_acc_goal[:, 3:6] = vel_acc_goal[:, 3:6] / self.lattice_primitive.acc_max

        if vel_acc_goal.shape[1] >= 9:
            goal_norm = vel_acc_goal[:, 6:9].norm(dim=1, keepdim=True)
            vel_acc_goal[:, 6:9] = vel_acc_goal[:, 6:9] / goal_norm.clamp(min=self.goal_length)
        return vel_acc_goal


def rotate_body2world(rot_wb, pos_b):
    """
    Rotate pos_b from body frame to world frame using quaternion q_wb.
    rot_wb: (..., 3, 3)
    pos_b: (..., 3)
    """
    pos_w = torch.matmul(rot_wb, pos_b.unsqueeze(-1)).squeeze(-1)
    return pos_w


def transform_body2world(rot_wb, t_w, pos_b):
    """
    Transform pos_b from body frame to world frame using quaternion q_wb and t_w.
    rot_wb: (..., 3, 3)
    t_w: (..., 3)
    pos_b: (..., 3)
    """
    return rotate_body2world(rot_wb, pos_b) + t_w


def state_body2world(pos_w, rot_wb, pos_b, vel_b, acc_b):
    pos_b = transform_body2world(rot_wb, pos_w, pos_b)
    vel_b = rotate_body2world(rot_wb, vel_b)
    acc_b = rotate_body2world(rot_wb, acc_b)
    return pos_b, vel_b, acc_b


def project_world_endstate_to_altitude(
    position_world: torch.Tensor,
    velocity_world: torch.Tensor,
    acceleration_world: torch.Tensor,
    altitude_world: torch.Tensor,
    active: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Project Route-mode terminal state onto a fixed world-frame altitude."""
    if (
        position_world.shape != velocity_world.shape
        or position_world.shape != acceleration_world.shape
        or position_world.ndim != 2
        or position_world.shape[1] != 3
    ):
        raise ValueError("world end states must all have shape [B, 3]")
    if altitude_world.shape != position_world.shape[:1] or active.shape != altitude_world.shape:
        raise ValueError("altitude and active mask must have shape [B]")
    active = active.to(dtype=torch.bool)
    position = position_world.clone()
    velocity = velocity_world.clone()
    acceleration = acceleration_world.clone()
    position[:, 2] = torch.where(active, altitude_world, position_world[:, 2])
    velocity[:, 2] = torch.where(active, torch.zeros_like(velocity_world[:, 2]), velocity_world[:, 2])
    acceleration[:, 2] = torch.where(
        active,
        torch.zeros_like(acceleration_world[:, 2]),
        acceleration_world[:, 2],
    )
    return position, velocity, acceleration


if __name__ == '__main__':
    CoordTransform = StateTransform()
