from __future__ import annotations

import math
from typing import Sequence

import numpy as np


NED_TO_ENU = np.array(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
    dtype=np.float64,
)
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0]).astype(np.float64)


def quaternion_wxyz_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    values = np.asarray(quaternion, dtype=np.float64)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("quaternion must contain four finite WXYZ values")
    norm = float(np.linalg.norm(values))
    if norm < 1.0e-8:
        raise ValueError("quaternion must not be zero")
    w, x, y, z = values / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array([w, x, y, z], dtype=np.float64)
    if quaternion[0] < 0.0:
        quaternion = -quaternion
    return quaternion / np.linalg.norm(quaternion)


def ned_to_enu(points: np.ndarray | Sequence[float]) -> np.ndarray:
    array = np.asarray(points, dtype=np.float64)
    if array.shape[-1:] != (3,) or not np.isfinite(array).all():
        raise ValueError("points must end in three finite NED coordinates")
    return (array @ NED_TO_ENU.T).astype(np.float32)


def enu_to_ned(points: np.ndarray | Sequence[float]) -> np.ndarray:
    return ned_to_enu(points)


def ned_frd_pose_to_enu_flu(
    position_ned: Sequence[float], quaternion_ned_frd_wxyz: Sequence[float]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    position_enu = ned_to_enu(position_ned)
    rotation_ned_frd = quaternion_wxyz_to_matrix(quaternion_ned_frd_wxyz)
    rotation_enu_flu = NED_TO_ENU @ rotation_ned_frd @ FLU_TO_FRD
    quaternion_enu_flu = matrix_to_quaternion_wxyz(rotation_enu_flu).astype(np.float32)
    return position_enu, quaternion_enu_flu, rotation_enu_flu.astype(np.float32)


def world_to_body_flu(
    points_world_enu: np.ndarray, position_world_enu: np.ndarray, rotation_enu_flu: np.ndarray
) -> np.ndarray:
    points = np.asarray(points_world_enu, dtype=np.float32)
    position = np.asarray(position_world_enu, dtype=np.float32)
    rotation = np.asarray(rotation_enu_flu, dtype=np.float32)
    if points.shape[-1:] != (3,) or position.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("invalid world/body transform shapes")
    return (points - position) @ rotation
