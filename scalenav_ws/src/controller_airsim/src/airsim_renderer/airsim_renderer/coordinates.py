import math

import numpy as np


def quaternion_to_matrix(x, y, z, w):
    quaternion = np.asarray([w, x, y, z], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("odometry orientation is not a finite quaternion")
    w, x, y, z = quaternion / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([w, x, y, z], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def ros_pose_to_airsim(pose, airsim_origin_enu):
    position_enu = np.asarray(
        [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
    )
    origin_enu = np.asarray(airsim_origin_enu, dtype=np.float64)
    if not np.all(np.isfinite(position_enu)) or not np.all(np.isfinite(origin_enu)):
        raise ValueError("odometry position or AirSim origin is not finite")

    ned_from_enu = np.asarray(
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
    )
    flu_from_frd = np.diag([1.0, -1.0, -1.0])
    rotation_enu_flu = quaternion_to_matrix(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )

    position_ned = ned_from_enu @ (position_enu - origin_enu)
    rotation_ned_frd = ned_from_enu @ rotation_enu_flu @ flu_from_frd
    w, x, y, z = matrix_to_quaternion(rotation_ned_frd)

    # Colosseum's MessagePack adaptors encode Vector3r and Quaternionr as arrays.
    return [position_ned.astype(float).tolist(), [float(w), float(x), float(y), float(z)]]
