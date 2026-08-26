import numpy as np

from data.coordinates import (
    ned_frd_pose_to_enu_flu,
    ned_to_enu,
    quaternion_wxyz_to_matrix,
    world_to_body_flu,
)


def test_ned_to_enu_axes():
    points = ned_to_enu(np.eye(3, dtype=np.float32))
    np.testing.assert_allclose(points, [[0, 1, 0], [1, 0, 0], [0, 0, -1]])


def test_identity_ned_frd_pose_maps_flu_axes_correctly():
    position, quaternion, rotation = ned_frd_pose_to_enu_flu(
        [10.0, 20.0, -3.0], [1.0, 0.0, 0.0, 0.0]
    )
    np.testing.assert_allclose(position, [20.0, 10.0, 3.0])
    np.testing.assert_allclose(quaternion_wxyz_to_matrix(quaternion), rotation, atol=1e-6)
    np.testing.assert_allclose(rotation[:, 0], [0.0, 1.0, 0.0])
    np.testing.assert_allclose(rotation[:, 1], [-1.0, 0.0, 0.0])
    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0])


def test_world_to_body_round_trip():
    position, _, rotation = ned_frd_pose_to_enu_flu([1, 2, -3], [1, 0, 0, 0])
    body = np.array([[2.0, -1.0, 0.5]], dtype=np.float32)
    world = body @ rotation.T + position
    np.testing.assert_allclose(world_to_body_flu(world, position, rotation), body, atol=1e-6)
