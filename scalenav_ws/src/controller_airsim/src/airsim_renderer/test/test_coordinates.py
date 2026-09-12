import math
import unittest
from types import SimpleNamespace

import numpy as np

from airsim_renderer.coordinates import ros_pose_to_airsim


def pose(position, yaw):
    return SimpleNamespace(
        position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
        orientation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
        ),
    )


class CoordinatesTest(unittest.TestCase):
    def test_enu_position_becomes_ned(self):
        result = ros_pose_to_airsim(pose((1.0, 2.0, 3.0), math.pi / 2.0), [0, 0, 0])
        np.testing.assert_allclose(result[0], [2.0, 1.0, -3.0], atol=1e-7)

    def test_ros_north_heading_is_airsim_zero_yaw(self):
        result = ros_pose_to_airsim(pose((0.0, 0.0, 0.0), math.pi / 2.0), [0, 0, 0])
        self.assertAlmostEqual(abs(result[1][0]), 1.0, places=7)
        np.testing.assert_allclose(result[1][1:], [0.0, 0.0, 0.0], atol=1e-7)

    def test_origin_offset_is_applied_first(self):
        result = ros_pose_to_airsim(
            pose((11.0, 22.0, 33.0), math.pi / 2.0), [10.0, 20.0, 30.0]
        )
        np.testing.assert_allclose(result[0], [2.0, 1.0, -3.0], atol=1e-7)


if __name__ == "__main__":
    unittest.main()
