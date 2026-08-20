from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from tools.visualize_text_yopo_test import depth_point_cloud


class DepthPointCloudTests(unittest.TestCase):
    def test_depth_planar_is_projected_to_body_flu(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "depth.exr"
            depth = np.full((3, 5), 2.0, dtype=np.float32)
            self.assertTrue(cv2.imwrite(str(path), depth))
            points = depth_point_cloud(path, 90.0, 60.0, 10.0, 100)

        center = points[np.argmin(np.linalg.norm(points - [2.0, 0.0, 0.0], axis=1))]
        np.testing.assert_allclose(center, [2.0, 0.0, 0.0], atol=1e-5)
        self.assertLess(float(points[:, 1].min()), 0.0)  # camera right -> body left sign
        self.assertGreater(float(points[:, 2].max()), 0.0)  # camera down -> body up sign


if __name__ == "__main__":
    unittest.main()
