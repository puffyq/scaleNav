import unittest

import numpy as np

from scalenav.ds_camera import (
    DoubleSphereIntrinsics,
    decode_recorded_depth,
    depth_to_original_camera_points,
    double_sphere_unproject_grid,
    make_perspective_to_ds_map,
    recorded_depth_to_perspective,
)


class DoubleSphereRemapTest(unittest.TestCase):
    def test_perspective_center_maps_to_ds_principal_point(self):
        source = DoubleSphereIntrinsics(200.0, 201.0, 300.0, 301.0, 0.3, 0.6, 640, 640)
        map_x, map_y, _ = make_perspective_to_ds_map(source, 5, 5, 90.0, 60.0)
        self.assertAlmostEqual(float(map_x[2, 2]), source.cx, places=5)
        self.assertAlmostEqual(float(map_y[2, 2]), source.cy, places=5)

    def test_maps_are_finite_and_monotonic(self):
        source = DoubleSphereIntrinsics(90.8, 90.9, 253.4, 259.2, -0.31, 0.564, 512, 512)
        map_x, map_y, pinhole = make_perspective_to_ds_map(source, 160, 96)
        self.assertTrue(np.isfinite(map_x).all())
        self.assertTrue(np.isfinite(map_y).all())
        self.assertTrue(np.all(np.diff(map_x[48]) > 0.0))
        self.assertTrue(np.all(np.diff(map_y[:, 80]) > 0.0))
        self.assertGreater(pinhole[0], 0.0)

    def test_recorded_inverse_depth_endpoints(self):
        decoded = decode_recorded_depth(np.asarray([[0, 255]], dtype=np.uint8))
        self.assertAlmostEqual(float(decoded[0, 0]), 50.0, places=4)
        self.assertAlmostEqual(float(decoded[0, 1]), 0.5500, places=3)

    def test_center_ray_and_source_coordinate_order(self):
        source = DoubleSphereIntrinsics(90.0, 90.0, 2.0, 2.0, -0.31, 0.564, 5, 5)
        rays, valid = double_sphere_unproject_grid(source, minimum_elevation_deg=-90.0)
        np.testing.assert_allclose(rays[2, 2], [0.0, 0.0, 1.0], atol=1e-6)
        self.assertTrue(valid[2, 2])
        image = np.zeros((5, 5), dtype=np.uint8)
        image[2, 2] = 255
        points = depth_to_original_camera_points(
            image, source, rays=rays, ray_valid=valid,
            max_distance_m=50.0, stride=1,
        )
        np.testing.assert_allclose(points[0], [0.0, decode_recorded_depth(image)[2, 2], 0.0], atol=1e-6)
        all_points = depth_to_original_camera_points(
            image, source, rays=rays, ray_valid=valid,
            max_distance_m=50.0, include_far_plane=True,
        )
        self.assertGreater(len(all_points), len(points))
        self.assertAlmostEqual(float(np.linalg.norm(all_points, axis=1).max()), 50.0, places=3)

    def test_invalid_ds_domain_is_filtered(self):
        source = DoubleSphereIntrinsics(1.0, 1.0, 2.0, 2.0, -0.31, 0.9, 5, 5)
        rays, valid = double_sphere_unproject_grid(source)
        self.assertFalse(valid[0, 0])
        np.testing.assert_array_equal(rays[0, 0], np.zeros(3))

    def test_perspective_depth_uses_optical_z(self):
        source = DoubleSphereIntrinsics(90.0, 90.0, 2.0, 2.0, -0.31, 0.564, 5, 5)
        image = np.zeros((5, 5), dtype=np.uint8)
        image[2, 2] = 255
        perspective, _ = recorded_depth_to_perspective(
            image, source, 5, 5, 90.0, 90.0, max_depth_m=20.0
        )
        self.assertAlmostEqual(float(perspective[2, 2]), 0.5500, places=3)


if __name__ == "__main__":
    unittest.main()
