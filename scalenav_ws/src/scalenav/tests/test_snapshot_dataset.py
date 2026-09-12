from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from openseek.data.snapshot_dataset import (
    AirSimSnapshotCollector,
    CaptureConfig,
    depth_planar_to_world_ned,
    export_static_mesh_point_cloud,
    generated_person_collision_points,
    load_generated_people,
    PoseSample,
    PoseSampler,
    read_ascii_point_cloud_ply,
    SceneValidationError,
    SceneWriter,
    validate_dataset,
    validate_scene,
    write_point_cloud_ply,
)


class _ImageResponse:
    def __init__(self, pixels, width, height, image_type, timestamp):
        self.image_data_uint8 = pixels if image_type == 0 else np.array([], dtype=np.uint8)
        self.image_data_float = pixels if image_type == 1 else np.array([], dtype=np.float32)
        self.width = width
        self.height = height
        self.image_type = image_type
        self.time_stamp = timestamp


class _Vector:
    def __init__(self, x, y, z):
        self.x_val, self.y_val, self.z_val = x, y, z


class _Quaternion:
    def __init__(self, w, x, y, z):
        self.w_val, self.x_val, self.y_val, self.z_val = w, x, y, z


class _Pose:
    def __init__(self, position, orientation):
        self.position, self.orientation = position, orientation


class _FakeClient:
    def __init__(self, width=8, height=4):
        self.width = width
        self.height = height
        self.poses = []
        self.requests = []

    def simSetVehiclePose(self, pose, ignore_collision, vehicle_name=""):
        self.poses.append((pose, ignore_collision, vehicle_name))

    def simGetVehiclePose(self, vehicle_name=""):
        return _Pose(_Vector(1.0, 2.0, -1.6), _Quaternion(1.0, 0.0, 0.0, 0.0))

    def simGetImages(self, requests, vehicle_name=""):
        self.requests.append((requests, vehicle_name))
        rgb = np.full((self.height, self.width, 3), 127, dtype=np.uint8)
        depth = np.full((self.height, self.width), 4.0, dtype=np.float32)
        return [
            _ImageResponse(rgb, self.width, self.height, 0, 100),
            _ImageResponse(depth, self.width, self.height, 1, 101),
        ]


class _Mesh:
    def __init__(self, vertices):
        self.vertices = vertices


class SnapshotDatasetTests(unittest.TestCase):
    def test_depth_planar_world_conversion_keeps_pose_and_axes(self):
        depth = np.full((3, 3), 2.0, dtype=np.float32)
        points = depth_planar_to_world_ned(
            depth,
            PoseSample((10.0, 20.0, -1.6), (1.0, 0.0, 0.0, 0.0)),
            90.0,
            60.0,
            20.0,
            stride=1,
            max_points=20,
        )
        self.assertTrue(np.any(np.all(np.isclose(points, [12.0, 20.0, -1.6]), axis=1)))

    def test_pose_sampler_is_deterministic_and_bounded(self):
        first = PoseSampler((-2, 2), (-3, 3), altitude_m=1.6, seed=7).sample(5)
        second = PoseSampler((-2, 2), (-3, 3), altitude_m=1.6, seed=7).sample(5)
        self.assertEqual(first, second)
        for pose in first:
            self.assertGreaterEqual(pose.position_ned[0], -2)
            self.assertLessEqual(pose.position_ned[0], 2)
            self.assertGreaterEqual(pose.position_ned[1], -3)
            self.assertLessEqual(pose.position_ned[1], 3)
            self.assertEqual(pose.position_ned[2], -1.6)

    def test_colosseum_bgr_response_is_saved_as_rgb(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            obstacle = write_point_cloud_ply(root / "obstacles.ply", np.zeros((1, 3)))
            client = _FakeClient(width=1, height=1)
            client.simGetImages = lambda requests, vehicle_name="": [
                _ImageResponse(np.array([[[1, 2, 3]]], dtype=np.uint8), 1, 1, 0, 100),
                _ImageResponse(np.array([[4.0]], dtype=np.float32), 1, 1, 1, 101),
            ]
            collector = AirSimSnapshotCollector(
                client, CaptureConfig(settle_time_s=0, color_order="bgr")
            )
            collector.collect_scene(
                root / "Scene_0001",
                [PoseSample((0, 0, -1.6), (1, 0, 0, 0))],
                obstacle,
            )
            saved = cv2.imread(
                str(root / "Scene_0001" / "Textures" / "rgb_000000.png"),
                cv2.IMREAD_COLOR,
            )
            np.testing.assert_array_equal(saved[0, 0], [1, 2, 3])

    def test_binary_msgpack_image_payload_is_decoded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            obstacle = write_point_cloud_ply(root / "obstacles.ply", np.zeros((1, 3)))
            client = _FakeClient(width=1, height=1)
            client.simGetImages = lambda requests, vehicle_name="": [
                {"image_data_uint8": bytes([1, 2, 3]), "width": 1, "height": 1, "image_type": 0},
                {"image_data_float": np.asarray([4.0], dtype=np.float32).tobytes(), "width": 1, "height": 1, "image_type": 1},
            ]
            collector = AirSimSnapshotCollector(
                client, CaptureConfig(settle_time_s=0, color_order="bgr")
            )
            collector.collect_scene(
                root / "Scene_0001",
                [PoseSample((0, 0, -1.6), (1, 0, 0, 0))],
                obstacle,
            )
            self.assertTrue((root / "Scene_0001" / "Textures" / "rgb_000000.png").is_file())

    def test_scene_writer_round_trip_and_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            obstacle = write_point_cloud_ply(root / "obstacles.ply", np.zeros((2, 3)))
            scene = root / "Scene_0001"
            writer = SceneWriter(scene, CaptureConfig(settle_time_s=0), overwrite=False)
            rgb = np.zeros((4, 8, 3), dtype=np.uint8)
            depth = np.full((4, 8), 5.0, dtype=np.float32)
            writer.write_frame(0, rgb, depth, PoseSample((1, 2, -1.6), (1, 0, 0, 0)), 10, "person")
            writer.finalize(obstacle)
            self.assertEqual(validate_scene(scene), 1)
            self.assertEqual(validate_dataset(root), {"Scene_0001": 1})
            loaded_depth = cv2.imread(
                str(scene / "Textures" / "depth_000000.exr"), cv2.IMREAD_ANYDEPTH
            )
            np.testing.assert_allclose(loaded_depth, depth, atol=1e-5)

    def test_collector_writes_each_requested_pose(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            obstacle = write_point_cloud_ply(root / "obstacles.ply", np.zeros((1, 3)))
            client = _FakeClient()
            collector = AirSimSnapshotCollector(client, CaptureConfig(settle_time_s=0))
            output = collector.collect_scene(
                root / "Scene_0001",
                [PoseSample((0, 0, -1.6), (1, 0, 0, 0)), PoseSample((1, 0, -1.6), (1, 0, 0, 0))],
                obstacle,
            )
            self.assertTrue(output.is_file())
            self.assertEqual(len(client.poses), 2)
            self.assertEqual(len(client.requests), 2)
            self.assertEqual(validate_scene(root / "Scene_0001"), 2)

    def test_scene_is_consumable_by_text_yopo_dataset(self):
        from openseek.text_tracker.dataset import TextYopoDataset

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            obstacle = write_point_cloud_ply(root / "obstacles.ply", np.zeros((1, 3)))
            scene = root / "Scene_0001"
            writer = SceneWriter(scene, CaptureConfig(settle_time_s=0))
            writer.write_frame(
                0,
                np.zeros((4, 8, 3), dtype=np.uint8),
                np.full((4, 8), 5.0, dtype=np.float32),
                PoseSample((1, 2, -1.6), (1, 0, 0, 0)),
                10,
                "person",
            )
            writer.finalize(obstacle)
            np.save(scene / "Textures" / "semantic_pearl_000000.npy", np.zeros((4, 8), dtype=np.float32))
            dataset = TextYopoDataset(str(root), image_size=(8, 4))
            sample = dataset[0]
            self.assertEqual(tuple(sample["image"].shape), (2, 4, 8))
            self.assertAlmostEqual(float(sample["image"][0].max()), 0.25, places=4)
            np.testing.assert_allclose(sample["position"].numpy(), [1.0, 2.0, -1.6])
            np.testing.assert_allclose(
                sample["rotation"].numpy(),
                np.diag([1.0, -1.0, -1.0]),
                atol=1e-6,
            )

    def test_validation_rejects_missing_frame(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = root / "Scene_0001"
            scene.mkdir()
            (scene / "Textures").mkdir()
            write_point_cloud_ply(scene / "tree.ply", np.zeros((1, 3)))
            (scene / "data.toml").write_text(
                '[[dataArray]]\nrgbFileName = "rgb_000000.png"\n', encoding="utf-8"
            )
            with self.assertRaises(SceneValidationError):
                validate_scene(scene)

    def test_validation_can_require_semantic_heatmaps(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            obstacle = write_point_cloud_ply(root / "obstacles.ply", np.zeros((1, 3)))
            scene = root / "Scene_0001"
            writer = SceneWriter(scene, CaptureConfig(settle_time_s=0))
            writer.write_frame(
                0,
                np.zeros((2, 2, 3), dtype=np.uint8),
                np.ones((2, 2), dtype=np.float32),
                PoseSample((0, 0, -1.6), (1, 0, 0, 0)),
                1,
                "person",
            )
            writer.finalize(obstacle)
            with self.assertRaises(SceneValidationError):
                validate_scene(scene, require_semantic=True)
            np.save(scene / "Textures" / "semantic_pearl_000000.npy", np.ones((2, 2), dtype=np.float32))
            self.assertEqual(validate_scene(scene, require_semantic=True), 1)

    def test_static_mesh_export_converts_unreal_centimeters_to_ned_meters(self):
        class MeshClient:
            def simGetMeshPositionVertexBuffers(self):
                return [_Mesh([100.0, -200.0, 300.0, 0.0, 0.0, 0.0, 1e12, 0.0, 0.0])]

        with tempfile.TemporaryDirectory() as temporary:
            output = export_static_mesh_point_cloud(MeshClient(), Path(temporary) / "tree.ply")
            lines = output.read_text(encoding="ascii").splitlines()
            self.assertEqual(lines[7], "1.000000 -2.000000 -3.000000")
            self.assertEqual(len(lines), 8)

    def test_generated_people_are_converted_and_approximated(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            people_path = root / "generated_people.json"
            people_path.write_text(
                '{"radiusMeters": 0.4, "heightMeters": 2.0, "people": '
                '[{"positionCm": [100, -200, 300]}]}',
                encoding="utf-8",
            )
            people = load_generated_people(people_path)
            self.assertEqual(len(people), 1)
            self.assertEqual(people[0].position_ned, (1.0, -2.0, -3.0))
            points = generated_person_collision_points(people, radial_samples=4, vertical_samples=2)
            self.assertEqual(points.shape, (10, 3))
            self.assertAlmostEqual(float(points[:, 2].max()), -3.0)
            self.assertAlmostEqual(float(points[:, 2].min()), -5.0)

    def test_person_points_are_merged_into_tree_ply(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            obstacle = write_point_cloud_ply(root / "obstacles.ply", np.array([[9, 9, 9]], dtype=np.float32))
            people_path = root / "generated_people.json"
            people_path.write_text(
                '{"people": [{"positionCm": [100, 200, 0]}]}', encoding="utf-8"
            )
            scene = root / "Scene_0001"
            writer = SceneWriter(scene, CaptureConfig(settle_time_s=0))
            writer.write_frame(
                0,
                np.zeros((2, 2, 3), dtype=np.uint8),
                np.ones((2, 2), dtype=np.float32),
                PoseSample((0, 0, -1.6), (1, 0, 0, 0)),
                1,
                "person",
            )
            writer.finalize(obstacle, person_positions=people_path)
            points = read_ascii_point_cloud_ply(scene / "tree.ply")
            self.assertEqual(len(points), 1 + 8 * 5 + 2)
            np.testing.assert_allclose(points[0], [9, 9, 9])

    def test_scene_collision_merge_rejects_binary_ply(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary_ply = root / "binary.ply"
            binary_ply.write_bytes(b"ply\nformat binary_little_endian 1.0\n")
            with self.assertRaises(ValueError):
                read_ascii_point_cloud_ply(binary_ply)


if __name__ == "__main__":
    unittest.main()
