from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from graph.frgraph_adapter import FRGraphAdapter
from graph.replay import load_scene_frame, run_frame


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class FRGraphAdapterTests(unittest.TestCase):
    def test_wall_keeps_only_the_open_camera_side(self) -> None:
        depth = np.full((96, 160), 20.0, dtype=np.float32)
        depth[:, :120] = 5.0
        heatmap = np.zeros((24, 40), dtype=np.float32)
        heatmap[:, 30:] = 0.75
        regions = FRGraphAdapter().extract(depth, heatmap)

        self.assertEqual(len(regions), 1)
        self.assertLess(regions[0].center_yaw_rad, 0.0)
        self.assertGreater(regions[0].pixel_count, 1000)
        self.assertGreater(regions[0].semantic_score, 0.5)

    def test_map2_region_produces_optimistic_goal_path(self) -> None:
        depth, position, rotation, horizontal_fov, vertical_fov, _ = load_scene_frame(
            PROJECT_ROOT / "data" / "Map2GraphData", "Scene_0002", 0
        )
        _, result = run_frame(
            depth,
            position_world=position,
            rotation_body_to_world=rotation,
            goal_body=np.array([20.0, 0.0, 0.0]),
            horizontal_fov_deg=horizontal_fov,
            vertical_fov_deg=vertical_fov,
            robot_radius_m=0.6,
            use_frgraph=True,
        )

        self.assertEqual(result["frgraph"]["regionCount"], 1)
        self.assertLess(result["frgraph"]["regions"][0]["centerYawDeg"], 0.0)
        self.assertEqual(result["optimisticPath"], [0, 2, 1])
        self.assertIn(
            {"source": 2, "target": 1, "state": "UNVALIDATED"},
            [
                {key: edge[key] for key in ("source", "target", "state")}
                for edge in result["graph"]["edges"]
            ],
        )


if __name__ == "__main__":
    unittest.main()
