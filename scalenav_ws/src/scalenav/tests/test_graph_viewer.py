from __future__ import annotations

import unittest
from pathlib import Path

from serve_graph_viewer import GraphViewerEngine


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class GraphViewerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.engine = GraphViewerEngine(PROJECT_ROOT / "data" / "Map2GraphData")

    def test_catalog_contains_map2_scenes(self) -> None:
        scenes = {scene["name"]: scene for scene in self.engine.catalog["scenes"]}
        self.assertIn("Scene_0001", scenes)
        self.assertIn("Scene_0002", scenes)
        self.assertEqual(len(scenes["Scene_0002"]["frames"]), 1)

    def test_map2_wall_payload_has_graph_layers_and_depth_png(self) -> None:
        payload = self.engine.frame_payload("Scene_0002", 0)
        names = {trace["name"] for trace in payload["figure"]["data"]}
        self.assertIn("CERTIFIED edge", names)
        self.assertIn("INVALID edge", names)
        self.assertIn("Goal", names)
        self.assertIn("Waypoint", names)
        self.assertTrue(payload["depthImage"].startswith("data:image/png;base64,"))
        self.assertGreater(payload["stateCounts"]["CERTIFIED"], 0)
        self.assertGreater(payload["stateCounts"]["INVALID"], 0)
        self.assertEqual(payload["certifiedPath"], [])
        self.assertEqual(payload["optimisticPath"], [0, 2, 1])
        self.assertGreaterEqual(payload["nodeCount"], 2)

    def test_obstacle_map_and_pearl_are_present(self) -> None:
        payload = self.engine.frame_payload("Scene_0002", 0)
        traces = {trace["name"]: trace for trace in payload["figure"]["data"]}
        self.assertIn("Obstacle map", traces)
        self.assertIn("Observed depth", traces)
        self.assertLessEqual(len(traces["Obstacle map"]["x"]), 15000)
        self.assertGreater(payload["observedDepthPointCount"], 0)
        self.assertGreater(payload["obstaclePointCount"], len(traces["Obstacle map"]["x"]))
        self.assertEqual(payload["pearlPrompt"], "obstacle")
        self.assertTrue(payload["pearlAvailable"], payload["pearlError"])
        self.assertTrue(payload["pearlImage"].startswith("data:image/png;base64,"))

    def test_candidate_six_cannot_connect_to_goal(self) -> None:
        payload = self.engine.frame_payload("Scene_0002", 0)
        self.assertIn(
            {"source": 2, "target": 1, "state": "UNVALIDATED"},
            payload["edgeDetails"],
        )
        self.assertIn(
            {"source": 6, "target": 1, "state": "INVALID"},
            payload["edgeDetails"],
        )

    def test_rejects_unknown_scene(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unknown scene"):
            self.engine.frame_payload("Scene_missing", 0)


if __name__ == "__main__":
    unittest.main()
