from __future__ import annotations

import unittest

import numpy as np

from graph.depth_query import DepthSafeVolumeQuery, ValidationState
from graph.replay import run_frame, synthetic_wall_depth
from graph.visualization import build_graph_visualization, enu_to_ned


class DepthSafeVolumeQueryTests(unittest.TestCase):
    def test_open_depth_certifies_forward_segment(self) -> None:
        query = DepthSafeVolumeQuery(np.full((96, 160), 20.0, dtype=np.float32))
        result = query.validate_segment(np.zeros(3), np.array([5.0, 0.0, 0.0]))
        self.assertEqual(result.state, ValidationState.CERTIFIED)
        self.assertGreater(result.known_fraction, 0.65)

    def test_far_clipped_goal_is_unknown_instead_of_occupied(self) -> None:
        query = DepthSafeVolumeQuery(np.full((96, 160), 20.0, dtype=np.float32))
        result = query.validate_segment(np.zeros(3), np.array([20.0, 0.0, 0.0]))
        self.assertEqual(result.state, ValidationState.UNVALIDATED)

    def test_wall_invalidates_forward_segment(self) -> None:
        query = DepthSafeVolumeQuery(synthetic_wall_depth())
        result = query.validate_segment(np.zeros(3), np.array([5.0, 0.0, 0.0]))
        self.assertEqual(result.state, ValidationState.INVALID)

    def test_missing_depth_remains_unvalidated(self) -> None:
        query = DepthSafeVolumeQuery(np.full((96, 160), np.nan, dtype=np.float32))
        result = query.validate_segment(np.zeros(3), np.array([5.0, 0.0, 0.0]))
        self.assertEqual(result.state, ValidationState.UNVALIDATED)

    def test_non_origin_edge_uses_surface_distance_not_screen_occlusion(self) -> None:
        depth = np.full((96, 160), 20.0, dtype=np.float32)
        depth[:, :120] = 5.0
        query = DepthSafeVolumeQuery(depth, robot_radius_m=0.6)
        goal = np.array([20.0, 0.0, 0.0])
        image_right = np.array([4.095760221, -2.867882181, 0.0])
        image_left = np.array([4.095760221, 2.867882181, 0.0])

        right_result = query.validate_optimistic_segment(image_right, goal)
        left_result = query.validate_optimistic_segment(image_left, goal)

        self.assertEqual(right_result.state, ValidationState.UNVALIDATED)
        self.assertEqual(left_result.state, ValidationState.INVALID)


class SparseDepthGraphTests(unittest.TestCase):
    def test_open_scene_selects_straight_waypoint(self) -> None:
        _, result = run_frame(
            np.full((96, 160), 20.0, dtype=np.float32),
            position_world=np.zeros(3),
            rotation_body_to_world=np.eye(3),
            goal_body=np.array([20.0, 0.0, 0.0]),
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            robot_radius_m=0.6,
        )
        waypoint = np.asarray(result["certifiedWaypointBody"])
        self.assertGreater(waypoint[0], 4.5)
        self.assertAlmostEqual(float(waypoint[1]), 0.0, places=5)

    def test_map2_wall_selects_lateral_certified_waypoint(self) -> None:
        graph, result = run_frame(
            synthetic_wall_depth(),
            position_world=np.zeros(3),
            rotation_body_to_world=np.eye(3),
            goal_body=np.array([20.0, 0.0, 0.0]),
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            robot_radius_m=0.6,
        )
        waypoint = np.asarray(result["certifiedWaypointBody"])
        self.assertGreater(waypoint[0], 3.5)
        self.assertGreater(abs(float(waypoint[1])), 1.0)
        self.assertGreater(result["stateCounts"]["INVALID"], 0)
        self.assertGreaterEqual(len(graph.nodes), 2)
        self.assertEqual(graph.goal_node_id, result["graph"]["goalNodeId"])
        self.assertEqual(result["certifiedPath"], [])
        self.assertEqual(result["optimisticPath"], [0, 2, 1])

    def test_unknown_depth_is_optimistic_but_not_executable(self) -> None:
        _, result = run_frame(
            np.full((96, 160), np.nan, dtype=np.float32),
            position_world=np.zeros(3),
            rotation_body_to_world=np.eye(3),
            goal_body=np.array([20.0, 0.0, 0.0]),
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            robot_radius_m=0.6,
        )
        self.assertIsNone(result["certifiedWaypointBody"])
        self.assertIsNotNone(result["optimisticWaypointBody"])
        self.assertGreater(result["stateCounts"]["UNVALIDATED"], 0)

    def test_unknown_space_keeps_start_to_goal_optimistic_edge(self) -> None:
        graph, result = run_frame(
            np.full((96, 160), np.nan, dtype=np.float32),
            position_world=np.zeros(3),
            rotation_body_to_world=np.eye(3),
            goal_body=np.array([20.0, 0.0, 0.0]),
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            robot_radius_m=0.6,
        )
        self.assertEqual(len(graph.nodes), 7)
        self.assertEqual(result["optimisticPath"][0], graph.current_node_id)
        self.assertEqual(result["optimisticPath"][-1], graph.goal_node_id)

    def test_graph_keeps_previous_topology_across_updates(self) -> None:
        graph, first = run_frame(
            np.full((96, 160), 20.0, dtype=np.float32),
            position_world=np.zeros(3),
            rotation_body_to_world=np.eye(3),
            goal_body=np.array([20.0, 0.0, 0.0]),
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            robot_radius_m=0.6,
        )
        previous_node_count = len(graph.nodes)
        next_position = np.asarray(first["certifiedWaypointBody"])
        graph.update(
            position_world=next_position,
            rotation_body_to_world=np.eye(3),
            goal_world=next_position + np.array([20.0, 0.0, 0.0]),
            depth_query=DepthSafeVolumeQuery(
                np.full((96, 160), 20.0, dtype=np.float32)
            ),
        )
        self.assertGreaterEqual(len(graph.nodes), previous_node_count)
        self.assertGreater(len(graph.edges), 0)
        self.assertEqual(
            graph.nodes[graph.current_node_id].state,
            ValidationState.CERTIFIED,
        )

    def test_visualization_snapshot_uses_graph_states_and_paths(self) -> None:
        graph, _ = run_frame(
            synthetic_wall_depth(),
            position_world=np.zeros(3),
            rotation_body_to_world=np.eye(3),
            goal_body=np.array([20.0, 0.0, 0.0]),
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            robot_radius_m=0.6,
        )
        update = graph.update(
            position_world=np.zeros(3),
            rotation_body_to_world=np.eye(3),
            goal_world=np.array([20.0, 0.0, 0.0]),
            depth_query=DepthSafeVolumeQuery(synthetic_wall_depth()),
        )
        snapshot = build_graph_visualization(
            graph, update, np.array([20.0, 0.0, 0.0])
        )
        self.assertGreater(len(snapshot.edge_segments["CERTIFIED"]), 0)
        self.assertGreater(len(snapshot.edge_segments["INVALID"]), 0)
        self.assertEqual(snapshot.certified_path, ())
        np.testing.assert_allclose(snapshot.goal, [20.0, 0.0, 0.0])
        self.assertIsNotNone(snapshot.waypoint)

    def test_enu_to_ned_inverts_colosseum_bridge_position_basis(self) -> None:
        np.testing.assert_allclose(
            enu_to_ned([2.0, 7.0, 3.0]), [7.0, 2.0, -3.0]
        )


if __name__ == "__main__":
    unittest.main()
