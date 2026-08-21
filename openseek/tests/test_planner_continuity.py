import unittest

import numpy as np

from planner_continuity import (
    accept_local_goal,
    is_final_subgoal,
    mission_goal_for_local_goal,
    mission_arrived,
    project_goal_to_fixed_altitude,
)


class PlannerContinuityTests(unittest.TestCase):
    def test_changed_epic_waypoint_keeps_current_trajectory_valid(self):
        goal, changed, trajectory_valid = accept_local_goal(
            np.array([2.0, 0.0, 1.7]),
            np.array([3.0, 1.0, 1.7]),
            trajectory_valid=True,
        )

        np.testing.assert_allclose(goal, [3.0, 1.0, 1.7])
        self.assertTrue(changed)
        self.assertTrue(trajectory_valid)

    def test_duplicate_waypoint_is_not_republished(self):
        current = np.array([3.0, 1.0, 1.7])
        goal, changed, trajectory_valid = accept_local_goal(
            current, current.copy(), trajectory_valid=True)

        np.testing.assert_allclose(goal, current)
        self.assertFalse(changed)
        self.assertTrue(trajectory_valid)

    def test_only_mission_goal_is_a_final_subgoal(self):
        mission = np.array([40.0, 0.0, 1.6])

        self.assertFalse(is_final_subgoal(np.array([35.0, 1.0, 1.6]), mission, 0.25))
        self.assertTrue(is_final_subgoal(np.array([39.9, 0.0, 1.6]), mission, 0.25))

    def test_direct_goal_is_also_the_mission_goal(self):
        goal = np.array([10.0, -2.0, 1.6])

        mission = mission_goal_for_local_goal(
            goal, mission_goal=None, has_separate_mission_goal=False
        )

        np.testing.assert_allclose(mission, goal)
        self.assertTrue(is_final_subgoal(goal, mission, 0.25))

    def test_epic_local_goal_does_not_replace_mission_goal(self):
        local = np.array([10.0, 0.0, 1.6])
        mission = np.array([40.0, 0.0, 1.6])

        resolved = mission_goal_for_local_goal(
            local, mission, has_separate_mission_goal=True
        )

        np.testing.assert_allclose(resolved, mission)

    def test_mission_completion_requires_low_speed(self):
        mission = np.array([40.0, 0.0, 1.6])
        position = np.array([39.8, 0.0, 1.6])

        self.assertFalse(
            mission_arrived(position, np.array([0.5, 0.0, 0.0]), mission, 0.5, 0.3)
        )
        self.assertTrue(
            mission_arrived(position, np.array([0.2, 0.0, 0.0]), mission, 0.5, 0.3)
        )

    def test_mission_goal_uses_epic_fixed_height_layer(self):
        projected = project_goal_to_fixed_altitude(
            np.array([40.0, 0.0, 0.05]), 1.6
        )

        np.testing.assert_allclose(projected, [40.0, 0.0, 1.6])

if __name__ == "__main__":
    unittest.main()
