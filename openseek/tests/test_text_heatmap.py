import unittest

import numpy as np
import torch

from text_tracker.heatmap import (
    GuidanceMode,
    HeatmapModeSelector,
    goal_body_to_heatmap,
    pool_heatmap_to_primitives,
    sample_heatmap_at_body_directions,
)
from text_tracker.loss import progress_weighted_heatmap_value


class GoalHeatmapTest(unittest.TestCase):
    def test_horizontal_goal_projection(self) -> None:
        forward = goal_body_to_heatmap(np.array([5.0, 0.0, 0.0]))
        left = goal_body_to_heatmap(np.array([5.0, 2.0, 0.0]))
        right = goal_body_to_heatmap(np.array([5.0, -2.0, 0.0]))
        self.assertLess(abs(np.argmax(forward[0]) - 79.5), 2.0)
        self.assertLess(np.argmax(left[0]), np.argmax(forward[0]))
        self.assertGreater(np.argmax(right[0]), np.argmax(forward[0]))
        self.assertAlmostEqual(float(forward.max()), 1.0, places=2)

    def test_three_dimensional_goal_projection(self) -> None:
        level = goal_body_to_heatmap(
            np.array([5.0, 0.0, 0.0]), horizontal_only=False
        )
        above = goal_body_to_heatmap(
            np.array([5.0, 0.0, 2.0]), horizontal_only=False
        )
        self.assertLess(np.unravel_index(above.argmax(), above.shape)[0],
                        np.unravel_index(level.argmax(), level.shape)[0])

    def test_goal_distance_controls_peak_amplitude(self) -> None:
        near = goal_body_to_heatmap(
            np.array([2.0, 0.0, 0.0]), distance_scale=10.0
        )
        far = goal_body_to_heatmap(
            np.array([10.0, 0.0, 0.0]), distance_scale=10.0
        )
        arrived = goal_body_to_heatmap(
            np.zeros(3, dtype=np.float32), distance_scale=10.0
        )

        self.assertAlmostEqual(float(near.max() / far.max()), 0.2, places=5)
        self.assertEqual(float(arrived.max()), 0.0)

    def test_signed_pooling_preserves_negative_values(self) -> None:
        heatmap = torch.zeros(1, 32, 160)
        heatmap[:, :, :32] = -0.8
        heatmap[:, :, 64:96] = 0.9
        pooled = pool_heatmap_to_primitives(heatmap, 1, 5)
        self.assertAlmostEqual(float(pooled[0, 0, 0]), -0.8, places=5)
        self.assertAlmostEqual(float(pooled[0, 0, 2]), 0.9, places=5)

    def test_direction_sampling_is_differentiable(self) -> None:
        heatmap = torch.from_numpy(
            goal_body_to_heatmap(np.array([5.0, 0.0, 0.0]))
        ).unsqueeze(0)
        directions = torch.tensor(
            [[[[5.0, 1.0, 0.0], [5.0, 0.0, 0.0]]]],
            requires_grad=True,
        )
        values = sample_heatmap_at_body_directions(
            heatmap,
            directions,
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            horizontal_only=True,
        )
        self.assertGreater(float(values[0, 0, 1]), float(values[0, 0, 0]))
        values.sum().backward()
        self.assertTrue(torch.isfinite(directions.grad).all())

    def test_zero_length_direction_has_finite_zero_gradient(self) -> None:
        heatmap = torch.rand(1, 32, 160)
        directions = torch.zeros(1, 1, 1, 3, requires_grad=True)
        values = sample_heatmap_at_body_directions(
            heatmap,
            directions,
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            horizontal_only=True,
        )

        self.assertEqual(float(values.item()), 0.0)
        values.sum().backward()
        self.assertTrue(torch.isfinite(directions.grad).all())
        self.assertTrue(torch.equal(directions.grad, torch.zeros_like(directions)))

    def test_heatmap_guidance_requires_forward_progress(self) -> None:
        heatmap_value = torch.tensor([1.0, 1.0, 1.0, -1.0])
        endpoints = torch.tensor(
            [[0.0, 0.0, 0.0], [2.5, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 0.0, 0.0]]
        )
        guided = progress_weighted_heatmap_value(
            heatmap_value, endpoints, distance_scale=10.0
        )
        torch.testing.assert_close(
            guided, torch.tensor([0.0, 0.25, 1.0, -1.0])
        )

    def test_mode_selector_uses_hysteresis(self) -> None:
        selector = HeatmapModeSelector(
            enter_threshold=0.5,
            exit_threshold=0.3,
            enter_frames=2,
            exit_frames=2,
        )
        goal = np.array([5.0, 0.0, 0.0])
        detected = np.full((32, 160), 0.6, dtype=np.float32)
        self.assertEqual(selector.update(goal, detected).mode, GuidanceMode.SEARCH)
        self.assertEqual(selector.update(goal, detected).mode, GuidanceMode.APPROACH)
        self.assertEqual(selector.update(goal, None).mode, GuidanceMode.APPROACH)
        self.assertEqual(selector.update(goal, None).mode, GuidanceMode.SEARCH)


if __name__ == "__main__":
    unittest.main()
