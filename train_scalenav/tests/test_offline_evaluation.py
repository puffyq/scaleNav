import numpy as np

from evaluate_yopo import _corridor_metrics, _sample_trajectory


def test_offline_trajectory_reconstruction_matches_boundary_positions():
    start = np.array(
        [[1.0, 2.0, 1.6], [0.5, 0.0, 0.0], [0.0, 0.0, 0.0]],
        dtype=np.float32,
    )
    end = np.array(
        [[7.0, 3.0, 1.6], [1.0, 0.2, 0.0], [0.0, 0.0, 0.0]],
        dtype=np.float32,
    )
    trajectory = _sample_trajectory(start, end, count=41)
    np.testing.assert_allclose(trajectory[0], start[0], atol=1.0e-5)
    np.testing.assert_allclose(trajectory[-1], end[0], atol=1.0e-4)
    assert np.isfinite(trajectory).all()


def test_offline_corridor_metrics_distinguish_inside_and_outside():
    x = np.linspace(0.0, 10.0, 41, dtype=np.float32)
    route = np.stack((x, np.zeros_like(x), np.full_like(x, 1.6)), axis=1)
    radii = np.ones(41, dtype=np.float32)
    inside = np.array([[0.0, 0.0, 1.6], [5.0, 0.5, 1.6]], dtype=np.float32)
    outside = np.array([[0.0, 0.0, 1.6], [5.0, 2.0, 1.6]], dtype=np.float32)
    inside_max, inside_mean, inside_progress = _corridor_metrics(inside, route, radii)
    outside_max, outside_mean, outside_progress = _corridor_metrics(outside, route, radii)
    assert inside_max == 0.0 and inside_mean == 0.0
    assert outside_max == 1.0 and outside_mean == 0.5
    assert inside_progress == outside_progress == 5.0
