from pathlib import Path

import numpy as np

from data.route_contract import (
    RouteQualityFlag,
    RouteQualityGate,
    RouteRecord,
    build_witness_corridor,
    dense_route_arrays,
    load_route_table,
    pack_route_records,
    sample_route_bubbles,
    save_route_table,
)
from data.epic_route_labeler import label_epic_routes
from data.synthetic_dataset import generate_synthetic_dataset


def _straight_record() -> RouteRecord:
    points = np.stack(
        [np.linspace(0.0, 4.0, 17), np.zeros(17), np.full(17, 1.6)], axis=1
    ).astype(np.float32)
    return RouteRecord(
        frame_index=0,
        mission_goal_world=np.array([8.0, 0.0, 1.6], dtype=np.float32),
        frontier_goal_world=points[-1],
        path_points_world=points,
        path_clearance_m=np.full(17, 1.2, dtype=np.float32),
        path_bubble_radius_m=np.full(17, 0.7, dtype=np.float32),
        topo_centers_world=points[[0, 8, 16]],
        topo_bubble_radius_m=np.full(3, 0.8, dtype=np.float32),
        topo_persistent_id=np.array([10, 11, 12], dtype=np.uint64),
        route_seed=42,
    )


def test_route_npz_round_trip_without_pickle(tmp_path: Path):
    path = save_route_table(tmp_path / "routes.npz", pack_route_records([_straight_record()]))
    table = load_route_table(path, frame_count=1)
    assert len(table) == 1
    points, clearance, radius = table.path(0)
    assert points.shape == (17, 3)
    np.testing.assert_allclose(clearance - radius, 0.5, atol=1e-6)
    assert int(table.topology(0)[2][-1]) == 12


def test_quality_gate_accepts_safe_forward_corridor():
    record = _straight_record()
    result = RouteQualityGate().evaluate(
        path_points_world=record.path_points_world,
        path_clearance_m=record.path_clearance_m,
        path_bubble_radius_m=record.path_bubble_radius_m,
        start_world=record.path_points_world[0],
        frontier_world=record.frontier_goal_world,
        start_rotation_world_body=np.eye(3, dtype=np.float32),
    )
    assert result.valid
    assert result.path_length_m == 4.0


def test_quality_gate_rejects_clearance_and_reverse_route():
    record = _straight_record()
    points = record.path_points_world.copy()
    points[:, 0] *= -1.0
    result = RouteQualityGate().evaluate(
        path_points_world=points,
        path_clearance_m=np.full(len(points), 0.45, dtype=np.float32),
        path_bubble_radius_m=np.full(len(points), -0.05, dtype=np.float32),
        start_world=points[0],
        frontier_world=points[-1],
        start_rotation_world_body=np.eye(3, dtype=np.float32),
    )
    assert result.flags & RouteQualityFlag.CLEARANCE
    assert result.flags & RouteQualityFlag.LATTICE_DIRECTION


def test_quality_gate_rejects_nonfinite_without_returning_nan_metrics():
    record = _straight_record()
    record.path_points_world[3, 0] = np.nan
    result = RouteQualityGate().evaluate(
        path_points_world=record.path_points_world,
        path_clearance_m=record.path_clearance_m,
        path_bubble_radius_m=record.path_bubble_radius_m,
        start_world=np.zeros(3, dtype=np.float32),
        frontier_world=record.frontier_goal_world,
    )
    assert result.flags & RouteQualityFlag.NON_FINITE
    assert np.isfinite(result.weight)


def test_corridor_clearance_comes_from_obstacle_distance():
    path = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)
    obstacles = np.stack(
        [np.linspace(0.0, 2.0, 9), np.ones(9), np.zeros(9)], axis=1
    ).astype(np.float32)
    points, clearance, radius = build_witness_corridor(
        path, obstacles, robot_radius_m=0.3, safety_margin_m=0.2, max_step_m=0.25
    )
    assert len(points) == 9
    np.testing.assert_allclose(clearance, 1.0, atol=1e-6)
    np.testing.assert_allclose(radius, 0.5, atol=1e-6)


def test_fixed_and_dense_route_sampling_are_masked_and_conservative():
    record = _straight_record()
    record.path_bubble_radius_m[5:8] = 0.25
    centers, radii, mask, distances = sample_route_bubbles(
        record.path_points_world, record.path_bubble_radius_m, [1, 2, 3, 5, 8]
    )
    assert centers.shape == (5, 3)
    assert distances.shape == (5,)
    np.testing.assert_array_equal(mask, [1, 1, 1, 0, 0])
    assert np.min(radii[:3]) == 0.25
    dense_points, dense_radii, dense_mask = dense_route_arrays(
        record.path_points_world,
        record.path_bubble_radius_m,
        count=24,
        step_m=0.25,
    )
    assert dense_points.shape == (24, 3)
    assert dense_radii.shape == (24,)
    assert int(dense_mask.sum()) == 17
    np.testing.assert_allclose(dense_points[-1], record.path_points_world[-1])


def test_epic_labeler_preserves_failures_and_builds_corridor(tmp_path: Path):
    root = generate_synthetic_dataset(tmp_path / "data", scene_count=1, frames_per_scene=1)
    scene = root / "Scene_0000"
    source = load_route_table(scene / "routes.npz")
    points, _, _ = source.path(0)
    frontier = source.arrays["frontier_goal_world"][0].tolist()
    records = [
        {
            "frame_index": 0,
            "mission_goal_world": [0.0, 15.0, 1.6],
            "frontier_goal_world": frontier,
            "path_points_world": points.tolist(),
            "topo_centers_world": points[::10].tolist(),
            "topo_bubble_radius_m": [1.0] * len(points[::10]),
            "topo_persistent_id": list(range(len(points[::10]))),
            "found": True,
            "committed": True,
        },
        {
            "frame_index": 0,
            "mission_goal_world": [0.0, 15.0, 1.6],
            "frontier_goal_world": frontier,
            "path_points_world": [],
            "found": False,
            "blocked": True,
        },
    ]
    output = label_epic_routes(scene, records, output_name="routes_rebuilt.npz")
    rebuilt = load_route_table(output, frame_count=1)
    assert len(rebuilt) == 2
    assert rebuilt.arrays["route_valid"].tolist() == [1, 0]
    assert int(rebuilt.arrays["route_quality_flags"][1]) & int(RouteQualityFlag.BLOCKED)
    assert len(rebuilt.path(0)[0]) > len(points)
