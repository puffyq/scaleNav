from pathlib import Path

import cv2
import json
import numpy as np

from data.ground_truth_dataset import (
    BoxObstacle,
    CylinderObstacle,
    GroundTruthConfig,
    GroundTruthScene,
    generate_ground_truth_dataset,
)
from data.build_dataset_viewer import build_dataset_viewer
from data.route_contract import load_route_table
from data.snapshot_dataset import validate_dataset
from data.snapshot_dataset import write_point_cloud_ply


def test_large_block_forces_ground_truth_astar_detour():
    config = GroundTruthConfig(
        map_size_x_m=20.0,
        map_size_y_m=20.0,
        obstacle_count=0,
        grid_resolution_m=0.2,
    )
    scene = GroundTruthScene(
        config,
        [BoxObstacle(0.0, 0.0, 4.0, 8.0, 4.0, 0.0)],
    )
    start = np.array([-7.0, 0.0, config.altitude_m], dtype=np.float32)
    goal = np.array([7.0, 0.0, config.altitude_m], dtype=np.float32)
    cells = scene.astar(scene.world_to_cell(start), scene.world_to_cell(goal))
    assert cells is not None
    path = scene.smooth_grid_path(cells)
    assert path is not None
    assert not scene.is_segment_free(start, goal)
    assert np.max(np.abs(path[:, 1])) > 4.5
    assert all(
        scene.is_segment_free(left, right, planning_margin=False)
        for left, right in zip(path[:-1], path[1:])
    )


def test_widest_shortest_removes_avoidable_route_neck():
    config = GroundTruthConfig(
        map_size_x_m=24.0,
        map_size_y_m=20.0,
        obstacle_count=0,
        grid_resolution_m=0.2,
        widest_detour_ratio=1.12,
        widest_clearance_target_m=1.2,
    )
    scene = GroundTruthScene(
        config,
        [BoxObstacle(0.0, 0.0, 4.0, 6.0, 4.0, 0.0)],
    )
    start = scene.world_to_cell(np.array([-9.0, 0.0, config.altitude_m]))
    goal = scene.world_to_cell(np.array([9.0, 0.0, config.altitude_m]))
    shortest = scene._astar_with_clearance(
        start, goal, minimum_safe_radius_m=config.planning_extra_margin_m
    )
    result = scene.widest_shortest_path(start, goal)

    assert shortest is not None and result is not None
    shortest_points = np.stack([scene.cell_to_world(cell) for cell in shortest[0]])
    shortest_safe_radius = (
        scene.clearance_at_world(shortest_points)
        - config.robot_radius_m
        - config.safety_margin_m
    )
    assert float(np.percentile(shortest_safe_radius, 5)) < 0.8
    assert result.safe_radius_p05_m >= 1.1
    assert result.detour_ratio <= config.widest_detour_ratio + 1.0e-5


def test_widest_shortest_does_not_escape_real_narrow_passage_with_long_detour():
    config = GroundTruthConfig(
        map_size_x_m=24.0,
        map_size_y_m=20.0,
        obstacle_count=0,
        grid_resolution_m=0.2,
        widest_detour_ratio=1.08,
        widest_clearance_target_m=1.8,
    )
    scene = GroundTruthScene(
        config,
        [
            BoxObstacle(0.0, 5.25, 18.0, 7.5, 4.0, 0.0),
            BoxObstacle(0.0, -5.25, 18.0, 7.5, 4.0, 0.0),
        ],
    )
    start = scene.world_to_cell(np.array([-10.0, 0.0, config.altitude_m]))
    goal = scene.world_to_cell(np.array([10.0, 0.0, config.altitude_m]))
    result = scene.widest_shortest_path(start, goal)

    assert result is not None
    assert result.detour_ratio <= config.widest_detour_ratio + 1.0e-5
    assert result.minimum_safe_radius_m < config.widest_clearance_target_m
    points = np.stack([scene.cell_to_world(cell) for cell in result.cells])
    assert np.max(np.abs(points[:, 1])) < 2.0


def test_centerline_refinement_moves_avoidable_neck_and_keeps_endpoints():
    config = GroundTruthConfig(
        map_size_x_m=24.0,
        map_size_y_m=20.0,
        obstacle_count=0,
        grid_resolution_m=0.2,
        centerline_iterations=8,
        centerline_step_m=0.1,
        centerline_max_deviation_m=0.8,
    )
    scene = GroundTruthScene(
        config,
        [BoxObstacle(0.0, 0.0, 4.0, 6.0, 4.0, 0.0)],
    )
    path = np.asarray(
        [[-9.0, 0.0, 1.6], [-3.0, 4.0, 1.6], [3.0, 4.0, 1.6], [9.0, 0.0, 1.6]],
        dtype=np.float32,
    )
    result = scene.refine_witness_centerline(path)

    np.testing.assert_allclose(result.points_world[0], path[0], atol=1.0e-6)
    np.testing.assert_allclose(result.points_world[-1], path[-1], atol=1.0e-6)
    assert result.iterations > 0
    assert result.safe_radius_p05_after_m >= result.safe_radius_p05_before_m + 0.19
    assert result.clearance_risk_after < result.clearance_risk_before
    assert all(
        scene.is_segment_free(left, right, planning_margin=True)
        for left, right in zip(result.points_world[:-1], result.points_world[1:])
    )


def test_centerline_refinement_does_not_invent_space_in_symmetric_narrow_passage():
    config = GroundTruthConfig(
        map_size_x_m=24.0,
        map_size_y_m=20.0,
        obstacle_count=0,
        grid_resolution_m=0.2,
        centerline_iterations=8,
    )
    scene = GroundTruthScene(
        config,
        [
            BoxObstacle(0.0, 5.25, 18.0, 7.5, 4.0, 0.0),
            BoxObstacle(0.0, -5.25, 18.0, 7.5, 4.0, 0.0),
        ],
    )
    path = np.stack(
        (np.linspace(-10.0, 10.0, 41), np.zeros(41), np.full(41, 1.6)), axis=1
    ).astype(np.float32)
    result = scene.refine_witness_centerline(path)

    assert result.gain_m < 0.05
    assert np.max(np.abs(result.points_world[:, 1])) < 0.05


def test_centerline_refinement_honors_selected_safe_radius():
    config = GroundTruthConfig(
        map_size_x_m=24.0,
        map_size_y_m=20.0,
        obstacle_count=0,
        grid_resolution_m=0.2,
        centerline_iterations=8,
    )
    scene = GroundTruthScene(
        config,
        [BoxObstacle(0.0, 0.0, 4.0, 6.0, 4.0, 0.0)],
    )
    path = np.asarray(
        [[-9.0, 0.0, 1.6], [-3.0, 5.0, 1.6], [3.0, 5.0, 1.6], [9.0, 0.0, 1.6]],
        dtype=np.float32,
    )
    result = scene.refine_witness_centerline(path, minimum_safe_radius_m=1.2)
    safe_radius = (
        scene.clearance_at_world(result.points_world)
        - config.robot_radius_m
        - config.safety_margin_m
    )
    assert float(np.min(safe_radius[1:-1])) + 1.0e-4 >= 1.2


def test_depth_renderer_sees_map2_style_block():
    config = GroundTruthConfig(
        map_size_x_m=20.0,
        map_size_y_m=20.0,
        obstacle_count=0,
        image_width=40,
        image_height=24,
    )
    scene = GroundTruthScene(
        config,
        [BoxObstacle(5.0, 0.0, 3.0, 5.0, 4.0, 0.0)],
    )
    depth = scene.render_depth(np.array([0.0, 0.0, config.altitude_m]), 0.0)
    assert depth.shape == (24, 40)
    assert 3.4 < float(depth[12, 20]) < 3.6
    assert float(depth[12, 0]) > float(depth[12, 20])


def test_clearance_field_does_not_alias_subgrid_tree_trunk():
    config = GroundTruthConfig(
        map_size_x_m=12.0,
        map_size_y_m=12.0,
        obstacle_count=0,
        grid_resolution_m=0.2,
    )
    tree = CylinderObstacle(0.0, 0.0, 0.06, 5.0)
    scene = GroundTruthScene(config, [tree])
    query = np.array([0.3, 0.0, config.altitude_m], dtype=np.float32)

    measured = float(scene.clearance_at_world(query))
    assert abs(measured - 0.24) < 0.04
    assert scene.planning_occupancy[scene.world_to_cell(query)]


def test_default_scene_contains_building_scale_block():
    config = GroundTruthConfig(obstacle_count=12)
    scene = GroundTruthScene.random(config, seed=101, style="blocks")
    boxes = [obstacle for obstacle in scene.obstacles if isinstance(obstacle, BoxObstacle)]
    assert boxes
    assert max(max(box.size_x_m, box.size_y_m) for box in boxes) >= 15.0
    assert max(max(box.size_x_m, box.size_y_m) for box in boxes) <= 30.0


def test_yopo_real_forest_uses_tree_asset_for_geometry_and_depth(tmp_path: Path):
    angles = np.linspace(0.0, 2.0 * np.pi, 32, endpoint=False, dtype=np.float32)
    heights = np.linspace(0.0, 4.0, 41, dtype=np.float32)
    angle_grid, height_grid = np.meshgrid(angles, heights, indexing="ij")
    trunk = np.stack(
        (
            0.2 * np.cos(angle_grid),
            0.2 * np.sin(angle_grid),
            height_grid,
        ),
        axis=-1,
    ).reshape(-1, 3)
    canopy_angles = np.linspace(0.0, 2.0 * np.pi, 48, endpoint=False, dtype=np.float32)
    canopy = np.column_stack(
        (
            1.4 * np.cos(canopy_angles),
            1.0 * np.sin(canopy_angles),
            np.full_like(canopy_angles, 3.5),
        )
    )
    asset = write_point_cloud_ply(tmp_path / "tree.ply", np.concatenate((trunk, canopy)))
    config = GroundTruthConfig(
        map_size_x_m=12.0,
        map_size_y_m=12.0,
        map_height_m=5.0,
        obstacle_count=0,
        point_resolution_m=0.15,
        image_width=40,
        image_height=24,
    )

    scene = GroundTruthScene.random(
        config, seed=17, style="yopo_real_forest", yopo_tree_ply=asset
    )

    assert scene.point_obstacles_world is not None
    assert len(scene.point_obstacles_world) > 100
    assert scene.scene_metadata["tree_instances"] == 9
    assert scene.route_blocker_centers_xy.shape == (9, 2)
    assert scene.scene_metadata["tree_asset"] == str(asset.resolve())
    assert np.any(scene.raw_occupancy)
    assert len(scene.obstacle_point_cloud()) > len(scene.point_obstacles_world)

    section = scene.point_obstacles_world[
        np.abs(scene.point_obstacles_world[:, 2] - config.altitude_m) < 0.2
    ]
    target = section[0]
    origin = target.copy()
    origin[0] -= 2.0
    depth = scene.render_depth(origin, 0.0)
    assert depth.shape == (24, 40)
    assert np.min(depth[:14]) < config.max_depth_m


def test_ground_truth_generator_writes_trainable_contract(tmp_path: Path):
    config = GroundTruthConfig(
        map_size_x_m=28.0,
        map_size_y_m=28.0,
        obstacle_count=18,
        routes_per_frame=2,
        maximum_frame_attempts=400,
    )
    root = generate_ground_truth_dataset(
        tmp_path / "dataset",
        scene_count=2,
        frames_per_scene=1,
        seed=11,
        config=config,
        preview_routes=2,
    )
    assert validate_dataset(root, require_routes=True) == {"Scene_0000": 1, "Scene_0001": 1}
    for scene_name in ("Scene_0000", "Scene_0001"):
        scene = root / scene_name
        routes = load_route_table(scene / "routes.npz", frame_count=1)
        assert len(routes) == 2
        assert routes.arrays["route_valid"].tolist() == [1, 1]
        depth = cv2.imread(
            str(scene / "Textures" / "depth_000000.exr"), cv2.IMREAD_ANYDEPTH
        )
        assert depth is not None and np.ptp(depth) > 0.1
    assert len(list((root / "route_previews").glob("*.png"))) == 2
    report = json.loads((root / "generation_report.json").read_text(encoding="utf-8"))
    assert report["route_count"] == 4
    assert report["detour_route_count"] >= 2
    assert report["minimum_clearance_m"]["min"] > 0.5


def test_static_viewer_contains_routes_and_depth_assets(tmp_path: Path):
    config = GroundTruthConfig(
        map_size_x_m=28.0,
        map_size_y_m=28.0,
        obstacle_count=16,
        routes_per_frame=2,
        maximum_frame_attempts=400,
    )
    root = generate_ground_truth_dataset(
        tmp_path / "dataset",
        scene_count=1,
        frames_per_scene=2,
        seed=71,
        config=config,
        preview_routes=0,
    )
    index = build_dataset_viewer(root)
    assert index.is_file()
    assert (index.parent / "assets" / "Scene_0000" / "map_topdown.png").is_file()
    assert len(list((index.parent / "assets").glob("Scene_*/depth_*.png"))) == 2
    script = (index.parent / "dataset.js").read_text(encoding="utf-8")
    assert '"frameCount":2' in script
    assert '"routeCount":4' in script
    assert '"minimumClearanceM"' in script
    assert '"dataset_role":"train"' in script
    assert 'src="dataset.js"' in index.read_text(encoding="utf-8")
