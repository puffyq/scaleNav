from pathlib import Path

import cv2
import numpy as np

from data.ground_truth_dataset import (
    BoxObstacle,
    GroundTruthConfig,
    GroundTruthScene,
    generate_ground_truth_dataset,
)
from data.route_contract import load_route_table
from data.snapshot_dataset import validate_dataset


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
