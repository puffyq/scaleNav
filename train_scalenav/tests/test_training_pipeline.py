import json
from pathlib import Path

import numpy as np
import pytest
import torch

from data.synthetic_dataset import generate_synthetic_dataset
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork
from policy.yopo_simple_baseline import YopoSimpleBaseline
from policy.yopo_trainer import YopoTrainer


@pytest.fixture(scope="module")
def synthetic_data(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return generate_synthetic_dataset(
        tmp_path_factory.mktemp("route_data"), scene_count=2, frames_per_scene=2
    )


def test_small_pilot_uses_disjoint_frame_groups_and_route_bubbles(
    synthetic_data: Path,
):
    train = YOPODataset("train", data_root=synthetic_data)
    valid = YOPODataset("valid", data_root=synthetic_data)
    def frame_keys(dataset: YOPODataset) -> set[tuple[int, int]]:
        return {
            (
                scene_index,
                int(dataset.scenes[scene_index].routes.arrays["frame_index"][route_index]),
            )
            for scene_index, route_index in dataset.samples
        }

    assert train.split_strategy == "frame_group_holdout"
    assert {sample[0] for sample in train.samples} == {0, 1}
    assert {sample[0] for sample in valid.samples} == {0, 1}
    assert frame_keys(train).isdisjoint(frame_keys(valid))
    sample = train[0]
    assert sample["route_bubbles"].shape == (12, 4)
    assert sample["route_points_world"].shape == (12, 3)
    assert sample["route_radii_world"].shape == (12,)
    assert torch.all(sample["route_radii_world"] > 0.0)
    assert sample["depth"].min() >= 0.0 and sample["depth"].max() <= 1.0


def test_three_scene_split_keeps_every_scene_in_train_and_validation(tmp_path: Path):
    data_root = generate_synthetic_dataset(
        tmp_path / "three_scenes", scene_count=3, frames_per_scene=3
    )
    train = YOPODataset("train", data_root=data_root)
    valid = YOPODataset("valid", data_root=data_root)

    assert train.split_strategy == "frame_group_holdout"
    assert valid.split_strategy == "frame_group_holdout"
    assert {scene for scene, _ in train.samples} == {0, 1, 2}
    assert {scene for scene, _ in valid.samples} == {0, 1, 2}

    def frame_keys(dataset: YOPODataset) -> set[tuple[int, int]]:
        return {
            (
                scene_index,
                int(dataset.scenes[scene_index].routes.arrays["frame_index"][route_index]),
            )
            for scene_index, route_index in dataset.samples
        }

    assert frame_keys(train).isdisjoint(frame_keys(valid))


def test_validation_motion_is_deterministic_per_sample(synthetic_data: Path):
    dataset = YOPODataset(
        "valid", data_root=synthetic_data, seed=42
    )
    torch.testing.assert_close(dataset[0]["motion_body"], dataset[0]["motion_body"])


def test_same_depth_and_frontier_respond_to_different_witness_routes(synthetic_data: Path):
    dataset = YOPODataset("all", data_root=synthetic_data)
    first = dataset[0]
    second = dataset[1]
    torch.testing.assert_close(first["depth"], second["depth"])
    assert not torch.allclose(first["route_bubbles"], second["route_bubbles"])
    model = YopoNetwork().eval()
    with torch.no_grad():
        model.yopo_head.model[0].weight[:, 73:].fill_(0.01)
    with torch.inference_mode():
        first_endstate, _ = model(
            first["depth"][None],
            first["motion_body"][None],
            first["frontier_body"][None],
            first["route_bubbles"][None],
        )
        second_endstate, _ = model(
            first["depth"][None],
            first["motion_body"][None],
            first["frontier_body"][None],
            second["route_bubbles"][None],
        )
    assert torch.max(torch.abs(first_endstate - second_endstate)).item() > 1.0e-7


def test_zero_initialized_route_extension_preserves_yopo_simple_outputs() -> None:
    torch.manual_seed(9)
    baseline = YopoSimpleBaseline().eval()
    route_model = YopoNetwork().eval()
    route_model.load_yopo_simple_state_dict(baseline.state_dict())
    depth = torch.rand(2, 1, 96, 160)
    motion = torch.randn(2, 6)
    goal = torch.randn(2, 3)
    route = torch.randn(2, 12, 4)
    with torch.inference_mode():
        expected = baseline(depth, motion, goal)
        actual = route_model(depth, motion, goal, route)
    torch.testing.assert_close(actual[0], expected[0], rtol=0.0, atol=0.0)
    torch.testing.assert_close(actual[1], expected[1], rtol=0.0, atol=0.0)


def test_one_training_batch_backpropagates_all_costs(
    synthetic_data: Path, tmp_path: Path
):
    torch.manual_seed(0)
    np.random.seed(0)
    trainer = YopoTrainer(
        data_root=synthetic_data,
        tensorboard_path=tmp_path / "runs",
        batch_size=1,
        num_workers=0,
        device="cpu",
    )
    before = trainer.policy.yopo_head.model[0].weight.detach().clone()
    metrics = trainer.run_epoch(trainer.train_dataloader, training=True, max_batches=1)
    after = trainer.policy.yopo_head.model[0].weight.detach()
    assert np.isfinite(list(metrics.values())).all()
    assert metrics["route_corridor"] >= 0.0
    assert "route_angle" not in metrics
    assert "route_centerline" not in metrics
    assert not torch.equal(before, after)
    trainer.save_checkpoint(trainer.output_path / "smoke.pth")
    checkpoint = torch.load(
        trainer.output_path / "smoke.pth", map_location="cpu", weights_only=False
    )
    assert checkpoint["route_dataset_version"] == 2
    assert checkpoint["local_subgoal_distance_m"] == 10.0
    assert checkpoint["route_bubble_count"] == 12
    assert checkpoint["active_loss_terms"] == [
        "smooth", "safety", "frontier", "acceleration", "route_corridor", "score_regression"
    ]
    assert checkpoint["safety_route_attraction_weight"] == 0.0
    metadata = json.loads((trainer.output_path / "run.json").read_text(encoding="utf-8"))
    assert metadata["checkpoint"] is None
    assert metadata["resume_training_state"] is True
    assert metadata["active_loss_terms"] == checkpoint["active_loss_terms"]
    assert metadata["safety_route_attraction_weight"] == 0.0
