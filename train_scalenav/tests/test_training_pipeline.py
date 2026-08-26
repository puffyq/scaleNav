from pathlib import Path

import numpy as np
import pytest
import torch

from data.synthetic_dataset import generate_synthetic_dataset
from policy.yopo_dataset import YOPODataset
from policy.yopo_network import YopoNetwork
from policy.yopo_trainer import YopoTrainer


@pytest.fixture(scope="module")
def synthetic_data(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return generate_synthetic_dataset(
        tmp_path_factory.mktemp("route_data"), scene_count=2, frames_per_scene=2
    )


def test_dataset_uses_scene_split_and_separate_dense_route(synthetic_data: Path):
    train = YOPODataset(
        "train", data_root=synthetic_data, route_dropout_probability=0.0
    )
    valid = YOPODataset("valid", data_root=synthetic_data)
    assert {sample[0] for sample in train.samples}.isdisjoint(
        {sample[0] for sample in valid.samples}
    )
    sample = train[0]
    assert sample["route_bubbles"].shape == (12, 4)
    assert sample["route_mask"].shape == (12,)
    assert sample["dense_route_world"].shape == (128, 3)
    assert sample["dense_route_mask"].sum() > sample["route_mask"].sum()
    assert sample["depth"].min() >= 0.0 and sample["depth"].max() <= 1.0


def test_route_dropout_keeps_frontier_but_masks_routes(synthetic_data: Path):
    dataset = YOPODataset(
        "train", data_root=synthetic_data, route_dropout_probability=1.0
    )
    sample = dataset[0]
    assert torch.count_nonzero(sample["route_mask"]) == 0
    assert torch.count_nonzero(sample["dense_route_mask"]) == 0
    assert torch.isfinite(sample["frontier_body"]).all()


def test_same_depth_and_frontier_respond_to_different_witness_routes(synthetic_data: Path):
    dataset = YOPODataset("all", data_root=synthetic_data, route_dropout_probability=0.0)
    first = dataset[0]
    second = dataset[1]
    torch.testing.assert_close(first["depth"], second["depth"])
    torch.testing.assert_close(first["frontier_body"], second["frontier_body"], atol=1e-5, rtol=1e-5)
    assert not torch.allclose(first["route_bubbles"], second["route_bubbles"])
    model = YopoNetwork().eval()
    with torch.inference_mode():
        first_endstate, _ = model(
            first["depth"][None],
            first["motion_body"][None],
            first["frontier_body"][None],
            first["route_bubbles"][None],
            first["route_mask"][None],
        )
        second_endstate, _ = model(
            first["depth"][None],
            first["motion_body"][None],
            first["frontier_body"][None],
            second["route_bubbles"][None],
            second["route_mask"][None],
        )
    assert torch.max(torch.abs(first_endstate - second_endstate)).item() > 1.0e-7


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
        route_dropout_probability=0.0,
    )
    before = trainer.policy.yopo_head.model[0].weight.detach().clone()
    metrics = trainer.run_epoch(trainer.train_dataloader, training=True, max_batches=1)
    after = trainer.policy.yopo_head.model[0].weight.detach()
    assert np.isfinite(list(metrics.values())).all()
    assert metrics["path_progress"] > 0.0
    assert not torch.equal(before, after)
    trainer.save_checkpoint(trainer.output_path / "smoke.pth")
    checkpoint = torch.load(
        trainer.output_path / "smoke.pth", map_location="cpu", weights_only=False
    )
    assert checkpoint["route_dataset_version"] == 1
    assert checkpoint["route_bubble_count"] == 12
