from __future__ import annotations

import pytest
import torch

from yopo_inference_scaling import YopoInferenceScaling


def test_ten_mps_inference_scale_matches_original_yopo_testing_rule():
    scaling = YopoInferenceScaling(
        training_speed_mps=6.0,
        training_acceleration_mps2=6.0,
        inference_speed_mps=10.0,
        base_segment_time_s=10.0 / 6.0,
    )

    assert scaling.speed_ratio == pytest.approx(10.0 / 6.0)
    assert scaling.inference_acceleration_mps2 == pytest.approx(100.0 / 6.0)
    assert scaling.segment_time_s == pytest.approx(1.0)


def test_frozen_six_mps_torchscript_input_and_output_are_rescaled():
    scaling = YopoInferenceScaling(6.0, 6.0, 10.0, 10.0 / 6.0)
    observation = torch.tensor(
        [[10.0, 0.0, 0.0, 100.0 / 6.0, 0.0, 0.0, 10.0, 2.0, 0.0]]
    )
    model_observation = scaling.model_input(observation)

    # The frozen model divides these values by its internal constant six,
    # producing the same unit state as a native 10 m/s inference export.
    assert model_observation[0, 0].item() / 6.0 == pytest.approx(1.0)
    assert model_observation[0, 3].item() / 6.0 == pytest.approx(1.0)
    torch.testing.assert_close(model_observation[:, 6:9], observation[:, 6:9])

    frozen_output = torch.zeros(1, 9, 3, 5)
    frozen_output[:, 3] = 6.0
    frozen_output[:, 6] = 6.0
    physical = scaling.physical_endstate(frozen_output)
    assert physical[0, 3, 0, 0].item() == pytest.approx(10.0)
    assert physical[0, 6, 0, 0].item() == pytest.approx(100.0 / 6.0)


@pytest.mark.parametrize("value", [0.0, -1.0, float("inf"), float("nan")])
def test_invalid_inference_speed_is_rejected(value):
    with pytest.raises(ValueError):
        YopoInferenceScaling(6.0, 6.0, value, 10.0 / 6.0)
