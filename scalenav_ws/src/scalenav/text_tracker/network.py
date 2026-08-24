from __future__ import annotations

from pathlib import Path
from typing import Mapping

import torch
from torch import nn

from config.config import cfg
from policy.models.backbone import YopoBackbone
from policy.models.head import YopoHead
from policy.state_transform import StateTransform


class TextYopoNetwork(nn.Module):
    """YOPO with Depth+PEARL image input and a retained 3-D goal."""

    def __init__(self, hidden_state: int = 64) -> None:
        super().__init__()
        self.state_transform = StateTransform()
        self.image_backbone = YopoBackbone(hidden_state)
        self.image_backbone.cnn.conv1 = nn.Conv2d(
            2, 64, kernel_size=7, stride=2, padding=3, bias=False
        )
        self.state_backbone = nn.Sequential()
        self.yopo_head = YopoHead(hidden_state + 9, 10)

    def forward(
        self, image: torch.Tensor, obs: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        normalized_obs = obs.clone()
        normalized_obs = self.state_transform.normalize_obs(normalized_obs)
        primitive_obs = self.state_transform.prepare_input(normalized_obs)
        image_features = self.image_backbone(image)
        raw = self.yopo_head(torch.cat((primitive_obs, image_features), dim=1))

        endstate_pred = torch.tanh(raw[:, :9])
        endstate = self.state_transform.pred_to_endstate(endstate_pred)
        score = nn.functional.softplus(raw[:, 9])
        return endstate, score


def load_original_yopo_weights(
    model: TextYopoNetwork,
    checkpoint: str | Path,
) -> int:
    """Initialize the 2-channel model from an original 1-channel YOPO model."""
    source_module = torch.jit.load(str(checkpoint), map_location="cpu").eval()
    source: Mapping[str, torch.Tensor] = source_module.state_dict()
    target = model.state_dict()
    loaded = 0
    for source_key, value in source.items():
        key = source_key.removeprefix("model.")
        if key not in target:
            continue
        if key == "image_backbone.cnn.conv1.weight":
            if value.shape[1] != 1 or target[key].shape[1] != 2:
                raise ValueError(
                    f"unexpected first convolution shapes: {tuple(value.shape)} -> "
                    f"{tuple(target[key].shape)}"
                )
            target[key][:, :1].copy_(value)
            target[key][:, 1:].zero_()
            loaded += 1
        elif target[key].shape == value.shape:
            target[key].copy_(value)
            loaded += 1
    if loaded == 0:
        raise ValueError(f"no compatible YOPO tensors found in {checkpoint}")
    model.load_state_dict(target)
    return loaded


def export_text_yopo_torchscript(
    model: TextYopoNetwork,
    output_path: str | Path,
    *,
    image_height: int,
    image_width: int,
) -> Path:
    """Export and verify the fixed Depth+PEARL, 9-D state contract."""
    if image_height <= 0 or image_width <= 0:
        raise ValueError("image dimensions must be positive")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    device = next(model.parameters()).device
    model.eval()
    example_image = torch.zeros(
        1, 2, image_height, image_width, device=device, dtype=torch.float32
    )
    example_obs = torch.zeros(1, 9, device=device, dtype=torch.float32)
    example_obs[:, 6] = 1.0
    with torch.inference_mode():
        traced = torch.jit.trace(
            model,
            (example_image, example_obs),
            check_inputs=[
                (
                    torch.zeros_like(example_image),
                    torch.tensor(
                        [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 1.0, 0.0]],
                        device=device,
                    ),
                )
            ],
        )
        traced.save(str(output))
        loaded = torch.jit.load(str(output), map_location=device).eval()
        endstate, score = loaded(example_image, example_obs)
    expected_endstate = (1, 9, int(cfg["vertical_num"]), int(cfg["horizon_num"]))
    expected_score = (1, int(cfg["vertical_num"]), int(cfg["horizon_num"]))
    if tuple(endstate.shape) != expected_endstate or tuple(score.shape) != expected_score:
        raise RuntimeError(
            "exported model contract mismatch: "
            f"endstate={tuple(endstate.shape)} score={tuple(score.shape)}"
        )
    if not torch.isfinite(endstate).all() or not torch.isfinite(score).all():
        raise FloatingPointError("exported model produced non-finite output")
    return output
