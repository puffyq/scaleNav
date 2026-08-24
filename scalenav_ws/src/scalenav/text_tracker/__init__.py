from __future__ import annotations

from typing import Any

__all__ = [
    "GuidanceMode",
    "HeatmapModeSelector",
    "TextYopoDataset",
    "TextYopoGuidanceLoss",
    "TextYopoNetwork",
    "export_text_yopo_torchscript",
    "load_original_yopo_weights",
    "goal_body_to_heatmap",
    "goal_world_to_heatmap",
]


def __getattr__(name: str) -> Any:
    if name in {
        "GuidanceMode",
        "HeatmapModeSelector",
        "goal_body_to_heatmap",
        "goal_world_to_heatmap",
    }:
        from . import heatmap

        return getattr(heatmap, name)
    if name == "TextYopoDataset":
        from .dataset import TextYopoDataset

        return TextYopoDataset
    if name == "TextYopoGuidanceLoss":
        from .loss import TextYopoGuidanceLoss

        return TextYopoGuidanceLoss
    if name == "TextYopoNetwork":
        from .network import TextYopoNetwork

        return TextYopoNetwork
    if name in {"export_text_yopo_torchscript", "load_original_yopo_weights"}:
        from . import network

        return getattr(network, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
