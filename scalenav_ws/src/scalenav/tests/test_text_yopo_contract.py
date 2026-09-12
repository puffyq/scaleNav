from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import torch

from text_tracker.network import (
    TextYopoNetwork,
    export_text_yopo_torchscript,
    load_original_yopo_weights,
)


class TextYopoContractTests(unittest.TestCase):
    def test_python_model_accepts_depth_pearl_and_3d_goal(self) -> None:
        model = TextYopoNetwork().cpu().eval()
        image = torch.zeros(2, 2, 96, 160)
        observation = torch.zeros(2, 9)
        observation[:, 6] = 5.0

        with torch.inference_mode():
            endstate, score = model(image, observation)

        self.assertEqual(tuple(endstate.shape), (2, 9, 3, 5))
        self.assertEqual(tuple(score.shape), (2, 3, 5))
        self.assertTrue(torch.isfinite(endstate).all())
        self.assertTrue(torch.isfinite(score).all())

    def test_exported_model_preserves_contract_for_multiple_batches(self) -> None:
        model = TextYopoNetwork().cpu().eval()
        with tempfile.TemporaryDirectory() as temporary:
            path = export_text_yopo_torchscript(
                model,
                Path(temporary) / "text_yopo.pt",
                image_height=96,
                image_width=160,
            )
            loaded = torch.jit.load(str(path), map_location="cpu").eval()
            for batch_size in (1, 2):
                image = torch.zeros(batch_size, 2, 96, 160)
                observation = torch.zeros(batch_size, 9)
                observation[:, 6] = 5.0
                with torch.inference_mode():
                    endstate, score = loaded(image, observation)
                self.assertEqual(tuple(endstate.shape), (batch_size, 9, 3, 5))
                self.assertEqual(tuple(score.shape), (batch_size, 3, 5))
                self.assertTrue(torch.isfinite(endstate).all())
                self.assertTrue(torch.isfinite(score).all())

    def test_original_yopo_initializes_graph_executor(self) -> None:
        source_path = (
            Path(__file__).resolve().parents[2]
            / "models"
            / "original_yopo_simple"
            / "model.pt"
        )
        model = TextYopoNetwork().cpu().eval()
        loaded = load_original_yopo_weights(model, source_path)

        self.assertGreater(loaded, 100)
        first = model.image_backbone.cnn.conv1.weight.detach()
        self.assertGreater(float(first[:, 0].abs().sum()), 0.0)
        self.assertEqual(float(first[:, 1].abs().sum()), 0.0)
        self.assertEqual(tuple(model.yopo_head.model[0].weight.shape), (256, 73, 1, 1))


if __name__ == "__main__":
    unittest.main()
