from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from torch import Tensor, nn
from torchvision.transforms import Compose, Normalize, ToTensor


class SCLIPHeatmapEncoder:
    """Thin adapter around the official wangf3014/SCLIP implementation."""

    def __init__(
        self,
        sclip_root: str,
        checkpoint: str = "ViT-B/16",
        device: torch.device | None = None,
    ) -> None:
        self.device = device or torch.device("cuda" if torch.cuda.is_available() else "cpu")
        root = Path(sclip_root).expanduser().resolve()
        if not (root / "clip" / "model.py").is_file():
            raise FileNotFoundError(
                f"Official SCLIP source not found at {root}. Clone "
                "https://github.com/wangf3014/SCLIP.git first."
            )
        sys.path.insert(0, str(root))
        try:
            import clip  # type: ignore[import-not-found]
        finally:
            sys.path.pop(0)

        self._clip = clip
        self._model, _ = clip.load(checkpoint, device=self.device, jit=False)
        self._model.eval()
        for parameter in self._model.parameters():
            parameter.requires_grad_(False)
        self._preprocess = Compose(
            [
                ToTensor(),
                Normalize(
                    (0.48145466, 0.4578275, 0.40821073),
                    (0.26862954, 0.26130258, 0.27577711),
                ),
            ]
        )
        self._text_cache: dict[str, Tensor] = {}

    @torch.inference_mode()
    def encode(self, rgb_path: str, prompt: str) -> np.ndarray:
        image = Image.open(rgb_path).convert("RGB")
        return self._encode_image(image, prompt)

    @torch.inference_mode()
    def encode_rgb(self, rgb: np.ndarray, prompt: str) -> np.ndarray:
        """Encode an RGB uint8 array received from a camera."""
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"Expected HxWx3 RGB input, got {rgb.shape}")
        return self._encode_image(Image.fromarray(rgb.astype(np.uint8), "RGB"), prompt)

    def _encode_image(self, image: Image.Image, prompt: str) -> np.ndarray:
        original_size = (image.height, image.width)
        image_tensor = self._preprocess(image).unsqueeze(0).to(self.device)
        image_features = self._model.encode_image(
            image_tensor, return_all=True, csa=True
        )[:, 1:]
        image_features = nn.functional.normalize(image_features.float(), dim=-1)
        text_features = self._encode_text(prompt)
        cosine = image_features @ text_features.T

        patch_size = int(self._model.visual.patch_size)
        patch_h = image_tensor.shape[-2] // patch_size
        patch_w = image_tensor.shape[-1] // patch_size
        heatmap = cosine[:, :, 0].reshape(1, 1, patch_h, patch_w)
        heatmap = nn.functional.interpolate(
            heatmap, size=original_size, mode="bilinear", align_corners=False
        )
        return heatmap[0, 0].cpu().numpy().astype(np.float32)

    @torch.inference_mode()
    def _encode_text(self, prompt: str) -> Tensor:
        if prompt not in self._text_cache:
            tokens = self._clip.tokenize([prompt]).to(self.device)
            self._text_cache[prompt] = nn.functional.normalize(
                self._model.encode_text(tokens).float(), dim=-1
            )
        return self._text_cache[prompt]


def load_heatmap(path: str, size: tuple[int, int]) -> Tensor:
    heatmap = np.load(path).astype(np.float32)
    if heatmap.ndim != 2:
        raise ValueError(f"Expected a 2-D SCLIP heatmap, got {heatmap.shape}: {path}")
    heatmap = cv2.resize(heatmap, size, interpolation=cv2.INTER_LINEAR)
    return torch.from_numpy(heatmap).unsqueeze(0)
