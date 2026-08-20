from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from torch import Tensor
from torchvision.transforms import functional as TF


CLIP_MEAN = (0.48145466, 0.4578275, 0.40821073)
CLIP_STD = (0.26862954, 0.26130258, 0.27577711)

# Matches PEARL's official VOC21 configuration. The target class at index 15 is
# replaced by the runtime prompt when the prompt is not "person".
VOC21_CLASSES = (
    "sky, wall, tree, wood, grass, road, sea, river, mountain, sands, desk, "
    "bed, building, cloud, lamp, door, window, wardrobe, ceiling, shelf, "
    "curtain, stair, floor, hill, rail, fence",
    "aeroplane",
    "bicycle",
    "bird",
    "ship",
    "bottle",
    "bus",
    "car",
    "cat",
    "chair",
    "cow",
    "table",
    "dog",
    "horse",
    "motorbike",
    "person, person in shirt, person in jeans, person in dress, person in "
    "sweater, person in skirt, person in jacket",
    "pottedplant",
    "sheep",
    "sofa",
    "train",
    "television monitor, tv monitor, monitor, television, screen",
)
TARGET_CLASS_INDEX = 15


class PEARLHeatmapEncoder:
    """Training-free PEARL target-probability encoder.

    The implementation follows the official VOC21 settings, with the
    model-supported 224/112 sliding window enabled for distant targets.
    """

    def __init__(
        self,
        pearl_root: str,
        checkpoint: str = "ViT-B/16",
        device: torch.device | None = None,
        short_side: int = 336,
        crop_size: int = 224,
        stride: int = 112,
        logit_scale: float = 40.0,
        use_propagation: bool = True,
    ) -> None:
        self.device = device or torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        self.short_side = short_side
        self.crop_size = crop_size
        self.stride = stride
        self.logit_scale = logit_scale

        root = Path(pearl_root).expanduser().resolve()
        if not (root / "clip" / "model.py").is_file() or not (
            root / "pearl" / "prop.py"
        ).is_file():
            raise FileNotFoundError(
                f"Official PEARL source not found at {root}. Clone "
                "https://github.com/PGSmall/PEARL.git first."
            )

        sys.path.insert(0, str(root))
        try:
            import clip  # type: ignore[import-not-found]
            from pearl.prop import TLP  # type: ignore[import-not-found]
            from prompts.imagenet_template import (  # type: ignore[import-not-found]
                openai_imagenet_template,
            )
        finally:
            sys.path.pop(0)

        clip_path = Path(clip.__file__).resolve()
        if root not in clip_path.parents:
            raise ImportError(
                f"Loaded a conflicting clip package from {clip_path}; expected {root / 'clip'}"
            )

        self._clip = clip
        self._templates = openai_imagenet_template
        self._model, _ = clip.load(checkpoint, device=self.device, jit=False)
        self._model.eval()
        self._model.visual.set_params("reduced", "pearl")
        self._model.visual.set_attn_options(
            use_kk=True,
            alpha_kk=0.08,
            align_solver="polar",
            polar_iters=5,
        )
        for parameter in self._model.parameters():
            parameter.requires_grad_(False)

        self._propagation = TLP(grid=80).to(self.device) if use_propagation else None
        self._propagation_prompt: str | None = None
        self._query_cache: dict[str, tuple[Tensor, Tensor]] = {}

    @torch.inference_mode()
    def encode(self, rgb_path: str, prompt: str) -> np.ndarray:
        image = Image.open(rgb_path).convert("RGB")
        return self._encode_image(image, prompt)

    @torch.inference_mode()
    def prepare_prompt(self, prompt: str) -> None:
        """Cache text embeddings before the first camera frame arrives."""
        query_features, _ = self._encode_queries(prompt)
        if self._propagation is not None:
            self._propagation.bind_text(query_features)
            self._propagation_prompt = prompt

    @torch.inference_mode()
    def encode_rgb(self, rgb: np.ndarray, prompt: str) -> np.ndarray:
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"Expected HxWx3 RGB input, got {rgb.shape}")
        image = Image.fromarray(rgb.astype(np.uint8), "RGB")
        return self._encode_image(image, prompt)

    def _encode_image(self, image: Image.Image, prompt: str) -> np.ndarray:
        original_size = (image.height, image.width)
        image = self._resize_short_side(image)
        image_tensor = TF.to_tensor(image)
        image_tensor = TF.normalize(image_tensor, CLIP_MEAN, CLIP_STD)
        image_tensor = image_tensor.unsqueeze(0).to(
            self.device, dtype=self._model.dtype
        )

        query_features, query_indices = self._encode_queries(prompt)
        logits = self._forward_slide(image_tensor, query_features)
        if self._propagation is not None:
            if self._propagation_prompt != prompt:
                self._propagation.bind_text(query_features)
                self._propagation_prompt = prompt
            logits = self._propagation(
                image_tensor, logits.to(image_tensor.dtype)
            ).to(query_features.dtype)

        query_probabilities = (logits[0] * self.logit_scale).softmax(dim=0)
        class_probabilities = torch.stack(
            [
                query_probabilities[query_indices == class_index].amax(dim=0)
                for class_index in range(len(VOC21_CLASSES))
            ]
        )
        class_probabilities /= class_probabilities.sum(dim=0, keepdim=True)
        target = class_probabilities[TARGET_CLASS_INDEX][None, None]
        target = F.interpolate(
            target, size=original_size, mode="bilinear", align_corners=False
        )
        return target[0, 0].float().cpu().numpy().astype(np.float32)

    def _resize_short_side(self, image: Image.Image) -> Image.Image:
        width, height = image.size
        scale = self.short_side / min(width, height)
        return image.resize(
            (round(width * scale), round(height * scale)),
            Image.Resampling.BICUBIC,
        )

    @torch.inference_mode()
    def _encode_queries(self, prompt: str) -> tuple[Tensor, Tensor]:
        prompt = prompt.strip()
        if not prompt:
            raise ValueError("PEARL target prompt cannot be empty")
        if prompt not in self._query_cache:
            classes = list(VOC21_CLASSES)
            if prompt.casefold() != "person":
                classes[TARGET_CLASS_INDEX] = prompt

            names: list[str] = []
            indices: list[int] = []
            for class_index, class_names in enumerate(classes):
                aliases = class_names.split(", ")
                names.extend(aliases)
                indices.extend([class_index] * len(aliases))

            query_features = []
            for name in names:
                tokens = self._clip.tokenize(
                    [template(name) for template in self._templates]
                ).to(self.device)
                features = F.normalize(self._model.encode_text(tokens), dim=-1)
                features = F.normalize(features.mean(dim=0), dim=0)
                query_features.append(features)
            self._query_cache[prompt] = (
                torch.stack(query_features),
                torch.tensor(indices, dtype=torch.long, device=self.device),
            )
        return self._query_cache[prompt]

    def _forward_feature(self, image: Tensor, query_features: Tensor) -> Tensor:
        batch, _, height, width = image.shape
        tokens = self._model.encode_image(image, return_all=True)[:, 1:, :]
        tokens = F.normalize(tokens, dim=-1)
        text = F.normalize(query_features, dim=-1)
        logits = torch.bmm(
            tokens,
            text.unsqueeze(0).expand(batch, -1, -1).transpose(1, 2),
        )
        patch_size = int(self._model.visual.patch_size)
        logits = logits.permute(0, 2, 1).reshape(
            batch, -1, height // patch_size, width // patch_size
        )
        return F.interpolate(
            logits, size=(height, width), mode="bilinear", align_corners=False
        )

    def _forward_slide(self, image: Tensor, query_features: Tensor) -> Tensor:
        if self.crop_size <= 0:
            return self._forward_feature(image, query_features)

        batch, _, height, width = image.shape
        rows = max(height - self.crop_size + self.stride - 1, 0) // self.stride + 1
        columns = (
            max(width - self.crop_size + self.stride - 1, 0) // self.stride + 1
        )
        output = image.new_zeros(
            (batch, query_features.shape[0], height, width)
        )
        count = image.new_zeros((batch, 1, height, width))
        for row in range(rows):
            for column in range(columns):
                y2 = min(row * self.stride + self.crop_size, height)
                x2 = min(column * self.stride + self.crop_size, width)
                y1 = max(y2 - self.crop_size, 0)
                x1 = max(x2 - self.crop_size, 0)
                output[:, :, y1:y2, x1:x2] += self._forward_feature(
                    image[:, :, y1:y2, x1:x2], query_features
                )
                count[:, :, y1:y2, x1:x2] += 1
        return output / count


def load_pearl_heatmap(path: str, size: tuple[int, int]) -> Tensor:
    heatmap = np.load(path).astype(np.float32)
    if heatmap.ndim != 2:
        raise ValueError(f"Expected a 2-D PEARL heatmap, got {heatmap.shape}: {path}")
    heatmap = np.clip(heatmap, 0.0, 1.0)
    heatmap = cv2.resize(heatmap, size, interpolation=cv2.INTER_LINEAR)
    return torch.from_numpy(heatmap).unsqueeze(0)
