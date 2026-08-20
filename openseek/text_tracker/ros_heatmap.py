from __future__ import annotations

import sys
import threading
from dataclasses import dataclass
from typing import Generic, TypeVar

import numpy as np


T = TypeVar("T")


class LatestValue(Generic[T]):
    """Thread-safe single-item queue that drops superseded camera frames."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._value: T | None = None
        self._closed = False

    def put(self, value: T) -> bool:
        with self._condition:
            if self._closed:
                return False
            replaced = self._value is not None
            self._value = value
            self._condition.notify_all()
            return replaced

    def get(self, timeout: float | None = None) -> T | None:
        with self._condition:
            if self._value is None and not self._closed:
                self._condition.wait(timeout)
            if self._value is None:
                return None
            value = self._value
            self._value = None
            return value

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._value = None
            self._condition.notify_all()


@dataclass(frozen=True)
class FloatImagePayload:
    height: int
    width: int
    step: int
    is_bigendian: bool
    data: bytes


def decode_color_image(message: object) -> np.ndarray:
    """Decode a ROS-like rgb8/bgr8 Image into a contiguous RGB uint8 array."""

    encoding = str(getattr(message, "encoding", "")).lower()
    if encoding not in {"rgb8", "bgr8"}:
        raise ValueError(
            f"unsupported color encoding: {encoding!r}; expected 'rgb8' or 'bgr8'"
        )

    height = int(getattr(message, "height"))
    width = int(getattr(message, "width"))
    step = int(getattr(message, "step"))
    if height <= 0 or width <= 0:
        raise ValueError(f"invalid color image size: {width}x{height}")
    if step < width * 3:
        raise ValueError("color image step is shorter than width*3")

    values = np.frombuffer(getattr(message, "data"), dtype=np.uint8)
    expected = height * step
    if values.size < expected:
        raise ValueError("color image payload is shorter than height*step")
    image = values[:expected].reshape(height, step)[:, : width * 3]
    image = image.reshape(height, width, 3)
    if encoding == "bgr8":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def encode_float_image(heatmap: np.ndarray) -> FloatImagePayload:
    """Serialize a two-dimensional heatmap using the ROS 32FC1 contract."""

    values = np.asarray(heatmap, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f"expected a 2-D heatmap, got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("heatmap contains non-finite values")
    values = np.ascontiguousarray(values)
    height, width = values.shape
    return FloatImagePayload(
        height=height,
        width=width,
        step=width * values.dtype.itemsize,
        is_bigendian=sys.byteorder == "big",
        data=values.tobytes(),
    )


def _colorize_unit_interval(values: np.ndarray) -> np.ndarray:
    """Apply dominant blue/cyan/green/yellow/red colors continuously."""

    clipped = np.clip(values, 0.0, 1.0)
    transition_points = np.array(
        (0.0, 0.18, 0.22, 0.38, 0.42, 0.58, 0.62, 0.78, 0.82, 1.0),
        dtype=np.float32,
    )
    colors = np.array(
        [
            (175, 64, 30),
            (175, 64, 30),    # blue plateau
            (215, 185, 35),
            (215, 185, 35),   # cyan plateau
            (95, 190, 45),
            (95, 190, 45),    # green plateau
            (55, 200, 245),
            (55, 200, 245),   # yellow plateau
            (45, 45, 220),
            (45, 45, 220),    # red plateau
        ],
        dtype=np.float32,
    )
    output = np.empty((*clipped.shape, 3), dtype=np.uint8)
    for channel in range(3):
        output[..., channel] = np.rint(
            np.interp(clipped, transition_points, colors[:, channel])
        ).astype(np.uint8)
    return output


def colorize_heatmap(heatmap: np.ndarray) -> np.ndarray:
    """Render five dominant probability colors with smooth transitions."""

    values = np.asarray(heatmap, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f"expected a 2-D heatmap, got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError("heatmap contains non-finite values")

    return _colorize_unit_interval(values)
