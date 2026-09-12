from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace

import numpy as np

from text_tracker.ros_heatmap import (
    LatestValue,
    colorize_heatmap,
    decode_color_image,
    encode_float_image,
)


def image_message(
    data: bytes, *, encoding: str, width: int, height: int, step: int
) -> SimpleNamespace:
    return SimpleNamespace(
        data=data, encoding=encoding, width=width, height=height, step=step
    )


class ColorImageTest(unittest.TestCase):
    def test_decodes_rgb8_with_row_padding(self) -> None:
        message = image_message(
            bytes([1, 2, 3, 4, 5, 6, 99, 99]),
            encoding="rgb8",
            width=2,
            height=1,
            step=8,
        )
        np.testing.assert_array_equal(
            decode_color_image(message),
            np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8),
        )

    def test_converts_bgr8_to_rgb(self) -> None:
        message = image_message(
            bytes([3, 2, 1]), encoding="bgr8", width=1, height=1, step=3
        )
        np.testing.assert_array_equal(
            decode_color_image(message), np.array([[[1, 2, 3]]], dtype=np.uint8)
        )

    def test_rejects_short_payload(self) -> None:
        message = image_message(
            bytes([1, 2]), encoding="rgb8", width=1, height=1, step=3
        )
        with self.assertRaisesRegex(ValueError, "payload is shorter"):
            decode_color_image(message)


class FloatImageTest(unittest.TestCase):
    def test_serializes_32fc1_layout(self) -> None:
        heatmap = np.array([[0.25, 0.5], [0.75, 1.0]], dtype=np.float32)
        payload = encode_float_image(heatmap)
        self.assertEqual((payload.width, payload.height, payload.step), (2, 2, 8))
        self.assertEqual(payload.is_bigendian, sys.byteorder == "big")
        self.assertEqual(len(payload.data), 16)
        np.testing.assert_array_equal(np.frombuffer(payload.data, np.float32), heatmap.ravel())


class HeatmapColorTest(unittest.TestCase):
    def test_uses_five_dominant_probability_colors(self) -> None:
        heatmap = np.array([[0.1, 0.3, 0.5, 0.7, 0.9]], dtype=np.float32)
        color = colorize_heatmap(heatmap)

        np.testing.assert_array_equal(
            color,
            np.array(
                [
                    [
                        [175, 64, 30],
                        [215, 185, 35],
                        [95, 190, 45],
                        [55, 200, 245],
                        [45, 45, 220],
                    ]
                ],
                dtype=np.uint8,
            ),
        )

    def test_blends_smoothly_between_dominant_colors(self) -> None:
        color = colorize_heatmap(
            np.array([[0.18, 0.19, 0.20, 0.21, 0.22]], dtype=np.float32)
        )[0]

        np.testing.assert_array_equal(color[0], [175, 64, 30])
        np.testing.assert_array_equal(color[-1], [215, 185, 35])
        self.assertTrue(np.all(np.diff(color.astype(np.int16), axis=0) >= 0))

    def test_same_probability_keeps_same_color_across_frames(self) -> None:
        first = colorize_heatmap(
            np.array([[0.1, 0.2], [0.0, 0.0]], dtype=np.float32)
        )
        second = colorize_heatmap(
            np.array([[0.1, 0.9], [0.0, 0.0]], dtype=np.float32)
        )

        np.testing.assert_array_equal(first[0, 0], second[0, 0])


class LatestValueTest(unittest.TestCase):
    def test_new_value_replaces_stale_value(self) -> None:
        slot: LatestValue[int] = LatestValue()
        self.assertFalse(slot.put(1))
        self.assertTrue(slot.put(2))
        self.assertEqual(slot.get(), 2)


if __name__ == "__main__":
    unittest.main()
