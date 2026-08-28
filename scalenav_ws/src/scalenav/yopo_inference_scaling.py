from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class YopoInferenceScaling:
    """Map a trained YOPO state contract to a requested inference speed."""

    training_speed_mps: float
    training_acceleration_mps2: float
    inference_speed_mps: float
    base_segment_time_s: float

    def __post_init__(self) -> None:
        values = (
            self.training_speed_mps,
            self.training_acceleration_mps2,
            self.inference_speed_mps,
            self.base_segment_time_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("YOPO inference scaling values must be finite and positive")

    @property
    def speed_ratio(self) -> float:
        return self.inference_speed_mps / self.training_speed_mps

    @property
    def acceleration_ratio(self) -> float:
        return self.speed_ratio * self.speed_ratio

    @property
    def inference_acceleration_mps2(self) -> float:
        return self.training_acceleration_mps2 * self.acceleration_ratio

    @property
    def segment_time_s(self) -> float:
        return self.base_segment_time_s / self.speed_ratio

    def model_input(self, observation):
        """Adapt physical PVA input for a TorchScript model frozen at training scale."""
        scaled = observation.clone()
        scaled[:, 0:3] /= self.speed_ratio
        scaled[:, 3:6] /= self.acceleration_ratio
        return scaled

    def physical_endstate(self, endstate):
        """Restore the configured inference PVA scale on TorchScript output."""
        scaled = endstate.clone()
        scaled[:, 3:6] *= self.speed_ratio
        scaled[:, 6:9] *= self.acceleration_ratio
        return scaled
