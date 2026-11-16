"""Adapted from LeRobot Chess BoardModel (Apache-2.0).

See THIRD_PARTY_NOTICES.md. Intersection coordinates replace square centers.
Board axes: X follows col, Y follows row; base transform defines orientation.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class BoardGeometry:
    size: int
    spacing_m: float

    def __post_init__(self):
        if type(self.size) is not int or self.size < 5:
            raise ValueError("Board size must be >= 5")
        if not np.isfinite(self.spacing_m) or self.spacing_m <= 0:
            raise ValueError("Positive finite spacing required")

    def intersection(self, row, col):
        if type(row) is not int or type(col) is not int or not (0 <= row < self.size and 0 <= col < self.size):
            raise ValueError("Invalid intersection")
        return np.array([col * self.spacing_m, row * self.spacing_m, 0.0])

    @property
    def span_m(self):
        return (self.size - 1) * self.spacing_m
