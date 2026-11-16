"""Shared immutable messages. Observations never modify the game directly."""
from dataclasses import dataclass
from enum import IntEnum
from math import isfinite


class Cell(IntEnum):
    WHITE = -1
    EMPTY = 0
    BLACK = 1
    UNKNOWN = 2


@dataclass(frozen=True)
class Move:
    row: int
    col: int
    color: Cell


@dataclass(frozen=True)
class BoardObservation:
    cells: tuple[tuple[Cell, ...], ...]
    confidence: tuple[tuple[float, ...], ...]
    frame_id: str
    captured_at: float
    calibration_id: str

    def __post_init__(self):
        n = len(self.cells)
        if not n or any(len(row) != n for row in self.cells):
            raise ValueError("Expected square cells")
        if len(self.confidence) != n or any(len(row) != n for row in self.confidence):
            raise ValueError("Confidence shape mismatch")
        if any(c not in tuple(Cell) for row in self.cells for c in row):
            raise ValueError("Invalid observation cell")
        if any(not isfinite(v) or not 0 <= v <= 1 for row in self.confidence for v in row):
            raise ValueError("Confidence must be finite and in [0, 1]")


@dataclass(frozen=True)
class ExecutionResult:
    task_id: str
    completed: bool
    stage: str
    reason: str = ""
    visual_confirmed: bool = False

