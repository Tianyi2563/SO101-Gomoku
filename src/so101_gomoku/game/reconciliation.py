"""Validate full observations without mutating confirmed state."""
from so101_gomoku.contracts.models import Cell, Move


def confirmed_cells(observation, min_confidence=0.9):
    if not 0 <= min_confidence <= 1:
        raise ValueError("Invalid confidence threshold")
    for r, row in enumerate(observation.cells):
        for c, cell in enumerate(row):
            if cell == Cell.UNKNOWN or observation.confidence[r][c] < min_confidence:
                raise ValueError("Observation is incomplete or uncertain")
    return observation.cells


def detect_single_move(state, observation, min_confidence=0.9):
    cells = confirmed_cells(observation, min_confidence)
    if len(cells) != state.size:
        raise ValueError("Board size mismatch")
    changes = [(r, c) for r in range(state.size) for c in range(state.size)
               if cells[r][c] != state.cells[r][c]]
    if len(changes) != 1:
        raise ValueError("Expected exactly one changed cell")
    r, c = changes[0]
    move = Move(r, c, cells[r][c])
    state.play(move)  # validate, do not commit
    return move


def verify_placement(state, move, observation, min_confidence=0.9):
    expected = state.play(move)
    if confirmed_cells(observation, min_confidence) != expected.cells:
        raise ValueError("Observed board does not match expected placement")
    return expected
