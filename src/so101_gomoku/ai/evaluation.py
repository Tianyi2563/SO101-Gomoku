from so101_gomoku.contracts.models import Cell
from .common import DIRECTIONS


def evaluate(state):
    """Five-cell windows, including broken lines; score for side to move."""
    weights = (0, 1, 10, 150, 5000, 1_000_000)
    total = 0
    for r in range(state.size):
        for c in range(state.size):
            for dr, dc in DIRECTIONS:
                if not (0 <= r+4*dr < state.size and 0 <= c+4*dc < state.size):
                    continue
                values = [state.cells[r+i*dr][c+i*dc] for i in range(5)]
                own, other = values.count(state.next_player), values.count(Cell(-state.next_player))
                if not other:
                    total += weights[own]
                if not own:
                    total -= weights[other]
    return total
