"""Engine-neutral decisions and tactical helpers for freestyle Gomoku."""
from dataclasses import dataclass
from math import isfinite
from so101_gomoku.contracts.models import Cell, Move

DIRECTIONS = ((0, 1), (1, 0), (1, 1), (1, -1))


@dataclass(frozen=True)
class Decision:
    move: Move
    engine: str
    reason: str
    score: float = 0.
    completed_depth: int = 0
    nodes: int = 0
    elapsed_s: float = 0.


def empty_points(state):
    return [(r, c) for r in range(state.size) for c in range(state.size) if state.cells[r][c] == Cell.EMPTY]


def wins_at(state, row, col, color):
    if state.cells[row][col] != Cell.EMPTY:
        return False
    for dr, dc in DIRECTIONS:
        count = 1
        for sign in (-1, 1):
            r, c = row + sign * dr, col + sign * dc
            while 0 <= r < state.size and 0 <= c < state.size and state.cells[r][c] == color:
                count += 1
                r, c = r + sign * dr, c + sign * dc
        if count >= 5:
            return True
    return False


def tactical_move(state):
    """Win now first; block one immediate threat otherwise (multiple may be lost)."""
    points = empty_points(state)
    for color, reason in ((state.next_player, 'win_now'), (Cell(-state.next_player), 'block_immediate_win')):
        for r, c in points:
            if wins_at(state, r, c, color):
                return Move(r, c, state.next_player), reason
    return None


def validate_search(state, time_limit_s):
    if state.finished:
        raise ValueError('Cannot choose a move after game end')
    if not isfinite(time_limit_s) or time_limit_s <= 0:
        raise ValueError('Positive finite search time required')
