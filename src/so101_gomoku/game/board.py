"""Freestyle rules; winner scan adapted from Koch VanillaBoard.has_a_winner.

Upstream MIT, Copyright (c) 2024 hri5-boardgame-robot.
See THIRD_PARTY_NOTICES.md. Removed hardware masks, forbidden moves and NN state.
"""
from dataclasses import dataclass
from so101_gomoku.contracts.models import Cell, Move


@dataclass(frozen=True)
class GameState:
    cells: tuple[tuple[Cell, ...], ...]
    next_player: Cell = Cell.BLACK
    version: int = 0
    last_move: Move | None = None

    def __post_init__(self):
        n = len(self.cells)
        if n < 5 or any(len(row) != n for row in self.cells):
            raise ValueError("Expected square board of size >= 5")
        if self.next_player not in (Cell.BLACK, Cell.WHITE):
            raise ValueError("Invalid next player")
        if any(c not in (Cell.EMPTY, Cell.BLACK, Cell.WHITE) for row in self.cells for c in row):
            raise ValueError("Unknown cells cannot enter confirmed game state")

    @classmethod
    def empty(cls, size=10):
        return cls(tuple(tuple(Cell.EMPTY for _ in range(size)) for _ in range(size)))

    @property
    def size(self):
        return len(self.cells)

    def winner(self):
        for r in range(self.size):
            for c in range(self.size):
                color = self.cells[r][c]
                if color == Cell.EMPTY:
                    continue
                for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
                    end_r, end_c = r + 4 * dr, c + 4 * dc
                    if 0 <= end_r < self.size and 0 <= end_c < self.size:
                        if all(self.cells[r + i * dr][c + i * dc] == color for i in range(5)):
                            return color
        return None

    @property
    def finished(self):
        return self.winner() is not None or all(c != Cell.EMPTY for row in self.cells for c in row)

    def play(self, move: Move):
        if self.finished:
            raise ValueError("Game has ended")
        if move.color != self.next_player:
            raise ValueError("Wrong player")
        if type(move.row) is not int or type(move.col) is not int:
            raise ValueError("Integer row/col required")
        if not (0 <= move.row < self.size and 0 <= move.col < self.size):
            raise ValueError("Move outside board")
        if self.cells[move.row][move.col] != Cell.EMPTY:
            raise ValueError("Cell occupied")
        cells = [list(row) for row in self.cells]
        cells[move.row][move.col] = move.color
        return GameState(tuple(tuple(row) for row in cells), Cell(-self.next_player), self.version + 1, move)
