from so101_gomoku.contracts.models import Cell
from .common import empty_points, wins_at, DIRECTIONS


def point_score(state, row, col, color):
    """Local contiguous line lengths, open ends, and immediate wins."""
    if wins_at(state, row, col, color):
        return 1_000_000
    score = 0
    for dr, dc in DIRECTIONS:
        length, opened = 1, 0
        for sign in (-1, 1):
            r, c = row + sign * dr, col + sign * dc
            while 0 <= r < state.size and 0 <= c < state.size and state.cells[r][c] == color:
                length += 1
                r, c = r + sign * dr, c + sign * dc
            opened += int(0 <= r < state.size and 0 <= c < state.size and state.cells[r][c] == Cell.EMPTY)
        score += (0, 2, 15, 150, 5000)[min(length, 4)] * opened
    return score


def candidates(state, radius=2):
    if type(radius) is not int or radius < 1:
        raise ValueError('Positive integer radius required')
    stones = [(r, c) for r in range(state.size) for c in range(state.size) if state.cells[r][c] != Cell.EMPTY]
    if not stones:
        return [(state.size // 2, state.size // 2)]
    points = set()
    for r, c in stones:
        for rr in range(max(0, r-radius), min(state.size, r+radius+1)):
            for cc in range(max(0, c-radius), min(state.size, c+radius+1)):
                if state.cells[rr][cc] == Cell.EMPTY:
                    points.add((rr, cc))
    center = (state.size - 1) / 2
    def rank(point):
        r, c = point
        attack = point_score(state, r, c, state.next_player)
        defense = point_score(state, r, c, Cell(-state.next_player))
        return (-max(attack, defense), -(attack+defense), abs(r-center)+abs(c-center), r, c)
    return sorted(points or empty_points(state), key=rank)
