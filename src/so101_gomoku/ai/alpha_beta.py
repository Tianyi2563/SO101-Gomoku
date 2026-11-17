"""Iterative deepening negamax with alpha-beta; independent of robot geometry."""
import time
from so101_gomoku.contracts.models import Move
from .common import Decision, tactical_move, validate_search
from .candidates import candidates
from .evaluation import evaluate


class SearchExpired(Exception):
    pass


class AlphaBetaEngine:
    def __init__(self, max_depth=3, max_candidates=12, node_limit=None):
        if type(max_depth) is not int or max_depth < 1 or type(max_candidates) is not int or max_candidates < 1:
            raise ValueError('Positive integer depth and candidate limit required')
        if node_limit is not None and (type(node_limit) is not int or node_limit < 1):
            raise ValueError('Positive node limit required')
        self.max_depth, self.max_candidates, self.node_limit = max_depth, max_candidates, node_limit

    def choose_move(self, state, time_limit_s=.5):
        validate_search(state, time_limit_s)
        start = time.monotonic()
        tactical = tactical_move(state)
        if tactical:
            return Decision(tactical[0], 'alphabeta', tactical[1], elapsed_s=time.monotonic()-start)
        root_points = candidates(state)
        best = Move(*root_points[0], state.next_player)
        best_score, completed_depth, nodes = 0., 0, 0
        deadline = start + time_limit_s

        def check():
            if time.monotonic() >= deadline or (self.node_limit is not None and nodes >= self.node_limit):
                raise SearchExpired

        def search(board, depth, alpha, beta):
            nonlocal nodes
            check()
            nodes += 1
            winner = board.winner()
            if winner is not None:
                return 10_000_000 + depth if winner == board.next_player else -10_000_000 - depth
            if board.finished:
                return 0
            if depth == 0:
                return evaluate(board)
            value = float('-inf')
            for r, c in candidates(board)[:self.max_candidates]:
                check()
                child = board.play(Move(r, c, board.next_player))
                value = max(value, -search(child, depth-1, -beta, -alpha))
                alpha = max(alpha, value)
                if alpha >= beta:
                    break
            return value

        for depth in range(1, self.max_depth+1):
            try:
                iteration_best, iteration_score = best, float('-inf')
                alpha = float('-inf')
                ordered = [(best.row, best.col)] + [p for p in root_points if p != (best.row, best.col)]
                for r, c in ordered[:self.max_candidates]:
                    check()
                    move = Move(r, c, state.next_player)
                    value = -search(state.play(move), depth-1, float('-inf'), -alpha)
                    if value > iteration_score:
                        iteration_best, iteration_score = move, value
                    alpha = max(alpha, value)
            except SearchExpired:
                break
            best, best_score, completed_depth = iteration_best, iteration_score, depth
        return Decision(best, 'alphabeta', 'search' if completed_depth else 'budget_fallback',
                        best_score, completed_depth, nodes, time.monotonic()-start)


def get_best_move(state, time_limit_s=.5):
    return AlphaBetaEngine().choose_move(state, time_limit_s).move
