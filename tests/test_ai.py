import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
from so101_gomoku.ai.candidates import candidates
from so101_gomoku.contracts.models import Cell, Move
from so101_gomoku.game.board import GameState


def position(stones, next_player=Cell.BLACK, size=9):
    cells = [[Cell.EMPTY]*size for _ in range(size)]
    for r, c, color in stones:
        cells[r][c] = color
    return GameState(tuple(map(tuple, cells)), next_player)


class AITests(unittest.TestCase):
    def test_center_and_legal(self):
        state = GameState.empty(9)
        decision = AlphaBetaEngine(max_depth=1).choose_move(state, 1)
        self.assertEqual((decision.move.row, decision.move.col), (4, 4))
        state.play(decision.move)

    def test_win_over_defense(self):
        state = position([(0, c, Cell.BLACK) for c in range(4)] + [(8, c, Cell.WHITE) for c in range(4)])
        d = AlphaBetaEngine().choose_move(state)
        self.assertEqual(d.move, Move(0, 4, Cell.BLACK))
        self.assertEqual(d.reason, 'win_now')

    def test_block_edge_and_white(self):
        for attacker in (Cell.WHITE, Cell.BLACK):
            state = position([(r, 8, attacker) for r in range(4)], Cell(-attacker))
            d = AlphaBetaEngine().choose_move(state)
            self.assertEqual(d.move, Move(4, 8, Cell(-attacker)))

    def test_timeout_fallback_and_no_mutation(self):
        state = GameState.empty(9).play(Move(4, 4, Cell.BLACK))
        original = state.cells
        d = AlphaBetaEngine(node_limit=1).choose_move(state, 1)
        self.assertEqual(d.completed_depth, 0)
        self.assertEqual(d.reason, 'budget_fallback')
        state.play(d.move)
        self.assertEqual(state.cells, original)
        tiny = AlphaBetaEngine().choose_move(state, 1e-9)
        state.play(tiny.move)

    def test_terminal_rejected_and_history(self):
        state = position([(0, c, Cell.BLACK) for c in range(5)])
        with self.assertRaises(ValueError):
            AlphaBetaEngine().choose_move(state)
        move = Move(2, 3, Cell.BLACK)
        self.assertEqual(GameState.empty(9).play(move).last_move, move)

    def test_candidates_cover_edges_and_exclude_occupied(self):
        state = GameState.empty(9).play(Move(0, 0, Cell.BLACK))
        points = candidates(state)
        self.assertIn((0, 1), points)
        self.assertNotIn((0, 0), points)
        self.assertTrue(all(0 <= r < 9 and 0 <= c < 9 for r, c in points))


if __name__ == '__main__':
    unittest.main()
