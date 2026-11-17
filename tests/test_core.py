import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
import numpy as np
from so101_gomoku.contracts.models import Cell, Move, BoardObservation
from so101_gomoku.game.board import GameState
from so101_gomoku.game.reconciliation import detect_single_move, verify_placement
from so101_gomoku.calibration.board_geometry import BoardGeometry
from so101_gomoku.calibration.transforms import fit_board_to_base, apply_transform, validation_errors


def observe(state):
    return BoardObservation(state.cells, tuple((1.,) * state.size for _ in state.cells), 'f1', 1., 'cal1')


class CoreTests(unittest.TestCase):
    def test_moves_and_immutability(self):
        state = GameState.empty(9)
        after = state.play(Move(0, 0, Cell.BLACK))
        self.assertEqual(state.cells[0][0], Cell.EMPTY)
        self.assertEqual(after.version, 1)
        for move in (Move(0, 0, Cell.WHITE), Move(-1, 1, Cell.WHITE), Move(0, 9, Cell.WHITE), Move(1, 1, Cell.BLACK)):
            with self.assertRaises(ValueError):
                after.play(move)

    def test_all_win_directions_and_long_line(self):
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            cells = [[Cell.EMPTY] * 9 for _ in range(9)]
            for i in range(6):
                cells[i * dr][7 + i * dc if dc == -1 else i * dc] = Cell.BLACK
            state = GameState(tuple(map(tuple, cells)))
            self.assertEqual(state.winner(), Cell.BLACK)
            with self.assertRaises(ValueError):
                state.play(Move(8, 8, Cell.BLACK))

    def test_reconcile(self):
        state = GameState.empty(9)
        move = Move(3, 4, Cell.BLACK)
        obs = observe(state.play(move))
        self.assertEqual(detect_single_move(state, obs), move)
        self.assertEqual(verify_placement(state, move, obs).version, 1)
        with self.assertRaises(ValueError):
            verify_placement(state, Move(2, 2, Cell.BLACK), obs)
        cells = [list(row) for row in obs.cells]
        cells[0][0] = Cell.UNKNOWN
        with self.assertRaises(ValueError):
            detect_single_move(state, BoardObservation(tuple(map(tuple, cells)), obs.confidence, 'f', 1., 'cal1'))

    def test_geometry_and_calibration(self):
        board = BoardGeometry(9, .022)
        np.testing.assert_allclose(board.intersection(0, 0), [0, 0, 0])
        self.assertAlmostEqual(board.span_m, .176)
        src = np.array([[0, 0, 0], [.176, 0, 0], [0, .176, 0], [.176, .176, 0]])
        t = np.eye(4)
        t[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        t[:3, 3] = [.1, -.2, .05]
        fit = fit_board_to_base(src, apply_transform(t, src))
        np.testing.assert_allclose(fit.matrix, t, atol=1e-12)
        heldout = [[.04, .03, 0]]
        self.assertLess(validation_errors(fit.matrix, heldout, apply_transform(t, heldout))[0], 1e-12)
        with self.assertRaises(ValueError):
            fit_board_to_base([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [[0, 0, 0], [1, 0, 0], [2, 0, 0]])

    def test_full_board_draw_and_multiple_changes(self):
        cells = tuple(tuple(Cell.BLACK if (r + 2 * c) % 4 < 2 else Cell.WHITE
                            for c in range(5)) for r in range(5))
        state = GameState(cells)
        self.assertIsNone(state.winner())
        self.assertTrue(state.finished)
        empty = GameState.empty(9)
        changed = empty.play(Move(1, 1, Cell.BLACK)).play(Move(1, 2, Cell.WHITE))
        with self.assertRaises(ValueError):
            detect_single_move(empty, observe(changed))


if __name__ == '__main__':
    unittest.main()
