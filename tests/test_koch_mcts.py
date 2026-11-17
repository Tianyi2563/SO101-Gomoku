import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
import numpy as np
import torch
from so101_gomoku.ai.koch_mcts import KochMCTSEngine, encode_board
from so101_gomoku.contracts.models import Cell, Move
from so101_gomoku.game.board import GameState

MODEL = Path(__file__).resolve().parents[1] / 'assets/models/koch/Vanilla_MCTS.model'


class KochTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)  # Explicit test-process setting, not library side effect.
        cls.engine = KochMCTSEngine(MODEL, n_playouts=8)

    def test_input_encoding_and_no_hardware_mask(self):
        state = GameState.empty(9).play(Move(0, 8, Cell.BLACK))
        planes = encode_board(state)
        self.assertEqual(planes.shape, (5, 9, 9))
        self.assertEqual(planes[1, 8, 8], 1)
        self.assertEqual(planes[2, 8, 8], 1)
        self.assertEqual(planes[4].sum(), 0)

    def test_real_model_and_search(self):
        state = GameState.empty(9).play(Move(4, 4, Cell.BLACK))
        priors, value = self.engine.policy_value(state)
        self.assertEqual(len(priors), 80)
        self.assertAlmostEqual(sum(p for _, p in priors), 1)
        self.assertTrue(-1 <= value <= 1)
        self.assertIn((0, 0), [point for point, _ in priors])
        self.assertNotIn((4, 4), [point for point, _ in priors])
        decision = self.engine.choose_move(state, 2)
        self.assertEqual(decision.engine, 'koch-mcts')
        self.assertEqual(decision.nodes, 8)
        state.play(decision.move)
        self.assertEqual(state.version, 1)

    def test_tactical_and_board_size(self):
        cells = [[Cell.EMPTY]*9 for _ in range(9)]
        cells[0][:4] = [Cell.BLACK]*4
        self.assertEqual(self.engine.choose_move(GameState(tuple(map(tuple, cells)))).move, Move(0, 4, Cell.BLACK))
        with self.assertRaises(ValueError):
            self.engine.choose_move(GameState.empty(5))
        with self.assertRaises(FileNotFoundError):
            KochMCTSEngine(MODEL.with_name('missing.model'))


if __name__ == '__main__':
    unittest.main()
