import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import unittest
import cv2
from so101_gomoku.game.board import GameState
from so101_gomoku.contracts.models import Move, Cell
from so101_gomoku.vision.board_detector import VisionConfig
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
from so101_gomoku.orchestration.controller import GameController, Phase
from so101_gomoku.orchestration.simulation import SimulatedTable
from tests.test_vision import fixture


class DimensionsTests(unittest.TestCase):
    def test_default_and_last_intersection(self):
        self.assertEqual(GameState.empty().size,10)
        self.assertEqual(VisionConfig().intersections,10)
        state=GameState.empty().play(Move(9,9,Cell.BLACK))
        self.assertEqual(state.cells[9][9],Cell.BLACK)
        with self.assertRaises(ValueError): state.play(Move(10,0,Cell.WHITE))

    def test_last_intersection_classification(self):
        cfg,empty,detector=fixture(10)
        image=empty.copy()
        cv2.circle(image,cfg.point(9,9),20,(20,20,20),-1)
        self.assertEqual(detector.detect(image,frame_id='1',captured_at=1).observation.cells[9][9],Cell.BLACK)

    def test_actual_board_robot_black_first_then_human_white(self):
        table=SimulatedTable()
        c=GameController(table,AlphaBetaEngine(max_depth=1),table,table.calibration_id,clock=table.clock)
        self.assertEqual(c.start(),Phase.THINKING)
        self.assertEqual(c.run_turn(),Phase.WAIT_HUMAN)
        self.assertEqual(c.state.last_move.color,Cell.BLACK)
        table.human_move(Move(9,9,Cell.WHITE))
        self.assertEqual(c.run_turn(),Phase.WAIT_HUMAN)
        self.assertEqual(c.state.size,10)
        self.assertEqual(c.state.version,3)


if __name__=='__main__': unittest.main()
