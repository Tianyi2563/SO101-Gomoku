import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
import unittest
import tempfile
import subprocess
import json
import numpy as np
import cv2
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
from so101_gomoku.contracts.models import Cell,Move
from so101_gomoku.game.board import GameState
from so101_gomoku.recording.session import SessionJournal,replay_session,state_from_dict
from so101_gomoku.orchestration.controller import GameController,Phase
from so101_gomoku.orchestration.simulation import SimulatedTable
from tests.test_vision import fixture


class OfflineRegressionTests(unittest.TestCase):
    def test_ai_all_four_win_directions_for_both_colors(self):
        for color in (Cell.BLACK,Cell.WHITE):
            for dr,dc in ((0,1),(1,0),(1,1),(1,-1)):
                cells=[[Cell.EMPTY]*10 for _ in range(10)]
                row,col=(0,9) if dc==-1 else (0,0)
                for i in range(4):cells[row+i*dr][col+i*dc]=color
                state=GameState(tuple(map(tuple,cells)),color)
                d=AlphaBetaEngine().choose_move(state,.01)
                self.assertEqual(state.play(d.move).winner(),color)

    def test_ai_random_legal_positions_do_not_mutate_state(self):
        rng=np.random.default_rng(17)
        for _ in range(12):
            state=GameState.empty()
            for _ in range(8):
                empty=[(r,c) for r in range(10) for c in range(10) if state.cells[r][c]==Cell.EMPTY]
                r,c=empty[int(rng.integers(len(empty)))];state=state.play(Move(r,c,state.next_player))
            before=state.cells
            d=AlphaBetaEngine(node_limit=5).choose_move(state,.02)
            self.assertEqual(state.play(d.move).version,9)
            self.assertEqual(state.cells,before)

    def test_white_on_white_is_not_claimed_reliable_white(self):
        cfg,empty,_=fixture(10)
        # White paper reference, dark grid; drawing is not a real camera photo.
        empty[:]=255
        for i in range(10):
            cv2.line(empty,cfg.point(i,0),cfg.point(i,9),(45,45,45),2)
            cv2.line(empty,cfg.point(0,i),cfg.point(9,i),(45,45,45),2)
        from so101_gomoku.vision.board_detector import IntersectionDetector
        detector=IntersectionDetector(empty,[cfg.point(0,0),cfg.point(0,9),cfg.point(9,9),cfg.point(9,0)],'paper',cfg)
        image=empty.copy();cv2.circle(image,cfg.point(4,4),20,(255,255,255),-1)
        result=detector.detect(image,frame_id='paper',captured_at=1)
        self.assertEqual(result.observation.cells[4][4],Cell.UNKNOWN)

    def test_real_process_exit_releases_lock_and_preserves_pending(self):
        code='''
import sys,os
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/'src'))
from so101_gomoku.recording.session import SessionJournal,state_to_dict
from so101_gomoku.recording.storage import atomic_json
from so101_gomoku.orchestration.controller import GameController
from so101_gomoku.orchestration.simulation import SimulatedTable
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
table=SimulatedTable();journal=SessionJournal(sys.argv[1],configuration={'mode':'SIMULATED'})
c=GameController(table,AlphaBetaEngine(max_depth=1),table,table.calibration_id,clock=table.clock,journal=journal)
c.start();execute=table.execute
def die(task):
    execute(task)
    atomic_json(Path(sys.argv[1])/'simulated_physical_board.json',state_to_dict(table.board))
    os._exit(17)
table.execute=die
c.run_turn()
'''
        with tempfile.TemporaryDirectory() as d:
            child=subprocess.run([sys.executable,'-X','utf8','-c',code,d],cwd=ROOT,capture_output=True)
            self.assertEqual(child.returncode,17,child.stderr.decode('utf-8',errors='replace'))
            physical=state_from_dict(json.loads((Path(d)/'simulated_physical_board.json').read_text(encoding='utf-8')))
            table=SimulatedTable(physical)
            with SessionJournal(d) as journal:
                restored=GameController.resume_from_journal(journal,table,AlphaBetaEngine(),table,
                    table.calibration_id,expected_configuration={'mode':'SIMULATED'},clock=table.clock)
                self.assertEqual(restored.phase,Phase.PAUSED)
                self.assertEqual(restored.recover(robot_stopped=True),Phase.WAIT_HUMAN)
                self.assertEqual(restored.state.version,1)
                self.assertEqual(table.tasks,[])
                with self.assertRaises(ValueError):
                    GameController.resume_from_journal(journal,table,AlphaBetaEngine(),table,
                        table.calibration_id,expected_configuration={'spacing_m':.02},clock=table.clock)
