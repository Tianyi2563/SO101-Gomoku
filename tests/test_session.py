import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import unittest
import tempfile
import json
import numpy as np
from so101_gomoku.recording.session import SessionJournal,replay_session,validate_records
from so101_gomoku.orchestration.controller import GameController,Phase
from so101_gomoku.orchestration.simulation import SimulatedTable
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
from so101_gomoku.contracts.models import Cell,Move


class SessionTests(unittest.TestCase):
    def make(self,path):
        table=SimulatedTable()
        journal=SessionJournal(path,configuration={'mode':'SIMULATED','spacing_m':.021})
        c=GameController(table,AlphaBetaEngine(max_depth=1),table,table.calibration_id,
                         clock=table.clock,journal=journal,search_time_s=.03)
        self.assertEqual(c.start(),Phase.THINKING)
        return table,journal,c

    def test_persist_round_replay_and_restart_requires_reconciliation(self):
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            self.assertEqual(c.run_turn(),Phase.WAIT_HUMAN)
            self.assertEqual(replay_session(d)['checkpoint']['state']['version'],1)
            self.assertEqual(replay_session(d)['checkpoint']['phase'],c.phase.value)
            journal.close()
            with SessionJournal(d) as reopened:
                restored=GameController.resume_from_journal(reopened,table,AlphaBetaEngine(),table,
                                                            table.calibration_id,clock=table.clock)
                self.assertEqual(restored.phase,Phase.PAUSED)
                self.assertEqual(len(table.tasks),1)
                with self.assertRaises(RuntimeError):restored.run_turn()
                self.assertEqual(restored.recover(robot_stopped=True),Phase.WAIT_HUMAN)
                self.assertEqual(len(table.tasks),1)

    def test_crash_after_motion_before_result_never_reexecutes(self):
        class SimulatedCrash(BaseException):pass
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            original=table.execute
            def execute(task):
                result=original(task)
                raise SimulatedCrash('process died after physical placement')
            table.execute=execute
            with self.assertRaises(SimulatedCrash):c.run_turn()
            journal.close()
            saved=replay_session(d)
            self.assertIsNotNone(saved['checkpoint']['pending_task'])
            self.assertEqual(saved['checkpoint']['state']['version'],0)
            with SessionJournal(d) as reopened:
                restored=GameController.resume_from_journal(reopened,table,AlphaBetaEngine(),table,
                                                            table.calibration_id,clock=table.clock)
                self.assertEqual(restored.recover(robot_stopped=True),Phase.WAIT_HUMAN)
                self.assertEqual(restored.state.version,1)
                self.assertEqual(len(table.tasks),1)

    def test_crash_before_motion_clears_pending_but_does_not_retry(self):
        class Crash(BaseException):pass
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            table.execute=lambda *_: (_ for _ in ()).throw(Crash())
            with self.assertRaises(Crash):c.run_turn()
            journal.close()
            with SessionJournal(d) as reopened:
                restored=GameController.resume_from_journal(reopened,table,AlphaBetaEngine(),table,
                                                            table.calibration_id,clock=table.clock)
                self.assertEqual(restored.recover(robot_stopped=True),Phase.THINKING)
                self.assertEqual(table.tasks,[])
                with self.assertRaises(ValueError):
                    GameController.resume_from_journal(reopened,table,AlphaBetaEngine(),table,'wrong-cal')

    def test_write_failure_stops_before_hardware(self):
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            append=journal.append_event
            def fail(event,*args):
                if event['event']=='execution_started':raise OSError('disk unavailable')
                return append(event,*args)
            journal.append_event=fail
            self.assertEqual(c.run_turn(),Phase.PAUSED)
            self.assertEqual(table.tasks,[])
            self.assertIsNotNone(replay_session(d)['checkpoint']['pending_task'])
            journal.close()

    def test_truncated_tail_and_corruption_and_writer_lock(self):
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            with self.assertRaises(OSError):SessionJournal(d)
            journal.close()
            path=Path(d)/'events.jsonl'
            with open(path,'ab') as f:f.write(b'{"partial":')
            with self.assertRaises(ValueError):replay_session(d)
            with self.assertRaises(ValueError):SessionJournal(d)
            with SessionJournal(d,repair_tail=True) as fixed:
                self.assertTrue(fixed.initialized)
            self.assertEqual(len(list(Path(d).glob('truncated-tail-*.bin'))),1)
            data=replay_session(d)['records']
            data[-1]['sequence']=999
            with self.assertRaises(ValueError):validate_records(data)

    def test_observations_images_and_invalid_state_history(self):
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            journal.record_image('test-frame',np.zeros((10,10,3),np.uint8))
            self.assertTrue((Path(d)/journal.take_image_reference('test-frame')['path']).exists())
            c.run_turn()
            journal.close()
            result=replay_session(d)
            self.assertTrue(any(r.get('event',{}).get('event')=='observation' for r in result['records']))
            result['records'][-1]['checkpoint']['state']['cells'][9][9]=1
            with self.assertRaises(ValueError):validate_records(result['records'])

    def test_observer_image_sink_in_full_controller(self):
        from tests.test_vision import fixture
        from so101_gomoku.vision.board_detector import BoardObserver
        import cv2
        cfg,empty,detector=fixture(10)
        table=SimulatedTable(calibration_id='test-cal')
        class Camera:
            def capture(self):
                image=empty.copy()
                for r,row in enumerate(table.board.cells):
                    for col,value in enumerate(row):
                        if value:
                            v=20 if value==1 else 245
                            cv2.circle(image,cfg.point(r,col),20,(v,v,v),-1)
                return image,table.clock()
        with tempfile.TemporaryDirectory() as d:
            with SessionJournal(d,configuration={'mode':'SYNTHETIC_IMAGES'}) as journal:
                observer=BoardObserver(Camera(),detector,image_sink=journal.record_image)
                controller=GameController(observer,AlphaBetaEngine(max_depth=1),table,'test-cal',
                    clock=table.clock,journal=journal,stable_frames=1,search_time_s=.03)
                self.assertEqual(controller.start(),Phase.THINKING)
                self.assertEqual(controller.run_turn(),Phase.WAIT_HUMAN)
            records=replay_session(d)['records']
            observations=[r['event'] for r in records if r.get('event',{}).get('event')=='observation']
            self.assertGreaterEqual(len(observations),4)
            self.assertTrue(all(event['image'] is not None for event in observations))
            self.assertEqual(len(list((Path(d)/'images').glob('*.png'))),len(observations))

    def test_complete_record_corruption_and_configuration_conflict(self):
        from copy import deepcopy
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d);c.run_turn();journal.close()
            saved=replay_session(d)['records']
            broken=deepcopy(saved);del broken[-1]['event']
            with self.assertRaises(ValueError):validate_records(broken)
            broken=deepcopy(saved);broken[-1]['checkpoint']['phase']='boot';broken[-1]['event']['phase']='boot'
            with self.assertRaises(ValueError):validate_records(broken)
            broken=deepcopy(saved)
            intent=next(r for r in broken if r.get('event',{}).get('event')=='execution_started')
            intent['checkpoint']['pending_task']=None
            with self.assertRaises(ValueError):validate_records(broken)
            with self.assertRaises(ValueError):SessionJournal(d,configuration={'spacing_m':.04})
            with SessionJournal(d) as reopened:
                self.assertEqual(reopened.configuration['spacing_m'],.021)

    def test_restart_with_uncommitted_human_move_needs_explicit_acceptance(self):
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            c.run_turn()
            table.human_move(Move(9,9,Cell.WHITE))
            journal.close()  # Program stopped before observing or committing human move.
            with SessionJournal(d) as reopened:
                restored=GameController.resume_from_journal(reopened,table,AlphaBetaEngine(),table,
                    table.calibration_id,clock=table.clock)
                self.assertEqual(restored.recover(robot_stopped=True),Phase.PAUSED)
                self.assertEqual(restored.state.version,1)
                self.assertEqual(restored.recover(robot_stopped=True,accept_human_move=True),Phase.THINKING)
                self.assertEqual(restored.state.version,2)
                self.assertEqual(len(table.tasks),1)
            self.assertEqual(replay_session(d)['checkpoint']['state']['version'],2)

    def test_self_contained_html_keeps_images_and_escapes_log_text(self):
        from so101_gomoku.recording.replay_html import export_replay_html
        with tempfile.TemporaryDirectory() as d:
            table,journal,c=self.make(d)
            journal.record_image('sample',np.zeros((10,10,3),np.uint8))
            c._pause(ValueError('</script><script>not executable</script>'))
            journal.close()
            path=export_replay_html(d,Path(d)/'replay.html')
            html=path.read_text(encoding='utf-8')
            self.assertIn('data:image/png;base64,',html)
            self.assertNotIn('</script><script>',html)
            self.assertIn('\\u003c/script',html)
