import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
from threading import Lock
import numpy as np
from so101_gomoku.calibration.board_geometry import BoardGeometry
from so101_gomoku.robot.planning.pick_place_plan import plan_pick_place
from so101_gomoku.robot.primitives.pick_and_place import PickAndPlace
from so101_gomoku.robot.drivers.mock_driver import MockDriver
from so101_gomoku.interfaces.demo import SimulatedIK


class MotionTests(unittest.TestCase):
    def plan(self):
        return plan_pick_place(BoardGeometry(9, .022), np.eye(4), 4, 4,
                               [-.04, 0, 0], np.eye(3), .1, [-.1, -.1, -.01], [.3, .3, .2])

    def executor(self, fail=None, grasp=True, ik=None, lock=None):
        driver = MockDriver({'mock_x': 0, 'mock_y': 0, 'mock_z': .1, 'gripper': 90}, fail)
        return driver, PickAndPlace(driver, ik or SimulatedIK(), 90, 5, lambda *_: grasp, 1, lock)

    def test_success_does_not_claim_visual_confirmation(self):
        driver, executor = self.executor()
        result = executor.execute('t', self.plan())
        self.assertTrue(result.completed)
        self.assertFalse(result.visual_confirmed)
        self.assertEqual(len(driver.commands), 11)

    def test_timeout_aborts_and_does_not_retry(self):
        driver, executor = self.executor(fail=3)
        result = executor.execute('t', self.plan())
        self.assertFalse(result.completed)
        self.assertEqual(result.stage, 'source_down')
        self.assertEqual(len(driver.commands), 3)

    def test_failed_grasp_no_lift(self):
        driver, executor = self.executor(grasp=False)
        result = executor.execute('t', self.plan())
        self.assertFalse(result.completed)
        self.assertEqual(result.stage, 'grasp')
        self.assertEqual(len(driver.commands), 5)

    def test_preflight_failure_no_commands(self):
        class BadIK(SimulatedIK):
            def solve(self, *_):
                raise RuntimeError('unreachable')
        driver, executor = self.executor(ik=BadIK())
        self.assertFalse(executor.execute('t', self.plan()).completed)
        self.assertEqual(driver.commands, [])

    def test_shared_lock(self):
        lock = Lock()
        driver, executor = self.executor(lock=lock)
        lock.acquire()
        try:
            self.assertEqual(executor.execute('t', self.plan()).stage, 'busy')
            self.assertEqual(driver.commands, [])
        finally:
            lock.release()

    def test_invalid_heights_and_workspace(self):
        with self.assertRaises(ValueError):
            plan_pick_place(BoardGeometry(9, .022), np.eye(4), 8, 8, [0, 0, 0],
                            np.eye(3), .1, [0, 0, 0], [.1, .1, .2])


if __name__ == '__main__':
    unittest.main()
