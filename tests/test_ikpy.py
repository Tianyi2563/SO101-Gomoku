import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
import numpy as np
from so101_gomoku.robot.kinematics.ikpy_kinematics import IKPyKinematics, SO101_JOINTS
from so101_gomoku.robot.kinematics.adapter import IKAdapter
from so101_gomoku.robot.drivers.mock_driver import MockDriver
from so101_gomoku.robot.primitives.pick_and_place import PickAndPlace
from so101_gomoku.robot.planning.pick_place_plan import Waypoint

MODEL = Path(__file__).resolve().parents[1] / 'assets/robot/so101/so101_new_calib.urdf'


class IKPyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.solver = IKPyKinematics(MODEL)
        cls.adapter = IKAdapter(cls.solver, SO101_JOINTS, cls.solver.limits_deg, 1e-4, 1e-3)

    def test_degree_round_trip_and_named_output(self):
        q = np.array([15., -25., 35., 20., -30.])
        target = self.solver.forward_kinematics(q)
        result = self.adapter.solve(q + .5, target)
        self.assertEqual(tuple(result), SO101_JOINTS)
        actual = self.solver.forward_kinematics(list(result.values()))
        np.testing.assert_allclose(actual, target, atol=1e-5)
        self.assertNotIn('gripper', result)

    def test_joint_mapping_is_by_name(self):
        reverse = IKPyKinematics(MODEL, tuple(reversed(SO101_JOINTS)))
        q = [10., 20., 30., -20., -10.]
        np.testing.assert_allclose(self.solver.forward_kinematics(q), reverse.forward_kinematics(q[::-1]), atol=1e-12)

    def test_invalid_input_and_limits(self):
        for q in ([0]*6, [float('nan')]*5, [999]*5):
            with self.assertRaises(ValueError):
                self.solver.forward_kinematics(q)
        with self.assertRaises(ValueError):
            IKPyKinematics(MODEL, ['gripper'])
        with self.assertRaises(ValueError):
            IKAdapter(self.solver, SO101_JOINTS[::-1], self.solver.limits_deg, .001, .01)

    def test_unreachable_rejected(self):
        target = np.eye(4)
        target[:3, 3] = [10., 10., 10.]
        with self.assertRaisesRegex(RuntimeError, 'residual'):
            self.adapter.solve(np.zeros(5), target)

    def test_axis_mode_and_validator_match(self):
        solver = IKPyKinematics(MODEL, orientation_mode='Z')
        adapter = IKAdapter(solver, SO101_JOINTS, solver.limits_deg, 1e-4, 1e-3, orientation_mode='Z')
        q = np.array([10., -20., 30., 20., -10.])
        target = solver.forward_kinematics(q)
        adapter.solve(q + .2, target)
        with self.assertRaises(ValueError):
            IKAdapter(solver, SO101_JOINTS, solver.limits_deg, .001, .01)

    def test_axis_validation_ignores_only_rotation_about_selected_axis(self):
        class RotatedSolver:
            def inverse_kinematics(self, *_):
                return [0.]
            def forward_kinematics(self, *_):
                return np.array([[0., -1., 0., 0.], [1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
        IKAdapter(RotatedSolver(), ['j'], [[-1, 1]], .001, .01, 'Z').solve([0], np.eye(4))
        with self.assertRaises(RuntimeError):
            IKAdapter(RotatedSolver(), ['j'], [[-1, 1]], .001, .01).solve([0], np.eye(4))

    def test_motion_executor_with_real_model_and_mock_motor(self):
        driver = MockDriver(dict(zip(SO101_JOINTS, [0.]*5)) | {'gripper': 90.})
        poses = [self.solver.forward_kinematics([i, -i, i, i, -i]) for i in (1., 2., 3.)]
        executor = PickAndPlace(driver, self.adapter, 90., 5., lambda *_: True, 1)
        plan = tuple(Waypoint(f'model_pose_{i}', pose) for i, pose in enumerate(poses))
        result = executor.execute('offline-real-ik-mock-driver', plan)
        self.assertTrue(result.completed, result.reason)
        self.assertFalse(result.visual_confirmed)
        self.assertEqual(len(driver.commands), 3)
        bad = Waypoint('unreachable', np.eye(4))
        bad.pose[:3, 3] = [10., 10., 10.]
        count = len(driver.commands)
        self.assertFalse(executor.execute('bad', (bad,)).completed)
        self.assertEqual(len(driver.commands), count)


if __name__ == '__main__':
    unittest.main()
