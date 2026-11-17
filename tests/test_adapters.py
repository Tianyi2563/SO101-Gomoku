import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import unittest
import numpy as np
from so101_gomoku.vision.geometry import get_grid_points, estimate_homography, warp_planar
from so101_gomoku.vision.stabilizer import BoardStabilizer
from so101_gomoku.contracts.models import BoardObservation
from so101_gomoku.game.board import GameState
from so101_gomoku.robot.kinematics.adapter import IKAdapter
from so101_gomoku.robot.drivers.lerobot_driver import LeRobotDriver
from types import SimpleNamespace


class Solver:
    def inverse_kinematics(self, current, pose):
        return [pose[0, 3]]
    def forward_kinematics(self, joints):
        result = np.eye(4)
        result[0, 3] = joints[0]
        return result


class AdapterTests(unittest.TestCase):
    def test_grid_and_no_implicit_resize(self):
        grid = get_grid_points((10, 20), (90, 100), size=9)
        np.testing.assert_allclose(grid[9], [10, 30])
        image = np.zeros((600, 800), dtype=np.uint8)
        image[400:410, 600:610] = 255
        np.testing.assert_array_equal(warp_planar(image, np.eye(3), (800, 600)), image)
        corners = np.array([[0, 0], [100, 0], [100, 100], [0, 100]], float)
        h = estimate_homography(corners, corners + [20, 30])
        np.testing.assert_allclose(h @ [0, 0, 1], [20, 30, 1], atol=1e-8)
        with self.assertRaises(ValueError):
            estimate_homography([[0, 0]] * 4, [[0, 0]] * 4)

    def test_stability_and_duplicate(self):
        cells = GameState.empty(9).cells
        stable = BoardStabilizer(frames=2)
        def obs(i, cal='c'):
            return BoardObservation(cells, ((1.,) * 9,) * 9, str(i), float(i), cal)
        self.assertIsNone(stable.update(obs(1)))
        self.assertIsNone(stable.update(obs(1)))
        self.assertIsNone(stable.update(obs(2)))
        self.assertIsNotNone(stable.update(obs(3)))
        self.assertIsNone(stable.update(obs(4, 'changed')))

    def test_ik_checks(self):
        adapter = IKAdapter(Solver(), ['joint'], [[-1, 1]], .001, .01)
        target = np.eye(4)
        target[0, 3] = .2
        self.assertAlmostEqual(adapter.solve([0], target)['joint'], .2)
        target[0, 3] = 2
        with self.assertRaises(RuntimeError):
            adapter.solve([0], target)
        target = np.eye(4)
        target[1, 3] = .1
        with self.assertRaises(RuntimeError):
            adapter.solve([0], target)

    def test_driver_does_not_treat_clipping_or_stall_as_success(self):
        class FakeRobot:
            def get_observation(self):
                return {'joint.pos': 0., 'gripper.pos': 90.}
            def send_action(self, targets):
                return {key: 0. for key in targets}  # emulate target clipping
        driver = LeRobotDriver(SimpleNamespace(use_degrees=True), .1, 1, .001)
        driver.robot = FakeRobot()
        with self.assertRaises(RuntimeError):
            driver.move_and_wait({'joint': 10.}, .01)
        driver.robot.send_action = lambda targets: targets
        self.assertFalse(driver.move_and_wait({'joint': 10.}, .005))
        self.assertTrue(driver.move_and_wait({'joint': 0.}, .01))


if __name__ == '__main__':
    unittest.main()
