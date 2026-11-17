"""Offline entry point. All output from this command is simulated."""
import json
from dataclasses import asdict
from pathlib import Path
import numpy as np
from so101_gomoku.contracts.models import Cell, Move, BoardObservation
from so101_gomoku.game.board import GameState
from so101_gomoku.game.reconciliation import verify_placement
from so101_gomoku.calibration.board_geometry import BoardGeometry
from so101_gomoku.robot.planning.pick_place_plan import plan_pick_place
from so101_gomoku.robot.primitives.pick_and_place import PickAndPlace
from so101_gomoku.robot.drivers.mock_driver import MockDriver


class SimulatedIK:
    """Synthetic XYZ-to-joints stand-in; NOT SO101 kinematics."""
    joint_names = ('mock_x', 'mock_y', 'mock_z')

    def solve(self, current, target):
        return dict(zip(self.joint_names, map(float, target[:3, 3])))


def run(output: Path) -> dict:
    state = GameState.empty(9)  # Legacy synthetic geometry; main game defaults to 10.
    move = Move(4, 4, Cell.BLACK)  # scripted move, NOT an AI decision
    geometry = BoardGeometry(9, .022)  # synthetic geometry, NOT real calibration
    plan = plan_pick_place(geometry, np.eye(4), move.row, move.col, [-.04, 0, 0],
                           np.eye(3), .1, [-.1, -.1, -.01], [.3, .3, .2])
    driver = MockDriver({'mock_x': 0, 'mock_y': 0, 'mock_z': .1, 'gripper': 90})
    executor = PickAndPlace(driver, SimulatedIK(), 90, 5, lambda *_: True, 1)
    execution = executor.execute('simulated-task-1', plan)
    if not execution.completed:
        raise RuntimeError(execution.reason)
    # Synthetic observation: this demonstrates reconciliation, not camera recognition.
    observation = BoardObservation(state.play(move).cells, ((1.,) * 9,) * 9, 'sim-frame', 1., 'sim-cal')
    confirmed = verify_placement(state, move, observation)
    result = {"mode": "SIMULATED", "hardware_connected": False,
              "limitations": ['scripted move', 'synthetic IK', 'assumed grasp', 'synthetic observation'],
              "execution": asdict(execution), "verified_game_version": confirmed.version,
              "command_count": len(driver.commands)}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result
