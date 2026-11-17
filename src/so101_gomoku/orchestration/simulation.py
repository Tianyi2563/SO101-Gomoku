"""Synthetic physical board for offline controller tests; no IK or hardware."""
import time
from so101_gomoku.contracts.models import BoardObservation, ExecutionResult
from so101_gomoku.game.board import GameState


class SimulationClock:
    """Monotonic synthetic epoch time, explicitly shared with the controller."""
    def __init__(self):
        self.value = time.time()

    def __call__(self):
        self.value += .001
        return self.value


class SimulatedTable:
    def __init__(self, state=None, calibration_id='simulated-calibration', clock=None):
        self.board = state if state is not None else GameState.empty()
        self.calibration_id = calibration_id
        self.clock = clock if clock is not None else SimulationClock()
        self.frames = 0
        self.tasks = []
        self.fail_execution = False
        self.apply_despite_failure = False

    def observe(self):
        self.frames += 1
        return BoardObservation(self.board.cells, tuple((1.,)*self.board.size for _ in self.board.cells),
                                f'sim-{self.frames}', self.clock(), self.calibration_id)

    def human_move(self, move):
        self.board = self.board.play(move)

    def execute(self, task):
        self.tasks.append(task)
        if task.expected_game_version != self.board.version or task.calibration_id != self.calibration_id:
            return ExecutionResult(task.task_id, False, 'simulated_preflight', 'Version/calibration mismatch')
        if not self.fail_execution or self.apply_despite_failure:
            self.board = self.board.play(task.move)
        return ExecutionResult(task.task_id, not self.fail_execution, 'simulated',
                               'Injected failure' if self.fail_execution else '')
