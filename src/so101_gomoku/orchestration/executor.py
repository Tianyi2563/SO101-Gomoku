"""Connect controller tasks to the existing geometric planner and primitive."""
from so101_gomoku.contracts.models import ExecutionResult


class TaskExecutor:
    def __init__(self, planner, primitive, calibration_id):
        self.planner, self.primitive, self.calibration_id = planner, primitive, calibration_id

    def execute(self, task):
        if task.calibration_id != self.calibration_id:
            return ExecutionResult(task.task_id, False, 'planning', 'Calibration mismatch')
        try:
            waypoints = self.planner(task)
        except Exception as exc:
            return ExecutionResult(task.task_id, False, 'planning', f'{type(exc).__name__}: {exc}')
        return self.primitive.execute(task.task_id, waypoints)
