"""Abort-on-failure execution. Does not commit game state or verify board images.

Adapted sequencing from LeRobot Chess tool_move_piece.py, Apache-2.0.
Removed unconditional success, static grasp defaults and language-tool coupling.
"""
import math
from threading import Lock
from so101_gomoku.contracts.models import ExecutionResult


class PickAndPlace:
    def __init__(self, driver, ik, open_percent, close_percent, grasp_check,
                 timeout_s, control_lock=None):
        if not all(math.isfinite(v) and 0 <= v <= 100 for v in (open_percent, close_percent)) or close_percent >= open_percent:
            raise ValueError("Provide calibrated gripper percentages: close < open")
        if timeout_s <= 0 or not math.isfinite(timeout_s) or not callable(grasp_check):
            raise ValueError("Positive timeout and grasp verification callback required")
        self.driver, self.ik = driver, ik
        self.open_percent, self.close_percent = open_percent, close_percent
        self.grasp_check, self.timeout_s = grasp_check, timeout_s
        self.lock = control_lock if control_lock is not None else Lock()

    def execute(self, task_id, waypoints):
        if not self.lock.acquire(blocking=False):
            return ExecutionResult(task_id, False, 'busy', 'Robot control is already in use')
        stage = 'preflight'
        try:
            if not waypoints:
                raise ValueError("Empty motion plan")
            current = self.driver.read_joints()
            seed = [current[name] for name in self.ik.joint_names]
            targets = []
            for waypoint in waypoints:
                if waypoint.gripper not in (None, 'open', 'grasp'):
                    raise ValueError("Unknown gripper operation")
                joints = self.ik.solve(seed, waypoint.pose)
                targets.append(joints)
                seed = [joints[name] for name in self.ik.joint_names]
            # Preflight validates endpoints; trajectory/collision verification remains separate.
            for waypoint, joints in zip(waypoints, targets):
                stage = waypoint.stage
                if not self.driver.move_and_wait(joints, self.timeout_s):
                    return ExecutionResult(task_id, False, stage, 'Arm target timeout')
                if waypoint.gripper == 'open':
                    if not self.driver.move_and_wait({'gripper': self.open_percent}, self.timeout_s):
                        return ExecutionResult(task_id, False, stage, 'Gripper open timeout')
                elif waypoint.gripper == 'grasp':
                    # Contact can prevent reaching the fully-closed target; use explicit sensing.
                    self.driver.command_gripper(self.close_percent)
                    if not self.grasp_check(self.driver, self.timeout_s):
                        return ExecutionResult(task_id, False, stage, 'Grasp unconfirmed')
            return ExecutionResult(task_id, True, 'motion_completed', visual_confirmed=False)
        except Exception as exc:
            return ExecutionResult(task_id, False, stage, f'{type(exc).__name__}: {exc}')
        finally:
            self.lock.release()
