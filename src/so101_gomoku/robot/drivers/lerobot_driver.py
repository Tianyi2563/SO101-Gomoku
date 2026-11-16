"""Project adapter to installed LeRobot. No import/connection at construction.

Upstream source copies in vendor/ are review references, not an importable fork.
Connection is explicit and requires an existing calibrated SO101 configuration.
"""
import math
import time


class LeRobotDriver:
    def __init__(self, config, position_tolerance_deg, gripper_tolerance_percent, poll_s=0.02):
        if not config.use_degrees:
            raise ValueError("SO101 config must use degrees")
        if not all(math.isfinite(v) and v > 0 for v in (position_tolerance_deg, gripper_tolerance_percent, poll_s)):
            raise ValueError("Positive finite tolerances required")
        self.config, self.robot = config, None
        self.arm_tol, self.gripper_tol, self.poll_s = position_tolerance_deg, gripper_tolerance_percent, poll_s

    def connect(self):
        if self.robot is not None:
            raise RuntimeError("Already connected")
        from lerobot.robots.so_follower import SO101Follower
        robot = SO101Follower(self.config)
        try:
            robot.connect(calibrate=False)
            if not robot.is_calibrated:
                raise RuntimeError("Motor calibration required before motion")
        except Exception:
            if robot.is_connected:
                robot.disconnect()
            raise
        self.robot = robot

    def disconnect(self):
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def read_joints(self):
        if self.robot is None:
            raise RuntimeError("Robot not connected")
        return {k.removesuffix('.pos'): float(v) for k, v in self.robot.get_observation().items() if k.endswith('.pos')}

    def command_gripper(self, percent):
        if self.robot is None:
            raise RuntimeError("Robot not connected")
        if not math.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError("Invalid gripper percentage")
        sent = self.robot.send_action({'gripper.pos': percent})
        if not math.isclose(sent.get('gripper.pos', float('nan')), percent, abs_tol=1e-6):
            raise RuntimeError("Driver clipped gripper target")

    def move_and_wait(self, targets, timeout_s):
        if self.robot is None:
            raise RuntimeError("Robot not connected")
        if not targets or timeout_s <= 0 or not math.isfinite(timeout_s):
            raise ValueError("Targets and positive finite timeout required")
        current = self.read_joints()
        if any(k not in current or not math.isfinite(v) for k, v in targets.items()):
            raise ValueError("Invalid joint targets")
        deadline = time.monotonic() + timeout_s
        # A clipped command is not success; stop the sequence rather than hiding it.
        sent = self.robot.send_action({f'{k}.pos': v for k, v in targets.items()})
        if any(not math.isclose(sent.get(f'{k}.pos', float('nan')), v, abs_tol=1e-6) for k, v in targets.items()):
            raise RuntimeError("Driver clipped requested targets; sequence aborted")
        while time.monotonic() < deadline:
            actual = self.read_joints()
            if all(abs(actual[k] - v) <= (self.gripper_tol if k == 'gripper' else self.arm_tol) for k, v in targets.items()):
                return True
            time.sleep(self.poll_s)
        return False
