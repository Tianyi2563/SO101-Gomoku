"""SIMULATED ONLY: instant targets, no physics, hardware or grasp sensing."""


class MockDriver:
    simulated = True

    def __init__(self, joints, fail_on_call=None):
        self.joints = dict(joints)
        self.fail_on_call = fail_on_call
        self.commands = []

    def read_joints(self):
        return self.joints.copy()

    def move_and_wait(self, targets, timeout_s):
        self.commands.append(dict(targets))
        if len(self.commands) == self.fail_on_call:
            return False
        self.joints.update(targets)
        return True

    def command_gripper(self, percent):
        self.commands.append({'gripper': percent})
        self.joints['gripper'] = percent
