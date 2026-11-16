"""IKPy backend exposing the project's degree-based FK/IK interface.

Use through IKAdapter: an optimizer result is not proof of target reachability.
No hardware access. Fixed links are held at zero; gripper opening is not a DOF.
"""
from pathlib import Path
import numpy as np
from so101_gomoku.calibration.transforms import validate_transform

SO101_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


class IKPyKinematics:
    def __init__(self, urdf_path, joint_names=SO101_JOINTS, *,
                 base='base_link', tip='gripper_frame_link', orientation_mode='all',
                 optimizer_budget=200):
        from ikpy.chain import Chain
        if orientation_mode not in ('all', 'X', 'Y', 'Z'):
            raise ValueError('Orientation mode must be all, X, Y or Z')
        if type(optimizer_budget) is not int or optimizer_budget <= 0:
            raise ValueError('Positive integer optimizer budget required')
        self.joint_names = tuple(joint_names)
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError('Distinct joint names required')
        self.urdf_path = Path(urdf_path).resolve(strict=True)
        self.chain = Chain.from_urdf_file(str(self.urdf_path), base=base, tip=tip)
        active = [link for link in self.chain.links if link.joint_type != 'fixed']
        if any(link.joint_type != 'revolute' for link in active):
            raise ValueError('Degree interface only supports revolute arm joints')
        if set(link.name for link in active) != set(self.joint_names) or len(active) != len(self.joint_names):
            raise ValueError('URDF active joints do not match requested arm joints')
        self.indices = tuple(next(i for i, link in enumerate(self.chain.links) if link.name == name)
                             for name in self.joint_names)
        limits = np.array([self.chain.links[i].bounds for i in self.indices], float)
        if not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError('Finite ordered joint limits required')
        self.limits_deg = np.rad2deg(limits)
        self.orientation_mode = orientation_mode
        self.optimizer_budget = optimizer_budget

    def _full_radians(self, joints_deg):
        joints = np.asarray(joints_deg, float)
        if joints.shape != (len(self.joint_names),) or not np.isfinite(joints).all():
            raise ValueError('Expected one finite degree value per arm joint, without gripper')
        if np.any(joints < self.limits_deg[:, 0]) or np.any(joints > self.limits_deg[:, 1]):
            raise ValueError('Joint vector outside URDF limits')
        full = np.zeros(len(self.chain.links))
        full[list(self.indices)] = np.deg2rad(joints)
        return full

    def forward_kinematics(self, joint_pos_deg):
        return self.chain.forward_kinematics(self._full_radians(joint_pos_deg))

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose):
        target = validate_transform(desired_ee_pose)
        full = self._full_radians(current_joint_pos)
        seed = [float(full[i]) if i in self.indices else False for i in range(len(full))]
        solved = np.asarray(self.chain.inverse_kinematics_frame(
            target, initial_position=seed, orientation_mode=self.orientation_mode,
            optimizer_budget=self.optimizer_budget), float)
        if solved.shape != full.shape or not np.isfinite(solved).all():
            raise RuntimeError('IKPy returned invalid joints')
        return np.rad2deg(solved[list(self.indices)])
