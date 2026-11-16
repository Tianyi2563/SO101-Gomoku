"""Validate any FK/IK solver implementing the transplanted LeRobot interface."""
import numpy as np
from so101_gomoku.calibration.transforms import validate_transform


class IKAdapter:
    def __init__(self, solver, joint_names, limits_deg, position_tolerance_m, orientation_tolerance_rad,
                 orientation_mode='all'):
        self.solver, self.joint_names = solver, tuple(joint_names)
        if orientation_mode not in ('all', 'X', 'Y', 'Z'):
            raise ValueError('Invalid orientation mode')
        if hasattr(solver, 'orientation_mode') and solver.orientation_mode != orientation_mode:
            raise ValueError('Solver and validator orientation modes must match')
        if hasattr(solver, 'joint_names') and tuple(solver.joint_names) != self.joint_names:
            raise ValueError('Solver and adapter joint order must match')
        self.orientation_mode = orientation_mode
        self.limits = np.asarray(limits_deg, float)
        self.position_tol, self.orientation_tol = position_tolerance_m, orientation_tolerance_rad
        if (self.limits.shape != (len(self.joint_names), 2) or not np.isfinite(self.limits).all()
                or np.any(self.limits[:, 0] >= self.limits[:, 1])):
            raise ValueError("Provide measured per-joint degree limits")
        if not np.isfinite([self.position_tol, self.orientation_tol]).all() or min(self.position_tol, self.orientation_tol) <= 0:
            raise ValueError("Positive finite tolerances required")

    def solve(self, current_deg, target_pose):
        target = validate_transform(target_pose)
        current = np.asarray(current_deg, float)
        if current.shape != (len(self.joint_names),) or not np.isfinite(current).all():
            raise ValueError("Invalid current joint vector")
        if np.any(current < self.limits[:, 0]) or np.any(current > self.limits[:, 1]):
            raise ValueError('Current joints outside configured limits')
        result = np.asarray(self.solver.inverse_kinematics(current.copy(), target.copy()), float)
        if result.shape != current.shape or not np.isfinite(result).all():
            raise RuntimeError("IK returned invalid joint vector")
        if np.any(result < self.limits[:, 0]) or np.any(result > self.limits[:, 1]):
            raise RuntimeError("IK exceeds configured joint limits")
        actual = validate_transform(self.solver.forward_kinematics(result))
        position_error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
        if self.orientation_mode == 'all':
            relative = target[:3, :3].T @ actual[:3, :3]
            angle_error = np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1))
        else:
            axis = 'XYZ'.index(self.orientation_mode)
            angle_error = np.arccos(np.clip(np.dot(target[:3, axis], actual[:3, axis]), -1, 1))
        if position_error > self.position_tol or angle_error > self.orientation_tol:
            raise RuntimeError(f"IK residual exceeds tolerance: {position_error:.6f}m, {angle_error:.6f}rad")
        return dict(zip(self.joint_names, map(float, result)))
