"""Adapted waypoint order from LeRobot Chess tool_move_piece.py (Apache-2.0).

Fixed supply -> intersection. All heights, orientation and workspace explicit.
Workspace checks are geometric bounds, NOT collision detection.
"""
from dataclasses import dataclass
import numpy as np
from so101_gomoku.calibration.transforms import apply_transform, validate_transform


@dataclass(frozen=True)
class Waypoint:
    stage: str
    pose: np.ndarray
    gripper: str | None = None


def plan_pick_place(geometry, board_to_base, row, col, supply_xyz,
                    rotation, safe_z_m, workspace_min, workspace_max):
    target = apply_transform(board_to_base, geometry.intersection(row, col))
    supply = np.asarray(supply_xyz, float)
    low, high = np.asarray(workspace_min, float), np.asarray(workspace_max, float)
    if any(p.shape != (3,) or not np.isfinite(p).all() for p in (supply, low, high)) or np.any(low >= high):
        raise ValueError("Explicit finite XYZ supply and workspace bounds required")
    if not np.isfinite(safe_z_m) or safe_z_m <= max(supply[2], target[2]):
        raise ValueError("Safe Z must be above supply and target")
    r = np.asarray(rotation, float)
    base_pose = np.eye(4)
    base_pose[:3, :3] = r
    validate_transform(base_pose)
    source_high, target_high = supply.copy(), target.copy()
    source_high[2] = target_high[2] = safe_z_m
    points = [
        ('source_above', source_high, 'open'), ('source_down', supply, None),
        ('grasp', supply, 'grasp'), ('source_lift', source_high, None),
        ('target_above', target_high, None), ('target_down', target, None),
        ('release', target, 'open'), ('retreat', target_high, None),
    ]
    result = []
    for stage, xyz, gripper in points:
        if np.any(xyz < low) or np.any(xyz > high):
            raise ValueError(f"Waypoint outside configured workspace: {stage}")
        pose = base_pose.copy()
        pose[:3, 3] = xyz
        result.append(Waypoint(stage, pose, gripper))
    return tuple(result)
