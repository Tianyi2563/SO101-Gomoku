"""Board-to-base rigid calibration. No camera homography is used here."""
from dataclasses import dataclass
import numpy as np


def validate_transform(matrix):
    t = np.asarray(matrix, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("Expected finite 4x4 transform")
    r = t[:3, :3]
    if not np.allclose(t[3], [0, 0, 0, 1]) or not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1):
        raise ValueError("Expected proper rigid transform")
    return t


def apply_transform(matrix, points):
    t = validate_transform(matrix)
    p = np.asarray(points, dtype=float)
    if p.shape[-1:] != (3,) or not np.isfinite(p).all():
        raise ValueError("Expected finite XYZ points")
    return p @ t[:3, :3].T + t[:3, 3]


@dataclass(frozen=True)
class CalibrationFit:
    matrix: np.ndarray
    rms_m: float
    max_error_m: float


def fit_board_to_base(board_xyz, base_xyz):
    """Rigid least squares fit; residuals are training fit, not held-out accuracy."""
    src, dst = np.asarray(board_xyz, float), np.asarray(base_xyz, float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3 or len(src) < 3:
        raise ValueError("Need >=3 corresponding XYZ points")
    if not np.isfinite(src).all() or not np.isfinite(dst).all():
        raise ValueError("Non-finite calibration points")
    a, b = src - src.mean(0), dst - dst.mean(0)
    if np.linalg.matrix_rank(a) < 2 or np.linalg.matrix_rank(b) < 2:
        raise ValueError("Calibration points must not be collinear")
    u, _, vt = np.linalg.svd(a.T @ b)
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ correction @ u.T
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = r, dst.mean(0) - r @ src.mean(0)
    errors = np.linalg.norm(apply_transform(matrix, src) - dst, axis=1)
    return CalibrationFit(matrix, float(np.sqrt(np.mean(errors**2))), float(errors.max()))


def validation_errors(matrix, board_xyz, base_xyz):
    src, dst = np.asarray(board_xyz, float), np.asarray(base_xyz, float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3 or not len(src):
        raise ValueError("Expected matching nonempty Nx3 validation points")
    if not np.isfinite(dst).all():
        raise ValueError("Non-finite validation points")
    return np.linalg.norm(apply_transform(matrix, src) - dst, axis=1)
