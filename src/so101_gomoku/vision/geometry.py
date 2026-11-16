"""Adapted from Koch vision/utils.py (MIT, hri5-boardgame-robot 2024).

Removed implicit image resizing: H always refers to the original input pixels.
Replaced manual DLT with OpenCV's homography estimation.
"""
import cv2
import numpy as np


def get_grid_points(top_left, bottom_right, size=9):
    if size < 2:
        raise ValueError("Grid size must be >=2")
    tl, br = np.asarray(top_left, float), np.asarray(bottom_right, float)
    if tl.shape != (2,) or br.shape != (2,) or not np.isfinite([tl, br]).all() or np.any(br <= tl):
        raise ValueError("Expected ordered XY bounds")
    xx, yy = np.meshgrid(np.linspace(tl[0], br[0], size), np.linspace(tl[1], br[1], size))
    return np.stack([xx.ravel(), yy.ravel()], axis=1)


def estimate_homography(image_points, board_pixels):
    src, dst = np.asarray(image_points, float), np.asarray(board_pixels, float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2 or len(src) < 4:
        raise ValueError("Need >=4 corresponding XY points")
    if not np.isfinite(src).all() or not np.isfinite(dst).all():
        raise ValueError("Non-finite image points")
    h, _ = cv2.findHomography(src, dst, method=0)
    if h is None or not np.isfinite(h).all() or np.linalg.matrix_rank(h) != 3:
        raise ValueError("Degenerate homography")
    return h


def warp_planar(image, homography, output_size):
    h = np.asarray(homography, float)
    if h.shape != (3, 3) or not np.isfinite(h).all() or np.linalg.matrix_rank(h) != 3:
        raise ValueError("Expected nonsingular finite homography")
    if len(output_size) != 2 or any(type(v) is not int or v <= 0 for v in output_size):
        raise ValueError("Output size is (width, height) in pixels")
    return cv2.warpPerspective(image, h, output_size)
