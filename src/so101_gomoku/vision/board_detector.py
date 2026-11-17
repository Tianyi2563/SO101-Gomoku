"""Reference-based classification around intersections, never square centers.

Requires an empty-board image, fixed exposure/view, and ordered outer grid points.
Rule confidence is not a calibrated probability. Unknowns must not update a game.
"""
from dataclasses import dataclass
import cv2
import numpy as np
from so101_gomoku.contracts.models import Cell, BoardObservation
from .geometry import estimate_homography, warp_planar


@dataclass(frozen=True)
class VisionConfig:
    intersections: int = 10
    spacing_px: int = 64
    margin_px: int = 48
    difference_threshold: float = 28.
    min_stone_radius: float = .18
    max_stone_radius: float = .43
    center_tolerance: float = .2
    black_delta: float = 30.
    white_delta: float = 25.
    min_circularity: float = .65

    def __post_init__(self):
        if type(self.intersections) is not int or self.intersections < 5:
            raise ValueError('Intersections must be >=5')
        if (type(self.spacing_px) is not int or self.spacing_px < 24
                or type(self.margin_px) is not int or self.margin_px < self.spacing_px/2):
            raise ValueError('Need spacing >=24px and margin >= half spacing')
        if not (0 < self.min_stone_radius < self.max_stone_radius < .5):
            raise ValueError('Stone radii are fractions of grid spacing')
        if not 0 < self.min_circularity <= 1 or not 0 < self.center_tolerance < .5:
            raise ValueError('Invalid shape tolerances')
        if not all(np.isfinite(v) and v > 0 for v in (self.difference_threshold, self.black_delta, self.white_delta)):
            raise ValueError('Positive finite photometric thresholds required')

    @property
    def image_size(self):
        return 2*self.margin_px + (self.intersections-1)*self.spacing_px + 1

    def point(self, row, col):
        return self.margin_px + col*self.spacing_px, self.margin_px + row*self.spacing_px


@dataclass
class Detection:
    observation: BoardObservation
    rectified: np.ndarray
    overlay: np.ndarray
    reasons: tuple[tuple[str, ...], ...]
    occlusion_mask: np.ndarray


class IntersectionDetector:
    def __init__(self, empty_image, outer_intersections_xy, calibration_id, config=None):
        self.config = config or VisionConfig()
        if not calibration_id:
            raise ValueError('Calibration id required')
        self.calibration_id = calibration_id
        self._check_image(empty_image)
        self.input_shape = empty_image.shape
        corners = np.asarray(outer_intersections_xy, float)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            raise ValueError('Four corners required: top-left, top-right, bottom-right, bottom-left grid intersections')
        h, w = empty_image.shape[:2]
        if np.any(corners < 0) or np.any(corners[:, 0] >= w) or np.any(corners[:, 1] >= h):
            raise ValueError('Grid corners outside image')
        if not cv2.isContourConvex(corners.astype(np.float32)) or cv2.contourArea(corners.astype(np.float32)) < 100:
            raise ValueError('Corners must form ordered convex quadrilateral')
        n = self.config.intersections-1
        target = [self.config.point(0, 0), self.config.point(0, n), self.config.point(n, n), self.config.point(n, 0)]
        self.homography = estimate_homography(corners, target)
        self.reference = self._warp(empty_image)
        self.reference_lab = cv2.cvtColor(self.reference, cv2.COLOR_BGR2LAB).astype(float)
        self.reference_gray = cv2.cvtColor(self.reference, cv2.COLOR_BGR2GRAY).astype(float)
        self.valid_pixels = self._warp(np.full(empty_image.shape[:2], 255, np.uint8)) > 254

    @staticmethod
    def _check_image(image):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('Expected uint8 BGR image')

    def _warp(self, image):
        size = self.config.image_size
        return warp_planar(image, self.homography, (size, size))

    def detect(self, image, *, frame_id, captured_at, occlusion_mask=None):
        self._check_image(image)
        if image.shape != self.input_shape:
            raise ValueError('Image dimensions changed: recalibrate first')
        if not frame_id or not np.isfinite(captured_at):
            raise ValueError('Frame id and finite capture timestamp required')
        cfg = self.config
        rect = self._warp(image)
        lab = cv2.cvtColor(rect, cv2.COLOR_BGR2LAB).astype(float)
        gray = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY).astype(float)
        delta = np.linalg.norm(lab-self.reference_lab, axis=2)
        changed = (delta > cfg.difference_threshold).astype(np.uint8)*255
        changed = cv2.morphologyEx(changed, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(changed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        occluded = (~self.valid_pixels).astype(np.uint8)*255
        circles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 6:
                continue
            perimeter = cv2.arcLength(contour, True)
            circularity = 4*np.pi*area / max(perimeter**2, 1.)
            (x, y), radius = cv2.minEnclosingCircle(contour)
            col = int(round((x-cfg.margin_px)/cfg.spacing_px))
            row = int(round((y-cfg.margin_px)/cfg.spacing_px))
            centered = (0 <= row < cfg.intersections and 0 <= col < cfg.intersections and
                        np.linalg.norm(np.array([x, y])-cfg.point(row, col)) <= cfg.center_tolerance*cfg.spacing_px)
            if (cfg.min_stone_radius*cfg.spacing_px <= radius <= cfg.max_stone_radius*cfg.spacing_px
                    and circularity >= cfg.min_circularity and centered):
                circles.append((row, col, x, y, radius, circularity))
            else:
                cv2.drawContours(occluded, [contour], -1, 255, cv2.FILLED)
        occluded = cv2.dilate(occluded, np.ones((5, 5), np.uint8))
        if occlusion_mask is not None:
            if occlusion_mask.shape != image.shape[:2]:
                raise ValueError('Occlusion mask must match original image shape')
            occluded[self._warp((occlusion_mask > 0).astype(np.uint8)*255) > 0] = 255
        cells, scores, reasons = [], [], []
        yy, xx = np.indices(gray.shape)
        overlay = rect.copy()
        for r in range(cfg.intersections):
            row_cells, row_scores, row_reasons = [], [], []
            for c in range(cfg.intersections):
                x, y = cfg.point(r, c)
                roi = (xx-x)**2+(yy-y)**2 <= (cfg.spacing_px*.45)**2
                candidates = [circle for circle in circles if circle[:2] == (r, c)]
                cell, confidence, reason = Cell.UNKNOWN, 0., 'ambiguous_change'
                if np.any(occluded[roi]):
                    reason = 'occlusion_or_invalid_view'
                elif len(candidates) == 1:
                    _, _, cx, cy, radius, circularity = candidates[0]
                    disk = (xx-cx)**2+(yy-cy)**2 <= (radius*.65)**2
                    brightness_delta = float(np.median(gray[disk]-self.reference_gray[disk]))
                    if brightness_delta <= -cfg.black_delta:
                        cell, confidence, reason = Cell.BLACK, .96, 'dark_round_piece'
                    elif brightness_delta >= cfg.white_delta:
                        cell, confidence, reason = Cell.WHITE, .96, 'light_round_piece'
                    else:
                        reason = 'insufficient_black_white_contrast'
                elif len(candidates) > 1:
                    reason = 'multiple_components'
                elif np.mean(changed[roi] > 0) < .015:
                    cell, confidence, reason = Cell.EMPTY, .96, 'matches_empty_reference'
                row_cells.append(cell)
                row_scores.append(confidence)
                row_reasons.append(reason)
                color = {Cell.BLACK:(40, 40, 40), Cell.WHITE:(255, 255, 255), Cell.EMPTY:(0, 160, 0), Cell.UNKNOWN:(0, 0, 255)}[cell]
                cv2.circle(overlay, (x, y), 6, color, 2)
                cv2.putText(overlay, f'{r},{c}', (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX, .3, (0, 0, 200), 1)
            cells.append(tuple(row_cells)); scores.append(tuple(row_scores)); reasons.append(tuple(row_reasons))
        obs = BoardObservation(tuple(cells), tuple(scores), frame_id, captured_at, self.calibration_id)
        return Detection(obs, rect, overlay, tuple(reasons), occluded)


class BoardObserver:
    """Bridge the existing Camera.capture() to controller camera.observe()."""
    def __init__(self, camera, detector, image_sink=None):
        self.camera, self.detector = camera, detector
        self.image_sink = image_sink
        self.frame_number = 0

    def observe(self):
        image, captured_at = self.camera.capture()
        self.frame_number += 1
        if self.image_sink is not None:
            self.image_sink(str(self.frame_number),image)
        return self.detector.detect(image, frame_id=str(self.frame_number), captured_at=captured_at).observation
