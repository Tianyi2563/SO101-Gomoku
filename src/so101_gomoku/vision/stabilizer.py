"""Require consecutive confident observations; distinct ordered frames only."""
from so101_gomoku.game.reconciliation import confirmed_cells


class BoardStabilizer:
    def __init__(self, frames=3, min_confidence=0.9):
        if frames < 1 or not 0 <= min_confidence <= 1:
            raise ValueError("Invalid stabilization configuration")
        self.frames, self.min_confidence = frames, min_confidence
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0

    def update(self, observation):
        old = self.previous
        if old and (observation.frame_id == old.frame_id or observation.captured_at <= old.captured_at):
            self.reset()
            return None
        try:
            confirmed_cells(observation, self.min_confidence)
        except ValueError:
            self.reset()
            return None
        same = old and old.cells == observation.cells and old.calibration_id == observation.calibration_id
        self.count = self.count + 1 if same else 1
        self.previous = observation
        return observation if self.count >= self.frames else None
