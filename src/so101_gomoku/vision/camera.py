"""Minimal OpenCV capture. Construction does not open hardware."""
import time
import cv2


class Camera:
    def __init__(self, index=0):
        self.index = index
        self.cap = None

    def open(self):
        if self.cap is not None:
            raise RuntimeError("Camera already open")
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError("Cannot open camera")
        self.cap = cap

    def capture(self):
        if self.cap is None:
            raise RuntimeError("Camera not open")
        ok, image = self.cap.read()
        if not ok:
            raise RuntimeError("Camera frame unavailable")
        return image, time.time()

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
