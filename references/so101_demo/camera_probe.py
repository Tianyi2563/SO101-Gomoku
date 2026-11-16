"""Save one frame from each available camera so the wrist camera can be selected."""
from pathlib import Path
import cv2

output = Path(__file__).resolve().parent.parent / "device_check"
output.mkdir(exist_ok=True)
for index in range(4):
    camera = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    try:
        if not camera.isOpened():
            print(f"camera {index}: unavailable")
            continue
        ok, frame = camera.read()
        if not ok or frame is None:
            print(f"camera {index}: no frame")
            continue
        path = output / f"camera_{index}.jpg"
        encoded, data = cv2.imencode(".jpg", frame)
        if not encoded:
            print(f"camera {index}: encode failed")
            continue
        data.tofile(str(path))
        print(f"camera {index}: {frame.shape[1]}x{frame.shape[0]} -> {path}")
    finally:
        camera.release()
