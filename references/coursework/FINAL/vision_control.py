from __future__ import annotations

import argparse
import msvcrt
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np

from so101_arm.arm_controller import ArmController

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(errors="replace")

FIRST_PERSON_CAMERA = 1
DEFAULT_BLACK_THRESHOLD = 129


@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    radius: float
    circularity: float


class LiveDetectionView:
    """Tk preview used alongside the control loop (works with headless OpenCV)."""

    def __init__(self) -> None:
        import tkinter as tk

        self.tk = tk
        self.closed = False
        self.root = tk.Tk()
        self.root.title("SO-101 视觉闭环：实时检测 + 自动运动")
        self.root.configure(bg="#202020")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda _event: self.close())
        self.root.bind("q", lambda _event: self.close())
        self.status = tk.StringVar(value="正在读取摄像头……")
        tk.Label(
            self.root,
            textvariable=self.status,
            bg="#202020",
            fg="white",
            font=("Microsoft YaHei UI", 12),
        ).pack(padx=10, pady=(10, 4))
        self.image_label = tk.Label(self.root, bg="black")
        self.image_label.pack(padx=10, pady=(4, 10))

    def show(self, frame: np.ndarray, threshold: int, target: Detection | None) -> None:
        if self.closed:
            return
        annotated = frame.copy()
        height, width = annotated.shape[:2]
        cv2.drawMarker(annotated, (width // 2, height // 2), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 7)
        mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)[1]
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        if target is None:
            self.status.set("未检测到锁定目标｜机械臂保持不动｜Q / Esc 停止")
        else:
            center = (round(target.x), round(target.y))
            cv2.circle(annotated, center, round(target.radius), (0, 255, 0), 3)
            cv2.circle(annotated, center, 4, (0, 255, 0), -1)
            self.status.set(
                f"目标圆心=({target.x:.0f}, {target.y:.0f})  "
                f"半径={target.radius:.0f}px  圆度={target.circularity:.2f}｜Q / Esc 停止"
            )
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_bgr, "BLACK MASK", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        annotated = cv2.resize(annotated, (480, 360))
        mask_bgr = cv2.resize(mask_bgr, (480, 360), interpolation=cv2.INTER_NEAREST)
        combined = np.hstack((annotated, mask_bgr))
        rgb = cv2.cvtColor(combined, cv2.COLOR_BGR2RGB)
        out_height, out_width = rgb.shape[:2]
        ppm = f"P6\n{out_width} {out_height}\n255\n".encode() + rgb.tobytes()
        photo = self.tk.PhotoImage(data=ppm, format="PPM")
        self.image_label.configure(image=photo)
        self.image_label.image = photo
        try:
            self.root.update_idletasks()
            self.root.update()
        except self.tk.TclError:
            self.closed = True

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass


def detect_black_circle(
    frame: np.ndarray,
    threshold: int = DEFAULT_BLACK_THRESHOLD,
    min_area: int = 500,
    min_radius: float = 28,
    edge_margin: int = 0,
    min_local_contrast: float = 45,
) -> Detection | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # A circle on a screen becomes an ellipse under perspective. Median blur
    # removes camera/screen moire without rounding rectangular objects as much
    # as Gaussian blur does.
    gray = cv2.medianBlur(gray, 7)
    mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)[1]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[tuple[float, Detection]] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area < min_area or perimeter <= 0:
            continue
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        (x, y), radius = cv2.minEnclosingCircle(contour)
        height, width = frame.shape[:2]
        touches_edge = (
            x - radius <= edge_margin
            or y - radius <= edge_margin
            or x + radius >= width - edge_margin
            or y + radius >= height - edge_margin
        )
        if len(contour) < 5:
            continue
        (_, _), (axis_a, axis_b), _ = cv2.fitEllipse(contour)
        ellipse_area = np.pi * axis_a * axis_b / 4
        ellipse_fill = area / ellipse_area if ellipse_area > 0 else 0
        axis_ratio = min(axis_a, axis_b) / max(axis_a, axis_b) if max(axis_a, axis_b) > 0 else 0
        contour_mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        expanded_mask = cv2.dilate(contour_mask, np.ones((31, 31), np.uint8))
        ring_mask = cv2.subtract(expanded_mask, contour_mask)
        inside_brightness = cv2.mean(gray, mask=contour_mask)[0]
        ring_brightness = cv2.mean(gray, mask=ring_mask)[0]
        local_contrast = ring_brightness - inside_brightness
        # ellipse_fill rejects black rectangles, while axis_ratio still allows
        # a strongly foreshortened screen circle. Local contrast identifies the
        # black target on its bright screen instead of dark round scene objects.
        if (
            circularity >= 0.58
            and ellipse_fill >= 0.82
            and axis_ratio >= 0.30
            and local_contrast >= min_local_contrast
            and radius >= min_radius
            and not touches_edge
        ):
            score = area * local_contrast
            candidates.append((score, Detection(x, y, radius, float(circularity))))
    return max(candidates, default=(0, None), key=lambda item: item[0])[1]


def visual_servo_delta(
    detection: Detection,
    width: int,
    height: int,
    *,
    step_m: float = 0.005,
    center_tolerance_px: int = 30,
    target_radius_px: float = 90,
    x_sign: float = 1.0,
    z_sign: float = 1.0,
) -> np.ndarray:
    """Map image error to robot +Y(left), +Z(up), then +X(forward)."""
    error_x = detection.x - width / 2
    error_y = detection.y - height / 2
    delta = np.zeros(3)
    if abs(error_x) > center_tolerance_px:
        delta[1] = -np.sign(error_x) * step_m * x_sign
    if abs(error_y) > center_tolerance_px:
        delta[2] = -np.sign(error_y) * step_m * z_sign
    lateral_norm = float(np.linalg.norm(delta))
    if lateral_norm > 0:
        # Keep the total Cartesian displacement equal to step_m even when two
        # image axes need correction at the same time.
        delta *= step_m / lateral_norm
    elif detection.radius < target_radius_px:
        delta[0] = step_m
    return delta


def stop_requested() -> bool:
    if not msvcrt.kbhit():
        return False
    return msvcrt.getwch().lower() in {"q", "\x1b", " "}


def main() -> int:
    parser = argparse.ArgumentParser(description="用腕部摄像头追踪屏幕上的黑色圆形")
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--step-mm", type=float, default=5.0)
    parser.add_argument("--target-radius", type=float, default=90.0)
    parser.add_argument(
        "--max-steps",
        type=int,
        default=0,
        help="最多执行多少次 IK 移动；0 表示持续运行直到手动停止",
    )
    parser.add_argument("--confirm-frames", type=int, default=3, help="连续稳定识别多少帧后才移动")
    parser.add_argument("--min-radius", type=float, default=28, help="忽略半径小于该像素值的黑圆")
    parser.add_argument(
        "--threshold",
        type=int,
        default=DEFAULT_BLACK_THRESHOLD,
        help="黑色灰度阈值，当前摄像头和屏幕实测推荐 129",
    )
    # Empirically calibrated for this wrist-camera mounting. These remain CLI
    # options so another camera orientation can reverse either image axis.
    parser.add_argument("--x-sign", type=float, choices=(-1, 1), default=1)
    parser.add_argument("--z-sign", type=float, choices=(-1, 1), default=-1)
    parser.add_argument("--live-view", action="store_true", help="运动时实时显示原图、检测圆和掩膜")
    parser.add_argument("--preview", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if (
        args.max_steps < 0
        or args.confirm_frames < 1
        or args.min_radius <= 0
        or not 0 <= args.threshold <= 255
    ):
        parser.error("--max-steps 不能为负，--confirm-frames 和 --min-radius 必须为正数")
    camera = cv2.VideoCapture(FIRST_PERSON_CAMERA, cv2.CAP_DSHOW)
    if not camera.isOpened():
        camera.release()
        camera = cv2.VideoCapture(FIRST_PERSON_CAMERA)
    if not camera.isOpened():
        raise RuntimeError(f"无法打开第一人称摄像头 {FIRST_PERSON_CAMERA}")
    print(f"使用第一人称摄像头 {FIRST_PERSON_CAMERA}")
    print("持续监控模式：Q/空格/Esc 或 Ctrl+C 才停止；--max-steps 只统计真实移动次数。")
    live_view = LiveDetectionView() if args.live_view or args.preview else None

    try:
        with ArmController(args.port, dry_run=args.dry_run) as arm:
            move_count = 0
            missing_count = 0
            stable_count = 0
            previous: Detection | None = None
            reached_announced = False
            while True:
                if stop_requested():
                    print("收到停止按键，退出视觉控制。")
                    break
                ok, frame = camera.read()
                if not ok:
                    print("摄像头取帧失败；保持不动并继续重试。")
                    time.sleep(0.2)
                    continue
                target = detect_black_circle(
                    frame,
                    threshold=args.threshold,
                    min_radius=args.min_radius,
                )
                if live_view is not None:
                    live_view.show(frame, args.threshold, target)
                    if live_view.closed:
                        print("实时画面窗口已关闭，退出视觉控制。")
                        break
                if target is None:
                    missing_count += 1
                    stable_count = 0
                    previous = None
                    reached_announced = False
                    if missing_count == 1 or missing_count % 10 == 0:
                        print(f"未检测到合格黑圆，继续等待（已等待 {missing_count} 帧）；机械臂保持不动。")
                    time.sleep(0.05)
                    continue
                missing_count = 0
                if previous is not None:
                    center_shift = float(np.hypot(target.x - previous.x, target.y - previous.y))
                    radius_shift = abs(target.radius - previous.radius)
                    stable_count = stable_count + 1 if center_shift <= 15 and radius_shift <= 8 else 1
                else:
                    stable_count = 1
                previous = target
                if stable_count < args.confirm_frames:
                    print(
                        f"发现候选黑圆，等待稳定 {stable_count}/{args.confirm_frames}: "
                        f"圆心=({target.x:.0f},{target.y:.0f}), 半径={target.radius:.0f}"
                    )
                    time.sleep(0.05)
                    continue
                delta = visual_servo_delta(
                    target,
                    frame.shape[1],
                    frame.shape[0],
                    step_m=args.step_mm / 1000,
                    target_radius_px=args.target_radius,
                    x_sign=args.x_sign,
                    z_sign=args.z_sign,
                )
                print(
                    f"闭环移动 {move_count + 1}: 圆心=({target.x:.0f},{target.y:.0f}), "
                    f"半径={target.radius:.0f}, 修正[m]={delta.tolist()}"
                )
                if not np.any(delta):
                    if not reached_announced:
                        print("目标已居中且足够接近；保持监控，不自动退出。")
                        reached_announced = True
                    time.sleep(0.1)
                    continue
                reached_announced = False
                arm.move_vector(delta)
                move_count += 1
                stable_count = 0
                previous = None
                if args.max_steps and move_count >= args.max_steps:
                    action_name = "IK 规划" if args.dry_run else "真实移动"
                    print(f"已完成设定的 {args.max_steps} 次{action_name}，退出。")
                    break
    except KeyboardInterrupt:
        print("收到 Ctrl+C，退出视觉控制。")
    finally:
        camera.release()
        if live_view is not None:
            live_view.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
