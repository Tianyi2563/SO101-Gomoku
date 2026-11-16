"""视觉闭环端到端模拟测试（不需要摄像头、不需要机械臂）。

用一个"虚拟屏幕 + 虚拟摄像头"把物理关系写死：

* 球固定在世界坐标 ``ball_world``；
* 摄像头装在末端、朝 +x 看，图像右 = 世界 -y、图像上 = 世界 +z；
* 投影：``u = cx - f·(y_ball-y_eef)/Δx``、``v = cy - f·(z_ball-z_eef)/Δx``、
  ``r = f·R/Δx``（Δx = 球相对末端的 x 距离）。

然后把这张"渲染出来的画面"喂给真正的 :class:`so101.vision.VisualServo`，
看它能不能一步一停地把误差压到容差以内。这验证的是**闭环控制律本身**
（映射方向、增益、限幅、收敛性），而不是摄像头硬件。

用法：
    python tools/test_vision_loop.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.config import VisionConfig  # noqa: E402
from so101.session import open_session  # noqa: E402
from so101.vision import BallDetector, VisualServo  # noqa: E402

FAIL = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        FAIL += 1


class FakeScreen:
    """虚拟摄像头：按当前末端位置渲染屏幕上的黑球。"""

    def __init__(
        self,
        arm,
        ball_world: tuple[float, float, float] = (0.75, 0.08, 0.30),
        ball_radius_m: float = 0.05,
        focal: float = 500.0,
        size: tuple[int, int] = (640, 480),
    ) -> None:
        self.arm = arm
        self.ball = np.asarray(ball_world, dtype=float)
        self.ball_r = ball_radius_m
        self.f = focal
        self.w, self.h = size
        # 把屏幕坐标系（x 前、y 左、z 上）映射到末端坐标系用的姿态保持不变，
        # 这里直接假设末端姿态不随运动改变（零空间正则项保证了这一点）。

    def read(self):
        import cv2  # type: ignore

        tip = self.arm.tip_position()
        rel = self.ball - tip
        img = np.full((self.h, self.w, 3), 235, dtype=np.uint8)
        if rel[0] < 0.12:  # 球跑到身后 / 太近，看不见
            return img
        u = self.w / 2.0 - self.f * rel[1] / rel[0]
        v = self.h / 2.0 - self.f * rel[2] / rel[0]
        r = self.f * self.ball_r / rel[0]
        if r < 3:
            return img
        cv2.circle(
            img,
            (int(round(u)), int(round(v))),
            int(round(r)),
            (10, 10, 10),
            -1,
        )
        return img


def main() -> int:
    try:
        import cv2  # type: ignore  # noqa: F401
    except ImportError:
        print("没有安装 opencv-python，跳过闭环测试")
        return 0

    vcfg = VisionConfig()
    vcfg.gain_xy = 0.00040
    vcfg.gain_z = 0.00040
    vcfg.gain_depth = 0.00015
    vcfg.center_tol_px = 25.0
    vcfg.max_step_mm = 25.0
    vcfg.max_iterations = 80
    vcfg.settle_frames = 3
    vcfg.show_window = False
    vcfg.black_threshold = 70

    with open_session(mock=True, verbose=False) as s:
        # 先把机械臂摆到"正对屏幕"的初始姿态
        s.arm.move_to({"shoulder_pan": 0.0, "shoulder_lift": -10.0, "elbow_flex": 5.0,
                       "wrist_flex": 5.0, "wrist_roll": 0.0}, duration=0.5, wait=True)
        p0 = s.arm.tip_position()
        print(f"初始末端位置 = ({p0[0]:.3f}, {p0[1]:.3f}, {p0[2]:.3f}) m")

        cam = FakeScreen(s.arm)
        det = BallDetector(vcfg)
        frame = cam.read()
        d0 = det.detect(frame, cv2 if False else None)
        check("虚拟画面里能检测到黑球", d0.found, f"r={d0.radius:.1f}px")
        if not d0.found:
            return 1
        du0 = d0.center[0] - 320  # type: ignore[index]
        dv0 = d0.center[1] - 240  # type: ignore[index]
        e0 = math.hypot(du0, dv0)
        print(f"初始像素误差 = ({du0:+.1f}, {dv0:+.1f}) px, 半径 = {d0.radius:.1f}px")
        check("初始误差足够大（有得可动）", e0 > 60, f"{e0:.1f}px")

        servo = VisualServo(arm=s.arm, model=s.model, cfg=vcfg)
        converged = servo.run(cam, show_window=False)

        p1 = s.arm.tip_position()
        info = ""
        last = servo.history[-1] if servo.history else None
        if last is not None:
            info = f"最终误差 = ({last.du:+.1f}, {last.dv:+.1f}) px, 半径 = {last.radius:.1f}px"
        print(f"最终末端位置 = ({p1[0]:.3f}, {p1[1]:.3f}, {p1[2]:.3f}) m")
        print(info)

        check("闭环收敛（|误差| <= 容差）", converged, info)
        if last is not None:
            check("末端移动方向朝球（y 方向靠拢）",
                  abs(p1[1] - cam.ball[1]) < abs(p0[1] - cam.ball[1]),
                  f"|Δy|: {abs(p0[1] - cam.ball[1]) * 100:.1f}cm -> {abs(p1[1] - cam.ball[1]) * 100:.1f}cm")
            check("末端移动方向朝球（z 方向靠拢）",
                  abs(p1[2] - cam.ball[2]) < abs(p0[2] - cam.ball[2]),
                  f"|Δz|: {abs(p0[2] - cam.ball[2]) * 100:.1f}cm -> {abs(p1[2] - cam.ball[2]) * 100:.1f}cm")
            check("迭代次数在限制内", len(servo.history) <= vcfg.max_iterations,
                  f"{len(servo.history)} 步")

        # --- 换个球的位置，验证"球动到哪就跟到哪" ---
        print()
        print("把球挪到另一个位置再跑一次 ...")
        cam2 = FakeScreen(s.arm, ball_world=(0.72, -0.06, 0.24), ball_radius_m=0.04)
        servo2 = VisualServo(arm=s.arm, model=s.model, cfg=vcfg)
        p2 = s.arm.tip_position()
        converged2 = servo2.run(cam2, show_window=False)
        p3 = s.arm.tip_position()
        last2 = servo2.history[-1] if servo2.history else None
        info2 = f"最终误差 = ({last2.du:+.1f}, {last2.dv:+.1f}) px" if last2 else ""
        check("换目标后同样收敛", converged2, info2)
        check("末端朝新方向移动（y 变负）", p3[1] < p2[1], f"y: {p2[1] * 100:.1f}cm -> {p3[1] * 100:.1f}cm")

    print()
    summary = "全部通过" if FAIL == 0 else f"{FAIL} 项失败"
    print(f"结果：{summary}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
