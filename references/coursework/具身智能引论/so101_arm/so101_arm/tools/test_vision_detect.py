"""黑球检测算法的离线自测（用合成图像，不需要摄像头）。

覆盖：
  * 白底黑圆 -> 能检出，圆心/半径准确；
  * 黑色方块 -> 因圆度不足被拒；
  * 纯白画面 -> found=False；
  * 多个黑圆 -> 选出更大的那个；
  * 闭环映射：像素误差 -> 末端位移的方向是否正确（左右/上下/远近）。

用法：
    python tools/test_vision_detect.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.config import VisionConfig  # noqa: E402
from so101.vision import BallDetector  # noqa: E402

FAIL = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        FAIL += 1


def white_canvas(w: int = 640, h: int = 480) -> np.ndarray:
    return np.full((h, w, 3), 230, dtype=np.uint8)


def main() -> int:
    try:
        import cv2  # type: ignore
    except ImportError:
        print("没有安装 opencv-python，跳过视觉检测测试（pip install opencv-python）")
        return 0

    cfg = VisionConfig()
    det = BallDetector(cfg)

    print("--- 1) 白底黑圆 ---")
    img = white_canvas()
    cv2.circle(img, (400, 300), 60, (12, 12, 12), -1)
    res = det.detect(img, cv2)
    check("能检出黑球", res.found)
    if res.found and res.center:
        du = abs(res.center[0] - 400)
        dv = abs(res.center[1] - 300)
        check("圆心位置准确", du < 3 and dv < 3, f"偏差 ({du:.1f},{dv:.1f}) px")
        check("半径接近 60px", abs(res.radius - 60) < 4, f"r={res.radius:.1f}")
        check("圆度接近 1", res.circularity > 0.85, f"circ={res.circularity:.3f}")
        check("填充率接近 1", res.fill_ratio > 0.9, f"fill={res.fill_ratio:.3f}")

    print()
    print("--- 2) 黑色方块（非圆，应被拒）---")
    img = white_canvas()
    cv2.rectangle(img, (250, 180), (390, 320), (10, 10, 10), -1)
    res = det.detect(img, cv2)
    check("方块被圆度条件拒绝", not res.found, f"found={res.found}")

    print()
    print("--- 3) 纯白画面（无目标）---")
    res = det.detect(white_canvas(), cv2)
    check("无目标时 found=False", not res.found)

    print()
    print("--- 4) 一大一小两个黑圆（选大的）---")
    img = white_canvas()
    cv2.circle(img, (120, 120), 20, (15, 15, 15), -1)
    cv2.circle(img, (420, 330), 70, (15, 15, 15), -1)
    res = det.detect(img, cv2)
    check("选中较大的圆", res.found and res.radius > 50, f"r={res.radius:.1f}")
    if res.found and res.center:
        check("中心在 (420,330) 附近", abs(res.center[0] - 420) < 5 and abs(res.center[1] - 330) < 5,
              f"({res.center[0]:.0f},{res.center[1]:.0f})")

    print()
    print("--- 5) 闭环映射方向 ---")
    from so101.vision import VisualServo

    class _DummyArm:
        pass

    servo = VisualServo(arm=_DummyArm(), model=None, cfg=cfg)  # type: ignore[arg-type]
    # 球在画面右侧 -> 末端应往"图像右轴"走
    d = servo.delta_from_error(100.0, 0.0, 1.0)
    check("球偏右 -> 沿 image_right_axis 移动", d @ np.asarray(cfg.image_right_axis) > 0, f"delta={np.round(d * 1000, 2)}mm")
    # 球在画面下方 -> 沿 image_down_axis
    d = servo.delta_from_error(0.0, 100.0, 1.0)
    check("球偏下 -> 沿 image_down_axis 移动", d @ np.asarray(cfg.image_down_axis) > 0, f"delta={np.round(d * 1000, 2)}mm")
    # 球比参考大 -> 后退（远离）
    d = servo.delta_from_error(0.0, 0.0, 1.5)
    check("球变大 -> 沿 toward_ball_axis 反向", d @ np.asarray(cfg.toward_ball_axis) < 0, f"delta={np.round(d * 1000, 2)}mm")
    d = servo.delta_from_error(0.0, 0.0, 0.6)
    check("球变小 -> 沿 toward_ball_axis 靠近", d @ np.asarray(cfg.toward_ball_axis) > 0, f"delta={np.round(d * 1000, 2)}mm")
    # 限幅
    d = servo.delta_from_error(1000.0, 1000.0, 1.0)
    check("单步位移被限幅", float(np.linalg.norm(d)) <= cfg.max_step_mm / 1000.0 + 1e-9,
          f"|delta|={np.linalg.norm(d) * 1000:.2f}mm <= {cfg.max_step_mm:.0f}mm")

    print()
    print("--- 6) 自动方向纠错 ---")
    servo2 = VisualServo(arm=_DummyArm(), model=None, cfg=cfg)  # type: ignore[arg-type]
    before = servo2.signs.copy()
    servo2._observe_error(100.0)
    servo2._observe_error(120.0)
    servo2._observe_error(150.0)
    check("误差连续变大后翻转符号", not np.array_equal(servo2.signs, before), f"signs={servo2.signs.astype(int).tolist()}")

    print()
    summary = "全部通过" if FAIL == 0 else f"{FAIL} 项失败"
    print(f"结果：{summary}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
