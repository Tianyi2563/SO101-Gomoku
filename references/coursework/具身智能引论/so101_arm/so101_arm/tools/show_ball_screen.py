"""在屏幕上全屏显示一个黑球，作为视觉闭环的目标（验收当靶子用）。

用法：
    python tools/show_ball_screen.py                 # 白底 + 屏幕中央黑球
    python tools/show_ball_screen.py --radius 110    # 指定半径（像素）
    python tools/show_ball_screen.py --bg 255 --ball 8

按键：
    方向键 / w a s d   移动球（演示"球动到哪，机械臂跟到哪"）
    + / -              放大 / 缩小
    r                  回到屏幕中央
    空格               在"纯白底"和"带网格底"之间切换（网格方便观察视角）
    esc / q            退出

提示：
    * 建议把窗口拖到机械臂摄像头正对的位置，然后按 F11 那种全屏效果已经默认开启；
    * 屏幕亮度调高一些、避免反光，黑球检测会更稳。
"""

from __future__ import annotations

import argparse


def main() -> int:
    ap = argparse.ArgumentParser(description="全屏黑球靶子")
    ap.add_argument("--radius", type=int, default=90, help="黑球半径（像素）")
    ap.add_argument("--bg", type=int, default=255, help="背景灰度 0~255")
    ap.add_argument("--ball", type=int, default=8, help="黑球灰度 0~255")
    ap.add_argument("--grid", action="store_true", help="背景叠加网格")
    args = ap.parse_args()

    import cv2  # type: ignore
    import numpy as np

    win = "SO101 ball target (arrows to move, +/-, r reset, esc quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    screen = None
    try:  # 让全屏窗口尺寸尽量匹配真实分辨率
        import ctypes

        user32 = ctypes.windll.user32
        user32.SetProcessDPIAware()
        screen = (user32.GetSystemMetrics(0), user32.GetSystemMetrics(1))
    except Exception:  # noqa: BLE001
        screen = None

    # 先显示一帧空画面，让窗口按全屏尺寸创建
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.imshow(win, blank)
    cv2.waitKey(50)

    radius = int(args.radius)
    show_grid = bool(args.grid)

    def render(w: int, h: int, cx: float, cy: float) -> "np.ndarray":
        img = np.full((h, w, 3), args.bg, dtype=np.uint8)
        if show_grid:
            step = 40
            for x in range(0, w, step):
                img[:, x] = np.minimum(img[:, x].astype(int) + 40, 255).astype(np.uint8)
            for y in range(0, h, step):
                img[y, :] = np.minimum(img[y, :].astype(int) + 40, 255).astype(np.uint8)
        cv2.circle(img, (int(round(cx)), int(round(cy))), radius, (args.ball,) * 3, -1)
        return img

    w = screen[0] if screen else 1280
    h = screen[1] if screen else 720
    cx, cy = w / 2.0, h / 2.0
    print(f"窗口分辨率约 {w}x{h}，黑球半径 {radius}px")
    print("方向键移动球，+/- 调大小，空格切换网格，esc 退出。")

    while True:
        img = render(w, h, cx, cy)
        cv2.imshow(win, img)
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q")):
            break
        step = 20
        if key in (82, ord("w"), ord("W")):      # 上
            cy -= step
        elif key in (84, ord("s"), ord("S")):    # 下
            cy += step
        elif key in (81, ord("a"), ord("A")):    # 左
            cx -= step
        elif key in (83, ord("d"), ord("D")):    # 右
            cx += step
        elif key in (ord("+"), ord("=")):
            radius = min(radius + 10, min(w, h) // 3)
        elif key in (ord("-"), ord("_")):
            radius = max(radius - 10, 20)
        elif key == ord("r"):
            cx, cy = w / 2.0, h / 2.0
        elif key == ord(" "):
            show_grid = not show_grid

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
