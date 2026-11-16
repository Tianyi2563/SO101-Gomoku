"""【进阶 2.2】视觉闭环演示程序。

用法
----
    python app_2_2_vision.py --probe            # 只开摄像头看检测效果（不动机械臂，先调参）
    python app_2_2_vision.py --list-cameras     # 列出可用摄像头编号
    python app_2_2_vision.py                    # 正式闭环：机械臂自动朝黑球移动
    python app_2_2_vision.py --mock             # 没有机械臂时，用模拟臂跑闭环逻辑
    python app_2_2_vision.py --black-threshold 90 --tol-px 35

验收演示步骤
------------
1. 电脑屏幕上放一张"黑球"图（白底黑圆），把机械臂的腕部摄像头对准屏幕；
2. 先跑 ``--probe`` 确认检测稳定（画面里球被绿圈框住、下方显示 du/dv）；
3. 再跑不加参数的本程序：机械臂会自己一步一停地朝黑球运动，终端逐行打印像素误差与位移；
4. 把球在屏幕上挪个位置，机械臂会继续跟过去——这就是"闭环"。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from so101.config import AppConfig  # noqa: E402
from so101.presets import PresetStore  # noqa: E402
from so101.session import add_common_args, session_from_args  # noqa: E402
from so101.vision import CameraSource, annotate, BallDetector, VisualServo  # noqa: E402


def probe(cfg: AppConfig) -> int:
    """只做检测：实时显示画面与误差，用来调阈值 / 增益，不动机械臂。"""
    import cv2  # type: ignore

    detector = BallDetector(cfg.vision)
    print("=" * 76)
    print("2.2 视觉检测调试（--probe）：只检测、不动机械臂")
    print("=" * 76)
    print("把黑球图放到摄像头前。按 q 或 ESC 退出窗口。")
    print(f"当前阈值：black_threshold={cfg.vision.black_threshold}  "
          f"circularity={cfg.vision.circularity}\n")

    with CameraSource(cfg.vision) as cam:
        print(f"已打开：{cam.describe()}")
        last_print = 0.0
        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.05)
                continue
            det = detector.detect(frame, cv2)
            h, w = frame.shape[:2]
            cx, cy = w / 2.0, h / 2.0
            extra = ""
            if det.found and det.center is not None:
                du, dv = det.offset_from(cx, cy)
                extra = f"du={du:+.0f}px dv={dv:+.0f}px R={det.radius:.1f}px"
                now = time.perf_counter()
                if now - last_print > 0.5:
                    print(f"检测到黑球：{extra}  圆度={det.circularity:.3f}")
                    last_print = now
            annotate(frame, det, cfg.vision, extra)
            if cfg.vision.show_window:
                cv2.imshow("SO101 vision probe", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            else:
                time.sleep(0.03)
    if cfg.vision.show_window:
        cv2.destroyAllWindows()
    return 0


def list_cameras() -> int:
    import cv2  # type: ignore

    print("尝试打开编号 0~4 的摄像头：")
    for i in range(5):
        cap = cv2.VideoCapture(i, getattr(cv2, "CAP_DSHOW", 0))
        ok = cap.isOpened()
        if ok:
            ok, frame = cap.read()
            size = f"{frame.shape[1]}x{frame.shape[0]}" if ok and frame is not None else "无画面"
            print(f"  [{i}] 可用  ({size})")
        else:
            print(f"  [{i}] 打不开")
        cap.release()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="2.2 基于视觉的闭环控制")
    add_common_args(ap)
    ap.add_argument("--camera", type=int, default=None, help="摄像头编号（默认取配置）")
    ap.add_argument("--probe", action="store_true", help="只做检测调试，不动机械臂")
    ap.add_argument("--list-cameras", action="store_true", help="列出可用摄像头")
    ap.add_argument("--black-threshold", type=int, default=None, help="黑色阈值 0~255，越大越宽松")
    ap.add_argument("--circularity", type=float, default=None, help="圆度阈值 0~1，越大越严格")
    ap.add_argument("--tol-px", type=float, default=None, help="认为对准的像素容差")
    ap.add_argument("--gain-xy", type=float, default=None, help="像素误差 -> 横向位移 (米/像素)")
    ap.add_argument("--gain-z", type=float, default=None, help="像素误差 -> 纵向位移 (米/像素)")
    ap.add_argument("--max-step-mm", type=float, default=None, help="单步最大位移（mm）")
    ap.add_argument("--max-iter", type=int, default=None, help="最大迭代次数")
    ap.add_argument("--no-window", action="store_true", help="不弹 OpenCV 窗口")
    ap.add_argument("--pre-pose", default=None, help="闭环开始前先切到某个预设姿态（例如 forward）")
    args = ap.parse_args()

    if args.list_cameras:
        return list_cameras()

    cfg = AppConfig.load(args.config)
    if args.camera is not None:
        cfg.vision.camera_index = args.camera
    if args.black_threshold is not None:
        cfg.vision.black_threshold = args.black_threshold
    if args.circularity is not None:
        cfg.vision.circularity = args.circularity
    if args.tol_px is not None:
        cfg.vision.center_tol_px = args.tol_px
    if args.gain_xy is not None:
        cfg.vision.gain_xy = args.gain_xy
    if args.gain_z is not None:
        cfg.vision.gain_z = args.gain_z
    if args.max_step_mm is not None:
        cfg.vision.max_step_mm = args.max_step_mm
    if args.no_window:
        cfg.vision.show_window = False
    cfg.save()  # 把这次调好的参数记下来，下次直接生效

    if args.probe:
        return probe(cfg)

    with session_from_args(args) as s:
        store = PresetStore.load(s.cfg.presets_path, model=s.model, jmap=s.jmap)
        if args.pre_pose:
            store.play_pose(s.arm, args.pre_pose, wait=True)
            s.arm.wait_until_settled(timeout=6.0)

        with CameraSource(cfg.vision) as cam:
            print(f"摄像头：{cam.describe()}")
            servo = VisualServo(arm=s.arm, model=s.model, cfg=cfg.vision)
            converged = servo.run(cam, max_iterations=args.max_iter)

        if servo.history:
            print("\n--- 闭环轨迹（每步的像素误差与末端位移）---")
            for st in servo.history[-15:]:
                print(
                    f"  #{st.index:02d} du={st.du:+7.1f} dv={st.dv:+7.1f} R={st.radius:6.1f} "
                    f"位移=({st.delta_mm[0]:+6.1f},{st.delta_mm[1]:+6.1f},{st.delta_mm[2]:+6.1f})mm"
                )
        print(f"\n闭环结束：{'已对准' if converged else '未完全对准（可按需求调增益后重跑）'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
