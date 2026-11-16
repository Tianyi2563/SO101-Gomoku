"""【基础功能 1.2】逆运动学：键盘控制末端夹爪走直线。

按键（每按一次沿该方向走一步，步长可调）
----------------------------------------
    w / ↑       末端向前 (+x)
    s / ↓       末端向后 (-x)
    a / ←       末端向左 (+y)
    d / →       末端向右 (-y)
    r / PgUp    末端向上 (+z)
    f / PgDn    末端向下 (-z)

    1 / 2 / 3   步长 5mm / 10mm / 20mm
    g           夹爪张开 / 闭合
    h           回到中位（预设里的第一个姿态）
    p           打印当前末端位置、关节角，以及 IK 是否顶到限位
    i           打印正/逆运动学原理速览（讲解用）
    q / esc     退出

要点
----
* 键盘只控制**末端位置**，5 个关节角全部由 :func:`so101.kinematics.ik_dls` 算出；
* 运动是"走直线"：先算目标点，再用 IK 反解关节角；
* 若目标点够不着，会打印 **IK 无解**，并把步长逐级减半，沿原方向走到可达边界为止。

示例
----
    python app_1_2_ik.py --mock
    python app_1_2_ik.py --port COM5 --step 5      # 每次 5mm，慢慢演示
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from so101.keys import make_key_reader  # noqa: E402
from so101.kinematics import explain_2link_demo  # noqa: E402
from so101.presets import PresetStore  # noqa: E402
from so101.session import add_common_args, session_from_args  # noqa: E402

# 键 -> (方向向量, 中文说明)
DIRECTIONS: dict[str, tuple[tuple[float, float, float], str]] = {
    "w": ((1.0, 0.0, 0.0), "前 (+x)"),
    "up": ((1.0, 0.0, 0.0), "前 (+x)"),
    "s": ((-1.0, 0.0, 0.0), "后 (-x)"),
    "down": ((-1.0, 0.0, 0.0), "后 (-x)"),
    "a": ((0.0, 1.0, 0.0), "左 (+y)"),
    "left": ((0.0, 1.0, 0.0), "左 (+y)"),
    "d": ((0.0, -1.0, 0.0), "右 (-y)"),
    "right": ((0.0, -1.0, 0.0), "右 (-y)"),
    "r": ((0.0, 0.0, 1.0), "上 (+z)"),
    "f": ((0.0, 0.0, -1.0), "下 (-z)"),
}


def print_state(session, note: str = "") -> None:
    st = session.arm.state()
    tip = st.tip
    print(
        f"末端 = ({tip[0] * 1000:+7.1f}, {tip[1] * 1000:+7.1f}, {tip[2] * 1000:+7.1f}) mm   "
        f"关节(度) = " + " ".join(f"{v:+6.1f}" for v in st.action.values())
        + (f"   {note}" if note else "")
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="1.2 逆运动学：键盘控制末端走直线")
    add_common_args(ap)
    ap.add_argument("--step", type=float, default=None, help="单步距离（mm），默认取配置")
    ap.add_argument("--speed", type=float, default=None, help="末端运动速度（mm/s）")
    args = ap.parse_args()

    with session_from_args(args) as s:
        step_mm = float(args.step if args.step is not None else s.cfg.ik.step_mm)
        speed = float(args.speed if args.speed is not None else s.cfg.ik.speed_mm_s)
        store = PresetStore.load(s.cfg.presets_path, model=s.model, jmap=s.jmap)
        reader = make_key_reader()
        gripper_open = False

        print("=" * 76)
        print("1.2 逆运动学：键盘操控末端（前后 / 左右 / 上下）走直线")
        print("=" * 76)
        print(f"键盘后端：{reader.describe()}")
        print(f"步长 {step_mm:.0f} mm，末端速度 {speed:.0f} mm/s")
        print("w/s 前后 | a/d 左右 | r/f 上下 | 1/2/3 换步长 | p 打印 | i 原理 | q 退出")
        print()
        print_state(s, "← 当前末端位置（相对基座，单位 mm）")

        single_steps = {"1": 5.0, "2": 10.0, "3": 20.0}
        try:
            while True:
                for key in reader.poll():
                    if key in ("q", "esc"):
                        print("退出。")
                        return 0
                    if key in DIRECTIONS:
                        direction, label = DIRECTIONS[key]
                        s.arm.move_tip_by(list(direction), step_mm / 1000.0, max_speed_mm_s=speed)
                        # 等这一步走完再接收下一个按键，保证轨迹可预测
                        s.arm.wait_until_settled(timeout=6.0)
                        print_state(s, f"← {label} {step_mm:.0f}mm")
                    elif key in single_steps:
                        step_mm = single_steps[key]
                        print(f"步长 -> {step_mm:.0f} mm")
                    elif key == "g":
                        gripper_open = not gripper_open
                        s.arm.set_gripper(90.0 if gripper_open else 10.0, wait=True)
                        print(f"夹爪 -> {'张开' if gripper_open else '闭合'}")
                    elif key == "p":
                        print_state(s)
                    elif key == "i":
                        print()
                        print(explain_2link_demo())
                        print("SO101 五自由度用的是上式的推广：数值迭代（阻尼最小二乘）")
                        print("  Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e + 零空间项，迭代到末端残差 < 0.1mm")
                        print()
                    elif key == "h":
                        first = next(iter(store.poses), None)
                        if first:
                            store.play_pose(s.arm, first, wait=True)
                            print_state(s, "← 回中位")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，停止运动并退出。")
            s.arm.hold()
        finally:
            reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
