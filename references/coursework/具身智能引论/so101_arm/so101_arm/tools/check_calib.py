"""标定自检：确认 URDF/IK 算出来的角度和真机是同一套刻度。

它会做三件事：

1. 打印关节↔动作值的换算法（offset / scale / sign），以及当前姿态；
2. 用 FK 预测当前末端位置，让你拿卷尺量一量对不对；
3. ``--wiggle``：依次让每个关节单独动 +10°，你观察真机往哪边动、动多少，
   立刻就能判断符号和尺度是否正确。

用法：
    python tools/check_calib.py                 # 只打印当前状态
    python tools/check_calib.py --wiggle        # 逐个关节小幅测试（会真的动！）
    python tools/check_calib.py --mock          # 无硬件时看看输出长什么样
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.config import ALL_MOTORS, ARM_JOINTS  # noqa: E402
from so101.frames import describe  # noqa: E402
from so101.session import add_common_args, session_from_args  # noqa: E402


def report(session, label: str = "") -> None:
    st = session.arm.state()
    q = st.q
    tip = st.tip
    print(f"{label}")
    print("  动作值(度) :", "  ".join(f"{k}={st.action[k]:+7.2f}" for k in ALL_MOTORS))
    print("  URDF 关节角(度):", "  ".join(f"{np.degrees(v):+7.2f}" for v in q))
    print("  末端位置(m):", "  ".join(f"{v:+.4f}" for v in tip))
    print(f"  末端离基座水平距离 = {np.hypot(tip[0], tip[1]) * 100:.1f} cm，高度 = {tip[2] * 100:.1f} cm")


def main() -> int:
    ap = argparse.ArgumentParser(description="SO101 标定自检")
    add_common_args(ap)
    ap.add_argument("--wiggle", action="store_true", help="逐个关节动 +10 度（会真的动机械臂）")
    ap.add_argument("--delta", type=float, default=10.0, help="wiggle 时的角度变化量（度）")
    args = ap.parse_args()

    with session_from_args(args) as s:
        print("=" * 76)
        print("SO101 标定自检")
        print("=" * 76)
        print(describe(s.jmap))
        print()
        print("换算关系：q_urdf[rad] = offset + sign * scale * 动作值")
        print("  offset 取 URDF 关节行程中点（new_calib 约定：标定零点=行程中点）")
        print("  scale  取 行程半宽/100，即动作值 ±100 对应 URDF 的整个行程")
        print()
        report(s, "当前状态：")
        print()

        if not args.wiggle:
            print("提示：")
            print("  1) 用卷尺量一下末端相对基座的 x/y/z，和上面的预测值对比；")
            print("     偏差大说明 scale 或 offset 不对，可以在 config.json 的 ik 段调整")
            print("     （或运行本脚本加 --wiggle 先确认方向）。")
            print("  2) 想让某个关节正负方向对调：把 config.json 里 ik.signs 对应项改成 -1。")
            return 0

        base = s.arm.state().action
        print("--- 逐关节测试（每个关节单独 +%.0f 度，1 秒后回位）---" % args.delta)
        print("请观察：真机的运动方向，和下面打印的'末端/关节变化'是否一致")
        print()
        for name in ARM_JOINTS:
            target = dict(base)
            target[name] = float(np.clip(base[name] + args.delta, -100.0, 100.0))
            print(f"* 测试 {name}: {base[name]:+.1f} -> {target[name]:+.1f}")
            s.arm.move_to(target, duration=0.8, wait=True)
            time.sleep(0.4)
            report(s, "  移动后：")
            s.arm.move_to(base, duration=0.8, wait=True)
            time.sleep(0.2)
            print()

        print("怎么判断：")
        print("  * 如果某个关节完全不动或方向相反 -> 把 config.json 中 ik.signs 对应位改成 -1；")
        print("  * 如果动得明显比 10 度多/少 -> 调整 ik.scales_deg_per_unit 对应项")
        print("    （默认 shoulder_pan=1.1, lift=1.0, elbow=0.9683, wrist_flex=0.95, wrist_roll=1.6）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
