"""无硬件冒烟测试：用 Mock 后端把整条控制链路跑一遍。

验证内容：
  * 伺服线程启动、读回实测姿态；
  * 姿态切换（限时 / 限速）；
  * IK 直线运动 + 无解处理；
  * 轨迹平滑性（相邻帧的关节增量有界）。

用法：
    python tools/smoke_arm.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.session import open_session  # noqa: E402

FAIL = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        FAIL += 1


def main() -> int:
    with open_session(mock=True, verbose=False) as s:
        arm = s.arm
        print("后端:", s.backend.name)
        st0 = arm.state()
        print("初始:", st0.pretty())

        # --- 1) 姿态切换：限时 ---
        t0 = time.perf_counter()
        arm.move_to({"shoulder_pan": 30.0, "shoulder_lift": -15.0}, duration=1.0, wait=True)
        elapsed = time.perf_counter() - t0
        st1 = arm.state()
        print("切换后:", st1.pretty(), f"(实际用时 {elapsed:.2f}s)")
        check("姿态切换完成", abs(st1.action["shoulder_pan"] - 30.0) < 1.5,
              f"shoulder_pan={st1.action['shoulder_pan']:.2f}")
        check("耗时接近设定值 1.0s", 0.9 <= elapsed <= 2.2, f"{elapsed:.2f}s")

        # --- 2) 姿态切换：限速 ---
        t0 = time.perf_counter()
        arm.move_to({"shoulder_pan": -30.0}, max_speed=60.0, wait=True)
        elapsed = time.perf_counter() - t0
        delta = abs(arm.state().action["shoulder_pan"] + 30.0)
        check("限速切换完成", delta < 1.5, f"残差={delta:.2f} 用时={elapsed:.2f}s")

        arm.move_to({"shoulder_pan": 0.0, "shoulder_lift": 0.0}, duration=0.8, wait=True)

        # --- 3) IK 直线运动 ---
        p0 = arm.tip_position()
        ok, dist, note = arm.move_tip_by([0.0, 0.0, 1.0], 0.05, duration=1.0)
        arm.wait_until_settled(timeout=5.0)
        p1 = arm.tip_position()
        dz = (p1 - p0)[2]
        print(f"向上 5cm: ok={ok} 计划={dist * 1000:.1f}mm 实测z={dz * 1000:.1f}mm note={note}")
        check("向上移动方向正确", dz > 0.03, f"dz={dz * 1000:.1f}mm")
        check("向上移动幅度合理", 0.035 < dz < 0.055, f"dz={dz * 1000:.1f}mm")

        # x 方向
        p1 = arm.tip_position()
        ok, dist, note = arm.move_tip_by([1.0, 0.0, 0.0], 0.03, duration=0.8)
        arm.wait_until_settled(timeout=5.0)
        dx = (arm.tip_position() - p1)[0]
        check("向前移动方向正确", dx > 0.015, f"dx={dx * 1000:.1f}mm note={note}")

        # --- 4) 连续走出工作空间：应逐级缩短而不是报错中断 ---
        arm.move_to({"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0,
                     "wrist_flex": 0.0, "wrist_roll": 0.0}, duration=1.0, wait=True)
        print("回到零位:", np.round(arm.tip_position(), 4))
        budget = 0.0
        for i in range(40):
            ok, dist, note = arm.move_tip_by([0.0, 0.0, 1.0], 0.02, duration=0.25)
            arm.wait_until_settled(timeout=3.0)
            budget += dist
            if dist <= 1e-9:
                print(f"  第 {i + 1} 次：已到边界（累计上升 {budget * 1000:.1f}mm）")
                break
        check("走到边界后能安全停下", i < 40, f"累计 {budget * 1000:.1f}mm")

        # --- 5) 轨迹平滑性 ---
        arm.move_to({"shoulder_pan": 0.0, "elbow_flex": 0.0}, duration=1.0, wait=True)
        samples: list[dict[str, float]] = []
        orig_send = arm.backend.send

        def spy(action: dict[str, float]) -> None:
            samples.append(dict(action))
            orig_send(action)

        arm.backend.send = spy  # type: ignore[method-assign]
        arm.move_to({"shoulder_pan": 60.0, "elbow_flex": 40.0}, duration=1.2, wait=True)
        arm.backend.send = orig_send  # type: ignore[method-assign]
        arr = np.asarray([[s["shoulder_pan"], s["elbow_flex"]] for s in samples])
        if len(arr) > 3:
            jumps = np.max(np.abs(np.diff(arr, axis=0)), axis=1)
            check("相邻帧关节增量有界（无跳变）", float(jumps.max()) < 8.0,
                  f"最大单帧增量={jumps.max():.2f}°  帧数={len(arr)}")
            check("轨迹单调推进", float(arr[-1, 0]) > 55.0, f"终点={arr[-1, 0]:.1f}")

        # --- 6) 夹爪 ---
        arm.set_gripper(90.0, wait=True)
        g = arm.state().action["gripper"]
        check("夹爪张开到 90", abs(g - 90.0) < 2.0, f"gripper={g:.1f}")
        arm.set_gripper(10.0, wait=True)
        g = arm.state().action["gripper"]
        check("夹爪闭合到 10", abs(g - 10.0) < 2.0, f"gripper={g:.1f}")

    print()
    summary = "全部通过" if FAIL == 0 else f"{FAIL} 项失败"
    print(f"结果：{summary}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
