"""【基础功能 1.1】预设姿态 / 预设动作的键盘控制演示。

按键
----
    1~9          运动到第 N 个预设姿态（用时由 --duration 或 [ ] 实时调整）
    a~h          播放第 N 个预设动作（"挥手"就是一条姿态序列）
    r            录制模式：按 r 之后再按 1~9，把机械臂**当前姿态**存进该槽位
    [ / ]        减小 / 增大姿态切换时间（0.2s 一档）
    - / =        减小 / 增大姿态切换速度上限（10 度/秒 一档）
    l            列出全部预设
    g            夹爪张开 / 闭合
    h            回到第一个预设（通常是"立正"）
    q 或 esc     退出（机械臂停在原位）

示例
----
    python app_1_1_presets.py                 # 连真机
    python app_1_1_presets.py --mock          # 没硬件时模拟演示
    python app_1_1_presets.py --duration 1.5  # 每次姿态切换固定 1.5 秒

验收话术：按 2 切到"抬起"、按 5 切到"前伸"，再按 a 播放"挥手"——挥手就是
"抬起 -> 左转 -> 右转 -> 抬起 -> 回中位"这条姿态序列，每段的时长定义在 presets.json。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from so101.keys import make_key_reader  # noqa: E402
from so101.presets import PresetStore  # noqa: E402
from so101.session import add_common_args, session_from_args  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="1.1 预设姿态与预设动作键盘演示")
    add_common_args(ap)
    ap.add_argument("--duration", type=float, default=None, help="姿态切换用时（秒）")
    ap.add_argument("--max-speed", type=float, default=None, help="姿态切换速度上限（度/秒）")
    ap.add_argument("--profile", default="minjerk", choices=["linear", "minjerk", "trapezoid"])
    ap.add_argument("--list", action="store_true", help="只列出预设然后退出")
    args = ap.parse_args()

    with session_from_args(args) as s:
        store = PresetStore.load(s.cfg.presets_path, model=s.model, jmap=s.jmap)
        if args.list:
            print(store.describe())
            return 0

        duration = args.duration if args.duration is not None else s.cfg.control.default_duration
        max_speed = args.max_speed
        pose_keys = store.pose_keys()
        action_keys = store.action_keys()
        reader = make_key_reader()
        gripper_open = False

        print("=" * 76)
        print("1.1 键盘控制预设姿态 / 预设动作")
        print("=" * 76)
        print(store.describe())
        print()
        print(f"键盘后端：{reader.describe()}")
        print(f"当前姿态切换用时：{duration:.2f}s；速度上限：{'默认' if max_speed is None else f'{max_speed:.0f} 度/秒'}")
        print("按键：1-9 切姿态 | a-h 播动作 | r 录制 | [ ] 调时间 | - = 调速度 | g 夹爪 | q 退出")
        print()

        recording = False
        try:
            while True:
                keys = reader.poll()
                for key in keys:
                    if recording:
                        if key in pose_keys:
                            name = pose_keys[key]
                            store.record(name, s.arm.state().action)
                            print(f"✔ 已把当前姿态录制到 [{key}] {name}")
                            recording = False
                        elif key in ("esc", "q"):
                            print("已取消录制")
                            recording = False
                        continue

                    if key in ("q", "esc"):
                        print("退出。")
                        return 0
                    elif key in pose_keys:
                        name = pose_keys[key]
                        store.play_pose(
                            s.arm, name, duration=duration, max_speed=max_speed, wait=False
                        )
                    elif key in action_keys and key not in pose_keys:
                        store.play_action(s.arm, action_keys[key], profile=args.profile, wait=False)
                    elif key == "r":
                        recording = True
                        print("录制模式：把机械臂摆到想要的姿态，然后按 1-9 存入对应槽位（esc 取消）")
                    elif key == "[":
                        duration = max(0.2, duration - 0.2)
                        print(f"姿态切换用时 -> {duration:.2f}s")
                    elif key == "]":
                        duration = min(6.0, duration + 0.2)
                        print(f"姿态切换用时 -> {duration:.2f}s")
                    elif key == "-":
                        max_speed = 30.0 if max_speed is None else max(10.0, max_speed - 10.0)
                        print(f"速度上限 -> {max_speed:.0f} 度/秒")
                    elif key == "=":
                        max_speed = 70.0 if max_speed is None else min(200.0, max_speed + 10.0)
                        print(f"速度上限 -> {max_speed:.0f} 度/秒")
                    elif key == "l":
                        print(store.describe())
                    elif key == "h":
                        first = next(iter(store.poses), None)
                        if first:
                            store.play_pose(s.arm, first, duration=duration, wait=False)
                    elif key == "g":
                        gripper_open = not gripper_open
                        s.arm.set_gripper(90.0 if gripper_open else 10.0)
                        print(f"夹爪 -> {'张开' if gripper_open else '闭合'}")
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，停止运动并退出。")
            s.arm.hold()
        finally:
            reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
