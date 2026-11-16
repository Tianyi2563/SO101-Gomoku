"""语音指令解析器的离线自测（不需要麦克风、不需要 whisper 模型）。

跑一批典型说法，检查解析出的意图是否正确。

用法：
    python tools/test_voice_parse.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.presets import DEFAULT_ACTIONS, DEFAULT_POSES, PresetStore  # noqa: E402
from so101.voice import cn_number_to_int, parse_command, parse_distance_mm  # noqa: E402

FAIL = 0


def check(text: str, expect_intent: str, expect_target: str | None = None,
          expect_mm: float | None = None, expect_dir: tuple | None = None) -> None:
    global FAIL
    cmd = parse_command(text, STORE)
    ok = cmd.intent == expect_intent
    detail = f"-> {cmd.intent}"
    if ok and expect_target is not None:
        ok = cmd.target == expect_target
        detail += f"/{cmd.target}"
    if ok and expect_mm is not None:
        ok = cmd.distance_mm is not None and abs(cmd.distance_mm - expect_mm) < 1e-6
        detail += f"/{cmd.distance_mm}mm"
    if ok and expect_dir is not None:
        ok = cmd.direction == expect_dir
        detail += f"/{cmd.direction}"
    print(f"[{'PASS' if ok else 'FAIL'}] {text!r:26s} {detail}")
    if not ok:
        FAIL += 1


def main() -> int:
    global FAIL
    print("--- 数字解析 ---")
    for text, want in [("五", 5), ("十", 10), ("十五", 15), ("二十", 20), ("二十三", 23), ("两", 2)]:
        got = cn_number_to_int(text)
        print(f"[{'PASS' if got == want else 'FAIL'}] {text} -> {got}")
        if got != want:
            FAIL += 1
    for text, want in [("5厘米", 50.0), ("两公分", 20.0), ("10mm", 10.0), ("0.1米", 100.0),
                       ("三厘米", 30.0), ("向前移动5厘米", 50.0), ("往上两厘米", 20.0)]:
        got = parse_distance_mm(text)
        print(f"[{'PASS' if got == want else 'FAIL'}] {text} -> {got} mm")
        if got != want:
            FAIL += 1

    print()
    print("--- 指令解析 ---")
    # 预设姿态
    check("立正", "pose", "stand")
    check("回到立正姿态", "pose", "stand")
    check("抬起手臂", "pose", "raise")
    check("举起手来", "pose", "raise")
    check("向左转", "pose", "left")
    check("往右边转一下", "pose", "right")
    check("前伸", "pose", "forward")
    # 预设动作
    check("挥挥手", "action", "wave")
    check("给我打个招呼", "action", "greet")
    check("左右扫一遍", "action", "sweep")
    # 末端移动
    check("向前移动5厘米", "move", None, 50.0, (1.0, 0.0, 0.0))
    check("往后移动3厘米", "move", None, 30.0, (-1.0, 0.0, 0.0))
    check("向上移动两厘米", "move", None, 20.0, (0.0, 0.0, 1.0))
    check("向左移动10毫米", "move", None, 10.0, (0.0, 1.0, 0.0))
    check("往右移动一点", "move", None, 50.0, (0.0, -1.0, 0.0))
    check("把末端往下降5厘米", "move", None, 50.0, (0.0, 0.0, -1.0))
    # 夹爪
    check("张开夹爪", "gripper")
    check("夹住它", "gripper")
    # 其他
    check("停下", "stop")
    check("回到中位", "home")
    check("今天天气不错", "unknown")

    print()
    summary = "全部通过" if FAIL == 0 else f"{FAIL} 项失败"
    print(f"结果：{summary}")
    return 1 if FAIL else 0


# 用一个临时的 store（不读写磁盘，避免污染 presets.json）
class _MemStore(PresetStore):
    def __init__(self) -> None:
        super().__init__(path=Path(":memory:"))
        from so101.presets import ActionSpec, Pose

        self.poses = {
            name: Pose(name, label, {m: 0.0 for m in ("shoulder_pan", "shoulder_lift", "elbow_flex",
                                                      "wrist_flex", "wrist_roll", "gripper")})
            for name, (label, _t, _g) in DEFAULT_POSES.items()
        }
        self.actions = {
            name: ActionSpec(name, label, list(steps))
            for name, (label, steps) in DEFAULT_ACTIONS.items()
        }


STORE = _MemStore()

if __name__ == "__main__":
    raise SystemExit(main())
