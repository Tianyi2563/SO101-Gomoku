"""混淆指令的诊断 / 回归测试。

对应小车文档里花了大篇幅强调的那类指令：
  * 否定     ：「不要前进」「别左转了」        -> 不应该执行被否定的动作
  * 反方向    ：「向左转的反方向」「前进的反方向」-> 应该执行相反的动作
  * 复合      ：「别左转了，前进」             -> 只执行后一个实质动作
  * 习惯说法  ：「倒车」「往后一点」           -> 映射到对应动作

用法：
    python tools/test_voice_ambiguous.py          # 只测规则解析（快）
    python tools/test_voice_ambiguous.py --llm    # 顺便测 LLM 兜底（需要 key）
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.presets import DEFAULT_ACTIONS, DEFAULT_POSES, PresetStore  # noqa: E402
from so101.voice import VoiceCommand, parse_command  # noqa: E402

FAIL = 0

# (指令, 可接受的结果集合, 说明)
# 每条结果写成 "intent" 或 "intent/target"；集合里的任意一个都算通过。
CASES: list[tuple[str, set[str], str]] = [
    # ---- 否定：不能执行被否定的动作 ----
    ("不要前进", {"stop", "unknown", "home"}, "否定前进 -> 不该往前"),
    ("别左转了", {"stop", "unknown", "home"}, "否定左转 -> 不该左转"),
    ("别倒车", {"stop", "unknown", "home"}, "否定倒车"),
    ("不要右转", {"stop", "unknown", "home"}, "否定右转"),
    ("别往下", {"stop", "unknown", "home"}, "否定向下"),
    # ---- 反方向：应该执行相反的动作 ----
    ("向左转的反方向", {"pose/right"}, "左转的反方向 = 右转"),
    ("向右转的反方向", {"pose/left"}, "右转的反方向 = 左转"),
    ("左转的反方向", {"pose/right"}, "同上（省略向）"),
    ("前进的反方向", {"pose/forward", "pose/stand"}, "前进的反方向 = 后退/收回"),
    ("向前移动的反方向", {"move/-x"}, "向前移动的反方向 = 向后移动"),
    ("向上移动的反方向", {"move/-z"}, "向上移动的反方向 = 向下移动"),
    # ---- 复合："别 X，Y" 只执行 Y ----
    ("别左转了，前进", {"pose/forward", "move/+x"}, "只执行后半句"),
    ("别停下来，继续前进", {"pose/forward", "move/+x"}, "只执行后半句"),
    ("不要倒车，向右转", {"pose/right", "move/-y"}, "只执行后半句"),
    # ---- 习惯说法 ----
    ("倒车", {"pose/stand", "move/-x"}, "倒车 = 往后"),
    ("刹车", {"stop"}, "停车 = 停止"),
    ("退后一点", {"pose/stand", "move/-x"}, "退后 = 往后"),
]


class _MemStore(PresetStore):
    """内存态的预设库，避免测试写盘。"""

    def __init__(self) -> None:
        super().__init__(path=Path(":memory:"))
        from so101.presets import ActionSpec, Pose

        motors = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
        self.poses = {
            name: Pose(name, label, {m: 0.0 for m in motors})
            for name, (label, _t, _g) in DEFAULT_POSES.items()
        }
        self.actions = {
            name: ActionSpec(name, label, list(steps))
            for name, (label, steps) in DEFAULT_ACTIONS.items()
        }


STORE = _MemStore()


def result_key(cmd: VoiceCommand) -> str:
    if cmd.intent == "move" and cmd.direction:
        axis = ["x", "y", "z"][int(max(range(3), key=lambda i: abs(cmd.direction[i])))]
        sign = "+" if cmd.direction[[0, 1, 2][int(axis == "y")]] >= 0 else "-"
        return f"move/{sign}{axis}"
    if cmd.target:
        return f"{cmd.intent}/{cmd.target}"
    return cmd.intent


def main() -> int:
    global FAIL
    ap = argparse.ArgumentParser(description="混淆指令测试")
    ap.add_argument("--llm", action="store_true", help="规则失败时再用 LLM 兜底试一次")
    args = ap.parse_args()

    print("=" * 78)
    print("混淆指令解析测试（对照小车文档强调的否定 / 反方向 / 复合说法）")
    print("=" * 78)
    print(f"{'指令':<20s} {'解析结果':<16s} {'可接受':<26s} 判定  说明")
    print("-" * 78)

    llm_ok = 0
    for text, acceptable, why in CASES:
        cmd = parse_command(text, STORE)
        key = result_key(cmd)
        ok = key in acceptable
        line = f"{text:<20s} {key:<16s} {','.join(sorted(acceptable)):<26s} {'PASS' if ok else 'FAIL'}  {why}"
        if not ok and args.llm:
            from so101.config import AppConfig
            from so101.voice import ask_llm

            llm_cmd = ask_llm(text, AppConfig.load().voice, STORE)
            if llm_cmd is not None:
                llm_key = result_key(llm_cmd)
                line += f"   [LLM -> {llm_key}]"
                if llm_key in acceptable:
                    llm_ok += 1
        print(line)
        if not ok:
            FAIL += 1

    total = len(CASES)
    passed = total - FAIL
    print("-" * 78)
    print(f"规则解析：{passed}/{total} 通过" + (f"；LLM 兜底额外救回 {llm_ok} 条" if args.llm else ""))
    if FAIL:
        print("失败的都是「否定 / 反方向 / 复合」这类需要语义理解的说法 ——")
        print("规则匹配只能靠关键词，这正是小车文档目标 3（训练分类模型）和目标 4（LLM）要解决的问题。")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
