"""Text/ASR -> external text LLM tool call -> bounded multi-pose SO-101 action."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

# VS Code 的旧终端可能显示 (.venv)，但 python 实际仍指向系统解释器。
# 直接运行本文件时统一切到项目环境，避免语音包只存在于另一个 Python 中。
if __name__ == "__main__":
    project_python = Path(__file__).resolve().parent.parent / ".venv" / "Scripts" / "python.exe"
    if project_python.is_file() and Path(sys.executable).resolve() != project_python.resolve():
        print(f"自动切换项目 Python：{project_python}", flush=True)
        raise SystemExit(subprocess.call([str(project_python), "-s", "-X", "utf8", "-m", "so101_demo.action_agent", *sys.argv[1:]]))

from .vision_imitation import JOINT_LIMITS, read_so101_pose
from .credentials import get_glm_api_key
from .scene_agent import AGENT_MODEL, Robot, chat

ROOT = Path(__file__).resolve().parent.parent
PROMPT = (Path(__file__).resolve().parent / "text_action_prompt.txt").read_text(encoding="utf-8")
SCHEMAS = [
    {"type": "function", "function": {"name": "get_robot_state", "description": "读取真实或演练关节位置；必须在动作前调用。", "parameters": {"type": "object", "properties": {}, "additionalProperties": False}}},
    {"type": "function", "function": {"name": "execute_action", "description": "自主播放多姿态动作序列；仅支持挥手 wave 和点头 nod。", "parameters": {"type": "object", "properties": {"action": {"type": "string", "enum": ["wave", "nod"]}}, "required": ["action"], "additionalProperties": False}}},
]
# 相对本次实测姿态的小幅动作；各步均由 Robot.move 匀速插值并回读。
SEQUENCES = {
    "wave": ("挥手", "shoulder_pan", (-12.0, 12.0, -12.0, 12.0, 0.0)),
    "nod": ("点头", "wrist_flex", (-10.0, 10.0, -10.0, 10.0, 0.0)),
}
# 文本动作只在 LeRobot 校准的完整归一化区间内运动；视觉模仿仍保留原来的 ±95 预留边界。
ACTION_LIMITS = {name: (-100.0, 100.0) for name in JOINT_LIMITS if name != "gripper"}
ACTION_LIMITS["gripper"] = JOINT_LIMITS["gripper"]


def sequence_targets(pose: dict[str, float], action: str) -> list[float]:
    _, joint, offsets = SEQUENCES[action]
    low, high = ACTION_LIMITS[joint]
    start = pose[joint]
    if not low <= start <= high:
        raise ValueError(f"{joint} 当前 {start:.2f} 超出校准范围 {low:g}~{high:g}")
    moving_offsets = offsets[:-1]
    shift = min(max(low - (start + min(moving_offsets)), 0.0), high - (start + max(moving_offsets)))
    return [start + offset + shift for offset in moving_offsets] + [start]


def action_feasibility(pose: dict[str, float], action: str) -> tuple[bool, str]:
    if action not in SEQUENCES:
        return False, "未知动作"
    _, joint, offsets = SEQUENCES[action]
    if joint not in pose:
        return False, f"缺少 {joint} 关节回读"
    low, high = ACTION_LIMITS[joint]
    try:
        targets = sequence_targets(pose, action)
    except ValueError as exc:
        return False, str(exc)
    return True, f"{joint} 当前 {pose[joint]:.2f}，整段目标 {min(targets):.2f}~{max(targets):.2f}"


def preflight_hardware(port: str, robot_id: str) -> dict[str, float]:
    try:
        pose = read_so101_pose(port, robot_id)
    except Exception as exc:
        detail = str(exc)
        if "Missing motor IDs" in detail and "Full found motor list" in detail:
            raise RuntimeError(
                f"{port} 串口已打开，但没有读到 SO-101 舵机。请检查机械臂电源、控制板到舵机的线和串口对应设备；本次未发送运动指令。"
            ) from exc
        raise RuntimeError(f"无法读取 {port} 的 SO-101 状态：{detail}") from exc
    return pose


class ActionTools:
    def __init__(self, robot: Robot, trace):
        self.robot, self.trace = robot, trace
        self.state_at = None
        self.action_requested = False
        self.executed = False
        self.completed = False
        self.stopped = False

    def event(self, kind: str, **fields):
        self.trace.write(json.dumps({"time": datetime.now().isoformat(), "event": kind, **fields}, ensure_ascii=False) + "\n")
        self.trace.flush()

    def dispatch(self, name: str, args: object):
        try:
            if self.stopped or not isinstance(args, dict):
                raise ValueError("会话已停止或参数格式错误")
            if name == "get_robot_state" and not args:
                pose = self.robot.state()
                self.state_at = time.monotonic()
                result = {"ok": True, "mode": "hardware" if self.robot.hardware else "dry_run", "joints": pose}
            elif name == "execute_action" and set(args) == {"action"}:
                self.action_requested = True
                action = args["action"]
                if action not in SEQUENCES or self.executed:
                    raise ValueError("仅允许一次 wave 或 nod 动作")
                if self.state_at is None or time.monotonic() - self.state_at > 30:
                    raise ValueError("动作前必须在30秒内读取关节状态")
                label, joint, _ = SEQUENCES[action]
                start = self.robot.state()
                feasible, reason = action_feasibility(start, action)
                if not feasible:
                    raise ValueError(reason)
                targets = sequence_targets(start, action)
                self.executed = True
                self.state_at = None
                mode_label = "执行" if self.robot.hardware else "模拟演练"
                print(f"{mode_label}{label}：{len(targets)}个连续姿态；控制关节 {joint}", flush=True)
                actual = start
                for index, joint_target in enumerate(targets, 1):
                    target = start.copy()
                    target[joint] = joint_target
                    actual = self.robot.move(target, limits=ACTION_LIMITS)
                    value_label = "实测回读" if self.robot.hardware else "模拟值"
                    print(f"  姿态 {index}/{len(targets)}，目标 {target[joint]:.2f}，{value_label} {actual[joint]:.2f}", flush=True)
                    self.event("pose_step", action=action, step=index, target=target, actual=actual)
                self.completed = True
                result = {"ok": True, "mode": "hardware" if self.robot.hardware else "dry_run", "action": action, "poses": len(targets), "actual_joints": actual}
            else:
                raise ValueError("未知工具或参数不匹配")
        except Exception as exc:
            self.stopped = True
            try:
                self.robot.hold()
            except Exception:
                pass
            result = {"ok": False, "error": str(exc)[:200], "stopped": True}
        print(f"工具结果 {name}: {json.dumps(result, ensure_ascii=False)}", flush=True)
        self.event("tool_result", name=name, arguments=args, result=result)
        return result


def run_agent(utterance: str, tools: ActionTools, key: str, model: str) -> str:
    messages = [{"role": "system", "content": PROMPT}, {"role": "user", "content": utterance}]
    for turn in range(5):
        response = chat({"model": model, "messages": messages, "tools": SCHEMAS, "tool_choice": "auto", "stream": False}, key)
        calls = response.get("tool_calls") or []
        message = {"role": "assistant", "content": response.get("content")}
        if calls:
            message["tool_calls"] = calls
        tools.event("llm_response", turn=turn, message=message)
        messages.append(message)
        if not calls:
            return response.get("content") or "本轮结束。"
        if len(calls) > 2:
            raise RuntimeError("模型单轮请求的工具过多")
        for call in calls:
            ident = call.get("id")
            fn = call.get("function") or {}
            if not isinstance(ident, str) or not ident:
                raise RuntimeError("模型工具调用缺少 ID")
            try:
                args = json.loads(fn.get("arguments", ""))
            except (TypeError, ValueError):
                args = None
            print(f"模型调用工具：{fn.get('name')} {json.dumps(args, ensure_ascii=False)}", flush=True)
            result = tools.dispatch(fn.get("name"), args)
            messages.append({"role": "tool", "tool_call_id": ident, "content": json.dumps(result, ensure_ascii=False)})
            if not result["ok"]:
                return "动作已停止：" + result["error"]
    raise RuntimeError("达到模型调用轮数上限")


def main():
    parser = argparse.ArgumentParser(description="文本/语音模型控制 SO-101 连贯动作")
    parser.add_argument("--text", help="文字命令；不传则使用麦克风录一句")
    parser.add_argument("--hardware", action="store_true", help="连接并驱动实体机械臂；默认演练")
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--robot-id", default="my_follower")
    parser.add_argument("--model", default=AGENT_MODEL, help="支持工具调用的文本模型")
    parser.add_argument("--show-prompt", action="store_true")
    parser.add_argument("--check", action="store_true", help="只读关节并检查挥手/点头可行性；不调用模型、不运动")
    parser.add_argument("--check-voice", action="store_true", help="加载语音模型并检查麦克风；不录音、不连接机械臂")
    args = parser.parse_args()
    if args.show_prompt:
        print(PROMPT)
        print(json.dumps(SCHEMAS, ensure_ascii=False, indent=2))
        return
    if args.check:
        pose = preflight_hardware(args.port, args.robot_id) if args.hardware else {name: 0.0 for name in JOINT_LIMITS}
        print("当前关节：" + json.dumps(pose, ensure_ascii=False))
        for action, (label, _, _) in SEQUENCES.items():
            feasible, reason = action_feasibility(pose, action)
            print(f"{label}：{'可执行' if feasible else '暂不可执行'}；{reason}")
        return
    if args.check_voice:
        from .voice_input import make_voice_input
        import numpy as np
        import sounddevice as sd

        _, asr = make_voice_input()
        asr.transcribe(np.zeros(32000, dtype=np.float32), 16000)
        microphone = sd.query_devices(kind="input")
        print(f"语音环境检查通过：识别模型与 VAD 转写可用，输入设备 {microphone['name']}；未录音、未驱动机械臂。")
        return
    if args.hardware:
        preflight_hardware(args.port, args.robot_id)
        print(f"真机连接检查通过：{args.port} / {args.robot_id}", flush=True)
    key = get_glm_api_key()
    if not key:
        parser.error("缺少 GLM_API_KEY")
    if args.text is None:
        from .voice_input import make_voice_input
        recorder, asr = make_voice_input()
        audio, rate = recorder.record_sentence()
        utterance = asr.transcribe(audio, rate)
        if not utterance:
            raise RuntimeError("没有识别出语音命令")
    else:
        utterance = args.text
    print("输入命令：" + utterance)
    folder = ROOT / "text_action_runs" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    folder.mkdir(parents=True)
    robot = Robot(args.hardware, args.port, args.robot_id)
    try:
        with (folder / "trace.jsonl").open("x", encoding="utf-8") as trace:
            tools = ActionTools(robot, trace)
            tools.event("run_config", mode="hardware" if args.hardware else "dry_run", model=args.model, input=utterance)
            answer = run_agent(utterance, tools, key, args.model)
            print("模型回答：" + answer)
            if tools.completed:
                status = "实体连续动作已完成" if args.hardware else "模拟演练完成；实体机械臂未运动"
            else:
                status = "动作中断" if tools.executed else ("动作工具被本地检查拒绝" if tools.action_requested else "未执行；模型未调用动作工具")
            print("动作执行：" + status)
    finally:
        robot.close()
    print("调用记录：" + str(folder / "trace.jsonl"))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError, EOFError, KeyboardInterrupt) as exc:
        print(f"已停止：{exc}", file=sys.stderr)
        sys.exit(1)
