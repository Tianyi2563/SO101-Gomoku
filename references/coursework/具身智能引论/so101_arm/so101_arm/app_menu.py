"""交互式验收菜单：一键进入四个演示。

用法：
    python app_menu.py
    python app_menu.py --mock        # 没有硬件时全部用模拟臂

菜单里每个选项都会以子进程方式启动对应的演示程序，退出后回到菜单。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

MENU = [
    ("1", "1.1  预设姿态 / 预设动作（键盘切换）", "app_1_1_presets.py", []),
    ("2", "1.2  逆运动学（键盘控制末端走直线）", "app_1_2_ik.py", []),
    ("3", "2.1  语音控制（faster-whisper + 麦克风）", "app_2_1_voice.py", []),
    ("4", "2.1' 语音控制（不接麦克风，直接打字）", "app_2_1_voice.py", ["--asr", "text"]),
    ("5", "2.2  视觉闭环（摄像头看黑球自动靠近）", "app_2_2_vision.py", []),
    ("6", "2.2' 视觉检测调试（不动机械臂）", "app_2_2_vision.py", ["--probe"]),
    ("7", "自检：运动学 + IK 精度", "tools/kin_sanity.py", []),
    ("8", "自检：无硬件控制链路冒烟测试", "tools/smoke_arm.py", []),
    ("9", "自检：语音指令解析（不需要麦克风）", "tools/test_voice_parse.py", []),
    ("10", "自检：黑球检测（合成图像，不需要摄像头）", "tools/test_vision_detect.py", []),
    ("11", "自检：视觉闭环模拟（不需要摄像头/机械臂）", "tools/test_vision_loop.py", []),
    ("12", "自检：标定换算（接真机时用）", "tools/check_calib.py", []),
    ("13", "列出录音设备", "app_2_1_voice.py", ["--list-devices"]),
    ("14", "列出可用摄像头", "app_2_2_vision.py", ["--list-cameras"]),
]


def main() -> int:
    ap = argparse.ArgumentParser(description="SO101 验收演示菜单")
    ap.add_argument("--mock", action="store_true", help="所有需要机械臂的演示都用模拟臂")
    ap.add_argument("--port", default=None, help="串口，例如 COM5")
    args = ap.parse_args()

    extra: list[str] = []
    if args.mock:
        extra.append("--mock")
    if args.port:
        extra += ["--port", args.port]

    while True:
        print()
        print("=" * 72)
        print("SO-ARM101 课程验收演示菜单" + ("   [模拟模式 --mock]" if args.mock else ""))
        print("=" * 72)
        for key, title, _script, _args in MENU:
            print(f"  {key:>3s}. {title}")
        print("   q. 退出")
        print()
        choice = input("请选择：").strip().lower()
        if choice in ("q", "quit", "exit", ""):
            return 0

        item = next((m for m in MENU if m[0] == choice), None)
        if item is None:
            print("无效选项，请重新输入。")
            continue

        _key, title, script, script_args = item
        cmd = [sys.executable, str(ROOT / script), *script_args]
        # 只有需要机械臂的程序才附加 --mock / --port
        if "list" not in " ".join(script_args) and script != "tools/kin_sanity.py":
            cmd += extra
        print(f"\n$ {' '.join(cmd)}\n")
        try:
            subprocess.run(cmd, cwd=str(ROOT))
        except KeyboardInterrupt:
            print("\n（已中断，回到菜单）")
        input("\n按回车回到菜单 ...")


if __name__ == "__main__":
    raise SystemExit(main())
