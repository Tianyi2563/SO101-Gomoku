"""【进阶 2.1】语音控制演示程序。

用法
----
    python app_2_1_voice.py                       # 默认：faster-whisper + 麦克风持续监听
    python app_2_1_voice.py --asr text            # 没有麦克风/模型时：直接打字当识别结果
    python app_2_1_voice.py --push-to-talk        # 按一下空格录一句（演示时更可控）
    python app_2_1_voice.py --seconds 3           # 每句固定录 3 秒
    python app_2_1_voice.py --say "向前移动5厘米"  # 直接跑一条指令，检查动作链路
    python app_2_1_voice.py --list-devices        # 列出可用麦克风
    python app_2_1_voice.py --mock                # 没有机械臂时全流程模拟

验收演示步骤
------------
1. 运行本程序，对着麦克风说「挥手」-> 终端打印转录文本 + 解析结果，机械臂播放挥手动作；
2. 说「向前移动5厘米」-> 末端真的向前走 5 cm（IK 计算关节角）；
3. 说「张开夹爪」「停下」等，验证指令覆盖预设姿态、预设动作、末端动作三类。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from so101.config import PROJECT_ROOT  # noqa: E402
from so101.keys import make_key_reader  # noqa: E402
from so101.presets import PresetStore  # noqa: E402
from so101.session import add_common_args, session_from_args  # noqa: E402
from so101.voice import (  # noqa: E402
    FasterWhisperAsr,
    MicrophoneRecorder,
    TextAsr,
    VoiceRunner,
)


def main() -> int:
    ap = argparse.ArgumentParser(description="2.1 语音控制演示")
    add_common_args(ap)
    ap.add_argument("--asr", default=None, choices=["faster-whisper", "text"],
                    help="识别后端；text = 手动打字（不需要麦克风）")
    ap.add_argument("--model", default=None, help="whisper 模型大小 tiny/base/small/medium")
    ap.add_argument("--model-dir", default=None, help="本地模型目录（离线部署时用）")
    ap.add_argument("--push-to-talk", action="store_true", help="按空格录一句，而不是持续监听")
    ap.add_argument("--seconds", type=float, default=None, help="固定录音时长（秒）；默认静音自动结束")
    ap.add_argument("--say", action="append", default=None, help="直接执行给定的文本指令（可重复）")
    ap.add_argument("--list-devices", action="store_true", help="列出录音设备后退出")
    ap.add_argument("--keyboard", default="auto", choices=["auto", "msvcrt", "pynput", "none"],
                    help="按键后端（push-to-talk 模式用）")
    args = ap.parse_args()

    if args.list_devices:
        print(MicrophoneRecorder.list_devices())
        return 0

    with session_from_args(args) as s:
        store = PresetStore.load(s.cfg.presets_path, model=s.model, jmap=s.jmap)

        asr_kind = args.asr or s.cfg.voice.asr
        if args.model:
            s.cfg.voice.model = args.model

        if asr_kind == "text":
            asr = TextAsr()
            recorder = None
        else:
            model_dir = args.model_dir or s.cfg.voice.model_dir or None
            if model_dir:
                p = Path(model_dir)
                model_dir = str(p if p.is_absolute() else PROJECT_ROOT / p)
            asr = FasterWhisperAsr(
                s.cfg.voice.model,
                device=s.cfg.voice.device,
                compute_type=s.cfg.voice.compute_type,
                language=s.cfg.voice.language,
                model_dir=model_dir,
            )
            recorder = MicrophoneRecorder(s.cfg.voice)

        runner = VoiceRunner(arm=s.arm, store=store, cfg=s.cfg, asr=asr, recorder=recorder)

        print("=" * 76)
        print("2.1 语音控制")
        print("=" * 76)
        print(f"识别后端：{asr.describe()}")
        print(f"预置姿态：{'、'.join(f'{k}({v.label})' for k, v in list(store.poses.items())[:6])}")
        print(f"预置动作：{'、'.join(f'{k}({v.label})' for k, v in store.actions.items())}")
        print()

        if args.say:
            for text in args.say:
                runner.handle_text(text)
                s.arm.wait_until_settled(timeout=12.0)
            print("\n--say 指令执行完毕。")
            return 0

        if args.push_to_talk:
            if recorder is None:
                print("--push-to-talk 需要麦克风，不能用 --asr text")
                return 1
            reader = make_key_reader(args.keyboard)
            print(f"键盘后端：{reader.describe()}")
            try:
                runner.run_push_to_talk(reader)
            finally:
                reader.close()
        else:
            runner.run(fixed_seconds=args.seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
