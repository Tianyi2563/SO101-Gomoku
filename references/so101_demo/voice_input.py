"""Load the existing SO-101 project's microphone and offline ASR components."""
import os
from pathlib import Path
import sys


def make_voice_input():
    source = Path(os.getenv("SO101_ARM_DIR", str(Path.home() / "Documents/ChatGPT/LEROBOT/so101_arm")))
    if not (source / "so101" / "voice.py").is_file():
        raise RuntimeError("找不到已有 so101_arm 项目；请设置 SO101_ARM_DIR")
    sys.path.insert(0, str(source))
    from so101.config import AppConfig
    from so101.voice import FasterWhisperAsr, MicrophoneRecorder
    cfg = AppConfig.load(source / "config.json")
    recorder = MicrophoneRecorder(cfg.voice)
    model_dir = Path(cfg.voice.model_dir) if cfg.voice.model_dir else None
    if model_dir is not None and not model_dir.is_absolute():
        model_dir = source / model_dir
    asr = FasterWhisperAsr(cfg.voice.model, device=cfg.voice.device,
                          compute_type=cfg.voice.compute_type,
                          language=cfg.voice.language,
                          model_dir=str(model_dir) if model_dir is not None else None)
    asr.warmup()
    return recorder, asr
