"""【进阶 2.1】语音控制：语音识别（ASR）-> 指令理解 -> 机械臂执行。

整条链路
--------
.. code-block:: text

    麦克风 --> 录音 + 端点检测(VAD) --> faster-whisper 转文本 --> 指令解析 --> 机械臂动作
                                          |                        |
                                     终端打印转录文本        关键字规则（本地，快）
                                                              └ 匹配不到时才问 LLM（可选）

三块独立可替换的组件
--------------------
* **ASR 后端**：:class:`FasterWhisperAsr`（本地离线，默认）/ :class:`TextAsr`（手动打字）。
* **录音**：:class:`MicrophoneRecorder`，用 sounddevice + 能量阈值做一句话切分。
* **指令解析**：:func:`parse_command`，纯本地规则（关键字 + 中文数字/单位解析），
  :func:`ask_llm` 作为可选的兜底（需要 OpenAI 兼容 API）。

支持的说法（示例）
------------------
* 预设姿态：``立正`` / ``抬起`` / ``向左转`` / ``向右转`` / ``前伸`` / ``向下探``
* 预设动作：``挥手`` / ``打招呼`` / ``左右扫一遍``
* 末端动作：``向前移动5厘米`` / ``往后3厘米`` / ``向上移动两厘米`` / ``向右10毫米``
* 夹爪：``张开夹爪`` / ``夹住``
* 其他：``停下`` / ``回中位``
"""

from __future__ import annotations

import json
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol

import numpy as np

from .config import AppConfig, VoiceConfig
from .presets import PresetStore

if TYPE_CHECKING:  # pragma: no cover
    from .arm import ArmController


# --------------------------------------------------------------------------- #
#  ASR 后端
# --------------------------------------------------------------------------- #
class AsrBackend(Protocol):
    def transcribe(self, audio: np.ndarray, sample_rate: int) -> str: ...

    def describe(self) -> str: ...


class TextAsr:
    """不接麦克风：直接在终端打字当作"识别结果"（调试 / 无麦克风时用）。"""

    def __init__(self, prompt: str = "请输入语音指令（直接打字，回车执行）：") -> None:
        self.prompt = prompt

    def describe(self) -> str:
        return "TextAsr（手动输入文本，不需要麦克风）"

    def transcribe(self, audio: np.ndarray | None = None, sample_rate: int = 16000) -> str:
        try:
            return input(self.prompt).strip()
        except EOFError:
            return ""


class FasterWhisperAsr:
    """本地离线语音识别（faster-whisper）。首次使用会自动下载模型。"""

    def __init__(
        self,
        model_size: str = "small",
        *,
        device: str = "cpu",
        compute_type: str = "int8",
        language: str = "zh",
        model_dir: str | None = None,
    ) -> None:
        self.model_size = model_size
        self.device = device
        self.compute_type = compute_type
        self.language = language
        self.model_dir = model_dir  # 本地模型目录（离线部署时用）
        self._model: Any = None

    def describe(self) -> str:
        return f"faster-whisper({self.model_size}, {self.device}/{self.compute_type}, lang={self.language})"

    def _load(self) -> Any:
        if self._model is None:
            from faster_whisper import WhisperModel  # 延迟导入：没装也不影响其它功能

            print(f"[asr] 加载 faster-whisper 模型 {self.model_size!r}（首次会下载，请稍等）...")
            t0 = time.perf_counter()
            try:
                self._model = WhisperModel(
                    self.model_size,
                    device=self.device,
                    compute_type=self.compute_type,
                    download_root=self.model_dir or None,
                )
            except Exception as exc:  # noqa: BLE001
                msg = str(exc)
                if "Connect" in msg or "Timeout" in msg or "LocalEntryNotFound" in msg or "10060" in msg:
                    raise RuntimeError(
                        "下载 whisper 模型失败（连不上 huggingface.co）。三种解决办法：\n"
                        "  ① 用国内镜像（推荐，命令行先设环境变量再运行）：\n"
                        "       PowerShell:  $env:HF_ENDPOINT='https://hf-mirror.com'\n"
                        "       CMD/bash  :  set HF_ENDPOINT=https://hf-mirror.com\n"
                        "     然后重跑本程序（模型只要下一次，之后就离线可用）；\n"
                        "  ② 手动下载模型目录，放到本地后用 --model-dir 指向它；\n"
                        "  ③ 先不接麦克风，用打字模式演示「指令 -> 动作」这一段：\n"
                        "       python app_2_1_voice.py --asr text\n"
                        f"原始错误：{exc}"
                    ) from exc
                raise
            print(f"[asr] 模型就绪，用时 {time.perf_counter() - t0:.1f}s")
        return self._model

    def warmup(self) -> None:
        """提前加载模型，避免第一次说话时卡住。"""
        self._load()

    def transcribe(self, audio: np.ndarray, sample_rate: int = 16000) -> str:
        model = self._load()
        audio = np.asarray(audio, dtype=np.float32).reshape(-1)
        if audio.size == 0:
            return ""
        if sample_rate != 16000:  # whisper 只吃 16 kHz
            audio = resample_linear(audio, sample_rate, 16000)
        segments, _info = model.transcribe(
            audio,
            language=self.language or None,
            beam_size=5,
            vad_filter=True,  # 先用 Silero VAD 掐掉静音段
            condition_on_previous_text=False,  # 避免上一句污染下一句
            no_speech_threshold=0.6,
            log_prob_threshold=-1.0,
            compression_ratio_threshold=2.4,
        )
        parts: list[str] = []
        for seg in segments:
            # whisper 在纯噪声上会"编"出字幕组/点赞这类句子，靠 no_speech_prob 挡掉
            if getattr(seg, "no_speech_prob", 0.0) > 0.75:
                continue
            parts.append(seg.text)
        text = "".join(parts).strip()
        if looks_like_hallucination(text):
            return ""
        return text


# whisper 在静音 / 非人声噪声上的高频"幻觉"文本，识别到就当成没听清
_HALLUCINATION_PATTERNS = (
    "字幕", "索兰娅", "点赞", "订阅", "转发", "打赏", "关注我", "感谢观看", "谢谢观看",
    "下集", "明镜", "阿弥", "MING PAO", "请不吝", "由amara", "字幕组",
)


def looks_like_hallucination(text: str) -> bool:
    """判断识别结果是不是 whisper 的幻觉（静音/噪声也会"编"出句子）。"""
    t = (text or "").strip()
    if not t:
        return True
    if all(not ch.isalnum() for ch in t):  # 只有标点
        return True
    return any(p in t for p in _HALLUCINATION_PATTERNS)


def resample_linear(audio: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    """线性插值重采样（够用；不求音频质量，只求 whisper 能听懂）。"""
    if src_rate == dst_rate or audio.size == 0:
        return audio
    n_dst = int(round(audio.size * dst_rate / src_rate))
    src_t = np.arange(audio.size, dtype=np.float64)
    dst_t = np.linspace(0.0, audio.size - 1, n_dst)
    return np.interp(dst_t, src_t, audio).astype(np.float32)


# --------------------------------------------------------------------------- #
#  录音
# --------------------------------------------------------------------------- #
class MicrophoneRecorder:
    """用 sounddevice 录音，靠能量阈值把一句话切出来。"""

    def __init__(self, cfg: VoiceConfig) -> None:
        self.cfg = cfg
        self._sd: Any = None

    def _sounddevice(self) -> Any:
        if self._sd is None:
            try:
                import sounddevice as sd  # type: ignore
            except ImportError as exc:  # pragma: no cover
                raise RuntimeError(
                    "没有安装 sounddevice，无法录音。请先 pip install sounddevice，\n"
                    "或者改用文本输入模式：python app_2_1_voice.py --asr text"
                ) from exc
            self._sd = sd
        return self._sd

    @staticmethod
    def list_devices() -> str:
        try:
            import sounddevice as sd  # type: ignore
        except ImportError:
            return "未安装 sounddevice，无法列出录音设备。"
        lines = ["可用的录音设备："]
        for i, dev in enumerate(sd.query_devices()):
            if dev.get("max_input_channels", 0) > 0:
                lines.append(
                    f"  [{i}] {dev['name']}  (输入通道 {dev['max_input_channels']}, "
                    f"默认采样率 {dev['default_samplerate']:.0f})"
                )
        return "\n".join(lines)

    def _open_stream(self, samplerate: int) -> tuple[Any, int]:
        sd = self._sounddevice()
        try:
            stream = sd.InputStream(
                samplerate=samplerate,
                channels=1,
                dtype="float32",
                device=self.cfg.input_device,
                blocksize=int(samplerate * 0.03),
            )
            return stream, samplerate
        except Exception:  # noqa: BLE001 - 设备可能不支持 16k，退回 48k
            if samplerate != 48000:
                return self._open_stream(48000)
            raise

    def record_sentence(self) -> tuple[np.ndarray, int]:
        """录一句话：从第一次超过能量阈值开始，到静音超过阈值时长结束。"""
        stream, rate = self._open_stream(self.cfg.sample_rate)
        block = int(rate * 0.03)
        silence_blocks = max(int(self.cfg.silence_seconds / 0.03), 2)
        max_blocks = int(self.cfg.max_record_seconds / 0.03)
        preroll_blocks = 8  # 保留说话前的 0.24s，避免吞字

        chunks: list[np.ndarray] = []
        preroll: list[np.ndarray] = []
        started = False
        silence = 0
        n = 0
        try:
            with stream:
                print("[mic] 请说话 ...", flush=True)
                while n < max_blocks:
                    data, _overflow = stream.read(block)
                    mono = np.asarray(data, dtype=np.float32).reshape(-1)
                    rms = float(np.sqrt(np.mean(mono**2))) if mono.size else 0.0
                    if not started:
                        preroll.append(mono)
                        if len(preroll) > preroll_blocks:
                            preroll.pop(0)
                        if rms > self.cfg.energy_threshold:
                            started = True
                            chunks.extend(preroll)
                            print("[mic] 检测到语音，录音中 ...", flush=True)
                        n += 1
                        continue

                    chunks.append(mono)
                    silence = silence + 1 if rms < self.cfg.energy_threshold else 0
                    if silence >= silence_blocks:
                        break
                    n += 1
        finally:
            pass

        if not chunks:
            return np.zeros(0, dtype=np.float32), rate
        audio = np.concatenate(chunks)
        # 掐掉尾部多余的静音
        trim = int(rate * 0.15)
        if audio.size > trim * 2:
            audio = audio[: audio.size - silence * block + trim]
        return audio.astype(np.float32), rate

    def record_fixed(self, seconds: float = 3.0) -> tuple[np.ndarray, int]:
        """固定时长录音（简单可靠，适合演示）。"""
        stream, rate = self._open_stream(self.cfg.sample_rate)
        frames = int(rate * seconds)
        try:
            with stream:
                print(f"[mic] 录音 {seconds:.1f}s ...", flush=True)
                data, _ = stream.read(frames)
        finally:
            pass
        return np.asarray(data, dtype=np.float32).reshape(-1), rate

    def measure_noise(self, seconds: float = 1.0) -> float:
        """测一下环境噪声能量，方便设置阈值。"""
        stream, rate = self._open_stream(self.cfg.sample_rate)
        try:
            with stream:
                data, _ = stream.read(int(rate * seconds))
        finally:
            pass
        return float(np.sqrt(np.mean(np.asarray(data, dtype=np.float32) ** 2)))


# --------------------------------------------------------------------------- #
#  指令解析
# --------------------------------------------------------------------------- #
_CN_DIGITS = {
    "零": 0, "〇": 0, "一": 1, "壹": 1, "二": 2, "两": 2, "贰": 2, "三": 3, "叁": 3,
    "四": 4, "肆": 4, "五": 5, "伍": 5, "六": 6, "陆": 6, "七": 7, "柒": 7,
    "八": 8, "捌": 8, "九": 9, "玖": 9,
}

# 单位 -> 毫米
_UNITS = {
    "毫米": 1.0, "mm": 1.0, "公厘": 1.0,
    "厘米": 10.0, "公分": 10.0, "cm": 10.0,
    "分米": 100.0, "dm": 100.0,
    "米": 1000.0, "m": 1000.0,
}

_DIRECTIONS: dict[str, tuple[tuple[float, float, float], str]] = {
    "前": ((1.0, 0.0, 0.0), "前 (+x)"),
    "往前": ((1.0, 0.0, 0.0), "前 (+x)"),
    "向前": ((1.0, 0.0, 0.0), "前 (+x)"),
    "后": ((-1.0, 0.0, 0.0), "后 (-x)"),
    "往后": ((-1.0, 0.0, 0.0), "后 (-x)"),
    "向后": ((-1.0, 0.0, 0.0), "后 (-x)"),
    "退": ((-1.0, 0.0, 0.0), "后 (-x)"),
    "左": ((0.0, 1.0, 0.0), "左 (+y)"),
    "往左": ((0.0, 1.0, 0.0), "左 (+y)"),
    "向左": ((0.0, 1.0, 0.0), "左 (+y)"),
    "右": ((0.0, -1.0, 0.0), "右 (-y)"),
    "往右": ((0.0, -1.0, 0.0), "右 (-y)"),
    "向右": ((0.0, -1.0, 0.0), "右 (-y)"),
    "上": ((0.0, 0.0, 1.0), "上 (+z)"),
    "往上": ((0.0, 0.0, 1.0), "上 (+z)"),
    "向上": ((0.0, 0.0, 1.0), "上 (+z)"),
    "升高": ((0.0, 0.0, 1.0), "上 (+z)"),
    "抬高": ((0.0, 0.0, 1.0), "上 (+z)"),
    "下": ((0.0, 0.0, -1.0), "下 (-z)"),
    "往下": ((0.0, 0.0, -1.0), "下 (-z)"),
    "向下": ((0.0, 0.0, -1.0), "下 (-z)"),
    "降低": ((0.0, 0.0, -1.0), "下 (-z)"),
    "放低": ((0.0, 0.0, -1.0), "下 (-z)"),
}

_GRIPPER_OPEN = ("张开", "打开", "松开", "放开", "开夹爪", "打开夹爪", "张")
_GRIPPER_CLOSE = ("闭合", "合上", "夹住", "夹紧", "抓住", "抓住它", "关上", "闭")

_STOP_WORDS = ("停下", "停止", "别动", "不要动", "刹车", "停")
_HOME_WORDS = ("回中位", "复位", "归位", "回原点", "回到中位", "回home", "回到起始")

_MOVE_VERBS = ("移动", "移", "走", "挪", "前进", "后退", "退", "升", "降")


@dataclass
class VoiceCommand:
    """解析后的语音指令。"""

    intent: str  # pose / action / move / gripper / stop / home / unknown
    raw_text: str = ""
    target: str | None = None
    direction: tuple[float, float, float] | None = None
    distance_mm: float | None = None
    gripper_value: float | None = None
    source: str = "keyword"  # keyword / llm
    note: str = ""

    def describe(self) -> str:
        if self.intent == "pose":
            return f"切到预设姿态 “{self.target}”"
        if self.intent == "action":
            return f"播放预设动作 “{self.target}”"
        if self.intent == "move":
            d = self.direction or (0, 0, 0)
            return f"末端沿 ({d[0]:+.0f},{d[1]:+.0f},{d[2]:+.0f}) 方向移动 {self.distance_mm:.0f} mm"
        if self.intent == "gripper":
            return "张开夹爪" if (self.gripper_value or 0) > 50 else "闭合夹爪"
        if self.intent == "stop":
            return "停止运动"
        if self.intent == "home":
            return "回到中位"
        return f"无法识别的指令：{self.raw_text!r}"


def cn_number_to_int(text: str) -> int | None:
    """把"五" / "十五" / "二十三" / "两" 这类中文数字转成整数。"""
    text = text.strip()
    if not text:
        return None
    if text.isdigit():
        return int(text)
    if text == "十":
        return 10
    if "十" in text:
        head, _, tail = text.partition("十")
        tens = _CN_DIGITS.get(head, 1 if head == "" else None)  # type: ignore[arg-type]
        if tens is None:
            return None
        ones = _CN_DIGITS.get(tail, 0) if tail else 0
        return tens * 10 + ones
    if len(text) == 1 and text in _CN_DIGITS:
        return _CN_DIGITS[text]
    # "一二三" 这种连写，按十进制拼接
    digits = [_CN_DIGITS[c] for c in text if c in _CN_DIGITS]
    if len(digits) == len(text) and digits:
        return int("".join(str(d) for d in digits))
    return None


_NUM_PATTERN = re.compile(
    r"([0-9]+(?:\.[0-9]+)?|[零〇一壹二两贰三叁四肆五伍六陆七柒八捌九玖十]{1,3})\s*"
    r"(毫米|公厘|厘米|公分|分米|米|mm|cm|dm|m)",
    re.IGNORECASE,
)


def parse_distance_mm(text: str) -> float | None:
    """从文本里解析距离，例如 "5厘米" / "两公分" / "10mm" / "0.1米"。"""
    m = _NUM_PATTERN.search(text)
    if not m:
        return None
    raw_num, unit = m.group(1), m.group(2).lower()
    value: float | None
    if re.fullmatch(r"[0-9]+(?:\.[0-9]+)?", raw_num):
        value = float(raw_num)
    else:
        n = cn_number_to_int(raw_num)
        value = float(n) if n is not None else None
    if value is None:
        return None
    return value * _UNITS.get(unit, 10.0)


def _find_direction(text: str, keys: tuple[str, ...] | None = None) -> tuple[tuple[float, float, float], str] | None:
    """按最长匹配找方向词，避免 "左" 抢在 "向左" 前面。"""
    items = sorted(_DIRECTIONS.items(), key=lambda kv: -len(kv[0]))
    for word, info in items:
        if keys and word not in keys:
            continue
        if word in text:
            return info
    return None


def parse_command(
    text: str,
    store: PresetStore | None = None,
    *,
    default_distance_mm: float = 50.0,
) -> VoiceCommand:
    """把一句中文指令解析成结构化指令（纯规则，本地、快、可解释）。"""
    raw = text or ""
    t = raw.strip().replace(" ", "")
    if not t:
        return VoiceCommand("unknown", raw_text=raw, note="识别结果为空")

    # 1) 停止 / 复位
    if any(w in t for w in _STOP_WORDS):
        return VoiceCommand("stop", raw_text=raw)
    if any(w in t for w in _HOME_WORDS):
        return VoiceCommand("home", raw_text=raw)

    distance = parse_distance_mm(t)
    pose_names = set(store.poses) if store else set()

    # 2) 预设动作（"挥手"这类词最具体，优先匹配）
    for key, spec in (store.actions.items() if store else []):
        aliases = [key, spec.label, "招呼" if key == "greet" else ""]
        if any(a and a in t for a in aliases) or (key == "wave" and "挥手" in t) or (
            key == "sweep" and ("扫" in t)
        ) or (key == "greet" and ("打招呼" in t or "招手" in t)):
            return VoiceCommand("action", raw_text=raw, target=key)

    # 3) 夹爪
    if any(w in t for w in _GRIPPER_OPEN):
        return VoiceCommand("gripper", raw_text=raw, gripper_value=90.0)
    if any(w in t for w in _GRIPPER_CLOSE):
        return VoiceCommand("gripper", raw_text=raw, gripper_value=10.0)

    # 4) 末端移动：出现方向词 + (距离 或 移动动词)
    has_verb = any(v in t for v in _MOVE_VERBS)
    dir_info = _find_direction(t)
    if dir_info is not None and (has_verb or distance is not None):
        direction, label = dir_info
        dist = distance if distance is not None else default_distance_mm
        return VoiceCommand(
            "move", raw_text=raw, direction=direction, distance_mm=dist, note=f"方向 {label}"
        )

    # 5) 预设姿态
    pose_aliases = {
        "stand": ("立正", "中位", "站好", "正位", "初始姿态"),
        "raise": ("抬起", "举手", "举起", "举高", "抬高", "把手举", "高一点"),
        "left": ("向左转", "左转", "朝左", "转到左", "往左转", "向左", "左移"),
        "right": ("向右转", "右转", "朝右", "转到右", "往右转", "向右", "右移"),
        "forward": ("前伸", "伸出去", "伸直", "往前伸", "伸出"),
        "down": ("向下探", "放低", "垂下", "向下"),
    }
    for name in pose_names:
        aliases = list(pose_aliases.get(name, ())) + [name]
        if store:
            aliases.append(store.pose(name).label)
        for a in aliases:
            if a and a in t:
                return VoiceCommand("pose", raw_text=raw, target=name)
    # 组合规则："转/朝" + 方向词 也算切换姿态（"往右边转一下"这种口语说法）
    if any(w in t for w in ("转", "朝", "面向")):
        for name, words in (("left", ("左",)), ("right", ("右",))):
            if name in pose_names and any(w in t for w in words):
                return VoiceCommand("pose", raw_text=raw, target=name)
    # 数字槽位："切到第二个姿态"
    if store and any(w in t for w in ("第", "个姿态", "号姿态")):
        n = None
        m = re.search(r"第([0-9一二两三四五六七八九十]+)", t)
        if m:
            n = int(m.group(1)) if m.group(1).isdigit() else cn_number_to_int(m.group(1))
        if n is not None and 1 <= n <= len(pose_names):
            return VoiceCommand("pose", raw_text=raw, target=list(store.poses)[n - 1])

    return VoiceCommand("unknown", raw_text=raw)


# --------------------------------------------------------------------------- #
#  可选：LLM 兜底（关键字匹配不到时才调用）
# --------------------------------------------------------------------------- #
def ask_llm(text: str, cfg: VoiceConfig, store: PresetStore | None = None) -> VoiceCommand | None:
    """把识别文本交给大模型，让它输出一个 JSON 指令。需要 OPENAI_API_KEY。"""
    import os

    api_key = os.environ.get(cfg.llm_api_key_env, "")
    if not api_key:
        print(f"[llm] 环境变量 {cfg.llm_api_key_env} 未设置，跳过 LLM 解析")
        return None

    poses = list(store.poses) if store else []
    actions = list(store.actions) if store else []
    schema = {
        "intent": "pose | action | move | gripper | stop | home | unknown",
        "target": f"pose/action 的名字，可选值：poses={poses}, actions={actions}",
        "direction": "move 时的方向向量 [x,y,z]，x 前为正，y 左为正，z 上为正",
        "distance_mm": "move 时移动的毫米数",
        "gripper_value": "gripper 时 0(闭合)~100(张开)",
    }
    prompt = (
        "你是机械臂指令解析器。把用户的中文指令转成 JSON，只输出 JSON，不要解释。\n"
        f"可用字段：{json.dumps(schema, ensure_ascii=False)}\n"
        f"用户指令：{text}"
    )
    payload = {
        "model": cfg.llm_model,
        "messages": [{"role": "user", "content": prompt}],
        "temperature": 0.0,
    }
    req = urllib.request.Request(
        f"{cfg.llm_base_url.rstrip('/')}/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {api_key}"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=20) as resp:
            body = json.loads(resp.read().decode("utf-8"))
        content = body["choices"][0]["message"]["content"]
        content = re.sub(r"^```(?:json)?|```$", "", content.strip(), flags=re.MULTILINE).strip()
        data = json.loads(content)
    except (urllib.error.URLError, KeyError, json.JSONDecodeError, TimeoutError) as exc:
        print(f"[llm] 调用失败：{exc}")
        return None

    cmd = VoiceCommand(
        intent=str(data.get("intent", "unknown")),
        raw_text=text,
        target=data.get("target"),
        distance_mm=float(data["distance_mm"]) if data.get("distance_mm") is not None else None,
        gripper_value=float(data["gripper_value"]) if data.get("gripper_value") is not None else None,
        source="llm",
    )
    if data.get("direction"):
        cmd.direction = tuple(float(v) for v in data["direction"])  # type: ignore[assignment]
    return cmd


# --------------------------------------------------------------------------- #
#  执行器
# --------------------------------------------------------------------------- #
class CommandExecutor:
    """把 :class:`VoiceCommand` 落到机械臂上。"""

    def __init__(self, arm: "ArmController", store: PresetStore, cfg: AppConfig) -> None:
        self.arm = arm
        self.store = store
        self.cfg = cfg

    def execute(self, cmd: VoiceCommand) -> bool:
        arm, store = self.arm, self.store
        if cmd.intent == "stop":
            arm.hold()
            print("[exec] 已停止运动")
            return True

        if cmd.intent == "home":
            first = next(iter(store.poses), None)
            if first:
                store.play_pose(arm, first, duration=self.cfg.control.default_duration, wait=False)
                print(f"[exec] 回到 {first}（{store.pose(first).label}）")
                return True
            return False

        if cmd.intent == "pose" and cmd.target:
            store.play_pose(arm, cmd.target, duration=self.cfg.control.default_duration, wait=False)
            return True

        if cmd.intent == "action" and cmd.target:
            store.play_action(arm, cmd.target, wait=False)
            return True

        if cmd.intent == "gripper" and cmd.gripper_value is not None:
            arm.set_gripper(cmd.gripper_value)
            print(f"[exec] 夹爪 -> {cmd.gripper_value:.0f}")
            return True

        if cmd.intent == "move" and cmd.direction is not None:
            dist_mm = float(cmd.distance_mm or self.cfg.ik.step_mm)
            dist_mm = max(min(dist_mm, self.cfg.ik.max_step_mm * 4), 1.0)  # 防止手误说出离谱距离
            arm.move_tip_by(list(cmd.direction), dist_mm / 1000.0)
            print(f"[exec] 末端沿 {cmd.note or cmd.direction} 移动 {dist_mm:.0f} mm")
            return True

        print(f"[exec] 没听懂：{cmd.raw_text!r}（可以说「挥手」「向前移动5厘米」「张开夹爪」…）")
        return False


# --------------------------------------------------------------------------- #
#  主循环
# --------------------------------------------------------------------------- #
@dataclass
class VoiceRunner:
    """语音主循环：录音 -> 识别 -> 解析 -> 执行。"""

    arm: "ArmController"
    store: PresetStore
    cfg: AppConfig
    asr: AsrBackend
    recorder: MicrophoneRecorder | None = None
    history: list[tuple[str, str]] = field(default_factory=list)

    def handle_text(self, text: str, *, allow_llm: bool = True) -> bool:
        """识别文本 -> 执行。返回是否成功执行。"""
        text = (text or "").strip()
        if not text:
            return False
        print(f"\n>>> 转录文本：{text}")
        cmd = parse_command(text, self.store, default_distance_mm=self.cfg.ik.step_mm)
        if cmd.intent == "unknown" and allow_llm and self.cfg.voice.use_llm_fallback:
            print("[voice] 关键字没匹配到，尝试 LLM 解析 ...")
            llm_cmd = ask_llm(text, self.cfg.voice, self.store)
            if llm_cmd is not None:
                cmd = llm_cmd
        print(f">>> 解析结果：[{cmd.source}] {cmd.describe()}")
        ok = CommandExecutor(self.arm, self.store, self.cfg).execute(cmd)
        self.history.append((text, cmd.describe()))
        return ok

    def run(self, *, max_turns: int | None = None, fixed_seconds: float | None = None) -> None:
        """持续监听模式。Ctrl+C 退出。"""
        voice = self.cfg.voice
        print("=" * 76)
        print("2.1 语音控制")
        print("=" * 76)
        print(f"ASR：{self.asr.describe()}")
        print(f"模式：{'持续监听（检测到说话就识别）' if voice.always_on else '按回车录一句'}")
        print("指令示例：挥手 / 抬起 / 向前移动5厘米 / 张开夹爪 / 停下")
        print("Ctrl+C 退出。\n")

        if hasattr(self.asr, "warmup"):
            try:
                self.asr.warmup()  # type: ignore[attr-defined]
            except Exception as exc:  # noqa: BLE001
                print(f"[asr] 预热失败（不影响后续重试）：{exc}")

        turns = 0
        while max_turns is None or turns < max_turns:
            try:
                if isinstance(self.asr, TextAsr):
                    text = self.asr.transcribe()
                elif fixed_seconds:
                    assert self.recorder is not None
                    audio, rate = self.recorder.record_fixed(fixed_seconds)
                    text = self.asr.transcribe(audio, rate)
                else:
                    assert self.recorder is not None
                    audio, rate = self.recorder.record_sentence()
                    if audio.size == 0:
                        continue
                    # 能量太低就别浪费时间去识别了（whisper 对静音会编出句子）
                    if float(np.sqrt(np.mean(audio**2))) < self.cfg.voice.energy_threshold * 0.5:
                        print("[mic] 声音太小，跳过这一句")
                        continue
                    t0 = time.perf_counter()
                    text = self.asr.transcribe(audio, rate)
                    print(f"[asr] 识别用时 {time.perf_counter() - t0:.2f}s")
                if not text:
                    print("[asr] 没听清（可能是因为没说话/噪声），继续听 ...")
                    continue
                self.handle_text(text)
                turns += 1
            except KeyboardInterrupt:
                print("\n退出语音控制。")
                break
        self.arm.hold()

    def run_push_to_talk(self, key_reader: Any, *, trigger: str = "space") -> None:
        """按下空格录一句、松开后识别（演示时比持续监听更可控）。"""
        voice = self.cfg.voice
        print("=" * 76)
        print("2.1 语音控制（按住/按一下 空格 录一句）")
        print("=" * 76)
        print(f"ASR：{self.asr.describe()}   录音：固定 {voice.max_record_seconds:.0f}s 上限，静音自动结束")
        print("按 q 退出。\n")
        try:
            while True:
                keys = key_reader.poll()
                if any(k in ("q", "esc") for k in keys):
                    break
                if trigger in keys:
                    assert self.recorder is not None
                    audio, rate = self.recorder.record_sentence()
                    if audio.size == 0:
                        print("[mic] 没听到声音，再试一次")
                        continue
                    text = self.asr.transcribe(audio, rate)
                    self.handle_text(text)
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.arm.hold()
