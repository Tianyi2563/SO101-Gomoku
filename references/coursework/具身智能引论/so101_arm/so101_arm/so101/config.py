"""配置加载。

用一个 JSON 文件（``config.json``）保存全部可调参数，不引入额外依赖。
所有字段都有默认值，缺项自动补齐。
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
ALL_MOTORS = ARM_JOINTS + [GRIPPER]


def _fill(cls: type, data: dict[str, Any] | None) -> Any:
    """按 dataclass 字段默认值补齐缺失项（未知字段忽略并提示）。"""
    data = dict(data or {})
    known = {f.name for f in fields(cls)}
    unknown = set(data) - known
    if unknown:
        print(f"[config] {cls.__name__} 忽略未知字段 {sorted(unknown)}")
    return cls(**{k: v for k, v in data.items() if k in known})


@dataclass
class RobotConfig:
    port: str = "COM3"  # Windows 上通常是 COM3/COM4/COM5，macOS/Linux 上是 /dev/ttyACM0
    id: str = "my_so101_follower"
    calibration_dir: str | None = None  # 留空则用 lerobot 默认标定目录
    mock: bool = False  # true = 不连接真机，只在内存里模拟
    mock_follow_speed_deg: float = 240.0  # 模拟舵机的跟随速度上限


@dataclass
class ControlConfig:
    fps: float = 50.0  # 伺服循环频率
    default_duration: float = 1.2  # 姿态切换默认用时（秒）
    max_speed_deg: float = 60.0  # 姿态切换默认速度上限（度/秒）
    profile: str = "minjerk"  # linear / minjerk / trapezoid
    gripper_speed: float = 250.0  # 夹爪开合速度（每秒百分比）
    settle_tol_deg: float = 1.5  # 判定"到位"的容差


@dataclass
class IkConfig:
    """IK 与关节角单位映射。详见 so101/frames.py 的说明。"""

    mode: str = "range"  # range: 动作值±100 对应 URDF 整个行程；degree: 1 单位 = 1 度
    signs: list[float] = field(default_factory=lambda: [1.0] * 5)
    offsets_deg: list[float] = field(default_factory=lambda: [0.0] * 5)
    scales_deg_per_unit: list[float] | None = None  # 指定后覆盖 mode
    step_mm: float = 10.0  # 键盘单次按键移动的距离
    max_step_mm: float = 50.0  # 单条指令允许的最大距离
    min_useful_mm: float = 1.0  # 回溯后可行距离小于它，就认为"到边界了"（不再微动）
    speed_mm_s: float = 40.0  # 直线运动时末端速度
    tol_mm: float = 0.5  # IK 位置残差阈值
    damping: float = 0.05  # 自适应阻尼初值
    limit_margin_deg: float = 0.5  # 求解时离关节限位的余量


@dataclass
class VoiceConfig:
    asr: str = "faster-whisper"  # faster-whisper / text(手动输入)
    model: str = "small"  # tiny/base/small/medium，越大越准越慢（中文建议 small 以上）
    model_dir: str = "models"  # 模型存放目录（相对项目根）；留空则用 HuggingFace 默认缓存
    language: str = "zh"
    device: str = "cpu"  # cpu / cuda
    compute_type: str = "int8"  # cpu 上 int8 最快
    sample_rate: int = 16000
    input_device: int | None = None  # 麦克风设备号，null = 系统默认
    silence_seconds: float = 1.0  # 静音多久算一句话结束
    energy_threshold: float = 0.012  # 语音活动检测的能量阈值（0~1）
    max_record_seconds: float = 8.0
    always_on: bool = True  # 持续监听（false 则按回车录一句）
    use_llm_fallback: bool = False  # 关键字匹配不到时，是否调用大模型解析
    llm_base_url: str = "https://api.openai.com/v1"
    llm_model: str = "gpt-4o-mini"
    llm_api_key_env: str = "OPENAI_API_KEY"


@dataclass
class VisionConfig:
    camera_index: int = 0  # 摄像头编号
    width: int = 640
    height: int = 480
    fps: float = 30.0
    backend: str = "dshow"  # dshow / msmf / any（Windows 上 dshow 打开更快）
    black_threshold: int = 70  # 灰度低于该值算"黑"
    min_radius_px: float = 6.0
    max_radius_px: float = 200.0
    circularity: float = 0.62  # 4*pi*A/P^2，越接近 1 越圆
    min_fill_ratio: float = 0.72  # 面积 / 最小外接圆面积：圆≈1.0，正方形≈0.64
    # ---- 闭环控制 ----
    gain_xy: float = 0.00035  # 像素误差 -> 末端横向位移(米/像素)
    gain_z: float = 0.00035  # 像素误差 -> 末端纵向位移(米/像素)
    gain_depth: float = 0.0012  # 半径比例误差 -> 前后位移(米)
    target_radius_ratio: float = 1.0  # 期望的球半径 / 初始半径
    max_step_mm: float = 28.0  # 单周期最大位移，避免大跳
    center_tol_px: float = 28.0  # 认为"对准了"的像素误差
    max_iterations: int = 90
    settle_frames: int = 5  # 连续几帧满足容差就算收敛
    # 相机坐标系 -> 末端坐标系 的方向映射。
    # 相机装在末端朝前看时：图像右 = 末端右侧 = -y，图像下 = -z，球变大 = 球在正前方 = +x。
    # 如果实际装配方向不同（或跑起来发现越走越远），程序会自动翻转符号，
    # 也可以在这里手动改；用 python app_2_2_vision.py --probe 可以观察每个方向的图像变化。
    image_right_axis: list[float] = field(default_factory=lambda: [0.0, -1.0, 0.0])
    image_down_axis: list[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])
    toward_ball_axis: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    show_window: bool = True


@dataclass
class AppConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    ik: IkConfig = field(default_factory=IkConfig)
    voice: VoiceConfig = field(default_factory=VoiceConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    presets_file: str = "presets.json"  # 相对项目根目录

    path: Path = field(default=DEFAULT_CONFIG_PATH, repr=False, compare=False)

    @classmethod
    def load(cls, path: str | Path | None = None) -> "AppConfig":
        path = Path(path) if path else DEFAULT_CONFIG_PATH
        if not path.exists():
            cfg = cls()
            cfg.path = path
            cfg.save(path)
            return cfg
        data = json.loads(path.read_text(encoding="utf-8"))
        cfg = cls(
            robot=_fill(RobotConfig, data.get("robot")),
            control=_fill(ControlConfig, data.get("control")),
            ik=_fill(IkConfig, data.get("ik")),
            voice=_fill(VoiceConfig, data.get("voice")),
            vision=_fill(VisionConfig, data.get("vision")),
            presets_file=data.get("presets_file", "presets.json"),
        )
        cfg.path = path
        return cfg

    def save(self, path: str | Path | None = None) -> Path:
        path = Path(path) if path else self.path
        data = {
            "robot": asdict(self.robot),
            "control": asdict(self.control),
            "ik": asdict(self.ik),
            "voice": asdict(self.voice),
            "vision": asdict(self.vision),
            "presets_file": self.presets_file,
        }
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        return path

    @property
    def presets_path(self) -> Path:
        p = Path(self.presets_file)
        return p if p.is_absolute() else PROJECT_ROOT / p
