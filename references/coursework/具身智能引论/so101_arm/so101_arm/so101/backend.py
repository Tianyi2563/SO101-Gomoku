"""硬件后端：把"读关节角 / 写关节角"这件事抽象出来。

两种实现：

* :class:`LerobotBackend` —— 真机。用 lerobot 的 ``SO101Follower`` 驱动 Feetech 舵机；
* :class:`MockBackend`   —— 无硬件模拟。在内存里维护关节状态，并以有限速度跟随目标，
  用于没插机械臂时开发和演示（``python app_xxx.py --mock``）。

统一约定
--------
* 动作字典的键是**电机名，不带 ``.pos`` 后缀**，例如 ``{"shoulder_pan": 12.0, ...}``；
* 值的单位是 lerobot 的动作值（臂关节 -100~100，夹爪 0~100）；
* lerobot 的 ``get_observation()`` / ``send_action()`` 用带 ``.pos`` 的键，
  转换由 :class:`LerobotBackend` 内部完成。
"""

from __future__ import annotations

import threading
import time
from typing import Any, Mapping, Protocol, runtime_checkable

from .config import ALL_MOTORS, GRIPPER

_DEFAULT_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 40.0,
}


@runtime_checkable
class Backend(Protocol):
    """硬件后端接口。"""

    @property
    def name(self) -> str: ...

    @property
    def is_mock(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read(self) -> dict[str, float]: ...

    def send(self, action: Mapping[str, float]) -> None: ...


# --------------------------------------------------------------------------- #
#  Mock 后端
# --------------------------------------------------------------------------- #
class MockBackend:
    """内存模拟的机械臂。

    行为上尽量像真机：``send`` 只是设置目标，``read`` 时关节以
    ``follow_speed_deg``（度/秒）的限速向目标靠拢。

    这样即使没有硬件，插值、限速、伺服循环这些逻辑也能真实地跑起来
    （末端会真的"走"过去，而不是瞬间到位）。
    """

    def __init__(
        self,
        initial: Mapping[str, float] | None = None,
        follow_speed_deg: float = 240.0,
    ) -> None:
        self._lock = threading.Lock()
        self._state = dict(_DEFAULT_POSE if initial is None else initial)
        for m in ALL_MOTORS:
            self._state.setdefault(m, _DEFAULT_POSE[m])
        self._target = dict(self._state)
        self._follow_speed = float(follow_speed_deg)
        self._last_t = time.perf_counter()
        self._connected = False
        self._send_count = 0

    # ------------------------------------------------------------------ 接口
    @property
    def name(self) -> str:
        return "MockBackend(内存模拟)"

    @property
    def is_mock(self) -> bool:
        return True

    @property
    def send_count(self) -> int:
        return self._send_count

    def connect(self) -> None:
        self._connected = True
        self._last_t = time.perf_counter()

    def disconnect(self) -> None:
        self._connected = False

    def read(self) -> dict[str, float]:
        with self._lock:
            now = time.perf_counter()
            dt = max(now - self._last_t, 0.0)
            self._last_t = now
            for m in ALL_MOTORS:
                cur, tgt = self._state[m], self._target[m]
                max_delta = self._follow_speed * dt
                delta = tgt - cur
                if abs(delta) <= max_delta:
                    self._state[m] = tgt
                else:
                    self._state[m] = cur + max_delta * (1.0 if delta > 0 else -1.0)
            return dict(self._state)

    def send(self, action: Mapping[str, float]) -> None:
        with self._lock:
            for k, v in action.items():
                if k in self._target:
                    self._target[k] = float(v)
            self._send_count += 1


# --------------------------------------------------------------------------- #
#  lerobot 真机后端
# --------------------------------------------------------------------------- #
_IMPORT_CANDIDATES = (
    ("lerobot.robots.so101_follower", "SO101Follower", "SO101FollowerConfig"),
    ("lerobot.common.robots.so101_follower", "SO101Follower", "SO101FollowerConfig"),
    ("lerobot.robots.so_follower", "SO101Follower", "SO101FollowerConfig"),
)


def lerobot_available() -> bool:
    """检测当前 Python 环境里能否 import lerobot 的 SO101 follower。"""
    try:
        _load_lerobot_classes()
        return True
    except Exception:
        return False


def _load_lerobot_classes() -> tuple[Any, Any]:
    errs: list[str] = []
    for module, cls_name, cfg_name in _IMPORT_CANDIDATES:
        try:
            mod = __import__(module, fromlist=[cls_name, cfg_name])
            return getattr(mod, cls_name), getattr(mod, cfg_name)
        except Exception as exc:  # noqa: BLE001
            errs.append(f"{module}: {exc}")
    raise ImportError(
        "无法导入 lerobot 的 SO101Follower。请先安装：pip install lerobot\n"
        + "\n".join(f"  - {e}" for e in errs)
    )


class LerobotBackend:
    """真机后端：封装 lerobot 的 SO101Follower。"""

    def __init__(
        self,
        port: str,
        robot_id: str = "my_so101_follower",
        calibration_dir: str | None = None,
    ) -> None:
        self.port = port
        self.robot_id = robot_id
        self.calibration_dir = calibration_dir
        self._robot: Any = None

    @property
    def name(self) -> str:
        return f"LerobotBackend({self.port}, id={self.robot_id})"

    @property
    def is_mock(self) -> bool:
        return False

    def connect(self) -> None:
        follower_cls, config_cls = _load_lerobot_classes()
        kwargs: dict[str, Any] = {"port": self.port, "id": self.robot_id}
        if self.calibration_dir:
            from pathlib import Path

            kwargs["calibration_dir"] = Path(self.calibration_dir)
        cfg = config_cls(**kwargs)
        self._robot = follower_cls(cfg)
        self._robot.connect()
        if not getattr(self._robot, "is_connected", True):
            raise RuntimeError(f"连接 {self.port} 失败，请检查串口与供电")

    def disconnect(self) -> None:
        if self._robot is not None:
            try:
                self._robot.disconnect()
            finally:
                self._robot = None

    def read(self) -> dict[str, float]:
        if self._robot is None:
            raise RuntimeError("后端尚未 connect()")
        obs = self._robot.get_observation()
        out: dict[str, float] = {}
        for motor in ALL_MOTORS:
            key = f"{motor}.pos"
            if key in obs:
                out[motor] = float(obs[key])
        return out

    def send(self, action: Mapping[str, float]) -> None:
        if self._robot is None:
            raise RuntimeError("后端尚未 connect()")
        self._robot.send_action({f"{k}.pos": float(v) for k, v in action.items()})

    # ------------------------------------------------------------- 额外能力
    def calibration_ranges(self) -> dict[str, tuple[float, float]]:
        """从 lerobot 的标定数据里读出每个电机的动作值范围（一般就是 ±100 / 0~100）。"""
        if self._robot is None:
            return {}
        bus = getattr(self._robot, "bus", None)
        motors = getattr(bus, "motors", {}) if bus is not None else {}
        out: dict[str, tuple[float, float]] = {}
        for name, motor in motors.items():
            cal = getattr(motor, "calibration", None)
            if cal is None:
                continue
            try:
                from lerobot.motors import MotorNormMode  # type: ignore

                norm = motor.norm_mode
                if norm == MotorNormMode.RANGE_0_100:
                    out[name] = (0.0, 100.0)
                else:
                    out[name] = (-100.0, 100.0)
            except Exception:  # noqa: BLE001 - 版本差异，退化为默认
                out[name] = (0.0, 100.0) if name == GRIPPER else (-100.0, 100.0)
        return out


# --------------------------------------------------------------------------- #
#  工厂
# --------------------------------------------------------------------------- #
def open_backend(
    *,
    mock: bool = False,
    port: str = "COM3",
    robot_id: str = "my_so101_follower",
    calibration_dir: str | None = None,
    mock_follow_speed_deg: float = 240.0,
) -> Backend:
    """按配置创建并连接后端。``mock=True`` 时不碰真实串口。"""
    if not mock and not lerobot_available():
        raise RuntimeError(
            "当前 Python 环境没有安装 lerobot，无法连接真机。\n"
            "  方案 A：pip install -r requirements.txt 安装依赖后重试；\n"
            "  方案 B：加 --mock 参数，在没有机械臂的情况下演示 / 调试。"
        )
    backend: Backend
    if mock:
        backend = MockBackend(follow_speed_deg=mock_follow_speed_deg)
    else:
        backend = LerobotBackend(port, robot_id, calibration_dir)
    backend.connect()
    return backend
