"""一次性把配置、URDF、单位映射、后端、控制器串起来，供各个演示程序复用。

用法：

.. code-block:: python

    from so101.session import open_session

    with open_session() as s:          # 真机；加 mock=True 则无硬件模拟
        s.arm.move_to({"shoulder_pan": 30}, duration=1.0, wait=True)
        print(s.arm.tip_position())
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .arm import ArmController
from .backend import Backend, open_backend
from .config import AppConfig
from .frames import JointMap, describe
from .urdf_model import UrdfModel, default_model


@dataclass
class Session:
    """一套完整的运行环境。"""

    cfg: AppConfig
    backend: Backend
    model: UrdfModel
    jmap: JointMap
    arm: ArmController

    def close(self) -> None:
        self.arm.stop()

    def __enter__(self) -> "Session":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


def build_joint_map(model: UrdfModel, cfg: AppConfig) -> JointMap:
    """按配置里的映射参数构造单位换算表。"""
    ik = cfg.ik
    return JointMap.from_model(
        model,
        mode=ik.mode,
        signs=ik.signs,
        offsets_deg=ik.offsets_deg,
        scales_deg_per_unit=ik.scales_deg_per_unit,
    )


def open_session(
    cfg: AppConfig | None = None,
    *,
    mock: bool | None = None,
    verbose: bool = True,
) -> Session:
    """打开一次会话（加载配置 -> 连后端 -> 启动伺服线程）。

    ``mock`` 为 None 时取配置里的 ``robot.mock``；命令行可以用 ``--mock`` 覆盖。
    """
    cfg = cfg or AppConfig.load()
    use_mock = cfg.robot.mock if mock is None else bool(mock)

    model = default_model()
    jmap = build_joint_map(model, cfg)
    backend = open_backend(
        mock=use_mock,
        port=cfg.robot.port,
        robot_id=cfg.robot.id,
        calibration_dir=cfg.robot.calibration_dir,
        mock_follow_speed_deg=cfg.robot.mock_follow_speed_deg,
    )
    arm = ArmController(backend, model, jmap, cfg, verbose=verbose)
    arm.start()

    if verbose:
        print("[session] 关节↔动作值映射（可对照 tools/check_calib.py 校正）：")
        print(describe(jmap))
        print()
    return Session(cfg=cfg, backend=backend, model=model, jmap=jmap, arm=arm)


def add_common_args(parser: Any) -> Any:
    """给 argparse 加上各演示程序共用的参数。"""
    parser.add_argument("--mock", action="store_true", help="不连接真机，用内存模拟演示")
    parser.add_argument("--config", type=Path, default=None, help="配置文件路径（默认 config.json）")
    parser.add_argument("--port", type=str, default=None, help="串口，例如 COM5（覆盖配置文件）")
    parser.add_argument("--quiet", action="store_true", help="少打印")
    return parser


def session_from_args(args: Any) -> Session:
    """按命令行参数打开会话。"""
    cfg = AppConfig.load(args.config)
    if getattr(args, "port", None):
        cfg.robot.port = args.port
    mock = True if getattr(args, "mock", False) else None
    return open_session(cfg, mock=mock, verbose=not getattr(args, "quiet", False))
