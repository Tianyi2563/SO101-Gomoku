"""预设姿态库与预设动作库（对应课程 1.1）。

* **预设姿态**：一整套 6 个电机的动作值，命名为 ``stand`` / ``raise`` 等；
* **预设动作**：姿态序列 + 每段的用时，例如"挥手 = 抬起(1.5s) -> 左转(1s) -> 右转(1s)"。

存在 ``presets.json`` 里，格式：

.. code-block:: json

    {
      "poses": {
        "stand": {"label": "立正", "action": {"shoulder_pan": 0.0, "...": 0.0}},
        "raise": {"label": "抬起", "action": {...}}
      },
      "actions": {
        "wave": {"label": "挥手", "steps": [["raise", 1.5], ["left", 1.0], ["right", 1.0]]}
      }
    }

首次运行时会用 IK 自动生成一套默认姿态（中位 / 抬起 / 左转 / 右转 / 前伸 …），
之后可以用键盘把现场姿态录制进去覆盖它。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np

from .arm import ArmController
from .config import ALL_MOTORS, GRIPPER
from .frames import JointMap
from .kinematics import ik_dls
from .urdf_model import UrdfModel

# 默认姿态：名字 -> (中文标签, 末端目标位置 (x,y,z) 米, 夹爪开合)
# 目标位置由 IK 反解成关节角，所以一定是可达且姿态自然的。
DEFAULT_POSES: dict[str, tuple[str, tuple[float, float, float], float]] = {
    "stand": ("立正（中位，手臂水平前伸）", (0.30, 0.00, 0.26), 40.0),
    "raise": ("抬起（手臂抬高）", (0.24, 0.00, 0.37), 40.0),
    "left": ("向左转", (0.22, 0.19, 0.26), 40.0),
    "right": ("向右转", (0.22, -0.19, 0.26), 40.0),
    "forward": ("前伸", (0.40, 0.00, 0.24), 40.0),
    "down": ("向下探", (0.28, 0.00, 0.09), 40.0),
}

# 默认动作：名字 -> (中文标签, [(姿态名, 用时秒), ...])
DEFAULT_ACTIONS: dict[str, tuple[str, list[tuple[str, float]]]] = {
    "wave": (
        "挥手",
        [("raise", 1.2), ("left", 0.7), ("right", 0.7), ("left", 0.7), ("right", 0.7), ("stand", 1.0)],
    ),
    "greet": (
        "打招呼（抬臂点头再放下）",
        [("raise", 1.0), ("forward", 0.7), ("raise", 0.7), ("forward", 0.7), ("stand", 1.0)],
    ),
    "sweep": (
        "左右扫一遍",
        [("stand", 0.8), ("left", 1.0), ("right", 1.6), ("stand", 1.0)],
    ),
}


@dataclass
class Pose:
    name: str
    label: str
    action: dict[str, float]

    def vector(self) -> list[float]:
        return [float(self.action.get(m, 0.0)) for m in ALL_MOTORS]


@dataclass
class ActionSpec:
    name: str
    label: str
    steps: list[tuple[str, float]] = field(default_factory=list)


class PresetStore:
    """预设姿态 + 预设动作的读写与播放。"""

    def __init__(
        self,
        path: str | Path,
        poses: dict[str, Pose] | None = None,
        actions: dict[str, ActionSpec] | None = None,
    ) -> None:
        self.path = Path(path)
        self.poses: dict[str, Pose] = poses or {}
        self.actions: dict[str, ActionSpec] = actions or {}

    # ------------------------------------------------------------------ 读写
    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        create_default: bool = True,
        model: UrdfModel | None = None,
        jmap: JointMap | None = None,
    ) -> "PresetStore":
        path = Path(path)
        store = cls(path)
        if path.exists():
            store._from_json(json.loads(path.read_text(encoding="utf-8")))
            return store

        if not create_default:
            return store
        if model is None or jmap is None:
            raise ValueError("首次生成默认预设需要提供 model 与 jmap")
        store.poses = _make_default_poses(model, jmap)
        store.actions = {
            name: ActionSpec(name, label, list(steps))
            for name, (label, steps) in DEFAULT_ACTIONS.items()
        }
        store.save()
        print(f"[presets] 已生成默认预设动作库：{path}")
        return store

    def _from_json(self, data: Mapping[str, object]) -> None:
        self.poses = {}
        for name, item in (data.get("poses") or {}).items():  # type: ignore[union-attr]
            item = dict(item)  # type: ignore[arg-type]
            self.poses[name] = Pose(name, str(item.get("label", name)), dict(item.get("action", {})))
        self.actions = {}
        for name, item in (data.get("actions") or {}).items():  # type: ignore[union-attr]
            item = dict(item)  # type: ignore[arg-type]
            steps = [(str(s[0]), float(s[1])) for s in item.get("steps", [])]
            self.actions[name] = ActionSpec(name, str(item.get("label", name)), steps)

    def to_json(self) -> dict[str, object]:
        return {
            "poses": {
                name: {"label": p.label, "action": {k: round(float(v), 3) for k, v in p.action.items()}}
                for name, p in self.poses.items()
            },
            "actions": {
                name: {"label": a.label, "steps": [[s[0], s[1]] for s in a.steps]}
                for name, a in self.actions.items()
            },
        }

    def save(self, path: str | Path | None = None) -> Path:
        path = Path(path) if path else self.path
        path.write_text(json.dumps(self.to_json(), ensure_ascii=False, indent=2), encoding="utf-8")
        return path

    # ------------------------------------------------------------------ 录制
    def record(
        self,
        name: str,
        action: Mapping[str, float],
        *,
        label: str | None = None,
        save: bool = True,
    ) -> Pose:
        """把当前姿态录成一个预设。"""
        full = {m: float(action.get(m, 0.0)) for m in ALL_MOTORS}
        pose = Pose(name, label or (self.poses[name].label if name in self.poses else name), full)
        self.poses[name] = pose
        if save:
            self.save()
        return pose

    def define_action(
        self,
        name: str,
        steps: Iterable[tuple[str, float]],
        *,
        label: str | None = None,
        save: bool = True,
    ) -> ActionSpec:
        spec = ActionSpec(name, label or name, [(str(s), float(d)) for s, d in steps])
        self.actions[name] = spec
        if save:
            self.save()
        return spec

    # ------------------------------------------------------------------ 播放
    def pose(self, name: str) -> Pose:
        if name not in self.poses:
            raise KeyError(f"没有名为 {name!r} 的预设姿态，现有：{list(self.poses)}")
        return self.poses[name]

    def play_pose(
        self,
        arm: ArmController,
        name: str,
        *,
        duration: float | None = None,
        max_speed: float | None = None,
        wait: bool = True,
    ) -> None:
        """运动到某个预设姿态（可以指定用时或速度）。"""
        pose = self.pose(name)
        print(f"[presets] 切到姿态 {name}（{pose.label}）")
        arm.move_to(pose.action, duration=duration, max_speed=max_speed, wait=wait)

    def play_action(
        self,
        arm: ArmController,
        name: str,
        *,
        profile: str | None = None,
        wait: bool = True,
        on_step: object = None,
    ) -> None:
        """播放一个预设动作（姿态序列）。"""
        if name not in self.actions:
            raise KeyError(f"没有名为 {name!r} 的预设动作，现有：{list(self.actions)}")
        spec = self.actions[name]
        print(f"[presets] 播放动作 {name}（{spec.label}）：" + " -> ".join(f"{s}@{d}s" for s, d in spec.steps))
        waypoints: list[tuple[dict[str, float], float]] = []
        for pose_name, dur in spec.steps:
            waypoints.append((self.pose(pose_name).action, dur))
        arm.play_sequence(waypoints, profile=profile, wait=wait)

    # ------------------------------------------------------------------ 展示
    def describe(self) -> str:
        lines = ["预设姿态（按键切换到对应姿态）："]
        for i, (name, p) in enumerate(self.poses.items(), start=1):
            vals = " ".join(f"{k[:6]}={p.action.get(k, 0.0):+6.1f}" for k in ALL_MOTORS)
            lines.append(f"  [{i}] {name:10s} {p.label:22s} {vals}")
        lines.append("预设动作（按对应字母播放）：")
        for key, (name, a) in zip("abcdefgh", self.actions.items()):
            steps = " -> ".join(f"{s}({d}s)" for s, d in a.steps)
            lines.append(f"  [{key}] {name:10s} {a.label:16s} {steps}")
        return "\n".join(lines)

    def pose_keys(self) -> dict[str, str]:
        """数字键 -> 姿态名。"""
        return {str(i): name for i, name in enumerate(self.poses, start=1) if i <= 9}

    def action_keys(self) -> dict[str, str]:
        """字母键 -> 动作名。"""
        return {k: name for k, name in zip("abcdefgh", self.actions)}


def _make_default_poses(model: UrdfModel, jmap: JointMap) -> dict[str, Pose]:
    """用 IK 把默认的末端位置反解成关节角，得到一组"一定能用"的预设姿态。"""
    q0 = model.zero_pose()
    poses: dict[str, Pose] = {}
    for name, (label, target, grip) in DEFAULT_POSES.items():
        sol = ik_dls(model, np.asarray(target, dtype=float), q0, rest_q=q0, tol=1e-4)
        if sol.ok:
            q = sol.q
        else:
            print(f"[presets] 默认姿态 {name} 的 IK 未收敛（{sol.reason}），已退回零位附近")
            q = q0.copy()
        action = jmap.urdf_to_action(q)
        action[GRIPPER] = float(grip)
        poses[name] = Pose(name, label, action)
    return poses
