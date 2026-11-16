"""关节角单位换算：lerobot 动作值 <-> URDF 关节角（弧度）。

背景
----
* URDF 里的关节角单位是**弧度**，零点由 URDF 自己定义（本文件叫它 URDF 坐标）；
* lerobot 下发给舵机的动作值是**无量纲的百分数**（SO101 各臂关节为 -100 ~ +100，
  夹爪为 0 ~ 100），lerobot 内部把这个值线性映射到标定时记录的编码器行程
  ``[range_min, range_max]``。

SO101 官方标定（``so101_new_calib``）的约定是：**标定零点 = 关节行程中点**。
于是两套坐标之间就是一条仿射关系：

.. code-block:: text

    q_urdf[rad] = offset[i] + sign[i] * scale[i] * action[i]

* ``offset``：URDF 行程中点 (lower+upper)/2，单位弧度 —— 对应动作值 0；
* ``scale`` ：每个动作单位对应多少弧度。默认 ``half_span / 100``，即假设 lerobot
  标定记录的行程端点（动作值 ±100）恰好落在 URDF 给出的机械限位上。
  SO101 实测行程 ±110° / ±100° / ±96.8° / ±95° / ±160° 与 URDF 完全吻合，
  所以这是合理的默认值；
* ``sign`` ：方向，默认全 +1。实测发现某个关节转向相反就把它改成 -1。

拿不准就跑 ``tools/check_calib.py``：打印动作值 + 按 FK 预测的末端位置，
用卷尺量一下就知道 scale / sign 对不对。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from .urdf_model import UrdfModel

ACTION_FULL_SCALE = 100.0  # lerobot 动作值满量程
LE_ROBOT_HARD_LIMIT = ACTION_FULL_SCALE  # lerobot 归一化后的固有范围


@dataclass
class JointMap:
    """URDF 关节角与 lerobot 动作值之间的双向映射（只覆盖臂部 5 个关节）。"""

    motor_names: list[str]  # 与 lerobot 电机名一致（不含 ".pos" 后缀）
    joint_names: list[str]  # URDF 中的关节名
    scales: np.ndarray  # (n,) rad / action-unit，恒为正
    offsets: np.ndarray  # (n,) rad
    signs: np.ndarray  # (n,) +1 / -1
    limits_action: np.ndarray  # (n, 2) URDF 限位换算到动作值空间后的范围

    # ------------------------------------------------------------------ 构造
    @classmethod
    def from_model(
        cls,
        model: UrdfModel,
        *,
        motor_names: Sequence[str] | None = None,
        mode: str = "range",
        offsets_deg: Sequence[float] | None = None,
        signs: Sequence[float] | None = None,
        scales_deg_per_unit: Sequence[float] | None = None,
    ) -> "JointMap":
        """按 URDF 自动生成映射。

        参数
        ----
        motor_names
            与 URDF 关节一一对应的 lerobot 电机名，默认与关节名相同
            （SO101 的 URDF 关节名恰好就是 ``shoulder_pan`` 等电机名）。
        mode
            ``"range"``（默认）：动作值 ±100 对应 URDF 整个行程；
            ``"degree"``：1 个动作单位 = 1 度。
        offsets_deg / signs / scales_deg_per_unit
            手动覆盖项，用于标定自检后微调。优先级最高。
        """
        joints = model.movable_joints
        n = len(joints)
        motors = list(motor_names) if motor_names is not None else [j.name for j in joints]
        if len(motors) != n:
            raise ValueError(f"motor_names 长度 {len(motors)} 与自由度 {n} 不一致")

        half_span_deg = np.asarray(
            [math.degrees(0.5 * (j.upper - j.lower)) for j in joints], dtype=float
        )
        zero_deg = np.asarray([math.degrees(j.zero) for j in joints], dtype=float)

        if scales_deg_per_unit is not None:
            scale_deg = np.asarray(scales_deg_per_unit, dtype=float).reshape(-1)
            if scale_deg.size != n:
                raise ValueError("scales_deg_per_unit 长度与自由度不一致")
        elif mode == "range":
            scale_deg = half_span_deg / ACTION_FULL_SCALE
        elif mode == "degree":
            scale_deg = np.ones(n, dtype=float)
        else:
            raise ValueError(f"未知 mode: {mode!r}（可选 'range' / 'degree'）")

        offset_deg = zero_deg + (
            np.zeros(n) if offsets_deg is None else np.asarray(offsets_deg, dtype=float).reshape(-1)
        )
        sign = np.ones(n) if signs is None else np.asarray(signs, dtype=float).reshape(-1)

        scales_rad = np.deg2rad(scale_deg)
        offsets_rad = np.deg2rad(offset_deg)
        lower = np.asarray([j.lower for j in joints], dtype=float)
        upper = np.asarray([j.upper for j in joints], dtype=float)
        a_lo = (lower - offsets_rad) / (sign * scales_rad)
        a_hi = (upper - offsets_rad) / (sign * scales_rad)
        limits = np.stack([np.minimum(a_lo, a_hi), np.maximum(a_lo, a_hi)], axis=1)

        return cls(
            motor_names=motors,
            joint_names=[j.name for j in joints],
            scales=scales_rad,
            offsets=offsets_rad,
            signs=sign,
            limits_action=limits,
        )

    # ------------------------------------------------------------------ 属性
    @property
    def n(self) -> int:
        return len(self.joint_names)

    def action_limits(self, margin_deg: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        """动作空间的软限位（同时考虑 URDF 行程与 lerobot 的 ±100 固有范围）。"""
        lo = np.maximum(self.limits_action[:, 0], -LE_ROBOT_HARD_LIMIT)
        hi = np.minimum(self.limits_action[:, 1], +LE_ROBOT_HARD_LIMIT)
        return lo + margin_deg, hi - margin_deg

    # ------------------------------------------------------------------ 换算
    def action_to_urdf(self, action: Mapping[str, float] | Sequence[float]) -> np.ndarray:
        """动作值 -> URDF 关节角（弧度）。

        ``action`` 可以是 ``{"shoulder_pan": 12.0, ...}``（键可带或不带 ``.pos``），
        也可以是长度 n 的序列。
        """
        a = self._as_vector(action)
        return self.offsets + self.signs * self.scales * a

    def urdf_to_action(self, q: Sequence[float]) -> dict[str, float]:
        """URDF 关节角（弧度）-> ``{"shoulder_pan": 12.0, ...}``。"""
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.n:
            raise ValueError(f"需要 {self.n} 个关节角，收到 {q.size} 个")
        a = (q - self.offsets) / (self.signs * self.scales)
        return {name: float(v) for name, v in zip(self.motor_names, a)}

    def clip_action(
        self, action: Mapping[str, float], margin_deg: float = 0.5
    ) -> dict[str, float]:
        """把动作值裁剪到限位内，返回新字典。"""
        lo, hi = self.action_limits(margin_deg=margin_deg)
        out: dict[str, float] = {}
        for i, name in enumerate(self.motor_names):
            out[name] = float(np.clip(self._lookup(action, name), lo[i], hi[i]))
        return out

    # ------------------------------------------------------------------ 内部
    def _as_vector(self, action: Mapping[str, float] | Sequence[float]) -> np.ndarray:
        if isinstance(action, Mapping):
            return np.asarray([self._lookup(action, name) for name in self.motor_names], dtype=float)
        a = np.asarray(action, dtype=float).reshape(-1)
        if a.size != self.n:
            raise ValueError(f"需要 {self.n} 个动作值，收到 {a.size} 个")
        return a

    @staticmethod
    def _lookup(action: Mapping[str, float], name: str) -> float:
        if name in action:
            return float(action[name])
        key = f"{name}.pos"
        if key in action:
            return float(action[key])
        raise KeyError(f"动作里找不到关节 {name!r}（也没有 {key!r}）")


def describe(jmap: JointMap) -> str:
    """人可读的映射说明，验收讲解时可直接打印。"""
    lines = [
        f"{'motor':14s} {'joint':14s} {'offset(deg)':>12s} {'scale(deg/unit)':>16s} {'sign':>5s} "
        f"{'action range':>18s}",
        "-" * 86,
    ]
    for i, (m, j) in enumerate(zip(jmap.motor_names, jmap.joint_names)):
        lines.append(
            f"{m:14s} {j:14s} {math.degrees(jmap.offsets[i]):12.3f} "
            f"{math.degrees(jmap.scales[i]):16.4f} {jmap.signs[i]:5.0f} "
            f"{'[%7.1f,%7.1f]' % tuple(jmap.limits_action[i]):>18s}"
        )
    return "\n".join(lines)
