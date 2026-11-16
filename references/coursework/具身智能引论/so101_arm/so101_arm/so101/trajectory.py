"""姿态之间的时间插值：让"从 A 姿态运动到 B 姿态，用时 T"平滑可调。

三个速度曲线：

* ``linear``    ：匀速。简单，但起停瞬间有速度突变，舵机会"点头"。
* ``minjerk``   ：最小 jerk（6t⁵-15t⁴+10t³）。起停加速度也连续，最平滑，
                  课程里"姿态切换 1s 内完成"这种需求用它效果最好（默认）。
* ``trapezoid`` ：梯形速度（加速-匀速-减速），可指定最大速度，适合"限速"场景。

另外提供 :func:`interpolate_steps`：把一段运动离散成 N 步，供伺服循环逐步下发。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterator, Sequence

import numpy as np

Profile = Callable[[float], float]


def linear_profile(t: float) -> float:
    return t


def minjerk_profile(t: float) -> float:
    """最小 jerk 位置曲线：s(t) = 10t³ - 15t⁴ + 6t⁵，s(0)=0, s(1)=1。"""
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def make_trapezoid_profile(max_vel: float, duration: float = 1.0) -> Profile:
    """梯形速度曲线，``max_vel`` 是归一化速度上限（单位：行程/秒，行程=1）。"""
    if max_vel <= 0:
        raise ValueError("max_vel 必须为正")

    if max_vel >= 2.0 / duration:  # 速度上限制约不了，退化成三角/最小 jerk 形状
        accel_time = duration / 2.0
        peak = 2.0 / duration
    else:
        accel_time = 1.0 / max_vel
        peak = max_vel

    def profile(t: float) -> float:
        t = min(max(t, 0.0), 1.0) * duration
        if t < accel_time:  # 加速段
            return 0.5 * (peak / accel_time) * t * t
        if t < duration - accel_time:  # 匀速段
            return 0.5 * peak * accel_time + peak * (t - accel_time)
        td = duration - t  # 减速段（对称）
        return 1.0 - 0.5 * (peak / accel_time) * td * td

    return profile


def get_profile(name: str = "minjerk", *, max_vel: float | None = None, duration: float = 1.0) -> Profile:
    name = name.lower()
    if name == "linear":
        return linear_profile
    if name == "minjerk":
        return minjerk_profile
    if name == "trapezoid":
        return make_trapezoid_profile(max_vel if max_vel else 2.0 / duration, duration)
    raise ValueError(f"未知的速度曲线: {name!r}（可选 linear/minjerk/trapezoid）")


@dataclass
class Motion:
    """一段 q0 -> q1、历时 ``duration`` 秒的运动。"""

    q0: np.ndarray
    q1: np.ndarray
    duration: float
    profile: Profile
    name: str = ""

    @property
    def max_delta(self) -> float:
        return float(np.max(np.abs(self.q1 - self.q0)))

    def sample(self, t: float) -> np.ndarray:
        """取 t 时刻（秒）的关节角。"""
        if self.duration <= 1e-9:
            return self.q1.copy()
        s = self.profile(min(max(t / self.duration, 0.0), 1.0))
        return self.q0 + (self.q1 - self.q0) * s

    def steps(self, fps: float = 50.0) -> Iterator[np.ndarray]:
        """按 fps 逐帧产出关节角，最后一帧必定等于 q1。"""
        n = max(1, int(round(self.duration * fps)))
        for i in range(1, n + 1):
            yield self.sample(self.duration * i / n)


def blend_sequence(
    q_start: Sequence[float],
    waypoints: Sequence[tuple[Sequence[float], float]],
    *,
    profile: str = "minjerk",
    max_vel: float | None = None,
) -> list[Motion]:
    """把 (目标姿态, 用时) 序列串成一段连续运动。

    例如"抬起 -> 1s 内向左转 -> 1s 内向右转"就是
    ``[(q_raise, 1.5), (q_left, 1.0), (q_right, 1.0)]``。
    """
    motions: list[Motion] = []
    q_prev = np.asarray(q_start, dtype=float).reshape(-1)
    for q_next, duration in waypoints:
        q_next = np.asarray(q_next, dtype=float).reshape(-1)
        prof = get_profile(profile, max_vel=max_vel, duration=duration)
        motions.append(Motion(q_prev, q_next, float(duration), prof))
        q_prev = q_next
    return motions


def speed_limited_steps(
    q0: Sequence[float],
    q1: Sequence[float],
    *,
    max_speed: float,
    dt: float = 0.02,
    profile: str = "minjerk",
) -> list[np.ndarray]:
    """按"关节角最大变化速度"(rad/s) 决定耗时，返回逐步目标。

    课程里"设置预设姿态之间的切换时间和速度"中的**速度**就用它：
    时长 = 最大关节行程 / 速度上限。
    """
    q0 = np.asarray(q0, dtype=float).reshape(-1)
    q1 = np.asarray(q1, dtype=float).reshape(-1)
    delta = float(np.max(np.abs(q1 - q0)))
    duration = max(delta / max(max_speed, 1e-6), 0.15)  # 至少 0.15s，避免过冲
    motion = Motion(q0, q1, duration, get_profile(profile, duration=duration))
    out: list[np.ndarray] = []
    n = max(1, int(math.ceil(duration / dt)))
    for i in range(1, n + 1):
        out.append(motion.sample(duration * i / n))
    return out
