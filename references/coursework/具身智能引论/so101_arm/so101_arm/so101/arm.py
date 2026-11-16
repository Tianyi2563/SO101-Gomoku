"""高层控制器：把"我要到某个姿态"翻译成实际的关节指令下发。

内部跑一个 **后台伺服线程**（默认 50 Hz），每个周期做三件事：

1. 从后端读回实测关节角（同时把指令状态与实测对齐，避免手动拖动后"跳"）；
2. 对当前运动做时间插值，得到本周期应该处的关节角（限速、平滑）；
3. 裁到关节限位后下发给舵机。

上层（键盘 / 语音 / 视觉）只需要调用 :meth:`ArmController.move_to`、
:meth:`ArmController.move_to_q` 这样的接口设置目标，不必关心时序。

状态向量统一是 **6 维动作值（度）**：``[肩pan, 肩lift, 肘, 腕flex, 腕roll, 夹爪]``。
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

import numpy as np

from .backend import Backend
from .config import ALL_MOTORS, GRIPPER, AppConfig
from .frames import JointMap
from .trajectory import Motion, get_profile
from .urdf_model import UrdfModel


@dataclass
class ArmState:
    """某一时刻的机械臂状态快照。"""

    action: dict[str, float]  # 实测动作值（6 个电机）
    commanded: dict[str, float]  # 指令动作值（6 个电机）
    q: np.ndarray  # 实测关节角（URDF 弧度，5 维）
    tip: np.ndarray  # 由实测关节角算出的末端位置 (3,) 米
    moving: bool

    def pretty(self) -> str:
        tip = ", ".join(f"{v:+.4f}" for v in self.tip)
        return (
            "  ".join(f"{k}={self.action[k]:+6.1f}" for k in ALL_MOTORS)
            + f"   | 末端=({tip}) m"
            + ("  [运动中]" if self.moving else "")
        )


def _vec(action: Mapping[str, float]) -> np.ndarray:
    return np.asarray([float(action[m]) for m in ALL_MOTORS], dtype=float)


def _dict(vec: Sequence[float]) -> dict[str, float]:
    return {m: float(v) for m, v in zip(ALL_MOTORS, vec)}


class ArmController:
    """带后台伺服线程的机械臂控制器。"""

    def __init__(
        self,
        backend: Backend,
        model: UrdfModel,
        jmap: JointMap,
        cfg: AppConfig,
        *,
        verbose: bool = True,
    ) -> None:
        self.backend = backend
        self.model = model
        self.jmap = jmap
        self.cfg = cfg
        self.verbose = verbose

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None

        self._cmd = np.zeros(6)
        self._measured = np.zeros(6)
        self._motions: deque[Motion] = deque()
        self._current: Motion | None = None
        self._current_t0 = 0.0
        self._ready = threading.Event()
        self._idle = threading.Event()
        self._idle.set()

        # 夹爪的软限位（动作值 0~100）
        self._gripper_lo, self._gripper_hi = 0.0, 100.0

    # ------------------------------------------------------------------ 生命周期
    def start(self) -> "ArmController":
        if self._thread is not None:
            return self
        # 初始目标 = 当前实测姿态，避免上电瞬间机械臂猛冲
        measured = self.backend.read()
        vec = _vec(measured)
        with self._lock:
            self._measured = vec.copy()
            self._cmd = vec.copy()

        self._stop.clear()
        self._thread = threading.Thread(target=self._servo_loop, name="so101-servo", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=2.0)
        if self.verbose:
            print(f"[arm] 伺服线程已启动 @ {self.cfg.control.fps:.0f} Hz，后端：{self.backend.name}")
            print(f"[arm] 初始姿态：{ArmState(self._action_of(vec), self._action_of(vec), self._q_of(vec), self._tip_of(vec), False).pretty()}")
        return self

    def stop(self, *, release_torque: bool = False) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._current = None
        self._motions.clear()
        try:
            self.backend.disconnect()
        except Exception as exc:  # noqa: BLE001
            if self.verbose:
                print(f"[arm] 断开后端时出错：{exc}")

    def __enter__(self) -> "ArmController":
        return self.start()

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    # ------------------------------------------------------------------ 伺服循环
    def _servo_loop(self) -> None:
        period = 1.0 / max(self.cfg.control.fps, 1.0)
        next_t = time.perf_counter()
        first = True
        while not self._stop.is_set():
            try:
                measured = _vec(self.backend.read())
                with self._lock:
                    self._measured = measured
                    if self._current is None and self._motions:
                        self._current = self._motions.popleft()
                        self._current_t0 = time.perf_counter()
                        self._idle.clear()

                    if self._current is None:
                        cmd = self._cmd
                    else:
                        elapsed = time.perf_counter() - self._current_t0
                        if elapsed >= self._current.duration:
                            cmd = self._current.q1.copy()
                            self._current = None
                            self._idle.set()
                        else:
                            cmd = self._current.sample(elapsed)
                    self._cmd = cmd
                    action = _dict(cmd)
                    self._clamp_action(action)

                self.backend.send(action)
                if first:
                    self._ready.set()
                    first = False
            except Exception as exc:  # noqa: BLE001 - 记录后退出，避免刷屏
                self._error = exc
                self._idle.set()
                self._ready.set()
                break

            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:  # 落后了就重新对齐节拍，避免累积漂移
                next_t = time.perf_counter()

    def _clamp_action(self, action: dict[str, float]) -> None:
        """按 URDF 行程 + lerobot 固有范围裁剪，夹爪单独处理。"""
        clipped = self.jmap.clip_action(action, margin_deg=0.5)
        for k, v in clipped.items():
            action[k] = v
        action[GRIPPER] = float(np.clip(action[GRIPPER], self._gripper_lo, self._gripper_hi))

    # ------------------------------------------------------------------ 查询
    def _action_of(self, vec: Sequence[float]) -> dict[str, float]:
        return _dict(vec)

    def _q_of(self, vec: Sequence[float]) -> np.ndarray:
        return self.jmap.action_to_urdf(_dict(vec))

    def _tip_of(self, vec: Sequence[float]) -> np.ndarray:
        return self.model.fk_position(self._q_of(vec))

    def state(self) -> ArmState:
        self._check_error()
        with self._lock:
            m, c = self._measured.copy(), self._cmd.copy()
            moving = self._current is not None or bool(self._motions)
        return ArmState(
            action=self._action_of(m),
            commanded=self._action_of(c),
            q=self._q_of(m),
            tip=self._tip_of(m),
            moving=moving,
        )

    def measured_q(self) -> np.ndarray:
        """实测关节角（URDF 弧度）。"""
        return self.state().q

    def tip_position(self) -> np.ndarray:
        """实测末端位置（米）。"""
        return self.state().tip

    def is_moving(self) -> bool:
        with self._lock:
            return self._current is not None or bool(self._motions)

    def at_target(self) -> bool:
        """实测是否已经跟到指令（在容差内）。"""
        self._check_error()
        with self._lock:
            diff = float(np.max(np.abs(self._measured - self._cmd)))
        return diff <= self.cfg.control.settle_tol_deg

    def wait_until_settled(self, timeout: float | None = None) -> bool:
        """等运动结束且实测到位。返回是否成功到位。"""
        t0 = time.perf_counter()
        while True:
            if not self.is_moving() and self.at_target():
                return True
            if timeout is not None and time.perf_counter() - t0 > timeout:
                return False
            self._check_error()
            time.sleep(0.02)

    def _check_error(self) -> None:
        if self._error is not None:
            err = self._error
            self._error = None
            raise RuntimeError(f"伺服循环异常终止：{err!r}") from err

    # ------------------------------------------------------------------ 目标设置
    def move_to(
        self,
        target: Mapping[str, float] | Sequence[float],
        *,
        duration: float | None = None,
        max_speed: float | None = None,
        profile: str | None = None,
        from_measured: bool | None = None,
        replace: bool = True,
        wait: bool = False,
        timeout: float | None = None,
    ) -> None:
        """运动到目标姿态。

        参数
        ----
        target : 6 个电机的动作值；字典（可缺项，缺的保持当前值）或长度 6 的序列。
        duration : 用时（秒）。给了就按它走完（对应课程里的"设定切换时间"）。
        max_speed : 关节最大速度（度/秒）。没给 duration 时用它算时长（对应"设定速度"）。
        profile : 速度曲线 linear / minjerk / trapezoid。
        from_measured : 起点是否用实测姿态。默认"当前没在运动就用实测"，
            这样手动摆过机械臂之后指令会自动跟上，不会跳。
        replace : True 表示替换正在排队的运动；False 表示排在后面按顺序执行。
        wait : 是否阻塞到到位。
        """
        with self._lock:
            if isinstance(target, Mapping):
                goal = self._cmd.copy()
                for i, m in enumerate(ALL_MOTORS):
                    if m in target:
                        goal[i] = float(target[m])
            else:
                goal = np.asarray(target, dtype=float).reshape(-1).copy()
                if goal.size != 6:
                    raise ValueError(f"目标姿态需要 6 个值，收到 {goal.size} 个")

            self._clamp_action({m: goal[i] for i, m in enumerate(ALL_MOTORS)})
            start = self._measured.copy() if (from_measured or (from_measured is None and self._current is None and not self._motions)) else self._cmd.copy()

            if profile is None:
                profile = self.cfg.control.profile
            dur = self._resolve_duration(start, goal, duration, max_speed)
            motion = Motion(start, goal, dur, get_profile(profile, duration=dur))

            if replace or (self._current is None and not self._motions):
                self._motions.clear()
                self._current = motion
                self._current_t0 = time.perf_counter()
                self._idle.clear()
            else:
                self._motions.append(motion)

        if self.verbose:
            print(f"[arm] -> {self._fmt_dict(_dict(goal))}  用时 {dur:.2f}s（{profile}）")
        if wait:
            self.wait_until_settled(timeout=timeout)

    def move_to_q(
        self,
        q: Sequence[float],
        gripper: float | None = None,
        **kwargs: object,
    ) -> None:
        """运动到给定的 URDF 关节角（弧度），供 IK 使用。"""
        action = self.jmap.urdf_to_action(q)
        if gripper is not None:
            action[GRIPPER] = float(gripper)
        self.move_to(action, **kwargs)  # type: ignore[arg-type]

    def move_to_action(self, action: Mapping[str, float], **kwargs: object) -> None:
        self.move_to(action, **kwargs)  # type: ignore[arg-type]

    def set_gripper(self, value: float, *, duration: float | None = None, wait: bool = False) -> None:
        """开合夹爪。``value`` 单位是动作值 0~100（0=闭合，100=张开）。"""
        value = float(np.clip(value, self._gripper_lo, self._gripper_hi))
        cur = float(self._cmd[ALL_MOTORS.index(GRIPPER)])
        if duration is None:
            speed = max(self.cfg.control.gripper_speed, 1.0)
            duration = max(abs(value - cur) / speed, 0.1)
        self.move_to({GRIPPER: value}, duration=duration, profile="minjerk", wait=wait)

    def play_sequence(
        self,
        waypoints: Iterable[tuple[Mapping[str, float], float]] | Iterable[Motion],
        *,
        profile: str | None = None,
        wait: bool = True,
        timeout: float | None = None,
    ) -> None:
        """依次执行一串 (目标姿态, 用时) —— 预设动作就是靠它实现的。"""
        self._check_error()
        for item in waypoints:
            if isinstance(item, Motion):
                with self._lock:
                    self._motions.append(item)
                continue
            target, duration = item  # type: ignore[misc]
            self.move_to(target, duration=duration, profile=profile, replace=False)  # type: ignore[arg-type]
        if wait:
            self.wait_until_settled(timeout=timeout)

    def hold(self) -> None:
        """停止当前运动，保持原位（不排队、不复位）。"""
        with self._lock:
            self._motions.clear()
            if self._current is not None:
                self._cmd = self._current.sample(time.perf_counter() - self._current_t0)
            self._current = None
            self._idle.set()

    # ------------------------------------------------------------------ 辅助
    def _resolve_duration(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        duration: float | None,
        max_speed: float | None,
    ) -> float:
        if duration is not None:
            return max(float(duration), 1e-3)
        # 夹爪按自己的速度，手臂按 max_speed
        arm_speed = float(max_speed if max_speed is not None else self.cfg.control.max_speed_deg)
        arm_delta = float(np.max(np.abs(goal[:5] - start[:5]))) if goal.size >= 5 else 0.0
        grip_delta = abs(float(goal[5] - start[5]))
        t_arm = arm_delta / max(arm_speed, 1e-6)
        t_grip = grip_delta / max(self.cfg.control.gripper_speed, 1e-6)
        return max(t_arm, t_grip, 0.05)

    @staticmethod
    def _fmt_dict(d: Mapping[str, float]) -> str:
        return "{" + ", ".join(f"{k}:{v:+.1f}" for k, v in d.items()) + "}"

    # ------------------------------------------------------------------ 单步笛卡尔运动
    def move_tip_by(
        self,
        direction: Sequence[float],
        distance_m: float,
        *,
        duration: float | None = None,
        max_speed_mm_s: float | None = None,
        okay_if_partial: bool = True,
    ) -> tuple[bool, float, str]:
        """让末端沿指定方向"走直线" ``distance_m`` 米，返回 (是否走满, 实际距离, 说明)。

        这是 1.2 的核心：用 IK 把笛卡尔位移翻译成关节角。
        够不着的时候 :func:`so101.kinematics.solve_linear_move` 会做回溯，
        沿原方向走到可达边界处停下。
        """
        from .kinematics import solve_linear_move

        state = self.state()
        q_now = state.q
        sol, achieved, note = solve_linear_move(
            self.model,
            q_now,
            direction,
            distance_m,
            tol=self.cfg.ik.tol_mm / 1000.0,
            damping=self.cfg.ik.damping,
            limit_margin=math.radians(self.cfg.ik.limit_margin_deg),
            rest_q=q_now,
        )

        min_useful = min(self.cfg.ik.min_useful_mm / 1000.0, distance_m * 0.3)
        if achieved <= max(min_useful, 1e-9):
            print(
                f"[IK] IK 无解：末端已到达该方向的可达边界（{note}），不再移动。"
                f"可换一个方向，或先回到工作空间中间位置。"
            )
            return False, 0.0, "blocked"

        speed = max_speed_mm_s if max_speed_mm_s else self.cfg.ik.speed_mm_s
        dur = duration if duration is not None else max(achieved / max(speed / 1000.0, 1e-6), 0.05)
        self.move_to_q(sol.q, duration=dur, profile="minjerk")

        if achieved + 1e-6 < distance_m:
            print(
                f"[IK] 目标超出可达范围：请走 {distance_m * 1000:.1f}mm，"
                f"实际沿原方向走出 {achieved * 1000:.1f}mm（{note}）"
            )
        return achieved + 1e-6 >= distance_m, achieved, note
