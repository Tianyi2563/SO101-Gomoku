"""Cartesian straight-line keyboard control for an SO-101 follower arm."""

from __future__ import annotations

import argparse
import importlib.util
import math
import msvcrt
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(errors="replace")

SCRIPT_DIR = Path(__file__).resolve().parent


def find_project_root() -> Path:
    """Find the LeRobot root even when this script lives in a subfolder."""
    for directory in (SCRIPT_DIR, *SCRIPT_DIR.parents):
        if (directory / "pyproject.toml").is_file():
            return directory
    return SCRIPT_DIR


PROJECT_ROOT = find_project_root()
VENV_PYTHON = PROJECT_ROOT / ".venv" / "Scripts" / "python.exe"
if importlib.util.find_spec("lerobot") is None and VENV_PYTHON.is_file():
    os.environ.setdefault("PYTHONUTF8", "1")
    completed = subprocess.run([str(VENV_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])
    raise SystemExit(completed.returncode)

from lerobot.model.urdf_kinematics import IKResult, URDFKinematics
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

# ============================================================================
# 验收参数区：在 VS Code 中只需要修改这里，然后直接运行本脚本
# ============================================================================
# "distance"：按下方设定的方向和距离移动一次后退出
# "keyboard"：进入 W/S/A/D/R/F 键盘控制模式
CONTROL_MODE = "distance"

# 可选方向："front" 前、"back" 后、"left" 左、"right" 右、"up" 上、"down" 下
MOVE_DIRECTION = "up"
MOVE_DISTANCE_CM = 5.0

# 串口和标定文件对应的机械臂 ID
ROBOT_PORT = "COM4"
ROBOT_ID = "my_follower"

# 运动速度和直线轨迹点间距，一般无需修改
MOVE_SPEED_CM_S = 2.5
WAYPOINT_SPACING_MM = 2.0
# 0° 表示完整使用实机标定范围；机械臂回到中间姿态后可按需改为 1°~2°
JOINT_MARGIN_DEG = 0.0
# 编码器在标定边界附近可能有小幅读数抖动；仅用于启动检查。
STARTUP_LIMIT_TOLERANCE_DEG = 1.0
# ============================================================================

JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
URDF_PATH = PROJECT_ROOT / "assets" / "so101_new_calib_kinematics.urdfmodel"
DIRECTIONS = {
    "front": np.array([1.0, 0.0, 0.0]),
    "back": np.array([-1.0, 0.0, 0.0]),
    "left": np.array([0.0, 1.0, 0.0]),
    "right": np.array([0.0, -1.0, 0.0]),
    "up": np.array([0.0, 0.0, 1.0]),
    "down": np.array([0.0, 0.0, -1.0]),
}
KEY_DIRECTIONS = {"w": "front", "s": "back", "a": "left", "d": "right", "r": "up", "f": "down"}


class EmergencyStop(Exception):
    pass


def joint_limit_overrun_deg(
    q_deg: np.ndarray, lower_deg: np.ndarray, upper_deg: np.ndarray
) -> np.ndarray:
    """Return each joint's distance outside the requested interval, or zero."""
    return np.maximum(np.maximum(lower_deg - q_deg, q_deg - upper_deg), 0.0)


@dataclass(frozen=True)
class LinearPlan:
    joint_waypoints: tuple[np.ndarray, ...]
    requested_m: float
    achieved_m: float
    reached_target: bool
    failure: IKResult | None = None


def calibrated_joint_limits(robot: SO101Follower) -> tuple[np.ndarray, np.ndarray]:
    """Convert each recorded raw motor range to LeRobot's degree coordinates."""
    resolution = robot.bus.model_resolution_table[robot.config.motor_model] - 1
    lower, upper = [], []
    for name in JOINT_NAMES:
        calibration = robot.calibration[name]
        midpoint = (calibration.range_min + calibration.range_max) / 2
        lower.append((calibration.range_min - midpoint) * 360 / resolution)
        upper.append((calibration.range_max - midpoint) * 360 / resolution)
    return np.array(lower), np.array(upper)


def effective_joint_limits(
    kinematics: URDFKinematics, calibration_limits: tuple[np.ndarray, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    """Use measured physical limits; the URDF supplies geometry, not calibration."""
    del kinematics
    calibration_lower, calibration_upper = calibration_limits
    return calibration_lower.copy(), calibration_upper.copy()


def solve_linear_plan(
    kinematics: URDFKinematics,
    start_q_deg: np.ndarray,
    delta_m: np.ndarray,
    lower_deg: np.ndarray,
    upper_deg: np.ndarray,
    *,
    waypoint_spacing_m: float = 0.002,
    orientation_weight: float = 0.05,
) -> LinearPlan:
    """Plan a Cartesian line, truncating it at the last IK-solvable point."""
    distance = float(np.linalg.norm(delta_m))
    if distance == 0:
        return LinearPlan((), 0, 0, True)
    if waypoint_spacing_m <= 0:
        raise ValueError("waypoint_spacing_m must be positive")

    start_pose = kinematics.forward(start_q_deg)
    waypoint_count = max(1, math.ceil(distance / waypoint_spacing_m))
    solutions: list[np.ndarray] = []
    seed = start_q_deg.copy()
    last_fraction = 0.0
    failure: IKResult | None = None

    def solve_at(fraction: float, solve_seed: np.ndarray) -> IKResult:
        target = start_pose.copy()
        target[:3, 3] += delta_m * fraction
        return kinematics.inverse(
            solve_seed,
            target,
            lower_deg=lower_deg,
            upper_deg=upper_deg,
            orientation_weight=orientation_weight,
        )

    for index in range(1, waypoint_count + 1):
        fraction = index / waypoint_count
        result = solve_at(fraction, seed)
        if result.success:
            solutions.append(result.q_deg.copy())
            seed = result.q_deg
            last_fraction = fraction
            continue

        failure = result
        # Preserve the requested direction and find the farthest reachable point
        # between the last good waypoint and the failed one.
        low, high = last_fraction, fraction
        best_q: np.ndarray | None = None
        best_fraction = last_fraction
        for _ in range(10):
            middle = (low + high) / 2
            boundary_result = solve_at(middle, seed)
            if boundary_result.success:
                low = middle
                best_fraction = middle
                best_q = boundary_result.q_deg.copy()
            else:
                high = middle
        if best_q is not None and (best_fraction - last_fraction) * distance >= 0.00025:
            solutions.append(best_q)
            last_fraction = best_fraction
        break

    final_q = solutions[-1] if solutions else start_q_deg
    actual_delta = kinematics.forward(final_q)[:3, 3] - start_pose[:3, 3]
    actual_progress = float(np.clip(np.dot(actual_delta, delta_m / distance), 0, distance))
    return LinearPlan(
        tuple(solutions),
        requested_m=distance,
        achieved_m=actual_progress,
        reached_target=last_fraction >= 1 - 1e-9,
        failure=failure,
    )


def current_state(robot: SO101Follower) -> tuple[np.ndarray, float]:
    positions = robot.bus.sync_read("Present_Position")
    return np.array([float(positions[name]) for name in JOINT_NAMES]), float(positions["gripper"])


def hold_current_pose(robot: SO101Follower) -> None:
    raw = robot.bus.sync_read("Present_Position", normalize=False)
    robot.bus.sync_write("Goal_Position", raw, normalize=False, num_retry=5)
    robot.bus.enable_torque(num_retry=5)


def check_emergency_stop() -> None:
    if msvcrt.kbhit() and msvcrt.getwch().lower() in {"q", " ", "\x1b"}:
        raise EmergencyStop


def execute_plan(
    robot: SO101Follower,
    plan: LinearPlan,
    gripper_position: float,
    waypoint_spacing_m: float,
    speed_m_s: float,
) -> None:
    interval = waypoint_spacing_m / speed_m_s
    next_time = time.perf_counter()
    for q_deg in plan.joint_waypoints:
        check_emergency_stop()
        action = {f"{name}.pos": float(q_deg[index]) for index, name in enumerate(JOINT_NAMES)}
        action["gripper.pos"] = gripper_position
        robot.send_action(action)
        next_time += interval
        remaining = next_time - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)


def print_position(kinematics: URDFKinematics, q_deg: np.ndarray, prefix: str = "末端位置") -> None:
    x, y, z = kinematics.forward(q_deg)[:3, 3]
    print(f"{prefix}: x={x * 100:.2f} cm, y={y * 100:.2f} cm, z={z * 100:.2f} cm")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=ROBOT_PORT)
    parser.add_argument("--robot-id", default=ROBOT_ID)
    parser.add_argument("--step-cm", type=float, default=1.0, help="Distance moved by each key press.")
    parser.add_argument("--speed-cm-s", type=float, default=MOVE_SPEED_CM_S)
    parser.add_argument("--waypoint-mm", type=float, default=WAYPOINT_SPACING_MM)
    parser.add_argument("--joint-margin-deg", type=float, default=JOINT_MARGIN_DEG)
    parser.add_argument(
        "--startup-tolerance-deg",
        type=float,
        default=STARTUP_LIMIT_TOLERANCE_DEG,
        help="Allow small encoder jitter outside the calibrated limit during startup.",
    )
    parser.add_argument(
        "--move",
        nargs=2,
        metavar=("DIRECTION", "CM"),
        help="Override MOVE_DIRECTION and MOVE_DISTANCE_CM for one run.",
    )
    parser.add_argument(
        "--keyboard",
        action="store_true",
        help="Override CONTROL_MODE and enter keyboard control.",
    )
    parser.add_argument(
        "--preserve-orientation",
        action="store_true",
        help="Also try to preserve gripper orientation (reduces the reachable workspace).",
    )
    parser.add_argument("--simulate", action="store_true", help="Run FK/IK without connecting to hardware.")
    parser.add_argument(
        "--start-joints",
        nargs=5,
        type=float,
        default=[0, -60, 45, 30, 0],
        metavar=("PAN", "LIFT", "ELBOW", "WRIST", "ROLL"),
    )
    args = parser.parse_args()
    if CONTROL_MODE not in {"distance", "keyboard"}:
        parser.error('CONTROL_MODE must be either "distance" or "keyboard"')
    if (
        args.step_cm <= 0
        or args.speed_cm_s <= 0
        or args.waypoint_mm <= 0
        or args.joint_margin_deg < 0
        or args.startup_tolerance_deg < 0
    ):
        parser.error("step, speed and waypoint spacing must all be positive")

    kinematics = URDFKinematics(URDF_PATH)
    robot: SO101Follower | None = None
    if args.simulate:
        q_deg = np.array(args.start_joints, dtype=float)
        lower_deg, upper_deg = kinematics.joint_limits_deg()
        gripper_position = 50.0
    else:
        robot = SO101Follower(
            SO101FollowerConfig(port=args.port, id=args.robot_id, use_degrees=True, cameras={})
        )
        if not robot.calibration:
            raise SystemExit(f"找不到标定文件: {robot.calibration_fpath}")
        robot.bus.connect()
        if not robot.is_calibrated:
            robot.bus.disconnect(disable_torque=False)
            raise SystemExit("电机中的标定与本地标定文件不一致，请先重新标定或同步标定。")
        q_deg, gripper_position = current_state(robot)
        lower_deg, upper_deg = effective_joint_limits(
            kinematics, calibrated_joint_limits(robot)
        )
    lower_deg = lower_deg + args.joint_margin_deg
    upper_deg = upper_deg - args.joint_margin_deg
    if np.any(lower_deg >= upper_deg):
        raise SystemExit("关节安全余量过大，导致可用关节范围为空。")

    overrun_deg = joint_limit_overrun_deg(q_deg, lower_deg, upper_deg)
    if np.any(overrun_deg > args.startup_tolerance_deg):
        details = ", ".join(
            f"{name}={q_deg[i]:.1f}° (允许 {lower_deg[i]:.1f}°~{upper_deg[i]:.1f}°)"
            for i, name in enumerate(JOINT_NAMES)
            if overrun_deg[i] > args.startup_tolerance_deg
        )
        if robot is not None and robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)
        raise SystemExit(f"当前姿态超出安全求解范围，请手动移回范围后重试: {details}")
    if np.any(overrun_deg > 0):
        details = ", ".join(
            f"{JOINT_NAMES[i]} 超出 {overrun_deg[i]:.2f}°"
            for i in range(len(JOINT_NAMES))
            if overrun_deg[i] > 0
        )
        print(f"提示：当前关节位于标定边界附近（{details}）；允许启动，IK 目标仍限制在安全范围内。")

    spacing_m = args.waypoint_mm / 1000
    # The assignment commands translation only. Position-only IK uses the two
    # redundant joints to maximize reach while the current seed keeps motion
    # continuous. Orientation holding can be enabled for grasping tasks.
    orientation_weight = 0.05 if args.preserve_orientation else 0.0

    def move(direction_name: str, distance_cm: float) -> LinearPlan:
        nonlocal q_deg
        delta = DIRECTIONS[direction_name] * distance_cm / 100
        plan = solve_linear_plan(
            kinematics,
            q_deg,
            delta,
            lower_deg,
            upper_deg,
            waypoint_spacing_m=spacing_m,
            orientation_weight=orientation_weight,
        )
        if robot is not None:
            execute_plan(robot, plan, gripper_position, spacing_m, args.speed_cm_s / 100)
        if plan.joint_waypoints:
            q_deg = plan.joint_waypoints[-1].copy()
        if not plan.reached_target:
            error_mm = plan.failure.position_error_m * 1000 if plan.failure else float("nan")
            print(
                f"IK 无解：请求 {plan.requested_m * 100:.2f} cm，沿原方向最多移动 "
                f"{plan.achieved_m * 100:.2f} cm（末端误差 {error_mm:.1f} mm）。"
            )
        else:
            print(f"完成 {direction_name} {distance_cm:.2f} cm。")
        print_position(kinematics, q_deg)
        return plan

    try:
        if robot is not None:
            hold_current_pose(robot)
        print_position(kinematics, q_deg, "起始末端位置")
        print(
            "坐标系：+X=前，+Y=左，+Z=上；"
            f"步长={args.step_cm:.1f} cm，轨迹点间距={args.waypoint_mm:.1f} mm。"
        )

        requested_move = args.move
        if requested_move is None and not args.keyboard and CONTROL_MODE == "distance":
            requested_move = (MOVE_DIRECTION, str(MOVE_DISTANCE_CM))

        if requested_move:
            direction_name, distance_text = requested_move
            direction_name = direction_name.lower()
            if direction_name not in DIRECTIONS:
                parser.error(f"unknown direction: {direction_name}")
            distance_cm = float(distance_text)
            if distance_cm <= 0:
                parser.error("move distance must be positive")
            move(direction_name, distance_cm)
            return 0

        if args.simulate:
            print("仿真检查完成。")
            return 0

        print("键盘：W/S 前后，A/D 左右，R/F 上下；Q/空格/Esc 急停并退出。")
        while True:
            key = msvcrt.getwch().lower()
            if key in {"q", " ", "\x1b"}:
                raise EmergencyStop
            if key in KEY_DIRECTIONS:
                move(KEY_DIRECTIONS[key], args.step_cm)
    except EmergencyStop:
        print("急停：正在关闭扭矩。")
    finally:
        if robot is not None and robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
