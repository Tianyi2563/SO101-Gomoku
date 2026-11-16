"""Move an SO-101 smoothly between recorded poses using keyboard shortcuts."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import msvcrt
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import TypeAlias

# Some Windows launchers expose a legacy output encoding even though the project
# path contains Chinese characters.  Replacing only unprintable path characters
# keeps diagnostics usable without affecting serial communication.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(errors="replace")

PROJECT_ROOT = Path(__file__).resolve().parent
VENV_PYTHON = PROJECT_ROOT / ".venv" / "Scripts" / "python.exe"
if importlib.util.find_spec("lerobot") is None and VENV_PYTHON.is_file():
    os.environ.setdefault("PYTHONUTF8", "1")
    completed = subprocess.run([str(VENV_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])
    raise SystemExit(completed.returncode)

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig

ACTION_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]
BODY_JOINTS = ACTION_KEYS[:5]
DEFAULT_RECORD_DIR = PROJECT_ROOT / "outputs" / "servo_records"
DEFAULT_BINDINGS = PROJECT_ROOT / "pose_bindings.json"
BindingValue: TypeAlias = str | list[str]
RecordedSample: TypeAlias = tuple[float, dict[str, float]]
ResolvedRecord: TypeAlias = tuple[Path, list[RecordedSample], list[str]]


class EmergencyStop(Exception):
    pass


def newest_record(record_dir: Path) -> Path:
    candidates = list(record_dir.glob("*.json"))
    if not candidates:
        raise FileNotFoundError(f"No recorded poses found in {record_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def resolve_binding(value: str, record_dir: Path, bindings_dir: Path) -> Path:
    if value.lower() == "latest":
        return newest_record(record_dir)
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = bindings_dir / path
    return path.resolve()


def load_record(path: Path) -> list[RecordedSample]:
    data = json.loads(path.read_text(encoding="utf-8"))
    samples = data.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"Pose file contains no samples: {path}")
    result: list[RecordedSample] = []
    for index, sample in enumerate(samples):
        action = sample.get("action")
        if not isinstance(action, dict) or set(action) != set(ACTION_KEYS):
            raise ValueError(f"Pose file sample {index} has invalid action keys: {path}")
        elapsed = float(sample.get("elapsed_seconds", index))
        result.append((elapsed, {key: float(action[key]) for key in ACTION_KEYS}))
    return result


def load_bindings(path: Path) -> dict[str, BindingValue]:
    data = json.loads(path.read_text(encoding="utf-8"))
    bindings = data.get("bindings", data)
    if not isinstance(bindings, dict):
        raise ValueError("Bindings file must contain a JSON object")
    result: dict[str, BindingValue] = {}
    for key, value in bindings.items():
        key = str(key).lower()
        if len(key) != 1 or not key.isalpha() or key in {"q", "r"}:
            raise ValueError(f"Invalid pose key: {key!r}")
        if isinstance(value, list):
            if not value or not all(isinstance(item, str) and item for item in value):
                raise ValueError(f"Pose sequence for {key!r} must be a non-empty list of paths")
            result[key] = value
        elif isinstance(value, str) and value:
            result[key] = value
        else:
            raise ValueError(f"Binding for {key!r} must be a path or a list of paths")
    return result


def resolve_poses(
    bindings: dict[str, BindingValue],
    record_dir: Path,
    bindings_dir: Path,
    robot: SO101Follower,
) -> dict[str, list[ResolvedRecord]]:
    resolved: dict[str, list[ResolvedRecord]] = {}
    for key, value in bindings.items():
        values = value if isinstance(value, list) else [value]
        sequence: list[ResolvedRecord] = []
        for item in values:
            pose_path = resolve_binding(item, record_dir, bindings_dir)
            samples = load_record(pose_path)
            warnings = [warning for _, action in samples for warning in validate_action(robot, action)]
            sequence.append((pose_path, samples, list(dict.fromkeys(warnings))))
        resolved[key] = sequence
    return resolved


def validate_action(robot: SO101Follower, action: dict[str, float]) -> list[str]:
    """Validate the record format and report, but do not reject, degree-limit overruns.

    LeRobot's degree normalization intentionally permits positions outside the
    range captured during calibration.  The servo's configured position limits
    remain the final safety boundary, so treating those values as a file error
    made valid read_servo recordings crash the controller before connection.
    """
    warnings: list[str] = []
    for key, value in action.items():
        if not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")
    for key in BODY_JOINTS:
        motor = key.removesuffix(".pos")
        cal = robot.calibration[motor]
        midpoint = (cal.range_min + cal.range_max) / 2
        raw_target = action[key] * 4095 / 360 + midpoint
        if not cal.range_min <= raw_target <= cal.range_max:
            warnings.append(
                f"{key}={action[key]:.2f}° maps to raw {raw_target:.0f}, outside the "
                f"recorded calibration range [{cal.range_min}, {cal.range_max}]; "
                "the servo's configured limit will remain in force"
            )
    if not 0 <= action["gripper.pos"] <= 100:
        raise ValueError("gripper.pos must be between 0 and 100 percent")
    return warnings


def current_action(robot: SO101Follower) -> dict[str, float]:
    positions = robot.bus.sync_read("Present_Position")
    return {f"{name}.pos": float(value) for name, value in positions.items()}


def hold_current_pose(robot: SO101Follower) -> None:
    raw = robot.bus.sync_read("Present_Position", normalize=False)
    robot.bus.sync_write("Goal_Position", raw, normalize=False, num_retry=5)
    robot.bus.enable_torque(num_retry=5)


def check_for_emergency_stop() -> None:
    if not msvcrt.kbhit():
        return
    key = msvcrt.getwch().lower()
    if key in {" ", "\x1b", "q"}:
        raise EmergencyStop


def move_smoothly(
    robot: SO101Follower,
    target: dict[str, float],
    duration: float,
    fps: float,
) -> None:
    start = current_action(robot)
    steps = max(1, round(duration * fps))
    interval = duration / steps
    started_at = time.perf_counter()

    for step in range(1, steps + 1):
        check_for_emergency_stop()
        t = step / steps
        alpha = t * t * (3 - 2 * t)  # smoothstep: gentle start and stop
        action = {key: start[key] + (target[key] - start[key]) * alpha for key in ACTION_KEYS}
        robot.send_action(action)
        next_step_at = started_at + step * interval
        remaining = next_step_at - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)


def replay_record(robot: SO101Follower, samples: list[RecordedSample], move_seconds: float, fps: float) -> None:
    """Move to a recording's first frame, then replay its original time axis."""
    move_smoothly(robot, samples[0][1], move_seconds, fps)
    if len(samples) == 1:
        return

    first_timestamp = samples[0][0]
    replay_started_at = time.perf_counter()
    for timestamp, action in samples[1:]:
        check_for_emergency_stop()
        due_at = replay_started_at + max(0.0, timestamp - first_timestamp)
        remaining = due_at - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        robot.send_action(action)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM4")
    parser.add_argument("--robot-id", default="my_follower")
    parser.add_argument("--bindings", type=Path, default=DEFAULT_BINDINGS)
    parser.add_argument("--record-dir", type=Path, default=DEFAULT_RECORD_DIR)
    parser.add_argument("--move-seconds", type=float, default=3.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--check", action="store_true", help="Validate setup without enabling torque.")
    args = parser.parse_args()
    if args.move_seconds <= 0 or not 1 <= args.fps <= 100:
        parser.error("--move-seconds must be positive and --fps must be in [1, 100]")

    bindings_path = args.bindings.expanduser().resolve()
    bindings = load_bindings(bindings_path)
    robot = SO101Follower(
        SO101FollowerConfig(port=args.port, id=args.robot_id, use_degrees=True, cameras={})
    )
    if not robot.calibration:
        raise SystemExit(f"Calibration file not found: {robot.calibration_fpath}")

    resolved = resolve_poses(bindings, args.record_dir.resolve(), bindings_path.parent, robot)

    robot.bus.connect()
    try:
        if not robot.is_calibrated:
            raise RuntimeError("Calibration file does not match motor calibration values")
        if args.check:
            for key, sequence in resolved.items():
                for index, (path, samples, warnings) in enumerate(sequence, start=1):
                    step = f" step {index}/{len(sequence)}" if len(sequence) > 1 else ""
                    duration = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0.0
                    print(f"{key.upper()}{step}: {path} ({len(samples)} samples, {duration:.2f}s)")
                    for warning in warnings:
                        print(f"WARNING: {warning}")
            print("CHECK_OK")
            return 0

        hold_current_pose(robot)
        print("LeRobot pose controller is ready.")
        for key, sequence in resolved.items():
            names = " -> ".join(path.name for path, _, _ in sequence)
            print(f"  {key.upper()} = {names}")
        print("  R = reload pose files, Q = quit, Space/Esc = emergency stop")

        while True:
            key = msvcrt.getwch().lower()
            if key == "q":
                break
            if key in {" ", "\x1b"}:
                raise EmergencyStop
            if key == "r":
                bindings = load_bindings(bindings_path)
                resolved = resolve_poses(
                    bindings, args.record_dir.resolve(), bindings_path.parent, robot
                )
                print("Pose files reloaded.")
                continue
            if key not in resolved:
                continue

            sequence = resolved[key]
            for index, (pose_path, samples, _) in enumerate(sequence, start=1):
                print(f"Moving {key.upper()} step {index}/{len(sequence)}: {pose_path.name}")
                replay_record(robot, samples, args.move_seconds, args.fps)
                action = samples[-1][1]
                final = current_action(robot)
                largest_error = max(abs(final[name] - action[name]) for name in ACTION_KEYS)
                print(f"Reached step {index}/{len(sequence)}; max error {largest_error:.2f}")
    except EmergencyStop:
        print("Emergency stop: torque disabled.")
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=not args.check)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
