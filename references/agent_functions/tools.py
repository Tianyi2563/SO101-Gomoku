"""Allowlisted tools over the existing so101_arm library; MockBackend only."""
from dataclasses import replace
import inspect
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

SOURCE = Path(os.getenv('SO101_ARM_DIR', str(Path.home() / 'Documents/ChatGPT/LEROBOT/so101_arm'))).resolve()
if not (SOURCE / 'so101/session.py').is_file():
    raise RuntimeError('Set SO101_ARM_DIR to the existing so101_arm project directory')
sys.path.insert(0, str(SOURCE))
from so101.config import AppConfig
from so101.presets import PresetStore
from so101.session import open_session
from so101.vision import BallDetector, CameraSource, VisualServo


def schema(name, description, properties=None, required=None):
    return {'type': 'function', 'function': {'name': name, 'description': description,
        'parameters': {'type': 'object', 'properties': properties or {},
                       'required': required or [], 'additionalProperties': False}}}


TOOLS = [
    schema('get_robot_state', '读取Mock关节动作值、URDF弧度、FK末端米坐标、停止和错误状态；每次运动前必须调用。'),
    schema('list_presets', '读取现有预设姿态和动作名称。'),
    schema('move_to_pose', '执行现有预设姿态，仅Mock；必须先查询状态并从list_presets选名称。', {'name': {'type': 'string'}}, ['name']),
    schema('move_tip', '复用IK沿基座坐标轴移动末端，仅Mock；1到20毫米，返回实际位移，可能部分可达。',
           {'axis': {'type': 'string', 'enum': ['+x', '-x', '+y', '-y', '+z', '-z']},
            'distance_mm': {'type': 'number', 'minimum': 1, 'maximum': 20}}, ['axis', 'distance_mm']),
    schema('set_gripper', '设置Mock夹爪，校准归一化开度20到80，0闭100开；先查状态。', {'opening': {'type': 'number', 'minimum': 20, 'maximum': 80}}, ['opening']),
    schema('detect_object', '只支持黑球/黑色圆球/black ball。默认合成场景；object_id为单目标会话ID，不是通用身份跟踪。', {'description': {'type': 'string'}}, ['description']),
    schema('track_object', '复用VisualServo执行最多3步有界视觉伺服后自动hold，不是无限后台任务。仅Mock；先查状态并成功检测新鲜目标。', {'object_id': {'type': 'integer', 'enum': [1]}}, ['object_id']),
    schema('stop_tracking', '停止运动并锁存会话，后续运动拒绝；不是实体急停。'),
    schema('stop_motion', '停止运动并锁存会话，后续运动拒绝；不是实体急停。'),
]


class SyntheticCamera:
    """Pinhole fixture based on tools/test_vision_loop.py; not real-camera evidence."""
    def __init__(self, arm):
        self.arm = arm
        self.ball = arm.tip_position() + np.array([0.5, 0.04, 0.03])
        self.visible = True

    def read(self):
        import cv2
        frame = np.full((480, 640, 3), 235, dtype=np.uint8)
        rel = self.ball - self.arm.tip_position()
        if self.visible and rel[0] > 0.12:
            u, v = 320 - 500 * rel[1] / rel[0], 240 - 500 * rel[2] / rel[0]
            cv2.circle(frame, (round(u), round(v)), round(500 * 0.025 / rel[0]), (10, 10, 10), -1)
        return frame

    def close(self):
        pass


class RobotTools:
    def __init__(self, allow_camera=False):
        cfg = AppConfig.load(SOURCE / 'config.json')
        self.session = open_session(cfg, mock=True, verbose=False)
        if not self.session.backend.is_mock:
            self.session.close()
            raise RuntimeError('Hardware backend prohibited')
        self.arm = self.session.arm
        try:
            self.store = PresetStore.load(cfg.presets_path, create_default=False)
            self.camera = CameraSource(cfg.vision).open() if allow_camera else SyntheticCamera(self.arm)
        except Exception:
            self.session.close()
            raise
        self.camera_kind = 'real camera + Mock arm' if allow_camera else 'synthetic pinhole fixture + Mock arm'
        self.detector = BallDetector(cfg.vision)
        self.stopped, self.tracking = False, False
        self.checked_at = self.detected_at = None
        self.distance_budget_mm, self.motion_count = 50.0, 0
        self.errors = []
        self.allowed = {t['function']['name'] for t in TOOLS}

    def get_robot_state(self):
        state = self.arm.state()
        self.checked_at = time.monotonic()
        return dict(ok=True, mode='mock', connected_to_hardware=False,
            joint_action_units='LeRobot calibrated normalized position; NOT degrees/radians',
            joint_actions=state.action, joint_angles_rad=state.q.tolist(),
            tip_position_m=state.tip.tolist(), moving=state.moving,
            tracking=self.tracking, stopped=self.stopped, errors=list(self.errors),
            scene=self.camera_kind, remaining_tip_budget_mm=self.distance_budget_mm)

    def list_presets(self):
        return dict(ok=True, poses={n: p.label for n, p in self.store.poses.items()},
                    actions={n: a.label for n, a in self.store.actions.items()},
                    note='Sequences inventoried but not exposed for execution in this minimal demo')

    def _before_motion(self):
        if self.stopped:
            raise ValueError('Stop latched; operator must restart session')
        if self.checked_at is None or time.monotonic() - self.checked_at > 30:
            raise ValueError('Call get_robot_state within 30s before every motion')
        if self.motion_count >= 8:
            raise ValueError('Session motion budget exceeded')
        self.checked_at = None
        self.motion_count += 1
        self.arm.state()

    def _wait(self):
        if not self.arm.wait_until_settled(timeout=8):
            raise ValueError('Controller settle timeout')

    def move_to_pose(self, name):
        if not isinstance(name, str) or name not in self.store.poses:
            raise ValueError('Unknown preset pose')
        self._before_motion()
        self.store.play_pose(self.arm, name, duration=1.2, wait=False)
        self._wait()
        return dict(ok=True, mode='mock', pose=name, tip_position_m=self.arm.tip_position().tolist())

    def move_tip(self, axis, distance_mm):
        axes = {'+x': [1,0,0], '-x': [-1,0,0], '+y': [0,1,0], '-y': [0,-1,0], '+z': [0,0,1], '-z': [0,0,-1]}
        if not isinstance(axis, str) or axis not in axes or type(distance_mm) not in (int, float) or not math.isfinite(distance_mm) or not 1 <= distance_mm <= 20:
            raise ValueError('Expected valid base-frame axis and finite distance 1..20 mm')
        if distance_mm > self.distance_budget_mm:
            raise ValueError('Session Cartesian travel budget exceeded')
        self._before_motion()
        before = self.arm.tip_position().copy()
        self.distance_budget_mm -= distance_mm
        full, achieved, note = self.arm.move_tip_by(axes[axis], distance_mm / 1000)
        self._wait()
        delta = (self.arm.tip_position() - before) * 1000
        return dict(ok=True, mode='mock', fully_reachable=bool(full), solver_distance_mm=float(achieved * 1000),
                    measured_delta_mm=delta.tolist(), note=note)

    def set_gripper(self, opening):
        if type(opening) not in (int, float) or not math.isfinite(opening) or not 20 <= opening <= 80:
            raise ValueError('Gripper normalized opening must be finite 20..80')
        self._before_motion()
        self.arm.set_gripper(opening, duration=0.4, wait=False)
        self._wait()
        return dict(ok=True, mode='mock', gripper=self.arm.state().action['gripper'])

    def detect_object(self, description):
        if description not in ('黑球', '黑色圆球', 'black ball'):
            raise ValueError('Existing detector only supports 黑球 / 黑色圆球 / black ball')
        frame = self.camera.read()
        if frame is None:
            raise ValueError('Camera frame unavailable')
        det = self.detector.detect(frame)
        self.detected_at = time.monotonic() if det.found else None
        if not det.found:
            self.stop_motion()
        return dict(ok=True, found=bool(det.found), object_id=1 if det.found else None,
                    center_px=det.center, radius_px=det.radius, scene=self.camera_kind,
                    identity_contract='single round black candidate; not persistent multi-object identity')

    def track_object(self, object_id):
        if type(object_id) is not int or object_id != 1:
            raise ValueError('Unknown object_id')
        if self.detected_at is None or time.monotonic() - self.detected_at > 15:
            raise ValueError('Run detect_object first; target expires after 15s')
        self._before_motion()
        if self.distance_budget_mm < 9:
            raise ValueError('Insufficient tracking travel budget')
        self.distance_budget_mm -= 9
        cfg = replace(self.session.cfg.vision, max_step_mm=3.0, show_window=False)
        owner = self
        class GuardedCamera:
            def read(self):
                if owner.stopped:
                    raise ValueError('Tracking stopped')
                frame = owner.camera.read()
                if frame is None or not owner.detector.detect(frame).found:
                    raise ValueError('Tracking target lost; holding position')
                return frame
        servo = VisualServo(self.arm, self.session.model, cfg)
        self.tracking = True
        try:
            converged = servo.run(GuardedCamera(), max_iterations=3, show_window=False)
            return dict(ok=True, mode='mock', converged=bool(converged), iterations=len(servo.history),
                        tracking=False, bounded_skill=True, scene=self.camera_kind,
                        history=[dict(du=s.du, dv=s.dv, delta_mm=s.delta_mm) for s in servo.history])
        finally:
            self.tracking = False
            self.arm.hold()

    def stop_motion(self):
        self.stopped = True
        self.checked_at = None
        self.tracking = False
        self.arm.hold()
        return dict(ok=True, mode='mock', stopped=True, hardware_emergency_stop=False)

    def stop_tracking(self):
        return self.stop_motion()

    def close(self):
        try:
            self.stop_motion()
        finally:
            self.camera.close()
            self.session.close()

    def dispatch(self, name, arguments):
        try:
            if not isinstance(name, str) or name not in self.allowed or not isinstance(arguments, dict):
                raise ValueError('Unknown tool or non-object arguments')
            method = getattr(self, name)
            inspect.signature(method).bind(**arguments)
            return method(**arguments)
        except (ValueError, TypeError) as exc:
            self.errors.append(str(exc))
            self.stop_motion()
            return dict(ok=False, error=str(exc), mode='mock', stopped=True)
        except Exception:
            self.errors.append('Tool failed; stopped')
            self.stop_motion()
            return dict(ok=False, error='Tool failed; stopped', mode='mock', stopped=True)
