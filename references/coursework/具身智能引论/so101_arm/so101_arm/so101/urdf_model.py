"""解析 SO101 的 URDF 模型。

只保留运动学需要的部分：关节链、origin 变换、旋转轴、关节限位。
不做动力学、不做 mesh 可视化。

两个关键约定（SO101 new_calib）：
1. URDF 里每个关节的限位是 [lower, upper]（弧度），且**行程中点即标定零点**。
   也就是说 lerobot 标定后动作值为 0 时，URDF 里对应的关节角是 (lower+upper)/2。
   这个零点由 :pyattr:`Joint.zero` 给出。
2. URDF 的关节角 q 与 lerobot 下发的动作值 a（单位：度）之间是仿射关系，
   具体换算见 :mod:`so101.frames`。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

MOVABLE_TYPES = {"revolute", "continuous"}


def _parse_floats(text: str | None, default: Sequence[float]) -> np.ndarray:
    if text is None or not text.strip():
        return np.asarray(default, dtype=float)
    return np.asarray([float(v) for v in text.split()], dtype=float)


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    """URDF 的 rpy 约定：固定轴 XYZ，即 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def make_transform(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """由平移 + rpy 构造 4x4 齐次变换矩阵。"""
    t = np.eye(4, dtype=float)
    t[:3, :3] = rpy_to_matrix(rpy)
    t[:3, 3] = np.asarray(xyz, dtype=float).reshape(3)
    return t


def rot_axis(axis: Sequence[float], angle: float) -> np.ndarray:
    """绕任意单位轴转 angle（弧度）的 4x4 齐次变换（Rodrigues 公式）。"""
    a = np.asarray(axis, dtype=float).reshape(3)
    n = float(np.linalg.norm(a))
    if n < 1e-12:  # 退化：当作纯平移
        return np.eye(4)
    a = a / n
    x, y, z = a
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    r = np.array(
        [
            [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
        ]
    )
    t = np.eye(4)
    t[:3, :3] = r
    return t


@dataclass
class Joint:
    """URDF 链上的一个关节。"""

    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray  # 4x4，父关节坐标系 -> 本关节坐标系
    axis: np.ndarray  # (3,)，在本关节坐标系下
    lower: float  # 弧度（URDF 坐标）
    upper: float

    @property
    def movable(self) -> bool:
        return self.type in MOVABLE_TYPES

    @property
    def zero(self) -> float:
        """new_calib 约定：标定零点 = 关节行程中点。"""
        return 0.5 * (self.lower + self.upper)

    @property
    def range_around_zero(self) -> tuple[float, float]:
        """相对标定零点的行程（弧度），形如 (min, max)。"""
        return (self.lower - self.zero, self.upper - self.zero)

    @property
    def range_around_zero_deg(self) -> tuple[float, float]:
        lo, hi = self.range_around_zero
        return (math.degrees(lo), math.degrees(hi))

    def __repr__(self) -> str:  # pragma: no cover - 仅调试用
        lo, hi = self.range_around_zero_deg
        return (
            f"<Joint {self.name} {self.type} {self.parent}->{self.child} "
            f"zero={math.degrees(self.zero):.2f}deg range=[{lo:.1f},{hi:.1f}]deg>"
        )


class UrdfModel:
    """从 URDF 文件解析出的一条运动链。"""

    def __init__(self, path: str | Path, chain: list[Joint], tip_link: str, robot_name: str):
        self.path = Path(path)
        self.chain = chain
        self.tip_link = tip_link
        self.robot_name = robot_name

    # ------------------------------------------------------------------ 解析
    @classmethod
    def load(
        cls,
        path: str | Path,
        tip_link: str | None = None,
    ) -> "UrdfModel":
        """读取 URDF，并从根 link 走到 ``tip_link`` 取出唯一一条链。

        ``tip_link`` 为空时自动选择（优先 gripper_frame_link / gripper / 叶子 link）。
        """
        path = Path(path)
        root = ET.parse(str(path)).getroot()

        joints: dict[str, Joint] = {}
        child_of: dict[str, str] = {}  # child_link -> joint_name
        all_links: set[str] = set()

        for j in root.findall("joint"):
            name = j.get("name", "")
            jtype = j.get("type", "fixed")
            parent = j.find("parent").get("link")  # type: ignore[union-attr]
            child = j.find("child").get("link")  # type: ignore[union-attr]
            origin_el = j.find("origin")
            axis_el = j.find("axis")
            limit_el = j.find("limit")

            origin = make_transform(
                _parse_floats(origin_el.get("xyz") if origin_el is not None else None, (0, 0, 0)),
                _parse_floats(origin_el.get("rpy") if origin_el is not None else None, (0, 0, 0)),
            )
            axis = _parse_floats(axis_el.get("xyz") if axis_el is not None else None, (1, 0, 0))
            if limit_el is not None and jtype in MOVABLE_TYPES:
                lower = float(limit_el.get("lower", 0.0))
                upper = float(limit_el.get("upper", 0.0))
            else:
                lower, upper = 0.0, 0.0

            joints[name] = Joint(name, jtype, parent, child, origin, axis, lower, upper)
            child_of[child] = name
            all_links.add(parent)
            all_links.add(child)

        if tip_link is None:
            for candidate in ("gripper_frame_link", "gripper", "moving_jaw_so101_v1"):
                if candidate in child_of:
                    tip_link = candidate
                    break
            else:
                # 取没有任何子关节的叶子 link（最深的那个）
                leaves = [link for link in all_links if link not in child_of]
                if not leaves:
                    raise ValueError(f"{path} 中没有找到可用的末端 link")
                tip_link = leaves[-1]

        if tip_link not in child_of:
            raise ValueError(f"URDF 中找不到 link: {tip_link}")

        # 从 tip 沿 parent 反查，得到根 -> tip 的唯一路径
        chain: list[Joint] = []
        cursor = tip_link
        while cursor in child_of:
            joint = joints[child_of[cursor]]
            chain.append(joint)
            cursor = joint.parent
        chain.reverse()
        if not chain:
            raise ValueError(f"从 {tip_link} 出发没有解析到任何关节")

        return cls(path, chain, tip_link, root.get("name", "unknown"))

    # ------------------------------------------------------------- 基本属性
    @property
    def movable_joints(self) -> list[Joint]:
        return [j for j in self.chain if j.movable]

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.movable_joints]

    @property
    def n_dof(self) -> int:
        return len(self.movable_joints)

    def zero_pose(self) -> np.ndarray:
        """所有可动关节处于标定零点（= 行程中点）时的关节角向量（弧度）。"""
        return np.asarray([j.zero for j in self.movable_joints], dtype=float)

    def limits(self) -> np.ndarray:
        """可动关节的 URDF 限位，形状 (n, 2)。"""
        return np.asarray([[j.lower, j.upper] for j in self.movable_joints], dtype=float)

    # ------------------------------------------------------- 正运动学 / 雅可比
    def fk_all(self, q: Sequence[float]) -> tuple[np.ndarray, list[np.ndarray]]:
        """正运动学。

        参数
        ----
        q : 长度 n_dof 的关节角（弧度，URDF 坐标系）

        返回
        ----
        (T_tip, joint_frames)
            T_tip : 4x4，基座坐标系下末端（tip_link）的位姿
            joint_frames : 每个可动关节的 4x4 位姿（绕轴旋转【前】的关节坐标系），
                           雅可比和解析 IK 都要用
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.size != self.n_dof:
            raise ValueError(f"需要 {self.n_dof} 个关节角，收到 {q.size} 个")

        t = np.eye(4)
        frames: list[np.ndarray] = []
        k = 0
        for joint in self.chain:
            t = t @ joint.origin
            if joint.movable:
                frames.append(t.copy())
                t = t @ rot_axis(joint.axis, float(q[k]))
                k += 1
        return t, frames

    def fk(self, q: Sequence[float]) -> np.ndarray:
        """末端位姿 4x4。"""
        return self.fk_all(q)[0]

    def fk_position(self, q: Sequence[float]) -> np.ndarray:
        """末端位置 (3,)，单位米。"""
        return self.fk(q)[:3, 3].copy()

    def jacobian(self, q: Sequence[float]) -> np.ndarray:
        """几何雅可比 6xN（基座坐标系下）。

        前 3 行是线速度，后 3 行是角速度。仅包含可动关节（不含夹爪以外的固定关节）。
        对旋转关节：Jv_i = axis_i x (p_tip - p_i)，Jw_i = axis_i
        """
        t_tip, frames = self.fk_all(q)
        p_tip = t_tip[:3, 3]
        jac = np.zeros((6, self.n_dof), dtype=float)
        for i, (joint, frame) in enumerate(zip(self.movable_joints, frames)):
            axis_world = frame[:3, :3] @ joint.axis
            jac[:3, i] = np.cross(axis_world, p_tip - frame[:3, 3])
            jac[3:, i] = axis_world
        return jac


def default_model() -> UrdfModel:
    """加载项目自带的 SO101 URDF。"""
    path = Path(__file__).resolve().parent.parent / "assets" / "so101_new_calib.urdf"
    return UrdfModel.load(path)


if __name__ == "__main__":  # pragma: no cover
    m = default_model()
    print(f"robot={m.robot_name}  tip={m.tip_link}  dof={m.n_dof}")
    for j in m.chain:
        print("  ", j)
