"""正运动学与逆运动学。

本模块提供三层能力：

1. :func:`ik_planar_2r` —— **两连杆解析解**，就是讲义里推导的那套公式，
   用于原理讲解和单元测试（看得见"两组解"和"够不着"）。
2. :meth:`so101.urdf_model.UrdfModel.fk` / :meth:`~so101.urdf_model.UrdfModel.jacobian`
   —— 五自由度 SO101 的正运动学与几何雅可比。
3. :func:`ik_dls` —— **阻尼最小二乘（Damped Least Squares, DLS）数值逆运动学**，
   实际驱动机械臂时用的就是它。

为什么实际控制用数值解而不是解析解
----------------------------------
讲义里的解析解只对"平面两连杆"成立。SO101 有 5 个自由度、连杆不在同一平面内，
且要满足关节限位，几何解非常繁琐。工程上更常用的做法是：

* 把 IK 变成一个**迭代优化**问题：不断用雅可比把末端位置误差"摊"到各关节上；
* 雅可比奇异（手臂伸直、够到边界）时用**阻尼**保证数值稳定；
* 多解（冗余）用**零空间投影**挑一个离当前姿态最近、腕部不漂移的解；
* 每步都裁到**关节限位**内。

这几件事合起来，效果就是"末端朝目标直线走过去，姿态基本不变，够不着就停下"
——正好是课程验收要求的行为。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .urdf_model import UrdfModel

# --------------------------------------------------------------------------- #
#  1) 两连杆解析解（原理讲解）
# --------------------------------------------------------------------------- #


def normalize_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class TwoLinkSolution:
    theta1: float  # 第一关节角（弧度）
    theta2: float  # 第二关节角（弧度）
    elbow_up: bool  # 肘部朝向（对应讲义里 ± 号的那一组解）


def ik_planar_2r(
    l1: float,
    l2: float,
    x: float,
    y: float,
) -> list[TwoLinkSolution] | None:
    """平面两连杆逆运动学（解析解）。

    正运动学（与讲义一致，把第二根连杆角定义为**相对第一根的夹角** θ2）：

    .. code-block:: text

        x = L1*cos(θ1) + L2*cos(θ1+θ2)
        y = L1*sin(θ1) + L2*sin(θ1+θ2)

    求解思路和讲义里"两圆交点"完全等价：

    .. code-block:: text

        r   = sqrt(x^2 + y^2)
        φ   = atan2(y, x)                        # 末端方位角
        cos θ2 = (r^2 - L1^2 - L2^2) / (2 L1 L2) # 余弦定理，得到 θ2 = ±β
        θ1  = φ ∓ atan2(L2 sin θ2, L1 + L2 cos θ2)

    可达性条件（讲义里的那个不等式）：

    .. code-block:: text

        |L1 - L2| <= r <= L1 + L2

    返回
    ----
    ``None`` 表示无解（太远或太近）；否则返回两组解（肘上 / 肘下）。
    """
    r = math.hypot(x, y)
    if r > l1 + l2 + 1e-12 or r < abs(l1 - l2) - 1e-12:
        return None  # 够不着：超出外圆或钻进内圆

    cos2 = (r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos2 = min(1.0, max(-1.0, cos2))  # 数值上贴边时夹一下，避免 acos 定义域越界
    beta = math.acos(cos2)

    solutions: list[TwoLinkSolution] = []
    for sign, elbow_up in ((+1.0, False), (-1.0, True)):
        theta2 = sign * beta
        theta1 = math.atan2(y, x) - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        solutions.append(
            TwoLinkSolution(normalize_angle(theta1), normalize_angle(theta2), elbow_up=elbow_up)
        )
    return solutions


def fk_planar_2r(l1: float, l2: float, theta1: float, theta2: float) -> tuple[float, float]:
    """平面两连杆正运动学，与 :func:`ik_planar_2r` 配套，用于自检。"""
    return (
        l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2),
        l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2),
    )


# --------------------------------------------------------------------------- #
#  2) 数值 IK：阻尼最小二乘
# --------------------------------------------------------------------------- #


@dataclass
class IkSolution:
    """数值 IK 的结果。"""

    q: np.ndarray  # 关节角（弧度，URDF 坐标），长度 = 自由度
    error: float  # 末端位置残差（米）
    iterations: int
    ok: bool
    reason: str = ""  # 失败原因：unreachable / limit / no_progress
    hit_limit: bool = False  # 解是否顶在关节限位上

    def __str__(self) -> str:
        state = "OK" if self.ok else f"FAIL({self.reason})"
        return f"<IkSolution {state} err={self.error * 1000:.2f}mm iters={self.iterations}>"


def clip_to_limits(
    q: np.ndarray, limits: np.ndarray, margin: float = 0.0
) -> tuple[np.ndarray, bool]:
    """把关节角裁到 [lower+margin, upper-margin]；返回 (裁剪后的 q, 是否被裁剪)。"""
    lo = limits[:, 0] + margin
    hi = limits[:, 1] - margin
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    q_clipped = np.clip(q, lo, hi)
    return q_clipped, bool(np.any(np.abs(q_clipped - q) > 1e-12))


def ik_dls(
    model: UrdfModel,
    target_position: Sequence[float],
    q_init: Sequence[float] | None = None,
    *,
    damping: float = 0.05,
    damping_min: float = 1e-4,
    damping_max: float = 2.0,
    step_limit: float = 0.12,
    tol: float = 1e-4,
    max_iter: int = 200,
    rest_q: Sequence[float] | None = None,
    rest_gain: float = 0.03,
    weights: Sequence[float] | None = None,
    limit_margin: float = math.radians(0.5),
) -> IkSolution:
    """阻尼最小二乘数值逆运动学（只约束末端**位置**）。

    每次迭代：

    .. code-block:: text

        e  = p_target - p_current                      # 3 维位置误差
        J  = ∂p/∂q                                     # 3xN 位置雅可比
        Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e  +  N (q_rest - q) * k
             └── 阻尼最小二乘主项 ─┘   └── 零空间正则项 ──┘
        q ← clip(q + clamp(Δq, ±step_limit), 限位)

    阻尼 λ 是**自适应**的（Levenberg–Marquardt 思路）：

    * 新解让误差变小 -> 接受该步，并把 λ 调小（越来越接近纯牛顿步，收敛快）；
    * 新解让误差变大 -> 拒绝该步，把 λ 调大（更像梯度下降，保证稳定）。

    固定 λ 会留下约 λ² 量级的稳态残差（实测约 1.3 mm），自适应阻尼能把它压到
    0.1 mm 以下，同时保留了"奇异位形不乱甩关节"的好处。

    参数
    ----
    damping : 初始阻尼 λ₀。
    step_limit : 单次迭代关节增量上限（弧度），限制"一步跨太远"。
    tol : 位置残差收敛阈值（米），默认 0.1 mm。
    rest_q : 零空间参考姿态。5 自由度做 3 维位置任务是冗余的，
             用 :math:`N = I - J⁺J` 投影把"多余的自由度"拉回参考姿态，
             这样解出来的腕部姿态不会莫名其妙地翻转。
    rest_gain : 零空间牵引增益（不要给太大，否则会和主任务抢）。
    weights : 三个位置分量的权重（默认等权）。
    limit_margin : 求解时就给限位留一点余量，避免顶死机械限位。

    返回
    ----
    :class:`IkSolution`：``ok=False`` 时 ``reason`` 说明是够不着（unreachable）
    还是被限位卡住（limit）。
    """
    limits = model.limits()
    target = np.asarray(target_position, dtype=float).reshape(3)
    w = np.ones(3) if weights is None else np.asarray(weights, dtype=float).reshape(3)

    q = model.zero_pose() if q_init is None else np.asarray(q_init, dtype=float).reshape(-1).copy()
    q, _ = clip_to_limits(q, limits, limit_margin)
    rest = q.copy() if rest_q is None else np.asarray(rest_q, dtype=float).reshape(-1).copy()

    n = model.n_dof
    I3 = np.eye(3)
    I5 = np.eye(n)
    lam = float(damping)
    hit_limit = False

    p = model.fk_position(q)
    err = float(np.linalg.norm(target - p))
    if err < tol:
        return IkSolution(q, err, 0, True, hit_limit=False)

    best_q, best_err = q.copy(), err
    stalls = 0

    for it in range(1, max_iter + 1):
        jac = model.jacobian(q)[:3, :] * w[:, None]
        e = (target - p) * w

        a = jac @ jac.T + (lam**2) * I3
        try:
            a_inv = np.linalg.inv(a)
        except np.linalg.LinAlgError:  # pragma: no cover
            a_inv = np.linalg.pinv(a)

        dq = jac.T @ (a_inv @ e)
        if rest_gain > 0.0:
            # 零空间投影 N = I - J⁺J（J⁺ 用带阻尼的伪逆）
            j_pinv = jac.T @ a_inv
            dq = dq + (I5 - j_pinv @ jac) @ (rest_gain * (rest - q))

        biggest = float(np.max(np.abs(dq))) if dq.size else 0.0
        if biggest > step_limit:
            dq = dq * (step_limit / biggest)

        q_try, clipped = clip_to_limits(q + dq, limits, limit_margin)
        p_try = model.fk_position(q_try)
        err_try = float(np.linalg.norm(target - p_try))

        if err_try < err - 1e-15:
            # 接受：误差下降，减小阻尼（趋向牛顿步，收敛快）
            q, p, err = q_try, p_try, err_try
            hit_limit = hit_limit or clipped
            lam = max(lam * 0.6, damping_min)
            if err < best_err:
                best_q, best_err = q.copy(), err
            if err < tol:
                return IkSolution(q, err, it, True, hit_limit=hit_limit)
            stalls = 0
        else:
            # 拒绝：加大阻尼重试（更稳的梯度方向）；到上限还不行就收工
            stalls += 1
            lam = min(lam * 3.0, damping_max)
            if lam >= damping_max and stalls >= 3:
                reason = "limit" if hit_limit else "unreachable"
                return IkSolution(best_q, best_err, it, False, reason=reason, hit_limit=hit_limit)

    accept = max(tol * 5.0, 5e-4)  # 迭代用尽时，0.5mm 以内也算可用解
    reason = "" if best_err < accept else ("limit" if hit_limit else "unreachable")
    return IkSolution(best_q, best_err, max_iter, best_err < accept, reason=reason, hit_limit=hit_limit)


def solve_linear_move(
    model: UrdfModel,
    q_now: Sequence[float],
    direction: Sequence[float] | np.ndarray,
    distance: float,
    *,
    tol: float = 1e-4,
    max_backtrack: int = 6,
    **ik_kwargs,
) -> tuple[IkSolution, float, str]:
    """沿指定方向"走直线"：从当前位姿出发，尽量走出 ``distance`` 米。

    遇到够不着的情况时，**保留尽可能多的原运动方向**——把步长逐次减半
    （``max_backtrack`` 次）直到 IK 有解，相当于让末端沿原方向一直走到
    可达边界处停下，而不是原地报错或拐弯。

    返回
    ----
    (ik_solution, 实际走出的距离, 说明文字)
    """
    q_now = np.asarray(q_now, dtype=float).reshape(-1)
    d = np.asarray(direction, dtype=float).reshape(3)
    norm = float(np.linalg.norm(d))
    if norm < 1e-12:
        raise ValueError("direction 不能是零向量")
    d = d / norm

    p_now = model.fk_position(q_now)
    best: IkSolution | None = None
    step = float(distance)

    for attempt in range(max_backtrack + 1):
        target = p_now + d * step
        sol = ik_dls(model, target, q_now, tol=tol, **ik_kwargs)
        if sol.ok:
            if attempt == 0:
                note = "整步到达"
            elif attempt == 1:
                note = "步长减半"
            else:
                note = f"步长缩到 1/{2**attempt}"
            return sol, step, note
        best = sol
        step *= 0.5

    return best if best is not None else ik_dls(model, p_now, q_now, **ik_kwargs), 0.0, "blocked"


# --------------------------------------------------------------------------- #
#  3) 讲解辅助：用两连杆解对照 SO101 的肩-肘平面
# --------------------------------------------------------------------------- #


def explain_2link_demo(l1: float = 0.1126, l2: float = 0.1349, x: float = 0.15, y: float = 0.05) -> str:
    """生成一段可直接在终端打印/讲解的两连杆 IK 演示文本。"""
    lines = [f"两连杆 IK 演示：L1={l1:.4f} L2={l2:.4f} 目标=({x:.3f},{y:.3f})"]
    r = math.hypot(x, y)
    lines.append(f"  r = {r:.4f}，可达条件 |L1-L2|<=r<=L1+L2 ")
    lines.append(f"      即 {abs(l1 - l2):.4f} <= {r:.4f} <= {l1 + l2:.4f} -> {'有解' if abs(l1 - l2) <= r <= l1 + l2 else '无解'}")
    sols = ik_planar_2r(l1, l2, x, y)
    if sols is None:
        lines.append("  IK 无解：目标点超出了 1+2 连杆能覆盖的圆环区域")
        return "\n".join(lines)
    for i, s in enumerate(sols, 1):
        px, py = fk_planar_2r(l1, l2, s.theta1, s.theta2)
        tag = "肘上" if s.elbow_up else "肘下"
        lines.append(
            f"  解{i}({tag}): θ1={math.degrees(s.theta1):8.3f}°  θ2={math.degrees(s.theta2):8.3f}°  "
            f"-> 正解回代 = ({px:.4f}, {py:.4f})  误差={math.hypot(px - x, py - y):.2e}"
        )
    lines.append("  说明：cosθ 反解天然带来 ± 两组解，这就是 IK 多解性")
    return "\n".join(lines)
