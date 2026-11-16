"""打印 SO101 URDF 的关节链、限位与零位，并做一次 FK 自检。

用法：
    python tools/inspect_urdf.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.urdf_model import default_model  # noqa: E402


def main() -> None:
    model = default_model()
    print(f"URDF : {model.path}")
    print(f"robot: {model.robot_name}")
    print(f"tip  : {model.tip_link}   DOF={model.n_dof}")
    print()
    print(f"{'joint':12s} {'type':9s} {'parent -> child':34s} {'zero(deg)':>10s} {'range(deg)':>18s}  axis")
    print("-" * 110)
    for j in model.chain:
        lo, hi = j.range_around_zero_deg
        rng = f"[{lo:7.2f},{hi:7.2f}]" if j.movable else "       -"
        print(
            f"{j.name:12s} {j.type:9s} {j.parent + ' -> ' + j.child:34s} "
            f"{math.degrees(j.zero):10.3f} {rng:>18s}  "
            f"{np.round(j.axis, 3) if j.movable else '-'}"
        )

    print()
    print("--- FK 自检：零位姿态 ---")
    q0 = model.zero_pose()
    t0 = model.fk(q0)
    print("q0(deg) =", np.round(np.degrees(q0 - q0 + 0), 3), "(全部为标定零点)")
    print("末端位置 =", np.round(t0[:3, 3], 5), "m")

    print()
    print("--- FK 自检：末端位置对关节角的敏感度（数值雅可比 vs 解析雅可比）---")
    eps = 1e-6
    num = np.zeros((3, model.n_dof))
    for i in range(model.n_dof):
        dq = np.zeros(model.n_dof)
        dq[i] = eps
        num[:, i] = (model.fk_position(q0 + dq) - model.fk_position(q0 - dq)) / (2 * eps)
    ana = model.jacobian(q0)[:3]
    print("解析 J (m/rad):")
    with np.printoptions(precision=4, suppress=True):
        print(ana)
    print("最大偏差 =", float(np.max(np.abs(ana - num))))

    print()
    print("--- 工作空间抽样（零位附近随机采样 2000 次，末端 z 高度范围）---")
    rng = np.random.default_rng(0)
    lo = np.asarray([j.range_around_zero[0] for j in model.movable_joints]) + q0
    hi = np.asarray([j.range_around_zero[1] for j in model.movable_joints]) + q0
    pts = []
    for _ in range(2000):
        q = rng.uniform(lo, hi)
        pts.append(model.fk_position(q))
    pts = np.asarray(pts)
    print("x:", np.round(pts[:, 0].min(), 4), "~", np.round(pts[:, 0].max(), 4))
    print("y:", np.round(pts[:, 1].min(), 4), "~", np.round(pts[:, 1].max(), 4))
    print("z:", np.round(pts[:, 2].min(), 4), "~", np.round(pts[:, 2].max(), 4))


if __name__ == "__main__":
    main()
