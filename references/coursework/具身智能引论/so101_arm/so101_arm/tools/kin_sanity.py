"""运动学与轨迹的自检脚本（不需要机械臂，纯 numpy 就能跑）。

用法：
    python tools/kin_sanity.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from so101.frames import JointMap, describe  # noqa: E402
from so101.kinematics import (  # noqa: E402
    explain_2link_demo,
    fk_planar_2r,
    ik_dls,
    ik_planar_2r,
    solve_linear_move,
)
from so101.trajectory import get_profile  # noqa: E402
from so101.urdf_model import default_model  # noqa: E402

FAIL = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global FAIL
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        FAIL += 1


def main() -> int:
    print("=" * 78)
    print("1) 两连杆解析 IK（讲义公式）")
    print("=" * 78)
    print(explain_2link_demo())
    for (x, y) in [(0.15, 0.05), (0.2, -0.1), (0.21, 0.0)]:
        sols = ik_planar_2r(0.1126, 0.1349, x, y)
        assert sols is not None
        for s in sols:
            px, py = fk_planar_2r(0.1126, 0.1349, s.theta1, s.theta2)
            check(
                f"解析解回代 ({x},{y})",
                math.hypot(px - x, py - y) < 1e-12,
                f"err={math.hypot(px - x, py - y):.2e}",
            )
    check("够不着时返回 None", ik_planar_2r(0.1126, 0.1349, 0.5, 0.0) is None)

    print()
    print("=" * 78)
    print("2) SO101 单位映射表")
    print("=" * 78)
    model = default_model()
    jmap = JointMap.from_model(model)
    print(describe(jmap))

    print()
    print("=" * 78)
    print("3) 数值 IK（DLS）精度与限位")
    print("=" * 78)
    q0 = model.zero_pose()
    p0 = model.fk_position(q0)
    print("零位末端 =", np.round(p0, 4))

    rng = np.random.default_rng(1)
    ok_count = 0
    trials = 300
    errs = []
    for _ in range(trials):
        # 在零位附近采样可达目标
        dq = rng.normal(0, 0.25, size=model.n_dof)
        q_ref = np.clip(
            q0 + dq,
            [j.lower for j in model.movable_joints],
            [j.upper for j in model.movable_joints],
        )
        target = model.fk_position(q_ref)
        sol = ik_dls(model, target, q0)
        if sol.ok:
            ok_count += 1
            errs.append(math.dist(model.fk_position(sol.q), target))
    check(
        f"随机可达目标求解成功率 {ok_count}/{trials}",
        ok_count >= trials * 0.9,
        f"最大残差={max(errs) if errs else float('nan'):.2e} m",
    )

    far = p0 + np.array([3.0, 0.0, 0.0])
    sol_far = ik_dls(model, far, q0)
    check("超出工作空间时 ok=False", not sol_far.ok, f"reason={sol_far.reason} err={sol_far.error:.3f}m")

    print()
    print("=" * 78)
    print("4) 直线运动 + 无解回溯（验收 1.2 的行为）")
    print("=" * 78)
    q = q0.copy()
    up = np.array([0.0, 0.0, 1.0])
    print(f"{'step':>4s} {'可行距离(mm)':>12s} {'残差(mm)':>10s}  说明")
    for i in range(12):
        sol, dist, note = solve_linear_move(model, q, up, 0.05, tol=1e-4, max_backtrack=8)
        p_before = model.fk_position(q)
        if dist > 1e-9:
            q = sol.q
        p_after = model.fk_position(q)
        moved = float(np.dot(p_after - p_before, up)) * 1000
        print(f"{i:>4d} {moved:12.2f} {sol.error * 1000:10.3f}  {note}")
        if dist <= 1e-9:
            break
    check("向上运动确实抬高末端", model.fk_position(q)[2] > p0[2], f"z: {p0[2]:.4f} -> {model.fk_position(q)[2]:.4f}")

    print()
    print("=" * 78)
    print("5) 轨迹插值（1s 内切换到目标姿态）")
    print("=" * 78)
    prof = get_profile("minjerk")
    check("minjerk(0)=0", abs(prof(0.0)) < 1e-12)
    check("minjerk(1)=1", abs(prof(1.0) - 1.0) < 1e-12)
    check("minjerk(0.5)=0.5", abs(prof(0.5) - 0.5) < 1e-12)

    print()
    print("=" * 78)
    summary = "全部通过" if FAIL == 0 else f"{FAIL} 项失败"
    print(f"结果：{summary}")
    print("=" * 78)
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
