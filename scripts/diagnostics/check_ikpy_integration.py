"""Exercise project IK adapter on official SO101 URDF; no hardware."""
import sys
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from so101_gomoku.robot.kinematics.ikpy_kinematics import IKPyKinematics, SO101_JOINTS
from so101_gomoku.robot.kinematics.adapter import IKAdapter


def run():
    model = ROOT / 'assets/robot/so101/so101_new_calib.urdf'
    rng = np.random.default_rng(101)
    cases = []
    for mode in ('all', 'Z'):
        solver = IKPyKinematics(model, orientation_mode=mode)
        adapter = IKAdapter(solver, SO101_JOINTS, solver.limits_deg, 1e-4, 1e-3, mode)
        for i in range(20):
            q = rng.uniform(solver.limits_deg[:, 0] * .6, solver.limits_deg[:, 1] * .6)
            target = solver.forward_kinematics(q)
            result = adapter.solve(q + .3, target)
            actual = solver.forward_kinematics(list(result.values()))
            if mode == 'all':
                angle = np.arccos(np.clip((np.trace(target[:3, :3].T @ actual[:3, :3]) - 1) / 2, -1, 1))
            else:
                angle = np.arccos(np.clip(np.dot(target[:3, 2], actual[:3, 2]), -1, 1))
            cases.append({'mode': mode, 'case': i, 'position_residual_m': float(np.linalg.norm(actual[:3, 3]-target[:3, 3])),
                          'orientation_residual_rad': float(angle), 'joints_deg': result})
        unreachable = np.eye(4)
        unreachable[:3, 3] = [10., 10., 10.]
        try:
            adapter.solve(np.zeros(5), unreachable)
        except RuntimeError:
            pass
        else:
            raise AssertionError('Unreachable target incorrectly accepted')
    report = {'mode': 'OFFLINE_MODEL_ONLY', 'hardware_connected': False, 'cases_passed': len(cases),
              'limits_source': 'URDF, NOT measured hardware', 'unreachable_rejected_modes': ['all', 'Z'],
              'position_tolerance_m': 1e-4, 'orientation_tolerance_rad': 1e-3,
              'limits': ['Reachable targets generated from same model', 'Near-target initial guesses',
                         'No board reachability, collision, hardware accuracy or global convergence validation'],
              'cases': cases}
    path = ROOT / 'data/ikpy_integration_report.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f"{len(cases)} project-adapter cases passed; unreachable targets rejected in both modes.")
    print(f"Max position residual: {max(c['position_residual_m'] for c in cases):.3g} m")
    print(f"Max constrained orientation residual: {max(c['orientation_residual_rad'] for c in cases):.3g} rad")
    return report


if __name__ == '__main__':
    run()
