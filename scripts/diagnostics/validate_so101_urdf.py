"""Offline validation of the official SO101 model. Never connects hardware.

FK/IK round trips use model-generated reachable targets, not board coordinates.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ikpy.chain import Chain

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / 'assets/robot/so101/so101_new_calib.urdf'
ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def direct_fk(joints, angles):
    """Independent URDF transform multiplication (SciPy rotation, no IKPy FK)."""
    t = np.eye(4)
    for joint, angle in zip(joints, angles):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        step = np.eye(4)
        step[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        step[:3, 3] = xyz
        t = t @ step
        if joint.get('type') != 'fixed':
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            step = np.eye(4)
            step[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            t = t @ step
    return t


def run():
    xml = ET.parse(MODEL).getroot()
    joints = {j.get('name'): j for j in xml.findall('joint')}
    links = {link.get('name') for link in xml.findall('link')}
    children = [j.find('child').get('link') for j in joints.values()]
    assert len(children) == len(set(children)), 'Multiple parents'
    roots = links - set(children)
    assert roots == {'base_link'}, roots
    for j in joints.values():
        assert j.find('parent').get('link') in links
        assert j.find('child').get('link') in links
    by_child = {j.find('child').get('link'): j for j in joints.values()}
    for link in links:
        seen = set()
        while link != 'base_link':
            assert link not in seen, 'Cycle in URDF'
            seen.add(link)
            link = by_child[link].find('parent').get('link')
    ordered = [joints[name] for name in ARM + ['gripper_frame_joint']]
    parent = 'base_link'
    for j in ordered:
        assert j.find('parent').get('link') == parent
        parent = j.find('child').get('link')
    assert parent == 'gripper_frame_link'
    chain = Chain.from_urdf_file(str(MODEL), base='base_link', tip='gripper_frame_link')
    assert [link.name for link in chain.links[1:-1]] == ARM
    assert chain.links[-1].joint_type == 'fixed'
    assert 'gripper' not in [link.name for link in chain.links]
    bounds = np.array([chain.links[i].bounds for i in range(1, 6)])
    assert np.isfinite(bounds).all() and (bounds[:, 0] < bounds[:, 1]).all()
    meshes = sorted({m.get('filename') for m in xml.findall('.//mesh')})
    missing = [m for m in meshes if not (MODEL.parent / m).is_file()]
    assert not missing, f'Missing meshes: {missing}'
    rng = np.random.default_rng(101)
    # Interior joint configurations generated only for mathematical consistency checks.
    samples = [np.zeros(5)] + [rng.uniform(bounds[:, 0] * .6, bounds[:, 1] * .6) for _ in range(19)]
    results = []
    for sample in samples:
        full = np.r_[0., sample, 0.]
        target = chain.forward_kinematics(full)
        independent = direct_fk(ordered, np.r_[sample, 0.])
        fk_error = float(np.max(np.abs(target - independent)))
        assert fk_error < 1e-8, fk_error
        seed = full.copy()
        seed[1:6] = np.clip(sample + rng.uniform(-.05, .05, 5), bounds[:, 0], bounds[:, 1])
        solved = chain.inverse_kinematics_frame(target, initial_position=[False, *seed[1:6], False], orientation_mode='all')
        actual = chain.forward_kinematics(solved)
        position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
        angle_error = float(Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude())
        in_limits = bool(np.all(solved[1:6] >= bounds[:, 0] - 1e-8) and np.all(solved[1:6] <= bounds[:, 1] + 1e-8))
        passed = position_error < 1e-5 and angle_error < 1e-4 and in_limits
        results.append({'target_joints_deg': np.rad2deg(sample).tolist(), 'position_error_m': position_error,
                        'orientation_error_rad': angle_error, 'independent_fk_max_error': fk_error,
                        'in_urdf_limits': in_limits, 'passed': passed})
    # Show why returned angles alone cannot certify reachability.
    far = chain.forward_kinematics(np.zeros(len(chain.links)))
    far[:3, 3] = [10, 10, 10]
    q = chain.inverse_kinematics_frame(far, initial_position=[False, 0., 0., 0., 0., 0., False], orientation_mode='all')
    unreachable_residual = float(np.linalg.norm(chain.forward_kinematics(q)[:3, 3] - far[:3, 3]))
    report = {'mode': 'OFFLINE_MODEL_ONLY', 'hardware_connected': False,
              'model': str(MODEL.relative_to(ROOT)), 'root': 'base_link', 'tip': 'gripper_frame_link',
              'joint_limits_deg': {name: np.rad2deg(bound).tolist() for name, bound in zip(ARM, bounds)},
              'mesh_count': len(meshes), 'missing_meshes': missing,
              'zero_pose': chain.forward_kinematics(np.zeros(len(chain.links))).tolist(),
              'cases': results, 'passed_cases': sum(case['passed'] for case in results),
              'unreachable_position_residual_m': unreachable_residual,
              'limitations': ['URDF limits are not measured motor limits', 'No collisions checked',
                             'No board-to-base calibration or real board reachability tested',
                             'TCP and joint zero/sign must match physical arm',
                             'Near-target initial guesses; not a global IK convergence benchmark']}
    output = ROOT / 'data/so101_urdf_validation.json'
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f"Meshes: {len(meshes)}, FK/IK: {report['passed_cases']}/{len(results)}")
    print(f"Max position residual: {max(r['position_error_m'] for r in results):.3g} m")
    print(f"Max rotation residual: {max(r['orientation_error_rad'] for r in results):.3g} rad")
    print(f"Unreachable target residual: {unreachable_residual:.3f} m")
    assert all(r['passed'] for r in results), 'FK/IK round trip failed'
    assert unreachable_residual > 1, 'Unexpected unreachable check'
    return report


if __name__ == '__main__':
    run()
