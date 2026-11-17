import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from so101_gomoku.robot.kinematics.adapter import IKAdapter
from so101_gomoku.robot.planning.pick_place_plan import Waypoint
from so101_gomoku.robot.planning.trajectory import generate_trajectory,interpolate_pose
from so101_gomoku.robot.kinematics.ikpy_kinematics import IKPyKinematics,SO101_JOINTS


class LinearSolver:
    joint_names=('x','y','z')
    def inverse_kinematics(self,current,pose):return pose[:3,3]*100
    def forward_kinematics(self,q):
        p=np.eye(4);p[:3,3]=np.asarray(q)/100;return p


class TrajectoryTests(unittest.TestCase):
    def adapter(self):return IKAdapter(LinearSolver(),('x','y','z'),[[-100,100]]*3,1e-7,1e-6)
    def kwargs(self):return dict(max_linear_speed_m_s=.1,max_angular_speed_rad_s=1.,joint_speed_deg_s=2.,joint_accel_deg_s2=5.)

    def test_smooth_rate_limits_actions_and_endpoint(self):
        end=np.eye(4);end[:3,3]=[.1,.02,0]
        result=generate_trajectory(self.adapter(),[0,0,0],[Waypoint('place',end,'open')],**self.kwargs())
        self.assertGreater(result.report['sample_count'],2)
        self.assertEqual(result.samples[-1].gripper,'open')
        self.assertTrue(all(s.gripper is None for s in result.samples[:-1]))
        np.testing.assert_allclose(result.samples[-1].pose,end)
        self.assertLessEqual(max(result.report['max_joint_velocity_deg_s'].values()),2.)
        self.assertLessEqual(max(result.report['max_joint_acceleration_deg_s2'].values()),5.)
        self.assertEqual(result.report['collision_status'],'NOT_CHECKED')

    def test_slerp_and_workspace_and_step_budget(self):
        end=np.eye(4);end[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
        angle=Rotation.from_matrix(interpolate_pose(np.eye(4),end,.5)[:3,:3]).magnitude()
        self.assertAlmostEqual(angle,np.pi/4)
        end=np.eye(4);end[0,3]=.2
        with self.assertRaises(ValueError):
            generate_trajectory(self.adapter(),[0,0,0],[Waypoint('bad',end)],workspace_min=[-.1]*3,workspace_max=[.1]*3,**self.kwargs())
        with self.assertRaises(ValueError):
            generate_trajectory(self.adapter(),[0,0,0],[Waypoint('bad',end)],max_samples=2,**self.kwargs())
        with self.assertRaisesRegex(ValueError,'branch jump'):
            generate_trajectory(self.adapter(),[0,0,0],[Waypoint('jump',end)],max_joint_step_deg=1e-6,**self.kwargs())

    def test_real_so101_short_model_path(self):
        model=Path(__file__).resolve().parents[1]/'assets/robot/so101/so101_new_calib.urdf'
        solver=IKPyKinematics(model)
        ik=IKAdapter(solver,SO101_JOINTS,solver.limits_deg,1e-4,1e-3)
        end=solver.forward_kinematics([.5,0,0,0,0])
        result=generate_trajectory(ik,[0]*5,[Waypoint('model_target',end)],**self.kwargs())
        self.assertLess(result.report['max_position_residual_m'],1e-4)
        self.assertFalse(result.report['hardware_execution'])
