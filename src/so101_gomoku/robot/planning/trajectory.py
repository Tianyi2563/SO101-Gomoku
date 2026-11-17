"""Offline sampled SE(3) trajectories with sequential IK and rate checks.

No hardware execution and no mesh/swept collision certification.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from so101_gomoku.calibration.transforms import validate_transform


@dataclass(frozen=True)
class TrajectorySample:
    time_s: float
    pose: np.ndarray
    joints_deg: dict
    stage: str
    gripper: str | None = None
    fk_pose: np.ndarray | None = None


@dataclass(frozen=True)
class Trajectory:
    samples: tuple[TrajectorySample,...]
    report: dict

    def to_dict(self):
        return {'schema_version':1,'mode':'OFFLINE_MODEL_ONLY','report':self.report,
                'samples':[{'time_s':s.time_s,'pose':s.pose.tolist(),'joints_deg':s.joints_deg,
                            'stage':s.stage,'gripper':s.gripper,
                            'fk_pose':(s.fk_pose if s.fk_pose is not None else s.pose).tolist()} for s in self.samples]}


def interpolate_pose(start, end, fraction):
    a,b=validate_transform(start),validate_transform(end)
    if not np.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError('Interpolation fraction must be in [0,1]')
    pose=np.eye(4)
    pose[:3,3]=a[:3,3]+fraction*(b[:3,3]-a[:3,3])
    pose[:3,:3]=Slerp([0.,1.],Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]])))(fraction).as_matrix()
    return pose


def _rates(times, q):
    dt=np.diff(times)
    if np.any(dt<=0):raise ValueError('Sample times must strictly increase')
    velocity=np.diff(q,axis=0)/dt[:,None]
    mids=(times[:-1]+times[1:])/2
    vt=np.r_[times[0],mids,times[-1]]
    vv=np.vstack([np.zeros(q.shape[1]),velocity,np.zeros(q.shape[1])])
    acceleration=np.diff(vv,axis=0)/np.diff(vt)[:,None]
    return np.max(np.abs(velocity),axis=0),np.max(np.abs(acceleration),axis=0)


def generate_trajectory(ik, current_deg, waypoints, *, max_linear_speed_m_s,
                        max_angular_speed_rad_s, joint_speed_deg_s, joint_accel_deg_s2,
                        sample_period_s=.02, max_cart_step_m=.003, max_angle_step_rad=.03,
                        max_joint_step_deg=5., workspace_min=None, workspace_max=None,
                        max_samples=10000):
    values=[max_linear_speed_m_s,max_angular_speed_rad_s,sample_period_s,max_cart_step_m,
            max_angle_step_rad,max_joint_step_deg]
    if not all(np.isfinite(v) and v>0 for v in values) or type(max_samples) is not int or max_samples<2:
        raise ValueError('Positive finite trajectory parameters required')
    names=ik.joint_names
    q=np.asarray(current_deg,float)
    speed=np.broadcast_to(np.asarray(joint_speed_deg_s,float),(len(names),))
    accel=np.broadcast_to(np.asarray(joint_accel_deg_s2,float),(len(names),))
    if not np.isfinite(speed).all() or not np.isfinite(accel).all() or np.any(speed<=0) or np.any(accel<=0):
        raise ValueError('Positive finite per-joint speed/acceleration required')
    if q.shape!=(len(names),) or not np.isfinite(q).all() or np.any(q<ik.limits[:,0]) or np.any(q>ik.limits[:,1]):
        raise ValueError('Invalid initial joints')
    if not waypoints:raise ValueError('Nonempty waypoint list required')
    check_workspace=workspace_min is not None or workspace_max is not None
    if check_workspace:
        low,high=np.asarray(workspace_min,float),np.asarray(workspace_max,float)
        if low.shape!=(3,) or high.shape!=(3,) or not np.isfinite([low,high]).all() or np.any(low>=high):
            raise ValueError('Matching finite workspace bounds required')
    start=validate_transform(ik.solver.forward_kinematics(q))
    samples=[TrajectorySample(0.,start.copy(),dict(zip(names,map(float,q))),'start')]
    if check_workspace and (np.any(start[:3,3]<low) or np.any(start[:3,3]>high)):
        raise ValueError('Initial TCP outside workspace')
    t=0.;max_position_error=0.;max_orientation_error=0.
    for waypoint in waypoints:
        end=validate_transform(waypoint.pose)
        if waypoint.gripper not in (None,'open','grasp'):raise ValueError('Unknown gripper action')
        distance=float(np.linalg.norm(end[:3,3]-start[:3,3]))
        angle=float(Rotation.from_matrix(start[:3,:3].T@end[:3,:3]).magnitude())
        duration=max(sample_period_s,1.875*distance/max_linear_speed_m_s,1.875*angle/max_angular_speed_rad_s)
        n=max(1,int(np.ceil(duration/sample_period_s)),int(np.ceil(1.875*distance/max_cart_step_m)),
              int(np.ceil(1.875*angle/max_angle_step_rad)))
        if len(samples)+n>max_samples:raise ValueError('Trajectory exceeds sample budget')
        for i in range(1,n+1):
            u=i/n;fraction=10*u**3-15*u**4+6*u**5
            pose=interpolate_pose(start,end,fraction)
            if check_workspace and (np.any(pose[:3,3]<low) or np.any(pose[:3,3]>high)):
                raise ValueError(f'TCP outside workspace at {waypoint.stage}')
            result=ik.solve(q,pose)
            next_q=np.array([result[name] for name in names])
            if np.any(np.abs(next_q-q)>max_joint_step_deg):
                raise ValueError(f'IK branch jump at {waypoint.stage}')
            actual=validate_transform(ik.solver.forward_kinematics(next_q))
            if check_workspace and (np.any(actual[:3,3]<low) or np.any(actual[:3,3]>high)):
                raise ValueError(f'FK TCP outside workspace at {waypoint.stage}')
            max_position_error=max(max_position_error,float(np.linalg.norm(actual[:3,3]-pose[:3,3])))
            if ik.orientation_mode=='all':
                error=Rotation.from_matrix(pose[:3,:3].T@actual[:3,:3]).magnitude()
            else:
                axis='XYZ'.index(ik.orientation_mode)
                error=np.arccos(np.clip(np.dot(pose[:3,axis],actual[:3,axis]),-1,1))
            max_orientation_error=max(max_orientation_error,float(error))
            samples.append(TrajectorySample(t+duration*i/n,pose,result,waypoint.stage,waypoint.gripper if i==n else None,actual.copy()))
            q=next_q
        t+=duration;start=end.copy()
    times=np.array([s.time_s for s in samples]);joints=np.array([[s.joints_deg[name] for name in names] for s in samples])
    velocity,acceleration=_rates(times,joints)
    actual_poses=np.array([s.fk_pose if s.fk_pose is not None else s.pose for s in samples])
    linear_rates=np.linalg.norm(np.diff(actual_poses[:,:3,3],axis=0),axis=1)/np.diff(times)
    angular_rates=np.array([Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()
                            for a,b in zip(actual_poses[:-1],actual_poses[1:])])/np.diff(times)
    scale=max(1.,float(np.max(velocity/speed)),float(np.sqrt(np.max(acceleration/accel))),
              float(linear_rates.max()/max_linear_speed_m_s),float(angular_rates.max()/max_angular_speed_rad_s))*1.000001
    samples=tuple(TrajectorySample(s.time_s*scale,s.pose,s.joints_deg,s.stage,s.gripper,s.fk_pose) for s in samples)
    velocity,acceleration=_rates(times*scale,joints)
    if np.any(velocity>speed*(1+1e-9)) or np.any(acceleration>accel*(1+1e-9)):
        raise RuntimeError('Rate checking failed after time scaling')
    report={'sample_count':len(samples),'duration_s':samples[-1].time_s,'time_scale':scale,
            'max_joint_velocity_deg_s':dict(zip(names,map(float,velocity))),
            'max_joint_acceleration_deg_s2':dict(zip(names,map(float,acceleration))),
            'max_position_residual_m':max_position_error,'max_constrained_orientation_residual_rad':max_orientation_error,
            'max_fk_linear_speed_m_s':float(linear_rates.max()/scale),
            'max_fk_angular_speed_rad_s':float(angular_rates.max()/scale),
            'nominal_sample_period_s':sample_period_s,'max_final_sample_period_s':float(np.diff(times*scale).max()),
            'orientation_mode':ik.orientation_mode,'workspace_checked':check_workspace,
            'collision_status':'NOT_CHECKED','hardware_execution':False,
            'limitations':['Sampled model checks, not continuous collision proof',
                          'Joint rates checked by finite differences, endpoints assumed stopped',
                          'No servo dynamics, load, jerk or real tracking verification']}
    return Trajectory(samples,report)
