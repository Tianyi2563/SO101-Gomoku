"""Generate/check a sampled model trajectory from JSON. Never sends commands."""
import sys
from pathlib import Path
import argparse
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from so101_gomoku.recording.storage import read_json,atomic_json
from so101_gomoku.robot.kinematics.ikpy_kinematics import IKPyKinematics,SO101_JOINTS
from so101_gomoku.robot.kinematics.adapter import IKAdapter
from so101_gomoku.robot.planning.pick_place_plan import Waypoint
from so101_gomoku.robot.planning.trajectory import generate_trajectory


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--request',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',type=Path,default=ROOT/'assets/robot/so101/so101_new_calib.urdf')
    args=p.parse_args();data=read_json(args.request)
    if data.get('schema_version')!=1 or data.get('mode')!='OFFLINE_MODEL_ONLY':
        raise ValueError('Request must explicitly be schema1 OFFLINE_MODEL_ONLY')
    solver=IKPyKinematics(args.model,orientation_mode=data.get('orientation_mode','all'))
    ik=IKAdapter(solver,SO101_JOINTS,solver.limits_deg,
        data['position_tolerance_m'],data['orientation_tolerance_rad'],solver.orientation_mode)
    waypoints=[Waypoint(item['stage'],np.asarray(item['pose'],float),item.get('gripper')) for item in data['waypoints']]
    trajectory=generate_trajectory(ik,data['current_deg'],waypoints,**data['constraints'])
    result=trajectory.to_dict();result['request']=data;result['limits_source']='official_URDF_not_measured_hardware'
    atomic_json(args.output,result)
    print(trajectory.report)


if __name__=='__main__':main()
