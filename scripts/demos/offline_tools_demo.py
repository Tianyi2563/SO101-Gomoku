"""Synthetic calibration, model trajectory, durable session and crash recovery."""
from pathlib import Path
import sys
import argparse
from datetime import datetime
from uuid import uuid4
import numpy as np
import cv2
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))
from so101_gomoku.calibration.workflow import point_template,fit_dataset,save_calibration,load_calibration
from so101_gomoku.recording.storage import atomic_json
from so101_gomoku.recording.session import SessionJournal,replay_session
from so101_gomoku.recording.replay_html import export_replay_html
from so101_gomoku.robot.kinematics.ikpy_kinematics import IKPyKinematics,SO101_JOINTS
from so101_gomoku.robot.kinematics.adapter import IKAdapter
from so101_gomoku.robot.planning.pick_place_plan import Waypoint
from so101_gomoku.robot.planning.trajectory import generate_trajectory
from so101_gomoku.orchestration.controller import GameController
from so101_gomoku.orchestration.simulation import SimulatedTable
from so101_gomoku.ai.alpha_beta import AlphaBetaEngine
from so101_gomoku.contracts.models import Move,Cell


class IllustratedTable(SimulatedTable):
    """Images rendered from simulated state; NOT camera recognition."""
    journal=None
    def observe(self):
        obs=super().observe()
        n=self.board.size;spacing=48;margin=30
        image=np.full((2*margin+(n-1)*spacing+1,)*2+(3,),(110,170,210),np.uint8)
        for i in range(n):
            a=margin+i*spacing;b=margin+(n-1)*spacing
            cv2.line(image,(margin,a),(b,a),(40,40,40),1);cv2.line(image,(a,margin),(a,b),(40,40,40),1)
        for r,row in enumerate(self.board.cells):
            for c,value in enumerate(row):
                if value:
                    v=20 if value==1 else 245
                    cv2.circle(image,(margin+c*spacing,margin+r*spacing),15,(v,v,v),-1)
        if self.journal:self.journal.record_image(obs.frame_id,image)
        return obs


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path)
    args=p.parse_args()
    directory=args.output_dir or ROOT/'data/offline-tools'/f'{datetime.now():%Y%m%d-%H%M%S}-{str(uuid4())[:6]}'
    directory.mkdir(parents=True,exist_ok=True)
    dataset=point_template(measurement_source='synthetic')
    for point in dataset['points']:
        point['base_xyz_m']=[.1+point['col']*.021,-.2+point['row']*.021,.04]
    atomic_json(directory/'SYNTHETIC_points.json',dataset)
    calibration=fit_dataset(dataset,calibration_id='SYNTHETIC-board-only')
    save_calibration(directory/'SYNTHETIC_calibration.json',calibration)
    load_calibration(directory/'SYNTHETIC_calibration.json')
    solver=IKPyKinematics(ROOT/'assets/robot/so101/so101_new_calib.urdf')
    ik=IKAdapter(solver,SO101_JOINTS,solver.limits_deg,1e-4,1e-3)
    target=solver.forward_kinematics([.5,0,0,0,0])
    atomic_json(directory/'MODEL_ONLY_trajectory_request.json',{
        'schema_version':1,'mode':'OFFLINE_MODEL_ONLY','orientation_mode':'all','current_deg':[0]*5,
        'position_tolerance_m':1e-4,'orientation_tolerance_rad':1e-3,
        'waypoints':[{'stage':'model_target','pose':target.tolist()}],
        'constraints':{'max_linear_speed_m_s':.02,'max_angular_speed_rad_s':.2,
                       'joint_speed_deg_s':5.,'joint_accel_deg_s2':10.}})
    trajectory=generate_trajectory(ik,[0]*5,[Waypoint('model_target',target)],
        max_linear_speed_m_s=.02,max_angular_speed_rad_s=.2,joint_speed_deg_s=5.,joint_accel_deg_s2=10.)
    atomic_json(directory/'MODEL_ONLY_trajectory.json',trajectory.to_dict())
    table=IllustratedTable(calibration_id='SYNTHETIC-board-only');engine=AlphaBetaEngine(max_depth=1)
    session_dir=directory/'SYNTHETIC_session'
    with SessionJournal(session_dir,configuration={'mode':'SIMULATED','spacing_m':.021,'spacing_verified':False}) as journal:
        table.journal=journal
        controller=GameController(table,engine,table,table.calibration_id,clock=table.clock,journal=journal,search_time_s=.03)
        controller.start();controller.run_turn()
        table.human_move(Move(9,9,Cell.WHITE))
        execute=table.execute
        class Crash(BaseException):pass
        def uncertain_execute(task):execute(task);raise Crash('simulated exit after placement')
        table.execute=uncertain_execute
        try:controller.run_turn()
        except Crash:pass
    task_count=len(table.tasks)
    with SessionJournal(session_dir) as journal:
        table.journal=journal
        restored=GameController.resume_from_journal(journal,table,engine,table,table.calibration_id,clock=table.clock)
        phase_before=restored.phase.value
        restored.recover(robot_stopped=True)
        assert len(table.tasks)==task_count and restored.state.version==3
    replay=replay_session(session_dir)
    export_replay_html(session_dir,directory/'replay.html')
    summary={'mode':'OFFLINE_SYNTHETIC','hardware_connected':False,'calibration_source':'synthetic',
             'validation_max_mm':calibration['report']['validation']['max_mm'],
             'trajectory_report':trajectory.report,'restart_initial_phase':phase_before,
             'phase_after_reconciliation':restored.phase.value,'game_version':restored.state.version,
             'robot_tasks_before_restart':task_count,'robot_tasks_after_restart':len(table.tasks),
             'journal_records':len(replay['records']),'images':len(list((session_dir/'images').glob('*.png')))}
    atomic_json(directory/'summary.json',summary)
    print('OFFLINE synthetic examples saved to:',directory)
    print('calibration, model trajectory, journal + images, replay.html; restart did not reexecute a task')


if __name__=='__main__':main()
