"""Synchronous turn owner: observe -> decide -> execute -> verify -> commit.

Hardware/vision are injected. Never retries a physical task automatically.
Callbacks must implement their own timeouts; this is not a hardware emergency stop.
"""
from enum import Enum
from math import isfinite
from threading import Lock
import time
from uuid import uuid4
from dataclasses import asdict
from so101_gomoku.contracts.models import Cell, RobotTask
from so101_gomoku.game.board import GameState
from so101_gomoku.game.reconciliation import confirmed_cells, detect_single_move, verify_placement
from so101_gomoku.vision.stabilizer import BoardStabilizer


class Phase(str, Enum):
    BOOT = 'boot'
    WAIT_HUMAN = 'wait_human'
    THINKING = 'thinking'
    EXECUTING = 'executing'
    VERIFYING = 'verifying'
    PAUSED = 'paused'
    FINISHED = 'finished'


class GameController:
    def __init__(self, camera, engine, robot, calibration_id, *, state=None,
                 human_color=Cell.WHITE, stable_frames=2, observation_attempts=8,
                 min_confidence=.9, max_age_s=5., search_time_s=.5, clock=time.time,
                 journal=None):
        if human_color not in (Cell.BLACK, Cell.WHITE) or not calibration_id:
            raise ValueError('Human color and calibration id required')
        if type(stable_frames) is not int or stable_frames < 1 or type(observation_attempts) is not int or observation_attempts < stable_frames:
            raise ValueError('Observation attempts must cover required stable frames')
        if not isfinite(max_age_s) or max_age_s <= 0 or not isfinite(search_time_s) or search_time_s <= 0:
            raise ValueError('Finite positive time limits required')
        if not isfinite(min_confidence) or not 0 <= min_confidence <= 1:
            raise ValueError('Invalid confidence threshold')
        self.camera, self.engine, self.robot = camera, engine, robot
        self.calibration_id, self.human_color = calibration_id, human_color
        self._state = state if state is not None else GameState.empty()
        self.stable_frames, self.observation_attempts = stable_frames, observation_attempts
        self.min_confidence, self.max_age_s, self.search_time_s = min_confidence, max_age_s, search_time_s
        self.clock = clock
        self.phase, self.pause_reason = Phase.BOOT, ''
        self.pending_task = None
        self.last_decision = None
        self.events = []
        self._last_time, self._last_frame = float('-inf'), None
        self._lock = Lock()
        self.journal = journal
        if self.journal is not None:
            self.journal.begin(self.state,self.calibration_id,self.human_color,
                               {'stable_frames':self.stable_frames,'observation_attempts':self.observation_attempts,
                                'min_confidence':self.min_confidence,'max_age_s':self.max_age_s,'search_time_s':self.search_time_s})

    @property
    def state(self):
        return self._state

    def _event(self, name, **details):
        event={'event': name, 'time': self.clock(), 'phase': self.phase.value,
               'game_version': self.state.version, **details}
        if self.journal is not None:
            self.journal.append_event(event,self.state,self.phase.value,self.pending_task)
        self.events.append(event)

    @classmethod
    def resume_from_journal(cls, journal, camera, engine, robot, calibration_id,
                            expected_configuration=None, **options):
        from so101_gomoku.recording.session import state_from_dict, task_from_dict
        saved=journal.checkpoint()
        meta=saved['metadata'];checkpoint=saved['checkpoint']
        if calibration_id!=meta['calibration_id']:
            raise ValueError('Cannot resume under a different calibration id')
        if 'state' in options or 'human_color' in options or 'journal' in options:
            raise ValueError('State/roles/journal come from the saved session')
        if expected_configuration is not None and expected_configuration!=meta['configuration']:
            raise ValueError('Runtime configuration does not match saved session')
        settings=meta.get('controller_settings',{})
        for key,value in settings.items():
            if key in options and options[key]!=value:
                raise ValueError(f'Resume must preserve saved controller setting: {key}')
            options[key]=value
        controller=cls(camera,engine,robot,calibration_id,state=state_from_dict(checkpoint['state']),
                       human_color=Cell(meta['human_color']),**options)
        controller.journal=journal
        controller.pending_task=task_from_dict(checkpoint['pending_task'])
        controller.phase=Phase.PAUSED
        controller.pause_reason='Restart requires robot_stopped confirmation and fresh board reconciliation'
        controller._event('restart_loaded',previous_phase=checkpoint['phase'])
        return controller

    def _pause(self, error):
        self.phase, self.pause_reason = Phase.PAUSED, f'{type(error).__name__}: {error}'
        self._event('paused', reason=self.pause_reason,
                    pending_task_id=self.pending_task.task_id if self.pending_task else None)

    def _next_phase(self):
        if self.state.finished:
            self.phase = Phase.FINISHED
        else:
            self.phase = Phase.WAIT_HUMAN if self.state.next_player == self.human_color else Phase.THINKING

    def _read_stable(self, validate, not_before=None):
        not_before = max(self.clock(), not_before if not_before is not None else float('-inf'))
        stabilizer = BoardStabilizer(self.stable_frames, self.min_confidence)
        for _ in range(self.observation_attempts):
            obs = self.camera.observe()
            now = self.clock()
            self._event('observation',observation=asdict(obs),
                        image=self.journal.take_image_reference(obs.frame_id) if self.journal else None)
            if obs.calibration_id != self.calibration_id or len(obs.cells) != self.state.size:
                raise ValueError('Observation calibration or board size changed')
            if (not isfinite(obs.captured_at) or not obs.frame_id or obs.frame_id == self._last_frame
                    or obs.captured_at <= self._last_time or now-obs.captured_at > self.max_age_s
                    or obs.captured_at-now > .5 or (not_before is not None and obs.captured_at <= not_before)):
                raise ValueError('Observation stale, repeated, out of order or before action completion')
            self._last_time, self._last_frame = obs.captured_at, obs.frame_id
            try:
                confirmed_cells(obs, self.min_confidence)
            except ValueError:
                stabilizer.reset()
                continue
            validate(obs)  # reject confident illegal board changes without committing
            stable = stabilizer.update(obs)
            if stable is not None:
                return stable
        raise TimeoutError('No stable confident observation within attempt budget')

    def _matches_state(self, obs):
        if obs.cells != self.state.cells:
            raise ValueError('Physical board does not match confirmed game')

    def start(self):
        if not self._lock.acquire(blocking=False):
            raise RuntimeError('Controller busy')
        try:
            if self.phase != Phase.BOOT:
                raise RuntimeError('Controller already started; use recover after a pause')
            try:
                self._read_stable(self._matches_state)
                self._next_phase()
                self._event('started')
            except Exception as exc:
                self._pause(exc)
            return self.phase
        finally:
            self._lock.release()

    def run_turn(self):
        if not self._lock.acquire(blocking=False):
            raise RuntimeError('Controller busy')
        try:
            if self.phase not in (Phase.WAIT_HUMAN, Phase.THINKING):
                raise RuntimeError(f'Cannot run turn during {self.phase.value}')
            try:
                if self.phase == Phase.WAIT_HUMAN:
                    def legal_human(obs):
                        if obs.cells != self.state.cells:
                            detect_single_move(self.state, obs, self.min_confidence)
                    obs = self._read_stable(legal_human)
                    if obs.cells == self.state.cells:
                        self._event('waiting_for_human')
                        return self.phase
                    move = detect_single_move(self.state, obs, self.min_confidence)
                    self._state = self.state.play(move)
                    self._next_phase()
                    self._event('human_committed', move=[move.row, move.col, int(move.color)])
                    if self.phase == Phase.FINISHED:
                        self._event('game_finished', winner=int(self.state.winner()) if self.state.winner() else None)
                        return self.phase
                else:
                    # Robot-first or explicit recovery: observe before a new physical task.
                    self._read_stable(self._matches_state)
                self.phase = Phase.THINKING
                decision = self.engine.choose_move(self.state, self.search_time_s)
                self.state.play(decision.move)  # validate without committing
                self.last_decision = decision
                self._read_stable(self._matches_state)  # Board may change during AI computation.
                task = RobotTask(str(uuid4()), decision.move, self.state.version, self.calibration_id)
                self.pending_task = task
                self._event('decision', task_id=task.task_id, engine=decision.engine,
                            reason=decision.reason, move=[task.move.row, task.move.col, int(task.move.color)],
                            decision=asdict(decision))
                if self.state.version != task.expected_game_version:
                    raise RuntimeError('Game version changed before execution')
                self.phase = Phase.EXECUTING
                self._event('execution_started', task_id=task.task_id)
                result = self.robot.execute(task)
                completed_at = self.clock()
                self._event('execution_result', task_id=result.task_id, completed=result.completed,
                            stage=result.stage, reason=result.reason)
                if result.task_id != task.task_id or not result.completed:
                    raise RuntimeError(f'Execution failed or task mismatch: {result.stage}: {result.reason}')
                self.phase = Phase.VERIFYING
                def verify(obs):
                    verify_placement(self.state, task.move, obs, self.min_confidence)
                obs = self._read_stable(verify, not_before=completed_at)
                self._state = verify_placement(self.state, task.move, obs, self.min_confidence)
                self.pending_task = None
                self._next_phase()
                self._event('robot_committed', task_id=task.task_id)
                if self.phase == Phase.FINISHED:
                    self._event('game_finished', winner=int(self.state.winner()) if self.state.winner() else None)
            except Exception as exc:
                self._pause(exc)
            return self.phase
        finally:
            self._lock.release()

    def recover(self, *, robot_stopped=False, accept_human_move=False):
        """Operator-triggered reconciliation only; never executes or retries a task."""
        if not self._lock.acquire(blocking=False):
            raise RuntimeError('Controller busy')
        try:
            if self.phase != Phase.PAUSED or robot_stopped is not True:
                raise RuntimeError('Recovery requires paused state and explicit robot_stopped=True')
            try:
                expected = self.state.play(self.pending_task.move) if self.pending_task else None
                def valid(obs):
                    if obs.cells != self.state.cells and (expected is None or obs.cells != expected.cells):
                        if accept_human_move is True and self.pending_task is None and self.state.next_player==self.human_color:
                            detect_single_move(self.state,obs,self.min_confidence)
                            return
                        raise ValueError('Recovery board matches neither confirmed nor pending state')
                obs = self._read_stable(valid)
                if expected is not None and obs.cells == expected.cells:
                    self._state = expected
                    task_id=self.pending_task.task_id
                    self.pending_task=None
                    self._event('pending_committed_after_recovery', task_id=task_id)
                elif obs.cells!=self.state.cells and accept_human_move is True:
                    move=detect_single_move(self.state,obs,self.min_confidence)
                    self._state=self.state.play(move)
                    self._event('human_committed_after_recovery',move=[move.row,move.col,int(move.color)])
                self.pending_task = None
                self.pause_reason = ''
                self._next_phase()
                self._event('recovered')
            except Exception as exc:
                self._pause(exc)
            return self.phase
        finally:
            self._lock.release()
