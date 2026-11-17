"""Single-writer, durable JSONL sessions and hardware-free validated replay."""
import json
import os
from pathlib import Path
from uuid import uuid4
from dataclasses import asdict
from math import isfinite
from so101_gomoku.contracts.models import Cell, Move, RobotTask, BoardObservation
from so101_gomoku.game.board import GameState


def state_to_dict(state):
    return {'cells':[[int(c) for c in row] for row in state.cells],
            'next_player':int(state.next_player),'version':state.version,
            'last_move':asdict(state.last_move) if state.last_move else None}


def state_from_dict(value):
    if type(value['version']) is not int or value['version']<0:
        raise ValueError('Invalid game version')
    move=Move(**value['last_move']) if value.get('last_move') is not None else None
    state=GameState(tuple(tuple(Cell(c) for c in row) for row in value['cells']),Cell(value['next_player']),value['version'],move)
    if move is not None:
        if (type(move.row) is not int or type(move.col) is not int
                or not 0<=move.row<state.size or not 0<=move.col<state.size
                or state.cells[move.row][move.col]!=move.color):
            raise ValueError('Inconsistent last move')
    return state


def task_from_dict(value):
    if value is None:return None
    return RobotTask(value['task_id'],Move(**value['move']),value['expected_game_version'],value['calibration_id'])


def _read_lines(path):
    raw=path.read_bytes() if path.exists() else b''
    if raw and not raw.endswith(b'\n'):
        raise ValueError('Incomplete final journal line; use explicit repair_tail before reopening')
    records=[]
    for line in raw.splitlines():
        if not line:raise ValueError('Blank line in session journal')
        records.append(json.loads(line))
    return records


def _validate_event(record, previous, previous_event, old_state, state, meta):
    event=record.get('event');new=record['checkpoint']
    if (not isinstance(event,dict) or not isinstance(event.get('event'),str)
            or event.get('phase')!=new['phase'] or event.get('game_version')!=state.version
            or not isinstance(event.get('time'),(int,float)) or not isfinite(event['time'])):
        raise ValueError('Missing or inconsistent event payload')
    name=event['event'];before=previous['phase'];after=new['phase']
    prior=task_from_dict(previous['pending_task']);pending=task_from_dict(new['pending_task'])
    delta=state.version-old_state.version
    if after=='boot' and before!='boot':raise ValueError('Cannot transition back to boot')
    if after=='wait_human' and (state.finished or state.next_player!=meta['human_color']):
        raise ValueError('Waiting phase does not match side to move')
    if after=='thinking' and (state.finished or state.next_player==meta['human_color']):
        raise ValueError('Thinking phase does not match robot turn')
    if after=='finished' and not state.finished:raise ValueError('Premature finished phase')
    if after in ('executing','verifying') and pending is None:
        raise ValueError('Execution/verification requires pending task')
    same_pending=prior==pending
    if name=='observation':
        obs=event.get('observation')
        if not isinstance(obs,dict):raise ValueError('Observation payload missing')
        BoardObservation(tuple(map(tuple,obs['cells'])),tuple(map(tuple,obs['confidence'])),
                         obs['frame_id'],obs['captured_at'],obs['calibration_id'])
        allowed=after==before and after in ('boot','thinking','wait_human','verifying','paused')
        if before=='executing' and after=='verifying':
            allowed=(previous_event is not None and previous_event.get('event')=='execution_result'
                     and previous_event.get('completed') is True and prior is not None
                     and previous_event.get('task_id')==prior.task_id)
        if not allowed or not same_pending or delta:raise ValueError('Invalid observation phase transition')
    elif name=='started':
        if before!='boot' or after not in ('thinking','wait_human','finished') or pending or delta:
            raise ValueError('Invalid start transition')
    elif name=='waiting_for_human':
        if before!=after or after!='wait_human' or pending or delta:raise ValueError('Invalid wait transition')
    elif name=='human_committed':
        if (before!='wait_human' or after not in ('thinking','finished') or delta!=1
                or state.last_move.color!=meta['human_color'] or pending):
            raise ValueError('Invalid human commit')
    elif name=='decision':
        if (before!='thinking' or after!='thinking' or delta or prior is not None or pending is None
                or event.get('task_id')!=pending.task_id):raise ValueError('Invalid decision intent')
    elif name=='execution_started':
        if (before!='thinking' or after!='executing' or delta or not same_pending or pending is None
                or event.get('task_id')!=pending.task_id):raise ValueError('Execution intent missing matching pending task')
    elif name=='execution_result':
        if (before!='executing' or after!='executing' or delta or not same_pending or pending is None
                or type(event.get('completed')) is not bool or not isinstance(event.get('stage'),str)
                or not isinstance(event.get('reason'),str)):
            raise ValueError('Invalid execution result record')
    elif name in ('robot_committed','pending_committed_after_recovery'):
        expected_before='verifying' if name=='robot_committed' else 'paused'
        allowed_after=('wait_human','finished') if name=='robot_committed' else ('paused',)
        if (before!=expected_before or after not in allowed_after or delta!=1 or prior is None or pending is not None
                or prior.move!=state.last_move or event.get('task_id')!=prior.task_id):
            raise ValueError('Commit does not match pending task lifecycle')
    elif name=='human_committed_after_recovery':
        if (before!='paused' or after!='paused' or delta!=1 or prior is not None or pending is not None
                or state.last_move.color!=meta['human_color']):
            raise ValueError('Invalid recovered human move')
    elif name=='restart_loaded':
        if after!='paused' or delta or not same_pending:raise ValueError('Invalid restart checkpoint')
    elif name=='recovered':
        if before!='paused' or after not in ('thinking','wait_human','finished') or delta or pending:
            raise ValueError('Invalid recovery transition')
    elif name=='paused':
        if after!='paused' or not isinstance(event.get('reason'),str):raise ValueError('Invalid pause event')
        if delta and not (delta==1 and ((before=='wait_human' and state.last_move.color==meta['human_color'])
                       or (before=='verifying' and prior is not None and prior.move==state.last_move))):
            raise ValueError('Unexpected game change while pausing')
    elif name=='game_finished':
        if before!='finished' or after!='finished' or delta or pending:raise ValueError('Invalid terminal event')
    else:raise ValueError('Unknown schema-1 event name')
    if delta and name not in ('human_committed','robot_committed','pending_committed_after_recovery','human_committed_after_recovery','paused'):
        raise ValueError('Unexpected commit event')


def validate_records(records):
    if not records or records[0].get('kind')!='header' or records[0].get('schema_version')!=1:
        raise ValueError('Missing or unsupported session header')
    header=records[0]
    meta=header['metadata']
    state=state_from_dict(header['initial_state'])
    if state.size!=meta['intersections'] or meta['human_color'] not in (-1,1) or not meta['calibration_id']:
        raise ValueError('Invalid session configuration')
    checkpoint={'state':state_to_dict(state),'phase':'boot','pending_task':None}
    previous_event=None
    phases={'boot','wait_human','thinking','executing','verifying','paused','finished'}
    for index,record in enumerate(records):
        if type(record.get('sequence')) is not int or record['sequence']!=index:
            raise ValueError('Session sequence is missing, reordered or duplicated')
        if index==0:continue
        if record.get('kind')=='image':
            asset=Path(record['asset']['path'])
            if asset.is_absolute() or '..' in asset.parts:raise ValueError('Invalid image asset path')
            continue
        if record.get('kind')!='event':raise ValueError('Unknown session record kind')
        new=record['checkpoint']
        next_state=state_from_dict(new['state'])
        if next_state.size!=state.size or new['phase'] not in phases:
            raise ValueError('Invalid checkpoint dimensions/phase')
        delta=next_state.version-state.version
        if delta==0:
            if next_state!=state:raise ValueError('Board changed without version increment')
        elif delta==1:
            if next_state.last_move is None or state.play(next_state.last_move)!=next_state:
                raise ValueError('Invalid committed game transition')
        else:raise ValueError('Game versions must advance one legal move at a time')
        pending=task_from_dict(new['pending_task'])
        if pending is not None:
            if (not pending.task_id or pending.expected_game_version!=next_state.version
                    or pending.calibration_id!=meta['calibration_id']):
                raise ValueError('Pending task configuration/version mismatch')
            next_state.play(pending.move)
        _validate_event(record,checkpoint,previous_event,state,next_state,meta)
        previous_event=record['event']
        checkpoint=new;state=next_state
    return {'metadata':meta,'checkpoint':checkpoint,'records':records}


def replay_session(directory):
    """Read-only; never constructs an executor or sends a robot command."""
    return validate_records(_read_lines(Path(directory)/'events.jsonl'))


class SessionJournal:
    def __init__(self, directory, *, repair_tail=False, configuration=None):
        self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=True)
        self.path=self.directory/'events.jsonl'
        self.configuration=configuration if configuration is not None else {}
        self._image_refs={}
        self._failed=False
        self._lock=open(self.directory/'writer.lock','a+b')
        self._file=None
        try:
            self._lock.seek(0,2)
            if self._lock.tell()==0:self._lock.write(b'0');self._lock.flush()
            self._lock.seek(0)
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(self._lock.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self._lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
            if repair_tail and self.path.exists():
                raw=self.path.read_bytes()
                if raw and not raw.endswith(b'\n'):
                    end=raw.rfind(b'\n')+1
                    (self.directory/f'truncated-tail-{uuid4()}.bin').write_bytes(raw[end:])
                    with open(self.path,'r+b') as f:f.truncate(end);f.flush();os.fsync(f.fileno())
            self._records=_read_lines(self.path)
            if self._records:
                saved=validate_records(self._records)
                if configuration is not None and configuration!=saved['metadata']['configuration']:
                    raise ValueError('Provided configuration conflicts with saved session')
                self.configuration=saved['metadata']['configuration']
            self._file=open(self.path,'ab')
        except Exception:
            self._lock.close()
            raise

    @property
    def initialized(self):return bool(self._records)

    def _append(self, record):
        if self._failed or self._file is None:raise RuntimeError('Session writer unavailable')
        record={**record,'sequence':len(self._records)}
        raw=json.dumps(record,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode('utf-8')+b'\n'
        try:
            self._file.write(raw);self._file.flush();os.fsync(self._file.fileno())
        except Exception:
            self._failed=True
            raise
        self._records.append(json.loads(raw))

    def begin(self, state, calibration_id, human_color, controller_settings=None):
        if self.initialized:raise RuntimeError('Existing session requires resume_from_journal, not a new game')
        self._append({'kind':'header','schema_version':1,'initial_state':state_to_dict(state),
                      'metadata':{'calibration_id':calibration_id,'human_color':int(human_color),
                                  'intersections':state.size,'configuration':self.configuration,
                                  'controller_settings':controller_settings or {}}})

    def append_event(self, event, state, phase, pending_task):
        if not self.initialized:raise RuntimeError('Session must begin before events')
        self._append({'kind':'event','event':event,'checkpoint':{'state':state_to_dict(state),
                     'phase':phase,'pending_task':asdict(pending_task) if pending_task else None}})

    def checkpoint(self):return validate_records(self._records)

    def record_image(self, frame_id, image):
        import cv2
        if not self.initialized:raise RuntimeError('Begin session before capturing images')
        if self._failed:raise RuntimeError('Session writer unavailable')
        ok,encoded=cv2.imencode('.png',image)
        if not ok:raise RuntimeError('Cannot encode session image')
        relative=f'images/frame-{uuid4()}.png'
        path=self.directory/relative;path.parent.mkdir(exist_ok=True)
        with open(path,'xb') as file:file.write(encoded.tobytes());file.flush();os.fsync(file.fileno())
        asset={'frame_id':frame_id,'path':relative,'shape':list(image.shape)}
        self._append({'kind':'image','asset':asset})
        self._image_refs[frame_id]=asset
        return asset

    def take_image_reference(self,frame_id):return self._image_refs.pop(frame_id,None)

    def close(self):
        if self._file is not None:self._file.close();self._file=None
        if self._lock is not None:self._lock.close();self._lock=None

    def __enter__(self):return self
    def __exit__(self,*_):self.close()
