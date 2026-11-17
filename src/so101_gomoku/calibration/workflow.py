"""Manual board point entry, rigid fit, held-out report and versioned storage."""
from datetime import datetime, timezone
from uuid import uuid4
import numpy as np
from .board_geometry import BoardGeometry
from .transforms import fit_board_to_base, apply_transform, validate_transform
from so101_gomoku.recording.storage import atomic_json, read_json


SOURCES = ('manual_input', 'fk_model', 'synthetic')


def point_template(size=10, spacing_mm=21., measurement_source='manual_input'):
    geometry = BoardGeometry(size, spacing_mm/1000)
    if measurement_source not in SOURCES:
        raise ValueError('Unknown measurement source')
    last, mid = size-1, (size-1)//2
    fit = [(0,0), (0,last), (last,0), (last,last), (mid,mid)]
    validation = [(0,mid), (mid,0), (mid,last), (last,mid)]
    return {'schema_version':1, 'units':'m', 'intersections':size, 'spacing_m':geometry.spacing_m,
            'spacing_verified':False, 'measurement_source':measurement_source,
            'points':[{'row':r,'col':c,'role':role,'base_xyz_m':None}
                      for role, items in [('fit',fit),('validation',validation)] for r,c in items]}


def set_point(dataset, row, col, base_xyz_mm, role):
    geometry = BoardGeometry(dataset['intersections'], dataset['spacing_m'])
    geometry.intersection(row, col)
    xyz = np.asarray(base_xyz_mm, float)
    if role not in ('fit', 'validation') or xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError('Role and finite XYZ millimeters required')
    point = {'row':row,'col':col,'role':role,'base_xyz_m':(xyz/1000).tolist()}
    matches = [p for p in dataset['points'] if (p['row'],p['col']) == (row,col)]
    if matches and matches[0]['role'] != role:
        raise ValueError('Fit and validation coordinates must be independent')
    dataset['points'] = [p for p in dataset['points'] if (p['row'],p['col']) != (row,col)] + [point]
    return dataset


def fit_dataset(dataset, validation_limit_mm=3., calibration_id=None):
    if dataset.get('schema_version') != 1 or dataset.get('units') != 'm' or dataset.get('measurement_source') not in SOURCES:
        raise ValueError('Unsupported calibration dataset schema/units/source')
    if not np.isfinite(validation_limit_mm) or validation_limit_mm <= 0:
        raise ValueError('Positive finite validation threshold required')
    geometry = BoardGeometry(dataset['intersections'], dataset['spacing_m'])
    groups = {'fit':[], 'validation':[]}
    seen = set()
    for p in dataset['points']:
        coordinate = (p['row'],p['col'])
        if coordinate in seen or p['role'] not in groups:
            raise ValueError('Duplicate grid point or invalid role')
        seen.add(coordinate)
        board = geometry.intersection(*coordinate)
        xyz = np.asarray(p['base_xyz_m'],float)
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError(f'Point {coordinate} needs finite base XYZ before fitting')
        groups[p['role']].append((p,board,xyz))
    if len(groups['fit']) < 3 or not groups['validation']:
        raise ValueError('Need >=3 noncollinear fit points and >=1 independent validation point')
    fitted = fit_board_to_base([p[1] for p in groups['fit']], [p[2] for p in groups['fit']])
    report = {}
    for role, samples in groups.items():
        predicted = apply_transform(fitted.matrix,[p[1] for p in samples])
        errors = np.linalg.norm(predicted-np.array([p[2] for p in samples]),axis=1)
        report[role] = {'count':len(samples),'rms_mm':float(np.sqrt(np.mean(errors**2))*1000),
                        'max_mm':float(errors.max()*1000),
                        'points':[{'row':p[0]['row'],'col':p[0]['col'],'error_mm':float(error*1000),
                                   'predicted_base_xyz_m':pred.tolist()} for p,error,pred in zip(samples,errors,predicted)]}
    return {'schema_version':1,'calibration_id':calibration_id or f'board-{uuid4()}',
            'created_at':datetime.now(timezone.utc).isoformat(), 'frame_from':'board_intersections',
            'frame_to':'robot_base', 'units':'m', 'intersections':geometry.size,'spacing_m':geometry.spacing_m,
            'spacing_verified':bool(dataset.get('spacing_verified',False)),
            'measurement_source':dataset['measurement_source'],'T_base_board':fitted.matrix.tolist(),
            'validation_limit_mm':validation_limit_mm,'within_validation_limit':report['validation']['max_mm'] <= validation_limit_mm,
            'report':report,'source_points':dataset['points'],
            'accuracy_note':'Fit residuals are not external accuracy; FK/model and synthetic points do not establish hardware precision.'}


def validate_calibration(record):
    if (record.get('schema_version') != 1 or record.get('units') != 'm'
            or record.get('frame_from') != 'board_intersections' or record.get('frame_to') != 'robot_base'
            or not isinstance(record.get('calibration_id'),str) or not record['calibration_id']):
        raise ValueError('Unsupported calibration record')
    BoardGeometry(record['intersections'],record['spacing_m'])
    validate_transform(record['T_base_board'])
    # Recompute report rather than trusting a stored pass flag or stale matrix.
    dataset = {k:record[k] for k in ('schema_version','units','intersections','spacing_m','measurement_source','spacing_verified')}
    dataset['points'] = record['source_points']
    recomputed = fit_dataset(dataset,record['validation_limit_mm'],record['calibration_id'])
    if not np.allclose(recomputed['T_base_board'],record['T_base_board'],atol=1e-9,rtol=0):
        raise ValueError('Transform inconsistent with source calibration points')
    if recomputed['within_validation_limit'] != record['within_validation_limit']:
        raise ValueError('Calibration report inconsistent with source points')
    for role in ('fit','validation'):
        expected,stored=recomputed['report'][role],record['report'][role]
        if expected['count']!=stored['count'] or len(expected['points'])!=len(stored['points']):
            raise ValueError('Report point count mismatch')
        if not np.allclose([expected['rms_mm'],expected['max_mm']],[stored['rms_mm'],stored['max_mm']],rtol=1e-9,atol=1e-7):
            raise ValueError('Report errors inconsistent with source points')
        for a,b in zip(expected['points'],stored['points']):
            if (a['row'],a['col'])!=(b['row'],b['col']) or not np.isclose(a['error_mm'],b['error_mm'],atol=1e-7,rtol=1e-9):
                raise ValueError('Report point mismatch')
            if not np.allclose(a['predicted_base_xyz_m'],b['predicted_base_xyz_m'],atol=1e-10,rtol=1e-9):
                raise ValueError('Report prediction mismatch')
    return record


def load_calibration(path):
    return validate_calibration(read_json(path))


def save_calibration(path, record):
    atomic_json(path,validate_calibration(record))
