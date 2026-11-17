import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import unittest
import tempfile
import numpy as np
from so101_gomoku.calibration.workflow import point_template,set_point,fit_dataset,save_calibration,load_calibration
from so101_gomoku.recording.storage import atomic_json


def synthetic_points():
    data=point_template(measurement_source='synthetic')
    for point in data['points']:
        point['base_xyz_m']=[.1+point['col']*.021,-.2+point['row']*.021,.04]
    return data


class CalibrationWorkflowTests(unittest.TestCase):
    def test_round_trip_and_report(self):
        record=fit_dataset(synthetic_points(),calibration_id='synthetic')
        self.assertTrue(record['within_validation_limit'])
        self.assertLess(record['report']['validation']['max_mm'],1e-9)
        self.assertFalse(record['spacing_verified'])
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'cal.json';save_calibration(path,record)
            self.assertEqual(load_calibration(path)['calibration_id'],'synthetic')
            record['T_base_board'][0][3]+=.01
            atomic_json(path,record)
            with self.assertRaises(ValueError):load_calibration(path)

    def test_mm_entry_and_invalid_points(self):
        data=point_template()
        set_point(data,0,0,[100,200,30],'fit')
        point=next(p for p in data['points'] if (p['row'],p['col'])==(0,0))
        self.assertEqual(point['base_xyz_m'],[.1,.2,.03])
        with self.assertRaises(ValueError):fit_dataset(data)
        with self.assertRaises(ValueError):set_point(data,0,0,[0,0,0],'validation')
        with self.assertRaises(ValueError):set_point(data,10,0,[0,0,0],'fit')
        data=synthetic_points();data['points'].append(data['points'][0].copy())
        with self.assertRaises(ValueError):fit_dataset(data)

    def test_validation_error_not_hidden_by_good_fit(self):
        data=synthetic_points()
        next(p for p in data['points'] if p['role']=='validation')['base_xyz_m'][0]+=.01
        record=fit_dataset(data)
        self.assertFalse(record['within_validation_limit'])
        self.assertGreater(record['report']['validation']['max_mm'],9.)
        self.assertLess(record['report']['fit']['max_mm'],1e-8)
