"""Offline tests only. Scripted chat fixtures are NOT external LLM evidence."""
import io
import json
import unittest

import numpy as np

from .agent import run
from .tools import RobotTools, TOOLS


class ToolTests(unittest.TestCase):
    def setUp(self):
        self.tools = RobotTools()

    def tearDown(self):
        self.tools.close()

    def call(self, tool_name, **arguments):
        return self.tools.dispatch(tool_name, arguments)

    def test_state_and_schema(self):
        state = self.call('get_robot_state')
        self.assertFalse(state['connected_to_hardware'])
        self.assertEqual(len(state['joint_angles_rad']), 5)
        self.assertEqual(len({t['function']['name'] for t in TOOLS}), 9)

    def test_ik_actual_displacement(self):
        self.call('get_robot_state')
        result = self.call('move_tip', axis='+y', distance_mm=2)
        self.assertTrue(result['ok'], result)
        self.assertGreater(result['measured_delta_mm'][1], 1.0)
        self.assertLess(np.linalg.norm(np.array(result['measured_delta_mm']) - [0, 2, 0]), 1.0)

    def test_existing_preset(self):
        poses = self.call('list_presets')['poses']
        self.assertIn('stand', poses)
        self.call('get_robot_state')
        self.assertTrue(self.call('move_to_pose', name='stand')['ok'])

    def test_gripper(self):
        self.call('get_robot_state')
        result = self.call('set_gripper', opening=55)
        self.assertTrue(result['ok'])
        self.assertAlmostEqual(result['gripper'], 55, delta=1.5)

    def test_detection_and_tracking_feedback(self):
        self.call('get_robot_state')
        det = self.call('detect_object', description='black ball')
        self.assertTrue(det['found'])
        result = self.call('track_object', object_id=det['object_id'])
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['iterations'], 3)
        errors = [np.hypot(h['du'], h['dv']) for h in result['history']]
        self.assertLess(errors[-1], errors[0], errors)

    def test_motion_needs_state(self):
        self.assertFalse(self.call('move_tip', axis='+y', distance_mm=2)['ok'])
        self.assertTrue(self.tools.stopped)

    def test_stop_latched(self):
        self.call('stop_tracking')
        self.call('get_robot_state')
        self.assertFalse(self.call('set_gripper', opening=55)['ok'])

    def test_lost_target_stops(self):
        self.tools.camera.visible = False
        result = self.call('detect_object', description='black ball')
        self.assertFalse(result['found'])
        self.assertTrue(self.tools.stopped)

    def test_loss_during_tracking_stops(self):
        self.call('get_robot_state')
        self.call('detect_object', description='black ball')
        self.tools.camera.visible = False
        self.assertFalse(self.call('track_object', object_id=1)['ok'])
        self.assertTrue(self.tools.stopped)

    def test_unknown_tool_and_bad_args_rejected(self):
        self.assertFalse(self.call('__import__', module='os')['ok'])
        self.assertFalse(self.call('get_robot_state', unexpected=True)['ok'])

    def test_nonfinite_and_boolean_rejected(self):
        for number in [float('nan'), float('inf'), True, 100]:
            self.assertFalse(self.call('move_tip', axis='+x', distance_mm=number)['ok'])

    def test_no_arbitrary_object_detection(self):
        self.assertFalse(self.call('detect_object', description='red cup')['ok'])

    def test_stale_target(self):
        self.call('get_robot_state')
        self.call('detect_object', description='black ball')
        self.tools.detected_at -= 100
        self.assertFalse(self.call('track_object', object_id=1)['ok'])

    def test_protocol_loop_offline_fixture(self):
        responses = iter([
            {'role': 'assistant', 'content': None, 'tool_calls': [dict(id='a', type='function', function=dict(name='get_robot_state', arguments='{}'))]},
            {'role': 'assistant', 'content': None, 'tool_calls': [dict(id='b', type='function', function=dict(name='move_tip', arguments='{"axis":"+y","distance_mm":2}'))]},
            {'role': 'assistant', 'content': None, 'tool_calls': [dict(id='c', type='function', function=dict(name='stop_motion', arguments='{}'))]},
            {'role': 'assistant', 'content': 'Offline protocol fixture completed; NOT external API validation'},
        ])
        seen = []
        def fake_chat(messages):
            seen.append(list(messages))
            return next(responses)
        trace = io.StringIO()
        run('offline fixture', self.tools, fake_chat, trace)
        tool_messages = [m for m in seen[-1] if m['role'] == 'tool']
        self.assertEqual([m['tool_call_id'] for m in tool_messages], ['a','b','c'])
        self.assertTrue(all(json.loads(m['content'])['ok'] for m in tool_messages))
        self.assertIn('session_cleanup', trace.getvalue())

    def test_plain_text_not_accepted_as_tool_call(self):
        with self.assertRaises(RuntimeError):
            run('test', self.tools, lambda _: {'role': 'assistant', 'content': 'done'}, io.StringIO())


if __name__ == '__main__':
    unittest.main()
