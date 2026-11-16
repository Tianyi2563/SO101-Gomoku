"""Audit real external run traces separately from offline fixture tests."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCENARIOS = {
    'IK': {'get_robot_state', 'move_tip', 'stop_motion'},
    'vision': {'get_robot_state', 'detect_object', 'track_object', 'stop_tracking'},
    'preset_gripper': {'list_presets', 'get_robot_state', 'move_to_pose', 'set_gripper', 'stop_motion'},
}


def audit():
    results = []
    for path in sorted((ROOT / 'runs').glob('*/trace.jsonl')):
        records = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
        config = records[0]
        responses = [r for r in records if r['event'] == 'llm_response']
        calls = [c for r in responses for c in r['message'].get('tool_calls', [])]
        tools = [r for r in records if r['event'] == 'tool_result']
        expected_pairs = [(c['id'], c['function']['name']) for c in calls]
        actual_pairs = [(r['tool_call_id'], r['name']) for r in tools]
        completed = any(r['event'] == 'completed' for r in records)
        valid = (config.get('external_api') is True and completed and bool(calls)
                 and expected_pairs == actual_pairs and all(r['result'].get('ok') for r in tools)
                 and all(r['result'].get('connected_to_hardware') is not True for r in tools))
        names = [r['name'] for r in tools]
        result = dict(run=path.parent.name, model=config['model'], valid_external_trace=valid,
                      api_responses=len(responses), tool_calls=names,
                      scenarios=[name for name, required in SCENARIOS.items() if valid and required <= set(names)],
                      note='API failure or incomplete' if not completed else 'Mock motion; not hardware validation')
        for entry in tools:
            if entry['name'] == 'track_object':
                result['vision_result'] = entry['result']
            if entry['name'] == 'move_tip':
                result['ik_result'] = entry['result']
        results.append(result)
    covered = {s for r in results for s in r['scenarios']}
    summary = dict(all_three_scenarios_passed=set(SCENARIOS) <= covered, runs=results,
                   warning='Trace consistency audit, not cryptographic proof of remote provider identity')
    (ROOT / 'acceptance_results.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


if __name__ == '__main__':
    audit()
