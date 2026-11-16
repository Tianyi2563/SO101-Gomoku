"""Check hardware dependencies without opening a serial port or camera."""
import importlib
import json
from pathlib import Path

result = {}
for name in ('torch', 'placo', 'lerobot.robots.so_follower'):
    try:
        module = importlib.import_module(name)
        result[name] = {'available': True, 'version': getattr(module, '__version__', None)}
        if name == 'torch':
            result[name]['tensor_sum'] = module.tensor([1., 2.]).sum().item()
        if name == 'placo':
            result[name]['ik_api'] = all(hasattr(module, attr) for attr in ('RobotWrapper', 'KinematicsSolver'))
    except Exception as exc:
        result[name] = {'available': False, 'error': f'{type(exc).__name__}: {exc}'}
output = Path(__file__).resolve().parents[2] / 'data/dependency-status.json'
output.write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps(result, indent=2))
