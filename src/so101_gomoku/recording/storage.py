"""Atomic JSON replacement for local configuration artifacts."""
import json
import os
from pathlib import Path
import tempfile


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as file:
            name = file.name
            file.write(payload)
            file.flush()
            os.fsync(file.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))
