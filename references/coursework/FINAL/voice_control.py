"""FINAL entry point for SO-101 voice control acceptance."""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from so101_arm.voice_control import main


if __name__ == "__main__":
    if "--config" not in sys.argv:
        sys.argv[1:1] = ["--config", str(Path(__file__).with_name("voice_commands.json"))]
    raise SystemExit(main())
