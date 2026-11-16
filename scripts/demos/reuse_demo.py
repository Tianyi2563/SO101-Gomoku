from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from so101_gomoku.interfaces.demo import run
if __name__ == "__main__":
    print(run(ROOT / "data/sessions/reuse_demo.json"))
