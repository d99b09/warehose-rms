from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
for package in (ROOT/'ros_ws/src').iterdir():
    if package.is_dir():
        sys.path.insert(0, str(package))
