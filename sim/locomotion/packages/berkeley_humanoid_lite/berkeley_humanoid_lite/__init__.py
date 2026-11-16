# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

# The course robot model lives in <repo>/source/robot_models and is imported as `source.robot_models` (same as legs/,
# sim/sim2sim/). Scripts run from sim/locomotion/scripts/ do not have the repo root on sys.path, so add it here.
import sys as _sys
from pathlib import Path as _Path

for _parent in _Path(__file__).resolve().parents:
    if (_parent / "source" / "robot_models").is_dir():
        if str(_parent) not in _sys.path:
            _sys.path.append(str(_parent))
        break
