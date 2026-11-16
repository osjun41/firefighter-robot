# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

# The course robot model lives in <repo>/source/robot_models and is imported as `source.robot_models` (same as legs/,
# sim/sim2sim/). Scripts run from scripts/isaaclab/ do not have the repo root on sys.path, so add it here.
import sys as _sys
from pathlib import Path as _Path

_REPO_ROOT = str(_Path(__file__).resolve().parents[3])
if _REPO_ROOT not in _sys.path:
    _sys.path.append(_REPO_ROOT)
