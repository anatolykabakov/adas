#!/usr/bin/env python3
"""Same job as ``scripts/_path.py``, from a subdirectory.

Python puts the script's own directory on ``sys.path``, not the repository's, so a script under
``bag/``, ``rlog/`` or ``tools/`` cannot ``import _path`` from the parent without this shim.
"""

from __future__ import annotations

import sys
from pathlib import Path

_SCRIPTS_ROOT = Path(__file__).resolve().parents[1]
for _p in (_SCRIPTS_ROOT, _SCRIPTS_ROOT / "proto"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
