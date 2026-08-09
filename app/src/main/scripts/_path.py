#!/usr/bin/env python3
"""Ensure ``scripts/`` is on ``sys.path`` so ``core``, ``vis``, ``sim`` import cleanly."""

from __future__ import annotations

import sys
from pathlib import Path

_SCRIPTS_ROOT = Path(__file__).resolve().parent
_root = str(_SCRIPTS_ROOT)
if _root not in sys.path:
    sys.path.insert(0, _root)
_proto = str(_SCRIPTS_ROOT / "proto")
if _proto not in sys.path:
    sys.path.insert(0, _proto)
