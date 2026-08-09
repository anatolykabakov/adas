#!/usr/bin/env python3
"""Incidents dictated by the operator during the drive.

The session carries the voice channel and its transcription
(``audio_<t0>.m4a`` / ``audio_<t0>.medium.json``); the number in the file name is the bag
clock at the first audio sample, which is what ties a spoken second to a bag timestamp.

The operator marks a place two ways — "инцидент" and "запиши …" — and in the second form
the rest of the phrase is the complaint itself ("запиши, не докрут руля"), so the text is
kept as the label rather than thrown away.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

KEYWORDS = ("инцидент", "запиши", "incident")


@dataclass
class Incident:
    t_ms: float  # bag clock, start of the phrase
    t_end_ms: float  # end of the phrase
    text: str


def _audio_start_ms(path: Path) -> Optional[int]:
    m = re.search(r"audio_(\d+)", path.stem)
    return int(m.group(1)) if m else None


def load(bag_dir: Optional[Path], keywords=KEYWORDS) -> List[Incident]:
    """Spoken marks of a session, in bag time. Empty when there is no transcription."""
    if bag_dir is None or not Path(bag_dir).is_dir():
        return []
    files = sorted(Path(bag_dir).glob("audio_*.json"))
    out: List[Incident] = []
    for path in files:
        t0 = _audio_start_ms(path)
        if t0 is None:
            continue
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        for seg in data.get("segments", []):
            text = str(seg.get("text", "")).strip()
            low = text.lower()
            if not any(k in low for k in keywords):
                continue
            start = float(seg.get("start", 0.0))
            end = float(seg.get("end", start))
            out.append(
                Incident(
                    t_ms=t0 + 1000.0 * start,
                    t_end_ms=t0 + 1000.0 * max(end, start + 0.2),
                    text=text,
                )
            )
    out.sort(key=lambda i: i.t_ms)
    return out
