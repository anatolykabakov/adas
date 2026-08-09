#!/usr/bin/env python3
"""Load Android ADAS bag session directories (topic/*.bin) into timed message streams."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
# Generated stubs live in scripts/proto (see generate_proto_python.sh).
PROTO_DIR = SCRIPT_DIR.parent / "proto"
if not PROTO_DIR.is_dir():
    PROTO_DIR = SCRIPT_DIR / "proto"  # legacy fallback
sys.path.insert(0, str(PROTO_DIR))

# Stubs must come from protoc >= 3.19 (generate_proto_python.sh checks it): older ones make the
# runtime fall back to its pure-Python parser, which decodes a bag ~300x slower.
import bag_pb2  # noqa: E402
import messages_pb2  # noqa: E402


def fix_topic(name: str) -> str:
    return name.replace("/", "__")


def unfix_topic(dir_name: str) -> str:
    return dir_name.replace("__", "/")


def parse_payload(zmq_msg: messages_pb2.ZMQMessage) -> Any:
    which = zmq_msg.WhichOneof("payload")
    if which is None:
        return None
    return getattr(zmq_msg, which)


def load_topic_messages(
    session_dir: Path, topic: str
) -> List[Tuple[int, Any, messages_pb2.ZMQMessage]]:
    """Return sorted (timestamp_ms, payload, raw_zmq) for a topic."""
    topic_dir = session_dir / fix_topic(topic)
    if not topic_dir.is_dir():
        # also try already-fixed name
        topic_dir = session_dir / topic
    if not topic_dir.is_dir():
        return []

    out: List[Tuple[int, Any, messages_pb2.ZMQMessage]] = []
    for bin_path in sorted(topic_dir.glob("*.bin")):
        bag = bag_pb2.Bag()
        bag.ParseFromString(bin_path.read_bytes())
        for zmq_msg in bag.messages:
            payload = parse_payload(zmq_msg)
            ts = int(zmq_msg.timestamp)
            out.append((ts, payload, zmq_msg))
    out.sort(key=lambda x: x[0])
    return out


def list_topics(session_dir: Path) -> List[str]:
    topics = []
    for p in session_dir.iterdir():
        if p.is_dir() and any(p.glob("*.bin")):
            topics.append(unfix_topic(p.name))
    return sorted(topics)


def nearest(
    ts: int, series: List[Tuple[int, Any, Any]], max_dt_ms: int = 100
) -> Optional[Tuple[int, Any]]:
    """Find nearest message by timestamp within max_dt_ms."""
    if not series:
        return None
    best = min(series, key=lambda x: abs(x[0] - ts))
    if abs(best[0] - ts) > max_dt_ms:
        return None
    return best[0], best[1]


def iter_aligned(
    primary: List[Tuple[int, Any, Any]],
    others: Dict[str, List[Tuple[int, Any, Any]]],
    max_dt_ms: int = 80,
) -> Iterable[Dict[str, Any]]:
    """For each primary message, attach nearest messages from other topics."""
    for ts, payload, _ in primary:
        row: Dict[str, Any] = {"t": ts, "primary": payload}
        for name, series in others.items():
            hit = nearest(ts, series, max_dt_ms=max_dt_ms)
            row[name] = hit[1] if hit else None
            row[f"{name}_t"] = hit[0] if hit else None
        yield row


def lateral_actuation_on(health_messages) -> "np.ndarray":
    """Per-message flag: was lateral torque actually reaching the rack?

    Not simply `controls_allowed`. With always-on lateral (`lat_always_on`) the panda passes HCA frames while
    `controls_allowed` is false — dragonpilot did exactly that in 64.3 % of a drive on this car — so a filter
    keyed on `controls_allowed` would drop precisely the frames where the assist was working. That is the same
    mistake, inverted, that made every lateral number in `docs/BACKLOG.md` wrong before 2026-08-06.

    The OR needs no per-bag heuristic and is exact in both regimes, because `PandaService::assistAllowed`
    returns true whenever `controls_allowed` is true: so `lat_actuation_allowed` is a superset of it on bags
    that have the field, and false everywhere on bags recorded before it existed.
    """
    import numpy as np

    return np.asarray(
        [
            bool(getattr(m, "lat_actuation_allowed", False))
            or bool(getattr(m, "controls_allowed", False))
            for _, m in health_messages
        ]
    )
