#!/usr/bin/env python3
"""Full vision→control latency from bag (BOOTTIME ms stamps).

Do **not** use raw controls/steer capture→publish: steer republishes on every
chassis tick with a stale vision_ts (ages up to the vision period).

Pipeline (one accepted frame)
  capture → prep+infer (e2e) → ZMQ/native → lane_keep publish
           → PID → steer (same vision_ts, fresh) → panda CAN TX (~10 ms)

Metrics
  vision/lanes
    infer_ms / e2e_ms / prep≈e2e−infer / interval / Hz
  control/lane_keep
    capture→publish   — full path to desired-steer publish
    vision→publish    — after ONNX only
  controls/steer (fresh only: vision→publish ≤ 50 ms)
    capture→publish   — proxy for command ready for CAN (same frame as onLanes)

Usage:
  python3 tools/latency.py /path/to/session
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import _path  # noqa: F401  — puts scripts/ and scripts/proto on sys.path

from vis.bag_io import load_topic_messages  # noqa: E402

# Steer msgs with vision→publish above this are chassis republishes of a stale path.
FRESH_STEER_VISION_MAX_MS = 50.0
MAX_DT_MS = 5000.0


def summarize(name: str, xs: List[float]) -> None:
    if not xs:
        print(f"{name}: no samples")
        return
    xs_sorted = sorted(xs)
    p95 = xs_sorted[int(0.95 * (len(xs_sorted) - 1))]
    print(
        f"{name}: n={len(xs)}  median={statistics.median(xs):.1f} ms  "
        f"mean={statistics.mean(xs):.1f} ms  p95={p95:.1f} ms  max={max(xs):.1f} ms"
    )


def _hz(timestamps_ms: List[int]) -> Optional[float]:
    if len(timestamps_ms) < 2:
        return None
    ts = sorted(timestamps_ms)
    dt_s = (ts[-1] - ts[0]) / 1000.0
    if dt_s <= 0:
        return None
    return (len(ts) - 1) / dt_s


def _intervals(timestamps_ms: List[int]) -> List[float]:
    ts = sorted(timestamps_ms)
    out: List[float] = []
    for a, b in zip(ts, ts[1:]):
        dt = b - a
        if 0 < dt <= MAX_DT_MS:
            out.append(float(dt))
    return out


def _pub(payload, zmq) -> int:
    publish = int(getattr(payload, "publish_ts_ms", 0) or 0)
    if publish <= 0:
        publish = int(getattr(payload, "timestamp", 0) or 0) or int(zmq.timestamp)
    return publish


def vision_stats(session: Path) -> Tuple[List[float], List[float]]:
    infer_dur: List[float] = []
    e2e: List[float] = []
    prep: List[float] = []
    capture_ts: List[int] = []
    # Frame arrival breakdown — present from 2026-08-06 bags on; older bags leave these empty.
    delivery: List[float] = []  # capture -> submitYuv: camera and ISP path
    queue: List[
        float
    ] = []  # submitYuv -> inference pickup: waiting on the previous frame
    dropped: List[float] = []  # captures overwritten in the 1-slot buffer

    for _, payload, zmq in load_topic_messages(session, "vision/lanes"):
        if payload is None:
            continue
        capture = int(getattr(payload, "capture_ts_ms", 0) or 0)
        infer = int(getattr(payload, "infer_ts_ms", 0) or 0)
        dur = float(getattr(payload, "infer_duration_ms", 0) or 0)
        if capture > 0:
            capture_ts.append(capture)
        if dur > 0:
            infer_dur.append(dur)
        if capture > 0 and infer >= capture:
            dt = float(infer - capture)
            if 0 <= dt <= MAX_DT_MS:
                e2e.append(dt)
                if dur > 0 and dt >= dur:
                    prep.append(dt - dur)

        submit = int(getattr(payload, "submit_ts_ms", 0) or 0)
        pickup = int(getattr(payload, "pickup_ts_ms", 0) or 0)
        if capture > 0 and submit >= capture and submit - capture <= MAX_DT_MS:
            delivery.append(float(submit - capture))
        if submit > 0 and pickup >= submit and pickup - submit <= MAX_DT_MS:
            queue.append(float(pickup - submit))
        if submit > 0:
            dropped.append(float(getattr(payload, "frames_dropped", 0) or 0))

    print("=== vision/lanes ===")
    summarize("infer_ms (session.run)", infer_dur)
    summarize("prep_ms (e2e − infer)", prep)
    summarize("e2e_ms (infer_ts − capture)", e2e)
    summarize("interval_ms (capture→capture)", _intervals(capture_ts))
    if delivery:
        summarize("delivery_ms (capture→submitYuv)", delivery)
        summarize("queue_ms (submitYuv→pickup)", queue)
        n = len(dropped)
        kept = sum(1 for d in dropped if d == 0)
        import statistics

        print(
            f"frames_dropped: {statistics.mean(dropped):.2f} per processed frame, "
            f"{100.0 * kept / n:.0f} % of cycles dropped nothing  (n={n})"
        )
        print(
            "  many drops + short delivery = inference too slow; "
            "few drops + long delivery = camera arriving late"
        )
    hz = _hz(capture_ts)
    if hz is not None:
        print(f"rate: {hz:.2f} Hz  (n={len(capture_ts)})")
    elif not capture_ts:
        print("rate: no capture_ts_ms (old bag?)")
    print()
    return e2e, infer_dur


def lane_keep_stats(session: Path) -> None:
    capture_pub: List[float] = []
    vision_pub: List[float] = []
    publish_ts: List[int] = []

    for _, payload, zmq in load_topic_messages(session, "control/lane_keep"):
        if payload is None:
            continue
        capture = int(getattr(payload, "capture_ts_ms", 0) or 0)
        vision = int(getattr(payload, "vision_ts_ms", 0) or 0)
        publish = _pub(payload, zmq)
        if publish > 0:
            publish_ts.append(publish)
        if capture > 0 and publish > 0:
            dt = float(publish - capture)
            if 0 <= dt <= MAX_DT_MS:
                capture_pub.append(dt)
        if vision > 0 and publish > 0:
            dt = float(publish - vision)
            if 0 <= dt <= MAX_DT_MS:
                vision_pub.append(dt)

    print("=== control/lane_keep (desired path / SWA) ===")
    summarize("capture → publish  [full → control]", capture_pub)
    summarize("vision → publish   [post-ONNX]", vision_pub)
    summarize("interval_ms", _intervals(publish_ts))
    hz = _hz(publish_ts)
    if hz is not None:
        print(f"rate: {hz:.2f} Hz  (n={len(publish_ts)})")
    print()


def steer_fresh_stats(session: Path) -> None:
    """Steer published on the onLanes path (fresh vision_ts), not chassis aging."""
    capture_pub: List[float] = []
    vision_pub: List[float] = []
    n_total = 0
    n_stale = 0

    for _, payload, zmq in load_topic_messages(session, "controls/steer"):
        if payload is None:
            continue
        n_total += 1
        capture = int(getattr(payload, "capture_ts_ms", 0) or 0)
        vision = int(getattr(payload, "vision_ts_ms", 0) or 0)
        publish = _pub(payload, zmq)
        if capture <= 0 or vision <= 0 or publish <= 0:
            continue
        vt = float(publish - vision)
        if vt < 0 or vt > MAX_DT_MS:
            continue
        if vt > FRESH_STEER_VISION_MAX_MS:
            n_stale += 1
            continue
        capture_pub.append(float(publish - capture))
        vision_pub.append(vt)

    print(
        "=== controls/steer fresh (vision→publish ≤ "
        f"{FRESH_STEER_VISION_MAX_MS:.0f} ms) ≈ ready for CAN ==="
    )
    print(
        f"(kept {len(capture_pub)} / {n_total} steer msgs; "
        f"skipped {n_stale} stale chassis republish)"
    )
    summarize("capture → publish  [full → steer/CAN cmd]", capture_pub)
    summarize("vision → publish", vision_pub)
    print("panda/tx is typically ~10 ms after; not stamped on steer.")
    print()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path)
    args = ap.parse_args()
    session = args.session.resolve()
    if not session.is_dir():
        parent = session.parent
        hint = ""
        if parent.is_dir():
            close = sorted(
                p.name
                for p in parent.iterdir()
                if p.is_dir() and p.name.startswith(session.name)
            )
            if close:
                hint = f"\nDid you mean: {', '.join(close[:5])}"
        raise SystemExit(f"session not found: {session}{hint}")

    print(f"session: {session}")
    print()
    vision_stats(session)
    lane_keep_stats(session)
    steer_fresh_stats(session)


if __name__ == "__main__":
    main()
