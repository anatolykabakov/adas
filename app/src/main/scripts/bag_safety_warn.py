#!/usr/bin/env python3
"""Count FCW / AEB / LDW over a recorded run — false-positive check on real driving.

Feeds bag ``vision/lanes`` + ``vehicle/state`` into the real C++ chain (``TopicConvertService``
→ ``vision/path`` → ``SafetyWarnService``, via ``pyadas``) and counts warnings. Every warning on
a normal run is a false positive by definition: these bags contain no collisions and no
unintentional lane departures.

The old lane-departure rule (a bare ``|cte| > 0.5 m`` on the model path, no speed gate, no
drift direction, no debounce) is recomputed from the same CTE for comparison.

  python3 bag_safety_warn.py /path/to/bag
  python3 bag_safety_warn.py /path/to/bag --csv warn.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from vis.bag_io import iter_aligned, load_topic_messages

OLD_LDW_THRESHOLD_M = 0.5


def replay(bag: Path, max_dt_ms: int) -> List[Dict]:
    lanes = load_topic_messages(bag, "vision/lanes")
    state = load_topic_messages(bag, "vehicle/state")
    if not lanes or not state:
        raise SystemExit(f"{bag}: need topics vision/lanes and vehicle/state")

    # topic_convert=True: C++ builds the path on-device, same as the phone, including lane flag.
    app = pyadas.AdasApp(wheelbase=2.636, topic_convert=True)
    rows: List[Dict] = []
    t_us = 0
    prev_ts_ms = None

    for row in iter_aligned(lanes, {"state": state}, max_dt_ms=max_dt_ms):
        ll, st = row["primary"], row["state"]
        if st is None:
            continue
        ts_ms = int(getattr(ll, "timestamp", 0) or 0)
        dt_us = (
            90_000
            if prev_ts_ms is None
            else max(1_000, min(500_000, (ts_ms - prev_ts_ms) * 1000))
        )
        prev_ts_ms = ts_ms
        t_us += dt_us

        speed = float(getattr(st, "v_ego", 0.0) or 0.0)
        app.publish_chassis(
            t_us,
            speed,
            float(np.deg2rad(getattr(st, "steering_angle_deg", 0.0) or 0.0)),
            float(getattr(st, "yaw_rate", 0.0) or 0.0),
        )
        # Raw model output: the C++ side does the fusion, so lane_anchored is the real verdict.
        ll_copy = type(ll)()
        ll_copy.CopyFrom(ll)
        ll_copy.timestamp = t_us // 1000
        ll_copy.capture_ts_ms = t_us // 1000
        app.publish_lane_lines(ll_copy.SerializeToString())
        app.step(t_us)

        for msg in app.pop_messages():
            if isinstance(msg, pyadas.SafetyWarnState):
                rows.append(
                    {
                        "t_s": t_us / 1e6,
                        "v_ego": msg.v_ego,
                        "cte_m": msg.cte_m,
                        "cte_rate_ms": msg.cte_rate_ms,
                        "fcw": bool(msg.fcw),
                        "aeb": bool(msg.aeb),
                        "lldw": bool(msg.lldw),
                        "rldw": bool(msg.rldw),
                        "threat_valid": bool(msg.threat_valid),
                        "ttc_s": msg.ttc_s,
                        "lateral_valid": bool(msg.lateral_valid),
                        "lane_anchored": bool(msg.lane_anchored),
                        "driver_steering": bool(msg.driver_steering),
                        "lead_prob": float(msg.lead_prob),
                    }
                )
    return rows


def episodes(flags: List[bool]) -> int:
    """Warning episodes, not frames: a 3 s beep is one false positive, not sixty."""
    return sum(1 for i, f in enumerate(flags) if f and not (i and flags[i - 1]))


def report(bag: Path, rows: List[Dict]) -> None:
    if not rows:
        print(f"{bag.name}: could not assemble any frames")
        return

    n = len(rows)
    seconds = rows[-1]["t_s"] - rows[0]["t_s"]
    cte = np.array([r["cte_m"] for r in rows])
    v = np.array([r["v_ego"] for r in rows])
    valid = np.array([r["lateral_valid"] for r in rows])

    old_ldw = [bool(vl and abs(c) > OLD_LDW_THRESHOLD_M) for c, vl in zip(cte, valid)]
    new_ldw = [r["lldw"] or r["rldw"] for r in rows]

    print(
        f"{bag.name}: {n} frames, {seconds / 60:.1f} min, v median {np.median(v):.1f} m/s, "
        f"|CTE| median {np.median(np.abs(cte)):.2f} m / p95 {np.percentile(np.abs(cte), 95):.2f}"
    )
    print(
        f"  LDW old rule (|cte|>{OLD_LDW_THRESHOLD_M} m):  "
        f"{episodes(old_ldw):>4} triggers, {100.0 * np.mean(old_ldw):.1f} % of time"
    )
    print(
        f"  LDW new rule:                    "
        f"{episodes(new_ldw):>4} triggers, {100.0 * np.mean(new_ldw):.1f} % of time"
    )
    # Why LDW was allowed to speak at all — a zero count means nothing without this.
    fast = np.array([r["v_ego"] >= 12.5 for r in rows])
    anchored = np.array([r["lane_anchored"] for r in rows])
    armed = fast & anchored
    print(
        f"  LDW armed: {100.0 * np.mean(armed):.0f} % of time "
        f"(v≥12.5 m/s {100.0 * np.mean(fast):.0f} %, lanes under path {100.0 * np.mean(anchored):.0f} %); "
        f"|CTE| in these frames median {np.median(np.abs(cte[armed])) if armed.any() else float('nan'):.2f} m / "
        f"max {np.max(np.abs(cte[armed])) if armed.any() else float('nan'):.2f}"
    )
    print(
        f"  FCW / AEB: {episodes([r['fcw'] for r in rows])} / {episodes([r['aeb'] for r in rows])}"
        f"   (frames with threat: {sum(r['threat_valid'] for r in rows)})"
    )
    if not any(r["lead_prob"] > 0 for r in rows):
        print("  bag has no lead vehicle data — FCW/AEB not checked on it")


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("bags", nargs="+", type=Path)
    p.add_argument("--max-dt-ms", type=int, default=120)
    p.add_argument("--csv", type=Path, default=None)
    args = p.parse_args()

    all_rows: List[Dict] = []
    for bag in args.bags:
        rows = replay(bag, args.max_dt_ms)
        report(bag, rows)
        all_rows += rows

    if args.csv and all_rows:
        with args.csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)
        print(f"CSV → {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
