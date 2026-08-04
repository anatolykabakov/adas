#!/usr/bin/env python3
"""Analyze middleware/stats from an ADAS bag session.

Prefers per-timer rows (TimerTiming). Falls back to service-level dt for older bags.

Usage:
  python3 bag_middleware_stats.py /path/to/session_dir
  python3 bag_middleware_stats.py /path/to/session_dir --csv out.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR / "vis"))
sys.path.insert(0, str(SCRIPT_DIR / "proto"))

from bag_io import load_topic_messages  # noqa: E402


def _timer_rows(svc: Any) -> List[Any]:
    timers = list(getattr(svc, "timers", []) or [])
    if timers:
        return timers

    # Legacy bags: synthesize one row from service-level fields.
    class _T:
        name = ""
        period_ms = float(getattr(svc, "period_ms", 0) or 0)
        mean_dt_ms = float(getattr(svc, "mean_dt_ms", 0) or 0)
        max_dt_ms = float(getattr(svc, "max_dt_ms", 0) or 0)
        lagging = bool(getattr(svc, "lagging", False))
        fired = int(getattr(svc, "timers_fired", 0) or 0)

    # Always keep a row so message-only services (lane_keep) still appear.
    return [_T()]


def summarize(session: Path) -> Tuple[List[Dict[str, Any]], Dict[str, Dict[str, float]]]:
    rows_raw = load_topic_messages(session, "middleware/stats")
    if not rows_raw:
        raise SystemExit(
            f"No middleware/stats in {session} (is logging enabled? rebuild app?)"
        )

    per_row: Dict[str, Dict[str, float]] = defaultdict(
        lambda: {
            "samples": 0,
            "lagging_samples": 0,
            "max_cb_ms": 0.0,
            "max_dt_ms": 0.0,
            "sum_mean_cb_ms": 0.0,
            "sum_mean_dt_ms": 0.0,
            "period_ms": 0.0,
            "last_drop": 0.0,
            "last_msg": 0.0,
            "last_tim": 0.0,
            "last_fired": 0.0,
        }
    )
    flat: List[Dict[str, Any]] = []

    for ts, payload, _ in rows_raw:
        if payload is None:
            continue
        dropped_total = getattr(payload, "dropped_total", 0)
        any_lag = getattr(payload, "any_lagging", False)
        for s in getattr(payload, "services_timing", []):
            svc_name = s.name or "?"
            mean_cb = float(s.mean_cb_ms)
            max_cb = float(s.max_cb_ms)
            for t in _timer_rows(s):
                tname = getattr(t, "name", "") or ""
                key = f"{svc_name}/{tname}" if tname else svc_name
                st = per_row[key]
                st["samples"] += 1
                if t.lagging:
                    st["lagging_samples"] += 1
                st["max_cb_ms"] = max(st["max_cb_ms"], max_cb)
                st["max_dt_ms"] = max(st["max_dt_ms"], float(t.max_dt_ms))
                st["sum_mean_cb_ms"] += mean_cb
                st["sum_mean_dt_ms"] += float(t.mean_dt_ms)
                st["period_ms"] = float(t.period_ms)
                st["last_drop"] = float(s.dropped)
                st["last_msg"] = float(s.messages_processed)
                st["last_tim"] = float(s.timers_fired)
                st["last_fired"] = float(getattr(t, "fired", 0) or 0)
                flat.append(
                    {
                        "ts_ms": ts,
                        "service": svc_name,
                        "timer": tname,
                        "running": int(s.running),
                        "lagging": int(t.lagging),
                        "any_lagging": int(any_lag),
                        "dropped_total": int(dropped_total),
                        "dropped": int(s.dropped),
                        "messages": int(s.messages_processed),
                        "timers": int(s.timers_fired),
                        "fired": int(getattr(t, "fired", 0) or 0),
                        "inbox": int(s.inbox_depth),
                        "backlog": int(s.backlog_depth),
                        "mean_cb_ms": mean_cb,
                        "max_cb_ms": max_cb,
                        "period_ms": float(t.period_ms),
                        "mean_dt_ms": float(t.mean_dt_ms),
                        "max_dt_ms": float(t.max_dt_ms),
                    }
                )
    return flat, per_row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path, help="Bag session directory")
    ap.add_argument("--csv", type=Path, default=None, help="Write per-sample CSV")
    args = ap.parse_args()

    flat, per_row = summarize(args.session.resolve())
    if not flat:
        raise SystemExit("middleware/stats present but empty")

    print(f"session: {args.session}")
    print(f"samples: {len({r['ts_ms'] for r in flat})} ticks, {len(flat)} timer-rows")
    print()
    print(
        f"{'service/timer':22} {'lag%':>6} {'period':>7} {'avg_cb':>8} {'max_cb':>8} "
        f"{'avg_dt':>8} {'max_dt':>8} {'drop':>6} {'msg':>8} {'fired':>8}"
    )
    for name, st in sorted(per_row.items()):
        n = max(st["samples"], 1)
        lag_pct = 100.0 * st["lagging_samples"] / n
        print(
            f"{name:22} {lag_pct:5.1f}% {st['period_ms']:7.1f} {st['sum_mean_cb_ms']/n:8.2f} "
            f"{st['max_cb_ms']:8.2f} {st['sum_mean_dt_ms']/n:8.2f} {st['max_dt_ms']:8.2f} "
            f"{st['last_drop']:6.0f} {st['last_msg']:8.0f} {st['last_fired']:8.0f}"
        )

    print()
    print("How to read:")
    print("  lag%     — share of 1 Hz ticks where that timer's mean_dt > period/0.9")
    print("  avg/max_cb — handler wall time on service thread (ms; shared across timers)")
    print("  avg/max_dt — observed wake interval for this timer only (ms)")
    print("  drop     — cumulative overflow drops (backlog full); rising = CPU starved")
    print(
        "  Phone OK if lag%~0 on control timers (panda/tx), drops flat, max_cb << period"
    )

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(flat[0].keys()))
            w.writeheader()
            w.writerows(flat)
        print(f"\nwrote {args.csv}")


if __name__ == "__main__":
    main()
