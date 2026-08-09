#!/usr/bin/env python3
"""Measure what deceleration this car can actually produce without the brakes.

Why this script exists: a Golf 7 Highline has plain cruise control — no radar, no ACC, and
therefore no brake-by-wire. The only way our longitudinal plan can slow the car is to lower the
cruise set speed, after which the car closes the throttle and decelerates on engine drag alone.
So the planner must never ask for more deceleration than engine drag delivers, and the only honest
source for that number is the car itself.

Method: find intervals where the driver held neither pedal (``gas_pressed == 0``,
``brake_pressed == 0``) and speed fell monotonically. Inside each interval fit dv/dt by least
squares and bucket the result by speed. The median per bucket is the coast envelope.

Both pedals released is exactly the state the car enters when the cruise set speed drops below the
current speed, so these intervals measure the right thing even though cruise was off in them.

Usage:
  python bag_coast_decel.py adas_logs/2026_08_04_21_00_18 [more bags...]
  python bag_coast_decel.py adas_logs/* --min-dur 1.5 -o /tmp/coast.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages

# Speed buckets, m/s. Edges chosen so each covers a gear range on a DSG: crawl, city, main road,
# highway. Engine drag depends on gear (pumping losses scale with rpm), so the answer is a curve.
BUCKET_EDGES = [0.0, 5.0, 8.3, 14.0, 20.0, 30.0, 60.0]


def coast_intervals(t, v, gas, brake, min_dur, min_drop):
    """Yield (i0, i1) slices where both pedals are up long enough to fit a slope."""
    free = (~gas) & (~brake)
    i = 0
    n = len(t)
    while i < n:
        if not free[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and free[j + 1]:
            j += 1
        # Require duration, a real speed drop, and no re-acceleration inside: a coast interval that
        # contains a hill crest would average two different physics.
        if j - i >= 2 and t[j] - t[i] >= min_dur and v[i] - v[j] >= min_drop:
            seg = v[i : j + 1]
            if np.all(np.diff(seg) <= 0.05):
                yield i, j
        i = j + 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument(
        "--min-dur", type=float, default=1.0, help="s, shortest interval kept"
    )
    ap.add_argument(
        "--min-drop", type=float, default=0.3, help="m/s, smallest speed drop kept"
    )
    ap.add_argument("-o", type=Path, help="CSV of individual intervals")
    args = ap.parse_args()

    rows = []
    for bag in args.bags:
        msgs = load_topic_messages(bag, "vehicle/state")
        if not msgs:
            print(f"{bag.name}: no vehicle/state", file=sys.stderr)
            continue

        t, v, gas, brake = [], [], [], []
        for ts, m, _ in msgs:
            cs = getattr(m, "car_state", None) or m
            t.append(ts * 1e-3)
            v.append(float(getattr(cs, "v_ego", 0.0)))
            gas.append(bool(getattr(cs, "gas_pressed", False)))
            brake.append(bool(getattr(cs, "brake_pressed", False)))
        t = np.asarray(t)
        v = np.asarray(v)
        gas = np.asarray(gas)
        brake = np.asarray(brake)

        n_int = 0
        for i, j in coast_intervals(t, v, gas, brake, args.min_dur, args.min_drop):
            tt, vv = t[i : j + 1], v[i : j + 1]
            slope = np.polyfit(tt - tt[0], vv, 1)[0]  # m/s², negative
            rows.append(
                {
                    "bag": bag.name,
                    "t_start": round(tt[0] - t[0], 2),
                    "dur_s": round(tt[-1] - tt[0], 2),
                    "v_start": round(vv[0], 2),
                    "v_end": round(vv[-1], 2),
                    "a_ms2": round(slope, 3),
                }
            )
            n_int += 1
        print(
            f"{bag.name}: {len(msgs)} state msgs, {n_int} coast intervals",
            file=sys.stderr,
        )

    if not rows:
        sys.exit(
            "no coast intervals found — check gas_pressed / brake_pressed are decoded in these bags"
        )

    print(f"\ncoast deceleration, {len(rows)} intervals")
    print(
        f"{'speed, m/s':>14} {'n':>4} {'median':>8} {'p10':>7} {'p90':>7}   (m/s², negative = slowing)"
    )
    curve = []
    for lo, hi in zip(BUCKET_EDGES, BUCKET_EDGES[1:]):
        a = np.asarray([r["a_ms2"] for r in rows if lo <= r["v_start"] < hi])
        if len(a) == 0:
            continue
        med = float(np.median(a))
        curve.append((0.5 * (lo + min(hi, 40.0)), med))
        print(
            f"{lo:5.1f}–{hi:<5.1f}  {len(a):>4} {med:>8.2f} "
            f"{np.percentile(a, 10):>7.2f} {np.percentile(a, 90):>7.2f}"
        )

    a_all = np.asarray([r["a_ms2"] for r in rows])
    print(
        f"\nall: median {np.median(a_all):.2f}, p10 {np.percentile(a_all, 10):.2f}, "
        f"p90 {np.percentile(a_all, 90):.2f}, strongest {a_all.min():.2f}"
    )
    print(
        "\nSuggested config: long_plan.a_coast_ms2 = "
        f"{max(-2.0, float(np.percentile(a_all, 75))):.2f}  "
        "(p75 — the plan should promise what the car makes most of the time, not its best case)"
    )

    if args.o:
        with args.o.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {args.o}")


if __name__ == "__main__":
    main()
