#!/usr/bin/env python3
"""Compare lane sigma between runs on matched conditions.

Comparing raw per-segment sigma between two runs is misleading when the routes differ: a tighter arc
pushes the far end of the lane line out of frame, so sigma grows for a reason that has nothing to do
with the camera. This script buckets frames by curvature magnitude and speed and compares within
each bucket, so a difference means the perception changed, not the road.

Sigma comes straight from the bag (``vision/lanes`` → ``lanes[i].y_std``, median over 5–40 m — the
same quantity ``laneLinesToPath`` weights blending by), so this reads what actually drove.

Usage:
  python bag/bag_lane_sigma_ab.py adas_logs/2026_08_04_21_00_18 adas_logs/2026_08_06_00_36_42
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages

KAPPA_EDGES = [0.0, 0.002, 0.004, 0.008, 0.05]
SPEED_EDGES = [5.0, 10.0, 14.0, 18.0, 40.0]


def collect(bag: Path):
    lanes = load_topic_messages(bag, "vision/lanes")
    state = load_topic_messages(bag, "vehicle/state")
    if not lanes or not state:
        return None

    # Speed lookup by binary search: vehicle/state runs at 100 Hz, so a linear scan per lane frame
    # would be 15k × 140k comparisons.
    st_ts = np.asarray([r[0] for r in state], dtype=np.float64)
    st_v = np.asarray(
        [
            float(getattr(getattr(r[1], "car_state", None) or r[1], "v_ego", 0.0))
            for r in state
        ],
        dtype=np.float64,
    )
    rows = []
    for ts, ll, _ in lanes:
        try:
            host = [ll.lanes[1], ll.lanes[2]]
        except (IndexError, AttributeError):
            continue
        stds = []
        for ln in host:
            ys = list(getattr(ln, "y_std", []) or [])
            stds.append(float(np.median(ys)) if ys else np.nan)
        if not np.isfinite(stds).all():
            continue

        # Curvature from the host lane centre, quadratic fit over 5–40 m — the same band the
        # blending weight is computed on. The x grid is shared by all four lines and lives on the
        # message, not on the polyline.
        try:
            xs = np.asarray(list(ll.x), dtype=np.float64)
            yl = np.asarray(list(ll.lanes[1].y), dtype=np.float64)
            yr = np.asarray(list(ll.lanes[2].y), dtype=np.float64)
        except (IndexError, AttributeError):
            continue
        m = (xs >= 5.0) & (xs <= 40.0)
        if m.sum() < 5 or len(yl) != len(xs) or len(yr) != len(xs):
            continue
        yc = 0.5 * (yl + yr)
        a, b, _ = np.polyfit(xs[m], yc[m], 2)
        kappa = 2.0 * a / (1.0 + b * b) ** 1.5

        j = int(np.searchsorted(st_ts, ts))
        j = min(max(j, 0), len(st_ts) - 1)
        if abs(st_ts[j] - ts) > 100:
            continue

        rows.append((abs(kappa), st_v[j], max(stds)))

    return np.asarray(rows) if rows else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_a", type=Path)
    ap.add_argument("bag_b", type=Path)
    args = ap.parse_args()

    a = collect(args.bag_a)
    b = collect(args.bag_b)
    for name, d in ((args.bag_a.name, a), (args.bag_b.name, b)):
        if d is None:
            raise SystemExit(f"{name}: no usable frames")
        print(f"{name}: {len(d)} frames with sigma")

    print(
        f"\nworst-line sigma, median (n) — A = {args.bag_a.name}, B = {args.bag_b.name}"
    )
    print(f"{'|kappa|':>16} {'speed':>12} {'A':>12} {'B':>12}   B/A")
    for k0, k1 in zip(KAPPA_EDGES, KAPPA_EDGES[1:]):
        for v0, v1 in zip(SPEED_EDGES, SPEED_EDGES[1:]):
            sel_a = a[
                (a[:, 0] >= k0) & (a[:, 0] < k1) & (a[:, 1] >= v0) & (a[:, 1] < v1), 2
            ]
            sel_b = b[
                (b[:, 0] >= k0) & (b[:, 0] < k1) & (b[:, 1] >= v0) & (b[:, 1] < v1), 2
            ]
            if len(sel_a) < 20 or len(sel_b) < 20:
                continue
            ma, mb = float(np.median(sel_a)), float(np.median(sel_b))
            print(
                f"{k0:.3f}–{k1:<.3f}".rjust(16)
                + f"{v0:.0f}–{v1:<.0f}".rjust(12)
                + f"{ma:>8.2f} ({len(sel_a):>4})".rjust(12)
                + f"{mb:>8.2f} ({len(sel_b):>4})".rjust(12)
                + f"   {mb / max(ma, 1e-6):>4.2f}"
            )

    print("\nshare of frames with worst sigma above the blending cut-off (1.5 m):")
    for name, d in ((args.bag_a.name, a), (args.bag_b.name, b)):
        print(
            f"  {name}: {100.0 * float((d[:, 2] > 1.5).mean()):.1f} %  "
            f"(median {float(np.median(d[:, 2])):.2f}, p90 {float(np.percentile(d[:, 2], 90)):.2f})"
        )


if __name__ == "__main__":
    main()
