#!/usr/bin/env python3
"""Compare the speed sources: CAN wheel speed, the localization filter, and GNSS Doppler.

Why this matters beyond tidiness. Everything lateral is scaled by speed — the understeer
compensation is `κ = δ / (L·(1 + K·v²))`, so a 2 % speed error becomes a 4 % curvature error at the
`v²` term, and the curvature speed limit in the long plan is `sqrt(a_lat / κ)`. If wheel speed carries
a tyre-radius scale error, every one of those is biased in the same direction, and the bias would look
exactly like a wrong `tire_stiffness_factor`.

GNSS speed here is the receiver's own speed field, which on Android is Doppler-derived and good to a
few cm/s when the fix is 3D — far better than anything wheel radius gives. It arrives at ~1 Hz, so this
compares it against the other two sampled at the same instants.

What to read:

* **scale** (wheel / GNSS): 1.00 means the tyre radius in the CAN scaling is right. A consistent offset
  is a calibration constant we are missing, not noise;
* **residual spread**: how much is left after the scale, i.e. what fusing could actually win;
* **the filter against GNSS**: whether `localization/pose.v` is already better than raw CAN, which is the
  question behind using the localizer output in the planners.

Doppler speed is unreliable below a few m/s and while accelerating hard, so slow and transient samples
are excluded and the counts are reported.

Usage:
  python bag_speed_sources.py adas_logs/2026_08_06_00_36_42 [more bags...]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages


def series(bag: Path, topic: str, getters):
    rows = load_topic_messages(bag, topic)
    if not rows:
        return None
    t = np.asarray([r[0] for r in rows], dtype=np.float64)
    cols = []
    for g in getters:
        cols.append(np.asarray([g(r[1]) for r in rows], dtype=np.float64))
    return t, cols


def at(t_ref, t_src, v_src, max_dt_ms=120.0):
    """Nearest-sample lookup with a staleness limit; returns values and a validity mask."""
    idx = np.clip(np.searchsorted(t_src, t_ref), 0, len(t_src) - 1)
    idx_prev = np.clip(idx - 1, 0, len(t_src) - 1)
    take_prev = np.abs(t_src[idx_prev] - t_ref) < np.abs(t_src[idx] - t_ref)
    idx = np.where(take_prev, idx_prev, idx)
    ok = np.abs(t_src[idx] - t_ref) <= max_dt_ms
    return v_src[idx], ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument(
        "--min-speed", type=float, default=5.0, help="m/s, below this Doppler is noisy"
    )
    ap.add_argument(
        "--max-accel",
        type=float,
        default=0.5,
        help="m/s^2, drop transients where the 1 Hz GNSS sample lags the wheels",
    )
    args = ap.parse_args()

    for bag in args.bags:
        gps = series(
            bag,
            "sensors/gps/location",
            [
                lambda m: float(getattr(m, "speed", 0.0)),
                lambda m: float(getattr(m, "fix_type", 0)),
                lambda m: float(getattr(m, "horizontal_accuracy", 99.0)),
            ],
        )
        state = series(
            bag,
            "vehicle/state",
            [lambda m: float(getattr(getattr(m, "car_state", None) or m, "v_ego", 0.0))],
        )
        pose = series(bag, "localization/pose", [lambda m: float(getattr(m, "v", 0.0))])
        if gps is None or state is None:
            print(f"{bag.name}: missing topics")
            continue

        gt, (gv, fix, hacc) = gps
        st, (sv,) = state

        # The recorder writes both GPS topics on the same millisecond, so timestamps repeat. Left in,
        # a repeat makes dt zero and the derivative below NaN, and every NaN then fails the transient
        # gate — that alone cut 1081 samples to 8 the first time this ran.
        keep_unique = np.concatenate(([True], np.diff(gt) > 0))
        gt, gv, fix, hacc = (
            gt[keep_unique],
            gv[keep_unique],
            fix[keep_unique],
            hacc[keep_unique],
        )

        can_at_gps, can_ok = at(gt, st, sv)

        # Reject where Doppler is untrustworthy, and where the car is accelerating enough that a
        # 1 Hz sample and a 100 Hz one are not describing the same instant.
        dt_s = np.maximum(np.gradient(gt) * 1e-3, 1e-3)
        accel = np.gradient(gv) / dt_s
        keep = (
            can_ok
            & (gv >= args.min_speed)
            & (fix >= 3)
            & (hacc < 10.0)
            & np.isfinite(accel)
            & (np.abs(accel) < args.max_accel)
        )

        print(f"\n=== {bag.name}")
        print(
            f"GNSS samples {len(gt)}, usable after gates {int(keep.sum())} "
            f"(speed>={args.min_speed}, 3D fix, hacc<10 m, |a|<{args.max_accel})"
        )
        if keep.sum() < 30:
            print("  too few usable samples")
            continue

        g, c = gv[keep], can_at_gps[keep]
        scale = c / g
        print(
            f"  CAN wheel / GNSS Doppler: scale median {np.median(scale):.4f} "
            f"(p10 {np.percentile(scale, 10):.4f}, p90 {np.percentile(scale, 90):.4f})"
        )
        print(
            f"    → CAN reads {100 * (np.median(scale) - 1):+.2f} % against Doppler; "
            f"at 20 m/s that is {20 * (np.median(scale) - 1):+.2f} m/s"
        )
        resid = c - np.median(scale) * g
        print(
            f"  residual after removing the scale: median |{np.median(np.abs(resid)):.3f}| m/s, "
            f"p90 {np.percentile(np.abs(resid), 90):.3f}"
        )
        print(
            f"  raw difference CAN − GNSS: median {np.median(c - g):+.3f} m/s, "
            f"p90 |{np.percentile(np.abs(c - g), 90):.3f}|"
        )

        if pose is not None:
            pt, (pv,) = pose
            pose_at_gps, pose_ok = at(gt, pt, pv)
            k2 = keep & pose_ok
            if k2.sum() > 30:
                pg, pp = gv[k2], pose_at_gps[k2]
                print(
                    f"  localization/pose.v / GNSS: scale median {np.median(pp / pg):.4f}, "
                    f"raw difference median {np.median(pp - pg):+.3f} m/s, "
                    f"p90 |{np.percentile(np.abs(pp - pg), 90):.3f}|"
                )
                print(
                    "    the filter is fed CAN speed as its velocity measurement, so it inherits the\n"
                    "    same scale — a matching number here means fusing has not added information,\n"
                    "    it has only smoothed."
                )

        # Speed dependence: a tyre-radius error is a constant scale, a slip or sensor issue is not.
        print(f"  {'speed band':>14} {'n':>5} {'scale':>8}")
        for lo, hi in [(5, 10), (10, 15), (15, 20), (20, 25), (25, 40)]:
            m = (g >= lo) & (g < hi)
            if m.sum() >= 15:
                print(
                    f"  {f'{lo}-{hi} m/s':>14} {int(m.sum()):>5} {np.median(scale[m]):>8.4f}"
                )
        print(
            "  a flat scale across bands is a wheel-radius constant; a sloping one is not."
        )


if __name__ == "__main__":
    main()
