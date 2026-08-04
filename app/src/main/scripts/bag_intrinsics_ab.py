#!/usr/bin/env python3
"""Camera intrinsics A/B on bag frames: run supercombo twice and compare output.

Why. Board calibration (2026-08-03) gave fx 993.4 vs 951 in config and principal point **off frame
center**: cx 621.9 (−18 px, −1.04° in yaw), cy 376.9 (+17 px, +0.97° in pitch). Effect can be
checked without driving: intrinsics only affect input-image warp, so one recording is enough to
run the model twice.

What to watch, in order of importance:

* **lane line σ** (`laneLineStds`) — main gap vs upstream: we see 0.19 on straights and
  0.60–0.93 in arcs, dragonpilot 0.05 (docs/BENCHMARK_COMMA2.md). Wrong input geometry
  should drop σ;
* line probabilities and lane width — same confidence from another angle;
* plan offset from lane center — the quantity that did not match across bags (50.2·κ) and is
  directly affected by principal-point error.

Bag frames are 640×360 while intrinsics are for 1280×720 — the script scales intrinsics to the
frame. Geometry is the same (same field of view); only source resolution is lost, so absolute σ
will be slightly worse than on device, but **difference between variants** is meaningful.

  python3 bag_intrinsics_ab.py adas_logs/<bag> --n 300
  python3 bag_intrinsics_ab.py adas_logs/<bag> --t0 1050 --t1 1078   # arc window
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

import _path  # noqa: F401

# Config before calibration vs board result (for 1280×720).
OLD = {"fx": 951.0, "fy": 951.0, "cx": 640.0, "cy": 360.0}
NEW = {"fx": 993.4, "fy": 995.2, "cx": 621.9, "cy": 376.9}
REF_W = 1280.0
FIT_X_MAX = 30.0

OLD_KEY = "old 951/640/360"
NEW_KEY = "board 993/622/377"


def fit_at_zero(x: np.ndarray, y: np.ndarray) -> Tuple[float, float]:
    m = (x >= 0.0) & (x <= FIT_X_MAX) & np.isfinite(y)
    if m.sum() < 5:
        return float("nan"), float("nan")
    c = np.polyfit(x[m], y[m], 2)
    return float(c[2]), float(2.0 * c[0])


def load_frames(bag: Path, n: int, t0: float, t1: float, stride: int):
    import cv2

    from vis.bag_io import load_topic_messages

    rows = load_topic_messages(bag, "sensors/camera/image")
    if not rows:
        raise SystemExit("bag has no sensors/camera/image")
    t_base = rows[0][0] / 1000.0
    out = []
    for i, r in enumerate(rows):
        ts = r[0] / 1000.0 - t_base
        if t0 >= 0 and not (t0 <= ts <= t1):
            continue
        if i % stride:
            continue
        data = bytes(getattr(r[1], "image_data", b"") or b"")
        if not data:
            continue
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            out.append((ts, img))
        if len(out) >= n:
            break
    if not out:
        raise SystemExit("no image frames in window")
    print(
        f"frames taken: {len(out)} ({out[0][1].shape[1]}×{out[0][1].shape[0]}), "
        f"from {out[0][0]:.0f} to {out[-1][0]:.0f} s"
    )
    return out


def run(
    model: Path,
    frames,
    calib: Dict[str, float],
    scale: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> Dict[str, np.ndarray]:
    from core.supercombo_compare import SupercomboBev
    from core.supercombo_parse import X_IDXS

    bev = SupercomboBev(model)
    bev.set_calib(
        roll_deg=roll,
        pitch_deg=pitch,
        yaw_deg=yaw,
        fx=calib["fx"] * scale,
        fy=calib["fy"] * scale,
        cx=calib["cx"] * scale,
        cy=calib["cy"] * scale,
    )
    acc: Dict[str, list] = {
        k: []
        for k in ("std_l", "std_r", "prob_l", "prob_r", "width", "mid0", "plan_bias")
    }
    xs = np.asarray(X_IDXS, dtype=float)
    sel = (xs >= 5.0) & (xs <= 40.0)
    for i, (_, bgr) in enumerate(frames):
        out = bev.infer(bgr, cache_key=i)
        if out is None or len(out.lanes) < 3:
            continue
        # Parser returns y left-positive; convert to right-positive frame as in bag and C++.
        yl = -np.asarray(out.lanes[1].y, dtype=float)
        yr = -np.asarray(out.lanes[2].y, dtype=float)
        if yl.size != xs.size:
            continue
        mid = 0.5 * (yl + yr)
        acc["mid0"].append(fit_at_zero(xs, mid)[0])
        acc["width"].append(float(np.mean(np.abs(yr[sel] - yl[sel]))))
        for key, idx in (("std_l", 1), ("std_r", 2)):
            v = out.lanes[idx].y_std
            acc[key].append(
                float(np.median(np.asarray(v, dtype=float)[sel]))
                if v is not None
                else np.nan
            )
        acc["prob_l"].append(float(out.lanes[1].prob))
        acc["prob_r"].append(float(out.lanes[2].prob))
        px = np.asarray(out.plan.x, dtype=float)
        py = -np.asarray(out.plan.y, dtype=float)
        acc["plan_bias"].append(
            fit_at_zero(xs, mid)[0] - fit_at_zero(px, py)[0]
            if px.size == py.size >= 6
            else np.nan
        )
    return {k: np.asarray(v, dtype=float) for k, v in acc.items()}


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("bag", type=Path)
    p.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "assets/supercombo.onnx",
    )
    p.add_argument(
        "--n", type=int, default=200, help="how many frames to run through the model"
    )
    p.add_argument("--stride", type=int, default=3, help="take every N-th frame")
    p.add_argument(
        "--t0", type=float, default=-1.0, help="time window in seconds from bag start"
    )
    p.add_argument("--t1", type=float, default=1e9)
    p.add_argument("--roll", type=float, default=0.0)
    p.add_argument("--pitch", type=float, default=-1.8)
    p.add_argument("--yaw", type=float, default=0.5)
    args = p.parse_args()

    frames = load_frames(args.bag, args.n, args.t0, args.t1, max(1, args.stride))
    scale = frames[0][1].shape[1] / REF_W
    print(f"intrinsics scaled to frame: ×{scale:.3f}")
    res = {}
    for name, calib in ((OLD_KEY, OLD), (NEW_KEY, NEW)):
        res[name] = run(args.model, frames, calib, scale, args.roll, args.pitch, args.yaw)
        print(f"  ran «{name}»: {len(res[name]['mid0'])} frames")

    print(f"\n{'metric':<34} {'before':>12} {'after board':>13} {'change':>12}")

    def line(label, key, fmt="{:.3f}", better_lower=True):
        a, b = res[OLD_KEY][key], res[NEW_KEY][key]
        a, b = a[np.isfinite(a)], b[np.isfinite(b)]
        if a.size < 5 or b.size < 5:
            print(f"{label:<34} insufficient data")
            return
        ma, mb = float(np.median(a)), float(np.median(b))
        d = (mb - ma) / abs(ma) * 100 if ma else float("nan")
        mark = ""
        if better_lower and abs(d) > 5:
            mark = "  ← better" if mb < ma else "  ← worse"
        print(
            f"{label:<34} {fmt.format(ma):>12} {fmt.format(mb):>13} {d:>+11.1f} %{mark}"
        )

    line("σ left line (median)", "std_l")
    line("σ right line", "std_r")
    line("left line probability", "prob_l", better_lower=False)
    line("right line probability", "prob_r", better_lower=False)
    line("lane width, m", "width", "{:.2f}", better_lower=False)
    line("plan offset from center, m", "plan_bias", "{:+.3f}")
    line("lane center offset at vehicle, m", "mid0", "{:+.3f}")
    print("\nMain signal — line σ: if input geometry was wrong, it should drop.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
