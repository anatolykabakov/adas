#!/usr/bin/env python3
"""Compare how bright recorded frames are between runs, and how bright the road part is.

Why: raising ``TARGET_FPS`` from 20 to 30 shortens the longest exposure the camera may choose from
50 ms to 33 ms. That buys pipeline rate but costs light, and at night light is what lane sigma is
made of. This script measures the cost directly instead of guessing: mean luma of the whole frame
and of the road band (lower-middle third, which is where the lane markings the model reads live).

Usage:
  python bag_frame_exposure.py adas_logs/2026_08_04_21_00_18 adas_logs/2026_08_06_00_36_42
  python bag_frame_exposure.py <bags...> --stride 40
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages


def stats(bag: Path, stride: int, limit: int):
    rows = load_topic_messages(bag, "sensors/camera/image")
    if not rows:
        return None

    full, road, sharp = [], [], []
    n_used = 0
    for i, r in enumerate(rows):
        if i % stride:
            continue
        if n_used >= limit:
            break
        data = bytes(getattr(r[1], "image_data", b"") or b"")
        if not data:
            continue
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        h, w = img.shape
        # Road band: below the horizon, excluding the bonnet strip and the outer thirds where
        # oncoming headlights and street lights dominate the average.
        band = img[int(0.55 * h) : int(0.90 * h), int(0.25 * w) : int(0.75 * w)]
        full.append(float(img.mean()))
        road.append(float(band.mean()))
        # Laplacian variance — a blur/noise proxy. Longer exposure at night trades noise for
        # motion blur, so this separates "darker" from "smearier".
        sharp.append(float(cv2.Laplacian(band, cv2.CV_64F).var()))
        n_used += 1

    if not full:
        return None
    return {
        "n": n_used,
        "frames_total": len(rows),
        "full": np.asarray(full),
        "road": np.asarray(road),
        "sharp": np.asarray(sharp),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument("--stride", type=int, default=25)
    ap.add_argument("--limit", type=int, default=400, help="frames decoded per bag")
    args = ap.parse_args()

    print(
        f"{'bag':22s} {'n':>5} {'luma full':>10} {'luma road':>10} {'road p10':>9} "
        f"{'road p90':>9} {'sharpness':>10}"
    )
    for bag in args.bags:
        s = stats(bag, args.stride, args.limit)
        if s is None:
            print(f"{bag.name:22s}   no frames")
            continue
        print(
            f"{bag.name:22s} {s['n']:>5} {np.median(s['full']):>10.1f} {np.median(s['road']):>10.1f} "
            f"{np.percentile(s['road'], 10):>9.1f} {np.percentile(s['road'], 90):>9.1f} "
            f"{np.median(s['sharp']):>10.1f}"
        )
    print("\nluma is 0–255 mean of the grey frame; road band is y 55–90 %, x 25–75 %.")
    print(
        "A darker road band at the same sharpness means shorter exposure, not a different route."
    )


if __name__ == "__main__":
    main()
