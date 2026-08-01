#!/usr/bin/env python3
"""Overlay vision/lanes onto bag camera JPEGs (Golf 7 extrinsics + pinhole)."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

import _path  # noqa: F401

from core.lane_projection import (
    CameraExtrinsics,
    explain_golf7_model,
    intrinsics_from_messages,
    overlay_lanes,
)
from vis.bag_io import iter_aligned, list_topics, load_topic_messages


def decode_jpeg(cam) -> "cv2.Mat":
    import numpy as np

    arr = np.frombuffer(cam.image_data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("Failed to decode JPEG")
    return img


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path, help="Bag session dir")
    ap.add_argument("-o", "--out", type=Path, default=None, help="Output dir")
    ap.add_argument("--max-dt", type=int, default=100, help="Max |t_cam - t_lanes| ms")
    ap.add_argument("--limit", type=int, default=0, help="Max frames (0=all)")
    ap.add_argument("--cam-h", type=float, default=1.40, help="Camera height (m)")
    ap.add_argument(
        "--cam-x", type=float, default=1.70, help="Camera X forward of ego (m)"
    )
    ap.add_argument("--pitch-deg", type=float, default=6.0, help="Pitch down (deg)")
    ap.add_argument(
        "--explain", action="store_true", help="Print geometry model and exit"
    )
    args = ap.parse_args()

    if args.explain:
        print(explain_golf7_model())
        return

    session = args.session.resolve()
    out_dir = args.out or (session / "overlay")
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Topics:", list_topics(session))
    print(explain_golf7_model())

    cams = load_topic_messages(session, "sensors/camera/image")
    lanes = load_topic_messages(session, "vision/lanes")
    intr = load_topic_messages(session, "camera/intrinsics")
    intr_msg = intr[0][1] if intr else None

    if not cams:
        raise SystemExit("No sensors/camera/image in session")
    if not lanes:
        raise SystemExit("No vision/lanes in session")

    extrinsics = CameraExtrinsics.golf7_windshield(height_m=args.cam_h)
    extrinsics.x = args.cam_x
    extrinsics.pitch = float(__import__("numpy").deg2rad(args.pitch_deg))

    n = 0
    for row in iter_aligned(cams, {"lanes": lanes}, max_dt_ms=args.max_dt):
        cam = row["primary"]
        ll = row["lanes"]
        if ll is None:
            continue
        img = decode_jpeg(cam)
        K = intrinsics_from_messages(cam, intr_msg)
        overlay_lanes(img, ll, K, extrinsics)
        path = out_dir / f"{row['t']:013d}.jpg"
        cv2.imwrite(str(path), img)
        n += 1
        if args.limit and n >= args.limit:
            break

    print(f"Wrote {n} overlays → {out_dir}")


if __name__ == "__main__":
    main()
