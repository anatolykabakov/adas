#!/usr/bin/env python3
"""Dynamic pitch/yaw calibration from vanishing point (AAD method).

Uses camera JPEGs (+ optional straight-motion gate from vehicle/state).
Writes ``calib_rpy.json`` with pitch_deg, yaw_deg (roll=0).

Reference: Algorithms-for-Automated-Driving
  ``calibrated_lane_detector.py`` → ``get_intersection`` / ``get_py_from_vp``.

Usage:
  python3 bag_calib_rpy.py /path/to/session
  python3 bag_calib_rpy.py /path/to/session -o calib_rpy.json --stride 2
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

import _path  # noqa: F401

from core.lane_projection import intrinsics_from_messages
from core.vanishing_point_calib import (
    K_from_fx_fy_cx_cy,
    VanishingPointCalibrator,
    lines_from_image_hough,
)
from vis.bag_io import iter_aligned, list_topics, load_topic_messages


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None)
    ap.add_argument("--max-dt", type=int, default=100)
    ap.add_argument("--v-min", type=float, default=5.0, help="m/s min (straight gate)")
    ap.add_argument("--yaw-max", type=float, default=0.08, help="rad/s max |yaw_rate|")
    ap.add_argument("--stride", type=int, default=2, help="use every Nth camera frame")
    ap.add_argument(
        "--history", type=int, default=50, help="AAD history before mean commit"
    )
    ap.add_argument(
        "--no-gate", action="store_true", help="ignore vehicle/state straight gate"
    )
    args = ap.parse_args()

    session = args.session.resolve()
    print("Topics:", list_topics(session))

    cams = load_topic_messages(session, "sensors/camera/image")
    if not cams:
        raise SystemExit("Need sensors/camera/image")
    intr = load_topic_messages(session, "camera/intrinsics")
    intr_msg = intr[0][1] if intr else None
    state = load_topic_messages(session, "vehicle/state")

    calib = VanishingPointCalibrator(history_len=args.history)
    n_used = 0
    n_straight = 0
    commits: list = []

    others = {} if args.no_gate or not state else {"state": state}
    for i, row in enumerate(iter_aligned(cams, others, max_dt_ms=args.max_dt)):
        if args.stride > 1 and (i % args.stride) != 0:
            continue
        cam = row["primary"]
        st = row.get("state")
        if not args.no_gate and state:
            if st is None:
                continue
            v = float(getattr(st, "v_ego", 0.0) or 0.0)
            yr = float(getattr(st, "yaw_rate", 0.0) or 0.0)
            if v < args.v_min or abs(yr) > args.yaw_max:
                continue
            n_straight += 1

        arr = np.frombuffer(cam.image_data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            continue

        K_intr = intrinsics_from_messages(cam, intr_msg)
        fx, fy = K_intr.fx, K_intr.fy
        if abs(fx / max(fy, 1e-6) - 1.0) > 0.15:
            fy = fx
        K = K_from_fx_fy_cx_cy(fx, fy, K_intr.cx, K_intr.cy)

        line_l, line_r, _ = lines_from_image_hough(img)
        if line_l is None or line_r is None:
            continue
        n_used += 1
        if calib.update_from_lines(line_l, line_r, K):
            commits.append(calib.to_dict())
            print(
                f"  commit #{calib.n_updates}: "
                f"pitch={calib.estimated_pitch_deg:.2f}° yaw={calib.estimated_yaw_deg:.2f}°"
            )

    result = calib.to_dict()
    result.update(
        {
            "n_frames_used": n_used,
            "n_straight_gated": n_straight,
            "n_commits": len(commits),
            "history_len": args.history,
            "session": str(session),
        }
    )
    # Also store radians for older consumers of calib_rpy.json
    result["roll"] = 0.0
    result["pitch"] = float(np.deg2rad(calib.estimated_pitch_deg))
    result["yaw"] = float(np.deg2rad(calib.estimated_yaw_deg))

    out = args.out or (session / "calib_rpy.json")
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    print(f"Wrote {out}")
    if not calib.calibration_success:
        print(
            "WARNING: no full history commit — need more straight frames with visible lanes. "
            "Try --no-gate or lower --v-min."
        )


if __name__ == "__main__":
    main()
