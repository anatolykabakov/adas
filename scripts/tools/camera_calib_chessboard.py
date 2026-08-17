#!/usr/bin/env python3
"""Camera intrinsics from chessboard — from bag or image folder.

Why. Intrinsics in `config.json` (fx = 951 at 1280×720) come from camera specs:
`fx = f_mm / sensor_width_mm · width_px`, i.e. 4.755/6.40·4000·0.16·2 = 951. That calculation has an
**unverified assumption**: that the 16:9 stream is a full-width 4:3 sensor crop by height. If the
stream is actually zoomed, real fx is larger and all model warp geometry drifts — which affects lane
σ and metric scale (camera odometry gives 0.844 of wheel speed). OnePlus 7T does not expose factory
`LENS_INTRINSIC_CALIBRATION` (five zeros), so the board is the only check.

Parameters from flowpilot (`selfdrive/calibration/java/.../CameraCalibratorIntrinsic.java`):
9×6 inner corners, 7×7 subpixel refinement, frames selected by diversity (RMSE between corner sets
above threshold), then `calibrateCamera`.

Note: the app writes 640×360 frames to the bag — integer downscale of the same 1280×720 buffer sent
to the model, so field of view is identical and fx need only be multiplied by 2.

Capture: show the board on a laptop screen, mount the phone as in the car, take 12–20 views — tilt
and move the board in frame so corners cover center and edges. Physical cell size is **not needed**:
it does not affect fx/fy/cx/cy (only extrinsic scale). Keep the screen far: the app locks focus at
infinity; close range will blur.

  python3 tools/camera_calib_chessboard.py --bag adas_logs/<bag with board>
  python3 tools/camera_calib_chessboard.py --images /path/to/folder --pattern 9x6
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Tuple

import numpy as np

CONFIG_FX_1280 = 951.0  # config.json value for 1280×720
MODEL_W = 1280  # buffer width sent to the model
SUBPIX_WIN = (7, 7)  # as in flowpilot
# flowpilot THRESHOLD_ERROR = 50 px, but their frame is 1164 wide — threshold is relative, otherwise
# on our 640 it is twice as strict and rejects good views.
DIVERSITY_RMSE_FRAC = 50.0 / 1164.0


def load_from_bag(bag: Path, limit: int) -> List[np.ndarray]:
    import cv2

    import _path  # noqa: F401
    from vis.bag_io import load_topic_messages

    rows = load_topic_messages(bag, "sensors/camera/image")
    if not rows:
        raise SystemExit(
            "bag has no sensors/camera/image (was image recording disabled?)"
        )
    out: List[np.ndarray] = []
    for _, m in [(r[0], r[1]) for r in rows]:
        data = bytes(getattr(m, "image_data", b"") or b"")
        if not data:
            continue
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if img is not None:
            out.append(img)
        if len(out) >= limit:
            break
    print(f"frames read from bag: {len(out)} ({out[0].shape[1]}×{out[0].shape[0]})")
    return out


def load_from_dir(d: Path, limit: int) -> List[np.ndarray]:
    import cv2

    files = sorted(
        [p for p in d.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")]
    )
    out = []
    for p in files[:limit]:
        img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if img is not None:
            out.append(img)
    if not out:
        raise SystemExit(f"no images in {d}")
    print(f"frames read from folder: {len(out)} ({out[0].shape[1]}×{out[0].shape[0]})")
    return out


def collect_corners(
    images: List[np.ndarray],
    cols: int,
    rows: int,
    diversity_frac: float = DIVERSITY_RMSE_FRAC,
) -> Tuple[List[np.ndarray], tuple]:
    import cv2

    pattern = (cols, rows)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    accepted: List[np.ndarray] = []
    found = 0
    size = None
    for img in images:
        size = (img.shape[1], img.shape[0])
        ok, corners = cv2.findChessboardCorners(
            img, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if not ok:
            continue
        found += 1
        corners = cv2.cornerSubPix(img, corners, SUBPIX_WIN, (-1, -1), crit)
        novel = True
        for prev in accepted:
            rmse = float(np.sqrt(np.mean(np.sum((corners - prev) ** 2, axis=2))))
            if rmse < diversity_frac * size[0]:
                novel = False
                break
        if novel:
            accepted.append(corners)
    print(
        f"board found in {found} frames, kept {len(accepted)} distinct views "
        f"(diversity threshold {diversity_frac * (size[0] if size else 0):.0f} px)"
    )
    return accepted, size


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", type=Path)
    src.add_argument("--images", type=Path)
    p.add_argument(
        "--pattern",
        default="9x6",
        help="inner corners, cols×rows (flowpilot: 9x6)",
    )
    p.add_argument("--limit", type=int, default=4000, help="how many frames to scan")
    p.add_argument(
        "--diversity",
        type=float,
        default=DIVERSITY_RMSE_FRAC,
        help="view diversity threshold as fraction of frame width",
    )
    args = p.parse_args()

    import cv2

    cols, rows = (int(v) for v in args.pattern.lower().split("x"))
    images = (
        load_from_bag(args.bag, args.limit)
        if args.bag
        else load_from_dir(args.images, args.limit)
    )
    corners, size = collect_corners(images, cols, rows, args.diversity)
    if len(corners) < 6:
        raise SystemExit(
            f"need at least 6 distinct views, have {len(corners)} — capture more, "
            f"tilting the board and moving it in frame"
        )

    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    rms, K, dist, _, _ = cv2.calibrateCamera(
        [objp] * len(corners), corners, size, None, None
    )

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    scale = MODEL_W / size[0]
    print(
        f"\nreprojection RMS error: {rms:.3f} px "
        f"({'good' if rms < 0.6 else 'high — check sharpness and view diversity'})"
    )
    print(f"\nat capture resolution {size[0]}×{size[1]}:")
    print(
        f"  fx {fx:.1f}   fy {fy:.1f}   cx {cx:.1f} (center {size[0]/2:.1f})   cy {cy:.1f} (center {size[1]/2:.1f})"
    )
    print(
        f"  distortion k1 {dist[0][0]:+.4f}  k2 {dist[0][1]:+.4f}  p1 {dist[0][2]:+.4f}  "
        f"p2 {dist[0][3]:+.4f}  k3 {dist[0][4]:+.4f}"
    )
    print(
        f"\nscaled to model buffer {MODEL_W}×{int(size[1]*scale)} (factor {scale:.2f}):"
    )
    print(
        f"  fx {fx*scale:.1f}   fy {fy*scale:.1f}   cx {cx*scale:.1f}   cy {cy*scale:.1f}"
    )
    print(
        f"\nconfig currently fx = {CONFIG_FX_1280:.0f} → discrepancy "
        f"{(fx*scale/CONFIG_FX_1280 - 1)*100:+.1f} %"
    )
    hfov = 2.0 * np.degrees(np.arctan(size[0] / (2.0 * fx)))
    hfov_cfg = 2.0 * np.degrees(np.arctan(MODEL_W / (2.0 * CONFIG_FX_1280)))
    print(f"  horizontal angle: measured {hfov:.1f}°, from config {hfov_cfg:.1f}°")
    print(
        "\nIf discrepancy exceeds 3%, update `calibration.camera.intrinsics_prior` in config.json "
        "and remeasure camera odometry scale (currently 0.844 of wheel speed)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
