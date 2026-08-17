#!/usr/bin/env python3
"""Chessboard for intrinsics calibration — for `tools/camera_calib_chessboard.py`.

9×6 inner corners means 10×7 squares. White margin around the board is required: without it
`cv2.findChessboardCorners` cannot find outer corners. Outputs a PNG sized to the screen.

  python3 tools/make_chessboard.py -o ~/chessboard_9x6.png --screen 1920x1080
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("-o", "--out", type=Path, required=True)
    p.add_argument("--screen", default="1920x1080", help="screen resolution, WxH")
    p.add_argument("--squares", default="10x7", help="squares WxH (one fewer corners)")
    p.add_argument(
        "--margin-frac",
        type=float,
        default=0.06,
        help="white margin as fraction of height",
    )
    args = p.parse_args()

    import cv2

    sw, sh = (int(v) for v in args.screen.lower().split("x"))
    nx, ny = (int(v) for v in args.squares.lower().split("x"))
    margin = int(args.margin_frac * sh)
    # Integer pixel squares: otherwise cell edges blur and subpixel refinement lies.
    sq = min((sw - 2 * margin) // nx, (sh - 2 * margin) // ny)
    bw, bh = sq * nx, sq * ny

    img = np.full((sh, sw), 255, np.uint8)
    x0, y0 = (sw - bw) // 2, (sh - bh) // 2
    for r in range(ny):
        for c in range(nx):
            if (r + c) % 2 == 0:
                img[y0 + r * sq : y0 + (r + 1) * sq, x0 + c * sq : x0 + (c + 1) * sq] = 0
    cv2.imwrite(str(args.out), img)
    print(f"→ {args.out}")
    print(
        f"   screen {sw}×{sh}, squares {nx}×{ny} at {sq} px, board {bw}×{bh}, margin {y0} px"
    )
    print(f"   inner corners {nx - 1}×{ny - 1} — use --pattern {nx - 1}x{ny - 1}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
