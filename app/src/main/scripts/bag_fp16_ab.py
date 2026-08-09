#!/usr/bin/env python3
"""Does half precision change what supercombo sees? Offline A/B on recorded frames.

Why offline. Enabling `NNAPIFlags.USE_FP16` on the phone is one line and should cut inference from
45.6 ms to 25-30 ms (the traffic-sign runner already does it), taking the vision loop from 13.2 Hz to
roughly 20 Hz. The risk is not speed but accuracy: NNAPI is then free to compute in fp16, and lane
sigma is the quantity our whole lateral chain is built on. A drive cannot answer this — two drives
differ by more than fp16 does. The same frames through two sessions can.

What this measures. NNAPI's fp16 mode is not reproducible on a desktop, so this converts the model
itself to fp16 with `onnxconverter_common` and runs both on CPU. That is a *stricter* test than what
the phone does: NNAPI keeps a float32 graph and is merely permitted to relax precision per node,
while a converted model stores every weight in fp16. If the converted model holds up, the phone will.

Watch, in order of importance:

* **lane sigma** — the blending weight reads it; the arc work turns on 1.5 m thresholds, so a shift
  of even 0.1 matters;
* line probabilities and lane width — same confidence from another angle;
* plan offset from lane centre — what the controller actually follows.

  python3 bag_fp16_ab.py adas_logs/2026_08_06_00_36_42 --n 200
  python3 bag_fp16_ab.py adas_logs/<bag> --t0 620 --t1 660      # a single arc
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
from pathlib import Path
from typing import Dict

import numpy as np

import _path  # noqa: F401
from bag_intrinsics_ab import REF_W, fit_at_zero, load_frames

# Calibration is held fixed across the two runs: this compares numeric precision, nothing else.
CALIB = {"fx": 993.4, "fy": 995.2, "cx": 640.0, "cy": 360.0}


def to_fp16(model: Path, out_dir: Path) -> Path:
    import onnx
    from onnxconverter_common import float16

    dst = out_dir / (model.stem + "_fp16.onnx")
    m = onnx.load(str(model))
    # keep_io_types: inputs and outputs stay float32, exactly like the phone's tensors
    m16 = float16.convert_float_to_float16(
        m, keep_io_types=True, disable_shape_infer=True
    )
    onnx.save(m16, str(dst))
    print(
        f"fp16 model: {dst} ({dst.stat().st_size / 1e6:.0f} MB "
        f"against {model.stat().st_size / 1e6:.0f} MB float32)"
    )
    return dst


def run(
    model: Path, frames, scale: float, roll: float, pitch: float, yaw: float
) -> Dict[str, np.ndarray]:
    from core.supercombo_compare import SupercomboBev
    from core.supercombo_parse import X_IDXS

    bev = SupercomboBev(model)
    bev.set_calib(
        roll_deg=roll,
        pitch_deg=pitch,
        yaw_deg=yaw,
        fx=CALIB["fx"] * scale,
        fy=CALIB["fy"] * scale,
        cx=CALIB["cx"] * scale,
        cy=CALIB["cy"] * scale,
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
    p.add_argument("--n", type=int, default=200)
    p.add_argument("--stride", type=int, default=3)
    p.add_argument("--t0", type=float, default=-1.0)
    p.add_argument("--t1", type=float, default=1e9)
    p.add_argument("--roll", type=float, default=0.0)
    p.add_argument("--pitch", type=float, default=-0.67)
    p.add_argument("--yaw", type=float, default=0.10)
    p.add_argument("--keep", type=Path, help="keep the converted fp16 model here")
    args = p.parse_args()

    frames = load_frames(args.bag, args.n, args.t0, args.t1, max(1, args.stride))
    scale = frames[0][1].shape[1] / REF_W
    print(f"intrinsics scaled to frame: x{scale:.3f}")

    tmp = Path(tempfile.mkdtemp(prefix="fp16_ab_"))
    try:
        model16 = to_fp16(args.model, tmp)
        res = {
            "float32": run(args.model, frames, scale, args.roll, args.pitch, args.yaw),
            "fp16": run(model16, frames, scale, args.roll, args.pitch, args.yaw),
        }
        if args.keep:
            shutil.copy2(model16, args.keep)
            print(f"kept {args.keep}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    for name, r in res.items():
        print(f"  ran «{name}»: {len(r['mid0'])} frames")

    labels = [
        ("lane sigma left, median", "std_l"),
        ("lane sigma right, median", "std_r"),
        ("lane sigma left, p90", "std_l"),
        ("lane sigma right, p90", "std_r"),
        ("line prob left", "prob_l"),
        ("line prob right", "prob_r"),
        ("lane width, m", "width"),
        ("lane centre at 0 m", "mid0"),
        ("plan offset from centre", "plan_bias"),
    ]
    print(f"\n{'quantity':30} {'float32':>10} {'fp16':>10} {'delta':>10}")
    for i, (label, key) in enumerate(labels):
        a, b = res["float32"][key], res["fp16"][key]
        stat = (
            (lambda v: float(np.nanpercentile(v, 90)))
            if "p90" in label
            else (lambda v: float(np.nanmedian(v)))
        )
        va, vb = stat(a), stat(b)
        print(f"{label:30} {va:>10.3f} {vb:>10.3f} {vb - va:>+10.3f}")

    # Per-frame agreement matters more than aggregate medians: a model that is right on average and
    # wrong frame to frame is worse for the controller than a small constant bias.
    print(f"\n{'per-frame difference':30} {'median |d|':>12} {'p95 |d|':>10}")
    for label, key in (
        ("lane sigma left", "std_l"),
        ("lane sigma right", "std_r"),
        ("lane centre at 0 m", "mid0"),
        ("plan offset", "plan_bias"),
    ):
        n = min(len(res["float32"][key]), len(res["fp16"][key]))
        d = np.abs(res["float32"][key][:n] - res["fp16"][key][:n])
        print(
            f"{label:30} {float(np.nanmedian(d)):>12.4f} {float(np.nanpercentile(d, 95)):>10.4f}"
        )

    print(
        "\nRule of thumb: sigma shifting by less than 0.05 m and the lane centre by less than 0.02 m\n"
        "is below what run-to-run calibration already moves, so fp16 is free. Anything larger is a\n"
        "trade to decide explicitly, not a one-line optimisation."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
