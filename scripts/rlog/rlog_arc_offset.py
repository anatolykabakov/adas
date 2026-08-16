#!/usr/bin/env python3
"""Lane offset from openpilot/dragonpilot logs — same metric as `bag/bag_arc_offset.py`.

Used to compare our lateral stack with upstream **on the same car and roads**, not via simulation.
Computes the same decomposition:

    offset from lane center = tracking error + setpoint offset

rlog sources:

* `modelV2.laneLines[1,2].y` — host lines, right-positive frame (left line negative).
  Lane center and curvature — quadratic fit on x∈[0,30] with value at x=0, same as ours: do not
  average "ahead"; on an arc the center itself shifts by ½κd².
* `modelV2.position.y` — model plan.
* **`lateralPlan.dPathPoints[0]` — reference at the vehicle**, i.e. plan after lane blend,
  camera shift, and centering term. Vehicle is at y = 0 in this frame, so `dPathPoints[0]` is
  tracking error with sign (left +).
* `lateralPlan.curvatures` — full curvature trajectory (17 nodes) that they publish and we do not.
* `controlsState` (100 Hz) — fast-loop `desiredCurvature`, `enabled`/`active`.
* `carState` — speed, steering angle, hands on wheel.

  OPENPILOT_ROOT=/path/to/openpilot python3 rlog/rlog_arc_offset.py <route directory> --cache /tmp/rlog.npz
"""

from __future__ import annotations

import argparse
import bz2
import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

FIT_X_MAX = 30.0
STRAIGHT_K = 0.002
ARC_K = 0.004


def _openpilot_root() -> Path:
    """Checkout with cereal/log.capnp (openpilot / dragonpilot / flowpilot fork)."""
    raw = os.environ.get("OPENPILOT_ROOT") or os.environ.get("DRAGONPILOT_ROOT")
    if not raw:
        raise SystemExit(
            "Set OPENPILOT_ROOT (or DRAGONPILOT_ROOT) to a checkout that contains "
            "cereal/log.capnp, or pass --op-root."
        )
    root = Path(raw).expanduser().resolve()
    if not (root / "cereal" / "log.capnp").is_file():
        raise SystemExit(f"cereal/log.capnp not found under {root}")
    return root


COLS = [
    "t",
    "mid0",
    "k_mid",
    "lw",
    "plan0",
    "dpath0",
    "prob_l",
    "prob_r",
    "std_l",
    "std_r",
    "use_lanes",
    "cur0",
    "v",
    "swa",
    "pressed",
    "enabled",
    "active",
    "des_curv",
    "act_curv",
    "lat_active",
    "steer_cmd",
]


def load_schema(op_root: Path):
    import capnp

    capnp.remove_import_hook()
    return capnp.load(
        str(op_root / "cereal/log.capnp"),
        imports=[str(op_root / "cereal"), str(op_root)],
    )


def fit_at_zero(x: np.ndarray, y: np.ndarray) -> Tuple[float, float]:
    m = (x >= 0.0) & (x <= FIT_X_MAX) & np.isfinite(y)
    if m.sum() < 5:
        return float("nan"), float("nan")
    c = np.polyfit(x[m], y[m], 2)
    return float(c[2]), float(2.0 * c[0])


def nearest(ts: np.ndarray, rows: List[tuple], t: float, max_dt: float = 0.15):
    """Nearest row in time, else None."""
    if ts.size == 0:
        return None
    i = int(np.searchsorted(ts, t))
    best = None
    for j in (i - 1, i):
        if 0 <= j < ts.size and (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
            best = j
    return rows[best] if best is not None and abs(ts[best] - t) <= max_dt else None


def extract(root: Path, op_root: Path) -> np.ndarray:
    log = load_schema(op_root)
    segs = sorted(root.rglob("rlog.bz2"))
    if not segs:
        raise SystemExit(f"no rlog.bz2 in {root}")
    print(f"segments: {len(segs)}")
    rows: List[tuple] = []
    for n, seg in enumerate(segs, 1):
        model: List[tuple] = []
        lat: List[tuple] = []
        car: List[tuple] = []
        ctl: List[tuple] = []
        ccl: List[tuple] = []
        try:
            data = bz2.open(seg, "rb").read()
        except Exception:
            d = bz2.BZ2Decompressor()
            chunks = []
            try:
                with open(seg, "rb") as fh:
                    while True:
                        blk = fh.read(1 << 20)
                        if not blk:
                            break
                        chunks.append(d.decompress(blk))
            except Exception:
                pass
            data = b"".join(chunks)
            if not data:
                print(f"  [{n}/{len(segs)}] {seg.parent.name}: unreadable")
                continue
        try:
            events = list(log.Event.read_multiple_bytes(data))
        except Exception:
            events = []
            try:
                for e in log.Event.read_multiple_bytes(data):
                    events.append(e)
            except Exception:
                pass
        for msg in events:
            w = msg.which()
            t = msg.logMonoTime / 1e9
            if w == "modelV2":
                m = msg.modelV2
                if len(m.laneLines) < 3:
                    continue
                model.append((t, m))
            elif w == "lateralPlan":
                lat.append((t, msg.lateralPlan))
            elif w == "carState":
                cs = msg.carState
                car.append(
                    (
                        t,
                        float(cs.vEgo),
                        float(cs.steeringAngleDeg),
                        bool(cs.steeringPressed),
                    )
                )
            elif w == "carControl":
                cc = msg.carControl
                ccl.append((t, bool(cc.latActive), float(cc.actuators.steer)))
            elif w == "controlsState":
                c = msg.controlsState
                ctl.append(
                    (
                        t,
                        bool(c.enabled),
                        bool(c.active),
                        float(c.desiredCurvature),
                        float(getattr(c, "curvature", 0.0) or 0.0),
                    )
                )
        lat_ts = np.array([r[0] for r in lat])
        car_ts = np.array([r[0] for r in car])
        ctl_ts = np.array([r[0] for r in ctl])
        ccl_ts = np.array([r[0] for r in ccl])
        for t, m in model:
            x = np.asarray(list(m.laneLines[1].x), dtype=np.float64)
            yl = np.asarray(list(m.laneLines[1].y), dtype=np.float64)
            yr = np.asarray(list(m.laneLines[2].y), dtype=np.float64)
            if x.size != yl.size or x.size != yr.size or x.size < 6:
                continue
            mid0, k_mid = fit_at_zero(x, 0.5 * (yl + yr))
            sel = (x >= 5.0) & (x <= 40.0)
            lw = float(np.mean(np.abs(yr[sel] - yl[sel]))) if sel.any() else np.nan
            px = np.asarray(list(m.position.x), dtype=np.float64)
            py = np.asarray(list(m.position.y), dtype=np.float64)
            plan0 = fit_at_zero(px, py)[0] if px.size == py.size >= 6 else np.nan
            probs = list(m.laneLineProbs)
            stds = list(m.laneLineStds)
            cc = nearest(ccl_ts, ccl, t)
            lp = nearest(lat_ts, lat, t)
            cr = nearest(car_ts, car, t)
            ct = nearest(ctl_ts, ctl, t)
            if lp is None or cr is None or ct is None or cc is None:
                continue
            dpath = list(lp[1].dPathPoints)
            curv = list(lp[1].curvatures)
            rows.append(
                (
                    t,
                    mid0,
                    k_mid,
                    lw,
                    plan0,
                    float(dpath[0]) if dpath else np.nan,
                    float(probs[1]) if len(probs) > 1 else 0.0,
                    float(probs[2]) if len(probs) > 2 else 0.0,
                    float(stds[1]) if len(stds) > 1 else np.nan,
                    float(stds[2]) if len(stds) > 2 else np.nan,
                    1.0 if bool(getattr(lp[1], "useLaneLines", False)) else 0.0,
                    float(curv[0]) if curv else np.nan,
                    cr[1],
                    cr[2],
                    1.0 if cr[3] else 0.0,
                    1.0 if ct[1] else 0.0,
                    1.0 if ct[2] else 0.0,
                    ct[3],
                    ct[4],
                    1.0 if cc[1] else 0.0,
                    cc[2],
                )
            )
        print(
            f"  [{n}/{len(segs)}] {seg.parent.parent.name}/{seg.parent.name}: "
            f"model frames {len(model)}, accumulated {len(rows)}"
        )
    return np.asarray(rows, dtype=np.float64)


def report(a: np.ndarray) -> None:
    c = {k: a[:, i] for i, k in enumerate(COLS)}
    off_lane = c["mid0"]
    track = c["dpath0"]
    ref = off_lane - track
    plan_bias = c["mid0"] - c["plan0"]
    k_road = -c["k_mid"]

    dur = (c["t"].max() - c["t"].min()) / 60.0
    print(f"\nframes {len(a)}, duration {dur:.1f} min")
    print(
        f"arc sign check: corr(κ lane, steering angle) = "
        f"{np.corrcoef(k_road[np.abs(k_road) > 0.003], c['swa'][np.abs(k_road) > 0.003])[0, 1]:+.3f}"
    )
    print(
        f"sign check: corr(κ lane, desiredCurvature) = "
        f"{np.corrcoef(k_road[np.abs(k_road) > 0.003], c['des_curv'][np.abs(k_road) > 0.003])[0, 1]:+.3f}"
        f"  (expected < 0: plan frame is right-positive)"
    )

    good = (
        (c["lat_active"] > 0.5)
        & (c["pressed"] < 0.5)
        & (c["v"] > 5.0)
        & (c["prob_l"] > 0.3)
        & (c["prob_r"] > 0.3)
        & (c["lw"] > 2.6)
        & (c["lw"] < 4.6)
        & np.isfinite(off_lane)
        & np.isfinite(track)
    )
    print(
        f"enabled {np.mean(c['enabled'] > 0.5) * 100:.0f} %, active {np.mean(c['active'] > 0.5) * 100:.0f} %, "
        f"latActive {np.mean(c['lat_active'] > 0.5) * 100:.0f} %, hands on wheel "
        f"{np.mean(c['pressed'] > 0.5) * 100:.0f} %, lanes in reference "
        f"{np.mean(c['use_lanes'] > 0.5) * 100:.0f} %"
    )
    print(
        f"under control, hands off wheel, lanes visible: {good.sum()} frames "
        f"({good.mean() * 100:.0f} %)"
    )

    print(
        f"\n{'segment':<22} {'n':>6} {'total':>7} {'tracking':>9} {'setpoint':>8} {'plan':>7} {'blend':>5} {'σ l/r':>10}"
    )
    for lab, sel in (
        ("straight |κ|<0.002", np.abs(k_road) < STRAIGHT_K),
        ("left arc κ>0.004", k_road >= ARC_K),
        ("right arc κ<−0.004", k_road <= -ARC_K),
    ):
        m = good & sel
        if m.sum() < 20:
            print(f"{lab:<22} {m.sum():>6}  too few frames")
            continue
        print(
            f"{lab:<22} {m.sum():>6} {np.median(off_lane[m]):>+7.2f} {np.median(track[m]):>+9.2f} "
            f"{np.median(ref[m]):>+8.2f} {np.median(plan_bias[m]):>+7.2f} {np.mean(c['use_lanes'][m]):>5.2f} "
            f"{np.nanmedian(c['std_l'][m]):>5.2f}/{np.nanmedian(c['std_r'][m]):<4.2f}"
        )
        print(
            f"{'':<22} {'':>6} |abs| {np.median(np.abs(off_lane[m])):.2f}, "
            f"p90 {np.percentile(np.abs(off_lane[m]), 90):.2f}, width {np.median(c['lw'][m]):.2f} m, "
            f"v {np.median(c['v'][m]):.0f} m/s"
        )


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("root", type=Path)
    p.add_argument("--cache", type=Path, default=None)
    p.add_argument(
        "--op-root",
        type=Path,
        default=None,
        help="openpilot/dragonpilot checkout with cereal/ (or set OPENPILOT_ROOT)",
    )
    args = p.parse_args()
    if args.cache and args.cache.exists():
        a = np.load(args.cache)["data"]
        print(f"from cache {args.cache}")
    else:
        op_root = (
            args.op_root.expanduser().resolve()
            if args.op_root is not None
            else _openpilot_root()
        )
        if args.op_root is not None and not (op_root / "cereal" / "log.capnp").is_file():
            raise SystemExit(f"cereal/log.capnp not found under {op_root}")
        a = extract(args.root, op_root)
        if args.cache:
            np.savez_compressed(args.cache, data=a)
            print(f"cache → {args.cache}")
    report(a)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
