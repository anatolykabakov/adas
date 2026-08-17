#!/usr/bin/env python3
"""Offline lane-keep via Simulated AdasApp (C++ PP and/or VisionPilot MPC).

From bag ``vision/lanes`` + ``vehicle/state``, recompute desired steer with the
**same path fusion as Android** (``laneLinesToPath``) and compare to measured
steering_angle_deg (no CAN TX).

Writes lane_keep_compare.csv and optional plot.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

import _path  # noqa: F401

from core.frames import DEFAULT_MAX_STEER_DEG
from core.lane_keep import LaneKeepController
from core.path_fusion import path_from_bag_lanes
from vis.bag_io import iter_aligned, list_topics, load_topic_messages


def _run_controller(
    ctrl: LaneKeepController,
    lanes,
    state,
    max_dt_ms: int,
    min_lane_prob: float,
    steer_ratio: float,
):
    rows = []
    for row in iter_aligned(lanes, {"state": state}, max_dt_ms=max_dt_ms):
        ll, st = row["primary"], row["state"]
        if st is None:
            continue
        poly = path_from_bag_lanes(ll, min_lane_prob=min_lane_prob)
        if poly is None or poly.shape[0] < 2:
            continue

        v = max(float(st.v_ego), 0.0)
        lk = ctrl.compute_from_polyline(v, poly, yaw_rate=float(st.yaw_rate))
        steer_cmd_deg = float(np.degrees(lk.steer_rad) * steer_ratio)
        steer_meas = float(st.steering_angle_deg)

        rows.append(
            {
                "t": row["t"],
                "v_ego": v,
                "yaw_rate": float(st.yaw_rate),
                "steer_cmd_deg": steer_cmd_deg,
                "steer_rad": float(lk.steer_rad),
                "steer_meas_deg": steer_meas,
                "steer_err_deg": steer_cmd_deg - steer_meas,
                "cte_m": float(lk.e_y),
                "epsi_rad": float(lk.e_psi),
                "curvature": float(lk.curvature),
                "status": lk.status,
                "controller": lk.controller,
            }
        )
    return rows


def _summarize(name: str, rows: list[dict]) -> None:
    if not rows:
        print(f"{name}: no rows")
        return
    ok = [r for r in rows if r["status"] == "ok"]
    rad = np.array([r["steer_rad"] for r in ok], dtype=float)
    cte = np.array([r["cte_m"] for r in ok], dtype=float)
    cmd = np.array([r["steer_cmd_deg"] for r in ok], dtype=float)
    err = np.array([r["steer_err_deg"] for r in ok], dtype=float)
    print(f"\n=== {name} ({len(ok)}/{len(rows)} ok) ===")
    print(
        f"  steer_rad  min/max/mean = {rad.min():+.4f} / {rad.max():+.4f} / {rad.mean():+.4f}"
    )
    print(
        f"  |steer_rad| p95         = {np.percentile(np.abs(rad), 95):.4f}  ({np.degrees(np.percentile(np.abs(rad), 95)):.1f}° wheel)"
    )
    print(
        f"  SWA_cmd    min/max/mean = {cmd.min():+.1f} / {cmd.max():+.1f} / {cmd.mean():+.1f} °"
    )
    print(
        f"  |err vs meas| mean/p95  = {np.mean(np.abs(err)):.1f} / {np.percentile(np.abs(err), 95):.1f} °"
    )
    if name.startswith("mpc") or any(r["controller"] == "mpc" for r in ok):
        print(
            f"  CTE_m      min/max/mean = {cte.min():+.3f} / {cte.max():+.3f} / {cte.mean():+.3f}"
        )
        print(f"  |CTE| p95               = {np.percentile(np.abs(cte), 95):.3f} m")
        # Adequacy heuristics
        flags = []
        if np.percentile(np.abs(rad), 95) > 0.45:
            flags.append("WARN: wheel δ often near ±25° saturation")
        if np.percentile(np.abs(cte), 50) > 1.5:
            flags.append("WARN: median |CTE| > 1.5 m (path/fit?)")
        if np.std(rad) < 1e-4:
            flags.append("WARN: δ nearly constant (solver stuck?)")
        v = np.array([float(r["v_ego"]) for r in ok], dtype=float)
        m = np.abs(cte) < 3.5
        m3 = m & (v > 3.0)
        corr = np.corrcoef(-cte[m], rad[m])[0, 1] if int(m.sum()) > 5 else float("nan")
        corr_v3 = (
            np.corrcoef(-cte[m3], rad[m3])[0, 1] if int(m3.sum()) > 5 else float("nan")
        )
        # CTE+ (right of path) → left steer (δ_device<0) ⇒ corr(-CTE, δ)>0
        print(f"  corr(-CTE, δ) all/v>3   = {corr:+.3f} / {corr_v3:+.3f}  (expect >0)")
        if not np.isfinite(corr_v3) or corr_v3 < 0.3:
            flags.append("WARN: weak/wrong CTE→steer correlation at v>3")
        if flags:
            for f in flags:
                print(f"  {f}")
        else:
            print("  adequacy: OK (ranges + CTE→steer sign look plausible)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None)
    ap.add_argument("--max-dt", type=int, default=80)
    ap.add_argument("--wheelbase", type=float, default=2.636)
    ap.add_argument("--steer-ratio", type=float, default=15.7)
    ap.add_argument("--pp-k-dd", type=float, default=0.4)
    ap.add_argument("--pp-ld-min", type=float, default=3.0)
    ap.add_argument("--pp-ld-max", type=float, default=20.0)
    ap.add_argument("--pp-shift", type=float, default=1.4)
    ap.add_argument("--max-steer-deg", type=float, default=DEFAULT_MAX_STEER_DEG)
    ap.add_argument("--min-lane-prob", type=float, default=0.3)
    ap.add_argument("--controller", choices=("pp", "mpc", "both"), default="both")
    ap.add_argument(
        "--cam-y-left",
        type=float,
        default=0.0,
        help="ISO camera y_left (m); calibration.camera.position_m.y_left (driver-side ≈ +0.10)",
    )
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    session = args.session.resolve()
    print("Topics:", list_topics(session))
    lanes = load_topic_messages(session, "vision/lanes")
    state = load_topic_messages(session, "vehicle/state")
    if not state:
        state = load_topic_messages(session, "carState")
    if not lanes or not state:
        raise SystemExit("Need vision/lanes and vehicle/state")

    modes = []
    if args.controller in ("pp", "both"):
        modes.append("pure_pursuit")
    if args.controller in ("mpc", "both"):
        modes.append("mpc")

    all_rows = []
    for mode in modes:
        ctrl = LaneKeepController(
            mode=mode,
            wheelbase=args.wheelbase,
            pp_k_dd=args.pp_k_dd,
            pp_ld_min=args.pp_ld_min,
            pp_ld_max=args.pp_ld_max,
            pp_shift=args.pp_shift,
            max_steer_deg=args.max_steer_deg,
            mpc_max_steer_deg=25.0,
            cam_y_left_m=args.cam_y_left,
        )
        rows = _run_controller(
            ctrl, lanes, state, args.max_dt, args.min_lane_prob, args.steer_ratio
        )
        tag = "mpc" if mode == "mpc" else "pp"
        suffix = f" cam_y={args.cam_y_left:+.3f}" if abs(args.cam_y_left) > 1e-9 else ""
        _summarize(f"{tag}{suffix}", rows)
        for r in rows:
            r2 = dict(r)
            r2["mode"] = tag
            all_rows.append(r2)

    if not all_rows:
        raise SystemExit("No aligned samples")

    out = args.out or (session / "lane_keep_compare.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        w.writeheader()
        w.writerows(all_rows)
    print(f"\nWrote {len(all_rows)} rows → {out}")

    if args.plot:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        for mode, color in (("pp", "C0"), ("mpc", "C1")):
            rs = [r for r in all_rows if r["mode"] == mode]
            if not rs:
                continue
            t = [r["t"] for r in rs]
            ax[0].plot(
                t, [r["steer_cmd_deg"] for r in rs], label=f"{mode} SWA_cmd", color=color
            )
            if mode == "mpc":
                ax[1].plot(t, [r["cte_m"] for r in rs], label="CTE", color=color)
        meas = [
            r for r in all_rows if r["mode"] == modes[0] and modes[0] == "pure_pursuit"
        ] or [r for r in all_rows if r["mode"] == "pp"]
        if not meas:
            meas = [r for r in all_rows if r["mode"] == "mpc"]
        # unique times for meas
        seen = set()
        tm, sm = [], []
        for r in all_rows:
            if r["t"] in seen:
                continue
            seen.add(r["t"])
            tm.append(r["t"])
            sm.append(r["steer_meas_deg"])
        ax[0].plot(tm, sm, "k--", alpha=0.5, label="meas SWA")
        ax[0].set_ylabel("SWA deg")
        ax[0].legend()
        ax[1].set_ylabel("CTE m")
        ax[1].set_xlabel("t")
        ax[1].legend()
        fig.tight_layout()
        plot_path = out.with_suffix(".png")
        fig.savefig(plot_path, dpi=120)
        print(f"Plot → {plot_path}")
        plt.close(fig)


if __name__ == "__main__":
    main()
