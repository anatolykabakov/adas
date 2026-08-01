#!/usr/bin/env python3
"""Open-loop A/B of the lane-keep controllers over a bag window.

Feeds the recorded `vision/lanes` + `vehicle/state` into the real C++ `LaneKeepService`
(via ``pyadas``) once per controller and compares the commanded steering-wheel angle to the
driver's rack:

  fp   — ADAS port of the flowpilot MPC (Eigen gradient descent, N=16, slew guard)
  mpc  — VisionPilot spatial MPC (FF-only seed per config.json)

Metrics on the clean mask (controls_allowed, no driver torque, v>3 m/s):
  |err|med / p95 vs rack, correlation, and HF = median |ΔSWA| between frames (chatter).

    ./bag_controller_ab.py adas_logs/<session> --t0 753 --t1 860 --plot out.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401

from bag_mpc_sim import BagWindow, _nearest, run_pyadas_openloop

CONTROLLERS = ("fp", "mpc")
STYLE = {
    "fp": ("C3", 1.4),
    "mpc": ("C0", 1.4),
}


def run_controller(win: BagWindow, name: str) -> np.ndarray:
    """Config.json defaults per controller, so this matches what runs on the phone."""
    kw = dict(
        ff_scale=1.0,
        kappa_yaw_blend=0.4,
        epsi_gain=0.0,
        cte_gain_base=0.0,
        cte_gain_floor=0.0,
        kappa_ema=0.5,
        epsi_ema=0.5,
        cte_ema=0.5,
    )
    return run_pyadas_openloop(
        win,
        kw["ff_scale"],
        kw["kappa_yaw_blend"],
        epsi_gain=kw["epsi_gain"],
        cte_gain_floor=kw["cte_gain_floor"],
        cte_gain_base=kw["cte_gain_base"],
        kappa_ema=kw["kappa_ema"],
        epsi_ema=kw["epsi_ema"],
        cte_ema=kw["cte_ema"],
        controller=name,
    )


def gain_and_lag(cmd: np.ndarray, ref: np.ndarray, mask: np.ndarray, dt_s: float = 0.118):
    """Least-squares gain vs the driver and the lag that maximises correlation.

    Positive lag = the controller trails the driver.
    """
    c, r = cmd[mask], ref[mask]
    ok = np.isfinite(c) & np.isfinite(r)
    if ok.sum() < 20:
        return float("nan"), float("nan")
    c, r = c[ok] - np.mean(c[ok]), r[ok] - np.mean(r[ok])
    gain = float(np.dot(c, r) / np.dot(r, r)) if np.dot(r, r) > 0 else float("nan")
    best, best_k = -2.0, 0
    for k in range(0, 26):  # up to ~3 s at ~8.5 Hz lane rate
        a, b = (c[k:], r[: len(r) - k]) if k else (c, r)
        if len(a) < 20:
            break
        cc = float(np.corrcoef(a, b)[0, 1])
        if cc > best:
            best, best_k = cc, k
    return gain, best_k * dt_s


def hf(series: np.ndarray, mask: np.ndarray) -> float:
    """Median |Δ| between consecutive in-mask frames — the chatter metric used in docs/."""
    idx = np.where(mask)[0]
    if idx.size < 3:
        return float("nan")
    keep = idx[1:][np.diff(idx) == 1]
    if keep.size < 2:
        return float("nan")
    d = np.abs(series[keep] - series[keep - 1])
    d = d[np.isfinite(d)]
    return float(np.median(d)) if d.size else float("nan")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path)
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--plot", type=Path, default=None)
    ap.add_argument("--controllers", default=",".join(CONTROLLERS))
    args = ap.parse_args()

    win = BagWindow(args.session.resolve(), args.t0, args.t1)
    names = [c.strip() for c in args.controllers.split(",") if c.strip()]
    print(f"window {args.t0}–{args.t1}s  lanes={len(win.lanes)}  state={len(win.state)}")

    runs = {}
    for name in names:
        rows = run_controller(win, name)
        print(f"  {name}: {len(rows)} frames")
        if len(rows) >= 5:
            runs[name] = rows
    if not runs:
        raise SystemExit("no controller produced frames")

    common = None
    for rows in runs.values():
        common = rows[:, 0] if common is None else np.intersect1d(common, rows[:, 0])
    t_ms = common
    swa = {n: r[np.searchsorted(r[:, 0], t_ms), 1] for n, r in runs.items()}
    kappa = {n: r[np.searchsorted(r[:, 0], t_ms), 4] for n, r in runs.items()}

    driver = np.array([_nearest(x, win.state, 5) for x in t_ms])
    logged = np.array([_nearest(x, win.dbg, 1) for x in t_ms])
    v = np.array([_nearest(x, win.state, 1) for x in t_ms])
    yaw = np.array([_nearest(x, win.state, 2) for x in t_ms])
    pressed = np.array([_nearest(x, win.state, 3) for x in t_ms])
    torque = np.array([_nearest(x, win.state, 4) for x in t_ms])
    ca = np.array([_nearest(x, win.panda, 1, 300.0) for x in t_ms])

    clean = (
        (ca > 0.5)
        & ~((pressed > 0.5) | (np.abs(torque) > 50.0))
        & (v > 3.0)
        & np.isfinite(driver)
    )
    # HCA off for the whole window (open-loop shadow): the driver is the reference, so fall
    # back to "moving" as the base slice instead of dropping every frame.
    base = clean if clean.sum() >= 20 else (v > 3.0) & np.isfinite(driver)
    shadow = clean.sum() < 20
    straight = base & (np.abs(yaw) < 0.03)
    curve = base & (np.abs(yaw) >= 0.03)

    print(
        f"\nframes={len(t_ms)}  clean(HCA)={int(clean.sum())}  base={int(base.sum())}"
        f"{'  [shadow: driver steering]' if shadow else ''}  straight={int(straight.sum())}  "
        f"curve={int(curve.sum())}  v={np.nanmedian(v[base]):.1f} m/s"
    )
    print(
        f"\n{'slice':16s}{'controller':12s}{'n':>6s}{'|err|med':>10s}{'p95':>8s}{'corr':>7s}{'HF':>7s}{'gain':>7s}{'lag,s':>7s}"
    )
    for label, mask in (("all", base), ("straights", straight), ("curves", curve)):
        if mask.sum() < 5:
            continue
        for n in list(runs) + ["driver", "logged"]:
            s = {"driver": driver, "logged": logged}.get(n, swa.get(n))
            if s is None or not np.isfinite(s[mask]).any():
                continue
            if n in ("driver",):
                print(
                    f"{label:16s}{n:12s}{int(mask.sum()):6d}{'—':>10s}{'—':>8s}{'—':>7s}"
                    f"{hf(s, mask):7.2f}{'—':>7s}{'—':>7s}"
                )
                continue
            err = np.abs(s[mask] - driver[mask])
            ok = np.isfinite(err)
            corr = (
                float(np.corrcoef(s[mask][ok], driver[mask][ok])[0, 1])
                if ok.sum() > 2
                else np.nan
            )
            g, lag = gain_and_lag(s, driver, mask)
            print(
                f"{label:16s}{n:12s}{int(ok.sum()):6d}{np.median(err[ok]):10.2f}"
                f"{np.percentile(err[ok], 95):8.2f}{corr:7.3f}{hf(s, mask):7.2f}{g:7.2f}{lag:7.2f}"
            )

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        tt = (t_ms - t_ms[0]) / 1000.0
        t_abs = t_ms / 1000.0
        fig, ax = plt.subplots(
            4,
            1,
            figsize=(16, 12),
            sharex=True,
            gridspec_kw={"height_ratios": [3, 2, 1.4, 1.2]},
        )
        ax[0].plot(tt, driver, "k-", lw=2.0, label="driver (rack)")
        ax[0].plot(
            tt, logged, color="#999", lw=0.9, alpha=0.8, label="on phone (fp, logged)"
        )
        for n in runs:
            c, lw = STYLE.get(n, ("C4", 1.2))
            ax[0].plot(tt, swa[n], color=c, lw=lw, label=f"{n}")
        pad = np.concatenate([[False], base if not shadow else clean, [False]])
        starts, ends = np.where(~pad[:-1] & pad[1:])[0], np.where(pad[:-1] & ~pad[1:])[0]
        for i, (s, e) in enumerate(zip(starts, ends)):
            ax[0].axvspan(
                tt[s],
                tt[min(e - 1, len(tt) - 1)],
                color="C2",
                alpha=0.07,
                label="clean (HCA, no driver)" if i == 0 else None,
            )
        ax[0].set_ylabel("SWA, deg")
        ax[0].legend(ncol=3, fontsize=9)
        ax[0].grid(alpha=0.3)
        span = (
            f"{args.t0:.0f}–{args.t1:.0f} s"
            if args.t0 is not None and args.t1 is not None
            else "full bag"
        )
        ax[0].set_title(f"{args.session.name}  {span} — steering angle")

        for n in runs:
            c, lw = STYLE.get(n, ("C4", 1.2))
            ax[1].plot(tt, swa[n] - driver, color=c, lw=lw, label=f"{n} − driver")
        ax[1].axhline(0, color="k", lw=0.8)
        ax[1].set_ylabel("error, deg")
        ax[1].legend(ncol=3, fontsize=9)
        ax[1].grid(alpha=0.3)

        for n in runs:
            c, lw = STYLE.get(n, ("C4", 1.2))
            ax[2].plot(tt, kappa[n], color=c, lw=lw, label=f"κ {n}")
        ax[2].plot(
            tt,
            yaw / np.maximum(v, 1.0),
            "k-",
            lw=0.9,
            alpha=0.6,
            label="κ from driver yaw",
        )
        ax[2].set_ylabel("κ, 1/m")
        ax[2].legend(ncol=4, fontsize=8)
        ax[2].grid(alpha=0.3)

        ax[3].plot(tt, v, "C1", lw=1.2, label="v, m/s")
        ax[3].plot(tt, np.degrees(yaw), "C4", lw=0.9, alpha=0.7, label="yaw rate, °/s")
        ax[3].set_ylabel("v / yaw")
        ax[3].set_xlabel(f"s from {t_abs[0]:.0f} s into bag")
        ax[3].legend(ncol=2, fontsize=9)
        ax[3].grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"\nplot -> {args.plot}")


if __name__ == "__main__":
    main()
