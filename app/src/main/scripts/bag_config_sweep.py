#!/usr/bin/env python3
"""Closed-loop A/B of lane-keep configurations on a bag window.

Replays the real C++ controller against `core.vehicle_model.LateralPlant` (measured understeer
+ actuator/yaw lag) and scores how well each configuration holds the human's line. Use windows
where the driver was steering with clear lane markings: the recorded pose is then a trustworthy
reference. The kinematic bicycle model that `bag_mpc_sim --mode closed` used before over-predicts
the car's response by up to 65 % and hides under-steer, hence the separate plant.

    ./bag_config_sweep.py adas_logs/<session> --t0 1360 --t1 1520

Two config sets: `--set default` (the July straight-line tuning history) and `--set arcs` — the
levers for cutting the inside of a curve, measured on arc-offset bags:

    ./bag_config_sweep.py adas_logs/2026_08_02_22_02_38 --t0 662 --t1 686 \
        --set arcs --cam-y-left 0

`--cam-y-left` must match the bag: it is where the camera sat relative to the centreline
(`control/lane_keep_debug.cam_y_left_m`). It feeds both the controller and the lane-centre metric,
so a wrong value shifts every number by that amount.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401

from bag_mpc_sim import (
    BagWindow,
    _interp_pose,
    _lanes_in_sim_frame,
    _nearest,
    cross_track,
    lane_centre_offset,
)
from core.path_fusion import DEFAULT_CAMERA_OFFSET_M
from core.vehicle_model import LateralPlant
from pyadas import core as pyadas

WHEELBASE = 2.636

# (name, {knob: value}) — knobs are applied to the fresh AdasApp before the run.
CONFIGS = [
    ("A. as in run 01_14_22 (kinematic, delay 0.35)", dict(vm=False, delay=0.35)),
    ("B. + vehicle model (tsf 0.64)", dict(vm=True, delay=0.35)),
    ("C. + measured delay 0.23", dict(vm=True, delay=0.23)),
    ("D. C, but delay 0.30", dict(vm=True, delay=0.30)),
    ("E. B + lane blend 0.5", dict(vm=True, delay=0.35, blend=0.5)),
    ("F. B + lane blend 1.0", dict(vm=True, delay=0.35, blend=1.0)),
    (
        "G. B + softer steer rate penalty (200)",
        dict(vm=True, delay=0.35, rate_weight=200.0),
    ),
    ("H. B + stiffer penalty (700, stock)", dict(vm=True, delay=0.35, rate_weight=700.0)),
    (
        "I. B + path shift right 0.08 (CAMERA_OFFSET)",
        dict(vm=True, delay=0.35, cam_y=0.02),
    ),
    (
        "K. FINAL: vehicle model + shift 0.08 + lane 0.3",
        dict(vm=True, delay=0.35, cam_y=0.02, blend=0.3),
    ),
    ("L. like K, but lane blend 0.5", dict(vm=True, delay=0.35, cam_y=0.02, blend=0.5)),
]

# Levers against cutting inside on arcs. `shift` — **full** path shift to the right
# (`path_camera_offset_m`). Note: `BagWindow` builds the reference via `path_bundle_from_bag_lanes`,
# which already adds `DEFAULT_CAMERA_OFFSET_M` (0.08). Therefore only the difference goes to the
# controller: the controller does `p.y() -= cam_y_left`, so cam_y_left = cam − (shift − 0.08). Without
# this, the shift is applied twice and all numbers shift 0.08 m to the right.
# `delay` is set explicitly in each row: by default `build_app` uses 0.23, but the run had 0.35,
# and without this the baseline row silently runs at a different delay (discovered via two
# bit-for-bit matching rows).
CONFIGS_ARC = [
    (
        "as in run: blend 0.3, delay 0.35, tsf 0.64, shift 0.08",
        dict(blend=0.3, shift=0.08, delay=0.35),
    ),
    ("path shift 0.05", dict(blend=0.3, shift=0.05, delay=0.35)),
    ("blend 0.6", dict(blend=0.6, shift=0.08, delay=0.35)),
    ("blend 1.0 (reference = lane center)", dict(blend=1.0, shift=0.08, delay=0.35)),
    ("delay 0.23 (measured)", dict(blend=0.3, shift=0.08, delay=0.23)),
    ("delay 0.23 + blend 0.6", dict(blend=0.6, shift=0.08, delay=0.23)),
    (
        "softer steer rate penalty (150)",
        dict(blend=0.3, shift=0.08, delay=0.35, rate_weight=150.0),
    ),
    (
        "stiffer penalty (800, stock acados)",
        dict(blend=0.3, shift=0.08, delay=0.35, rate_weight=800.0),
    ),
    (
        "tsf 0.50 (stronger understeer compensation)",
        dict(blend=0.3, shift=0.08, delay=0.35, tsf=0.50),
    ),
    (
        "delay 0.23 + blend 0.6 + tsf 0.50",
        dict(blend=0.6, shift=0.08, delay=0.23, tsf=0.50),
    ),
    (
        "centering 0.4 + blend 0.6",
        dict(blend=0.6, shift=0.08, delay=0.35, center_force=0.4),
    ),
    (
        "centering 0.55 + blend 0.6",
        dict(blend=0.6, shift=0.08, delay=0.35, center_force=0.55),
    ),
    ("centering 0.7", dict(blend=0.3, shift=0.08, delay=0.35, center_force=0.7)),
    (
        "centering 0.7 + blend 0.6",
        dict(blend=0.6, shift=0.08, delay=0.35, center_force=0.7),
    ),
    (
        "centering 1.2 (upstream max) + blend 0.6",
        dict(blend=0.6, shift=0.08, delay=0.35, center_force=1.2),
    ),
]

# Short set to validate the methodology itself (resync modes), without sweeping coefficients.
CONFIGS_CORE = [
    ("as in run: blend 0.3, shift 0.08", dict(blend=0.3, shift=0.08, delay=0.35)),
    ("blend 0.6", dict(blend=0.6, shift=0.08, delay=0.35)),
    (
        "centering 0.4 + blend 0.6",
        dict(blend=0.6, shift=0.08, delay=0.35, center_force=0.4),
    ),
    (
        "centering 1.2 + blend 0.6",
        dict(blend=0.6, shift=0.08, delay=0.35, center_force=1.2),
    ),
]

# Camera position in the bag (`lane_keep_debug.cam_y_left_m`); overridden by `--cam-y-left`.
CAM_Y_LEFT = 0.10


def build_app(cfg: dict) -> "pyadas.AdasApp":
    app = pyadas.AdasApp(wheelbase=WHEELBASE)
    app.set_lane_keep_controller(cfg.get("controller", "fp"))
    app.set_lane_keep_max_steer_deg(25.0)
    app.set_lane_keep_mpc_ema_alphas(1.0, 1.0, 1.0)
    app.set_lane_keep_vehicle_model(
        bool(cfg.get("vm", True)), float(cfg.get("tsf", 0.64))
    )
    app.set_lane_keep_fp_steer_delay_s(float(cfg.get("delay", 0.23)))
    # `shift` — full path shift; 0.08 of it is already inside the reference (see CONFIGS_ARC)
    cam_y = cfg.get(
        "cam_y",
        CAM_Y_LEFT - (float(cfg["shift"]) - DEFAULT_CAMERA_OFFSET_M)
        if "shift" in cfg
        else CAM_Y_LEFT,
    )
    app.set_lane_keep_cam_y_left_m(float(cam_y))
    if "rate_weight" in cfg and hasattr(app, "set_lane_keep_fp_steering_rate_weight"):
        app.set_lane_keep_fp_steering_rate_weight(float(cfg["rate_weight"]))
    return app


def _ramp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if x1 <= x0:
        return y0
    t = min(max((x - x0) / (x1 - x0), 0.0), 1.0)
    return y0 + t * (y1 - y0)


def center_force_shift(offset_m: float, width_m: float, kappa: float, cfg: dict) -> float:
    """Mirror of `center_force` from C++ `laneLinesToPath` — keep in sync with it.

    In simulation the reference is assembled in Python and then reprojected into the simulated
    car frame, so the term cannot be computed inside `path_bundle_from_bag_lanes`: the offset there
    is recorded, not the simulated car's offset. Computed here, after reprojection.

    `offset_m` — lane center offset relative to the car, right+ (same as returned by
    `lane_centre_offset`). Returns — shift of the entire reference to the right, m.
    """
    gain = float(cfg.get("center_force", 0.0))
    if (
        gain <= 0.0
        or not np.isfinite(offset_m)
        or not np.isfinite(width_m)
        or width_m <= 0.0
    ):
        return 0.0
    cf = gain * (3.4 / width_m) * offset_m
    cf *= _ramp(width_m, 2.6, 2.8, 0.0, 1.0)
    cf *= _ramp(width_m, 4.0, 6.0, 1.0, 0.0)
    if abs(kappa) > 5e-4 and cf * kappa > 0.0:
        cf *= float(cfg.get("center_force_turn_scale", 0.7))
    lim = float(cfg.get("center_force_max", 0.8))
    return float(min(max(cf, -lim), lim))


def _lane_width_at(win: BagWindow, t: float) -> float:
    """Lane width from the nearest lane-marking frame."""
    lt = np.array([a for a, _ in win.lanes])
    if lt.size == 0:
        return float("nan")
    j = int(np.argmin(np.abs(lt - t)))
    if abs(lt[j] - t) > 300.0:
        return float("nan")
    b = win.lanes[j][1]
    return (
        float(b.get("lane_width", float("nan"))) if isinstance(b, dict) else float("nan")
    )


def _kappa_at_car(poly: np.ndarray, x_max: float = 30.0) -> float:
    """Reference κ at the car (right+) — quadratic fit, like fitAtZero in C++."""
    p = np.asarray(poly, dtype=float)
    m = (p[:, 0] >= 0.0) & (p[:, 0] <= x_max)
    if m.sum() < 4:
        return 0.0
    return float(2.0 * np.polyfit(p[m, 0], p[m, 1], 2)[0])


def _resync_pose(
    win: BagWindow, t: float, xs: float, ys: float, psi: float, keep_lateral: bool
):
    """Re-anchor pose to the recorded one.

    `keep_lateral=False` — old behavior: pose is replaced entirely, including lateral position.
    This puts the car where the driver was, i.e. zeroes accumulated controller offset,
    and the metric median after each reset includes the transient. Offset is then partly
    measured relative to the driver's line.

    `keep_lateral=True` — only along-road drift and absolute heading drift are reset; lateral
    offset and heading error accumulated by the controller are preserved. Those are what we measure.
    """
    xr, yr, yawr, _, _ = _interp_pose(win.pose, t)
    if not keep_lateral:
        return xr, yr, yawr
    # current lateral offset and heading error relative to recorded path
    px, py, yaw = win.pose[:, 1], win.pose[:, 2], np.unwrap(win.pose[:, 3])
    i = int(np.argmin((px - xs) ** 2 + (py - ys) ** 2))
    normal = np.array([-np.sin(yaw[i]), np.cos(yaw[i])])
    e_y = float(np.array([xs - px[i], ys - py[i]]) @ normal)
    e_psi = float(psi - yaw[i])
    n_new = np.array([-np.sin(yawr), np.cos(yawr)])
    return xr + e_y * n_new[0], yr + e_y * n_new[1], yawr + e_psi


def run_closed(
    win: BagWindow,
    cfg: dict,
    dt: float = 0.05,
    resync_s: float = 20.0,
    vision_latency_s: float = 0.069,
    keep_lateral: bool = True,
    real_vision_rate: bool = False,
):
    """Closed-loop replay; returns (trajectory, commanded SWA, lane-centre offset).

    Every `resync_s` the sim is re-anchored to the recorded pose. Without it a 200 s window
    accumulates heading drift into tens of metres of cross-track and the metric stops measuring
    the controller. 0 disables re-anchoring.

    `vision_latency_s` feeds the controller the road as it looked that long ago (measured
    capture→publish is 69 ms). Without it the feedback path is optimistically fast and the sim
    cannot see loop-gain problems.
    """
    # Lane frame index at which the controller last computed: with real_vision_rate
    # the command updates only on a new frame and is held between them — as in the car.
    last_lane_idx = -1
    t_lo = max(win.lanes[0][0], win.pose[0, 0]) + 500.0
    t_hi = min(win.lanes[-1][0], win.pose[-1, 0]) - 500.0
    app = build_app(cfg)
    plant = LateralPlant(wheelbase=WHEELBASE)
    xs, ys, psi, _, _ = _interp_pose(win.pose, t_lo)
    us, t, traj, swa, centre = 0, t_lo, [], [], []
    delta_held = 0.0
    next_sync = t_lo + resync_s * 1000.0 if resync_s > 0 else np.inf
    while t < t_hi:
        if t >= next_sync:
            xs, ys, psi = _resync_pose(win, t, xs, ys, psi, keep_lateral=keep_lateral)
            next_sync += resync_s * 1000.0
        v = max(_nearest(t, win.state, 1, 200.0), 0.1)
        if np.isnan(v):
            v = 0.1
        t_see = t - vision_latency_s * 1000.0
        if real_vision_rate:
            lt = np.array([a for a, _ in win.lanes])
            idx = int(np.argmin(np.abs(lt - t_see)))
            if idx == last_lane_idx:
                # no new frame yet — same command as before
                swa.append(-np.degrees(delta_held) * 15.7)
                psi += plant.step(delta_held, v, dt) * dt
                xs += v * np.cos(psi) * dt
                ys += v * np.sin(psi) * dt
                traj.append((t, xs, ys, psi))
                centre.append(
                    lane_centre_offset(win, t, xs, ys, psi, cam_y_left=CAM_Y_LEFT)
                )
                t += dt * 1000.0
                continue
            last_lane_idx = idx
        poly = _lanes_in_sim_frame(win, t - vision_latency_s * 1000.0, xs, ys, psi)
        if poly is not None and cfg.get("center_force", 0.0) > 0.0:
            t_see = t - vision_latency_s * 1000.0
            off = lane_centre_offset(win, t_see, xs, ys, psi, cam_y_left=CAM_Y_LEFT)
            poly = np.asarray(poly, dtype=float).copy()
            poly[:, 1] += center_force_shift(
                off, _lane_width_at(win, t_see), _kappa_at_car(poly), cfg
            )
        delta = 0.0
        if poly is not None:
            us += int(dt * 1e6)
            app.publish_chassis(us, float(v), 0.0, float(plant.yaw_rate))
            app.publish_lanes(us, [(float(a), float(b)) for a, b in poly])
            app.step(us)
            out = [m for m in app.pop_messages() if isinstance(m, pyadas.LaneKeepOutput)]
            if out:
                delta = -float(out[-1].steer_rad)  # device right+ → math left+
        delta = float(np.clip(delta, -np.radians(35), np.radians(35)))
        delta_held = delta
        swa.append(-np.degrees(delta) * 15.7)
        psi += plant.step(delta, v, dt) * dt
        xs += v * np.cos(psi) * dt
        ys += v * np.sin(psi) * dt
        traj.append((t, xs, ys, psi))
        centre.append(lane_centre_offset(win, t, xs, ys, psi, cam_y_left=CAM_Y_LEFT))
        t += dt * 1000.0
    return np.array(traj), np.array(swa), np.array(centre)


def main() -> None:
    global CAM_Y_LEFT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session", type=Path)
    ap.add_argument("--t0", type=float, required=True)
    ap.add_argument("--t1", type=float, required=True)
    ap.add_argument("--plot", type=Path, default=None)
    ap.add_argument(
        "--vision-latency", type=float, default=0.069, help="vision latency in loop, s"
    )
    ap.add_argument(
        "--resync",
        type=float,
        default=20.0,
        help="re-anchor to recorded pose, s (0 = disable)",
    )
    ap.add_argument(
        "--real-vision-rate",
        action="store_true",
        help="update command only on new lane frame and hold between them, as in the car "
        "(bag runs at 12.5 Hz, p99 161 ms). By default the controller runs every 20 Hz step, "
        "so the sweep understates slow-loop impact",
    )
    ap.add_argument(
        "--resync-full",
        action="store_true",
        help="on re-anchor replace pose entirely, including lateral position. This was the old "
        "behavior, but it puts the car on the driver's line and zeroes the measured offset",
    )
    ap.add_argument(
        "--set",
        dest="config_set",
        choices=("default", "arcs", "core"),
        default="default",
        help="which config set to run: straight-line tuning history or arc levers",
    )
    ap.add_argument(
        "--cam-y-left",
        type=float,
        default=CAM_Y_LEFT,
        help="camera position in bag (lane_keep_debug.cam_y_left_m); feeds controller and metric",
    )
    args = ap.parse_args()

    CAM_Y_LEFT = args.cam_y_left
    configs = {"arcs": CONFIGS_ARC, "core": CONFIGS_CORE}.get(args.config_set, CONFIGS)

    wins: dict = {}

    def window_for(blend: float) -> BagWindow:
        if blend not in wins:
            wins[blend] = BagWindow(
                args.session.resolve(), args.t0, args.t1, lane_blend_scale=blend
            )
        return wins[blend]

    win = window_for(0.0)
    print(
        f"window {args.t0:.0f}–{args.t1:.0f} s, lane frames {len(win.lanes)}, "
        f"set {args.config_set}, cam_y_left {CAM_Y_LEFT:.2f}, "
        f"resync {'full' if args.resync_full else 'lateral preserved'} every {args.resync:.0f} s, "
        f"command rate {'lane frame rate' if args.real_vision_rate else '20 Hz'}"
    )
    print(
        f"\n{'configuration':52s}{'|CT|med':>9}{'p95':>7}{'|centre|med':>12}{'p95':>7}{'offset':>7}{'|dSWA|':>8}"
    )
    results = {}
    for name, cfg in configs:
        traj, swa, centre = run_closed(
            window_for(float(cfg.get("blend", 0.0))),
            cfg,
            resync_s=args.resync,
            vision_latency_s=args.vision_latency,
            keep_lateral=not args.resync_full,
            real_vision_rate=args.real_vision_rate,
        )
        ct = cross_track(traj, win.pose)
        a = np.abs(ct[:, 1])
        c = centre[np.isfinite(centre)]
        results[name] = (traj, ct, swa)
        flag = "  ← diverging" if a.max() > 5 else ""
        cm = np.median(np.abs(c)) if c.size else float("nan")
        c95 = np.percentile(np.abs(c), 95) if c.size else float("nan")
        cbias = np.median(c) if c.size else float("nan")
        print(
            f"{name:52s}{np.median(a):9.2f}{np.percentile(a,95):7.2f}{cm:12.2f}{c95:7.2f}"
            f"{cbias:+7.2f}{np.median(np.abs(np.diff(swa))):8.2f}{flag}"
        )

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
        for i, (name, (traj, ct, swa)) in enumerate(results.items()):
            tt = (ct[:, 0] - ct[0, 0]) / 1000.0
            ax[0].plot(tt, ct[:, 1], lw=1.4, label=name)
            ax[1].plot(tt[: len(swa)], swa, lw=1.0, label=name)
        ax[0].axhline(0, color="k", lw=0.8)
        ax[0].set_ylabel("lateral deviation from driver line, m")
        ax[0].legend(fontsize=8)
        ax[0].grid(alpha=0.3)
        ax[1].set_ylabel("SWA command, deg")
        ax[1].set_xlabel("s")
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"\nplot -> {args.plot}")


if __name__ == "__main__":
    main()
