#!/usr/bin/env python3
"""Closed-loop + shadow MPC simulator over an Android ADAS bag.

Drives the real C++ MPC (``pyadas.AdasApp``) with the bag's ``vision/lanes`` +
``vehicle/state`` and answers two questions offline (no CAN TX):

  --mode closed   Roll a bicycle model forward under the MPC's delta and compare the
                  simulated trajectory to ``localization/pose``. Lanes are reprojected
                  into the sim-car frame each step (local pose-delta correction), so the
                  loop stays faithful while the sim stays near the recorded path.

  --mode shadow   Seed-formula retune on logged internals (fast). Compare desired to
                  driver rack & Ackermann (``--ff-scale`` / ``--kappa-yaw-blend`` /
                  ``--epsi-gain``).

  --mode pyadas   Real C++ MPC via ``pyadas.AdasApp`` open-loop on bag lanes+state.
                  A/B ``--epsi-gain-base`` vs ``--epsi-gain`` (and ff/blend); plot vs driver.

  --scan          List clean-MPC curve segments (controls_allowed & !driver & |yaw|>thr).

The warm-start setter must run AFTER the AdasApp ctor: the ctor calls
``set_warm_start_gains(config defaults)`` and would otherwise clobber it.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from core.path_fusion import path_bundle_from_bag_lanes, path_from_bag_lanes
from core.vehicle_model import LateralPlant
from vis.bag_io import load_topic_messages

WHEELBASE = 2.636
LF = 2.67
K_US = 0.0015
STEER_RATIO = 15.7


def _rot(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _series(session: Path, topic: str, extract):
    return [
        (int(ts), extract(payload))
        for ts, payload, _ in load_topic_messages(session, topic)
        if payload is not None
    ]


def _arr(session: Path, topic: str, extract) -> np.ndarray:
    rows = _series(session, topic, extract)
    return (
        np.array([(ts,) + tuple(v) for ts, v in rows], dtype=float)
        if rows
        else np.empty((0, 0))
    )


def _nearest(t: float, table: np.ndarray, col: int, max_dt_ms: float = 100.0) -> float:
    if table.size == 0:
        return np.nan
    i = int(np.argmin(np.abs(table[:, 0] - t)))
    return table[i, col] if abs(table[i, 0] - t) <= max_dt_ms else np.nan


def _lane_centre_from_msg(ll) -> Optional[np.ndarray]:
    """Midpoint of the two near lane lines (device Y right+) — reference for centring metrics."""
    lanes = list(getattr(ll, "lanes", []) or [])
    xs = np.asarray(list(getattr(ll, "x", []) or []), dtype=float)
    if len(lanes) < 3 or xs.size < 4:
        return None
    left, right = lanes[1], lanes[2]
    if min(getattr(left, "prob", 0.0), getattr(right, "prob", 0.0)) < 0.5:
        return None
    ly = np.asarray(list(left.y), dtype=float)
    ry = np.asarray(list(right.y), dtype=float)
    if ly.size != xs.size or ry.size != xs.size:
        return None
    return np.stack([xs, (ly + ry) / 2.0], axis=1)


def _lane_width_from_msg(ll) -> float:
    """Average lane width at 5–40 m, as in C++ `laneLinesToPath`; NaN if no lines."""
    lanes = list(getattr(ll, "lanes", []) or [])
    xs = np.asarray(list(getattr(ll, "x", []) or []), dtype=float)
    if len(lanes) < 3 or xs.size < 4:
        return float("nan")
    ly = np.asarray(list(lanes[1].y), dtype=float)
    ry = np.asarray(list(lanes[2].y), dtype=float)
    if ly.size != xs.size or ry.size != xs.size:
        return float("nan")
    m = (xs >= 5.0) & (xs <= 40.0)
    return float(np.mean(np.abs(ry[m] - ly[m]))) if m.any() else float("nan")


class BagWindow:
    """Cached per-topic arrays for a time window (ms, relative to first debug frame)."""

    def __init__(
        self,
        session: Path,
        t_lo_s: Optional[float] = None,
        t_hi_s: Optional[float] = None,
        lane_blend_scale: float = 0.0,
    ):
        self.session = session
        dbg = _series(
            session,
            "control/lane_keep_debug",
            lambda d: (
                d.desired_swa_deg,
                d.mpc_kappa_used,
                d.mpc_cte_m,
                np.degrees(d.mpc_epsi_rad),
                d.steer_output_enabled,
            ),
        )
        if not dbg:
            raise SystemExit("no control/lane_keep_debug in bag")
        self.t0 = dbg[0][0]
        lo = -np.inf if t_lo_s is None else t_lo_s * 1000.0
        hi = np.inf if t_hi_s is None else t_hi_s * 1000.0

        def rel(rows):
            out = [((ts - self.t0),) + tuple(v) for ts, v in rows]
            return np.array([r for r in out if lo <= r[0] <= hi], dtype=float)

        self.dbg = rel(dbg)
        self.state = rel(
            _series(
                session,
                "vehicle/state",
                lambda s: (
                    s.v_ego,
                    s.yaw_rate,
                    1.0 if s.steering_pressed else 0.0,
                    s.steering_torque,
                    s.steering_angle_deg,
                ),
            )
        )
        self.pose = rel(
            _series(
                session, "localization/pose", lambda p: (p.x, p.y, p.yaw, p.v, p.yaw_rate)
            )
        )
        self.panda = rel(
            _series(
                # OR, not `controls_allowed` alone: with `lat_always_on` the panda passes torque while
                # `controls_allowed` is false, and `assistAllowed` is a superset of it — see
                # `vis.bag_io.lateral_actuation_on`.
                session,
                "panda/health",
                lambda p: (
                    1.0
                    if (getattr(p, "lat_actuation_allowed", False) or p.controls_allowed)
                    else 0.0,
                ),
            )
        )
        lanes = load_topic_messages(session, "vision/lanes")
        self.lanes = []
        for ts, payload, _ in lanes:
            trel = int(ts) - self.t0
            if payload is None or not (lo <= trel <= hi):
                continue
            bundle = path_bundle_from_bag_lanes(
                payload, min_lane_prob=0.3, lane_blend_scale=lane_blend_scale
            )
            if bundle is not None and bundle["polyline"].shape[0] >= 2:
                bundle = dict(bundle)
                bundle["lane_centre"] = _lane_centre_from_msg(payload)
                bundle["lane_width"] = _lane_width_from_msg(payload)
                self.lanes.append((trel, bundle))

    def clean_mask(self, table: np.ndarray) -> np.ndarray:
        t = table[:, 0]
        pressed = np.array([_nearest(x, self.state, 3) for x in t])
        torque = np.array([_nearest(x, self.state, 4) for x in t])
        v = np.array([_nearest(x, self.state, 1) for x in t])
        ca = np.array([_nearest(x, self.panda, 1, 300.0) for x in t])
        driver = (pressed > 0.5) | (np.abs(torque) > 50.0)
        return (ca > 0.5) & ~driver & (v > 3.0)


def _new_app(
    ff_scale: float,
    kappa_yaw_blend: float,
    epsi_gain: float = 0.0,
    max_steer_deg: float = 25.0,
    cte_gain_floor: float = 0.0,
    cte_gain_base: float = 0.0,
    kappa_ema: float = 1.0,
    epsi_ema: float = 1.0,
    cte_ema: float = 1.0,
    controller: str = "fp",
):
    app = pyadas.AdasApp(wheelbase=WHEELBASE)
    app.set_lane_keep_controller(controller)
    app.set_lane_keep_max_steer_deg(max_steer_deg)
    if controller == "mpc":
        app.set_lane_keep_mpc_kappa_yaw_blend(kappa_yaw_blend)
        if hasattr(app, "set_lane_keep_mpc_ema_alphas"):
            app.set_lane_keep_mpc_ema_alphas(kappa_ema, epsi_ema, cte_ema)
        # AFTER ctor — ctor calls set_warm_start_gains(config defaults) and would clobber.
        pyadas.set_mpc_warm_start_gains(epsi_gain, ff_scale)
        if hasattr(pyadas, "set_mpc_cte_gain_base"):
            pyadas.set_mpc_cte_gain_base(cte_gain_base)
        if hasattr(pyadas, "set_mpc_cte_gain_floor"):
            pyadas.set_mpc_cte_gain_floor(cte_gain_floor)
    elif controller == "fp" and hasattr(app, "set_lane_keep_mpc_ema_alphas"):
        app.set_lane_keep_mpc_ema_alphas(kappa_ema, epsi_ema, cte_ema)
    return app


def run_pyadas_openloop(
    win: BagWindow,
    ff_scale: float,
    kappa_yaw_blend: float,
    epsi_gain: float = 0.0,
    cte_gain_floor: float = 0.0,
    cte_gain_base: float = 0.0,
    kappa_ema: float = 1.0,
    epsi_ema: float = 1.0,
    cte_ema: float = 1.0,
    controller: str = "fp",
):
    """Open-loop: feed recorded ego lanes + chassis into real C++ lane-keep each frame.

    ``controller``: ``mpc`` (VisionPilot) or ``fp`` (flowpilot-style). Returns SWA deg
    series aligned to lane timestamps that produced an output.
    """
    if not win.lanes:
        raise SystemExit("pyadas mode needs vision/lanes in the window")
    app = _new_app(
        ff_scale,
        kappa_yaw_blend,
        epsi_gain,
        cte_gain_floor=cte_gain_floor,
        cte_gain_base=cte_gain_base,
        kappa_ema=kappa_ema,
        epsi_ema=epsi_ema,
        cte_ema=cte_ema,
        controller=controller,
    )
    # Real bag time, not a synthetic 50 ms step: LaneKeepService derives the MPC solve period
    # from these stamps (x0 advance, jerk clips, slew guards). Feeding 20 Hz here would hide
    # exactly the sample-rate bug we are measuring.
    us = 0
    rows = []
    for t_ms, bundle in win.lanes:
        poly = bundle["polyline"] if isinstance(bundle, dict) else bundle
        v = _nearest(t_ms, win.state, 1, 200.0)
        yr = _nearest(t_ms, win.state, 2, 200.0)
        if np.isnan(v) or np.isnan(yr):
            continue
        next_us = int(round(float(t_ms) * 1000.0))
        us = next_us if next_us > us else us + 1_000  # monotonic guard
        app.publish_chassis(us, float(max(v, 0.1)), 0.0, float(yr))
        kwargs = {}
        if isinstance(bundle, dict):
            if bundle.get("plan_poly") is not None:
                kwargs["plan_poly"] = [
                    (float(a), float(b)) for a, b in bundle["plan_poly"]
                ]
            if bundle.get("plan_yaw") is not None:
                kwargs["plan_yaw"] = [float(x) for x in bundle["plan_yaw"]]
            if bundle.get("plan_yaw_rate") is not None:
                kwargs["plan_yaw_rate"] = [float(x) for x in bundle["plan_yaw_rate"]]
        app.publish_lanes(us, [(float(a), float(b)) for a, b in poly], **kwargs)
        app.step(us)
        out = [m for m in app.pop_messages() if isinstance(m, pyadas.LaneKeepOutput)]
        if not out:
            continue
        o = out[-1]
        # LaneKeepOutput.desired_swa_deg is not filled in the pyadas pop path (stays 0);
        # mirror lane_keep_service: desired_swa = steer_sign * deg(steer_rad) * ratio (VW sign=-1).
        swa = -float(np.degrees(o.steer_rad)) * STEER_RATIO
        rows.append((t_ms, swa, float(o.cte_m), float(o.epsi_rad), float(o.curvature)))
    return np.array(rows, dtype=float) if rows else np.empty((0, 5))


def _interp_pose(pose: np.ndarray, t: float):
    tt = pose[:, 0]
    yaw = np.interp(t, tt, np.unwrap(pose[:, 3]))
    return (
        np.interp(t, tt, pose[:, 1]),
        np.interp(t, tt, pose[:, 2]),
        yaw,
        np.interp(t, tt, pose[:, 4]),
        np.interp(t, tt, pose[:, 5]),
    )


def _poly_in_sim_frame(
    win: "BagWindow", idx: int, poly_ego, xs: float, ys: float, psi: float
):
    """Ego polyline recorded at lane frame `idx` → current sim ego frame."""
    poly_ego = np.asarray(poly_ego, dtype=float)
    if poly_ego.ndim != 2 or poly_ego.shape[0] < 2:
        return None
    lt = win.lanes[idx][0]
    xr, yr, yawr, _, _ = _interp_pose(win.pose, lt)
    world = (
        np.array([xr, yr]) + (_rot(yawr) @ np.stack([poly_ego[:, 0], -poly_ego[:, 1]])).T
    )
    ego = (world - np.array([xs, ys])) @ _rot(-psi).T
    return np.stack([ego[:, 0], -ego[:, 1]], axis=1)


def lane_centre_offset(
    win: "BagWindow", t: float, xs: float, ys: float, psi: float, cam_y_left: float = 0.10
):
    """Signed distance from the sim car to the lane centre, + = car left of centre, else NaN.

    The reprojected lane centre is in the camera frame; the vehicle centre sits at +cam_y_left
    there (camera mounted that far left), so it has to be subtracted to make the number
    comparable with the on-road metric `(lane centre @0) − cam_y_left`.
    """
    lt = np.array([a for a, _ in win.lanes])
    if lt.size == 0:
        return float("nan")
    j = int(np.argmin(np.abs(lt - t)))
    if abs(lt[j] - t) > 300.0:
        return float("nan")
    centre = (
        win.lanes[j][1].get("lane_centre") if isinstance(win.lanes[j][1], dict) else None
    )
    if centre is None:
        return float("nan")
    ego = _poly_in_sim_frame(win, j, centre, xs, ys, psi)
    if ego is None:
        return float("nan")
    order = np.argsort(ego[:, 0])
    ex, ey = ego[order, 0], ego[order, 1]
    if ex[0] > 0.5 or ex[-1] < 0.5:
        return float("nan")
    return float(np.interp(0.0, ex, ey)) - cam_y_left


def _lanes_in_sim_frame(
    win: BagWindow,
    t: float,
    xs: float,
    ys: float,
    psi: float,
    x_max: float = 25.0,
    n: int = 24,
):
    lt = np.array([a for a, _ in win.lanes])
    if lt.size == 0:
        return None
    j = int(np.argmin(np.abs(lt - t)))
    if abs(lt[j] - t) > 300.0:
        return None
    bundle = win.lanes[j][1]
    poly_ego = bundle["polyline"] if isinstance(bundle, dict) else bundle
    poly_ego = np.asarray(poly_ego, dtype=float)
    xr, yr, yawr, _, _ = _interp_pose(win.pose, lt[j])
    world = (
        np.array([xr, yr]) + (_rot(yawr) @ np.stack([poly_ego[:, 0], -poly_ego[:, 1]])).T
    )
    ego = (world - np.array([xs, ys])) @ _rot(-psi).T
    ex, ey = ego[:, 0], -ego[:, 1]
    m = (ex > 0.3) & (ex < x_max) & (np.abs(ey) < 8.0)
    if m.sum() < 4:
        return None
    order = np.argsort(ex[m])
    exs, eys = ex[m][order], ey[m][order]
    keep = np.concatenate([[True], np.diff(exs) > 1e-3])
    exs, eys = exs[keep], eys[keep]
    if exs.size < 4:
        return None
    grid = np.linspace(max(1.0, exs[0]), min(x_max, exs[-1]), n)
    return np.stack([grid, np.interp(grid, exs, eys)], axis=1)


def run_closed(
    win: BagWindow,
    ff_scale: float,
    kappa_yaw_blend: float,
    dt: float = 0.05,
    controller: str = "fp",
    plant: Optional[LateralPlant] = None,
):
    """Closed-loop replay on the measured lateral plant.

    The plant is `core.vehicle_model.LateralPlant` (understeer + actuator/yaw lag), not the bare
    kinematic bicycle: the kinematic model over-predicts the car's response by up to 65 % at
    22 m/s and hides exactly the under-steer this loop suffers from. Pass `plant` to override.
    """
    if win.pose.size == 0:
        raise SystemExit("closed mode needs localization/pose")
    t_lo = max(win.lanes[0][0], win.pose[0, 0]) + 500.0
    t_hi = min(win.lanes[-1][0], win.pose[-1, 0]) - 500.0
    app = _new_app(ff_scale, kappa_yaw_blend, controller=controller)
    plant = plant or LateralPlant(wheelbase=WHEELBASE)
    plant.reset()
    xs, ys, psi, _, _ = _interp_pose(win.pose, t_lo)
    us, t = 0, t_lo
    traj = []
    while t < t_hi:
        _, _, _, _, yr = _interp_pose(win.pose, t)
        v = max(_nearest(t, win.state, 1, 200.0), 0.1)
        if np.isnan(v):
            v = 0.1
        poly = _lanes_in_sim_frame(win, t, xs, ys, psi)
        delta = 0.0
        if poly is not None:
            us += int(dt * 1e6)
            app.publish_chassis(us, float(v), 0.0, float(yr))
            app.publish_lanes(us, [(float(a), float(b)) for a, b in poly])
            app.step(us)
            out = [m for m in app.pop_messages() if isinstance(m, pyadas.LaneKeepOutput)]
            if out:
                delta = -float(out[-1].steer_rad)  # device -> math (left+)
        delta = float(np.clip(delta, -np.radians(35), np.radians(35)))
        yaw_rate = plant.step(delta, v, dt)
        xs += v * np.cos(psi) * dt
        ys += v * np.sin(psi) * dt
        psi += yaw_rate * dt
        traj.append((t, xs, ys, psi))
        t += dt * 1000.0  # window time is in ms; physics dt is seconds
    return np.array(traj)


def cross_track(traj: np.ndarray, pose: np.ndarray) -> np.ndarray:
    px, py, yaw = pose[:, 1], pose[:, 2], np.unwrap(pose[:, 3])
    out = []
    for t, x, y, _ in traj:
        i = int(np.argmin((px - x) ** 2 + (py - y) ** 2))
        normal = np.array([-np.sin(yaw[i]), np.cos(yaw[i])])
        out.append((t, float(np.array([x - px[i], y - py[i]]) @ normal)))
    return np.array(out)


def cte_gain_shipped(v):
    """Shipped seed CTE feedback gain (lateral_planning.cpp:134) — rolls off to ~0 at speed."""
    return 0.6 / (1.0 + v * v)


def _seed_swa(
    win: BagWindow,
    ff_scale: float,
    kappa_yaw_blend: float,
    epsi_gain: float = 0.3,
    cte_gain_fn=cte_gain_shipped,
):
    """Warm-start seed command in SWA deg from logged internals (seed == MPC output; GD is a no-op).

    Reproduces the on-device desired_swa (validated corr +0.998 vs logged at ff=2.0/blend=0).
    cte_gain_fn(v) overrides the CTE-feedback gain profile for offline retune sweeps."""
    t = win.dbg[:, 0]
    kus = win.dbg[:, 2]
    cte = win.dbg[:, 3]
    epsi = np.radians(win.dbg[:, 4])
    v = np.array([_nearest(x, win.state, 1) for x in t])
    yaw = np.array([_nearest(x, win.state, 2) for x in t])
    kappa = (1.0 - kappa_yaw_blend) * kus + kappa_yaw_blend * (yaw / np.maximum(v, 1e-3))
    delta_fb = np.clip(cte_gain_fn(v) * cte + epsi_gain * epsi, -0.25, 0.25)
    delta_ff = ff_scale * (np.arctan(LF * kappa) + K_US * v * v * kappa)
    return np.degrees(np.clip(delta_ff + delta_fb, -0.442, 0.442)) * STEER_RATIO


def run_shadow(
    win: BagWindow,
    ff_scale: float,
    kappa_yaw_blend: float,
    epsi_gain: float = 0.3,
    epsi_gain_base: float = 0.3,
    cte_gain_fn=None,
):
    """Shadow retune via seed formula. ``was`` = epsi_gain_base / ff2/blend0 legacy
    unless base gains match current defaults — then ``was`` is logged desired."""
    t = win.dbg[:, 0]
    desired_logged = win.dbg[:, 1]
    meas = np.array([_nearest(x, win.state, 5) for x in t])
    v = np.array([_nearest(x, win.state, 1) for x in t])
    yaw = np.array([_nearest(x, win.state, 2) for x in t])
    cg = cte_gain_fn or cte_gain_shipped
    # Baseline: legacy ff2/blend0 OR same ff/blend with base epsi (for epsi-only A/B)
    was = _seed_swa(win, 2.0, 0.0, epsi_gain=epsi_gain_base, cte_gain_fn=cg)
    now = _seed_swa(win, ff_scale, kappa_yaw_blend, epsi_gain=epsi_gain, cte_gain_fn=cg)
    # If bag already ran with ff/blend≈candidate, transfer only the param delta onto logged.
    desired_now = desired_logged + (now - was)
    # For epsi-only A/B on a bag that already has ff=1/blend=0.4: also expose recreate
    base_same = _seed_swa(
        win, ff_scale, kappa_yaw_blend, epsi_gain=epsi_gain_base, cte_gain_fn=cg
    )
    cand = _seed_swa(win, ff_scale, kappa_yaw_blend, epsi_gain=epsi_gain, cte_gain_fn=cg)
    desired_base = desired_logged + (base_same - was)  # not used when bag params differ
    # Prefer direct seeds when comparing epsi at fixed ff/blend:
    ackermann = (
        np.degrees(np.arctan(LF * np.abs(yaw / np.maximum(v, 1e-3)))) * STEER_RATIO
    )
    return dict(
        t=t,
        driver=meas,
        was=desired_logged,
        now=desired_now,
        base=base_same,
        cand=cand,
        logged=desired_logged,
        ackermann=ackermann,
        v=v,
        yaw=yaw,
    )


def scan_curves(win: BagWindow, yaw_thr: float, min_dur_s: float):
    st = win.state
    if st.size == 0:
        return []
    clean = win.clean_mask(st)
    t, yaw, v = st[:, 0], st[:, 2], st[:, 1]
    turning = clean & (np.abs(yaw) > yaw_thr)
    segs, i, n = [], 0, len(t)
    while i < n:
        if turning[i]:
            j = i
            while j < n and clean[j] and (turning[j] or (j - i) < 6):
                j += 1
            if (t[j - 1] - t[i]) >= min_dur_s * 1000.0:
                m = slice(i, j)
                segs.append(
                    (
                        t[i] / 1000.0,
                        t[j - 1] / 1000.0,
                        (t[j - 1] - t[i]) / 1000.0,
                        float(np.max(np.abs(yaw[m]))),
                        float(np.median(v[m])),
                    )
                )
            i = j
        else:
            i += 1
    return segs


def _plot_closed(traj_a, traj_b, ct_a, ct_b, pose, ff_b, blend_b, out_png):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 2, figsize=(13, 6))
    ax[0].plot(pose[:, 1], pose[:, 2], "k-", lw=3, alpha=0.4, label="real pose")
    ax[0].plot(traj_a[:, 1], traj_a[:, 2], "C3", lw=1.5, label="sim MPC (baseline)")
    ax[0].plot(
        traj_b[:, 1],
        traj_b[:, 2],
        "C2",
        lw=1.5,
        label=f"sim MPC (ff={ff_b},blend={blend_b})",
    )
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("X [m]")
    ax[0].set_ylabel("Y [m]")
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[1].plot(ct_a[:, 0] / 1000.0, np.abs(ct_a[:, 1]), "C3", lw=1.5, label="baseline")
    ax[1].plot(
        ct_b[:, 0] / 1000.0,
        np.abs(ct_b[:, 1]),
        "C2",
        lw=1.5,
        label=f"ff={ff_b},blend={blend_b}",
    )
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("|cross-track vs pose| [m]")
    ax[1].legend()
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
    plt.close(fig)


def _plot_shadow(sh, ff, blend, clean, out_png, epsi=None):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ts = (sh["t"] - sh["t"][0]) / 1000.0
    fig, ax = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)
    label_now = (
        f"MPC after (ff={ff},blend={blend}"
        + (f",epsi={epsi}" if epsi is not None else "")
        + ")"
    )
    ax[0].plot(ts, sh["driver"], "k-", lw=2, label="driver (rack)")
    ax[0].plot(ts, sh["was"], "C3", lw=1.4, alpha=0.85, label="MPC before / logged")
    ax[0].plot(ts, sh["now"], "C2", lw=1.5, label=label_now)
    ax[0].set_ylabel("steering wheel angle [deg]")
    ax[0].legend(fontsize=9)
    ax[0].grid(alpha=0.3)
    ax[1].plot(
        ts, sh["was"] - sh["driver"], "C3", lw=1.4, alpha=0.85, label="before - driver"
    )
    ax[1].plot(ts, sh["now"] - sh["driver"], "C2", lw=1.5, label="after - driver")
    ax[1].axhline(0, color="k", lw=0.6)
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("delta from driver [deg]")
    ax[1].legend(fontsize=9)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
    plt.close(fig)


def _plot_pyadas(t_s, driver, logged, base, cand, eg_base, eg_cand, clean, out_png):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tt = t_s - t_s[0]
    fig, ax = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    ax[0].plot(tt, driver, "k-", lw=2.2, label="driver (rack)")
    if logged is not None:
        ax[0].plot(tt, logged, color="#999", lw=1.0, alpha=0.75, label="MPC logged")
    ax[0].plot(tt, base, "C3", lw=1.3, alpha=0.9, label=f"pyadas eg={eg_base}")
    ax[0].plot(tt, cand, "C2", lw=1.6, label=f"pyadas eg={eg_cand}")
    if clean is not None and clean.any():
        pad = np.concatenate([[False], clean, [False]])
        starts = np.where(~pad[:-1] & pad[1:])[0]
        ends = np.where(pad[:-1] & ~pad[1:])[0]
        first = True
        for s, e in zip(starts, ends):
            ax[0].axvspan(
                tt[s],
                tt[min(e - 1, len(tt) - 1)],
                color="C2",
                alpha=0.08,
                label="clean MPC" if first else None,
            )
            first = False
    ax[0].set_ylabel("steering angle [deg]")
    ax[0].set_title("pyadas open-loop: driver vs MPC", fontweight="bold")
    ax[0].legend(fontsize=9, ncol=2)
    ax[0].grid(alpha=0.3)

    ax[1].plot(
        tt, base - driver, "C3", lw=1.2, alpha=0.85, label=f"eg={eg_base} − driver"
    )
    ax[1].plot(tt, cand - driver, "C2", lw=1.4, label=f"eg={eg_cand} − driver")
    ax[1].axhline(0, color="k", lw=0.6)
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("Δ [deg]")
    ax[1].set_title("Error vs driver (closer to 0 = better)", fontweight="bold")
    ax[1].legend(fontsize=9)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
    plt.close(fig)


def _metrics(cmd, driver, mask, name):
    if mask.sum() < 5:
        print(f"  {name}: n={int(mask.sum())} — skip")
        return
    err = np.abs(cmd[mask] - driver[mask])
    corr = float(np.corrcoef(cmd[mask], driver[mask])[0, 1])
    print(
        f"  {name}: n={int(mask.sum())}  |err|med={np.median(err):.2f}  p95={np.percentile(err,95):.1f}  "
        f"corr={corr:.3f}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("session", type=Path)
    ap.add_argument(
        "--mode", choices=("closed", "shadow", "scan", "pyadas"), default="shadow"
    )
    ap.add_argument(
        "--t0", type=float, default=None, help="window start [s from bag start]"
    )
    ap.add_argument("--t1", type=float, default=None, help="window end [s]")
    ap.add_argument(
        "--ff-scale", type=float, default=1.0, help="patched mpc_ff_scale (baseline=2.0)"
    )
    ap.add_argument(
        "--kappa-yaw-blend",
        type=float,
        default=0.4,
        help="patched mpc_kappa_yaw_blend (baseline=0.0)",
    )
    ap.add_argument(
        "--epsi-gain", type=float, default=0.0, help="candidate mpc_epsi_gain"
    )
    ap.add_argument(
        "--epsi-gain-base",
        type=float,
        default=0.15,
        help="baseline mpc_epsi_gain for A/B",
    )
    ap.add_argument(
        "--cte-gain-floor",
        type=float,
        default=0.0,
        help="mpc_cte_gain_floor (pyadas if wired)",
    )
    ap.add_argument(
        "--cte-gain-base",
        type=float,
        default=0.0,
        help="mpc_cte_gain_base (0=FF-only seed)",
    )
    ap.add_argument(
        "--controller",
        choices=("mpc", "fp"),
        default="fp",
        help="lane-keep controller for pyadas open-loop",
    )
    ap.add_argument(
        "--kappa-ema", type=float, default=0.5, help="mpc_kappa_ema_alpha (1=off)"
    )
    ap.add_argument(
        "--epsi-ema", type=float, default=0.5, help="mpc_epsi_ema_alpha candidate (1=off)"
    )
    ap.add_argument(
        "--cte-ema", type=float, default=0.5, help="mpc_cte_ema_alpha candidate (1=off)"
    )
    ap.add_argument(
        "--epsi-ema-base", type=float, default=1.0, help="baseline mpc_epsi_ema_alpha"
    )
    ap.add_argument(
        "--cte-ema-base", type=float, default=1.0, help="baseline mpc_cte_ema_alpha"
    )
    ap.add_argument(
        "--yaw-thr",
        type=float,
        default=0.03,
        help="scan: |yaw_rate| curve threshold [rad/s]",
    )
    ap.add_argument(
        "--min-dur", type=float, default=3.0, help="scan: min segment duration [s]"
    )
    ap.add_argument("--plot", type=Path, default=None, help="write a PNG")
    args = ap.parse_args()

    session = args.session.resolve()

    if args.mode == "scan":
        win = BagWindow(session, args.t0, args.t1)
        segs = scan_curves(win, args.yaw_thr, args.min_dur)
        print(
            f"clean-MPC curves (|yaw|>{args.yaw_thr} for >={args.min_dur}s): {len(segs)}"
        )
        for a, b, dur, ymax, v in sorted(segs, key=lambda s: -s[2]):
            print(
                f"  {a:8.1f}-{b:8.1f}s  dur={dur:4.1f}s  |yaw|max={ymax:.3f} (R~{1/ymax:.0f}m)  v={v:.0f}"
            )
        return

    win = BagWindow(session, args.t0, args.t1)

    if args.mode == "closed":
        traj_a = run_closed(win, 2.0, 0.0)
        traj_b = run_closed(win, args.ff_scale, args.kappa_yaw_blend)
        ct_a, ct_b = cross_track(traj_a, win.pose), cross_track(traj_b, win.pose)
        for name, ct in (
            ("baseline ff2.0/blend0", ct_a),
            (f"patch ff{args.ff_scale}/blend{args.kappa_yaw_blend}", ct_b),
        ):
            v = np.abs(ct[:, 1])
            print(
                f"[closed] {name}: |cross-track| med={np.median(v):.2f} p95={np.percentile(v,95):.2f} "
                f"max={v.max():.2f} m  final={ct[-1,1]:+.2f} m"
            )
        if args.plot:
            _plot_closed(
                traj_a,
                traj_b,
                ct_a,
                ct_b,
                win.pose,
                args.ff_scale,
                args.kappa_yaw_blend,
                args.plot,
            )
            print(f"plot -> {args.plot}")
        return

    if args.mode == "pyadas":
        floor = args.cte_gain_floor
        print(
            f"[pyadas] open-loop A/B ctrl={args.controller} eg {args.epsi_gain_base}→{args.epsi_gain}  "
            f"epsi_ema {args.epsi_ema_base}→{args.epsi_ema}  cte_ema {args.cte_ema_base}→{args.cte_ema}  "
            f"κ_ema={args.kappa_ema} ff={args.ff_scale} blend={args.kappa_yaw_blend} "
            f"floor={floor} base={args.cte_gain_base}  lanes={len(win.lanes)}"
        )
        print("[pyadas] running baseline…")
        base = run_pyadas_openloop(
            win,
            args.ff_scale,
            args.kappa_yaw_blend,
            epsi_gain=args.epsi_gain_base,
            cte_gain_floor=floor,
            cte_gain_base=args.cte_gain_base,
            kappa_ema=args.kappa_ema,
            epsi_ema=args.epsi_ema_base,
            cte_ema=args.cte_ema_base,
            controller=args.controller,
        )
        print(f"[pyadas] baseline frames={len(base)}")
        print("[pyadas] running candidate…")
        cand = run_pyadas_openloop(
            win,
            args.ff_scale,
            args.kappa_yaw_blend,
            epsi_gain=args.epsi_gain,
            cte_gain_floor=floor,
            cte_gain_base=args.cte_gain_base,
            kappa_ema=args.kappa_ema,
            epsi_ema=args.epsi_ema,
            cte_ema=args.cte_ema,
            controller=args.controller,
        )
        print(f"[pyadas] candidate frames={len(cand)}")
        if len(base) < 5 or len(cand) < 5:
            raise SystemExit(
                "pyadas produced too few frames — check window has lanes+state"
            )
        # Align on common timestamps (intersection)
        t_b, t_c = base[:, 0], cand[:, 0]
        common = np.intersect1d(t_b, t_c)
        ib = np.searchsorted(t_b, common)
        ic = np.searchsorted(t_c, common)
        t_ms = common
        swa_b, swa_c = base[ib, 1], cand[ic, 1]
        driver = np.array([_nearest(x, win.state, 5) for x in t_ms])
        logged = np.array([_nearest(x, win.dbg, 1) for x in t_ms])
        yaw = np.array([_nearest(x, win.state, 2) for x in t_ms])
        v = np.array([_nearest(x, win.state, 1) for x in t_ms])
        pressed = np.array([_nearest(x, win.state, 3) for x in t_ms])
        torque = np.array([_nearest(x, win.state, 4) for x in t_ms])
        ca = np.array([_nearest(x, win.panda, 1, 300.0) for x in t_ms])
        clean = (ca > 0.5) & ~((pressed > 0.5) | (np.abs(torque) > 50.0)) & (v > 3.0)
        hi = clean & (v >= 16)
        print("[pyadas] vs driver:")
        _metrics(swa_b, driver, clean, f"base eg={args.epsi_gain_base} clean")
        _metrics(swa_c, driver, clean, f"cand eg={args.epsi_gain}+ema clean")
        _metrics(swa_b, driver, hi, f"base eg={args.epsi_gain_base} clean v>=16")
        _metrics(swa_c, driver, hi, f"cand eg={args.epsi_gain}+ema clean v>=16")
        ok = np.isfinite(logged)
        if ok.sum() > 10:
            print(
                f"[pyadas] recreate vs logged: corr(eg_base,logged)="
                f"{np.corrcoef(swa_b[ok], logged[ok])[0,1]:.3f}  "
                f"|Δ|med={np.median(np.abs(swa_b[ok]-logged[ok])):.2f}°"
            )
        if args.plot:
            _plot_pyadas(
                t_ms / 1000.0,
                driver,
                logged,
                swa_b,
                swa_c,
                args.epsi_gain_base,
                args.epsi_gain,
                clean,
                args.plot,
            )
            print(f"plot -> {args.plot}")
        return

    # floor-aware seed for epsi A/B on bags that already have ff/blend patched
    def cte_fn(v):
        return np.maximum(cte_gain_shipped(v), args.cte_gain_floor)

    sh = run_shadow(
        win,
        args.ff_scale,
        args.kappa_yaw_blend,
        epsi_gain=args.epsi_gain,
        epsi_gain_base=args.epsi_gain_base,
        cte_gain_fn=cte_fn,
    )
    # For epsi-only compare use direct seeds at same ff/blend
    sh["was"] = sh["base"]
    sh["now"] = sh["cand"]
    clean = win.clean_mask(win.dbg)
    curve = clean & (np.abs(sh["yaw"]) > 0.03)
    hi = clean & (sh["v"] >= 16) & (np.abs(sh["yaw"]) < 0.02)
    sel = curve if curve.sum() >= 5 else np.isfinite(sh["driver"])
    print(
        f"[shadow] epsi A/B {args.epsi_gain_base} → {args.epsi_gain}  "
        f"(ff={args.ff_scale} blend={args.kappa_yaw_blend} floor={args.cte_gain_floor})"
    )
    _metrics(sh["was"], sh["driver"], clean, f"eg={args.epsi_gain_base} clean")
    _metrics(sh["now"], sh["driver"], clean, f"eg={args.epsi_gain} clean")
    _metrics(sh["was"], sh["driver"], hi, f"eg={args.epsi_gain_base} straight v>=16")
    _metrics(sh["now"], sh["driver"], hi, f"eg={args.epsi_gain} straight v>=16")
    _metrics(sh["was"], sh["driver"], sel, f"eg={args.epsi_gain_base} turns")
    _metrics(sh["now"], sh["driver"], sel, f"eg={args.epsi_gain} turns")
    if args.plot:
        _plot_shadow(
            sh, args.ff_scale, args.kappa_yaw_blend, clean, args.plot, epsi=args.epsi_gain
        )
        print(f"plot -> {args.plot}")


if __name__ == "__main__":
    main()
