#!/usr/bin/env python3
"""Bag window + lane geometry for the offline stands.

This used to be a standalone MPC simulator with closed/shadow modes; the kinematic bicycle model
over-predicted and the stand was retired (git history, 2026-08-20). What earned its keep is the data
half — `BagWindow` and the lane-frame helpers — which `bag_config_sweep.py` drives through the real
C++ Planner/Control via pyadas.
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


def cross_track(traj: np.ndarray, pose: np.ndarray) -> np.ndarray:
    px, py, yaw = pose[:, 1], pose[:, 2], np.unwrap(pose[:, 3])
    out = []
    for t, x, y, _ in traj:
        i = int(np.argmin((px - x) ** 2 + (py - y) ** 2))
        normal = np.array([-np.sin(yaw[i]), np.cos(yaw[i])])
        out.append((t, float(np.array([x - px[i], y - py[i]]) @ normal)))
    return np.array(out)
