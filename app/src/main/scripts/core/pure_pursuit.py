#!/usr/bin/env python3
"""Pure pursuit viz helpers. Algorithm: ``pyadas.core.PurePursuit`` / ``LaneKeepService``."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Tuple

import cv2
import numpy as np

from .lane_projection import CameraGeometry, project_iso_xyz
from .phone_rt import PhoneRtGeometry, project_overlay_xyz


@dataclass
class PurePursuitResult:
    """HUD-facing PP snapshot (built from ``LaneKeepOutput`` / ``PurePursuitResult`` C++)."""

    lookahead_m: float
    target_ra: Optional[np.ndarray]
    target_ego: Optional[np.ndarray]
    alpha_rad: float
    steer_rad: float
    speed_mps: float
    polyline_ego: np.ndarray
    wheel_base: float = 2.636

    @property
    def curvature(self) -> float:
        return float(np.tan(self.steer_rad) / max(self.wheel_base, 1e-6))

    @property
    def radius_m(self) -> float:
        k = self.curvature
        if abs(k) < 1e-6:
            return float("inf")
        return float(1.0 / k)


def pp_result_from_output(
    out: Any,
    poly: np.ndarray,
    speed_mps: float,
    *,
    waypoint_shift: float,
    wheel_base: float,
) -> PurePursuitResult:
    """Map C++ ``LaneKeepOutput`` / PP result fields into a viz dataclass."""
    target_ego = None
    target_ra = None
    if getattr(out, "has_target", False):
        target_ego = np.array([out.target_x, out.target_y], dtype=np.float64)
        target_ra = np.array(
            [out.target_x + waypoint_shift, out.target_y], dtype=np.float64
        )
    return PurePursuitResult(
        lookahead_m=float(out.lookahead_m),
        target_ra=target_ra,
        target_ego=target_ego,
        alpha_rad=float(getattr(out, "alpha_rad", 0.0)),
        steer_rad=float(out.steer_rad),
        speed_mps=float(speed_mps),
        polyline_ego=poly,
        wheel_base=float(wheel_base),
    )


def plan_to_polyline_ego(
    plan_x: np.ndarray,
    plan_y: np.ndarray,
    y_sign: float = 1.0,
    x_min: float = 0.5,
    recenter: bool = False,
    recenter_at_x: float = 5.0,
) -> np.ndarray:
    """Supercombo/bag plan → Nx2 ego polyline for PP.

    Default ``y_sign=1`` keeps **device Y-right** (Android TopicConvert / LaneKeep).
    Use ``y_sign=-1`` only when converting to ISO Y-left for MetaDrive GT overlays.
    """
    from .frames import PP_Y_SIGN

    if y_sign is None:
        y_sign = PP_Y_SIGN
    xs = np.asarray(plan_x, dtype=np.float64)
    ys = float(y_sign) * np.asarray(plan_y, dtype=np.float64)
    ok = np.isfinite(xs) & np.isfinite(ys) & (xs >= x_min)
    poly = np.stack([xs[ok], ys[ok]], axis=1)
    if not recenter or poly.shape[0] < 2:
        return poly
    order = np.argsort(poly[:, 0])
    poly = poly[order]
    x0 = float(np.clip(recenter_at_x, poly[0, 0], poly[-1, 0]))
    y_ref = float(np.interp(x0, poly[:, 0], poly[:, 1]))
    poly = poly.copy()
    poly[:, 1] -= y_ref
    return poly


def bicycle_arc_points(
    curvature: float,
    length_m: float = 40.0,
    n: int = 40,
) -> np.ndarray:
    s = np.linspace(0.0, length_m, n)
    k = float(curvature)
    if abs(k) < 1e-6:
        return np.stack([s, np.zeros_like(s)], axis=1)
    x = np.sin(k * s) / k
    y = (1.0 - np.cos(k * s)) / k
    return np.stack([x, y], axis=1)


def draw_steering_wheel(
    bgr: np.ndarray,
    steer_rad: float,
    center: Optional[Tuple[int, int]] = None,
    radius: int = 48,
    steer_ratio: float = 15.7,
) -> None:
    h, w = bgr.shape[:2]
    if center is None:
        center = (w - radius - 16, h - radius - 40)
    cx, cy = center
    wheel_deg = float(np.clip(np.rad2deg(steer_rad) * steer_ratio, -120.0, 120.0))
    ang = np.deg2rad(wheel_deg)

    overlay = bgr.copy()
    cv2.circle(overlay, (cx, cy), radius + 6, (30, 30, 30), -1, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.55, bgr, 0.45, 0, bgr)
    cv2.circle(bgr, (cx, cy), radius, (220, 220, 220), 3, cv2.LINE_AA)
    cv2.circle(bgr, (cx, cy), int(radius * 0.35), (180, 180, 180), 2, cv2.LINE_AA)

    def _rot(px: float, py: float) -> Tuple[int, int]:
        c, s = np.cos(ang), np.sin(ang)
        return int(round(cx + c * px + s * py)), int(round(cy - s * px + c * py))

    for a0 in (90.0, 210.0, 330.0):
        a = np.deg2rad(a0)
        cv2.line(
            bgr,
            _rot(0.0, 0.0),
            _rot(radius * 0.92 * np.cos(a), -radius * 0.92 * np.sin(a)),
            (200, 200, 200),
            2,
            cv2.LINE_AA,
        )
    top = _rot(0.0, -radius * 0.85)
    cv2.circle(bgr, (cx, cy), 5, (0, 200, 255), -1, cv2.LINE_AA)
    cv2.circle(bgr, top, 5, (0, 0, 255), -1, cv2.LINE_AA)
    cv2.putText(
        bgr,
        f"{np.rad2deg(steer_rad):+.1f}° road",
        (cx - radius, cy + radius + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (0, 200, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        bgr,
        f"SW {wheel_deg:+.0f}°",
        (cx - radius, cy + radius + 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )


def draw_pure_pursuit(
    bgr: np.ndarray,
    pp: PurePursuitResult,
    geom: CameraGeometry | PhoneRtGeometry,
    waypoint_shift: float,
    y_sign_for_project: float = 1.0,
    steer_ratio: float = 15.7,
    draw_wheel: bool = True,
) -> None:
    h, w = bgr.shape[:2]
    ld = pp.lookahead_m
    cx_ra = -waypoint_shift
    cy_ra = 0.0

    thetas = np.linspace(0.0, 2.0 * np.pi, 72, endpoint=False)
    circ_x = cx_ra + ld * np.cos(thetas)
    circ_y = cy_ra + ld * np.sin(thetas)
    circ_pts = project_overlay_xyz(
        circ_x,
        circ_y,
        np.zeros_like(circ_x),
        geom,
        w,
        h,
        x_min=0.3,
        y_sign=y_sign_for_project,
    )
    if len(circ_pts) >= 2:
        for a, b in zip(circ_pts, circ_pts[1:]):
            cv2.line(bgr, a, b, (255, 128, 0), 1, cv2.LINE_AA)
        cv2.line(bgr, circ_pts[-1], circ_pts[0], (255, 128, 0), 1, cv2.LINE_AA)

    kappa = pp.curvature
    arc_ra = bicycle_arc_points(kappa, length_m=min(max(ld * 1.5, 15.0), 50.0), n=48)
    arc_pts = project_overlay_xyz(
        arc_ra[:, 0] + cx_ra,
        arc_ra[:, 1] + cy_ra,
        np.zeros(len(arc_ra)),
        geom,
        w,
        h,
        x_min=0.3,
        y_sign=y_sign_for_project,
    )
    if len(arc_pts) >= 2:
        for a, b in zip(arc_pts, arc_pts[1:]):
            cv2.line(bgr, a, b, (255, 0, 255), 3, cv2.LINE_AA)

    ra_pts = project_overlay_xyz(
        np.array([cx_ra]),
        np.array([cy_ra]),
        np.zeros(1),
        geom,
        w,
        h,
        x_min=-5.0,
        y_sign=y_sign_for_project,
        margin=200.0,
    )
    if ra_pts:
        cv2.drawMarker(bgr, ra_pts[0], (255, 128, 0), cv2.MARKER_TILTED_CROSS, 12, 2)

    if pp.target_ego is not None:
        tgt = project_overlay_xyz(
            np.array([float(pp.target_ego[0])]),
            np.array([float(pp.target_ego[1])]),
            np.zeros(1),
            geom,
            w,
            h,
            x_min=0.3,
            y_sign=y_sign_for_project,
        )
        if tgt:
            cv2.circle(bgr, tgt[0], 7, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(bgr, tgt[0], 9, (255, 255, 255), 1, cv2.LINE_AA)
            if ra_pts:
                cv2.line(bgr, ra_pts[0], tgt[0], (0, 0, 255), 2, cv2.LINE_AA)

    if draw_wheel:
        draw_steering_wheel(bgr, pp.steer_rad, steer_ratio=steer_ratio)

    curv_txt = (
        f"κ={kappa:.4f}/m  R={pp.radius_m:.1f}m"
        if np.isfinite(pp.radius_m)
        else "κ=0  R=∞"
    )
    status = (
        f"PP Ld={ld:.1f}m  v={pp.speed_mps:.1f}  "
        f"α={np.rad2deg(pp.alpha_rad):.1f}°  δ={np.rad2deg(pp.steer_rad):.1f}°  {curv_txt}"
    )
    if pp.target_ego is None:
        status += "  (no target)"
    cv2.putText(
        bgr,
        status,
        (8, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 165, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        bgr,
        "magenta=curvature arc  orange=Ld  red=target",
        (8, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (200, 150, 200),
        1,
        cv2.LINE_AA,
    )
