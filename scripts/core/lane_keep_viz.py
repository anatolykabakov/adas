#!/usr/bin/env python3
"""Shared lane-keep overlays for interactive visualizer and MetaDrive sim."""

from __future__ import annotations

from typing import Any, Dict, Optional, Union

import cv2
import numpy as np

from .lane_keep import LaneKeepResult
from .lane_projection import CameraGeometry
from .phone_rt import PhoneRtGeometry, project_overlay_xyz
from .pure_pursuit import draw_pure_pursuit, draw_steering_wheel
from .supercombo_compare import draw_pts, make_overlay_geometry

OverlayGeom = Union[CameraGeometry, PhoneRtGeometry]


def _to_uint8(img: np.ndarray) -> np.ndarray:
    if img.dtype == np.uint8:
        return img
    return (np.clip(img, 0.0, 1.0) * 255.0).astype(np.uint8)


def _draw_ego_polyline(
    img: np.ndarray,
    polyline: Optional[np.ndarray],
    geom: OverlayGeom,
    w: int,
    h: int,
    color: tuple[int, int, int],
    thickness: int = 2,
    x_min: float = 0.0,
    y_sign: float = 1.0,
) -> None:
    if polyline is None:
        return
    poly = np.asarray(polyline, dtype=np.float64)
    if poly.ndim != 2 or poly.shape[0] < 2:
        return
    pts = project_overlay_xyz(
        poly[:, 0],
        poly[:, 1],
        np.zeros(poly.shape[0], dtype=np.float64),
        geom,
        w,
        h,
        x_min=x_min,
        y_sign=y_sign,
    )
    draw_pts(img, pts, color, thickness)


def draw_bev_inset(
    img: np.ndarray,
    lk: LaneKeepResult,
    lanes: Optional[Dict[str, Any]] = None,
    size: int = 180,
    scale: float = 4.0,
    gt_poly: Optional[np.ndarray] = None,
    gt_lk: Optional[LaneKeepResult] = None,
) -> None:
    """Top-down ego view: boundaries, centerline, PP target.

    Optional GT centerline (white) and GT PP target (white ring) for compare mode.
    """
    h, w = img.shape[:2]
    inset = np.zeros((size, size, 3), dtype=np.uint8)
    cx, cy = size // 2, size - 24

    def to_px(x: float, y: float) -> tuple[int, int]:
        return int(round(cx + y * scale)), int(round(cy - x * scale))

    cv2.line(inset, (0, cy), (size, cy), (40, 40, 40), 1)
    cv2.line(inset, (cx, 0), (cx, size), (40, 40, 40), 1)
    cv2.circle(inset, (cx, cy), 4, (200, 200, 200), -1)

    if lanes is not None:
        for key, color in (("right_road", (0, 0, 255)), ("left_road", (0, 255, 0))):
            pts = lanes.get(key)
            if pts is None or len(pts) < 2:
                continue
            px = [to_px(float(x), float(y)) for x, y in pts]
            for a, b in zip(px[:-1], px[1:]):
                cv2.line(inset, a, b, color, 1, cv2.LINE_AA)

    if gt_poly is not None and len(gt_poly) >= 2:
        px = [to_px(float(x), float(y)) for x, y in gt_poly]
        for a, b in zip(px[:-1], px[1:]):
            cv2.line(inset, a, b, (255, 255, 255), 1, cv2.LINE_AA)

    if lk.polyline is not None and len(lk.polyline) >= 2:
        px = [to_px(float(x), float(y)) for x, y in lk.polyline]
        for a, b in zip(px[:-1], px[1:]):
            cv2.line(inset, a, b, (0, 255, 255), 2, cv2.LINE_AA)

    if lk.pure_pursuit is not None and lk.pure_pursuit.target_ego is not None:
        tx, ty = lk.pure_pursuit.target_ego
        cv2.circle(inset, to_px(float(tx), float(ty)), 5, (0, 0, 255), -1)
    elif lk.controller == "mpc" or lk.mode == "mpc":
        # CTE in VP frame (+ right of path); device Y is right+ → show lateral offset
        # as a red tick at x=8 m, y = -cte (path is left of ego when cte>0 in VP)
        y_dev = -float(lk.e_y)
        cv2.circle(inset, to_px(8.0, y_dev), 5, (0, 128, 255), -1)
        cv2.arrowedLine(
            inset, (cx, cy - 32), to_px(8.0, y_dev), (0, 128, 255), 1, tipLength=0.3
        )

    if (
        gt_lk is not None
        and gt_lk.pure_pursuit is not None
        and gt_lk.pure_pursuit.target_ego is not None
    ):
        tx, ty = gt_lk.pure_pursuit.target_ego
        cv2.circle(inset, to_px(float(tx), float(ty)), 7, (255, 255, 255), 1)

    if lk.controller == "mpc" or lk.mode == "mpc":
        label = f"MPC δ={np.rad2deg(lk.steer_rad):+.1f}° CTE={lk.e_y:+.2f}"
    else:
        label = f"{lk.mode} δ={np.rad2deg(lk.steer_rad):+.1f}°"
    if gt_lk is not None:
        label += f" GT={np.rad2deg(gt_lk.steer_rad):+.1f}°"
    cv2.putText(
        inset,
        label,
        (4, 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.32,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    img[8 : 8 + size, w - size - 8 : w - 8] = inset


def draw_lane_keep_overlay(
    img: np.ndarray,
    lk: LaneKeepResult,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    w: int,
    h: int,
    lanes: Optional[Dict[str, Any]] = None,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    roll_deg: float = 0.0,
    camera_height: float = 1.40,
    waypoint_shift: float = 1.40,
    y_sign: float = 1.0,
    draw_bev: bool = False,
    draw_footer: bool = True,
    draw_wheel: bool = True,
    wheel_steer_rad: Optional[float] = None,
    geom: Optional[OverlayGeom] = None,
    gt_poly: Optional[np.ndarray] = None,
    gt_lk: Optional[LaneKeepResult] = None,
) -> np.ndarray:
    """Project lane boundaries / centerline + Pure Pursuit / MPC HUD.

    ``wheel_steer_rad`` overrides the HUD wheel angle (e.g. bag measured SWA→road).
    Default geom is AAD. Pass ``PhoneRtGeometry`` to match ModelCalibWarp / phone overlay.
    """
    out = _to_uint8(img.copy())

    if geom is None:
        geom = make_overlay_geometry(
            fx,
            fy,
            cx,
            cy,
            w,
            h,
            camera_height=camera_height,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            roll_deg=roll_deg,
        )

    if lanes is not None:
        _draw_ego_polyline(
            out, lanes.get("right_road"), geom, w, h, (0, 0, 255), 2, y_sign=y_sign
        )
        _draw_ego_polyline(
            out, lanes.get("left_road"), geom, w, h, (0, 255, 0), 2, y_sign=y_sign
        )

    if gt_poly is not None:
        _draw_ego_polyline(out, gt_poly, geom, w, h, (255, 255, 255), 2, y_sign=y_sign)

    if lk.polyline is not None:
        _draw_ego_polyline(out, lk.polyline, geom, w, h, (0, 255, 255), 3, y_sign=y_sign)

    if lk.mode == "pure_pursuit" and lk.pure_pursuit is not None:
        draw_pure_pursuit(
            out,
            lk.pure_pursuit,
            geom,
            waypoint_shift=waypoint_shift,
            y_sign_for_project=y_sign,
            draw_wheel=False,
        )

    if draw_wheel:
        delta = lk.steer_rad if wheel_steer_rad is None else float(wheel_steer_rad)
        draw_steering_wheel(out, delta)

    if draw_footer:
        ctrl = lk.controller or lk.mode
        status = f"LK {ctrl}  steer={np.rad2deg(lk.steer_rad):+.1f}°  κ={lk.curvature:.4f}  {lk.status}"
        if ctrl == "mpc" or lk.mode == "mpc":
            status += f"  CTE={lk.e_y:+.2f}m eψ={np.rad2deg(lk.e_psi):+.1f}°"
        if gt_lk is not None:
            status += f"  GT δ={np.rad2deg(gt_lk.steer_rad):+.1f}°"
        cv2.putText(
            out,
            status,
            (8, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )

    if draw_bev:
        draw_bev_inset(out, lk, lanes=lanes, gt_poly=gt_poly, gt_lk=gt_lk)
    return out
