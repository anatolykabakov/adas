"""Phone / flowpilot overlay projection: Rt = V·R(rpy)·V⁻¹ (inverse of ModelCalibWarp).

Matches ``LaneOverlayView.projectDevice`` — same RPY/K as ``warp_matrix_deg``.
Unlike AAD ``CameraGeometry``, height / cam_x do **not** enter this projector.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple, Union

import numpy as np

from .model_calib_warp import VIEW_FROM_DEVICE, _rot_from_euler

PATH_LIFT_M = 1.28


def rt_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """3×3 Rt = V · R · V⁻¹ (row-major), identical to LaneOverlayView.rebuildRt."""
    R = _rot_from_euler(np.deg2rad(roll_deg), np.deg2rad(pitch_deg), np.deg2rad(yaw_deg))
    V = VIEW_FROM_DEVICE
    Vi = V.T  # permutation inverse
    return V @ R @ Vi


@dataclass
class PhoneRtGeometry:
    """Pinhole overlay matched to ModelCalibWarp (no mount translation)."""

    fx: float
    fy: float
    cx: float
    cy: float
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    width: int = 640
    height: int = 480
    path_lift_m: float = PATH_LIFT_M
    camera_height_m: float = 1.5  # only for HUD points at camera Z (PP target)

    def __post_init__(self) -> None:
        if self.fy <= 1.0 or abs(self.fx / max(self.fy, 1e-6) - 1.0) > 0.15:
            self.fy = self.fx
        self.Rt = rt_from_rpy_deg(self.roll_deg, self.pitch_deg, self.yaw_deg)

    def project_device(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        zs: np.ndarray,
        *,
        x_min: float = 1.5,
        path_lift: bool = False,
        margin: float = 80.0,
        keep_gaps: bool = False,
    ) -> List[Tuple[int, int]]:
        """Device frame: X fwd, Y right+, Z **down** (openpilot FRD) → image pixels.

        ``keep_gaps`` marks an out-of-frame point as ``None`` instead of dropping it, so a drawer can
        break the polyline there rather than joining the two surviving neighbours across the image.
        The body already relied on it; without it in the signature every call through
        ``project_overlay_xyz`` raised ``TypeError``, and a direct call raised ``NameError`` the moment
        a point left the frame.
        """
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        zs = np.asarray(zs, dtype=np.float64)
        if zs.shape != xs.shape:
            zs = np.zeros_like(xs)

        lift = self.path_lift_m if path_lift else 0.0
        # Z≈0 without lift collapses to the horizon (v≈cy) under Rt. Same as
        # LaneOverlayView.projectEgo: put ground XY on the camera-height plane.
        if not path_lift and not np.any(np.abs(zs) > 0.05):
            zs = np.full_like(xs, self.camera_height_m)

        ok = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs) & (xs >= x_min)
        if not np.any(ok):
            return []

        # Remap (X,Y,Z)→(Y, Z+lift, X) then Rt
        cam = np.stack([ys[ok], zs[ok] + lift, xs[ok]], axis=1)  # N×3
        xyz = (self.Rt @ cam.T).T
        z = xyz[:, 2]
        front = z > 0.15
        if not np.any(front):
            return []
        xyz = xyz[front]
        z = xyz[:, 2]
        u = self.fx * (xyz[:, 0] / z) + self.cx
        v = self.fy * (xyz[:, 1] / z) + self.cy
        pts: List[Tuple[int, int]] = []
        w, h = self.width, self.height
        for ui, vi in zip(u, v):
            if -margin <= ui < w + margin and -margin <= vi < h + margin:
                pts.append((int(round(ui)), int(round(vi))))
            elif keep_gaps:
                pts.append(None)
        return pts

    def project_ego_xy(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        *,
        x_min: float = 1.5,
        z: Optional[float] = None,
    ) -> List[Tuple[int, int]]:
        """PP / HUD: lateral path on plane Z≈camera height (LaneOverlayView.projectEgo)."""
        z_val = self.camera_height_m if z is None else float(z)
        zs = np.full_like(np.asarray(xs, dtype=np.float64), z_val)
        return self.project_device(xs, ys, zs, x_min=x_min, path_lift=False)


OverlayGeom = Union["PhoneRtGeometry", object]  # CameraGeometry duck-typed


def project_overlay_xyz(
    xs: Sequence[float] | np.ndarray,
    ys: Sequence[float] | np.ndarray,
    zs: Sequence[float] | np.ndarray,
    geom: OverlayGeom,
    w: int,
    h: int,
    *,
    x_min: float = 1.5,
    y_sign: float = 1.0,
    path_lift: bool = False,
    margin: float = 80.0,
    keep_gaps: bool = False,
) -> List[Tuple[int, int]]:
    """Dispatch: PhoneRt (device Y-right) or AAD CameraGeometry (ISO via y_sign).

    ``keep_gaps`` marks out-of-frame points as ``None`` so a drawer can break the polyline there.
    """
    if isinstance(geom, PhoneRtGeometry):
        # Callers may still pass DRAW_Y_SIGN with device Y; undo to device.
        ys_dev = np.asarray(ys, dtype=np.float64)
        if abs(y_sign + 1.0) < 1e-9:
            ys_dev = -ys_dev
        elif abs(y_sign - 1.0) > 1e-9:
            ys_dev = y_sign * ys_dev
        g = geom
        if g.width != w or g.height != h:
            g = PhoneRtGeometry(
                fx=g.fx,
                fy=g.fy,
                cx=g.cx,
                cy=g.cy,
                roll_deg=g.roll_deg,
                pitch_deg=g.pitch_deg,
                yaw_deg=g.yaw_deg,
                width=w,
                height=h,
                path_lift_m=g.path_lift_m,
                camera_height_m=g.camera_height_m,
            )
        return g.project_device(
            np.asarray(xs, dtype=np.float64),
            ys_dev,
            np.asarray(zs, dtype=np.float64),
            x_min=x_min,
            path_lift=path_lift,
            margin=margin,
            keep_gaps=keep_gaps,
        )

    from .lane_projection import project_iso_xyz

    return project_iso_xyz(
        xs,
        ys,
        zs,
        geom,
        w,
        h,
        x_min=x_min,
        y_sign=y_sign,
        margin=margin,
        keep_gaps=keep_gaps,
    )
