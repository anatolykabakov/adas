#!/usr/bin/env python3
"""VP camera calib — Hough/image helpers + ``pyadas.AdasApp`` (Simulated).

Host (sim / bag): extract image-space lane lines → UV → publish_lane_uv + step.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence, Tuple

import cv2
import numpy as np

Line2 = Tuple[float, float]  # (m, c) for v = m*u + c, or poly1d coeffs


def get_intersection(line1: Line2, line2: Line2) -> Optional[Tuple[float, float]]:
    """AAD ``get_intersection``: intersection of v=m1*u+c1 and v=m2*u+c2."""
    m1, c1 = line1
    m2, c2 = line2
    if abs(m1 - m2) < 1e-9:
        return None
    u_i = (c2 - c1) / (m1 - m2)
    v_i = m1 * u_i + c1
    return float(u_i), float(v_i)


def fit_line_v_of_u(
    us: np.ndarray,
    vs: np.ndarray,
    mean_residuals_thresh: float = 25.0,
) -> Optional[Line2]:
    """AAD ``_fit_line_v_of_u``: polyfit v(u), reject curved / noisy fits."""
    us = np.asarray(us, dtype=np.float64).reshape(-1)
    vs = np.asarray(vs, dtype=np.float64).reshape(-1)
    mask = np.isfinite(us) & np.isfinite(vs)
    us, vs = us[mask], vs[mask]
    if us.size < 8:
        return None
    coeffs, residuals, _, _, _ = np.polyfit(us, vs, deg=1, full=True)
    if residuals.size == 0:
        return None
    mean_residuals = float(residuals[0] / len(us))
    if mean_residuals > mean_residuals_thresh:
        return None
    m, c = float(coeffs[0]), float(coeffs[1])
    return m, c


def _sample_line_uv(
    line: Line2, w: int, h: int, n: int = 24
) -> List[Tuple[float, float]]:
    """Sample v=m*u+c in the lower ~55% of the frame (road region)."""
    m, c = line
    v_lo = 0.45 * float(h)
    # u such that v = m*u+c ∈ [v_lo, h-1]
    us = np.linspace(0.0, float(max(w - 1, 1)), n * 2)
    pts: List[Tuple[float, float]] = []
    for u in us:
        v = m * u + c
        if v_lo <= v <= float(h - 1):
            pts.append((float(u), float(v)))
    if len(pts) >= 8:
        return pts[:: max(1, len(pts) // n)][:n]
    # Fallback: full-width sample
    us = np.linspace(0.0, float(max(w - 1, 1)), n)
    return [(float(u), float(m * u + c)) for u in us]


@dataclass
class VanishingPointCalibrator:
    """Online AAD calibrator via Simulated ``pyadas.AdasApp``."""

    history_len: int = 50
    mean_residuals_thresh: float = 25.0
    estimated_pitch_deg: float = 0.0
    estimated_yaw_deg: float = 0.0
    camera_height_m: float = 1.40
    calibration_success: bool = False
    pitch_yaw_history: List[List[float]] = field(default_factory=list)
    last_vp: Optional[Tuple[float, float]] = None
    n_updates: int = 0
    _app: Any = field(default=None, repr=False, compare=False)
    _owns_app: bool = field(default=True, repr=False, compare=False)
    _t_us: int = field(default=0, repr=False, compare=False)

    def __post_init__(self) -> None:
        from pyadas import core as pyadas

        if self._app is None:
            app = pyadas.AdasApp(
                wheelbase=2.636,
                pitch0_deg=float(self.estimated_pitch_deg),
                yaw0_deg=float(self.estimated_yaw_deg),
                camera_height=float(self.camera_height_m),
                camera_calib_history_len=int(self.history_len),
            )
            object.__setattr__(self, "_app", app)
            object.__setattr__(self, "_owns_app", True)
        else:
            object.__setattr__(self, "_owns_app", False)
            self._app.set_camera_estimate(
                float(self.estimated_pitch_deg), float(self.estimated_yaw_deg)
            )
            self._app.set_camera_height(float(self.camera_height_m))

    def reset(self) -> None:
        self.pitch_yaw_history.clear()
        self.calibration_success = False
        self.last_vp = None
        self.n_updates = 0
        if self._app is not None:
            self._app.reset_camera_calib()
            self._app.set_camera_estimate(
                self.estimated_pitch_deg, self.estimated_yaw_deg
            )
            self._app.set_camera_height(self.camera_height_m)
