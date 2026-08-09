#!/usr/bin/env python3
"""Camera frame projected onto the road plane (inverse perspective).

Laid under the trajectory panel, it answers the question the lane lines alone cannot:
is the detector's geometry wrong, or is the planner reacting to geometry that is right?
Everything here assumes a flat road and the mounting from ``calib_rpy.json`` — a pitch
error of a degree already stretches the far end by metres, so read distances near the car.
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from core.lane_projection import iso_to_road_points, project_polyline


def ground_to_image(geom, x_fwd: np.ndarray, y_left: np.ndarray) -> np.ndarray:
    """ISO ground points (z = 0) → image pixels, floats, no clipping."""
    road = iso_to_road_points(np.asarray(x_fwd), np.asarray(y_left), 0.0)
    return project_polyline(road, geom.trafo_road_to_cam, geom.intrinsic_matrix)


def warp_to_road(
    img: np.ndarray,
    geom,
    x_range: Tuple[float, float] = (3.0, 45.0),
    y_half_m: float = 12.0,
    px_per_m: float = 8.0,
    fade_to: float = 0.35,
) -> Optional[np.ndarray]:
    """Top-down RGBA patch of the road ahead.

    Column 0 is ``x_range[0]`` and columns run forward; row 0 is ``+y_half_m`` (left) and
    rows run right — i.e. the patch is already in ego axes, ready for an ``imshow`` with
    ``extent=[x0, x1, -y_half, +y_half]``.
    """
    x0, x1 = float(x_range[0]), float(x_range[1])
    if x1 <= x0 or y_half_m <= 0.0:
        return None
    w = int(round((x1 - x0) * px_per_m))
    h = int(round(2.0 * y_half_m * px_per_m))
    if w < 8 or h < 8:
        return None

    corners_x = np.array([x0, x1, x1, x0], dtype=np.float64)
    corners_y = np.array([y_half_m, y_half_m, -y_half_m, -y_half_m], dtype=np.float64)
    src = ground_to_image(geom, corners_x, corners_y).astype(np.float32)
    if not np.isfinite(src).all():
        return None
    # Points at or above the horizon come back as huge or mirrored coordinates.
    if np.abs(src).max() > 1e5:
        return None

    dst = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src, dst)

    rgba = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
    rgba[:, :, 3] = 255
    out = cv2.warpPerspective(
        rgba,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0, 0),
    )
    # One image row covers metres at the far end and centimetres near the car; fading the
    # distance keeps the stretched part from reading as evidence.
    fade = np.linspace(1.0, fade_to, w, dtype=np.float32)
    out[:, :, 3] = (out[:, :, 3].astype(np.float32) * fade[None, :]).astype(np.uint8)
    return out
