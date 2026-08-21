"""Flowpilot / Android ModelCalibWarp — camera K + RPY → model 512×256 warp.

Matches ``adas.app.vision.ModelCalibWarp`` (medmodel FL=910, cy=47.6).
Homography maps **model pixel → camera pixel** (same as TransformCL).
"""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np

MODEL_W = 512
MODEL_H = 256
MED_FL = 910.0
MED_CY = 47.6
# flowpilot `SBIGMODEL_FL`: half the medmodel focal length, so twice the field of view. The 0.9.x
# network takes two pictures, narrow and wide; with one camera both come from the same frame, warped
# by different *model* intrinsics — the camera's own K does not change. Mirrors
# `ModelCalibWarp.warpMatrix(..., bigModel)`.
SBIG_FL = 455.0
SBIG_CY = 0.5 * (MODEL_H + MED_CY)

# device (x forward, y right, z down) → camera view
VIEW_FROM_DEVICE = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]], dtype=np.float64
)


def _rot_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Radians; same as Java ModelCalibWarp.rotFromEuler (then transpose)."""
    cp, sp = np.cos(pitch), np.sin(pitch)
    sr, cr = np.sin(roll), np.cos(roll)
    sy, cy = np.sin(yaw), np.cos(yaw)
    rot = np.array(
        [
            [cp * cy, cp * sy, -sp],
            [(sr * sp * cy) - (cr * sy), (sr * sp * sy) + (cr * cy), sr * cp],
            [(cr * sp * cy) + (sr * sy), (cr * sp * sy) - (sr * cy), cr * cp],
        ],
        dtype=np.float64,
    )
    return rot.T


def warp_matrix(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    big_model: bool = False,
) -> np.ndarray:
    """3×3 model→camera homography (row-major); ``big_model`` for the wide input."""
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    mfl = SBIG_FL if big_model else MED_FL
    mcy = SBIG_CY if big_model else MED_CY
    med_k = np.array(
        [[mfl, 0.0, 0.5 * MODEL_W], [0.0, mfl, mcy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    med_from_calib = med_k @ VIEW_FROM_DEVICE
    calib_from_model = np.linalg.inv(med_from_calib)
    device_from_calib = _rot_from_euler(roll_rad, pitch_rad, yaw_rad)
    view_from_calib = VIEW_FROM_DEVICE @ device_from_calib
    camera_from_calib = K @ view_from_calib
    return camera_from_calib @ calib_from_model


def warp_matrix_deg(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    big_model: bool = False,
) -> np.ndarray:
    return warp_matrix(
        np.deg2rad(roll_deg),
        np.deg2rad(pitch_deg),
        np.deg2rad(yaw_deg),
        fx,
        fy,
        cx,
        cy,
        big_model,
    )


def warp_to_model(bgr: np.ndarray, m_model_to_cam: np.ndarray) -> np.ndarray:
    """Warp BGR camera image to MODEL_W×MODEL_H (black outside)."""
    # cv2.warpPerspective expects dst←src map; we have model→cam, so invert.
    m_cam_to_model = np.linalg.inv(m_model_to_cam.astype(np.float64))
    return cv2.warpPerspective(
        bgr,
        m_cam_to_model,
        (MODEL_W, MODEL_H),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
