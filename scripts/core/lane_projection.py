#!/usr/bin/env python3
"""Project vision/lanes onto camera image.

Geometry follows *Algorithms for Automated Driving* (AAD)
`code/solutions/lane_detection/camera_geometry.py`:

  - Road frame: X right, Y down, Z forward; road plane Y=0
  - Camera: OpenCV (X right, Y down, Z forward)
  - ``pitch_deg < 0`` → looking down (AAD sign)
  - ``project_polyline``: ``uv ~ K @ T_road_to_cam @ [X,Y,Z,1]``

Our bag lanes are ISO 8855 ego:
  X forward, Y left, Z up, ground Z=0.

Conversion (same as AAD ``uv_to_roadXYZ_roadframe_iso8855`` inverse):
  (X_r, Y_r, Z_r) = (−Y_iso, −Z_iso, X_iso)

Golf 7 windshield prior
-----------------------
  height = 1.40 m
  cam_x  = 1.70 m   (forward of ego ≈ rear axle → windshield, road Z)
  cam_y  = 0        (center)
  pitch_deg = 0     (default; set negative to look down)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np

from .supercombo_parse import X_IDXS


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


def get_intrinsic_matrix_fov(
    field_of_view_deg: float, image_width: int, image_height: int
) -> np.ndarray:
    """AAD ``get_intrinsic_matrix`` (square pixels, principal point at center)."""
    field_of_view_rad = field_of_view_deg * np.pi / 180.0
    alpha = (image_width / 2.0) / np.tan(field_of_view_rad / 2.0)
    cu = image_width / 2.0
    cv = image_height / 2.0
    return np.array(
        [[alpha, 0.0, cu], [0.0, alpha, cv], [0.0, 0.0, 1.0]], dtype=np.float64
    )


def project_polyline(
    polyline_world: np.ndarray, trafo_world_to_cam: np.ndarray, K: np.ndarray
) -> np.ndarray:
    """AAD ``project_polyline``: world Nx3 → image Nx2."""
    x, y, z = polyline_world[:, 0], polyline_world[:, 1], polyline_world[:, 2]
    homvec = np.stack((x, y, z, np.ones_like(x)))
    pl_uv_cam = (K @ trafo_world_to_cam[:3, :] @ homvec).T
    u = pl_uv_cam[:, 0] / pl_uv_cam[:, 2]
    v = pl_uv_cam[:, 1] / pl_uv_cam[:, 2]
    return np.stack((u, v), axis=1)


class CameraGeometry:
    """AAD CameraGeometry + Golf 7 longitudinal offset.

    Parameters match AAD (pitch_deg negative = looking down).
    ``cam_x`` shifts the camera forward along the road (Z_road).
    """

    def __init__(
        self,
        height: float = 1.4,
        yaw_deg: float = 0.0,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0,
        cam_x: float = 1.70,
        cam_y_left: float = 0.0,
        image_width: int = 640,
        image_height: int = 360,
        field_of_view_deg: Optional[float] = None,
        intrinsic_matrix: Optional[np.ndarray] = None,
    ):
        self.height = float(height)
        self.pitch_deg = float(pitch_deg)
        self.roll_deg = float(roll_deg)
        self.yaw_deg = float(yaw_deg)
        self.cam_x = float(cam_x)
        self.cam_y_left = float(cam_y_left)
        self.image_width = int(image_width)
        self.image_height = int(image_height)

        if intrinsic_matrix is not None:
            self.intrinsic_matrix = np.asarray(intrinsic_matrix, dtype=np.float64)
        else:
            fov = 45.0 if field_of_view_deg is None else float(field_of_view_deg)
            self.field_of_view_deg = fov
            self.intrinsic_matrix = get_intrinsic_matrix_fov(
                fov, self.image_width, self.image_height
            )

        self.inverse_intrinsic_matrix = np.linalg.inv(self.intrinsic_matrix)

        yaw = np.deg2rad(self.yaw_deg)
        pitch = np.deg2rad(self.pitch_deg)
        roll = np.deg2rad(self.roll_deg)
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)

        # Exact AAD rotation_road_to_cam
        rotation_road_to_cam = np.array(
            [
                [cr * cy + sp * sr * sy, cr * sp * sy - cy * sr, -cp * sy],
                [cp * sr, cp * cr, sp],
                [cr * sy - cy * sp * sr, -cr * cy * sp - sr * sy, cp * cy],
            ],
            dtype=np.float64,
        )
        self.rotation_cam_to_road = rotation_road_to_cam.T
        # AAD: [0, -height, 0]; add longitudinal / lateral mount offset in road frame
        # ISO cam at (cam_x, cam_y_left, height) → road ( -cam_y_left, -height, cam_x )
        self.translation_cam_to_road = np.array(
            [-self.cam_y_left, -self.height, self.cam_x], dtype=np.float64
        )
        self.trafo_cam_to_road = np.eye(4, dtype=np.float64)
        self.trafo_cam_to_road[0:3, 0:3] = self.rotation_cam_to_road
        self.trafo_cam_to_road[0:3, 3] = self.translation_cam_to_road
        self.trafo_road_to_cam = np.linalg.inv(self.trafo_cam_to_road)
        self.road_normal_camframe = self.rotation_cam_to_road.T @ np.array(
            [0.0, 1.0, 0.0]
        )

    @staticmethod
    def golf7_windshield(
        height: float = 1.4,
        pitch_down_deg: float = 0.0,
        cam_x: float = 1.70,
        **kwargs: Any,
    ) -> "CameraGeometry":
        """Golf 7 center-windshield prior (pitch_down_deg > 0 → AAD pitch_deg < 0)."""
        return CameraGeometry(
            height=height,
            pitch_deg=-abs(pitch_down_deg) if pitch_down_deg else 0.0,
            cam_x=cam_x,
            **kwargs,
        )


def iso_to_road_points(
    X_fwd: np.ndarray,
    Y_left: np.ndarray,
    Z_up: float | np.ndarray = 0.0,
) -> np.ndarray:
    """ISO8855 ego points → AAD road-frame Nx3.

    (X_r, Y_r, Z_r) = (−Y_iso, −Z_iso, X_iso)
    """
    X_fwd = np.asarray(X_fwd, dtype=np.float64)
    Y_left = np.asarray(Y_left, dtype=np.float64)
    if np.isscalar(Z_up):
        Z = np.full_like(X_fwd, -float(Z_up))
    else:
        Z = -np.asarray(Z_up, dtype=np.float64)
    return np.stack((-Y_left, Z, X_fwd), axis=1)


def project_iso_xyz(
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
    geom: "CameraGeometry",
    w: int,
    h: int,
    x_min: float = 1.5,
    y_sign: float = 1.0,
    margin: float = 80.0,
    keep_gaps: bool = False,
) -> list:
    """ISO ego XYZ → image pixel list via AAD CameraGeometry.

    With ``keep_gaps`` a point outside the frame becomes ``None`` instead of vanishing from the list.
    Silently dropping it makes the two surviving neighbours adjacent, and a polyline drawer then joins
    them with a straight line across the whole image — the near field of a 0…192 m lane diverges
    laterally (a 1.5 m line sits at ±422 px at x=1.69 m on a 640-wide frame), so those joins looked
    like lane lines lying far off the road.
    """
    xs = np.asarray(xs, dtype=np.float64)
    ys = y_sign * np.asarray(ys, dtype=np.float64)
    zs = np.asarray(zs, dtype=np.float64)
    if zs.shape != xs.shape:
        zs = np.zeros_like(xs)

    ok = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs) & (xs >= x_min)
    if not np.any(ok):
        return []
    road = iso_to_road_points(xs[ok], ys[ok], zs[ok])
    uv = project_polyline(road, geom.trafo_road_to_cam, geom.intrinsic_matrix)
    cam = (geom.trafo_road_to_cam @ np.vstack([road.T, np.ones((1, road.shape[0]))])).T
    pts = []
    for i in range(uv.shape[0]):
        if cam[i, 2] <= 0.2:
            if keep_gaps:
                pts.append(None)
            continue
        u, v = float(uv[i, 0]), float(uv[i, 1])
        if -margin <= u < w + margin and -margin <= v < h + margin:
            pts.append((int(round(u)), int(round(v))))
        elif keep_gaps:
            pts.append(None)
    return pts


def intrinsics_from_messages(
    cam_msg: Any = None,
    intr_msg: Any = None,
    width: int = 640,
    height: int = 360,
) -> CameraIntrinsics:
    """Build K from CameraImage and/or CameraIntrinsics proto."""
    w = int(
        getattr(cam_msg, "width", 0) or getattr(intr_msg, "capture_width", 0) or width
    )
    h = int(
        getattr(cam_msg, "height", 0) or getattr(intr_msg, "capture_height", 0) or height
    )

    fx = float(getattr(cam_msg, "focal_length_x", 0.0) or 0.0)
    fy = float(getattr(cam_msg, "focal_length_y", 0.0) or 0.0)
    cx = float(getattr(cam_msg, "principal_point_x", 0.0) or 0.0)
    cy = float(getattr(cam_msg, "principal_point_y", 0.0) or 0.0)

    if fx <= 1.0 and intr_msg is not None:
        f_mm = float(getattr(intr_msg, "physical_focal_length_mm", 0.0) or 0.0)
        sw = float(getattr(intr_msg, "sensor_width_mm", 0.0) or 0.0)
        sh = float(getattr(intr_msg, "sensor_height_mm", 0.0) or 0.0)
        if f_mm > 0 and sw > 0 and sh > 0:
            # Capture buffer is already WxH with (approx) square pixels.
            # Do NOT use sensor_height for fy — that assumes the full 4:3 sensor
            # is mapped to the frame and makes fy too small on 16:9 crops,
            # which lifts ground projections up by ~camera-height in the image.
            fx = f_mm / sw * w
            fy = fx

    if fx <= 1.0:
        K = get_intrinsic_matrix_fov(70.0, w, h)
        fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    if fy <= 1.0:
        fy = fx
    # Prefer square pixels when fy looks anamorphic from bad sensor_height mapping
    if fx > 1.0 and fy > 1.0 and abs(fx / fy - 1.0) > 0.15:
        fy = fx
    if cx <= 1.0:
        cx = 0.5 * w
    if cy <= 1.0:
        cy = 0.5 * h

    return CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h)


# Back-compat alias used by interactive_visualizer / bag_overlay
@dataclass
class CameraExtrinsics:
    """Thin wrapper kept for existing call sites (pitch stored as looking-down radians)."""

    x: float = 1.70
    y: float = 0.00
    z: float = 1.40
    roll: float = 0.0
    pitch: float = np.deg2rad(6.0)  # looking-down magnitude (UI)
    yaw: float = 0.0

    @staticmethod
    def golf7_windshield(height_m: float = 1.40) -> "CameraExtrinsics":
        return CameraExtrinsics(x=1.70, y=0.0, z=height_m, pitch=np.deg2rad(6.0))

    def to_geometry(self, K: CameraIntrinsics) -> CameraGeometry:
        return CameraGeometry(
            height=self.z,
            yaw_deg=np.degrees(self.yaw),
            pitch_deg=-abs(np.degrees(self.pitch)),
            roll_deg=np.degrees(self.roll),
            cam_x=self.x,
            cam_y_left=self.y,
            image_width=K.width,
            image_height=K.height,
            intrinsic_matrix=K.K(),
        )


def project_points(
    X: np.ndarray,
    Y: np.ndarray,
    K: CameraIntrinsics,
    extrinsics: CameraExtrinsics,
    z_min: float = 0.5,
    x_min: float = 1.5,
) -> np.ndarray:
    """ISO lane samples → image uv (NaN if behind camera / out of frame)."""
    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    geom = extrinsics.to_geometry(K)
    road = iso_to_road_points(X, Y, 0.0)
    uv = project_polyline(road, geom.trafo_road_to_cam, geom.intrinsic_matrix)

    # Invalidate points behind camera or too near ego
    cam = (geom.trafo_road_to_cam @ np.vstack([road.T, np.ones((1, len(X)))])).T
    valid = (cam[:, 2] > z_min) & (X >= x_min)
    out = np.full_like(uv, np.nan)
    out[valid] = uv[valid]

    in_frame = (
        (out[:, 0] >= -50)
        & (out[:, 0] < K.width + 50)
        & (out[:, 1] >= -50)
        & (out[:, 1] < K.height + 50)
    )
    out[~in_frame] = np.nan
    return out


def lane_xy_arrays(lane_msg: Any) -> Tuple[np.ndarray, List[np.ndarray], List[float]]:
    xs = np.asarray(list(lane_msg.x), dtype=np.float64) if lane_msg.x else X_IDXS.copy()
    ys: List[np.ndarray] = []
    probs: List[float] = []
    for lane in lane_msg.lanes:
        ys.append(np.asarray(list(lane.y), dtype=np.float64))
        probs.append(float(lane.prob))
    return xs, ys, probs


def draw_polyline(
    img: np.ndarray,
    uv: np.ndarray,
    color: Tuple[int, int, int],
    thickness: int = 2,
) -> None:
    pts = [
        (int(round(u)), int(round(v))) for u, v in uv if np.isfinite(u) and np.isfinite(v)
    ]
    for a, b in zip(pts[:-1], pts[1:]):
        cv2.line(img, a, b, color, thickness, lineType=cv2.LINE_AA)


def overlay_lanes(
    img: np.ndarray,
    lane_msg: Any,
    K: CameraIntrinsics,
    extrinsics: Optional[CameraExtrinsics] = None,
    min_prob: float = 0.15,
    draw_edges: bool = True,
) -> np.ndarray:
    if extrinsics is None:
        extrinsics = CameraExtrinsics.golf7_windshield()

    xs, ys, probs = lane_xy_arrays(lane_msg)
    colors = [
        (0, 255, 255),
        (0, 220, 255),
        (0, 200, 255),
        (0, 180, 255),
        (255, 200, 0),
        (255, 150, 0),
    ]

    for i, (y, prob) in enumerate(zip(ys, probs)):
        if prob < min_prob or y.size != xs.size:
            continue
        draw_polyline(
            img, project_points(xs, y, K, extrinsics), colors[i % len(colors)], 2
        )

    if draw_edges:
        for edge in getattr(lane_msg, "edges", []):
            y = np.asarray(list(edge.y), dtype=np.float64)
            if y.size != xs.size:
                continue
            draw_polyline(img, project_points(xs, y, K, extrinsics), (0, 0, 255), 2)

    hud = (
        f"AAD h={extrinsics.z:.2f}m x={extrinsics.x:.2f}m "
        f"pitch={-abs(np.degrees(extrinsics.pitch)):.1f}deg "
        f"fx={K.fx:.0f}"
    )
    cv2.putText(
        img, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA
    )
    return img


def explain_golf7_model() -> str:
    return """
Golf 7 + AAD CameraGeometry
===========================
Reference: Algorithms-for-Automated-Driving
  code/solutions/lane_detection/camera_geometry.py

Road frame (AAD):  X right, Y down, Z forward; ground Y=0
Camera (OpenCV):   X right, Y down, Z forward
pitch_deg < 0  → looking down (AAD sign)

ISO8855 ego lanes (bag):
  X forward, Y left, Z up → road (−Y, −Z, X)

Golf 7 windshield prior:
  height = 1.40 m
  cam_x  = 1.70 m   (ego rear-axle → windshield along Z_road)
  cam_y  = 0
  pitch_deg = 0     (default; set negative to look down)

Projection (AAD project_polyline):
  λ [u v 1]^T = K · T_road→cam · [X Y Z 1]^T

Tune: pitch (±2–3°), cam_x (±0.3 m), height if overlay is off.
"""
