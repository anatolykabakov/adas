#!/usr/bin/env python3
"""Temporary: run supercombo.onnx and draw BEV (same as original demo)."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .lane_projection import CameraGeometry, project_iso_xyz
from .phone_rt import PhoneRtGeometry, project_overlay_xyz
from .supercombo_parse import (
    X_IDXS,
    parse_supercombo,
    SupercomboOut,
    explain_output,
)

# Extra model directory (optional): SUPERCOMBO_DIR=/path/to/models
SUPERCOMBO_DIR = (
    Path(os.environ["SUPERCOMBO_DIR"]) if os.environ.get("SUPERCOMBO_DIR") else None
)
# Search order: explicit path → SUPERCOMBO_MODEL → repo models/ → app assets/
# supercombo_compare.py -> scripts/core/ -> scripts/ -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[2]
_ASSETS_MODEL = _REPO_ROOT / "app" / "src" / "main" / "assets" / "supercombo.onnx"
# Repo downloads (gitignored): models/
_REPO_MODELS = _REPO_ROOT / "models"
_DEFAULT_DEVICE_MODEL = _REPO_MODELS / "sc_v0.8.13.onnx"


def resolve_supercombo_model(explicit: Optional[str | Path] = None) -> Path:
    """Resolve supercombo.onnx path (env / assets / models/sc_v0.8.13 / default dir)."""
    if explicit is not None:
        p = Path(explicit).expanduser()
        if p.is_file():
            return p
    env_model = os.environ.get("SUPERCOMBO_MODEL")
    if env_model:
        p = Path(env_model).expanduser()
        if p.is_file():
            return p
    candidates = [
        _DEFAULT_DEVICE_MODEL,
        _ASSETS_MODEL,
        _REPO_MODELS / "sc_v0.8.5.onnx",
    ]
    if SUPERCOMBO_DIR is not None:
        candidates.append(SUPERCOMBO_DIR / "supercombo.onnx")
    for c in candidates:
        if c.is_file():
            return c
    return candidates[0]


DEFAULT_MODEL = resolve_supercombo_model()


def parse_image_yuv(frame_yuv_i420: np.ndarray) -> np.ndarray:
    H = (frame_yuv_i420.shape[0] * 2) // 3
    W = frame_yuv_i420.shape[1]
    parsed = np.zeros((6, H // 2, W // 2), dtype=np.uint8)
    parsed[0] = frame_yuv_i420[0:H:2, 0::2]
    parsed[1] = frame_yuv_i420[1:H:2, 0::2]
    parsed[2] = frame_yuv_i420[0:H:2, 1::2]
    parsed[3] = frame_yuv_i420[1:H:2, 1::2]
    parsed[4] = frame_yuv_i420[H : H + H // 4].reshape((-1, H // 2, W // 2))
    parsed[5] = frame_yuv_i420[H + H // 4 : H + H // 2].reshape((-1, H // 2, W // 2))
    return parsed


def make_overlay_geometry(
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    w: int,
    h: int,
    camera_height: float = 1.22,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    roll_deg: float = 0.0,
    cam_x: float = 0.0,
    cam_y_left: float = 0.0,
) -> CameraGeometry:
    """Build an AAD CameraGeometry from **our** calibration angles.

    Every caller passes the RPY that `calibration/camera` publishes — the same numbers
    ``ModelCalibWarp`` / ``warp_matrix_deg`` use to build the model input. AAD's ``CameraGeometry``
    runs **pitch and yaw the other way** (its own docstring: ``pitch_deg < 0`` = looking down), while
    roll agrees. Handing it the calibration unconverted does not tilt the overlay a little — it tilts
    it by *twice* the angle, in the wrong direction, and that was drawing bag overlays off the road:
    at pitch −1.21° / yaw −1.66° the lines landed 24 px high and 28 px right on a 640×360 preview,
    which is ~2.9 m of lateral miss at 50 m, and far points crossed the horizon instead of lying on
    the tarmac.

    Verified against ``PhoneRtGeometry`` (the inverse of the warp, and what the phone screen draws):
    with pitch/yaw negated and roll kept, the two projectors agree to **0 px** over roll ±5°,
    pitch ±3°, yaw ±2.5°; unconverted they differ by up to 53 px.
    """
    if fy <= 1.0 or (fx > 1.0 and abs(fx / max(fy, 1e-6) - 1.0) > 0.15):
        fy = fx
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    return CameraGeometry(
        height=camera_height,
        pitch_deg=-float(pitch_deg),
        yaw_deg=-float(yaw_deg),
        roll_deg=roll_deg,
        cam_x=cam_x,
        cam_y_left=cam_y_left,
        image_width=w,
        image_height=h,
        intrinsic_matrix=K,
    )


def project_xyz(
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    camera_height: float,
    y_sign: float,
    w: int,
    h: int,
    x_min: float = 1.5,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    geom: Optional[CameraGeometry] = None,
) -> List[Tuple[int, int]]:
    """Ego (X fwd, Y left, Z up) → image via AAD CameraGeometry (pitch/yaw)."""
    if geom is None:
        geom = make_overlay_geometry(
            fx, fy, cx, cy, w, h, camera_height, pitch_deg=pitch_deg, yaw_deg=yaw_deg
        )
    return project_iso_xyz(xs, ys, zs, geom, w, h, x_min=x_min, y_sign=y_sign)


# Nearest distance drawn for a lane, edge or plan [m]. The lateral projection is fx·y/x, so it runs
# away as x → 0: a line 1.5 m off centre sits at ±422 px at x=1.69 m on a 640-wide frame. Those points
# carry no information about where the lane is on the road, and drawing them only produced long streaks.
LANE_X_MIN_M = 4.0


def draw_pts(img: np.ndarray, pts: List[Tuple[int, int]], color, thickness: int) -> None:
    """Draw a polyline, breaking it wherever the projector reported a gap (``None``)."""
    for a, b in zip(pts[:-1], pts[1:]):
        if a is None or b is None:
            continue
        cv2.line(img, a, b, color, thickness, lineType=cv2.LINE_AA)


def draw_runtime_lanes(
    img: np.ndarray,
    out: SupercomboOut,
    geom: CameraGeometry | PhoneRtGeometry,
    w: int,
    h: int,
    y_sign: float = -1.0,
    min_lane_prob: float = 0.3,
) -> None:
    """Draw lane lines from live supercombo inference."""
    for lane in out.lanes:
        if lane.prob < min_lane_prob:
            continue
        zs = getattr(lane, "z", None)
        if zs is None:
            zs = np.zeros_like(lane.y)
        pts = project_overlay_xyz(
            X_IDXS,
            lane.y,
            zs,
            geom,
            w,
            h,
            y_sign=y_sign,
        )
        draw_pts(img, pts, (0, 255, 255), 2)


def draw_bag_lanes(
    img: np.ndarray,
    lane_msg: Any,
    geom: CameraGeometry | PhoneRtGeometry,
    w: int,
    h: int,
    y_sign: float = -1.0,
    min_lane_prob: float = 0.3,
    draw_edges: bool = True,
    draw_plan: bool = True,
) -> None:
    """Draw lane lines from bag ``vision/lanes``.

    Values are device Y-right (flowpilot). AAD needs ``y_sign=-1`` (ISO);
    phone-Rt undoes that automatically when ``y_sign=-1``.
    """
    xs = np.asarray(list(lane_msg.x), dtype=np.float64) if lane_msg.x else X_IDXS.copy()
    for lane in lane_msg.lanes:
        if float(lane.prob) < min_lane_prob:
            continue
        y = np.asarray(list(lane.y), dtype=np.float64)
        if y.size != xs.size:
            continue
        pts = project_overlay_xyz(
            xs,
            y,
            np.zeros_like(y),
            geom,
            w,
            h,
            x_min=LANE_X_MIN_M,
            y_sign=y_sign,
            keep_gaps=True,
        )
        draw_pts(img, pts, (0, 255, 255), 2)

    if draw_edges:
        for edge in getattr(lane_msg, "edges", []):
            y = np.asarray(list(edge.y), dtype=np.float64)
            if y.size != xs.size:
                continue
            pts = project_overlay_xyz(
                xs,
                y,
                np.zeros_like(y),
                geom,
                w,
                h,
                x_min=LANE_X_MIN_M,
                y_sign=y_sign,
                keep_gaps=True,
            )
            draw_pts(img, pts, (0, 0, 255), 2)

    if (
        draw_plan
        and getattr(lane_msg, "plan_x", None)
        and getattr(lane_msg, "plan_y", None)
    ):
        px = np.asarray(list(lane_msg.plan_x), dtype=np.float64)
        py = np.asarray(list(lane_msg.plan_y), dtype=np.float64)
        if px.size >= 2 and px.size == py.size:
            pts = project_overlay_xyz(
                px,
                py,
                np.zeros_like(py),
                geom,
                w,
                h,
                x_min=LANE_X_MIN_M,
                y_sign=y_sign,
                keep_gaps=True,
                path_lift=isinstance(geom, PhoneRtGeometry),
            )
            draw_pts(img, pts, (0, 255, 0), 3)


def supercombo_lanes_to_ego(
    out: SupercomboOut,
    *,
    min_lane_prob: float = 0.3,
    y_sign: float = 1.0,
    x_min: float = 1.0,
    x_max: float = 40.0,
) -> Dict[str, np.ndarray]:
    """Near-lanes → Nx2 polylines.

    Default ``y_sign=1`` keeps **device Y-right** (Android PP). Pass ``y_sign=-1``
    only for ISO Y-left MetaDrive GT-style overlays.
    """
    # LANE_NAMES: leftFar, leftNear, rightNear, rightFar
    by_name = {lane.name: lane for lane in out.lanes}
    left = by_name.get("leftNear")
    right = by_name.get("rightNear")
    result: Dict[str, np.ndarray] = {
        "left_road": np.empty((0, 2), dtype=np.float64),
        "right_road": np.empty((0, 2), dtype=np.float64),
    }
    xs = X_IDXS
    ok = (xs >= x_min) & (xs <= x_max) & np.isfinite(xs)
    if left is not None and left.prob >= min_lane_prob:
        y = y_sign * np.asarray(left.y, dtype=np.float64)
        m = ok & np.isfinite(y)
        result["left_road"] = np.stack([xs[m], y[m]], axis=1)
    if right is not None and right.prob >= min_lane_prob:
        y = y_sign * np.asarray(right.y, dtype=np.float64)
        m = ok & np.isfinite(y)
        result["right_road"] = np.stack([xs[m], y[m]], axis=1)
    return result


def draw_supercombo_overlay(
    img: np.ndarray,
    out: SupercomboOut,
    geom: CameraGeometry | PhoneRtGeometry,
    w: int,
    h: int,
    *,
    y_sign: float = -1.0,
    min_lane_prob: float = 0.3,
    draw_plan: bool = True,
    draw_edges: bool = True,
    draw_lanes: bool = True,
    lane_tag: str = "runtime",
) -> None:
    """Draw plan (green), lanes (yellow), road edges (red) — same as visualizer."""
    if draw_edges:
        for edge in out.edges:
            zs = getattr(edge, "z", None)
            if zs is None:
                zs = np.zeros_like(edge.y)
            pts = project_overlay_xyz(
                X_IDXS,
                edge.y,
                zs,
                geom,
                w,
                h,
                y_sign=y_sign,
            )
            draw_pts(img, pts, (0, 0, 255), 2)

    if draw_lanes:
        draw_runtime_lanes(
            img, out, geom, w, h, y_sign=y_sign, min_lane_prob=min_lane_prob
        )

    if draw_plan:
        pts = project_overlay_xyz(
            out.plan.x,
            out.plan.y,
            out.plan.z,
            geom,
            w,
            h,
            x_min=0.5,
            y_sign=y_sign,
            path_lift=isinstance(geom, PhoneRtGeometry),
        )
        draw_pts(img, pts, (0, 255, 0), 3)

    mode = "phone-rt" if isinstance(geom, PhoneRtGeometry) else "aad"
    probs = [f"{lane.prob:.2f}" for lane in out.lanes]
    cv2.putText(
        img,
        f"supercombo plan#{out.plan.hyp_index}  lanes={lane_tag}  draw={mode}  p={probs}",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


class SupercomboBev:
    """Lazy ONNX runner; Android-parity preprocess (calib warp + RNN) + parse."""

    def __init__(self, model_path: Optional[Path] = None):
        self.model_path = resolve_supercombo_model(model_path)
        self._session = None
        self._names: Optional[Tuple[str, str, str, str, str]] = None
        self._prev: Optional[np.ndarray] = None
        self._prev_key: Optional[int] = None
        self._cache_key: Optional[int] = None
        self._cache_out: Optional[SupercomboOut] = None
        self._rnn = np.zeros((1, 512), np.float32)
        self.error: Optional[str] = None
        # Calib for ModelCalibWarp (phone full-res prior by default).
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.fx = 930.0
        self.fy = 930.0
        self.cx = 640.0
        self.cy = 360.0
        self.use_calib_warp = True

    def set_calib(
        self,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        fx: float = 930.0,
        fy: float = 930.0,
        cx: float = 640.0,
        cy: float = 360.0,
        *,
        use_warp: bool = True,
    ) -> None:
        changed = (
            self.roll_deg != float(roll_deg)
            or self.pitch_deg != float(pitch_deg)
            or self.yaw_deg != float(yaw_deg)
            or self.fx != float(fx)
            or self.fy != float(fy)
            or self.cx != float(cx)
            or self.cy != float(cy)
            or self.use_calib_warp != bool(use_warp)
        )
        self.roll_deg = float(roll_deg)
        self.pitch_deg = float(pitch_deg)
        self.yaw_deg = float(yaw_deg)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.use_calib_warp = bool(use_warp)
        if changed:
            # Intrinsics / RPY change → temporal stack + RNN invalid.
            self.reset()

    def _ensure(self) -> bool:
        if self._session is not None:
            return True
        if not self.model_path.is_file():
            self.error = f"model missing: {self.model_path}"
            return False
        try:
            import onnxruntime as ort

            self._session = ort.InferenceSession(
                str(self.model_path), providers=["CPUExecutionProvider"]
            )
            ins = self._session.get_inputs()
            outs = self._session.get_outputs()
            self._names = (
                ins[0].name,
                ins[1].name,
                ins[2].name,
                ins[3].name,
                outs[0].name,
            )
            self.error = None
            return True
        except Exception as e:
            self.error = str(e)
            return False

    def reset(self) -> None:
        self._prev = None
        self._prev_key = None
        self._cache_key = None
        self._cache_out = None
        self._rnn = np.zeros((1, 512), np.float32)

    def _preprocess_yuv6(self, bgr: np.ndarray) -> np.ndarray:
        from .model_calib_warp import warp_matrix_deg, warp_to_model

        if self.use_calib_warp:
            m = warp_matrix_deg(
                self.roll_deg,
                self.pitch_deg,
                self.yaw_deg,
                self.fx,
                self.fy,
                self.cx,
                self.cy,
            )
            img = warp_to_model(bgr, m)
        else:
            img = cv2.resize(bgr, (512, 256))
        yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV_I420)
        return parse_image_yuv(yuv).astype(np.float32)

    def infer(
        self, bgr: np.ndarray, cache_key: Optional[int] = None
    ) -> Optional[SupercomboOut]:
        if (
            cache_key is not None
            and cache_key == self._cache_key
            and self._cache_out is not None
        ):
            return self._cache_out
        if not self._ensure():
            return None

        assert self._session is not None and self._names is not None
        parsed = self._preprocess_yuv6(bgr)
        # Temporal pair must be consecutive frames. Skipping (fast play / scrub)
        # makes lanes lag — duplicate current instead of a stale _prev (Android-like).
        consecutive = (
            cache_key is not None
            and self._prev_key is not None
            and cache_key == self._prev_key + 1
        )
        if self._prev is None or not consecutive:
            prev = parsed
        else:
            prev = self._prev
        data = np.stack([prev, parsed], axis=0).reshape(1, 12, 128, 256)
        self._prev = parsed
        self._prev_key = cache_key

        desire = np.zeros((1, 8), np.float32)
        traffic = np.array([[1.0, 0.0]], np.float32)
        in_imgs, in_desire, in_traffic, in_state, out_name = self._names
        (out,) = self._session.run(
            [out_name],
            {
                in_imgs: data,
                in_desire: desire,
                in_traffic: traffic,
                in_state: self._rnn,
            },
        )
        flat = out.reshape(-1)
        # Recurrent GRU state — last 512 floats (Android SupercomboOnnxRunner).
        if flat.size >= 512:
            self._rnn = flat[-512:].astype(np.float32).reshape(1, 512).copy()
        parsed_out = parse_supercombo(flat)
        if cache_key is not None:
            self._cache_key = cache_key
            self._cache_out = parsed_out
        return parsed_out

    def overlay_on_image(
        self,
        bgr: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        camera_height: float = 1.22,
        cache_key: Optional[int] = None,
        y_sign: float = -1.0,
        min_lane_prob: float = 0.3,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        roll_deg: float = 0.0,
        geom: Optional[CameraGeometry] = None,
    ) -> np.ndarray:
        """Draw PLAN (green), lanes (yellow), road edges (red) on camera image.

        ``y_sign=-1``: this ONNX matches Android ``u=cx+fx*Y/X`` (Y positive right).
        ``project_iso_xyz`` assumes ISO Y-left, so we flip once.
        """
        h_img, w_img = bgr.shape[:2]
        out = self.infer(bgr, cache_key=cache_key)
        if out is None:
            cv2.putText(
                bgr,
                f"supercombo ERR: {(self.error or '')[:50]}",
                (8, h_img - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
            return bgr

        if geom is None:
            geom = make_overlay_geometry(
                fx,
                fy,
                cx,
                cy,
                w_img,
                h_img,
                camera_height,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                roll_deg=roll_deg,
            )

        # road edges (z≈noise in this ONNX — project on ground like LaneOverlayView)
        for edge in out.edges:
            pts = project_iso_xyz(
                X_IDXS,
                edge.y,
                np.zeros_like(edge.y),
                geom,
                w_img,
                h_img,
                y_sign=y_sign,
            )
            draw_pts(bgr, pts, (0, 0, 255), 2)

        draw_runtime_lanes(
            bgr,
            out,
            geom,
            w_img,
            h_img,
            y_sign=y_sign,
            min_lane_prob=min_lane_prob,
        )

        # planned path (best of 5 MHP) — this is the primary model output
        pts = project_iso_xyz(
            out.plan.x,
            out.plan.y,
            out.plan.z,
            geom,
            w_img,
            h_img,
            x_min=0.5,
            y_sign=y_sign,
        )
        draw_pts(bgr, pts, (0, 255, 0), 3)

        probs = [f"{l.prob:.2f}" for l in out.lanes]
        cv2.putText(
            bgr,
            f"plan hyp#{out.plan.hyp_index}  lanes p={probs}",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            bgr,
            "green=PLAN  yellow=lanes  red=edges",
            (8, h_img - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        return bgr
