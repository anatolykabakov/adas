"""Path fusion matching C++ ``adas::laneLinesToPath`` (device Y-right).

Use the returned Nx2 polyline with ``AdasApp.publish_lanes`` / PP — same frame
as Android TopicConvert → LaneKeep.
"""

from __future__ import annotations

from typing import Any, List, Optional, Sequence

import numpy as np

from .frames import DEFAULT_MIN_LANE_PROB, PP_Y_SIGN
from .pure_pursuit import plan_to_polyline_ego
from .supercombo_parse import X_IDXS, SupercomboOut


def _soft_lane_prob(p: float, min_p: float) -> float:
    if not (p >= min_p):
        return 0.0
    span = max(1e-3, 1.0 - float(min_p))
    return min(1.0, (float(p) - float(min_p)) / span)


def _interp_y(x: float, xs: np.ndarray, ys: np.ndarray) -> float:
    if xs.size == 0 or xs.size != ys.size:
        return 0.0
    if x <= xs[0]:
        return float(ys[0])
    if x >= xs[-1]:
        return float(ys[-1])
    return float(np.interp(x, xs, ys))


# Defaults kept in sync with C++ LanePathConfig / assets/config.json.
DEFAULT_LANE_STD_GOOD_M = 0.3
DEFAULT_LANE_STD_BAD_M = 1.5
DEFAULT_LANE_WIDTH_MIN_M = 2.6
DEFAULT_LANE_WIDTH_MAX_M = 4.6


def _median_std(stds: Optional[np.ndarray], xs: np.ndarray) -> float:
    """Median lane sigma over 5–40 m; -1 if sigma missing (no penalty)."""
    if stds is None:
        return -1.0
    s = np.asarray(stds, dtype=np.float64)
    if s.size != xs.size:
        return -1.0
    sel = (xs >= 5.0) & (xs <= 40.0) & np.isfinite(s) & (s > 0.0)
    return float(np.median(s[sel])) if sel.any() else -1.0


def _std_confidence(median_std: float, good_m: float, bad_m: float) -> float:
    if median_std < 0.0 or bad_m <= good_m:
        return 1.0
    if median_std <= good_m:
        return 1.0
    if median_std >= bad_m:
        return 0.0
    return (bad_m - median_std) / (bad_m - good_m)


def lane_lines_to_path(
    plan_x: Optional[np.ndarray],
    plan_y: Optional[np.ndarray],
    x_idxs: np.ndarray,
    lane_ys: Sequence[Optional[np.ndarray]],
    lane_probs: Sequence[float],
    min_lane_prob: float = DEFAULT_MIN_LANE_PROB,
    lane_blend_scale: float = 1.0,
    lane_stds: Optional[Sequence[Optional[np.ndarray]]] = None,
    lane_std_good_m: float = DEFAULT_LANE_STD_GOOD_M,
    lane_std_bad_m: float = DEFAULT_LANE_STD_BAD_M,
    lane_width_min_m: float = DEFAULT_LANE_WIDTH_MIN_M,
    lane_width_max_m: float = DEFAULT_LANE_WIDTH_MAX_M,
) -> Optional[np.ndarray]:
    """Fuse PLAN + near L/R lanes → Nx2 device-frame polyline (Y right+).

    ``lane_ys`` / ``lane_probs`` indexed like proto: 0=leftFar, 1=leftNear,
    2=rightNear, 3=rightFar. Only indices 1 and 2 are used (Android/C++).
    ``lane_blend_scale`` 0 ⇒ prefer model plan only (matches C++ path_lane_blend_scale).

    Mirror of C++ ``laneLinesToPath``; keep in sync. ``lane_stds`` is the same
    ``LanePolyline.y_std`` C++ reads (median over 5–40 m); None means no sigma in frame
    and applies no penalty (bags before 2026-08-02).
    """
    poly: Optional[np.ndarray] = None
    if plan_x is not None and plan_y is not None and len(plan_x) >= 2:
        poly = plan_to_polyline_ego(
            np.asarray(plan_x, dtype=np.float64),
            np.asarray(plan_y, dtype=np.float64),
            y_sign=PP_Y_SIGN,
            x_min=1.0,
        )
        if poly.shape[0] < 2:
            poly = None

    blend_scale = float(np.clip(lane_blend_scale, 0.0, 1.0))
    if blend_scale <= 1e-9 and poly is not None:
        return poly

    xs = np.asarray(x_idxs, dtype=np.float64)
    have_l = (
        len(lane_ys) > 1
        and lane_ys[1] is not None
        and len(lane_ys[1]) == len(xs)
        and len(xs) >= 2
    )
    have_r = (
        len(lane_ys) > 2
        and lane_ys[2] is not None
        and len(lane_ys[2]) == len(xs)
        and len(xs) >= 2
    )
    both = have_l and have_r
    l_prob = (
        _soft_lane_prob(
            float(lane_probs[1]) if len(lane_probs) > 1 else 0.0, min_lane_prob
        )
        if both
        else 0.0
    )
    r_prob = (
        _soft_lane_prob(
            float(lane_probs[2]) if len(lane_probs) > 2 else 0.0, min_lane_prob
        )
        if both
        else 0.0
    )
    if both and lane_stds is not None:
        l_prob *= _std_confidence(
            _median_std(lane_stds[1] if len(lane_stds) > 1 else None, xs),
            lane_std_good_m,
            lane_std_bad_m,
        )
        r_prob *= _std_confidence(
            _median_std(lane_stds[2] if len(lane_stds) > 2 else None, xs),
            lane_std_good_m,
            lane_std_bad_m,
        )
    width_ok = False
    if both:
        sel = (xs >= 5.0) & (xs <= 40.0)
        if sel.any():
            w_med = float(
                np.mean(np.abs(np.asarray(lane_ys[2])[sel] - np.asarray(lane_ys[1])[sel]))
            )
            width_ok = lane_width_min_m < w_med < lane_width_max_m
    anchored = width_ok and l_prob > 0.0 and r_prob > 0.0
    d_prob = (l_prob + r_prob - l_prob * r_prob) * blend_scale if anchored else 0.0

    if d_prob <= 1e-6:
        return poly

    yl = np.asarray(lane_ys[1], dtype=np.float64) if have_l else None
    yr = np.asarray(lane_ys[2], dtype=np.float64) if have_r else None
    lane_xs: List[float] = []
    lane_mid: List[float] = []
    for i in range(len(xs)):
        x = float(xs[i])
        y_l = float(yl[i]) if yl is not None else 0.0
        y_r = float(yr[i]) if yr is not None else 0.0
        w = min(lane_width_max_m, max(lane_width_min_m, abs(y_l - y_r)))
        from_l = y_l + 0.5 * w
        from_r = y_r - 0.5 * w
        lane_y = (l_prob * from_l + r_prob * from_r) / (l_prob + r_prob + 1e-6)
        lane_xs.append(x)
        lane_mid.append(lane_y)

    xs_a = np.asarray(lane_xs, dtype=np.float64)
    ys_a = np.asarray(lane_mid, dtype=np.float64)

    if poly is not None and poly.shape[0] >= 2:
        out = poly.copy()
        for i in range(out.shape[0]):
            y_lane = _interp_y(float(out[i, 0]), xs_a, ys_a)
            out[i, 1] = d_prob * y_lane + (1.0 - d_prob) * out[i, 1]
        return out

    ok = xs_a >= 1.0
    if not np.any(ok):
        return None
    return np.stack([xs_a[ok], ys_a[ok]], axis=1)


def path_from_supercombo(
    out: SupercomboOut,
    min_lane_prob: float = DEFAULT_MIN_LANE_PROB,
    lane_blend_scale: float = 0.0,
) -> Optional[np.ndarray]:
    lane_ys = []
    probs = []
    for i in range(4):
        if i < len(out.lanes):
            lane_ys.append(np.asarray(out.lanes[i].y, dtype=np.float64))
            probs.append(float(out.lanes[i].prob))
        else:
            lane_ys.append(None)
            probs.append(0.0)
    plan_x = out.plan.x if out.plan is not None else None
    plan_y = out.plan.y if out.plan is not None else None
    return lane_lines_to_path(
        plan_x, plan_y, X_IDXS, lane_ys, probs, min_lane_prob, lane_blend_scale
    )


def path_from_bag_lanes(
    ll: Any,
    min_lane_prob: float = DEFAULT_MIN_LANE_PROB,
    lane_blend_scale: float = 0.0,
) -> Optional[np.ndarray]:
    """``vision/lanes`` protobuf (or similar) → device-frame PP polyline."""
    bundle = path_bundle_from_bag_lanes(ll, min_lane_prob, lane_blend_scale)
    return None if bundle is None else bundle["polyline"]


def plan_orientation_from_lanes(
    ll: Any,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """Return (plan_xy Nx2, yaw, yaw_rate) from proto fields or model_out re-parse."""
    plan_x = (
        np.asarray(list(ll.plan_x), dtype=np.float64)
        if getattr(ll, "plan_x", None)
        else None
    )
    plan_y = (
        np.asarray(list(ll.plan_y), dtype=np.float64)
        if getattr(ll, "plan_y", None)
        else None
    )
    yaw = (
        np.asarray(list(ll.plan_yaw), dtype=np.float64)
        if getattr(ll, "plan_yaw", None) and len(ll.plan_yaw)
        else None
    )
    yaw_rate = (
        np.asarray(list(ll.plan_yaw_rate), dtype=np.float64)
        if getattr(ll, "plan_yaw_rate", None) and len(ll.plan_yaw_rate)
        else None
    )
    if yaw is None or yaw_rate is None:
        mo = list(getattr(ll, "model_out", None) or [])
        if len(mo) >= 4955:
            from .supercombo_parse import parse_supercombo

            parsed = parse_supercombo(np.asarray(mo, dtype=np.float64))
            if parsed.plan is not None:
                plan_x = parsed.plan.x
                plan_y = parsed.plan.y
                yaw = parsed.plan.yaw
                yaw_rate = parsed.plan.yaw_rate
    if plan_x is None or plan_y is None or yaw is None or yaw_rate is None:
        return None, None, None
    n = min(len(plan_x), len(plan_y), len(yaw), len(yaw_rate))
    if n < 2:
        return None, None, None
    plan_xy = np.stack([plan_x[:n], plan_y[:n]], axis=1)
    return plan_xy, yaw[:n].astype(float), yaw_rate[:n].astype(float)


# stock lane_planner.CAMERA_OFFSET (flowpilot 0.08). Keep in sync with the C++ default in
# TopicConvertService::Config::path_camera_offset_m / assets/config.json.
DEFAULT_CAMERA_OFFSET_M = 0.08  # measured plan left bias; see docs/RUN_0801_LEFT_DRIFT.md


def path_bundle_from_bag_lanes(
    ll: Any,
    min_lane_prob: float = DEFAULT_MIN_LANE_PROB,
    lane_blend_scale: float = 0.0,
    camera_offset_m: float = DEFAULT_CAMERA_OFFSET_M,
) -> Optional[dict]:
    """Polyline + optional PLAN orientation for FP LatMpc.

    ``camera_offset_m`` mirrors the C++ ``laneLinesToPath``: the model plans ~0.10 m left of the
    midpoint of its own lane lines, so the path is shifted right (device Y right+).
    """
    plan_x = (
        np.asarray(list(ll.plan_x), dtype=np.float64)
        if getattr(ll, "plan_x", None)
        else None
    )
    plan_y = (
        np.asarray(list(ll.plan_y), dtype=np.float64)
        if getattr(ll, "plan_y", None)
        else None
    )
    xs = (
        np.asarray(list(ll.x), dtype=np.float64)
        if getattr(ll, "x", None) and len(ll.x)
        else X_IDXS
    )
    lane_ys: List[Optional[np.ndarray]] = []
    probs: List[float] = []
    stds: List[Optional[np.ndarray]] = []
    lanes = list(getattr(ll, "lanes", []) or [])
    for i in range(4):
        if i < len(lanes) and lanes[i].y:
            lane_ys.append(np.asarray(list(lanes[i].y), dtype=np.float64))
            probs.append(float(getattr(lanes[i], "prob", 0.0) or 0.0))
            y_std = list(getattr(lanes[i], "y_std", []) or [])
            stds.append(np.asarray(y_std, dtype=np.float64) if y_std else None)
        else:
            lane_ys.append(None)
            probs.append(0.0)
            stds.append(None)
    poly = lane_lines_to_path(
        plan_x,
        plan_y,
        xs,
        lane_ys,
        probs,
        min_lane_prob,
        lane_blend_scale,
        lane_stds=stds,
    )
    if poly is None or poly.shape[0] < 2:
        return None
    plan_xy, yaw, yaw_rate = plan_orientation_from_lanes(ll)
    if abs(camera_offset_m) > 1e-9:
        poly = poly.copy()
        poly[:, 1] += camera_offset_m
        if plan_xy is not None and len(plan_xy):
            plan_xy = np.asarray(plan_xy, dtype=np.float64).copy()
            plan_xy[:, 1] += camera_offset_m
    return {
        "polyline": poly,
        "plan_poly": plan_xy,
        "plan_yaw": yaw,
        "plan_yaw_rate": yaw_rate,
    }


def iso_left_polyline_to_device(poly: np.ndarray) -> np.ndarray:
    """MetaDrive / ISO Y-left Nx2 → device Y-right for PP."""
    out = np.asarray(poly, dtype=np.float64).copy()
    if out.ndim == 2 and out.shape[1] >= 2:
        out[:, 1] = -out[:, 1]
    return out
