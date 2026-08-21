"""Path fusion matching C++ ``adas::laneLinesToPath`` (device Y-right).

Stock comma ``LanePlanner.get_d_path``. Use the returned Nx2 polyline with
``AdasApp.publish_lanes`` / PP — same frame as Android TopicConvert → LaneKeep.
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
# Range the sigma summary covers. Was a fixed 40 m; measured on run 2026_08_06_00_36_42, worst-line
# sigma roughly doubles from the near half to the far half of that window (right arc 0.34 over
# 5-20 m against 0.78 over 20-40 m) because on a bend the inner line leaves the frame and its far
# samples are extrapolation. See `LanePathConfig::lane_std_range_m` for the full table.
DEFAULT_LANE_STD_RANGE_M = 20.0
DEFAULT_LANE_WIDTH_MIN_M = 2.6
DEFAULT_LANE_WIDTH_MAX_M = 4.6


def _median_std(
    stds: Optional[np.ndarray],
    xs: np.ndarray,
    range_m: float = DEFAULT_LANE_STD_RANGE_M,
) -> float:
    """Median lane sigma from 5 m out to ``range_m``; -1 if sigma missing (no penalty)."""
    if stds is None:
        return -1.0
    s = np.asarray(stds, dtype=np.float64)
    if s.size != xs.size:
        return -1.0
    x_max = range_m if range_m > 5.0 else 40.0
    sel = (xs >= 5.0) & (xs <= x_max) & np.isfinite(s) & (s > 0.0)
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
    lane_std_range_m: float = DEFAULT_LANE_STD_RANGE_M,
    lane_width_min_m: float = DEFAULT_LANE_WIDTH_MIN_M,
    lane_width_max_m: float = DEFAULT_LANE_WIDTH_MAX_M,
    v_ego: float = 0.0,
    state: Optional[dict] = None,
) -> Optional[np.ndarray]:
    """Fuse PLAN + near L/R lanes → Nx2 device-frame polyline (Y right+).

    Port of comma/openpilot ``LanePlanner.get_d_path``. Extra kwargs are kept so
    callers do not break; only ``v_ego`` / ``state`` / the polylines affect the mix.
    ``lane_stds[i][0]`` is comma's scalar ``laneLineStds[i]``; empty/None is no penalty.
    """
    del min_lane_prob, lane_blend_scale, lane_std_good_m, lane_std_bad_m
    del lane_std_range_m, lane_width_min_m, lane_width_max_m

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

    xs = np.asarray(x_idxs, dtype=np.float64)
    if xs.size < 2:
        return poly

    def _line(idx: int) -> np.ndarray:
        if (
            len(lane_ys) > idx
            and lane_ys[idx] is not None
            and len(lane_ys[idx]) == len(xs)
        ):
            return np.asarray(lane_ys[idx], dtype=np.float64)
        return np.zeros_like(xs)

    def _prob(idx: int) -> float:
        if (
            len(lane_ys) > idx
            and lane_ys[idx] is not None
            and len(lane_ys[idx]) == len(xs)
        ):
            return float(lane_probs[idx]) if len(lane_probs) > idx else 0.0
        return 0.0

    def _std0(idx: int) -> float:
        if lane_stds is None or len(lane_stds) <= idx or lane_stds[idx] is None:
            return 0.0
        s = np.asarray(lane_stds[idx], dtype=np.float64)
        if s.size == 0 or not np.isfinite(s[0]) or s[0] <= 0.0:
            return 0.0
        return float(s[0])

    have_any = (
        _prob(1) > 0.0
        or _prob(2) > 0.0
        or (len(lane_ys) > 1 and lane_ys[1] is not None)
        or (len(lane_ys) > 2 and lane_ys[2] is not None)
    )
    if not have_any:
        return poly

    lll_y = _line(1)
    rll_y = _line(2)
    l_prob = _prob(1)
    r_prob = _prob(2)
    width_pts = rll_y - lll_y
    mod = 1.0
    for t_check in (0.0, 1.5, 3.0):
        width_at_t = _interp_y(t_check * (float(v_ego) + 7.0), xs, width_pts)
        mod = min(mod, float(np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0])))
    l_prob *= mod
    r_prob *= mod
    l_prob *= float(np.interp(_std0(1), [0.15, 0.3], [1.0, 0.0]))
    r_prob *= float(np.interp(_std0(2), [0.15, 0.3], [1.0, 0.0]))

    st = state if state is not None else {}
    width_est = float(st.get("width_est_m", 3.7))
    certainty = float(st.get("width_certainty", 1.0))
    k_est = 0.05 / (9.95 + 0.05)
    k_cert = 0.05 / (0.95 + 0.05)
    certainty = (1.0 - k_cert) * certainty + k_cert * (l_prob * r_prob)
    width_est = (1.0 - k_est) * width_est + k_est * abs(float(rll_y[0]) - float(lll_y[0]))
    speed_w = float(np.interp(float(v_ego), [0.0, 31.0], [2.8, 3.5]))
    lane_w = certainty * width_est + (1.0 - certainty) * speed_w
    if state is not None:
        state["width_est_m"] = width_est
        state["width_certainty"] = certainty
        state["lane_width_m"] = lane_w

    clipped = min(4.0, lane_w)
    from_l = lll_y + clipped / 2.0
    from_r = rll_y - clipped / 2.0
    d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * from_l + r_prob * from_r) / (l_prob + r_prob + 0.0001)

    if d_prob <= 1e-6:
        return poly
    if poly is not None and poly.shape[0] >= 2:
        out = poly.copy()
        for i in range(out.shape[0]):
            y_lane = _interp_y(float(out[i, 0]), xs, lane_path_y)
            out[i, 1] = d_prob * y_lane + (1.0 - d_prob) * out[i, 1]
        return out
    ok = xs >= 1.0
    if not np.any(ok):
        return None
    return np.stack([xs[ok], lane_path_y[ok]], axis=1)


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
DEFAULT_CAMERA_OFFSET_M = 0.08  # measured plan left bias


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
