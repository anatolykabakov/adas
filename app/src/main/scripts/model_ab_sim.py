#!/usr/bin/env python3
"""A/B supercombo models on a bag: lateral (pyadas) + CIPV/long/safety.

Compatible models: single-cam float32, 4 inputs (imgs/desire/traffic/state).
Outputs 6409 (assets/v0.8.5) or 6472 (v0.8.12/13, full F2).

Example:
  python model_ab_sim.py ../../../adas_logs/2026_07_31_10_33_17 \\
      --models ../../../models/sc_v0.8.5.onnx ../../../models/sc_v0.8.13.onnx \\
      --n-frames 40 --controllers pp,fp
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

import _path  # noqa: F401

from core.path_fusion import path_from_supercombo
from core.supercombo_compare import SupercomboBev
from core.supercombo_parse import parse_model_long, parse_supercombo
from core.viz_params_ui import load_camera_priors
from vis.bag_io import load_topic_messages, nearest
from vis.android_bag_player import AndroidBagPlayer  # type: ignore

try:
    from pyadas import core as pyadas
except Exception as e:  # pragma: no cover
    pyadas = None
    _pyadas_err = e

STEER_RATIO = 15.7
WHEELBASE = 2.636


def idm_accel(ego_v, kappa, has_lead, lead_v, gap, speed_limit=27.778):
    """Classic Treiber IDM (matches safety_planner.hpp). Δv = ego − lead."""
    mu, g = 0.5, 9.81
    a_max, b, T, s0, expn = 1.5, 3.0, 1.5, 2.0, 4.0
    abs_k = abs(kappa)
    v_lim = min(speed_limit, (mu * g / abs_k) ** 0.5) if abs_k > 1e-6 else speed_limit
    delta_v = ego_v - lead_v
    dyn = (ego_v * delta_v) / (2.0 * (a_max * b) ** 0.5)
    s_star = s0 + max(0.0, ego_v * T + dyn)
    gap = max(0.5, gap)
    free = (ego_v / max(0.1, v_lim)) ** expn
    inter = (s_star / gap) ** 2 if has_lead else 0.0
    return a_max * (1.0 - free - inter)


def load_camera_jpegs(session: Path, n: int, stride: int) -> List[Tuple[int, np.ndarray]]:
    """Return list of (ts_ms, BGR)."""
    # Prefer lightweight bag_io if images are huge — use AndroidBagPlayer like viz.
    try:
        from vis.android_bag_player import AndroidBagPlayer as P

        player = P(str(session))
        if "sensors/camera/image" not in player.topics:
            return []
        out = []
        msgs = player.get_topic_msgs("sensors/camera/image")
        for i, (ts, msg) in enumerate(msgs):
            if i % stride:
                continue
            data = bytes(msg.image_data) if hasattr(msg, "image_data") else None
            if not data:
                continue
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            out.append((int(ts), img))
            if len(out) >= n:
                break
        return out
    except Exception:
        pass

    # Fallback: raw vision bins via bag_io
    frames = []
    for ts, cam, _ in load_topic_messages(session, "sensors/camera/image"):
        data = bytes(cam.image_data)
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        if len(frames) % stride == 0:
            frames.append((int(ts), img))
        if len(frames) >= n:
            break
    return frames


def model_compatible(path: Path) -> Tuple[bool, str]:
    import onnxruntime as ort

    s = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    ins = s.get_inputs()
    outs = s.get_outputs()
    if len(ins) != 4 or len(outs) != 1:
        return False, f"need 4in/1out got {len(ins)}/{len(outs)}"
    if "float16" in ins[0].type:
        return False, "fp16"
    if ins[0].name != "input_imgs":
        return False, f"in0={ins[0].name}"
    n = outs[0].shape[-1]
    if not isinstance(n, int) or n < 5860:
        return False, f"out={outs[0].shape}"
    return True, f"out={n}"


def run_model_on_frames(
    model: Path,
    frames: List[Tuple[int, np.ndarray]],
    calib: Dict[str, float],
) -> Dict[str, Any]:
    bev = SupercomboBev(model)
    bev.set_calib(
        roll_deg=calib.get("roll", 0.0),
        pitch_deg=calib.get("pitch", -1.8),
        yaw_deg=calib.get("yaw", 0.5),
        fx=calib.get("fx", 951.0),
        fy=calib.get("fy", 951.0),
        cx=calib.get("cx", 640.0),
        cy=calib.get("cy", 360.0),
    )
    leads_p, leads_d, plan_v, paths = [], [], [], []
    raw_flats = []
    for i, (ts, bgr) in enumerate(frames):
        out = bev.infer(bgr, cache_key=i)
        if out is None:
            continue
        # Re-run to get flat: session stored; peek via second infer of cached
        # SupercomboBev caches SupercomboOut only — re-parse from session
        assert bev._session is not None and bev._names is not None
        parsed = bev._preprocess_yuv6(bgr)
        if bev._prev is None:
            bev._prev = parsed
            continue
        stacked = np.concatenate([bev._prev, parsed], axis=0)[None].astype(np.float32)
        bev._prev = parsed
        feeds = {
            bev._names[0]: stacked,
            bev._names[1]: np.zeros((1, 8), np.float32),
            bev._names[2]: np.array([[1.0, 0.0]], np.float32),
            bev._names[3]: bev._rnn,
        }
        flat = bev._session.run([bev._names[4]], feeds)[0].reshape(-1).astype(np.float32)
        if flat.size >= 512:
            bev._rnn = flat[-512:][None].copy()
        raw_flats.append(flat)
        ml = parse_model_long(flat)
        if ml is None:
            continue
        lead = ml.best_lead()
        leads_p.append(lead.prob)
        leads_d.append(lead.d_rel)
        plan_v.append(float(ml.plan_vx[0]))
        sc = parse_supercombo(flat)
        poly = path_from_supercombo(sc, lane_blend_scale=0.0)
        if poly is not None and len(poly) >= 2:
            paths.append((ts, poly, ml, lead))
    return {
        "n": len(raw_flats),
        "lead_p_max": float(np.max(leads_p)) if leads_p else 0.0,
        "lead_p_med": float(np.median(leads_p)) if leads_p else 0.0,
        "lead_d_min": float(np.min(leads_d)) if leads_d else 0.0,
        "lead_d_med": float(np.median(leads_d)) if leads_d else 0.0,
        "plan_v_med": float(np.median(plan_v)) if plan_v else 0.0,
        "paths": paths,
        "out_size": int(raw_flats[0].size) if raw_flats else 0,
        "error": bev.error,
    }


def lateral_stats(
    paths: List[Tuple[int, np.ndarray, Any, Any]],
    state_msgs,
    controller: str,
) -> Dict[str, float]:
    if pyadas is None or not paths:
        return {"n": 0, "swa_p95": float("nan"), "cte_p95": float("nan")}
    app = pyadas.AdasApp(wheelbase=WHEELBASE)
    app.set_lane_keep_controller(controller)
    app.set_lane_keep_max_steer_deg(25.0)
    us = 0
    swa, cte = [], []
    for ts, poly, ml, lead in paths:
        hit = nearest(ts, state_msgs, max_dt_ms=150) if state_msgs else None
        v = float(hit[1].v_ego) if hit else max(float(ml.plan_vx[0]), 0.5)
        yr = float(getattr(hit[1], "yaw_rate", 0.0)) if hit else 0.0
        next_us = int(
            round(float(ts) * 1000.0)
        )  # real bag time (drives the solve period)
        us = next_us if next_us > us else us + 1_000
        app.publish_chassis(us, max(v, 0.1), 0.0, yr)
        app.publish_lanes(us, [(float(x), float(y)) for x, y in poly])
        app.step(us)
        outs = [m for m in app.pop_messages() if isinstance(m, pyadas.LaneKeepOutput)]
        if not outs:
            continue
        o = outs[-1]
        swa.append(-float(np.degrees(o.steer_rad)) * STEER_RATIO)
        cte.append(float(o.cte_m))
    if not swa:
        return {"n": 0, "swa_p95": float("nan"), "cte_p95": float("nan")}
    return {
        "n": len(swa),
        "swa_med": float(np.median(swa)),
        "swa_p95": float(np.percentile(np.abs(swa), 95)),
        "cte_med": float(np.median(np.abs(cte))),
        "cte_p95": float(np.percentile(np.abs(cte), 95)),
    }


def long_safety_stats(
    paths: List[Tuple[int, np.ndarray, Any, Any]],
    state_msgs,
) -> Dict[str, float]:
    n_lead = n_fcw = n_aeb = n_ldw = 0
    for ts, poly, ml, lead in paths:
        hit = nearest(ts, state_msgs, max_dt_ms=150) if state_msgs else None
        v_ego = float(hit[1].v_ego) if hit else max(float(ml.plan_vx[0]), 0.0)
        has = lead.prob >= 0.4 and 1.0 < lead.d_rel < 120.0
        if has:
            n_lead += 1
        a = idm_accel(v_ego, 0.0, has, float(lead.v[0]), max(0.5, lead.d_rel - 1.5))
        if -5.0 <= a <= -3.0:
            n_fcw += 1
        if a < -5.0:
            n_aeb += 1
        # crude CTE from path y at x≈2
        if poly is not None and len(poly) >= 2:
            j = int(np.argmin(np.abs(poly[:, 0] - 2.0)))
            cte = float(-poly[j, 1])
            if abs(cte) > 0.5:
                n_ldw += 1
    n = max(len(paths), 1)
    return {
        "lead_frac": n_lead / n,
        "fcw_frac": n_fcw / n,
        "aeb_frac": n_aeb / n,
        "ldw_frac": n_ldw / n,
        "n": len(paths),
    }


def bag_ref_lateral(session: Path, n: int) -> Dict[str, float]:
    dbg = load_topic_messages(session, "control/lane_keep_debug")
    if not dbg:
        return {}
    swa = [
        float(d.desired_swa_deg)
        for _, d, _ in dbg[: n * 5]
        if hasattr(d, "desired_swa_deg")
    ]
    cte = [abs(float(d.mpc_cte_m)) for _, d, _ in dbg[: n * 5] if hasattr(d, "mpc_cte_m")]
    if not swa:
        return {}
    return {
        "swa_med": float(np.median(swa)),
        "swa_p95": float(np.percentile(np.abs(swa), 95)),
        "cte_med": float(np.median(cte)) if cte else float("nan"),
        "cte_p95": float(np.percentile(cte, 95)) if cte else float("nan"),
        "n": len(swa),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--models", nargs="+", type=Path, required=True)
    ap.add_argument("--n-frames", type=int, default=40)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--controllers", default="pp,fp", help="comma list: pp,mpc,fp")
    ap.add_argument("-o", type=Path, help="CSV summary")
    args = ap.parse_args()

    if pyadas is None:
        print("pyadas missing:", _pyadas_err, file=sys.stderr)

    frames = load_camera_jpegs(args.bag, args.n_frames, args.stride)
    if len(frames) < 4:
        sys.exit(f"need camera frames, got {len(frames)}")
    print(f"camera frames: {len(frames)}")

    apri = load_camera_priors()
    calib = {
        "roll": 0.0,
        "pitch": getattr(apri, "pitch_deg", -1.8),
        "yaw": getattr(apri, "yaw_deg", 0.5),
        "fx": getattr(apri, "fx", 951.0),
        "fy": getattr(apri, "fy", 951.0),
        "cx": getattr(apri, "cx", 640.0),
        "cy": getattr(apri, "cy", 360.0),
    }
    # Prefer bag calib if present
    try:
        cams = load_topic_messages(args.bag, "calibration/camera")
        if cams:
            c = cams[-1][1]
            calib["pitch"] = float(getattr(c, "pitch_deg", calib["pitch"]))
            calib["yaw"] = float(getattr(c, "yaw_deg", calib["yaw"]))
            calib["roll"] = float(getattr(c, "roll_deg", 0.0))
    except Exception:
        pass

    state = load_topic_messages(args.bag, "vehicle/state")
    ref = bag_ref_lateral(args.bag, args.n_frames)
    if ref:
        print(
            f"bag lane_keep_debug ref: swa_med={ref['swa_med']:.1f} "
            f"swa_p95={ref['swa_p95']:.1f} cte_p95={ref.get('cte_p95', float('nan')):.2f}"
        )

    controllers = [c.strip() for c in args.controllers.split(",") if c.strip()]
    rows = []
    for mp in args.models:
        ok, why = model_compatible(mp)
        print(f"\n=== {mp.name} ({why}) ===")
        if not ok:
            print("  SKIP incompatible")
            continue
        stats = run_model_on_frames(mp, frames, calib)
        print(
            f"  infer n={stats['n']} out={stats['out_size']} "
            f"lead_p max/med={stats['lead_p_max']:.3g}/{stats['lead_p_med']:.3g} "
            f"lead_d min/med={stats['lead_d_min']:.1f}/{stats['lead_d_med']:.1f} "
            f"plan_v≈{stats['plan_v_med']:.1f}"
        )
        if stats["error"]:
            print("  error:", stats["error"])
        ls = long_safety_stats(stats["paths"], state)
        print(
            f"  long/safety: lead_frac={ls['lead_frac']:.2f} "
            f"fcw={ls['fcw_frac']:.2f} aeb={ls['aeb_frac']:.2f} ldw={ls['ldw_frac']:.2f}"
        )
        for ctrl in controllers:
            lat = lateral_stats(stats["paths"], state, ctrl)
            d_swa = (
                lat["swa_p95"] - ref["swa_p95"]
                if ref and lat["n"] and np.isfinite(lat["swa_p95"])
                else float("nan")
            )
            print(
                f"  lat[{ctrl}]: n={lat['n']} swa_med={lat.get('swa_med', float('nan')):.1f} "
                f"swa_p95={lat.get('swa_p95', float('nan')):.1f} "
                f"cte_p95={lat.get('cte_p95', float('nan')):.2f} "
                f"Δswa_p95_vs_bag={d_swa:+.1f}"
            )
            rows.append(
                {
                    "model": mp.name,
                    "out_size": stats["out_size"],
                    "lead_p_max": stats["lead_p_max"],
                    "lead_d_min": stats["lead_d_min"],
                    "lead_frac": ls["lead_frac"],
                    "fcw_frac": ls["fcw_frac"],
                    "aeb_frac": ls["aeb_frac"],
                    "ldw_frac": ls["ldw_frac"],
                    "controller": ctrl,
                    "swa_p95": lat.get("swa_p95", float("nan")),
                    "cte_p95": lat.get("cte_p95", float("nan")),
                    "d_swa_p95_vs_bag": d_swa,
                }
            )

    if args.o and rows:
        args.o.parent.mkdir(parents=True, exist_ok=True)
        with args.o.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("\nwrote", args.o)


if __name__ == "__main__":
    main()
