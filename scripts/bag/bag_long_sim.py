#!/usr/bin/env python3
"""Offline CIPV / long-plan / safety check from bag ``vision/lanes.model_out``.

Re-parses F2 lead block (LEAD_IDX=5755) + PLAN velocity, runs the same IDM /
LDW thresholds as ``SafetyWarnService`` / ``LongPlanService`` stubs.

Usage:
  python bag/bag_long_sim.py adas_logs/2026_07_31_10_33_17
  python bag/bag_long_sim.py adas_logs/2026_07_31_10_33_17 -o /tmp/long.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from core.supercombo_parse import parse_model_long
from core.path_fusion import path_from_bag_lanes
from vis.bag_io import load_topic_messages, nearest


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("-o", type=Path, help="CSV out")
    ap.add_argument("--stride", type=int, default=5)
    args = ap.parse_args()

    lanes = load_topic_messages(args.bag, "vision/lanes")
    state = load_topic_messages(args.bag, "vehicle/state")
    if not lanes:
        sys.exit("no vision/lanes")

    rows = []
    n_valid = 0
    for i, (ts, ll, _) in enumerate(lanes):
        if i % args.stride:
            continue
        mo = list(getattr(ll, "model_out", None) or [])
        if len(mo) < 5860:
            continue
        ml = parse_model_long(np.asarray(mo, dtype=np.float64))
        if ml is None:
            continue
        lead = ml.best_lead()
        # CTE from path if available
        poly = path_from_bag_lanes(ll)
        cte = 0.0
        kappa = 0.0
        lat_ok = False
        if poly is not None and len(poly) >= 3:
            # crude: y at nearest x~0..5
            j = int(np.argmin(np.abs(poly[:, 0] - 2.0)))
            cte = float(-poly[j, 1])  # device y-right → VP left+ approx flip
            lat_ok = True

        # nearest chassis
        v_ego = float(ml.plan_vx[0])
        if state:
            hit = nearest(ts, state, max_dt_ms=150)
            if hit is not None:
                v_ego = float(hit[1].v_ego)

        has_lead = lead.prob >= 0.4 and 1.0 < lead.d_rel < 120.0
        if has_lead:
            n_valid += 1
        a_cmd = idm_accel(
            v_ego,
            kappa,
            has_lead,
            float(lead.v[0]),
            max(0.5, lead.d_rel - 1.5),
        )
        fcw = -5.0 <= a_cmd <= -3.0
        aeb = a_cmd < -5.0
        lldw = lat_ok and cte < -0.5
        rldw = lat_ok and cte > 0.5

        rows.append(
            {
                "ts_ms": ts,
                "plan_v0": float(ml.plan_vx[0]),
                "pose_vx": float(ml.pose_vx),
                "lead_d": float(lead.d_rel),
                "lead_y": float(lead.y[0]),
                "lead_v": float(lead.v[0]),
                "lead_prob": float(lead.prob),
                "lead_valid": int(has_lead),
                "v_ego": v_ego,
                "a_idm": a_cmd,
                "fcw": int(fcw),
                "aeb": int(aeb),
                "lldw": int(lldw),
                "rldw": int(rldw),
                "cte": cte,
            }
        )

    if not rows:
        sys.exit("no frames with model_out")

    d = np.array([r["lead_d"] for r in rows])
    p = np.array([r["lead_prob"] for r in rows])
    print(f"frames={len(rows)} lead_valid={n_valid}")
    print(f"lead_d min/med/max={d.min():.1f}/{np.median(d):.1f}/{d.max():.1f}")
    print(
        f"lead_prob max={p.max():.3g}  (F2 LEAD_IDX=5755; if max≪0.4 presence head idle)"
    )
    print(f"plan_v0 med={np.median([r['plan_v0'] for r in rows]):.2f}")
    print(f"fcw/aeb frames={sum(r['fcw'] for r in rows)}/{sum(r['aeb'] for r in rows)}")

    if args.o:
        args.o.parent.mkdir(parents=True, exist_ok=True)
        with args.o.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("wrote", args.o)


if __name__ == "__main__":
    main()
