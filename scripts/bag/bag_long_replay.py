#!/usr/bin/env python3
"""Replay the longitudinal plan rules over a recorded run and compare old behaviour with new.

Mirror of ``utils/long_planner.hpp``; keep in sync. The point is to answer "what will the cruise
buttons do next drive" without driving, the way ``bag/bag_safety_warn.py`` does for the warnings.

The actuator matters more than the plan here: ``PandaService::computeCruiseButtons`` tips the set
speed by 1 km/h whenever ``|v_target - v_set|`` exceeds ``cruise_deadband_ms`` (0.7 m/s) and at most
once per ``cruise_tip_cooldown_ms`` (200 ms), so this script also counts the tips each rule set
would have produced.

Usage:
  python bag/bag_long_replay.py adas_logs/2026_08_06_00_36_42
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages

# Defaults mirroring config.json / long_planner.hpp
A_COAST = -0.30
COAST_HORIZON_S = 3.0
LEAD_MIN_SPEED = 2.0
LEAD_PROB = 0.5
T_FOLLOW = 1.5
MIN_GAP = 4.0
KP_GAP = 0.35
A_MAX = 1.2
A_MIN_OLD = -2.5
DEADBAND = 0.7
COOLDOWN_MS = 200


def plan_new(v_ego, prob, d, v_lead):
    """New rules: lead0 only, moving leads only, target derived from accel, coast-limited, no plan_v."""
    if prob >= LEAD_PROB and 1.0 < d < 120.0 and v_lead >= LEAD_MIN_SPEED:
        gap_des = max(MIN_GAP, T_FOLLOW * v_ego)
        a = KP_GAP * (d - gap_des) + 0.5 * (v_lead - v_ego)
        v_t = max(0.0, v_ego + a * COAST_HORIZON_S)
        if a < 0.0:
            v_t = max(v_t, v_lead)
    else:
        a, v_t = 0.0, v_ego
    if a < A_COAST:
        v_t = max(v_t, v_ego + A_COAST * COAST_HORIZON_S)
    return v_t, max(min(a, A_MAX), A_COAST)


def count_tips(t_ms, v_target, v_ego, engaged, one_sided=False):
    """Cruise-button tips the actuator would emit, latching the set speed the way it really does.

    ``one_sided`` models the new rule in ``PandaService``: the speed the driver had at engage is a
    ceiling for the whole engagement, so the assistant can only give speed back and restore it.
    """
    v_set = None
    ceiling = None
    next_ok = 0.0
    up = down = 0
    for t, vt, ve, en in zip(t_ms, v_target, v_ego, engaged):
        if not en:
            v_set = None
            continue
        if v_set is None:
            v_set = ve  # latched at engage, as in PandaService
            ceiling = ve
            continue
        if t < next_ok:
            continue
        err = (min(vt, ceiling) if one_sided else vt) - v_set
        if abs(err) < DEADBAND:
            continue
        step = 1.0 / 3.6
        if err > 0:
            up += 1
            v_set = min(ceiling, v_set + step) if one_sided else v_set + step
        else:
            down += 1
            v_set = max(0.0, v_set - step)
        next_ok = t + COOLDOWN_MS
    return up, down


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    args = ap.parse_args()

    lp = load_topic_messages(args.bag, "control/long_plan")
    st = load_topic_messages(args.bag, "vehicle/state")
    if not lp:
        raise SystemExit("no control/long_plan in bag")

    t = np.asarray([r[0] for r in lp], float)
    v_ego = np.asarray([r[1].v_ego for r in lp], float)
    v_old = np.asarray([r[1].v_target for r in lp], float)
    a_old = np.asarray([r[1].a_target for r in lp], float)
    prob = np.asarray([r[1].lead_prob for r in lp], float)
    d = np.asarray([r[1].lead_d for r in lp], float)
    v_lead = np.asarray([r[1].lead_v for r in lp], float)

    st_t = np.asarray([r[0] for r in st], float)
    st_en = np.asarray(
        [
            1.0
            if getattr(getattr(r[1], "car_state", None) or r[1], "cruise_engaged", False)
            else 0.0
            for r in st
        ],
        float,
    )
    idx = np.clip(np.searchsorted(st_t, t), 0, len(st_t) - 1)
    engaged = st_en[idx] > 0.5

    v_new = np.empty_like(v_old)
    a_new = np.empty_like(a_old)
    for i in range(len(t)):
        v_new[i], a_new[i] = plan_new(v_ego[i], prob[i], d[i], v_lead[i])

    print(
        f"{args.bag.name}: {len(t)} ticks, cruise engaged {100 * engaged.mean():.0f} % of them"
    )
    print(f"\n{'':28} {'old (as driven)':>17} {'new rules':>12}")
    rows = [
        (
            "v_target - v_ego, median",
            f"{np.median(v_old - v_ego):+.2f} m/s",
            f"{np.median(v_new - v_ego):+.2f} m/s",
        ),
        (
            "v_target - v_ego, p5",
            f"{np.percentile(v_old - v_ego, 5):+.2f} m/s",
            f"{np.percentile(v_new - v_ego, 5):+.2f} m/s",
        ),
        (
            "a_target, median",
            f"{np.median(a_old):+.2f} m/s2",
            f"{np.median(a_new):+.2f} m/s2",
        ),
        ("a_target, min", f"{a_old.min():+.2f} m/s2", f"{a_new.min():+.2f} m/s2"),
        (
            "asks decel > 0.5 m/s2",
            f"{100 * (a_old < -0.5).mean():.1f} %",
            f"{100 * (a_new < -0.5).mean():.1f} %",
        ),
        (
            "target below deadband",
            f"{100 * ((v_old - v_ego) < -DEADBAND).mean():.1f} %",
            f"{100 * ((v_new - v_ego) < -DEADBAND).mean():.1f} %",
        ),
    ]
    for name, a, b in rows:
        print(f"{name:28} {a:>17} {b:>12}")

    up_o, dn_o = count_tips(t, v_old, v_ego, engaged)
    up_n, dn_n = count_tips(t, v_new, v_ego, engaged, one_sided=True)
    dur_min = (t[-1] - t[0]) / 60000.0
    print(f"\ncruise tips the actuator would send over {dur_min:.1f} min:")
    print(f"{'':28} {'old':>17} {'new':>12}")
    print(f"{'down-tips':28} {dn_o:>17} {dn_n:>12}")
    print(f"{'up-tips':28} {up_o:>17} {up_n:>12}")
    print(
        f"{'tips per minute':28} {(up_o + dn_o) / dur_min:>17.1f} {(up_n + dn_n) / dur_min:>12.1f}"
    )
    print(
        "\nFor reference, the bus showed 715 rising edges of cruise_decel and 690 of cruise_accel\n"
        "on run 2026_08_06_00_36_42 — driver and assistant presses are indistinguishable there,\n"
        "but the count is far past anything a driver does by hand."
    )


if __name__ == "__main__":
    main()
