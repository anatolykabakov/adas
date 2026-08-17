#!/usr/bin/env python3
"""Measure the MetaDrive ego's understeer, the same way it was measured on the Golf.

The controller turns curvature into a road-wheel angle through a *measured* understeer model
(``utils/vehicle_model.h``): κ = tan(δ)/L/(1 − sf·v²). On the Golf that model says the car
delivers 0.61 of the kinematic curvature at 22 m/s. The MetaDrive ego is a different car, so
the same command produces a different arc, and the resulting offset would be read as a
controller defect.

Measurement uses steady state on the track's own arcs: once the car settles on an arc of known
radius at constant speed, the achieved curvature *is* 1/R, and the commanded wheel angle is
known — no need to steer the car off the road to excite it.

  python3 -m sim.vehicle_calib
  python3 -m sim.vehicle_calib --speeds 12,18,25
"""

from __future__ import annotations

import argparse
import math
from typing import Dict, List, Tuple

import numpy as np

import _path  # noqa: F401

# MetaDrive DefaultVehicle: FRONT_WHEELBASE + REAR_WHEELBASE.
SIM_WHEELBASE_M = 1.05234 + 1.4166

CIVIC_MASS = 1326.0 + 136.0
CIVIC_WB = 2.70
CIVIC_C2F = CIVIC_WB * 0.4
CIVIC_C2R = CIVIC_WB - CIVIC_C2F
CIVIC_CF = 192150.0
CIVIC_CR = 202500.0


def slip_factor(
    tire_stiffness_factor: float,
    wheelbase: float = 2.636,
    mass: float = 1533.0,
    c2f_frac: float = 0.45,
) -> float:
    """Python mirror of ``adas::slipFactor`` (openpilot scale_tire_stiffness + calc_slip_factor)."""
    a_f = wheelbase * c2f_frac
    a_r = wheelbase - a_f
    c_f = (
        CIVIC_CF
        * tire_stiffness_factor
        * mass
        / CIVIC_MASS
        * (a_r / wheelbase)
        / (CIVIC_C2R / CIVIC_WB)
    )
    c_r = (
        CIVIC_CR
        * tire_stiffness_factor
        * mass
        / CIVIC_MASS
        * (a_f / wheelbase)
        / (CIVIC_C2F / CIVIC_WB)
    )
    if c_f <= 0 or c_r <= 0:
        return 0.0
    return mass * (c_f * a_f - c_r * a_r) / (wheelbase * wheelbase * c_f * c_r)


def tire_factor_for_slip(target_sf: float) -> float:
    """Invert slip_factor numerically (it is negative and rises toward 0 with stiffness)."""
    lo, hi = 0.05, 40.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if slip_factor(mid) < target_sf:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def steady_arc_samples(
    result, min_hold_s: float = 3.0, max_cte_std_m: float = 0.03
) -> List[Dict]:
    """Contiguous arc stretches where the car has settled: constant offset, constant speed."""
    out: List[Dict] = []
    run: List = []
    last_radius = None

    def flush(rows: List) -> None:
        if len(rows) < 4 or not last_radius:
            return
        dt = rows[1].t_s - rows[0].t_s
        keep = rows[len(rows) // 2 :]  # drop the transient half of the arc
        if len(keep) * dt < min_hold_s:
            return
        cte = np.array([r.cte_m for r in keep])
        if float(np.std(cte)) > max_cte_std_m:
            return
        steer = np.array([r.steer_deg for r in keep])
        v = float(np.mean([r.speed_mps for r in keep]))
        delta = float(np.mean(np.abs(steer)))
        if delta < 0.05:
            return
        kappa_actual = 1.0 / last_radius
        kappa_kin = math.tan(math.radians(delta)) / SIM_WHEELBASE_M
        out.append(
            {
                "v": v,
                "radius_m": last_radius,
                "steer_deg": delta,
                "kappa_actual": kappa_actual,
                "kappa_kin": kappa_kin,
                "ratio": kappa_actual / kappa_kin if kappa_kin else 0.0,
                "seconds": len(keep) * dt,
            }
        )

    for s in result.samples:
        if s.radius_m > 0 and (
            last_radius is None or abs(s.radius_m - last_radius) < 1e-6
        ):
            last_radius = s.radius_m
            run.append(s)
            continue
        flush(run)
        run = [s] if s.radius_m > 0 else []
        last_radius = s.radius_m if s.radius_m > 0 else None
    flush(run)
    return out


def fit_slip_factor(rows: List[Dict]) -> Tuple[float, float]:
    """Least squares on 1/ratio = 1 − sf·v²  →  (sf, rms)."""
    usable = [r for r in rows if r["ratio"] > 0]
    if not usable:
        return 0.0, 0.0
    v2 = np.array([r["v"] ** 2 for r in usable])
    inv = np.array([1.0 / r["ratio"] for r in usable])
    sf = float(-np.sum(v2 * (inv - 1.0)) / np.sum(v2 * v2))
    rms = float(np.sqrt(np.mean((inv - (1.0 - sf * v2)) ** 2)))
    return sf, rms


def main() -> int:
    from sim.eval import run_once

    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--track", default="curvy", help="track with arcs of various radii")
    p.add_argument("--speeds", default="12,18,24")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--max-steps", type=int, default=1400)
    args = p.parse_args()

    rows: List[Dict] = []
    for v in [float(x) for x in args.speeds.split(",")]:
        result = run_once("fp", args.track, args.seed, v, args.max_steps, warmup_s=8.0)
        rows += steady_arc_samples(result)

    if not rows:
        print(
            "no steady-state segments found — increase --max-steps or pick another track"
        )
        return 1

    print(
        f"{'v, m/s':>8}{'R, m':>8}{'δ, °':>8}{'κ act':>11}{'κ kin':>11}{'κ/κ_kin':>10}{'sec':>7}"
    )
    for r in sorted(rows, key=lambda x: (x["v"], x["radius_m"])):
        print(
            f"{r['v']:>8.1f}{r['radius_m']:>8.0f}{r['steer_deg']:>8.2f}{r['kappa_actual']:>11.5f}"
            f"{r['kappa_kin']:>11.5f}{r['ratio']:>10.2f}{r['seconds']:>7.1f}"
        )

    sf, rms = fit_slip_factor(rows)
    tsf = tire_factor_for_slip(sf)
    ratio22 = 1.0 / (1.0 - sf * 22.0 ** 2)
    print()
    print(f"simulator vehicle slip factor: {sf:+.6f} (rms {rms:.3f})")
    print(f"equivalent tire_stiffness_factor: {tsf:.2f}    (Golf 0.64)")
    print(f"κ/κ_kin at 22 m/s: sim {ratio22:.2f} … Golf 0.61")
    print()
    if ratio22 > 0.9:
        print("simulator vehicle is nearly kinematic — run test with --kinematic,")
        print(
            "otherwise Golf understeer compensation adds extra turn and constant offset in arcs"
        )
    else:
        print(
            f"test tuned for simulator vehicle:  python3 -m sim.eval --tire-stiffness {tsf:.2f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
