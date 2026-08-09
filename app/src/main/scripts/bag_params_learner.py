#!/usr/bin/env python3
"""What does the vehicle-parameter learner say about *this* car, on a drive we already have?

`utils/params_learner.h` ships disabled behind two flags, and the whole point of shipping it disabled is
that the flags get turned on by evidence rather than by optimism. This script is that evidence: it replays
the same filter over a recorded bag, so the question "what would it have learned" is answered before the
learner ever influences a command.

## The question it is here to settle

`tire_stiffness_factor` is one number in `config.json` and the thing it represents is not one number.
Measured on this car by comparing commanded curvature against the yaw rate that resulted:

    6-9 m/s    kappa_fact / kappa_kin = 0.97
    12-15 m/s                           0.80
    21-26 m/s                           0.54

A value picked for the city under-commands the highway. Worse, two independent estimates of it point in
opposite directions — our 0.54 against comma's learned 1.319 on the same car — which is exactly the sort of
disagreement a filter with a stated uncertainty is supposed to resolve, and a hand-tuned constant cannot.

## What it estimates

Three states, same as the C++ and in the same units, so the numbers can be compared directly:

    x = [tire_stiffness_factor, steer_ratio, angle_offset_deg]

against one measurement per CAN sample, the yaw rate, predicted through *the controller's own* curvature
model:

    delta = (SWA - offset) / steer_ratio
    kappa = curvatureFromSteer(delta, v, L, slipFactor(tsf))
    yaw_pred (z down) = v * kappa + g * sin(roll) / v

## Why roll is not optional

The second term is the road. A banked road supplies part of the lateral acceleration for free, so at a
given steering angle the car turns more than the flat-road model predicts — which is indistinguishable from
a stiffer car. At 20 m/s one degree of bank is 0.0086 rad/s, a few percent of a normal cornering rate and a
systematic error, so it lands in the learned stiffness rather than averaging out. The bank comes from the
same construction as `bag_road_roll.py`: mount frame from gravity and braking, then

    sin(phi) = (-v * yaw_rate_can - f_y) / g

and mind that minus. `chassis.yaw_rate` is decoded as flowpilot decodes `ESP_02`, i.e. ISO — z up, positive
for a **left** turn — while this frame is z down. The wrong sign does not look like a bug, it looks like a
result: a body-roll gradient of 116 deg/g instead of the 3-7 a car can have.

So the script runs the filter twice, told and not told about the bank, and prints both. On synthetic corners
spanning 12-25 m/s a 1.5 deg bank costs only 0.02-0.04 of stiffness when ignored, and the reason is worth
more than the number: bank enters as g*sin(phi)/v and stiffness through 1/(1+K*v^2), so a data set with
speed variety separates them. On one stretch at one speed it cannot.

  python3 bag_params_learner.py adas_logs/2026_08_06_00_36_42
  python3 bag_params_learner.py adas_logs/* --no-roll
  python3 bag_params_learner.py adas_logs/2026_08_06_00_36_42 --speed-band 12 26
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages

G = 9.81

# The vehicle constants, from assets/config.json. Duplicated here rather than parsed because a replay that
# silently follows a changed config cannot be compared against yesterday's replay.
WHEELBASE_M = 2.636
CENTER_TO_FRONT_FRAC = 0.45
MASS_KG = 1533.0
STIFFNESS_INIT = 0.64
STEER_RATIO_INIT = 15.7
# Positive CAN steering angle is a LEFT turn on this car, and this frame is z-down (right-positive), so the
# angle is negated on the way into the model. Measured, not assumed: regressing the ISO yaw rate against the
# kinematic prediction from SWA/ratio gives a slope of +0.824 at a correlation of 0.987 over 28636 cornering
# samples of run 2026_08_06_00_36_42. Leaving this out is what made the first replay of this script pin both
# parameters to their bounds — the prediction opposed the measurement on every corner.
STEER_SIGN = -1.0


# ---------------------------------------------------------------------------------------------------
# The vehicle model, transcribed from utils/vehicle_model.h. Same Civic reference constants, same
# arithmetic: a replay that uses its own idea of the model measures the transcription, not the car.
# ---------------------------------------------------------------------------------------------------


def slip_factor(tire_stiffness_factor: float) -> float:
    civic_mass = 1326.0 + 136.0
    civic_wb = 2.70
    civic_c2f = civic_wb * 0.4
    civic_c2r = civic_wb - civic_c2f
    civic_cf = 192150.0
    civic_cr = 202500.0

    wb = WHEELBASE_M
    a_f = wb * CENTER_TO_FRONT_FRAC
    a_r = wb - a_f
    tsf = max(tire_stiffness_factor, 1e-3)
    c_f = civic_cf * tsf * MASS_KG / civic_mass * (a_r / wb) / (civic_c2r / civic_wb)
    c_r = civic_cr * tsf * MASS_KG / civic_mass * (a_f / wb) / (civic_c2f / civic_wb)
    # Negative for an understeering car, which is why the model's denominator grows with speed.
    return (
        MASS_KG
        * (CENTER_TO_FRONT_FRAC * c_f - (1.0 - CENTER_TO_FRONT_FRAC) * c_r)
        / (wb * c_f * c_r)
    )


def curvature_from_steer(delta_rad: float, v: float, sf: float) -> float:
    return np.tan(delta_rad) / (WHEELBASE_M * (1.0 - sf * v * v))


def predict_yaw(x: np.ndarray, v: float, swa_deg: float, roll_deg: float) -> float:
    """Yaw rate in the z-down frame for the state x. Exactly ParamsLearner::predictWith."""
    tsf, ratio, offset = x
    v = max(v, 1e-3)
    delta = STEER_SIGN * np.radians(swa_deg - offset) / max(ratio, 1e-3)
    kappa = curvature_from_steer(delta, v, slip_factor(tsf))
    return v * kappa + G * np.sin(np.radians(roll_deg)) / v


# ---------------------------------------------------------------------------------------------------
# The filter. Diagonal covariance and numeric Jacobians, as in the header — three states at CAN rate cost
# nothing, and the analytic derivative through slipFactor is the kind of expression that is wrong for a
# month before anyone notices.
# ---------------------------------------------------------------------------------------------------


class ParamsLearner:
    def __init__(self, use_roll: bool = True):
        self.use_roll = use_roll
        self.x = np.array([STIFFNESS_INIT, STEER_RATIO_INIT, 0.0])
        # The tight prior on the steer ratio is the design, not caution. Stiffness and ratio are close to
        # degenerate against a yaw-rate measurement — both scale the predicted curvature — and with a 0.5
        # prior on the ratio this replay wandered to 14.06/0.374 and predicted the yaw rate 3 % worse than
        # the shipped constants while reporting itself converged. A rack's ratio is a mechanical fact known
        # to a few percent; that knowledge belongs in the prior.
        self.p = np.array([0.5 ** 2, 0.1 ** 2, 1.0 ** 2])
        self.q = np.array([0.005 ** 2, 0.00005 ** 2, 0.02 ** 2])
        self.r = 0.02 ** 2
        # Ceilings, not pseudo-measurements. Upstream writes this as "observe the state with its own current
        # value at high noise"; copied literally that is a bug, because the innovation is identically zero,
        # so at CAN rate it shrinks the covariance a hundred times a second without any information arriving
        # and the real measurement loses all gain. Measured on the synthetic test: a true 0.45 came out 0.515,
        # pinned near the 0.64 it started from.
        self.p_max = np.array([0.5 ** 2, 0.3 ** 2, np.inf])
        self.lo = np.array([0.2, 12.0, -10.0])
        self.hi = np.array([5.0, 20.0, 10.0])
        self.n = 0
        self.prev_ay = None

    def update(self, v, swa_deg, yaw_rate_can, roll_deg, roll_std_deg, dt) -> bool:
        if (
            not (0.0 < dt <= 1.0)
            or v < 5.0
            or abs(swa_deg) > 45.0
            or abs(yaw_rate_can) > 1.0
        ):
            return False
        if not np.isfinite([v, swa_deg, yaw_rate_can]).all():
            return False

        yaw = -yaw_rate_can  # ISO in, z-down inside
        a_y = v * yaw
        if self.prev_ay is not None and abs(a_y - self.prev_ay) / dt > 1.0:
            self.prev_ay = a_y  # the model is steady-state; a transient is not evidence about the car
            return False
        self.prev_ay = a_y

        roll = (
            roll_deg
            if (self.use_roll and np.isfinite(roll_deg) and roll_std_deg <= 3.0)
            else 0.0
        )

        self.p = self.p + self.q * dt

        h = np.zeros(3)
        eps = np.array([1e-4, 1e-3, 1e-3])
        for i in range(3):
            xp, xm = self.x.copy(), self.x.copy()
            xp[i] += eps[i]
            xm[i] -= eps[i]
            h[i] = (
                predict_yaw(xp, v, swa_deg, roll) - predict_yaw(xm, v, swa_deg, roll)
            ) / (2 * eps[i])

        s = float(h @ (self.p * h)) + self.r
        k = (self.p * h) / max(s, 1e-12)
        self.x = self.x + k * (yaw - predict_yaw(self.x, v, swa_deg, roll))
        self.p = self.p * (1.0 - k * h)

        self.p = np.minimum(self.p, self.p_max)
        self.x = np.clip(self.x, self.lo, self.hi)
        self.n += 1
        return True

    @property
    def std(self):
        return np.sqrt(np.maximum(self.p, 0.0))

    def valid(self) -> bool:
        """Mirrors ParamsLearner::valid, bounds included.

        The bounds are not decoration. This method first checked only the count and the sigma, and on run
        2026_08_06_00_36_42 it reported `valid=True` for a stiffness sitting exactly on its 0.200 floor and a
        steer ratio on its 20.0 ceiling — the estimate was saturated, which means it had stopped moving,
        which means a small sigma. Saturation is the failure mode that looks most like convergence.
        """
        return (
            self.n >= 500
            and self.std[0] < 0.15
            and bool(np.all(self.x > self.lo + 1e-9))
            and bool(np.all(self.x < self.hi - 1e-9))
        )


# ---------------------------------------------------------------------------------------------------
# Bag loading and the road-bank input.
# ---------------------------------------------------------------------------------------------------


def at(t_ref, t_src, v_src, max_dt_ms=120.0):
    idx = np.clip(np.searchsorted(t_src, t_ref), 0, len(t_src) - 1)
    prev = np.clip(idx - 1, 0, len(t_src) - 1)
    take_prev = np.abs(t_src[prev] - t_ref) < np.abs(t_src[idx] - t_ref)
    idx = np.where(take_prev, prev, idx)
    return v_src[idx], np.abs(t_src[idx] - t_ref) <= max_dt_ms


def load(bag: Path):
    state = load_topic_messages(bag, "vehicle/state")
    if not state:
        return None
    imu = load_topic_messages(bag, "sensors/imu")

    t = np.asarray([r[0] for r in state], dtype=np.float64)

    def field(name):
        return np.asarray(
            [
                float(getattr(getattr(r[1], "car_state", None) or r[1], name, 0.0))
                for r in state
            ],
            dtype=np.float64,
        )

    keep = np.concatenate(([True], np.diff(t) > 0))
    out = {
        "t": t[keep],
        "v": field("v_ego")[keep],
        "swa": field("steering_angle_deg")[keep],
        "yaw_rate": field("yaw_rate")[keep],
    }
    if imu:
        out["t_imu"] = np.asarray([r[0] for r in imu], dtype=np.float64)
        out["accel"] = np.asarray(
            [[r[1].accel_x, r[1].accel_y, r[1].accel_z] for r in imu], dtype=np.float64
        )
    return out


def mount_frame(accel, v_at_imu, a_at_imu):
    """Phone axes -> vehicle axes (x fwd, y right, z down). Same construction as bag_road_roll.py."""
    still = v_at_imu < 0.2
    if still.sum() < 50:
        return None
    g_phone = accel[still].mean(axis=0)
    if not (8.0 < np.linalg.norm(g_phone) < 11.5):
        return None
    down = -g_phone / np.linalg.norm(g_phone)
    horiz = accel - np.outer(accel @ down, down)

    m = np.isfinite(a_at_imu) & (np.abs(a_at_imu) > 0.4) & (v_at_imu > 2.0)
    if m.sum() < 100:
        return None
    seed = np.array([1.0, 0.0, 0.0])
    if abs(seed @ down) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    e1 = seed - (seed @ down) * down
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(down, e1)
    c1 = float(np.dot(horiz[m] @ e1, a_at_imu[m]))
    c2 = float(np.dot(horiz[m] @ e2, a_at_imu[m]))
    if abs(c1) + abs(c2) < 1e-9:
        return None
    fwd = c1 * e1 + c2 * e2
    fwd /= np.linalg.norm(fwd)
    right = np.cross(down, fwd)
    return np.vstack([fwd, right, down])


def road_roll_series(d):
    """Bank in degrees, on the chassis timeline, plus its running sigma. None if the IMU cannot supply it."""
    if "accel" not in d:
        return None
    t_st, v_st = d["t"], d["v"]
    dv = np.gradient(v_st) / np.maximum(np.gradient(t_st) * 1e-3, 1e-3)
    k = max(3, int(round(0.5 / max(np.median(np.diff(t_st)) * 1e-3, 1e-3))))
    a_wheel = np.convolve(dv, np.ones(k) / k, mode="same")

    v_i, ok_v = at(d["t_imu"], t_st, v_st)
    a_i, ok_a = at(d["t_imu"], t_st, a_wheel)
    r_i, ok_r = at(d["t_imu"], t_st, d["yaw_rate"])
    R = mount_frame(d["accel"], np.where(ok_v, v_i, 1e9), np.where(ok_a, a_i, np.nan))
    if R is None:
        return None

    f_y = (d["accel"] @ R.T)[:, 1]
    a_y_kin = -v_i * r_i  # CAN yaw rate is ISO; this frame is z down
    sin_phi = (a_y_kin - f_y) / G
    raw = np.degrees(np.arcsin(np.clip(sin_phi, -1.0, 1.0)))

    # Subtract body roll on the suspension, measured at 2.8-3.2 deg/g on this car, then low-pass with the
    # 10 s time constant the C++ estimator uses. Road camber changes over hundreds of metres; the phone's
    # per-sample noise is 2.2 deg, so the averaging is what makes the input usable at all.
    raw = raw - 3.0 * (a_y_kin / G)
    ok = ok_v & ok_r & np.isfinite(raw) & (v_i >= 8.0)

    filt = np.full(len(raw), np.nan)
    acc, n_acc = 0.0, 0
    dt_imu = max(float(np.median(np.diff(d["t_imu"]))) * 1e-3, 1e-3)
    alpha = dt_imu / (10.0 + dt_imu)
    for i in range(len(raw)):
        if ok[i]:
            acc = raw[i] if n_acc == 0 else acc + alpha * (raw[i] - acc)
            n_acc += 1
        if n_acc > 0:
            filt[i] = acc
    # Sigma: 2.2 deg per sample falling as 1/sqrt(n) but floored at 0.65, where averaging stops helping
    # because the residual is the road's own camber changing rather than noise.
    n_eff = np.maximum(np.minimum(np.arange(len(raw)) + 1, 10.0 / dt_imu), 1.0)
    sigma = np.maximum(2.2 / np.sqrt(n_eff), 0.65)
    sigma = np.where(np.isfinite(filt), sigma, 99.0)

    roll_at_st, ok_st = at(d["t"], d["t_imu"], np.nan_to_num(filt, nan=0.0))
    sig_at_st, _ = at(d["t"], d["t_imu"], sigma)
    return np.where(ok_st, roll_at_st, 0.0), np.where(ok_st, sig_at_st, 99.0)


def run(d, roll, sigma, use_roll, band):
    est = ParamsLearner(use_roll=use_roll)
    t, v, swa, yr = d["t"], d["v"], d["swa"], d["yaw_rate"]
    hist = []
    for i in range(1, len(t)):
        dt = (t[i] - t[i - 1]) * 1e-3
        if band and not (band[0] <= v[i] <= band[1]):
            continue
        if est.update(v[i], swa[i], yr[i], roll[i], sigma[i], dt):
            hist.append((t[i], est.x.copy(), est.std.copy()))
    return est, hist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument(
        "--no-roll", action="store_true", help="run only the flat-road variant"
    )
    ap.add_argument(
        "--speed-band",
        nargs=2,
        type=float,
        default=None,
        metavar=("LO", "HI"),
        help="restrict to one speed band — the way to see the degeneracy the bank hides in",
    )
    args = ap.parse_args()

    for bag in args.bags:
        print(f"\n=== {bag.name}")
        d = load(bag)
        if d is None:
            print("  нет vehicle/state")
            continue

        rr = road_roll_series(d)
        if rr is None:
            print(
                "  крен дороги недоступен (нет sensors/imu или не вышла оценка крепления) — только плоская модель"
            )
            roll = np.zeros(len(d["t"]))
            sigma = np.full(len(d["t"]), 99.0)
            variants = [("плоская", False)]
        else:
            roll, sigma = rr
            usable = sigma <= 3.0
            print(
                f"  крен дороги: медиана {np.nanmedian(roll[usable]):+.2f}°, "
                f"пригоден на {100.0 * usable.mean():.0f}% отсчётов"
            )
            variants = (
                [("плоская", False)]
                if args.no_roll
                else [("с креном", True), ("плоская", False)]
            )

        band = tuple(args.speed_band) if args.speed_band else None
        if band:
            print(f"  полоса скоростей: {band[0]:.0f}–{band[1]:.0f} м/с")

        for name, use_roll in variants:
            est, hist = run(d, roll, sigma, use_roll, band)
            if est.n < 500:
                print(
                    f"  {name:9s}: принято {est.n} отсчётов — мало для выводов "
                    f"(нужны дуги: v ≥ 5, |SWA| ≤ 45, установившийся режим)"
                )
                continue
            print(
                f"  {name:9s}: жёсткость {est.x[0]:.3f} ± {est.std[0]:.3f} "
                f"(в конфиге {STIFFNESS_INIT:.2f}), передаточное {est.x[1]:.2f} ± {est.std[1]:.2f} "
                f"(в конфиге {STEER_RATIO_INIT:.1f}), смещение руля {est.x[2]:+.2f}° ± {est.std[2]:.2f}, "
                f"отсчётов {est.n}, valid={est.valid()}"
            )

            # Convergence, not just the endpoint: a filter that is still moving has not learned anything, it
            # is following the last corner. Quarters of the accepted stream.
            xs = np.asarray([h[1][0] for h in hist])
            q = [
                xs[int(f * len(xs)) : int((f + 0.25) * len(xs))]
                for f in (0.0, 0.25, 0.5, 0.75)
            ]
            print(
                "             жёсткость по четвертям: "
                + "  ".join(f"{np.median(qq):.3f}" for qq in q if len(qq))
            )

            # What the estimate buys, measured on the thing that matters: how well the yaw rate is predicted.
            v, swa, yr = d["v"], d["swa"], d["yaw_rate"]
            m = (v >= 5.0) & (np.abs(swa) <= 45.0) & (np.abs(yr) > 0.02)
            if band:
                m &= (v >= band[0]) & (v <= band[1])
            if m.sum() > 200:
                x0 = np.array([STIFFNESS_INIT, STEER_RATIO_INIT, 0.0])
                rl = roll if use_roll else np.zeros(len(roll))
                res_cfg = [
                    (-yr[i]) - predict_yaw(x0, v[i], swa[i], rl[i])
                    for i in np.flatnonzero(m)
                ]
                res_est = [
                    (-yr[i]) - predict_yaw(est.x, v[i], swa[i], rl[i])
                    for i in np.flatnonzero(m)
                ]
                print(
                    f"             остаток рыска, ср.кв.: конфиг {np.std(res_cfg):.4f} → "
                    f"выучено {np.std(res_est):.4f} рад/с "
                    f"({100.0 * (1 - np.std(res_est) / max(np.std(res_cfg), 1e-9)):+.0f}%)"
                )


if __name__ == "__main__":
    main()
