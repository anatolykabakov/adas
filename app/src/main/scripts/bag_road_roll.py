#!/usr/bin/env python3
"""Can we estimate road bank from the phone accelerometer? Measure before building.

Why this matters. `paramsd` needs road roll to separate a banked road from an understeering car: both
produce the same lateral-acceleration signature, and without splitting them the learned tyre stiffness
absorbs the road. We have no orientation estimate at all, so this script asks whether the sensors we
already log can supply one, and — more importantly — what confounds it.

The physics is one line. In the vehicle frame (x forward, y right, z down) with the road banked by phi,
gravity has a lateral component, so a 3-axis accelerometer reads

    f_y = a_y_kinematic - g sin(phi)         with  a_y_kinematic = -v * yaw_rate_can

hence  sin(phi) = (-v * yaw_rate_can - f_y) / g.

Mind that minus. `chassis.yaw_rate` is decoded exactly as flowpilot decodes `ESP_02`, so it carries
openpilot's ISO convention — z up, positive for a **left** turn — while this frame has z down and positive
for a right turn. Getting it backwards does not look like a sign bug, it looks like a physics result: the
estimator then reports a body-roll gradient of **116 deg/g** instead of the 3-7 that a car can have, with a
correlation of 0.95. The number was 17x outside the physical range, which is the only reason it was caught.
Checked rather than fitted: regressing `f_y` on `v * yaw_rate_can` gives a slope of -0.986 with correlation
-0.92, i.e. the accelerometer and the yaw sensor agree to 1.4 % and only the sign convention differs.

Everything hard is in getting `f_y`: the phone is mounted at an unknown attitude, and the bag does not
carry the calibrated IMU frame (`sensors/imu_yaw` is not logged). So the script derives the mount frame
from the bag itself:

1. **down** from the mean accelerometer vector in stationary windows — gravity, two of three angles;
2. **forward** from the horizontal specific force during braking and acceleration, regressed against the
   wheel-speed derivative. That fixes the remaining yaw.

Then the confound, which is the real question. The phone is bolted to the body, and the body rolls on its
suspension *away* from the turn by roughly 3-7 degrees per g. That tilt is indistinguishable from road
bank in a single measurement, and it is proportional to lateral acceleration — so the script regresses the
estimate against `a_y` and reports the slope. A large slope means we are measuring the suspension.

  python3 bag_road_roll.py adas_logs/2026_08_06_00_36_42
  python3 bag_road_roll.py adas_logs/* --min-speed 8
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages

G = 9.81


def load(bag: Path):
    imu = load_topic_messages(bag, "sensors/imu")
    state = load_topic_messages(bag, "vehicle/state")
    if not imu or not state:
        return None

    t_imu = np.asarray([r[0] for r in imu], dtype=np.float64)
    accel = np.asarray(
        [[r[1].accel_x, r[1].accel_y, r[1].accel_z] for r in imu], dtype=np.float64
    )
    gyro = np.asarray(
        [[r[1].gyro_x, r[1].gyro_y, r[1].gyro_z] for r in imu], dtype=np.float64
    )

    t_st = np.asarray([r[0] for r in state], dtype=np.float64)

    def field(name):
        return np.asarray(
            [
                float(getattr(getattr(r[1], "car_state", None) or r[1], name, 0.0))
                for r in state
            ],
            dtype=np.float64,
        )

    keep = np.concatenate(([True], np.diff(t_st) > 0))
    return {
        "t_imu": t_imu,
        "accel": accel,
        "gyro": gyro,
        "t_st": t_st[keep],
        "v": field("v_ego")[keep],
        "yaw_rate": field("yaw_rate")[keep],
    }


def at(t_ref, t_src, v_src, max_dt_ms=120.0):
    idx = np.clip(np.searchsorted(t_src, t_ref), 0, len(t_src) - 1)
    prev = np.clip(idx - 1, 0, len(t_src) - 1)
    take_prev = np.abs(t_src[prev] - t_ref) < np.abs(t_src[idx] - t_ref)
    idx = np.where(take_prev, prev, idx)
    return v_src[idx], np.abs(t_src[idx] - t_ref) <= max_dt_ms


def mount_frame(accel, gyro, v_at_imu, a_at_imu):
    """Rotation from phone axes to vehicle axes (x forward, y right, z down), or None.

    Down comes from gravity while stationary; forward from the horizontal specific force during real
    longitudinal acceleration. Both are needed: gravity alone leaves the heading of the phone free, and a
    wrong heading turns longitudinal acceleration into a fake lateral one.
    """
    still = v_at_imu < 0.2
    if still.sum() < 50:
        return None, "too few stationary samples to find gravity"

    g_phone = accel[still].mean(axis=0)
    if not (8.0 < np.linalg.norm(g_phone) < 11.5):
        return None, f"stationary |accel| = {np.linalg.norm(g_phone):.2f}, not gravity"

    # An accelerometer at rest reads +g along the axis pointing UP, so down is the opposite.
    down = -g_phone / np.linalg.norm(g_phone)

    # Remove the gravity component; what is left is the horizontal specific force.
    horiz = accel - np.outer(accel @ down, down)

    # Forward is the horizontal direction whose projection best matches the wheel-speed derivative.
    # Use only samples with real longitudinal action, or the regression fits noise.
    m = np.isfinite(a_at_imu) & (np.abs(a_at_imu) > 0.4) & (v_at_imu > 2.0)
    if m.sum() < 100:
        return None, "too few braking/accelerating samples to find the forward axis"

    # Two orthonormal horizontal basis vectors.
    seed = np.array([1.0, 0.0, 0.0])
    if abs(seed @ down) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    e1 = seed - (seed @ down) * down
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(down, e1)

    c1 = float(np.dot(horiz[m] @ e1, a_at_imu[m]))
    c2 = float(np.dot(horiz[m] @ e2, a_at_imu[m]))
    if abs(c1) + abs(c2) < 1e-9:
        return None, "forward axis not identifiable"
    fwd = c1 * e1 + c2 * e2
    fwd /= np.linalg.norm(fwd)

    right = np.cross(down, fwd)  # x fwd, y right, z down is right-handed
    R = np.vstack([fwd, right, down])  # rows map phone vector -> vehicle components
    return R, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument(
        "--min-speed",
        type=float,
        default=8.0,
        help="m/s; bank is meaningless when crawling",
    )
    ap.add_argument(
        "--max-jerk",
        type=float,
        default=0.8,
        help="m/s^3-ish gate on d(a_y)/dt: the relation is a steady-state one",
    )
    args = ap.parse_args()

    for bag in args.bags:
        d = load(bag)
        print(f"\n=== {bag.name}")
        if d is None:
            print("  нет sensors/imu или vehicle/state")
            continue

        dt_imu = np.median(np.diff(d["t_imu"])) * 1e-3
        print(f"  IMU {len(d['t_imu'])} отсчётов, {1.0 / dt_imu:.0f} Гц")

        # Wheel acceleration, smoothed: the raw derivative of a quantised speed is useless (see
        # SpeedFilter in the localization chapter), and here it only has to identify an axis.
        v_st = d["v"]
        t_st = d["t_st"]
        dv = np.gradient(v_st) / np.maximum(np.gradient(t_st) * 1e-3, 1e-3)
        k = max(3, int(round(0.5 / max(np.median(np.diff(t_st)) * 1e-3, 1e-3))))
        kernel = np.ones(k) / k
        a_wheel = np.convolve(dv, kernel, mode="same")

        v_at_imu, ok_v = at(d["t_imu"], t_st, v_st)
        a_at_imu, ok_a = at(d["t_imu"], t_st, a_wheel)
        r_at_imu, ok_r = at(d["t_imu"], t_st, d["yaw_rate"])

        R, err = mount_frame(
            d["accel"],
            d["gyro"],
            np.where(ok_v, v_at_imu, 1e9),
            np.where(ok_a, a_at_imu, np.nan),
        )
        if R is None:
            print(f"  оценка системы крепления не вышла: {err}")
            continue
        print("  оси крепления (строки = вперёд / вправо / вниз в осях телефона):")
        for name, row in zip(("вперёд", "вправо", "вниз  "), R):
            print(f"    {name} {np.round(row, 3)}")

        f_veh = d["accel"] @ R.T  # specific force in vehicle axes
        f_y = f_veh[:, 1]
        # See the module docstring: CAN yaw rate is ISO (z up, left-positive); this frame is z down.
        a_y_kin = -v_at_imu * r_at_imu

        sin_phi = (a_y_kin - f_y) / G
        roll_deg = np.degrees(np.arcsin(np.clip(sin_phi, -1.0, 1.0)))

        d_ay = np.abs(np.gradient(a_y_kin) / max(dt_imu, 1e-3))
        good = (
            ok_v
            & ok_r
            & (v_at_imu >= args.min_speed)
            & (d_ay < args.max_jerk)
            & np.isfinite(roll_deg)
        )
        n = int(good.sum())
        print(
            f"  пригодных отсчётов: {n} из {len(roll_deg)} "
            f"(v ≥ {args.min_speed}, |d a_y/dt| < {args.max_jerk})"
        )
        if n < 200:
            print("  слишком мало для выводов")
            continue

        rd = roll_deg[good]
        ay = a_y_kin[good]
        print(
            f"  оценка крена: медиана {np.median(rd):+.2f}°, p10 {np.percentile(rd, 10):+.2f}, "
            f"p90 {np.percentile(rd, 90):+.2f}, ск.кв. {np.std(rd):.2f}"
        )

        # The confound. Body roll is proportional to lateral acceleration; road bank on these roads is
        # mostly camber and should not be. A large slope means the suspension, not the road.
        A = np.vstack([ay, np.ones_like(ay)]).T
        slope, intercept = np.linalg.lstsq(A, rd, rcond=None)[0]
        resid = rd - (A @ np.array([slope, intercept]))
        corr = float(np.corrcoef(ay, rd)[0, 1])
        print(
            f"  против поперечного ускорения: наклон {slope:+.2f} °/(м/с²) = {slope * G:+.1f} °/g, "
            f"сдвиг {intercept:+.2f}°, корреляция {corr:+.2f}"
        )
        print(
            f"  остаток после снятия наклона: ск.кв. {np.std(resid):.2f}° "
            f"(было {np.std(rd):.2f}°)"
        )
        print(
            "  наклон 3–7 °/g — это крен кузова на подвеске, а не дорога; тогда сдвиг ближе к дороге"
        )
    print(
        "\nПеречитайте наклон прежде, чем радоваться медиане: если он в диапазоне крена кузова,\n"
        "оценка измеряет подвеску, и без её вычитания в paramsd она даст выученную жёсткость,\n"
        "впитавшую дорогу."
    )


if __name__ == "__main__":
    main()
