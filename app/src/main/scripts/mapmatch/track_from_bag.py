#!/usr/bin/env python3
"""Motion profile from a bag → track and maneuver chain (C++ `pyadas.core.mapmatch`).

GNSS is not used as input: in the city it drops out, but odometry knows the route shape without it.
If the bag contains GNSS, the script uses it only as a reference for verification — to measure how
well the track shape matches the actual trajectory.

  python3 -m mapmatch.track_from_bag <bag>
  python3 -m mapmatch.track_from_bag <bag> --plot track.png --csv track.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from vis.bag_io import list_topics, load_topic_messages

mm = pyadas.mapmatch


def motion_profile(bag: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(t, v, ω) from vehicle/state: wheel speed and yaw rate from the ESP sensor."""
    rows = load_topic_messages(bag, "vehicle/state")
    if not rows:
        raise SystemExit(f"{bag}: no vehicle/state topic")
    t = np.array([float(m.timestamp) for _, m in [(r[0], r[1]) for r in rows]]) / 1000.0
    v = np.array(
        [float(getattr(m, "v_ego", 0.0) or 0.0) for _, m in [(r[0], r[1]) for r in rows]]
    )
    w = np.array(
        [
            float(getattr(m, "yaw_rate", 0.0) or 0.0)
            for _, m in [(r[0], r[1]) for r in rows]
        ]
    )
    order = np.argsort(t)
    return t[order], v[order], w[order]


def imu_samples(bag: Path):
    """Raw phone IMU from the bag, if present — a second heading source."""
    if "sensors/imu" not in list_topics(bag):
        return None
    rows = load_topic_messages(bag, "sensors/imu")
    if len(rows) < 50:
        return None
    imu = mm.ImuSamples()
    msgs = [r[1] for r in rows]
    imu.t_s = [float(m.timestamp) / 1000.0 for m in msgs]
    imu.gyro_x = [float(m.gyro_x) for m in msgs]
    imu.gyro_y = [float(m.gyro_y) for m in msgs]
    imu.gyro_z = [float(m.gyro_z) for m in msgs]
    imu.accel_x = [float(m.accel_x) for m in msgs]
    imu.accel_y = [float(m.accel_y) for m in msgs]
    imu.accel_z = [float(m.accel_z) for m in msgs]
    return imu


def gnss_reference(bag: Path) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Reference for verification: GNSS in local meters + speed. None if absent."""
    if "sensors/gps/location" not in list_topics(bag):
        return None
    rows = load_topic_messages(bag, "sensors/gps/location")
    pts = [
        (float(m.latitude), float(m.longitude), float(getattr(m, "speed", 0.0) or 0.0))
        for _, m in [(r[0], r[1]) for r in rows]
        if abs(float(m.latitude)) > 1e-6
    ]
    if len(pts) < 5:
        return None
    lat = np.array([p[0] for p in pts])
    lon = np.array([p[1] for p in pts])
    spd = np.array([p[2] for p in pts])
    frame = mm.LocalFrame(lat0_deg=float(lat[0]), lon0_deg=float(lon[0]))
    xy = np.array([frame.to_local(float(a), float(b)) for a, b in zip(lat, lon)])
    return xy, spd


def shape_agreement(
    track, gnss: Optional[Tuple[np.ndarray, np.ndarray]]
) -> Optional[dict]:
    """How well the track shape matches the GNSS track, without absolute pose alignment.

    Compare quantities independent of initial position and heading: path length and total
    turn. GNSS heading is computed from position differences and only where the vehicle
    is moving: at standstill and low speed the points jitter, and heading accumulated from
    them is garbage (using it I initially got +147° instead of +47°). Full overlay is the
    job of the matching step; here we do a coarse check.
    """
    if gnss is None:
        return None
    xy, spd = gnss
    if len(xy) < 5:
        return None
    step = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    moving = (step > 3.0) & (step < 200.0) & (spd[:-1] > 3.0)
    if moving.sum() < 5:
        return None
    head = np.unwrap(np.arctan2(np.diff(xy[:, 1]), np.diff(xy[:, 0]))[moving])
    return {
        "gnss_len_m": float(np.sum(step[step < 200.0])),
        "odo_len_m": float(track.length_m),
        "len_ratio": float(track.length_m / max(1.0, float(np.sum(step[step < 200.0])))),
        "gnss_turn_deg": float(np.rad2deg(head[-1] - head[0])),
        "odo_turn_deg": float(track.total_turn_deg),
        "n_moving": int(moving.sum()),
    }


def print_report(bag: Path, track, agree: Optional[dict]) -> None:
    turns = [m for m in track.maneuvers if m.is_turn]
    straights = [m for m in track.maneuvers if not m.is_turn]
    print(
        f"{bag.name}: path {track.length_m:.0f} m, total turn {track.total_turn_deg:+.0f}°, "
        f"maneuvers {len(track.maneuvers)} ({len(turns)} turns)"
    )
    if straights:
        lens = np.array([m.length_m for m in straights])
        print(
            f"  straights: {len(straights)}, median {np.median(lens):.0f} m, longest {lens.max():.0f} m"
        )
    if turns:
        ang = np.array([abs(m.angle_deg) for m in turns])
        rad = np.array([m.radius_m for m in turns])
        print(
            f"  turns: median angle {np.median(ang):.0f}°, R median {np.median(rad):.0f} m, "
            f"left {sum(1 for m in turns if m.is_left)} / right {sum(1 for m in turns if not m.is_left)}"
        )
    if agree:
        print(
            f"  GNSS check (reference, not used as input): path {agree['odo_len_m']:.0f} vs "
            f"{agree['gnss_len_m']:.0f} m (×{agree['len_ratio']:.3f}), "
            f"turn {agree['odo_turn_deg']:+.0f}° vs {agree['gnss_turn_deg']:+.0f}° "
            f"({agree['n_moving']} moving steps)"
        )
    print()
    print("  " + (track.describe() or "no maneuvers found"))


def plot(path: Path, track, gnss_xy: Optional[np.ndarray]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    x = np.array(track.x_m)
    y = np.array(track.y_m)
    axes[0].plot(x, y, lw=1.2, label="odometry")
    for m in track.maneuvers:
        if not m.is_turn:
            continue
        i = int(np.searchsorted(np.array(track.s_m), 0.5 * (m.s_start_m + m.s_end_m)))
        i = min(i, len(x) - 1)
        axes[0].plot(
            x[i], y[i], "o", ms=6, color="tab:red" if m.is_left else "tab:orange"
        )
        axes[0].annotate(
            f"{m.angle_deg:+.0f}°",
            (x[i], y[i]),
            fontsize=8,
            textcoords="offset points",
            xytext=(6, 4),
        )
    axes[0].set_aspect("equal")
    axes[0].set_title("Track shape from odometry (red = left, orange = right)")
    axes[0].set_xlabel("m")
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)

    if gnss_xy is not None and len(gnss_xy) > 4:
        axes[1].plot(gnss_xy[:, 0], gnss_xy[:, 1], lw=1.0, color="tab:green")
        axes[1].set_aspect("equal")
        axes[1].set_title("GNSS from the same bag — verification only")
        axes[1].set_xlabel("m")
        axes[1].grid(alpha=0.3)
    else:
        axes[1].text(0.5, 0.5, "no GNSS in bag", ha="center", transform=axes[1].transAxes)
        axes[1].axis("off")

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"plot → {path}")


def write_csv(path: Path, track) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["s_m", "x_m", "y_m", "theta_deg"])
        for s, x, y, th in zip(track.s_m, track.x_m, track.y_m, track.theta_rad):
            w.writerow([f"{s:.1f}", f"{x:.2f}", f"{y:.2f}", f"{np.rad2deg(th):.2f}"])
    print(f"CSV → {path}")


def build(bag: Path, args) -> object:
    t, v, w = motion_profile(bag)
    cfg = mm.TrackConfig()
    cfg.resample_m = args.resample_m
    cfg.speed_scale = args.speed_scale
    cfg.yaw_rate_scale = args.yaw_rate_scale
    seg = mm.SegmentConfig()
    seg.turn_radius_m = args.turn_radius_m
    seg.min_turn_deg = args.min_turn_deg
    seg.merge_gap_m = args.merge_gap_m
    seg.smooth_m = args.smooth_m
    seg.min_straight_m = args.min_straight_m
    return mm.build_track(list(t), list(v), list(w), cfg, seg)


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("bags", nargs="+", type=Path)
    p.add_argument("--resample-m", type=float, default=2.0)
    p.add_argument(
        "--speed-scale", type=float, default=1.0, help="speed scale (wheel radius)"
    )
    p.add_argument(
        "--yaw-rate-scale",
        type=float,
        default=1.0,
        help="yaw-rate sensor scale",
    )
    p.add_argument(
        "--turn-radius-m",
        type=float,
        default=80.0,
        help="sharper than this counts as a turn",
    )
    p.add_argument("--min-turn-deg", type=float, default=25.0)
    p.add_argument("--merge-gap-m", type=float, default=25.0)
    p.add_argument("--smooth-m", type=float, default=10.0)
    p.add_argument(
        "--min-straight-m",
        type=float,
        default=20.0,
        help="shorter straight = junction of two turns, not a segment",
    )
    p.add_argument("--plot", type=Path, default=None)
    p.add_argument("--csv", type=Path, default=None)
    args = p.parse_args()

    for bag in args.bags:
        track = build(bag, args)
        if track.length_m <= 0:
            print(f"{bag.name}: track not built (insufficient data)")
            continue
        gnss = gnss_reference(bag)
        print_report(bag, track, shape_agreement(track, gnss))
        if args.plot:
            out = (
                args.plot
                if len(args.bags) == 1
                else args.plot.with_stem(f"{args.plot.stem}_{bag.name}")
            )
            plot(out, track, gnss[0] if gnss else None)
        if args.csv:
            out = (
                args.csv
                if len(args.bags) == 1
                else args.csv.with_stem(f"{args.csv.stem}_{bag.name}")
            )
            write_csv(out, track)
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
