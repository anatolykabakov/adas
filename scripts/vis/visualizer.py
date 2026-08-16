#!/usr/bin/env python3
"""Trajectory visualization for Android ADAS bag sessions.

Reads sessions via vis.android_bag_player (topic__name/*.bin), same layout as
adas_bags/YYYY_MM_DD_HH_MM_SS.

Usage:
  python3 vis/visualizer.py /path/to/2026_07_18_09_45_15
  python3 vis/visualizer.py /path/to/session.zip -o plots --plot-trajectory
"""

from __future__ import annotations

import _path  # noqa: F401

import argparse
import os
import sys
from pathlib import Path
from typing import Any, List, Optional, Tuple

import numpy as np
import matplotlib

matplotlib.use("Agg")

from vis.android_bag_player import AndroidBagPlayer
from core.gps_utils import gps_to_local_coords
from vis.plotting import plot_trajectory

# VW MQB GE_Fahrstufe → name used by VehicleModel
_GEAR_NAME = {
    5: "PARK",
    6: "REVERSE",
    7: "NEUTRAL",
    8: "DRIVE",
    9: "SPORT",
    10: "ECO",
}

# VehicleModel converts abs*unit_to_degrees; invert so bag degrees round-trip.
_UNIT_TO_DEG = 32.72 / 400.0
_MPS_TO_KMH = 3.6
# car_state.steering_angle_deg is LWI_Lenkradwinkel (steering-wheel deg, ±~500).
# Bicycle model needs road-wheel angle ≈ SW / steer_ratio.
_STEER_RATIO = 15.7  # Golf 7 / MQB typical; ~17.5 also fits yaw_rate on this bag


def _gear_name(raw: int) -> str:
    return _GEAR_NAME.get(int(raw), "DRIVE")


def extract_vehicle_series(
    player: AndroidBagPlayer,
) -> Tuple[List, List, List]:
    """wheel [[t, vr, vl, hr, hl] km/h], steering [[t, abs, sign]], gear [[t, name, raw]]."""
    wheel: List = []
    steering: List = []
    gear: List = []
    for ts, msg in player.get_topic_msgs("vehicle/state"):
        ws = msg.wheel_speeds
        # wheel speed order: vr, vl, hr, hl (= fr, fl, rr, rl)
        wheel.append(
            [
                ts,
                float(ws.fr) * _MPS_TO_KMH,
                float(ws.fl) * _MPS_TO_KMH,
                float(ws.rr) * _MPS_TO_KMH,
                float(ws.rl) * _MPS_TO_KMH,
            ]
        )
        deg = float(msg.steering_angle_deg) / _STEER_RATIO  # SW → road wheel
        steering.append([ts, abs(deg) / _UNIT_TO_DEG, 1.0 if deg < 0 else 0.0])
        gear.append([ts, _gear_name(msg.gear), int(msg.gear)])
    return wheel, steering, gear


def extract_imu(player: AndroidBagPlayer) -> Optional[np.ndarray]:
    """(N, 10): timestamp, ax, ay, az, gx, gy, gz, mx, my, mz."""
    rows = []
    for ts, msg in player.get_topic_msgs("sensors/imu"):
        rows.append(
            [
                ts,
                float(msg.accel_x),
                float(msg.accel_y),
                float(msg.accel_z),
                float(msg.gyro_x),
                float(msg.gyro_y),
                float(msg.gyro_z),
                float(msg.mag_x),
                float(msg.mag_y),
                float(msg.mag_z),
            ]
        )
    return np.asarray(rows, dtype=np.float64) if rows else None


def extract_gps(player: AndroidBagPlayer) -> Optional[np.ndarray]:
    """(N, 8): timestamp, lat, lon, alt, speed, bearing_deg, horizontal_accuracy_m, satellites.

    The last two columns are what tells a 10 m fix from a 153 m one. Without them the offline EKF
    admits every fix at the same weight and the accuracy gate in the filter can never fire, so the
    replayed track tangles where the receiver degraded while the on-device pose did not.
    """
    topic = (
        "sensors/gps/location"
        if "sensors/gps/location" in player.topics
        else "sensors/gps/data"
    )
    if topic not in player.topics:
        return None
    rows = []
    for ts, msg in player.get_topic_msgs(topic):
        bearing = float(getattr(msg, "bearing", 0.0) or 0.0)
        rows.append(
            [
                ts,
                float(msg.latitude),
                float(msg.longitude),
                float(msg.altitude),
                float(msg.speed),
                bearing,
                float(getattr(msg, "horizontal_accuracy", 0.0) or 0.0),
                float(getattr(msg, "satellites_used", 0) or 0),
            ]
        )
    return np.asarray(rows, dtype=np.float64) if rows else None


def extract_pose(player: AndroidBagPlayer) -> Optional[np.ndarray]:
    """(N, 3): timestamp, east_m, north_m — the pose the device published, straight from the bag."""
    topic = "localization/pose"
    if topic not in player.topics:
        return None
    rows = [[ts, float(m.x), float(m.y)] for ts, m in player.get_topic_msgs(topic)]
    return np.asarray(rows, dtype=np.float64) if rows else None


def extract_camera_calib(player: AndroidBagPlayer) -> Optional[np.ndarray]:
    """(N, 5): timestamp, pitch_deg, yaw_deg, height_m, converged — the mounting the car actually used."""
    topic = "calibration/camera"
    if topic not in player.topics:
        return None
    rows = [
        [
            ts,
            float(m.pitch_deg),
            float(m.yaw_deg),
            float(m.camera_height_m),
            1.0 if bool(m.calibration_success) else 0.0,
        ]
        for ts, m in player.get_topic_msgs(topic)
    ]
    return np.asarray(rows, dtype=np.float64) if rows else None


def align_pose_to_gps_frame(
    pose: np.ndarray, gps_data: np.ndarray, max_acc: float = 10.0
):
    """Rigid shift of a recorded pose into the panel's GPS-anchored frame.

    The device anchors its ENU plane at its own origin, not at the first fix in this bag, so the two
    frames differ by a constant translation — 436 m on 2026_08_11_09_49_43, constant to 0.3 m over
    1236 s. Drawing them together without this reads as a huge localisation error that is not there.
    Only fixes at or below ``max_acc`` are used, since bad fixes would bias the offset itself.
    """
    if pose is None or gps_data is None or len(pose) < 2 or len(gps_data) < 2:
        return pose
    x_gps, y_gps = gps_to_local_coords(gps_data, origin_idx=0)
    acc = gps_data[:, 6] if gps_data.shape[1] >= 7 else np.zeros(len(gps_data))
    k = (acc > 0) & (acc <= max_acc)
    if k.sum() < 5:
        k = np.ones(len(gps_data), dtype=bool)
    tg = gps_data[k, 0]
    dx = float(np.median(np.interp(tg, pose[:, 0], pose[:, 1]) - x_gps[k]))
    dy = float(np.median(np.interp(tg, pose[:, 0], pose[:, 2]) - y_gps[k]))
    out = pose.copy()
    out[:, 1] -= dx
    out[:, 2] -= dy
    return out


def print_summary(player: AndroidBagPlayer) -> None:
    start, end = player.get_time_range()
    print("\n" + "=" * 60)
    print("BAG SUMMARY")
    print("=" * 60)
    print(f"Path: {player.session_dir}")
    for topic in player.topics:
        print(f"  {topic}: {len(player.get_topic_msgs(topic))} msgs")
    if start or end:
        print(f"Duration: {player.get_duration()} ({(end - start) / 1000.0:.1f} s)")
    print("=" * 60 + "\n")


def build_trajectories(
    player: AndroidBagPlayer,
    gps_data: Optional[np.ndarray],
    output_dir: str,
) -> None:
    """Plot what the bag holds: the GNSS fixes and the pose the device published.

    Odometry, an IMU track and an offline EKF were computed here before. Each re-derived the drive from
    Python with its own inputs and tuning, and the EKF one published fixes without their accuracy, so it
    admitted 150 m fixes at full weight and tangled where the device's own pose ran clean.
    """
    x_gps = y_gps = None
    if gps_data is not None and len(gps_data) > 0:
        x_gps, y_gps = gps_to_local_coords(gps_data, origin_idx=0)
        print(f"GPS: {len(x_gps)} points")

    pose = align_pose_to_gps_frame(extract_pose(player), gps_data)
    x_pose = pose[:, 1] if pose is not None else None
    y_pose = pose[:, 2] if pose is not None else None
    if pose is not None:
        print(f"Pose: {len(pose)} points")

    if x_gps is None and x_pose is None:
        print("No GNSS and no pose in the bag — skip trajectory plot")
        return

    plot_trajectory(x_gps, y_gps, x_pose, y_pose, output_dir)
    print(f"Saved trajectory → {output_dir}/trajectory.png")


def maybe_dump_first_image(player: AndroidBagPlayer, output_dir: str) -> None:
    topic = "sensors/camera/image"
    if topic not in player.topics:
        return
    try:
        import cv2
    except ImportError:
        return
    for _ts, msg in player.get_topic_msgs(topic)[:1]:
        data = getattr(msg, "image_data", b"") or b""
        if not data:
            return
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        os.makedirs(output_dir, exist_ok=True)
        out = Path(output_dir) / "first_frame.jpg"
        cv2.imwrite(str(out), img)
        print(f"First camera frame → {out} ({img.shape[1]}x{img.shape[0]})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "bag",
        nargs="?",
        type=Path,
        help="Session dir or .zip/.tar.gz (topic__*/.bin)",
    )
    ap.add_argument(
        "--input",
        "-i",
        type=Path,
        help="Alias for bag path (compat)",
    )
    ap.add_argument(
        "--output",
        "-o",
        default="plots",
        help="Output directory for plots",
    )
    ap.add_argument(
        "--plot-trajectory",
        action="store_true",
        default=True,
        help="Build trajectory plot (default: on)",
    )
    ap.add_argument(
        "--no-plot-trajectory",
        action="store_true",
        help="Skip trajectory plot",
    )
    ap.add_argument(
        "--summary-only",
        action="store_true",
        help="Only print bag summary",
    )
    args = ap.parse_args()

    bag_path = args.bag or args.input
    if bag_path is None:
        ap.error("provide bag path or -i/--input")
    if not bag_path.exists():
        print(f"Not found: {bag_path}", file=sys.stderr)
        return 1

    with AndroidBagPlayer(bag_path) as player:
        print_summary(player)
        if args.summary_only:
            return 0

        # print_summary already lists every topic with its message count, and wheel / steering / IMU
        # series are no longer needed here: nothing is re-derived from them.
        gps_data = extract_gps(player)

        do_plot = args.plot_trajectory and not args.no_plot_trajectory
        if do_plot:
            build_trajectories(player, gps_data, args.output)
        maybe_dump_first_image(player, args.output)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
