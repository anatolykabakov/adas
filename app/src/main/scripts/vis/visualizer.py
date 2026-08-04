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
from core.gps_utils import calculate_initial_heading_from_gps, gps_to_local_coords
from core.imu_utils import process_imu_for_odometry
from vis.plotting import plot_trajectory
from vis.trajectory_calculators import (
    calculate_trajectory,
    calculate_trajectory_ekf,
    calculate_trajectory_imu,
)

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
        # calculate_trajectory expects vr, vl, hr, hl (= fr, fl, rr, rl)
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
    """(N, 6): timestamp, lat, lon, alt, speed, bearing_deg. Prefer sensors/gps/location."""
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
            ]
        )
    return np.asarray(rows, dtype=np.float64) if rows else None


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
    wheel: List,
    steering: List,
    gear: List,
    gps_data: Optional[np.ndarray],
    imu_data: Optional[np.ndarray],
    output_dir: str,
) -> None:
    initial_yaw = 0.0
    if gps_data is not None and len(gps_data) >= 2:
        initial_yaw = calculate_initial_heading_from_gps(gps_data, n_points=5)
        print(
            f"Initial heading from GPS: {np.degrees(initial_yaw):.1f}° "
            f"(yaw={initial_yaw:.3f} rad)"
        )

    x_gps = y_gps = None
    if gps_data is not None and len(gps_data) > 0:
        print("Converting GPS to local frame...")
        x_gps, y_gps = gps_to_local_coords(gps_data, origin_idx=0)
        print(f"GPS trajectory: {len(x_gps)} points")

    x_imu = y_imu = x_ekf = y_ekf = None
    x_fusion = y_fusion = x_gps_fusion = y_gps_fusion = None

    if (
        imu_data is not None
        and len(imu_data) > 0
        and wheel
        and gps_data is not None
        and len(gps_data) > 0
    ):
        print("\nProcessing IMU...")
        try:
            imu_processed = process_imu_for_odometry(
                imu_data,
                np.array(wheel),
                speed_threshold_orientation=0.1,
                speed_threshold_bias=0.5,
                time_window_sec=20.0,
                invert_yaw_rate=True,
            )
            print("IMU trajectory...")
            x_imu, y_imu = calculate_trajectory_imu(
                wheel,
                imu_processed["yaw_rate"],
                imu_processed["imu_calibrated"][:, 0],
                initial_yaw=initial_yaw,
            )
            print(f"IMU trajectory: {len(x_imu)} points")

            print("EKF trajectory...")
            x_ekf, y_ekf, _ekf = calculate_trajectory_ekf(
                wheel,
                steering,
                gear,
                imu_processed["yaw_rate"],
                imu_processed["imu_calibrated"][:, 0],
                gps_data,
                wheelbase=2.636,
                initial_yaw=initial_yaw,
                alpha_imu=0.7,
                gps_update_interval=1.0,
                imu_update_interval=0.01,
            )
            print(f"EKF trajectory: {len(x_ekf)} points")
        except Exception as e:
            print(f"Warning: IMU/EKF failed: {e}")
            import traceback

            traceback.print_exc()
            x_imu = y_imu = x_ekf = y_ekf = None

    if not wheel:
        print("No vehicle/state wheel data — skip trajectory plot")
        return

    print("Odometry (steering) trajectory...")
    x_odom, y_odom = calculate_trajectory(
        wheel, steering, gear, wheelbase=2.636, initial_yaw=initial_yaw
    )
    plot_trajectory(
        x_odom,
        y_odom,
        x_gps,
        y_gps,
        x_imu,
        y_imu,
        x_fusion,
        y_fusion,
        x_gps_fusion,
        y_gps_fusion,
        x_ekf,
        y_ekf,
        output_dir,
    )
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

        wheel, steering, gear = extract_vehicle_series(player)
        gps_data = extract_gps(player)
        imu_data = extract_imu(player)
        print(
            f"Extracted: wheel={len(wheel)} steering={len(steering)} "
            f"gear={len(gear)} gps={0 if gps_data is None else len(gps_data)} "
            f"imu={0 if imu_data is None else len(imu_data)}"
        )

        do_plot = args.plot_trajectory and not args.no_plot_trajectory
        if do_plot:
            build_trajectories(wheel, steering, gear, gps_data, imu_data, args.output)
        maybe_dump_first_image(player, args.output)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
