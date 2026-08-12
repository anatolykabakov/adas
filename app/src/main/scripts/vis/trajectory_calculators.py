#!/usr/bin/env python3
"""Functions for calculating vehicle trajectory using various methods.

Supported methods:
1. Odometry (Steering) - classic bicycle model
2. IMU (Gyroscope) - direct yaw rate measurement
3. Fusion (IMU + Steering) - sensor fusion with G-H filter
"""

from __future__ import annotations

import _path  # noqa: F401

import bisect
from typing import List, Tuple

import numpy as np

from pyadas import core as pyadas

from core.vehicle_model import VehicleModel, normalize_angle
from core.gps_utils import gps_to_local_coords


def calculate_trajectory_imu(
    wheel: List,
    imu_yaw_rate: np.ndarray,
    imu_timestamps: np.ndarray,
    initial_yaw: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate vehicle trajectory using IMU (gyroscope) data.

    Method:
    - Speed is taken from odometry (wheel speed)
    - Yaw rate (angular velocity) is taken from the IMU gyroscope

    Args:
        wheel: List of wheel data [[timestamp, vr, vl, hr, hl], ...]
        imu_yaw_rate: numpy array (N,) - angular velocity from IMU (gz_vehicle)
        imu_timestamps: numpy array (N,) - IMU data timestamps
        initial_yaw: Initial orientation in radians

    Returns:
        Tuple (x_array, y_array) - trajectory coordinates
    """
    wheel_array = np.array(wheel)
    if len(wheel_array) < 2:
        return np.array([0]), np.array([0])

    x_list, y_list = [0.0], [0.0]
    yaw = float(initial_yaw)

    prev_time = wheel_array[0, 0]

    for i in range(1, len(wheel_array)):
        curr_time = wheel_array[i, 0]
        dt = (curr_time - prev_time) / 1000.0  # Convert to seconds

        if dt < 0.0001 or dt > 1.0:  # Anomaly filter
            continue

        # Speed from odometry
        # Format: [timestamp, vr_kmh, vl_kmh, hr_kmh, hl_kmh]
        v_kmh = np.mean(wheel_array[i, 1:5])  # Average wheel speed in km/h
        v = v_kmh / 3.6  # Convert km/h → m/s

        # Yaw rate from IMU (interpolate to wheel timestamp)
        idx = bisect.bisect_left(imu_timestamps, curr_time)
        if idx >= len(imu_yaw_rate):
            idx = len(imu_yaw_rate) - 1
        elif idx > 0 and idx < len(imu_timestamps):
            # Linear interpolation between points
            t1, t2 = imu_timestamps[idx - 1], imu_timestamps[idx]
            if t2 != t1:
                alpha = (curr_time - t1) / (t2 - t1)
                omega = imu_yaw_rate[idx - 1] * (1 - alpha) + imu_yaw_rate[idx] * alpha
            else:
                omega = imu_yaw_rate[idx]
        else:
            omega = imu_yaw_rate[idx]

        # Update state
        # (Sign inversion already applied in process_imu_for_odometry)
        yaw += omega * dt

        dx = v * np.cos(yaw) * dt
        dy = v * np.sin(yaw) * dt

        x_list.append(x_list[-1] + dx)
        y_list.append(y_list[-1] + dy)

        prev_time = curr_time

    return np.array(x_list), np.array(y_list)


def calculate_trajectory_fusion(
    wheel: List,
    steering: List,
    gear: List,
    imu_yaw_rate: np.ndarray,
    imu_timestamps: np.ndarray,
    wheelbase: float = 2.636,
    initial_yaw: float = 0.0,
    alpha: float = 0.7,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate trajectory using sensor fusion (IMU + odometry combined).

    Method: G-H filter (simple alpha-beta filter)
    yaw_rate_fusion = alpha * yaw_rate_imu + (1 - alpha) * yaw_rate_odom

    where:
    - yaw_rate_imu: angular velocity from gyroscope (direct measurement)
    - yaw_rate_odom: angular velocity from bicycle model (v * tan(δ) / L)
    - alpha: IMU weight (0.7 = trust IMU more, 0.3 = trust odometry more)

    Args:
        wheel: List of wheel data [[timestamp, vr, vl, hr, hl], ...]
        steering: List of steering data [[timestamp, angle_abs, sign], ...]
        gear: List of gear data [[timestamp, gear_name, gear_value], ...]
        imu_yaw_rate: numpy array (N,) - angular velocity from IMU (gz_vehicle)
        imu_timestamps: numpy array (N,) - IMU data timestamps
        wheelbase: Vehicle wheelbase in meters
        initial_yaw: Initial orientation in radians
        alpha: IMU weight in fusion (0-1), where 1 = IMU only, 0 = odometry only

    Returns:
        Tuple (x_array, y_array) - trajectory coordinates

    Example:
        # alpha = 0.7: 70% IMU, 30% odometry
        # alpha = 0.5: 50/50 (equal weight)
        # alpha = 0.3: 30% IMU, 70% odometry
    """
    # Create vehicle model
    vehicle_model = VehicleModel(wheelbase=wheelbase, initial_yaw=initial_yaw)

    wheel_array = np.array(wheel)
    if len(wheel_array) < 2:
        return np.array([0]), np.array([0])

    # Prepare steering data
    if steering is not None and len(steering) > 0:
        steering_array = np.array(steering)
        steering_timestamps = steering_array[:, 0].astype(int).tolist()
        steering_values = steering_array[:, 1:]
    else:
        steering_timestamps = []
        steering_values = np.array([])

    # Prepare gear data
    if gear is not None and len(gear) > 0:
        gear_array = np.array(gear, dtype=object)
        gear_timestamps = [int(g[0]) for g in gear]
        gear_names = [g[1] for g in gear]
    else:
        gear_timestamps = []
        gear_names = []

    # Pre-allocate arrays
    n = len(wheel_array)
    x_arr = np.zeros(n)
    y_arr = np.zeros(n)

    timestamps = wheel_array[:, 0]
    dt_arr = np.diff(timestamps) / 1e3  # seconds
    valid_indices = np.where(dt_arr >= 0.0001)[0] + 1

    result_idx = 0
    yaw = initial_yaw

    print(f"Fusion processing {len(valid_indices)} points (alpha={alpha:.1f})...")

    for i in valid_indices:
        t = int(timestamps[i])
        vr, vl, hr, hl = wheel_array[i, 1:5]
        dt = dt_arr[i - 1]

        # Vehicle speed
        v_kmh = (vr + vl + hr + hl) / 4.0
        v = v_kmh / 3.6  # km/h → m/s

        # Get steering angle
        steering_angle_abs = 0.0
        steering_angle_sign = 0.0
        if steering_timestamps:
            idx = bisect.bisect_left(steering_timestamps, t)
            if idx >= len(steering_timestamps):
                idx = len(steering_timestamps) - 1
            elif idx > 0 and abs(steering_timestamps[idx - 1] - t) < abs(
                steering_timestamps[idx] - t
            ):
                idx -= 1
            steering_angle_abs = steering_values[idx, 0]
            steering_angle_sign = steering_values[idx, 1]

        # Get gear
        current_gear_name = "DRIVE"
        if gear_timestamps:
            idx = bisect.bisect_left(gear_timestamps, t)
            if idx >= len(gear_timestamps):
                idx = len(gear_timestamps) - 1
            elif idx > 0 and abs(gear_timestamps[idx - 1] - t) < abs(
                gear_timestamps[idx] - t
            ):
                idx -= 1
            current_gear_name = gear_names[idx]

        # Yaw rate from ODOMETRY (bicycle model)
        steering_angle_deg = steering_angle_abs * vehicle_model.unit_to_degrees
        if steering_angle_sign != 0:
            steering_angle_deg = -steering_angle_deg
        steering_angle_rad = np.radians(steering_angle_deg)

        direction = -1.0 if current_gear_name == "REVERSE" else 1.0
        v_directed = v * direction

        if abs(steering_angle_rad) >= 0.001 and abs(v_directed) > 0.01:
            yaw_rate_odom = v_directed * np.tan(steering_angle_rad) / wheelbase
        else:
            yaw_rate_odom = 0.0

        # Yaw rate from IMU (interpolation)
        idx = bisect.bisect_left(imu_timestamps, t)
        if idx >= len(imu_yaw_rate):
            idx = len(imu_yaw_rate) - 1
        elif idx > 0 and idx < len(imu_timestamps):
            t1, t2 = imu_timestamps[idx - 1], imu_timestamps[idx]
            if t2 != t1:
                alpha_interp = (t - t1) / (t2 - t1)
                yaw_rate_imu = (
                    imu_yaw_rate[idx - 1] * (1 - alpha_interp)
                    + imu_yaw_rate[idx] * alpha_interp
                )
            else:
                yaw_rate_imu = imu_yaw_rate[idx]
        else:
            yaw_rate_imu = imu_yaw_rate[idx]

        # ===== G-H FILTER (SENSOR FUSION) =====
        # ======================================
        # Combine yaw_rate from two sources:
        #   alpha = IMU weight (0.7 = trust IMU more)
        #   (1-alpha) = odometry weight (0.3)
        yaw_rate_fusion = alpha * yaw_rate_imu + (1 - alpha) * yaw_rate_odom

        # Update state with fusion yaw_rate
        yaw += yaw_rate_fusion * dt
        yaw = normalize_angle(yaw)

        dx = v_directed * np.cos(yaw) * dt
        dy = v_directed * np.sin(yaw) * dt

        x_arr[result_idx] = x_arr[result_idx - 1] + dx if result_idx > 0 else 0
        y_arr[result_idx] = y_arr[result_idx - 1] + dy if result_idx > 0 else 0
        result_idx += 1

    print(
        f"Fusion trajectory: {result_idx} points (IMU weight={alpha:.1%}, Odom weight={(1-alpha):.1%})"
    )

    return x_arr[:result_idx], y_arr[:result_idx]


def calculate_trajectory(
    wheel: List,
    steering: List,
    gear: List,
    wheelbase: float = 2.636,
    initial_yaw: float = 0.0,
) -> Tuple[List[float], List[float]]:
    """
    Calculate vehicle trajectory from odometry data.
    Uses VehicleModel to update state.

    Args:
        wheel: List of wheel data [[timestamp, vr, vl, hr, hl], ...]
        steering: List of steering data [[timestamp, angle_abs, sign], ...]
        gear: List of gear data [[timestamp, gear_name, gear_value], ...]
        wheelbase: Vehicle wheelbase in meters
        initial_yaw: Initial orientation in radians (0 = east, π/2 = north)

    Returns:
        Tuple (x_list, y_list) - trajectory coordinates
    """
    # Create vehicle model with initial orientation
    vehicle_model = VehicleModel(wheelbase=wheelbase, initial_yaw=initial_yaw)

    # Convert to NumPy arrays for fast processing
    wheel_array = np.array(wheel)
    if len(wheel_array) < 2:
        return [0], [0]

    # Create sorted arrays for binary search
    if steering is not None and len(steering) > 0:
        steering_array = np.array(steering)
        steering_timestamps = steering_array[:, 0].astype(int).tolist()
        steering_values = steering_array[:, 1:]
    else:
        steering_timestamps = []
        steering_values = np.array([])

    if gear is not None and len(gear) > 0:
        gear_array = np.array(gear, dtype=object)
        gear_timestamps = [int(g[0]) for g in gear]
        gear_names = [g[1] for g in gear]
    else:
        gear_timestamps = []
        gear_names = []

    # Pre-allocate arrays (faster than append)
    n = len(wheel_array)
    x_arr = np.zeros(n)
    y_arr = np.zeros(n)

    # Compute dt
    timestamps = wheel_array[:, 0]
    dt_arr = np.diff(timestamps) / 1e3  # seconds

    # Filter small dt (0.0001 sec = 0.1 ms)
    # For high-frequency data (~100-1000 Hz) use a small threshold
    valid_indices = np.where(dt_arr >= 0.0001)[0] + 1

    result_idx = 0

    print(f"Processing {len(valid_indices)} trajectory points...")

    for i in valid_indices:
        t = int(timestamps[i])
        vr, vl, hr, hl = wheel_array[i, 1:5]
        dt = dt_arr[i - 1]

        # Fast lookup of nearest steering and gear values
        # Use binary search (bisect) - O(log n) instead of O(n)
        steering_angle_abs = 0.0
        steering_angle_sign = 0.0
        if steering_timestamps:
            idx = bisect.bisect_left(steering_timestamps, t)
            if idx >= len(steering_timestamps):
                idx = len(steering_timestamps) - 1
            elif idx > 0 and abs(steering_timestamps[idx - 1] - t) < abs(
                steering_timestamps[idx] - t
            ):
                idx -= 1
            steering_angle_abs = steering_values[idx, 0]
            steering_angle_sign = steering_values[idx, 1]

        current_gear_name = "DRIVE"
        if gear_timestamps:
            idx = bisect.bisect_left(gear_timestamps, t)
            if idx >= len(gear_timestamps):
                idx = len(gear_timestamps) - 1
            elif idx > 0 and abs(gear_timestamps[idx - 1] - t) < abs(
                gear_timestamps[idx] - t
            ):
                idx -= 1
            current_gear_name = gear_names[idx]

        # Update vehicle state via VehicleModel
        x, y, yaw = vehicle_model.update(
            vr, vl, hr, hl, steering_angle_abs, steering_angle_sign, current_gear_name, dt
        )

        x_arr[result_idx] = x
        y_arr[result_idx] = y
        result_idx += 1

    # Trim arrays to actual size
    x_list = x_arr[:result_idx].tolist()
    y_list = y_arr[:result_idx].tolist()

    print(f"Trajectory calculated: {result_idx} points")

    return x_list, y_list


def calculate_trajectory_gps_fusion(
    wheel: List,
    steering: List,
    gear: List,
    imu_yaw_rate: np.ndarray,
    imu_timestamps: np.ndarray,
    gps_data: np.ndarray,
    wheelbase: float = 2.636,
    initial_yaw: float = 0.0,
    alpha_imu: float = 0.7,
    gps_position_gain: float = 0.1,
    gps_yaw_gain: float = 0.05,
    gps_correction_interval: float = 5.0,
    min_gps_speed: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate trajectory with GPS loose coupling (periodic GPS correction).

    Method:
    1. Primary trajectory: Fusion (alpha*IMU + (1-alpha)*Steering)
    2. Periodic GPS correction (every N seconds):
       - Position correction (x, y)
       - Orientation correction (yaw)
    3. Soft correction (small gain coefficients)

    Formulas:
        yaw_rate = alpha_imu * yaw_rate_imu + (1 - alpha_imu) * yaw_rate_odom

        if time_for_gps_correction:
            x += gps_position_gain * (gps_x - x)
            y += gps_position_gain * (gps_y - y)
            yaw += gps_yaw_gain * normalize_angle(gps_heading - yaw)

    Args:
        wheel: List of wheel data [[timestamp, vr, vl, hr, hl], ...]
        steering: List of steering data [[timestamp, angle_abs, sign], ...]
        gear: List of gear data [[timestamp, gear_name, gear_value], ...]
        imu_yaw_rate: numpy array (N,) - angular velocity from IMU
        imu_timestamps: numpy array (N,) - IMU timestamps
        gps_data: numpy array (M, 5) [timestamp, lat, lon, alt, speed]
        wheelbase: Wheelbase in meters
        initial_yaw: Initial orientation in radians
        alpha_imu: IMU weight in fusion (0-1)
        gps_position_gain: GPS position correction coefficient (0-1, recommended 0.05-0.2)
        gps_yaw_gain: GPS yaw correction coefficient (0-1, recommended 0.02-0.1)
        gps_correction_interval: GPS correction interval in seconds
        min_gps_speed: minimum speed for using GPS heading (m/s)

    Returns:
        Tuple (x_array, y_array) - trajectory coordinates with GPS correction

    Example:
        x, y = calculate_trajectory_gps_fusion(
            wheel, steering, gear, imu_yaw_rate, imu_timestamps, gps_data,
            alpha_imu=0.7,           # 70% IMU, 30% steering
            gps_position_gain=0.1,   # 10% position correction every 5s
            gps_yaw_gain=0.05,       # 5% yaw correction
            gps_correction_interval=5.0
        )
    """
    # Create vehicle model
    vehicle_model = VehicleModel(wheelbase=wheelbase, initial_yaw=initial_yaw)

    wheel_array = np.array(wheel)
    if len(wheel_array) < 2:
        return np.array([0]), np.array([0])

    # Convert GPS to local coordinates
    x_gps, y_gps = gps_to_local_coords(gps_data, origin_idx=0)
    gps_timestamps = gps_data[:, 0]
    gps_speeds = gps_data[:, 4]  # GPS speed

    # Prepare steering and gear data
    if steering is not None and len(steering) > 0:
        steering_array = np.array(steering)
        steering_timestamps = steering_array[:, 0].astype(int).tolist()
        steering_values = steering_array[:, 1:]
    else:
        steering_timestamps = []
        steering_values = np.array([])

    if gear is not None and len(gear) > 0:
        gear_array = np.array(gear, dtype=object)
        gear_timestamps = [int(g[0]) for g in gear]
        gear_names = [g[1] for g in gear]
    else:
        gear_timestamps = []
        gear_names = []

    # Pre-allocate arrays
    n = len(wheel_array)
    x_arr = np.zeros(n)
    y_arr = np.zeros(n)

    timestamps = wheel_array[:, 0]
    dt_arr = np.diff(timestamps) / 1e3  # seconds
    valid_indices = np.where(dt_arr >= 0.0001)[0] + 1

    result_idx = 0
    yaw = initial_yaw
    last_gps_correction_time = timestamps[0]
    gps_corrections_count = 0
    gps_corrections_skipped_position = 0  # Skipped due to large position error
    gps_corrections_skipped_yaw_rate = 0  # Skipped due to sharp maneuver
    gps_corrections_reduced_gain = 0  # Correction with reduced gain

    print(f"GPS Fusion processing {len(valid_indices)} points...")
    print(
        f"  Parameters: alpha_IMU={alpha_imu:.1f}, GPS_pos_gain={gps_position_gain:.2f}, GPS_yaw_gain={gps_yaw_gain:.3f}"
    )
    print(f"  GPS correction every {gps_correction_interval:.1f}s")
    print(
        f"  Protection: adaptive gain (0-20m→100%, 20-50m→50%, 50-200m→20%, >200m→skip)"
    )
    print(f"          yaw only during smooth motion (<17°/s), yaw_error<45°")

    for i in valid_indices:
        t = int(timestamps[i])
        vr, vl, hr, hl = wheel_array[i, 1:5]
        dt = dt_arr[i - 1]

        # Vehicle speed
        v_kmh = (vr + vl + hr + hl) / 4.0
        v = v_kmh / 3.6  # km/h → m/s

        # Get steering angle and gear (same as fusion)
        steering_angle_abs = 0.0
        steering_angle_sign = 0.0
        if steering_timestamps:
            idx = bisect.bisect_left(steering_timestamps, t)
            if idx >= len(steering_timestamps):
                idx = len(steering_timestamps) - 1
            elif idx > 0 and abs(steering_timestamps[idx - 1] - t) < abs(
                steering_timestamps[idx] - t
            ):
                idx -= 1
            steering_angle_abs = steering_values[idx, 0]
            steering_angle_sign = steering_values[idx, 1]

        current_gear_name = "DRIVE"
        if gear_timestamps:
            idx = bisect.bisect_left(gear_timestamps, t)
            if idx >= len(gear_timestamps):
                idx = len(gear_timestamps) - 1
            elif idx > 0 and abs(gear_timestamps[idx - 1] - t) < abs(
                gear_timestamps[idx] - t
            ):
                idx -= 1
            current_gear_name = gear_names[idx]

        # Yaw rate from odometry (bicycle model)
        steering_angle_deg = steering_angle_abs * vehicle_model.unit_to_degrees
        if steering_angle_sign != 0:
            steering_angle_deg = -steering_angle_deg
        steering_angle_rad = np.radians(steering_angle_deg)

        direction = -1.0 if current_gear_name == "REVERSE" else 1.0
        v_directed = v * direction

        if abs(steering_angle_rad) >= 0.001 and abs(v_directed) > 0.01:
            yaw_rate_odom = v_directed * np.tan(steering_angle_rad) / wheelbase
        else:
            yaw_rate_odom = 0.0

        # Yaw rate from IMU (interpolation)
        idx = bisect.bisect_left(imu_timestamps, t)
        if idx >= len(imu_yaw_rate):
            idx = len(imu_yaw_rate) - 1
        elif idx > 0 and idx < len(imu_timestamps):
            t1, t2 = imu_timestamps[idx - 1], imu_timestamps[idx]
            if t2 != t1:
                alpha_interp = (t - t1) / (t2 - t1)
                yaw_rate_imu = (
                    imu_yaw_rate[idx - 1] * (1 - alpha_interp)
                    + imu_yaw_rate[idx] * alpha_interp
                )
            else:
                yaw_rate_imu = imu_yaw_rate[idx]
        else:
            yaw_rate_imu = imu_yaw_rate[idx]

        # Sensor fusion (IMU + Steering)
        yaw_rate_fusion = alpha_imu * yaw_rate_imu + (1 - alpha_imu) * yaw_rate_odom

        # Update state
        yaw += yaw_rate_fusion * dt
        yaw = normalize_angle(yaw)

        dx = v_directed * np.cos(yaw) * dt
        dy = v_directed * np.sin(yaw) * dt

        x_current = (x_arr[result_idx - 1] if result_idx > 0 else 0) + dx
        y_current = (y_arr[result_idx - 1] if result_idx > 0 else 0) + dy

        # ===== GPS CORRECTION (periodic with checks) =====
        # =================================================
        time_since_last_correction = (t - last_gps_correction_time) / 1000.0

        if time_since_last_correction >= gps_correction_interval:
            # Find nearest GPS point
            gps_idx = bisect.bisect_left(gps_timestamps, t)
            if gps_idx >= len(gps_timestamps):
                gps_idx = len(gps_timestamps) - 1
            elif gps_idx > 0:
                # Select nearest by time
                if abs(gps_timestamps[gps_idx - 1] - t) < abs(
                    gps_timestamps[gps_idx] - t
                ):
                    gps_idx = gps_idx - 1

            # Check GPS speed (motion required for reliable heading)
            if gps_idx < len(gps_speeds) and gps_speeds[gps_idx] >= min_gps_speed:
                # Position correction
                gps_x_current = x_gps[gps_idx]
                gps_y_current = y_gps[gps_idx]

                dx_error = gps_x_current - x_current
                dy_error = gps_y_current - y_current
                position_error = np.sqrt(dx_error ** 2 + dy_error ** 2)

                # ===== CHECK 1: Adaptive gain based on error =====
                # ==================================================
                # Strategy: do NOT reject correction, adapt gain instead
                #
                # Error 0-20m:   gain = 100% (full correction)
                # Error 20-50m:  gain = 50%  (half)
                # Error 50-100m: gain = 20%  (small correction)
                # Error >200m:   gain = 0%   (skip - GPS anomaly)

                max_acceptable_error = 200.0  # meters (GPS anomaly threshold)

                if position_error > max_acceptable_error:
                    # Abnormally large error (>200m) - GPS clearly failing
                    gps_corrections_skipped_position += 1
                else:
                    # Adaptive gain: smooth reduction as error grows
                    if position_error < 20.0:
                        adaptive_position_gain = gps_position_gain  # 100%
                    elif position_error < 50.0:
                        adaptive_position_gain = gps_position_gain * 0.5  # 50%
                        gps_corrections_reduced_gain += 1
                    else:  # 50-200m
                        adaptive_position_gain = gps_position_gain * 0.2  # 20%
                        gps_corrections_reduced_gain += 1

                    # Soft position correction
                    x_current += adaptive_position_gain * dx_error
                    y_current += adaptive_position_gain * dy_error

                    # ===== CHECK 2: Yaw correction only during smooth motion =====
                    # =============================================================
                    # Compute current angular velocity (magnitude)
                    current_yaw_rate = abs(yaw_rate_fusion)
                    max_yaw_rate_for_gps_correction = 0.3  # rad/s ≈ 17°/s

                    # If maneuver is too sharp (parking, U-turn):
                    #   - yaw_rate > 0.3 rad/s (17°/s)
                    #   → Do not correct yaw (trust IMU + Steering)
                    if current_yaw_rate < max_yaw_rate_for_gps_correction:
                        # Yaw correction (from GPS motion direction)
                        # Use several GPS points to compute heading
                        if gps_idx >= 2 and gps_idx < len(x_gps) - 2:
                            # Use 5 points to reduce GPS noise
                            gps_dx = x_gps[gps_idx + 1] - x_gps[gps_idx - 1]
                            gps_dy = y_gps[gps_idx + 1] - y_gps[gps_idx - 1]

                            # Minimum displacement for reliable heading
                            if gps_dx ** 2 + gps_dy ** 2 > 4.0:  # > 2 meters
                                gps_heading = np.arctan2(gps_dy, gps_dx)
                                yaw_error = normalize_angle(gps_heading - yaw)

                                # ===== CHECK 3: Yaw error not too large =====
                                # =============================================
                                # If error > 45°, possibly:
                                #   - Sharp turn while parking
                                #   - GPS noise
                                # → Reduce gain or skip
                                max_acceptable_yaw_error = np.radians(45)  # 45°

                                if abs(yaw_error) > max_acceptable_yaw_error:
                                    # Yaw error too large - use small gain
                                    adaptive_yaw_gain = gps_yaw_gain * 0.2
                                    gps_corrections_reduced_gain += 1
                                else:
                                    adaptive_yaw_gain = gps_yaw_gain

                                # Soft yaw correction
                                yaw += adaptive_yaw_gain * yaw_error
                                yaw = normalize_angle(yaw)
                    else:
                        # Sharp maneuver - skip yaw correction
                        gps_corrections_skipped_yaw_rate += 1

                    last_gps_correction_time = t
                    gps_corrections_count += 1

        x_arr[result_idx] = x_current
        y_arr[result_idx] = y_current
        result_idx += 1

    print(f"GPS Fusion trajectory: {result_idx} points")
    print(f"  ✅ GPS corrections applied: {gps_corrections_count}")
    print(f"  🔽 Corrections with reduced gain: {gps_corrections_reduced_gain}")
    print(f"  ⚠️  Skipped (anomaly >200m): {gps_corrections_skipped_position}")
    print(
        f"  ⚠️  Skipped yaw (sharp maneuver >17°/s): {gps_corrections_skipped_yaw_rate}"
    )
    print(
        f"  IMU weight={alpha_imu:.1%}, Steering weight={(1-alpha_imu):.1%}, GPS correction={gps_position_gain:.1%}"
    )

    return x_arr[:result_idx], y_arr[:result_idx]
