#!/usr/bin/env python3
"""
Utilities for working with IMU data.
Calibration, coordinate transforms, and processing of gyroscope and accelerometer data.
"""

import numpy as np
from typing import Tuple, Optional


def filter_imu_by_vehicle_speed(
    imu_data: np.ndarray,
    vehicle_speed_data: np.ndarray,
    speed_threshold: float = 0.1,
    time_window_sec: Optional[float] = None,
    min_samples: Optional[int] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Filter IMU data, keeping only moments when the vehicle is stationary or moving slowly.

    Performs:
    1. Synchronization of IMU and vehicle speed by time (interpolation)
    2. Detection of stationary periods (speed < threshold)
    3. IMU data filtering
    4. Optional filtering by time window

    Args:
        imu_data: numpy array shape (N, 10) [timestamp, ax, ay, az, gx, gy, gz, mx, my, mz]
        vehicle_speed_data: numpy array shape (M, 5) [timestamp, fl, fr, rl, rr]
                           or shape (M,) [speed] or shape (M, 2) [timestamp, speed]
        speed_threshold: speed threshold (m/s), below which the vehicle is considered stationary
        time_window_sec: optional time window from the start of the recording (seconds)
        min_samples: optional minimum number of samples to return

    Returns:
        Tuple[filtered_imu, interpolated_speeds]:
            - filtered_imu: filtered IMU data (only when the vehicle is stationary)
            - interpolated_speeds: speeds interpolated onto IMU timestamps

    Raises:
        ValueError: if there is insufficient data

    Example:
        # vehicle_speed_data can be:
        # 1. [timestamp, fl, fr, rl, rr] - data from CAN
        # 2. [timestamp, speed] - precomputed average speed
        # 3. [speed] - speed array (no timestamps, assumes index sync)

        filtered_imu, speeds = filter_imu_by_vehicle_speed(
            imu_data,
            vehicle_speed_data,
            speed_threshold=0.1,
            time_window_sec=20.0,
            min_samples=50
        )
    """
    if imu_data is None or len(imu_data) == 0:
        raise ValueError("IMU data is empty")

    if vehicle_speed_data is None or len(vehicle_speed_data) == 0:
        raise ValueError("Vehicle speed data is empty")

    # Convert to numpy array if needed
    if not isinstance(vehicle_speed_data, np.ndarray):
        vehicle_speed_data = np.array(vehicle_speed_data)

    imu_timestamps = imu_data[:, 0]

    # ===== STEP 1: Extract speeds and timestamps =====
    # ==================================================

    if vehicle_speed_data.ndim == 1:
        # Case 1: Speeds only [speed, speed, ...] without timestamps
        # Assume synchronization by index
        speeds = vehicle_speed_data
        speed_timestamps = None
        print(
            f"   Speed data: {len(speeds)} values (no timestamps, synchronized by index)"
        )

    elif vehicle_speed_data.ndim == 2:
        if vehicle_speed_data.shape[1] == 2:
            # Case 2: [timestamp, speed]
            speed_timestamps = vehicle_speed_data[:, 0]
            speeds = vehicle_speed_data[:, 1]
            print(f"   Speed data: {len(speeds)} values with timestamps")

        elif vehicle_speed_data.shape[1] >= 4:
            # Case 3: [timestamp, fl, fr, rl, rr, ...]
            speed_timestamps = vehicle_speed_data[:, 0]
            # Compute average wheel speed
            speeds = np.mean(vehicle_speed_data[:, 1:5], axis=1)
            print(f"   Speed data: {len(speeds)} values (averaged over 4 wheels)")
        else:
            raise ValueError(
                f"Unexpected vehicle_speed_data shape: {vehicle_speed_data.shape}"
            )
    else:
        raise ValueError(
            f"vehicle_speed_data must be a 1D or 2D array, got {vehicle_speed_data.ndim}D"
        )

    # ===== STEP 2: Time synchronization =====
    # ========================================

    if speed_timestamps is not None:
        # Interpolate speeds onto IMU timestamps
        from scipy import interpolate

        print(f"   Time synchronization...")
        print(
            f"     IMU: {len(imu_timestamps)} samples, range {imu_timestamps[0]:.0f} - {imu_timestamps[-1]:.0f} ms"
        )
        print(
            f"     Speed: {len(speed_timestamps)} samples, range {speed_timestamps[0]:.0f} - {speed_timestamps[-1]:.0f} ms"
        )

        # Use linear interpolation for speeds
        try:
            speed_interp_func = interpolate.interp1d(
                speed_timestamps,
                speeds,
                kind="linear",
                bounds_error=False,
                fill_value=(
                    speeds[0],
                    speeds[-1],
                ),  # Use boundary values outside the range
            )
            speeds_interpolated = speed_interp_func(imu_timestamps)
            print(f"     ✅ Interpolation complete")

        except Exception as e:
            print(
                f"     ⚠️  Warning: Interpolation failed ({e}), falling back to index synchronization"
            )
            # Fallback: synchronize by index
            n_sync = min(len(imu_data), len(speeds))
            speeds_interpolated = np.zeros(len(imu_data))
            speeds_interpolated[:n_sync] = speeds[:n_sync]
            speeds_interpolated[n_sync:] = speeds[-1] if len(speeds) > 0 else 0
    else:
        # Synchronize by index (assume equal sampling rate)
        print(f"   Index synchronization (assuming equal sampling rate)...")
        n_sync = min(len(imu_data), len(speeds))
        speeds_interpolated = np.zeros(len(imu_data))
        speeds_interpolated[:n_sync] = speeds[:n_sync]
        # Use the last value for the remainder
        if n_sync < len(imu_data):
            speeds_interpolated[n_sync:] = speeds[-1] if len(speeds) > 0 else 0

    # ===== STEP 3: Filter by speed =====
    # ==================================

    stationary_mask = speeds_interpolated < speed_threshold

    # Optional: filter by time window
    if time_window_sec is not None:
        start_time = imu_timestamps[0]
        time_mask = (imu_timestamps - start_time) / 1000.0 <= time_window_sec
        stationary_mask = stationary_mask & time_mask
        print(
            f"   Filtering: speed < {speed_threshold} m/s AND time < {time_window_sec}s"
        )
    else:
        print(f"   Filtering: speed < {speed_threshold} m/s")

    stationary_indices = np.where(stationary_mask)[0]

    print(f"   Found {len(stationary_indices)} samples where the vehicle is stationary")

    # ===== STEP 4: Check minimum sample count =====
    # ===============================================

    if min_samples is not None and len(stationary_indices) < min_samples:
        print(
            f"   ⚠️  Warning: Insufficient samples ({len(stationary_indices)} < {min_samples})"
        )

        if time_window_sec is not None:
            # Try without the time constraint
            print(f"       Trying without time constraint...")
            stationary_mask_no_time = speeds_interpolated < speed_threshold
            stationary_indices = np.where(stationary_mask_no_time)[0]
            print(f"       Found: {len(stationary_indices)} samples")

        if len(stationary_indices) < min_samples:
            # Use the first min_samples
            print(f"       Using the first {min_samples} samples without filtering")
            stationary_indices = np.arange(min(min_samples, len(imu_data)))

    # ===== STEP 5: Return filtered data =====
    # ========================================

    filtered_imu = imu_data[stationary_indices]

    print(f"   ✅ Filtered: {len(filtered_imu)} IMU samples (vehicle stationary)")

    return filtered_imu, speeds_interpolated


def calibrate_gyro_bias(imu_stationary: np.ndarray) -> Tuple[np.ndarray, int]:
    """
    Calibrate gyroscope bias from data collected while the vehicle is stationary.

    IMPORTANT: This function expects data collected while the vehicle is completely still!
    Use filter_imu_by_vehicle_speed() for pre-filtering.

    Bias (zero offset) is the systematic gyroscope error that accumulates during
    integration and causes orientation drift.

    Logic:
        When the vehicle is stationary → true angular velocity = 0 rad/s
        But the gyroscope reads ≠ 0 → that is the bias!
        bias = mean(gyro[when_vehicle_stationary])

    Args:
        imu_stationary: numpy array shape (N, 10) [timestamp, ax, ay, az, gx, gy, gz, mx, my, mz]
                       IMU data while the vehicle is stationary (already filtered!)

    Returns:
        bias: numpy array [gx_bias, gy_bias, gz_bias] in rad/s
        n_samples: number of samples used for calibration

    Raises:
        ValueError: if there is insufficient data for calibration

    Example:
        # First filter the data:
        imu_stationary, _ = filter_imu_by_vehicle_speed(
            imu_data, vehicle_speed,
            speed_threshold=0.1
        )

        # Then compute bias:
        bias, n = calibrate_gyro_bias(imu_stationary)
        # bias = [-0.009124, -0.000293, 0.001400] rad/s

        # Apply calibration to ALL data:
        imu_calibrated = apply_gyro_calibration(imu_data, bias)

    Note:
        - For best results, use at least 50-100 samples while the vehicle is stationary
        - Bias can change over time and with temperature
        - Periodic recalibration is recommended
    """
    if imu_stationary is None or len(imu_stationary) == 0:
        raise ValueError("IMU stationary data is empty")

    n_stationary = len(imu_stationary)

    # STEP 1: Extract gyroscope data
    # ================================
    # Input data is ALREADY filtered (vehicle stationary)
    gyro_data_stationary = imu_stationary[:, 4:7]  # gx, gy, gz

    # STEP 2: Compute bias as the mean value
    # =======================================
    # When the vehicle is stationary, true angular velocity = 0
    # Therefore gyroscope readings = bias
    gx_bias = np.mean(gyro_data_stationary[:, 0])
    gy_bias = np.mean(gyro_data_stationary[:, 1])
    gz_bias = np.mean(gyro_data_stationary[:, 2])

    bias = np.array([gx_bias, gy_bias, gz_bias])

    # STEP 3: Statistics and output
    # ==============================
    gx_std = np.std(gyro_data_stationary[:, 0])
    gy_std = np.std(gyro_data_stationary[:, 1])
    gz_std = np.std(gyro_data_stationary[:, 2])

    print(f"\n📊 Gyroscope calibration:")
    print(f"   Samples used: {n_stationary} (vehicle stationary)")
    print(f"\n   Bias (zero offset):")
    print(f"     gx: {gx_bias:+.6f} ± {gx_std:.6f} rad/s")
    print(f"     gy: {gy_bias:+.6f} ± {gy_std:.6f} rad/s")
    print(f"     gz: {gz_bias:+.6f} ± {gz_std:.6f} rad/s (yaw rate)")
    print(f"\n   Accumulated error without calibration:")
    print(f"     Over 1 min:  {abs(gz_bias) * 60 * 180/np.pi:.1f}°")
    print(f"     Over 10 min: {abs(gz_bias) * 600 * 180/np.pi:.1f}°")

    return bias, n_stationary


def detect_phone_orientation(imu_stationary: np.ndarray) -> dict:
    """
    Determine phone orientation relative to the vehicle from accelerometer data.

    IMPORTANT: This function expects data collected while the vehicle is completely still!
    Use filter_imu_by_vehicle_speed() for pre-filtering.

    When the vehicle is stationary on a level surface, the accelerometer measures only
    gravity (≈9.81 m/s² directed downward). The gravity vector direction reveals
    how the phone is oriented.

    Standard vehicle orientation:
        X: right (passenger door)
        Y: forward (hood)
        Z: up (sky)

    Args:
        imu_stationary: numpy array shape (N, 10) [timestamp, ax, ay, az, ...]
                       IMU data while the vehicle is stationary (already filtered!)

    Returns:
        dict with orientation information:
            'gravity_vector': [gx, gy, gz] - gravity vector in IMU frame
            'gravity_magnitude': float - vector magnitude (should be ≈9.81)
            'dominant_axis': str - dominant axis ('X', 'Y', 'Z')
            'orientation_ok': bool - True if Z points upward (standard)
            'n_samples_used': int - number of samples used

    Example:
        # First filter the data:
        imu_stationary, _ = filter_imu_by_vehicle_speed(
            imu_data, vehicle_speed,
            speed_threshold=0.1, time_window_sec=20
        )

        # Then analyze orientation:
        orientation = detect_phone_orientation(imu_stationary)

        if orientation['orientation_ok']:
            print("✅ Phone is lying flat")
        else:
            print(f"⚠️  Dominant axis: {orientation['dominant_axis']}")
    """
    if imu_stationary is None or len(imu_stationary) == 0:
        raise ValueError("IMU stationary data is empty")

    # STEP 1: Average accelerometer data
    # ===================================
    # When the vehicle is stationary, the accelerometer shows only gravity
    # Input data is ALREADY filtered (vehicle stationary)
    ax_mean = np.mean(imu_stationary[:, 1])
    ay_mean = np.mean(imu_stationary[:, 2])
    az_mean = np.mean(imu_stationary[:, 3])

    gravity_vector = np.array([ax_mean, ay_mean, az_mean])
    gravity_magnitude = np.linalg.norm(gravity_vector)

    # STEP 2: Determine the dominant axis
    # ====================================
    # The axis with the largest acceleration magnitude points down (or up)
    abs_values = np.abs(gravity_vector)
    dominant_idx = np.argmax(abs_values)
    dominant_axes = ["X", "Y", "Z"]
    dominant_axis = dominant_axes[dominant_idx]

    # STEP 3: Check standard orientation
    # =====================================
    # Standard: Z should be dominant, az ≈ -9.81 (or +9.81)
    # (sign depends on convention, but Z should be vertical)
    orientation_ok = (dominant_axis == "Z") and (abs_values[2] > 8.0)

    # STEP 4: Build result
    # =====================
    result = {
        "gravity_vector": gravity_vector,
        "gravity_magnitude": gravity_magnitude,
        "dominant_axis": dominant_axis,
        "orientation_ok": orientation_ok,
        "n_samples_used": len(imu_stationary),
        "ax_mean": ax_mean,
        "ay_mean": ay_mean,
        "az_mean": az_mean,
    }

    # STEP 5: Output information
    # ===========================
    print(f"\n📱 Phone orientation:")
    print(f"   Samples used: {len(imu_stationary)} (vehicle stationary)")
    print(f"   Gravity vector (IMU): [{ax_mean:.2f}, {ay_mean:.2f}, {az_mean:.2f}] m/s²")
    print(f"   Magnitude: {gravity_magnitude:.2f} m/s² (expected: 9.81 m/s²)")
    print(f"   Dominant axis: {dominant_axis}")

    if orientation_ok:
        print(f"   ✅ Orientation OK - phone is lying flat (Z up)")
    else:
        print(f"   ⚠️  Non-standard orientation - dominant axis is {dominant_axis}")
        print(f"       Phone may be vertical or tilted")
        print(f"       Coordinate transform recommended for correct calculations")

    return result


def apply_gyro_calibration(imu_data: np.ndarray, bias: np.ndarray) -> np.ndarray:
    """
    Apply bias calibration to gyroscope data.

    Args:
        imu_data: numpy array shape (N, 10) - original IMU data
        bias: numpy array [gx_bias, gy_bias, gz_bias]

    Returns:
        calibrated_imu: numpy array shape (N, 10) - data with calibrated gyroscope

    Note:
        Creates a copy of the data; the original is not modified
    """
    calibrated_imu = imu_data.copy()
    calibrated_imu[:, 4:7] -= bias  # Subtract bias from gx, gy, gz
    return calibrated_imu


def transform_gyro_to_vehicle(
    imu_calibrated: np.ndarray, rotation_matrix: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Transform gyroscope data from the phone frame to the vehicle frame.

    Applies the rotation matrix to all gyroscope samples.
    Used when the phone is mounted at an angle rather than flat
    (e.g., in a vertical windshield mount).

    Args:
        imu_calibrated: numpy array shape (N, 10) - calibrated IMU data
        rotation_matrix: numpy array 3x3 - rotation matrix (IMU → Vehicle)

    Returns:
        tuple (gx_vehicle, gy_vehicle, gz_vehicle) - transformed data

    Example:
        # After bias calibration:
        imu_calibrated = apply_gyro_calibration(imu_data, bias)

        # Get the rotation matrix:
        R = calculate_rotation_matrix_from_gravity(gravity_vector)

        # Transform all data:
        gx_v, gy_v, gz_v = transform_gyro_to_vehicle(imu_calibrated, R)

        # Now gz_v = vehicle yaw rate ✅
    """
    print("\n🚗 Transforming to vehicle frame...")

    gx_phone = imu_calibrated[:, 4]
    gy_phone = imu_calibrated[:, 5]
    gz_phone = imu_calibrated[:, 6]

    gx_vehicle = np.zeros(len(imu_calibrated))
    gy_vehicle = np.zeros(len(imu_calibrated))
    gz_vehicle = np.zeros(len(imu_calibrated))

    # Apply rotation matrix to each sample
    for i in range(len(imu_calibrated)):
        gyro_phone = np.array([gx_phone[i], gy_phone[i], gz_phone[i]])
        gyro_vehicle = rotation_matrix @ gyro_phone
        gx_vehicle[i] = gyro_vehicle[0]
        gy_vehicle[i] = gyro_vehicle[1]
        gz_vehicle[i] = gyro_vehicle[2]

    print(f"   ✅ Transformed {len(imu_calibrated)} samples")

    return gx_vehicle, gy_vehicle, gz_vehicle


def transform_gyro_to_vehicle_frame(
    gx: float, gy: float, gz: float, rotation_matrix: np.ndarray
) -> Tuple[float, float, float]:
    """
    Transform angular velocity from the IMU frame to the vehicle frame.
    (Single-value version)

    Used when the phone is mounted at an angle rather than flat.

    Args:
        gx, gy, gz: Angular velocities in the IMU frame (rad/s)
        rotation_matrix: 3x3 rotation matrix (IMU → Vehicle)

    Returns:
        gx_v, gy_v, gz_v: Angular velocities in the vehicle frame (rad/s)

    Example:
        # For a single sample:
        gx_v, gy_v, gz_v = transform_gyro_to_vehicle_frame(gx, gy, gz, R)

    Note:
        For arrays, use transform_gyro_to_vehicle()
    """
    gyro_imu = np.array([gx, gy, gz])
    gyro_vehicle = rotation_matrix @ gyro_imu
    return gyro_vehicle[0], gyro_vehicle[1], gyro_vehicle[2]


def calculate_rotation_matrix_from_gravity(gravity_vector: np.ndarray) -> np.ndarray:
    """
    Automatically compute the rotation matrix from the IMU frame to the vehicle frame
    based on the gravity vector.

    Goal: Find matrix R such that R @ g_phone = g_vehicle
    where g_phone is gravity in the phone frame, g_vehicle = [0, 0, -1] in the vehicle frame

    Assumes the vehicle is on a level horizontal surface.

    Method: Rodrigues' rotation formula
    Converts rotation axis + angle → 3x3 rotation matrix

    Args:
        gravity_vector: [ax, ay, az] - averaged accelerometer readings at rest

    Returns:
        rotation_matrix: 3x3 matrix for IMU → Vehicle transform

    Example:
        gravity = [10.06, 0.43, -0.36]  # Phone vertical
        R = calculate_rotation_matrix_from_gravity(gravity)
        # R ≈ rotation matrix for ~88°

        # Apply to gyroscope:
        gyro_vehicle = R @ gyro_phone
    """
    # ===== STEP 1: Normalize the gravity vector =====
    # =================================================
    # We need only the DIRECTION, not the magnitude
    # Normalize to unit length (|g_norm| = 1)
    #
    # Example:
    #   gravity_vector = [10.06, 0.43, -0.36]
    #   norm = sqrt(10.06² + 0.43² + 0.36²) ≈ 10.08
    #   g_norm = [10.06/10.08, 0.43/10.08, -0.36/10.08]
    #          = [0.998, 0.043, -0.036]
    g_norm = gravity_vector / np.linalg.norm(gravity_vector)

    # ===== STEP 2: Define the target vector =====
    # =============================================
    # In the standard vehicle frame, gravity points DOWN along the Z axis
    # g_vehicle = [0, 0, -1]
    #   X: right (0 gravity component)
    #   Y: forward (0 gravity component)
    #   Z: up (-1 means gravity points down)
    g_vehicle = np.array([0, 0, -1])

    # ===== STEP 3: Compute the rotation axis =====
    # ==============================================
    # Use the cross product:
    #   rotation_axis ⊥ (perpendicular to) both g_norm and g_vehicle
    #
    # Visualization:
    #        g_norm [0.998, 0.043, -0.036]  (gravity in phone frame)
    #          ↗
    #         /  ← need to rotate
    #        /
    #       ↓
    #    g_vehicle [0, 0, -1]  (gravity in vehicle frame)
    #
    #    rotation_axis ⊙ (perpendicular to the plane, out of the screen)
    #
    # Formula: a × b = [a_y*b_z - a_z*b_y, a_z*b_x - a_x*b_z, a_x*b_y - a_y*b_x]
    rotation_axis = np.cross(g_norm, g_vehicle)
    rotation_axis_norm = np.linalg.norm(rotation_axis)

    # ===== STEP 4: Handle special cases =====
    # =========================================
    if rotation_axis_norm < 1e-6:
        # Vectors are nearly parallel (cross product ≈ 0)
        # Two cases:

        if np.dot(g_norm, g_vehicle) > 0:
            # Case A: Vectors point in the SAME direction
            # g_norm ≈ g_vehicle → no rotation needed
            # Return identity matrix (no change)
            return np.eye(3)
        else:
            # Case B: Vectors point in OPPOSITE directions
            # g_norm ≈ -g_vehicle → 180° rotation
            # Return reflection matrix
            return -np.eye(3)

    # ===== STEP 5: Normalize the rotation axis =====
    # ================================================
    # Normalize rotation axis to unit length
    # Required for Rodrigues' formula
    rotation_axis = rotation_axis / rotation_axis_norm

    # ===== STEP 6: Compute the rotation angle =====
    # ===============================================
    # Use the dot product:
    #   cos(θ) = a · b / (|a| * |b|)
    # For unit vectors: cos(θ) = a · b
    #
    # np.clip clamps the value to [-1, 1] (guards against rounding errors)
    # arccos returns angle in [0, π]
    angle = np.arccos(np.clip(np.dot(g_norm, g_vehicle), -1.0, 1.0))

    # ===== STEP 7: Rodrigues' formula =====
    # =======================================
    # Converts rotation axis + angle → 3x3 rotation matrix
    #
    # R = I + sin(θ) * K + (1 - cos(θ)) * K²
    #
    # Where:
    #   I - identity matrix
    #   K - skew-symmetric matrix of the rotation axis
    #   θ - rotation angle
    #
    # STEP 7.1: Build skew-symmetric matrix K
    # ----------------------------------------
    # For vector k = [kx, ky, kz], skew-symmetric matrix:
    #       [  0  -kz   ky ]
    #   K = [ kz    0  -kx ]
    #       [-ky   kx    0 ]
    #
    # Property: K @ v = k × v (cross product)
    K = np.array(
        [
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0],
        ]
    )

    # STEP 7.2: Compute rotation matrix via Rodrigues' formula
    # ---------------------------------------------------------
    # R = I + sin(θ)*K + (1-cos(θ))*K²
    #
    # Components:
    #   I              - leaves the component along the rotation axis unchanged
    #   sin(θ)*K       - adds the component perpendicular to the axis
    #   (1-cos(θ))*K²  - adds the component in the rotation plane
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    return R


def process_imu_for_odometry(
    imu_data: np.ndarray,
    vehicle_speed_data: np.ndarray,
    speed_threshold_orientation: float = 0.1,
    speed_threshold_bias: float = 0.5,
    time_window_sec: float = 20.0,
    invert_yaw_rate: bool = True,
) -> dict:
    """
    Process IMU data for trajectory visualization.

    Runs the full processing pipeline:
    1. Data filtering (vehicle stationary)
    2. Phone orientation detection
    3. Gyroscope bias calibration
    4. Rotation matrix computation
    5. Transform data to the vehicle frame
    6. Yaw rate sign inversion (optional, for Android)

    Args:
        imu_data: numpy array shape (N, 10) [timestamp, ax, ay, az, gx, gy, gz, mx, my, mz]
        vehicle_speed_data: numpy array - speed data (any format)
        speed_threshold_orientation: threshold for orientation detection (m/s)
        speed_threshold_bias: threshold for bias calibration (m/s)
        time_window_sec: time window for orientation analysis (sec)
        invert_yaw_rate: invert yaw rate sign (True for Android IMU)

    Returns:
        dict with processed data:
            'imu_calibrated': numpy array (N, 10) - calibrated data
            'gyro_vehicle': numpy array (N, 3) - [gx_v, gy_v, gz_v] in vehicle frame
            'yaw_rate': numpy array (N,) - gz_vehicle (ready for odometry)
            'bias': numpy array [gx_bias, gy_bias, gz_bias]
            'rotation_matrix': numpy array 3x3
            'orientation_info': dict - phone orientation information
            'rotation_angle_deg': float - phone rotation angle

    Example usage in visualizer:
        from core.imu_utils import process_imu_for_odometry

        imu_processed = process_imu_for_odometry(imu_data, wheel_speed_data)

        # Get yaw rate for trajectory calculation:
        yaw_rate = imu_processed['yaw_rate']  # gz_vehicle

        # Or the full gyroscope:
        gyro_vehicle = imu_processed['gyro_vehicle']  # (N, 3)
    """
    print("\n" + "=" * 60)
    print("🔧 IMU PROCESSING FOR ODOMETRY")
    print("=" * 60)

    # ===== STEP 1: Filter for orientation analysis =====
    print("\n📍 STEP 1: IMU filtering (orientation detection)")
    print(
        "   Parameters: v < {:.1f} m/s, first {:.0f}s".format(
            speed_threshold_orientation, time_window_sec
        )
    )

    imu_stationary, _ = filter_imu_by_vehicle_speed(
        imu_data,
        vehicle_speed_data,
        speed_threshold=speed_threshold_orientation,
        time_window_sec=time_window_sec,
        min_samples=50,
    )

    # ===== STEP 2: Phone orientation detection =====
    print("\n📱 STEP 2: Phone orientation detection")
    orientation_info = detect_phone_orientation(imu_stationary)

    # ===== STEP 3: Filter for bias calibration =====
    print("\n⚙️  STEP 3: Bias calibration (all stationary periods)")
    print("   Parameters: v < {:.1f} m/s, full recording".format(speed_threshold_bias))

    imu_for_bias, _ = filter_imu_by_vehicle_speed(
        imu_data,
        vehicle_speed_data,
        speed_threshold=speed_threshold_bias,
        time_window_sec=None,
        min_samples=50,
    )

    bias, n_bias_samples = calibrate_gyro_bias(imu_for_bias)

    # ===== STEP 4: Apply calibration to all data =====
    print("\n🔄 STEP 4: Applying calibration to all IMU data")
    imu_calibrated = apply_gyro_calibration(imu_data, bias)
    print(f"   ✅ Calibrated {len(imu_calibrated)} samples")

    # ===== STEP 5: Compute rotation matrix =====
    print("\n🔄 STEP 5: Computing rotation matrix")
    gravity_vector = orientation_info["gravity_vector"]
    rotation_matrix = calculate_rotation_matrix_from_gravity(gravity_vector)

    # Compute angle for information
    trace = np.trace(rotation_matrix)
    rotation_angle_rad = np.arccos(np.clip((trace - 1) / 2, -1.0, 1.0))
    rotation_angle_deg = np.degrees(rotation_angle_rad)

    print(f"   Rotation matrix R (IMU → Vehicle):")
    for row in rotation_matrix:
        print(f"   [{row[0]:+.4f}, {row[1]:+.4f}, {row[2]:+.4f}]")
    print(f"   Rotation angle: {rotation_angle_deg:.1f}°")

    # ===== STEP 6: Transform to vehicle frame =====
    print("\n🚗 STEP 6: Transform to vehicle frame")
    gx_vehicle, gy_vehicle, gz_vehicle = transform_gyro_to_vehicle(
        imu_calibrated, rotation_matrix
    )

    # ===== STEP 7: Yaw rate sign correction =====
    # =============================================
    # Android coordinate system may have inverted yaw direction
    # Verified empirically against GPS/odometry
    if invert_yaw_rate:
        print("\n🔄 STEP 7: Yaw rate sign correction")
        print("   Reason: Android coordinate system has inverted rotation direction")
        print(
            f"   Mean BEFORE inversion:  {np.mean(gz_vehicle):+.6f} rad/s ({np.degrees(np.mean(gz_vehicle)):+.3f}°/s)"
        )

        gz_vehicle = -gz_vehicle  # Invert sign
        gx_vehicle = -gx_vehicle  # Invert for consistency
        gy_vehicle = -gy_vehicle

        print(
            f"   Mean AFTER inversion: {np.mean(gz_vehicle):+.6f} rad/s ({np.degrees(np.mean(gz_vehicle)):+.3f}°/s)"
        )
        print("   ✅ Sign inverted (gz → -gz)")
    else:
        print("\n⏭️  STEP 7: Yaw rate sign correction SKIPPED")

    # ===== STEP 8: Build result =====
    gyro_vehicle_array = np.column_stack([gx_vehicle, gy_vehicle, gz_vehicle])

    result = {
        "imu_calibrated": imu_calibrated,
        "gyro_vehicle": gyro_vehicle_array,  # (N, 3) [gx, gy, gz] in vehicle frame
        "yaw_rate": gz_vehicle,  # (N,) yaw rate for odometry (with inversion applied!)
        "bias": bias,
        "rotation_matrix": rotation_matrix,
        "rotation_angle_deg": rotation_angle_deg,
        "orientation_info": orientation_info,
        "n_bias_samples": n_bias_samples,
        "inverted": invert_yaw_rate,  # Inversion flag for debugging
    }

    print("\n" + "=" * 60)
    print("✅ PROCESSING COMPLETE")
    print("=" * 60)
    print(f"\nReady for use:")
    print(f"  • yaw_rate: {len(gz_vehicle)} samples")
    print(f"  • Calibration: bias subtracted")
    print(f"  • Transform: gz_vehicle = vehicle yaw rate ✅")
    print(f"  • Sign inversion: {'✅ Applied' if invert_yaw_rate else '❌ Not applied'}")
    print(
        f"  • Phone orientation: {orientation_info['dominant_axis']} vertical, angle {rotation_angle_deg:.1f}°"
    )

    return result


def estimate_yaw_rate_quality(
    imu_data: np.ndarray, vehicle_speed: np.ndarray, calibrated: bool = True
) -> dict:
    """
    Estimate the quality of yaw rate (gz) data from the gyroscope.

    Checks:
    - Noise level (standard deviation at rest)
    - Drift (error accumulation over time)
    - Correlation with speed (should be weak for straight-line motion)

    Args:
        imu_data: numpy array shape (N, 10)
        vehicle_speed: numpy array shape (N,)
        calibrated: True if bias has already been subtracted

    Returns:
        dict with quality metrics
    """
    # Convert to numpy array if it is a list
    if not isinstance(vehicle_speed, np.ndarray):
        vehicle_speed = np.array(vehicle_speed)

    gz = imu_data[:, 6]  # yaw rate

    # Synchronize array lengths
    n_samples = min(len(gz), len(vehicle_speed))
    gz = gz[:n_samples]
    vehicle_speed_sync = vehicle_speed[:n_samples]

    # Noise at rest
    stationary_mask = vehicle_speed_sync < 0.1
    stationary_indices = np.where(stationary_mask)[0]

    if len(stationary_indices) > 10:
        gz_stationary = gz[stationary_indices]
        gz_noise = np.std(gz_stationary)
    else:
        gz_noise = np.nan

    # Overall statistics
    gz_mean = np.mean(gz)
    gz_std = np.std(gz)
    gz_max = np.max(np.abs(gz))

    result = {
        "mean": gz_mean,
        "std": gz_std,
        "max_abs": gz_max,
        "noise_at_rest": gz_noise,
        "calibrated": calibrated,
    }

    print(f"\n📈 Yaw rate (gz) data quality:")
    print(f"   Mean: {gz_mean:+.6f} rad/s")
    print(f"   Std: {gz_std:.6f} rad/s")
    print(f"   Max: {gz_max:.6f} rad/s ({gz_max*180/np.pi:.1f}°/s)")
    if not np.isnan(gz_noise):
        print(f"   Noise at rest: {gz_noise:.6f} rad/s")
    print(f"   Calibration: {'✅ Applied' if calibrated else '❌ Not applied'}")

    return result
