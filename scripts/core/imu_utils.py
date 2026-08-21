#!/usr/bin/env python3
"""
Utilities for working with IMU data.
Calibration, coordinate transforms, and processing of gyroscope and accelerometer data.
"""

import numpy as np
from typing import Tuple, Optional


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
