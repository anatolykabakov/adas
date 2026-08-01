#!/usr/bin/env python3
"""
Functions for visualizing vehicle trajectories.
"""

import numpy as np
import matplotlib.pyplot as plt
import os
from typing import Optional


def plot_trajectory(
    x_odom,
    y_odom,
    x_gps=None,
    y_gps=None,
    x_imu=None,
    y_imu=None,
    x_fusion=None,
    y_fusion=None,
    x_gps_fusion=None,
    y_gps_fusion=None,
    x_ekf=None,
    y_ekf=None,
    output_dir="plots",
):
    """
    Plot the vehicle motion trajectory.
    Displays odometry, IMU (gyroscope), Fusion, and GPS (if available).

    Args:
        x_odom: List of X coordinates from odometry (steering)
        y_odom: List of Y coordinates from odometry (steering)
        x_gps: List of X coordinates from GPS (optional)
        y_gps: List of Y coordinates from GPS (optional)
        x_imu: List of X coordinates from IMU (gyroscope) (optional)
        y_imu: List of Y coordinates from IMU (gyroscope) (optional)
        x_fusion: List of X coordinates from Fusion (IMU + Steering) (optional)
        y_fusion: List of Y coordinates from Fusion (IMU + Steering) (optional)
        x_gps_fusion: List of X coordinates from GPS Fusion (IMU + Steering + GPS) (optional)
        y_gps_fusion: List of Y coordinates from GPS Fusion (IMU + Steering + GPS) (optional)
        x_ekf: List of X coordinates from EKF (Extended Kalman Filter) (optional)
        y_ekf: List of Y coordinates from EKF (Extended Kalman Filter) (optional)
        output_dir: Directory for saving plots
    """
    # Check for empty data
    if not x_odom or not y_odom or len(x_odom) == 0:
        print("Warning: No odometry data available for trajectory plotting")
        return

    # Create output directory for plots
    os.makedirs(output_dir, exist_ok=True)

    # Configure matplotlib
    plt.style.use("default")
    fig_size = (14, 10)

    # Convert lists to numpy arrays
    x_odom_array = np.array(x_odom)
    y_odom_array = np.array(y_odom)

    # ===== TRAJECTORY STATISTICS =====
    print("\n" + "=" * 60)
    print("📊 TRAJECTORY STATISTICS:")
    print("=" * 60)

    print(f"\n🔵 Odometry (Steering):")
    print(f"   Points: {len(x_odom_array)}")
    print(
        f"   X: [{x_odom_array.min():.2f}, {x_odom_array.max():.2f}] m, range={x_odom_array.max()-x_odom_array.min():.2f} m"
    )
    print(
        f"   Y: [{y_odom_array.min():.2f}, {y_odom_array.max():.2f}] m, range={y_odom_array.max()-y_odom_array.min():.2f} m"
    )
    print(f"   Start: ({x_odom_array[0]:.2f}, {y_odom_array[0]:.2f})")
    print(f"   End:   ({x_odom_array[-1]:.2f}, {y_odom_array[-1]:.2f})")
    distance_odom = np.sum(
        np.sqrt(np.diff(x_odom_array) ** 2 + np.diff(y_odom_array) ** 2)
    )
    print(f"   Distance traveled: {distance_odom:.2f} m")

    if x_imu is not None and y_imu is not None and len(x_imu) > 0:
        x_imu_array = np.array(x_imu)
        y_imu_array = np.array(y_imu)
        print(f"\n🟢 IMU (Gyroscope):")
        print(f"   Points: {len(x_imu_array)}")
        print(
            f"   X: [{x_imu_array.min():.2f}, {x_imu_array.max():.2f}] m, range={x_imu_array.max()-x_imu_array.min():.2f} m"
        )
        print(
            f"   Y: [{y_imu_array.min():.2f}, {y_imu_array.max():.2f}] m, range={y_imu_array.max()-y_imu_array.min():.2f} m"
        )
        print(f"   Start: ({x_imu_array[0]:.2f}, {y_imu_array[0]:.2f})")
        print(f"   End:   ({x_imu_array[-1]:.2f}, {y_imu_array[-1]:.2f})")
        distance_imu = np.sum(
            np.sqrt(np.diff(x_imu_array) ** 2 + np.diff(y_imu_array) ** 2)
        )
        print(f"   Distance traveled: {distance_imu:.2f} m")
        print(
            f"   Difference from odometry: {distance_imu - distance_odom:.2f} m ({(distance_imu/distance_odom - 1)*100:.1f}%)"
        )

    if x_fusion is not None and y_fusion is not None and len(x_fusion) > 0:
        x_fusion_array = np.array(x_fusion)
        y_fusion_array = np.array(y_fusion)
        print(f"\n🟣 Fusion (IMU + Steering):")
        print(f"   Points: {len(x_fusion_array)}")
        print(
            f"   X: [{x_fusion_array.min():.2f}, {x_fusion_array.max():.2f}] m, range={x_fusion_array.max()-x_fusion_array.min():.2f} m"
        )
        print(
            f"   Y: [{y_fusion_array.min():.2f}, {y_fusion_array.max():.2f}] m, range={y_fusion_array.max()-y_fusion_array.min():.2f} m"
        )
        print(f"   Start: ({x_fusion_array[0]:.2f}, {y_fusion_array[0]:.2f})")
        print(f"   End:   ({x_fusion_array[-1]:.2f}, {y_fusion_array[-1]:.2f})")
        distance_fusion = np.sum(
            np.sqrt(np.diff(x_fusion_array) ** 2 + np.diff(y_fusion_array) ** 2)
        )
        print(f"   Distance traveled: {distance_fusion:.2f} m")
        print(
            f"   Difference from odometry: {distance_fusion - distance_odom:.2f} m ({(distance_fusion/distance_odom - 1)*100:.1f}%)"
        )

    if x_gps_fusion is not None and y_gps_fusion is not None and len(x_gps_fusion) > 0:
        x_gps_fusion_array = np.array(x_gps_fusion)
        y_gps_fusion_array = np.array(y_gps_fusion)
        print(f"\n🟠 GPS Fusion (IMU + Steering + GPS correction):")
        print(f"   Points: {len(x_gps_fusion_array)}")
        print(
            f"   X: [{x_gps_fusion_array.min():.2f}, {x_gps_fusion_array.max():.2f}] m, range={x_gps_fusion_array.max()-x_gps_fusion_array.min():.2f} m"
        )
        print(
            f"   Y: [{y_gps_fusion_array.min():.2f}, {y_gps_fusion_array.max():.2f}] m, range={y_gps_fusion_array.max()-y_gps_fusion_array.min():.2f} m"
        )
        print(f"   Start: ({x_gps_fusion_array[0]:.2f}, {y_gps_fusion_array[0]:.2f})")
        print(f"   End:   ({x_gps_fusion_array[-1]:.2f}, {y_gps_fusion_array[-1]:.2f})")
        distance_gps_fusion = np.sum(
            np.sqrt(np.diff(x_gps_fusion_array) ** 2 + np.diff(y_gps_fusion_array) ** 2)
        )
        print(f"   Distance traveled: {distance_gps_fusion:.2f} m")
        print(
            f"   Difference from odometry: {distance_gps_fusion - distance_odom:.2f} m ({(distance_gps_fusion/distance_odom - 1)*100:.1f}%)"
        )

    if x_ekf is not None and y_ekf is not None and len(x_ekf) > 0:
        x_ekf_array = np.array(x_ekf)
        y_ekf_array = np.array(y_ekf)
        print(f"\n⭐ EKF (Extended Kalman Filter):")
        print(f"   Points: {len(x_ekf_array)}")
        print(
            f"   X: [{x_ekf_array.min():.2f}, {x_ekf_array.max():.2f}] m, range={x_ekf_array.max()-x_ekf_array.min():.2f} m"
        )
        print(
            f"   Y: [{y_ekf_array.min():.2f}, {y_ekf_array.max():.2f}] m, range={y_ekf_array.max()-y_ekf_array.min():.2f} m"
        )
        print(f"   Start: ({x_ekf_array[0]:.2f}, {y_ekf_array[0]:.2f})")
        print(f"   End:   ({x_ekf_array[-1]:.2f}, {y_ekf_array[-1]:.2f})")
        distance_ekf = np.sum(
            np.sqrt(np.diff(x_ekf_array) ** 2 + np.diff(y_ekf_array) ** 2)
        )
        print(f"   Distance traveled: {distance_ekf:.2f} m")
        print(
            f"   Difference from odometry: {distance_ekf - distance_odom:.2f} m ({(distance_ekf/distance_odom - 1)*100:.1f}%)"
        )

    if x_gps is not None and y_gps is not None and len(x_gps) > 0:
        x_gps_array = np.array(x_gps)
        y_gps_array = np.array(y_gps)
        print(f"\n🔴 GPS (Ground Truth):")
        print(f"   Points: {len(x_gps_array)}")
        print(
            f"   X: [{x_gps_array.min():.2f}, {x_gps_array.max():.2f}] m, range={x_gps_array.max()-x_gps_array.min():.2f} m"
        )
        print(
            f"   Y: [{y_gps_array.min():.2f}, {y_gps_array.max():.2f}] m, range={y_gps_array.max()-y_gps_array.min():.2f} m"
        )
        print(f"   Start: ({x_gps_array[0]:.2f}, {y_gps_array[0]:.2f})")
        print(f"   End:   ({x_gps_array[-1]:.2f}, {y_gps_array[-1]:.2f})")
        distance_gps = np.sum(
            np.sqrt(np.diff(x_gps_array) ** 2 + np.diff(y_gps_array) ** 2)
        )
        print(f"   Distance traveled: {distance_gps:.2f} m")

    print("\n" + "=" * 60)

    # Create figure
    plt.figure(figsize=fig_size)

    # Odometry trajectory (steering)
    plt.plot(
        x_odom_array,
        y_odom_array,
        "b-",
        linewidth=2,
        alpha=0.7,
        label="Odometry (Steering)",
    )
    plt.scatter(
        x_odom_array[0],
        y_odom_array[0],
        color="green",
        s=150,
        label="Start",
        zorder=5,
        marker="o",
    )
    plt.scatter(
        x_odom_array[-1],
        y_odom_array[-1],
        color="red",
        s=150,
        label="End",
        zorder=5,
        marker="s",
    )

    # IMU trajectory (gyroscope) (if available)
    if x_imu is not None and y_imu is not None and len(x_imu) > 0:
        x_imu_array = np.array(x_imu)
        y_imu_array = np.array(y_imu)
        plt.plot(
            x_imu_array,
            y_imu_array,
            "g-",
            linewidth=2,
            alpha=0.6,
            label="IMU (Gyroscope)",
        )
        plt.scatter(
            x_imu_array[0], y_imu_array[0], color="green", s=100, zorder=4, marker="D"
        )
        plt.scatter(
            x_imu_array[-1], y_imu_array[-1], color="red", s=100, zorder=4, marker="D"
        )

    # Fusion trajectory (IMU + Steering) (if available)
    if x_fusion is not None and y_fusion is not None and len(x_fusion) > 0:
        x_fusion_array = np.array(x_fusion)
        y_fusion_array = np.array(y_fusion)
        plt.plot(
            x_fusion_array,
            y_fusion_array,
            "m-",
            linewidth=2,
            alpha=0.6,
            label="Fusion (IMU+Steering)",
            zorder=2,
        )
        plt.scatter(
            x_fusion_array[0],
            y_fusion_array[0],
            color="green",
            s=100,
            zorder=4,
            marker="*",
        )
        plt.scatter(
            x_fusion_array[-1],
            y_fusion_array[-1],
            color="red",
            s=100,
            zorder=4,
            marker="*",
        )

    # GPS Fusion trajectory (IMU + Steering + GPS correction) (if available)
    if x_gps_fusion is not None and y_gps_fusion is not None and len(x_gps_fusion) > 0:
        x_gps_fusion_array = np.array(x_gps_fusion)
        y_gps_fusion_array = np.array(y_gps_fusion)
        plt.plot(
            x_gps_fusion_array,
            y_gps_fusion_array,
            "orange",
            linewidth=2,
            alpha=0.7,
            label="GPS Fusion (IMU+Str+GPS)",
            zorder=3,
        )
        plt.scatter(
            x_gps_fusion_array[0],
            y_gps_fusion_array[0],
            color="green",
            s=120,
            zorder=5,
            marker="P",
        )
        plt.scatter(
            x_gps_fusion_array[-1],
            y_gps_fusion_array[-1],
            color="red",
            s=120,
            zorder=5,
            marker="P",
        )

    # EKF trajectory (Extended Kalman Filter) (if available) ⭐ BEST
    if x_ekf is not None and y_ekf is not None and len(x_ekf) > 0:
        x_ekf_array = np.array(x_ekf)
        y_ekf_array = np.array(y_ekf)
        plt.plot(
            x_ekf_array,
            y_ekf_array,
            color="cyan",
            linewidth=3.5,
            alpha=1.0,
            label="EKF (Kalman Filter)",
            zorder=6,
        )
        plt.scatter(
            x_ekf_array[0],
            y_ekf_array[0],
            color="cyan",
            s=180,
            zorder=7,
            marker="*",
            edgecolors="black",
            linewidths=2,
        )
        plt.scatter(
            x_ekf_array[-1],
            y_ekf_array[-1],
            color="cyan",
            s=180,
            zorder=7,
            marker="*",
            edgecolors="black",
            linewidths=2,
        )

    # GPS trajectory (if available)
    if x_gps is not None and y_gps is not None and len(x_gps) > 0:
        x_gps_array = np.array(x_gps)
        y_gps_array = np.array(y_gps)
        plt.plot(
            x_gps_array,
            y_gps_array,
            "r--",
            linewidth=2,
            alpha=0.6,
            label="GPS (Ground Truth)",
        )
        plt.scatter(
            x_gps_array[0], y_gps_array[0], color="green", s=100, zorder=4, marker="^"
        )
        plt.scatter(
            x_gps_array[-1], y_gps_array[-1], color="red", s=100, zorder=4, marker="v"
        )

    plt.xlabel("X (m)", fontsize=12)
    plt.ylabel("Y (m)", fontsize=12)
    plt.title("Vehicle Motion Trajectory", fontsize=14, fontweight="bold")
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3, linestyle="--")
    plt.axis("equal")

    # Save plot
    output_path = os.path.join(output_dir, "trajectory.png")
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Plot saved: {output_path}")
