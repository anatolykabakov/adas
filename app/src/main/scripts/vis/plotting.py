#!/usr/bin/env python3
"""
Functions for visualizing vehicle trajectories.
"""

import numpy as np
import matplotlib.pyplot as plt
import os
from typing import Optional


def plot_trajectory(x_gps, y_gps, x_pose=None, y_pose=None, output_dir="plots"):
    """Trajectory plot of what the bag holds: raw GNSS fixes and the pose the device published.

    Odometry, IMU, fusion and an offline EKF were drawn here too, all re-derived in Python. They are gone
    on purpose — a reconstruction next to a recording invites reading the reconstruction's defects as the
    car's, which is exactly what happened with the offline EKF and the accuracy it never passed on.
    """
    series = []
    if x_gps is not None and len(x_gps) > 0:
        series.append(
            ("ГНСС (сырые фиксы)", np.asarray(x_gps), np.asarray(y_gps), "red", "--", 1.6)
        )
    if x_pose is not None and len(x_pose) > 0:
        series.append(
            ("Поза (из бега)", np.asarray(x_pose), np.asarray(y_pose), "orange", "-", 2.2)
        )
    if not series:
        print("Warning: neither GNSS nor pose available for the trajectory plot")
        return

    os.makedirs(output_dir, exist_ok=True)
    print("\n" + "=" * 60)
    print("TRAJECTORY")
    print("=" * 60)
    for name, x, y, _c, _ls, _lw in series:
        dist = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
        print(
            f"{name}: {len(x)} точек, путь {dist:.1f} м, "
            f"старт ({x[0]:.1f}, {y[0]:.1f}), финиш ({x[-1]:.1f}, {y[-1]:.1f})"
        )

    plt.style.use("default")
    plt.figure(figsize=(12, 10))
    for name, x, y, color, ls, lw in series:
        plt.plot(x, y, color=color, linestyle=ls, linewidth=lw, alpha=0.85, label=name)
    x0, y0 = series[0][1][0], series[0][2][0]
    plt.scatter([x0], [y0], c="green", s=90, zorder=5, label="старт")
    plt.gca().set_aspect("equal")
    plt.grid(alpha=0.25, linewidth=0.5)
    plt.xlabel("восток, м")
    plt.ylabel("север, м")
    plt.title("Траектория из бега")
    plt.legend(loc="best")
    out = os.path.join(output_dir, "trajectory.png")
    plt.savefig(out, dpi=130, bbox_inches="tight")
    plt.close()
