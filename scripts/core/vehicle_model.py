#!/usr/bin/env python3
"""
Vehicle kinematics model for trajectory computation.
"""

import numpy as np
from typing import Tuple


def normalize_angle(angle):
    """
    Normalize angle to [-π, π].

    Args:
        angle: Angle in radians (may be an array)

    Returns:
        Normalized angle in [-π, π]
    """
    while np.any(angle > np.pi):
        angle = np.where(angle > np.pi, angle - 2.0 * np.pi, angle)
    while np.any(angle < -np.pi):
        angle = np.where(angle < -np.pi, angle + 2.0 * np.pi, angle)
    return angle


class VehicleModel:
    """
    Vehicle kinematics model (bicycle model).
    Computes trajectory from wheel speeds, steering angle, and gear.
    """

    def __init__(self, wheelbase: float = 2.636, initial_yaw: float = 0.0):
        """
        Initialize vehicle model.

        Args:
            wheelbase: Wheelbase (axle distance) in meters
            initial_yaw: Initial heading in radians (0 = east, π/2 = north)
        """
        self.wheelbase = wheelbase
        self.min_speed_threshold = 0.01  # m/s
        self.max_steering_angle = 32.72  # degrees
        self.max_steer_raw_value = 400
        self.unit_to_degrees = self.max_steering_angle / self.max_steer_raw_value

        self.x = 0.0
        self.y = 0.0
        self.yaw = initial_yaw
        self.initial_yaw = initial_yaw  # kept for reset()

    def reset(self):
        """Reset vehicle state to initial values"""
        self.x = 0.0
        self.y = 0.0
        self.yaw = self.initial_yaw

    def update(
        self,
        vr: float,
        vl: float,
        hr: float,
        hl: float,
        steering_angle_abs: float,
        steering_angle_sign: float,
        gear_name: str,
        dt: float,
    ) -> Tuple[float, float, float]:
        """
        Update vehicle state from odometry inputs.

        Args:
            vr, vl, hr, hl: Wheel speeds in km/h
            steering_angle_abs: |front wheel angle| in raw units
                (road-wheel deg / unit_to_degrees). Not LWI steering angle!
            steering_angle_sign: Angle sign (0=left, 1=right)
            gear_name: Gear name
            dt: Time step in seconds

        Returns:
            Tuple (x, y, yaw) — current position and heading
        """
        steering_angle_deg = steering_angle_abs * self.unit_to_degrees
        if steering_angle_sign != 0:
            steering_angle_deg = -steering_angle_deg

        steering_angle_deg = np.clip(
            steering_angle_deg, -self.max_steering_angle, self.max_steering_angle
        )
        steering_angle_rad = np.radians(steering_angle_deg)

        v_vehicle_kmh = (vr + vl + hr + hl) / 4.0
        v_vehicle_ms = v_vehicle_kmh / 3.6  # km/h -> m/s

        direction_multiplier = -1.0 if gear_name == "REVERSE" else 1.0
        v_vehicle_ms *= direction_multiplier

        if abs(v_vehicle_ms) <= self.min_speed_threshold:
            return self.x, self.y, self.yaw

        if abs(steering_angle_rad) >= 0.001:
            omega = v_vehicle_ms * np.tan(steering_angle_rad) / self.wheelbase
        else:
            omega = 0.0

        self.yaw += omega * dt
        self.yaw = normalize_angle(self.yaw)
        self.x += v_vehicle_ms * dt * np.cos(self.yaw)
        self.y += v_vehicle_ms * dt * np.sin(self.yaw)

        return self.x, self.y, self.yaw


# --- Measured lateral dynamics Golf 7 MQB (bag 2026_08_01_01_14_22) -------------------
#
# Regression κ_actual = yaw_rate/v vs κ_kin = tan(SWA/ratio)/wheelbase over 19k frames
# (corr 0.98–0.997) shows growing curvature shortfall with speed:
#
#     v, m/s   |  6     10     14     18     22
#     κa/κk    | 0.99  0.93   0.85   0.76   0.61
#
# openpilot VehicleModel shape (1/(1−sf·v²)) fits this at tire_stiffness_factor ≈ 0.64
# (stock 1.0 underestimates shortfall: 0.76 vs 0.61 at 22 m/s).
#
# Check on logged steering: yaw rate prediction RMS 0.32 vs 0.52 °/s kinematic,
# slope 0.91–1.03 across speed bins (kinematic: 0.60–0.98).
MEASURED_SLIP_FACTOR = -0.00099  # 1/(m/s)², matches tire_stiffness_factor ≈ 0.64
YAW_RESPONSE_TAU_S = (
    0.12  # yaw rate lag behind steering angle (cross-correlation measured)
)
STEER_ACTUATOR_TAU_S = 0.04  # command → actual rack angle


def curvature_from_steer(
    steer_wheel_rad: float,
    v_mps: float,
    wheelbase: float = 2.636,
    slip_factor: float = MEASURED_SLIP_FACTOR,
) -> float:
    """Steady-state curvature from wheel angle with understeer shortfall.

    slip_factor=0 gives pure kinematics. Inverse — :func:`steer_from_curvature`.
    """
    return np.tan(steer_wheel_rad) / wheelbase / (1.0 - slip_factor * v_mps * v_mps)


def steer_from_curvature(
    curvature: float,
    v_mps: float,
    wheelbase: float = 2.636,
    slip_factor: float = MEASURED_SLIP_FACTOR,
) -> float:
    """Wheel angle for target curvature (what the controller should command)."""
    return np.arctan(curvature * wheelbase * (1.0 - slip_factor * v_mps * v_mps))


class LateralPlant:
    """Lateral dynamics for closed-loop runs: curvature shortfall + two delays.

    Kinematic model (yaw = v·tan(δ)/L) overstates vehicle response by up to 65% at 22 m/s and
    therefore hides steering shortfall in the controller — this class replaces it.
    """

    def __init__(
        self,
        wheelbase: float = 2.636,
        slip_factor: float = MEASURED_SLIP_FACTOR,
        yaw_tau: float = YAW_RESPONSE_TAU_S,
        actuator_tau: float = STEER_ACTUATOR_TAU_S,
    ):
        self.wheelbase = wheelbase
        self.slip_factor = slip_factor
        self.yaw_tau = max(yaw_tau, 1e-3)
        self.actuator_tau = max(actuator_tau, 1e-3)
        self.delta = 0.0
        self.yaw_rate = 0.0

    def reset(self, delta: float = 0.0, yaw_rate: float = 0.0) -> None:
        self.delta, self.yaw_rate = delta, yaw_rate

    def step(self, delta_cmd: float, v_mps: float, dt: float) -> float:
        """One step: wheel angle command → actual yaw rate [rad/s]."""
        self.delta += (delta_cmd - self.delta) * min(1.0, dt / self.actuator_tau)
        target = (
            curvature_from_steer(self.delta, v_mps, self.wheelbase, self.slip_factor)
            * v_mps
        )
        self.yaw_rate += (target - self.yaw_rate) * min(1.0, dt / self.yaw_tau)
        return self.yaw_rate
