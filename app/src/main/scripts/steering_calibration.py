#!/usr/bin/env python3
"""
Steering Wheel Calibration Module

Two-stage steering wheel calibration:
1. STRAIGHT - find steering zero point
2. ROTATION - find turn range and coefficients
"""

import yaml
import os
import math
from collections import Counter


class ConfigManager:
    """Helper for configuration files"""

    def __init__(self, config_path: str):
        self.config_path = config_path

    def load_config(self):
        """Load configuration from file"""
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r") as f:
                    return yaml.safe_load(f) or {}
            except:
                pass

        return {"cars": {"steering": {}}}

    def save_config(self, config):
        """Save configuration to file"""
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        with open(self.config_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False)

    def update_steering_config(self, **kwargs):
        """Update steering parameters in configuration"""
        config = self.load_config()

        if "cars" not in config:
            config["cars"] = {}
        if "steering" not in config["cars"]:
            config["cars"]["steering"] = {}

        for key, value in kwargs.items():
            config["cars"]["steering"][key] = value

        self.save_config(config)


class SteeringCalibrator:
    """Steering wheel calibrator"""

    def __init__(self, base_length: float, min_rotation_radius: float):
        """
        Args:
            base_length: Vehicle wheelbase in meters
            min_rotation_radius: Minimum turn radius in meters
        """
        self.base_length = base_length
        self.min_rotation_radius = min_rotation_radius
        self.steering_data = []

        angle_rad = math.atan2(base_length, min_rotation_radius)
        self.max_wheel_angle = math.degrees(angle_rad)

    def push(self, value):
        """Add one steering angle measurement"""
        self.steering_data.append(value)

    def calibrate_zero(self):
        """Zero point calibration"""
        if not self.steering_data:
            return 0.0

        rounded_data = [round(x) for x in self.steering_data]
        counter = Counter(rounded_data)
        return float(counter.most_common(1)[0][0])

    def calibrate_steer(self):
        """Turn range calibration"""
        if not self.steering_data:
            return (0.0, 0.0)

        left_limit = max(self.steering_data)
        right_limit = min(self.steering_data)

        left_coeff = self.max_wheel_angle / abs(left_limit) if abs(left_limit) > 0 else 0
        right_coeff = (
            self.max_wheel_angle / abs(right_limit) if abs(right_limit) > 0 else 0
        )

        return (left_coeff, right_coeff)


def main():
    """Usage example"""
    base_length = 2.63
    min_rotation_radius = 5.55
    config_path = "/workspace/app/src/main/scripts/steering_calibration.yml"

    print("=== Steering wheel calibration ===")

    calibrator = SteeringCalibrator(base_length, min_rotation_radius)
    config_manager = ConfigManager(config_path)

    print("\n--- Stage 1: Zero point calibration ---")
    straight_data = [0, 1, -1, 0, 2, -2, 0, 1, -1, 0] * 10
    for value in straight_data:
        calibrator.push(value)

    zero_point = calibrator.calibrate_zero()
    config_manager.update_steering_config(zero_point=zero_point)
    print(f"Zero point: {zero_point}")

    print("\n--- Stage 2: Turn range calibration ---")
    calibrator.steering_data = []

    rotation_data = list(range(-500, 501, 10)) + list(range(500, -501, -10))
    for value in rotation_data:
        calibrator.push(value)

    left_coeff, right_coeff = calibrator.calibrate_steer()
    max_left = max(calibrator.steering_data) - 10
    max_right = min(calibrator.steering_data) + 10

    config_manager.update_steering_config(
        max_left=max_left,
        max_right=max_right,
        left_wheel={"turn_left_1_unit": left_coeff, "turn_right_1_unit": right_coeff},
        right_wheel={"turn_left_1_unit": left_coeff, "turn_right_1_unit": right_coeff},
    )

    print(f"Limits: left={max_left:.1f}, right={max_right:.1f}")
    print(f"Coefficients: {left_coeff:.6f}, {right_coeff:.6f}")
    print(f"\nResults saved to: {config_path}")


if __name__ == "__main__":
    main()
