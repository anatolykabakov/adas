#!/usr/bin/env python3
"""Bag / sim localization helpers (IMU warm-up, trajectory HUD).

Algorithm path: ``AdasApp`` publish gps/imu/chassis → step → pop_messages(LocalizationPose).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from pyadas import core as pyadas

from .imu_utils import (
    calculate_rotation_matrix_from_gravity,
    calibrate_gyro_bias,
    detect_phone_orientation,
    transform_gyro_to_vehicle_frame,
)


class OnlineImuProcessor:
    """Calibrate gyro online from low-speed samples, then emit vehicle yaw_rate."""

    def __init__(
        self,
        *,
        speed_threshold_orientation: float = 0.1,
        speed_threshold_bias: float = 0.5,
        min_orient_samples: int = 50,
        min_bias_samples: int = 50,
        invert_yaw_rate: bool = True,
    ):
        self.speed_threshold_orientation = float(speed_threshold_orientation)
        self.speed_threshold_bias = float(speed_threshold_bias)
        self.min_orient_samples = int(min_orient_samples)
        self.min_bias_samples = int(min_bias_samples)
        self.invert_yaw_rate = bool(invert_yaw_rate)

        self._speed_mps = 0.0
        self._orient_buf: List[np.ndarray] = []
        self._bias_buf: List[np.ndarray] = []
        self.ready = False
        self.bias: Optional[np.ndarray] = None
        self.rotation_matrix: Optional[np.ndarray] = None
        self.orientation_info: Optional[dict] = None

    def set_speed(self, speed_mps: float) -> None:
        self._speed_mps = float(speed_mps)

    def reset_calibration(self) -> None:
        self._orient_buf.clear()
        self._bias_buf.clear()
        self.ready = False
        self.bias = None
        self.rotation_matrix = None
        self.orientation_info = None

    def _try_finalize(self) -> None:
        if self.ready:
            return
        if (
            len(self._orient_buf) < self.min_orient_samples
            or len(self._bias_buf) < self.min_bias_samples
        ):
            return
        orient = np.stack(self._orient_buf, axis=0)
        bias_src = np.stack(self._bias_buf, axis=0)
        n_o = orient.shape[0]
        n_b = bias_src.shape[0]
        imu_o = np.concatenate(
            [np.arange(n_o, dtype=np.float64)[:, None], orient], axis=1
        )
        imu_b = np.concatenate(
            [np.arange(n_b, dtype=np.float64)[:, None], bias_src], axis=1
        )
        self.orientation_info = detect_phone_orientation(imu_o)
        self.bias, _ = calibrate_gyro_bias(imu_b)
        self.rotation_matrix = calculate_rotation_matrix_from_gravity(
            self.orientation_info["gravity_vector"]
        )
        self.ready = True

    def push(
        self,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float,
        mx: float = 0.0,
        my: float = 0.0,
        mz: float = 0.0,
    ) -> Optional[float]:
        sample9 = np.array([ax, ay, az, gx, gy, gz, mx, my, mz], dtype=np.float64)
        if not self.ready:
            if self._speed_mps < self.speed_threshold_orientation:
                self._orient_buf.append(sample9.copy())
            if self._speed_mps < self.speed_threshold_bias:
                self._bias_buf.append(sample9.copy())
            self._try_finalize()
            if not self.ready:
                return None

        assert self.bias is not None and self.rotation_matrix is not None
        g_cal = np.array([gx, gy, gz], dtype=np.float64) - self.bias
        _, _, gz_v = transform_gyro_to_vehicle_frame(
            float(g_cal[0]), float(g_cal[1]), float(g_cal[2]), self.rotation_matrix
        )
        if self.invert_yaw_rate:
            gz_v = -float(gz_v)
        return float(gz_v)

    def process_batch(
        self,
        imu_data: np.ndarray,
        speeds_mps: np.ndarray,
    ) -> dict:
        self.reset_calibration()
        n = len(imu_data)
        yaw = np.full(n, np.nan, dtype=np.float64)
        for i in range(n):
            self.set_speed(float(speeds_mps[i]) if i < len(speeds_mps) else 0.0)
            row = imu_data[i]
            yr = self.push(
                float(row[1]),
                float(row[2]),
                float(row[3]),
                float(row[4]),
                float(row[5]),
                float(row[6]),
                float(row[7]) if row.shape[0] > 7 else 0.0,
                float(row[8]) if row.shape[0] > 8 else 0.0,
                float(row[9]) if row.shape[0] > 9 else 0.0,
            )
            if yr is not None:
                yaw[i] = yr
        valid = np.isfinite(yaw)
        if np.any(valid):
            first = float(yaw[np.argmax(valid)])
            yaw = np.where(valid, yaw, first)
        else:
            yaw = np.zeros(n, dtype=np.float64)
        return {
            "yaw_rate": yaw,
            "imu_calibrated": imu_data,
            "bias": self.bias,
            "rotation_matrix": self.rotation_matrix,
            "orientation_info": self.orientation_info,
            "ready": self.ready,
        }


@dataclass
class TrajectoryBuffers:
    """World-frame polylines for compare / HUD."""

    ref_x: List[float] = field(default_factory=list)
    ref_y: List[float] = field(default_factory=list)
    odom_x: List[float] = field(default_factory=list)
    odom_y: List[float] = field(default_factory=list)
    ekf_x: List[float] = field(default_factory=list)
    ekf_y: List[float] = field(default_factory=list)

    @property
    def gt_x(self) -> List[float]:
        return self.ref_x

    @property
    def gt_y(self) -> List[float]:
        return self.ref_y

    def clear(self) -> None:
        self.ref_x.clear()
        self.ref_y.clear()
        self.odom_x.clear()
        self.odom_y.clear()
        self.ekf_x.clear()
        self.ekf_y.clear()


class OnlineVehicleEkf:
    """MetaDrive helper: GT pose as GPS over Simulated ``AdasApp``."""

    def __init__(
        self,
        *,
        wheelbase: float = 2.636,
        gps_noise_pos: float = 0.5,
        gps_update_interval: float = 0.2,
        gps_meas_noise: float = 0.0,
        imu_every_step: bool = True,
        app=None,
        **_ignored,
    ):
        _ = imu_every_step
        self.wheelbase = float(wheelbase)
        self.gps_meas_noise = float(gps_meas_noise)
        self._owns_app = app is None
        self._app = app or pyadas.AdasApp(
            wheelbase=float(wheelbase),
            gps_noise_pos=float(gps_noise_pos),
            gps_update_interval=float(gps_update_interval),
        )
        self.buffers = TrajectoryBuffers()
        self._initialized = False
        self._t_us = 0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def reset(
        self,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        v: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> None:
        self._app.reset_localization(
            float(x), float(y), float(yaw), float(v), float(yaw_rate)
        )
        self.buffers.clear()
        self._record(float(x), float(y), float(x), float(y), float(x), float(y))
        self._x, self._y, self._yaw = float(x), float(y), float(yaw)
        self._initialized = True

    def _record(
        self,
        ref_x: float,
        ref_y: float,
        odom_x: float,
        odom_y: float,
        ekf_x: float,
        ekf_y: float,
    ) -> None:
        self.buffers.ref_x.append(ref_x)
        self.buffers.ref_y.append(ref_y)
        self.buffers.odom_x.append(odom_x)
        self.buffers.odom_y.append(odom_y)
        self.buffers.ekf_x.append(ekf_x)
        self.buffers.ekf_y.append(ekf_y)

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def yaw(self) -> float:
        return self._yaw

    def step(
        self,
        *,
        gt_x: float,
        gt_y: float,
        gt_yaw: float,
        speed_mps: float,
        steer_rad: float,
        yaw_rate: float,
        dt: float,
    ) -> None:
        if not self._initialized:
            self.reset(gt_x, gt_y, gt_yaw, v=speed_mps, yaw_rate=yaw_rate)
        if self.buffers.ref_x:
            dx = gt_x - self.buffers.ref_x[-1]
            dy = gt_y - self.buffers.ref_y[-1]
            if dx * dx + dy * dy > 25.0:
                self.reset(gt_x, gt_y, gt_yaw, v=speed_mps, yaw_rate=yaw_rate)

        gps_x, gps_y = float(gt_x), float(gt_y)
        if self.gps_meas_noise > 0:
            gps_x += float(np.random.normal(0.0, self.gps_meas_noise))
            gps_y += float(np.random.normal(0.0, self.gps_meas_noise))

        self._t_us += max(1, int(round(float(dt) * 1e6)))
        self._app.publish_gps(self._t_us, gps_x, gps_y)
        self._app.publish_imu(self._t_us, float(yaw_rate))
        self._app.publish_chassis(
            self._t_us, float(speed_mps), float(steer_rad), float(yaw_rate)
        )
        self._app.step(self._t_us)

        pose = None
        for msg in self._app.pop_messages():
            if isinstance(msg, pyadas.LocalizationPose):
                pose = msg
        if pose is None:
            return
        self._x, self._y, self._yaw = float(pose.x), float(pose.y), float(pose.yaw)
        self._record(
            float(gt_x),
            float(gt_y),
            float(pose.odom_x),
            float(pose.odom_y),
            float(pose.ekf_x),
            float(pose.ekf_y),
        )

    def position_errors(self) -> Tuple[float, float]:
        b = self.buffers
        if len(b.ref_x) < 2:
            return float("nan"), float("nan")
        ref = np.stack([b.ref_x, b.ref_y], axis=1)
        ekf = np.stack([b.ekf_x, b.ekf_y], axis=1)
        odom = np.stack([b.odom_x, b.odom_y], axis=1)
        e_ekf = float(np.sqrt(np.mean(np.sum((ekf - ref) ** 2, axis=1))))
        e_odom = float(np.sqrt(np.mean(np.sum((odom - ref) ** 2, axis=1))))
        return e_ekf, e_odom


def draw_trajectory_panel(
    buffers: TrajectoryBuffers,
    *,
    size: int = 480,
    margin: int = 40,
    title: str = "Trajectory  ref / Odom / EKF",
    ref_label: str = "GT",
) -> np.ndarray:
    """Top-down world plot (bag / sim HUD)."""
    canvas = np.full((size, size, 3), 30, dtype=np.uint8)
    cv2.putText(
        canvas,
        title,
        (8, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )

    series: List[Tuple[str, Sequence[float], Sequence[float], Tuple[int, int, int]]] = [
        (ref_label, buffers.ref_x, buffers.ref_y, (255, 120, 50)),
        ("Odom", buffers.odom_x, buffers.odom_y, (80, 200, 80)),
        ("EKF", buffers.ekf_x, buffers.ekf_y, (0, 140, 255)),
    ]

    xs_all: List[float] = []
    ys_all: List[float] = []
    for _, xs, ys, _ in series:
        if len(xs) >= 1:
            xs_all.extend(xs)
            ys_all.extend(ys)
    if len(xs_all) < 1:
        return canvas

    xmin, xmax = float(min(xs_all)), float(max(xs_all))
    ymin, ymax = float(min(ys_all)), float(max(ys_all))
    span = max(xmax - xmin, ymax - ymin, 5.0)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    scale = (size - 2 * margin) / span

    def to_px(x: float, y: float) -> Tuple[int, int]:
        u = int(round((x - cx) * scale + size * 0.5))
        v = int(round(size * 0.5 - (y - cy) * scale))
        return u, v

    o = to_px(0.0, 0.0)
    cv2.line(canvas, (0, o[1]), (size, o[1]), (50, 50, 50), 1)
    cv2.line(canvas, (o[0], 0), (o[0], size), (50, 50, 50), 1)

    legend_y = 36
    for name, xs, ys, color in series:
        if len(xs) < 2:
            continue
        pts = np.array(
            [to_px(float(x), float(y)) for x, y in zip(xs, ys)], dtype=np.int32
        )
        cv2.polylines(canvas, [pts], False, color, 2, cv2.LINE_AA)
        cv2.circle(canvas, tuple(pts[-1]), 5, color, -1, cv2.LINE_AA)
        cv2.putText(
            canvas,
            name,
            (8, legend_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            color,
            1,
            cv2.LINE_AA,
        )
        legend_y += 16

    if buffers.ref_x:
        cv2.circle(
            canvas,
            to_px(buffers.ref_x[0], buffers.ref_y[0]),
            6,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )
    return canvas
