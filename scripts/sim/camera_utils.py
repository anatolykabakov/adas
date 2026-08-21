#! /usr/bin/env python3
"""MetaDrive camera intrinsics / extrinsics → AAD overlay parameters."""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import numpy as np

try:
    from metadrive.constants import DEFAULT_SENSOR_HPR, DEFAULT_SENSOR_OFFSET
except ImportError:  # pragma: no cover
    DEFAULT_SENSOR_OFFSET = (0.0, 0.8, 1.5)
    DEFAULT_SENSOR_HPR = (0.0, 0.0, 0.0)

# Windshield mount used when MetaDrive HPR is identity (see CameraParams.mount_on_agent).
# ISO: offset panda (X right, Y forward, Z up) → height=z, cam_x=y.
METADRIVE_MOUNT_OFFSET = tuple(DEFAULT_SENSOR_OFFSET)  # (0, 0.8, 1.5)
METADRIVE_MOUNT_HPR_DEG = (0.0, 0.59681, 0.0)  # heading, pitch look-up, roll
METADRIVE_MOUNT_HEIGHT_M = float(METADRIVE_MOUNT_OFFSET[2])  # 1.5
METADRIVE_MOUNT_CAM_X_M = float(METADRIVE_MOUNT_OFFSET[1])  # 0.8
METADRIVE_MOUNT_PITCH_DEG = float(METADRIVE_MOUNT_HPR_DEG[1])  # ≈0.6


class ZupRightHandCS:
    def __init__(self):
        self.R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

    def transform(self, x, y, z, yaw_deg, pitch_deg, roll_deg):
        T = np.eye(4)
        T[:3, :3] = self.rotation_zyx(yaw_deg, pitch_deg, roll_deg)
        T[:3, 3] = np.array([x, y, z])
        return T

    def rotation_zyx(self, yaw_deg, pitch_deg, roll_deg):
        return (
            self.rotation_z(yaw_deg)
            @ self.rotation_y(pitch_deg)
            @ self.rotation_x(roll_deg)
        )

    def rotation_x(self, angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        return np.array(
            [
                [1, 0, 0],
                [0, np.cos(angle_rad), -np.sin(angle_rad)],
                [0, np.sin(angle_rad), np.cos(angle_rad)],
            ]
        )

    def rotation_y(self, angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        return np.array(
            [
                [np.cos(angle_rad), 0, np.sin(angle_rad)],
                [0, 1, 0],
                [-np.sin(angle_rad), 0, np.cos(angle_rad)],
            ]
        )

    def rotation_z(self, angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        return np.array(
            [
                [np.cos(angle_rad), -np.sin(angle_rad), 0],
                [np.sin(angle_rad), np.cos(angle_rad), 0],
                [0, 0, 1],
            ]
        )


def panda_vehicle_to_iso(xyz_panda: np.ndarray) -> np.ndarray:
    """Panda vehicle frame (X right, Y forward, Z up) → ISO (X fwd, Y left, Z up)."""
    x_r, y_f, z_u = float(xyz_panda[0]), float(xyz_panda[1]), float(xyz_panda[2])
    return np.array([y_f, -x_r, z_u], dtype=np.float64)


class CameraParams:
    """Read MetaDrive RGB camera K / pose and expose AAD overlay params."""

    def __init__(self, env, sensor_name: str = "rgb"):
        self.env = env
        rgb_camera = env.engine.get_sensor(sensor_name)
        self.sensor = rgb_camera
        self.fov = rgb_camera.lens.getFov()
        self.width = rgb_camera.BUFFER_W
        self.height = rgb_camera.BUFFER_H
        self.intrinsics = self.make_intrinsics(self.fov, self.width, self.height)

        # Ensure camera is mounted on the ego (same as MetaDrive image obs)
        self.mount_on_agent()
        self.refresh_pose()

    def mount_on_agent(
        self,
        offset: Optional[Tuple[float, float, float]] = None,
        hpr: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        """Track RGB camera to ego with MetaDrive default windshield mount."""
        offset = tuple(offset) if offset is not None else tuple(METADRIVE_MOUNT_OFFSET)
        hpr = tuple(hpr) if hpr is not None else tuple(DEFAULT_SENSOR_HPR)
        # MetaDrive sometimes uses a small look-up pitch when switching agents
        if hpr == tuple(DEFAULT_SENSOR_HPR):
            # Keep slight windshield pitch used by base_env.switch_to_next_vehicle
            hpr = METADRIVE_MOUNT_HPR_DEG
        self.mount_offset = np.array(offset, dtype=np.float64)
        self.mount_hpr = np.array(hpr, dtype=np.float64)
        self.sensor.track(self.env.agent.origin, offset, hpr)

    def refresh_pose(self) -> None:
        """Re-read pose relative to vehicle origin (Panda frame).

        The mount offset is measured from ``agent.origin``, and that node is **not on the road** — it
        sits at the chassis reference, 0.51 m up on the default vehicle. Taking its z as the camera
        height (which this did) under-reports the height by exactly that much, and every ground-plane
        projection then compresses distance by the ratio: a point drawn at 10 m is really at 13.4 m,
        and lane edges at ±1.75 m land a metre outside the painted ones. Nothing in the model input
        depends on it (the warp is a rotation), but the overlay, the BEV patch and any comparison of
        model output against simulator ground truth all do.
        """
        rgb_camera = self.sensor
        agent = self.env.agent
        hpr = np.array(rgb_camera.cam.getHpr(agent.origin), dtype=np.float64)
        xyz_panda = np.array(rgb_camera.cam.getPos(agent.origin), dtype=np.float64)
        self.hpr = hpr  # heading, pitch, roll (deg) in Panda vehicle frame
        self.xyz_panda = xyz_panda  # X right, Y forward, Z up
        self.xyz_iso = panda_vehicle_to_iso(xyz_panda)  # X fwd, Y left, Z up
        self.origin_height_m = float(np.array(agent.origin.getPos(), dtype=np.float64)[2])
        self.cam_x = float(self.xyz_iso[0])
        self.cam_y_left = float(self.xyz_iso[1])
        self.height_m = float(self.xyz_iso[2]) + self.origin_height_m
        # The extrinsics are handed out to projectors, so they carry the road-relative height too.
        xyz_road = np.array(
            [self.xyz_iso[0], self.xyz_iso[1], self.height_m], dtype=np.float64
        )
        self.extrinsics = self.make_extrinsics_iso(xyz_road, hpr)

    def aad_overlay_params(self) -> Dict[str, float]:
        """Parameters for ``make_overlay_geometry`` / AAD ``CameraGeometry``.

        MetaDrive mount defaults ≈ offset (0, 0.8, 1.5) panda, pitch ≈ +0.6°
        (slight look-up). AAD: pitch_deg < 0 looks down, so panda pitch maps 1:1
        (positive panda pitch → positive AAD pitch = look up).
        """
        heading, pitch, roll = (float(x) for x in self.hpr)
        return {
            "camera_height": float(self.height_m),
            "cam_x": float(self.cam_x),
            "cam_y_left": float(self.cam_y_left),
            # Panda heading+: left turn; AAD yaw uses opposite sign in road frame
            "yaw_deg": float(-heading),
            "pitch_deg": float(pitch),
            "roll_deg": float(roll),
        }

    def get_fov(self):
        return self.fov

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_intrinsics(self):
        return self.intrinsics

    def get_extrinsics(self):
        return self.extrinsics

    def make_intrinsics(self, fov, width, height):
        f_x = (width / 2) / (np.tan(fov[0] / 2 / 180 * np.pi))
        f_y = (height / 2) / (np.tan(fov[1] / 2 / 180 * np.pi))
        u0 = width / 2
        v0 = height / 2
        return np.array([[f_x, 0, u0], [0, f_y, v0], [0, 0, 1]], dtype=np.float64)

    def make_extrinsics_iso(self, xyz_iso: np.ndarray, hpr: np.ndarray) -> np.ndarray:
        """ISO ego (X fwd, Y left, Z up) → OpenCV camera 4×4.

        Uses vehicle-local mount (not world Z).
        """
        height = float(xyz_iso[2])
        cam_x = float(xyz_iso[0])
        cam_y_left = float(xyz_iso[1])
        # Build pose in ISO then convert axes to OpenCV camera
        # Camera center in ISO: (cam_x, cam_y_left, height)
        # Rotation from HPR in Panda: apply pitch about vehicle lateral
        pitch_deg = float(hpr[1])
        yaw_deg = float(hpr[0])
        roll_deg = float(hpr[2])

        zup = ZupRightHandCS()
        # Translate so camera is at origin looking with R; road points expressed in cam
        # T_iso_cam_from_road: rotate then translate by -R @ C
        R_yaw = zup.rotation_z(yaw_deg)
        R_pitch = zup.rotation_y(pitch_deg)
        R_roll = zup.rotation_x(roll_deg)
        # Vehicle-local ISO rotation (yaw about up, pitch about left→right? use Y-left pitch)
        # Match previous pipeline: pitch about Y (left), yaw about Z (up)
        R_iso = R_yaw @ R_pitch @ R_roll
        C = np.array([cam_x, cam_y_left, height], dtype=np.float64)
        # World(ISO)→camera_ISO (still Xfwd Yleft Zup axes at camera)
        W2C_iso = np.eye(4)
        W2C_iso[:3, :3] = R_iso.T
        W2C_iso[:3, 3] = -R_iso.T @ C

        # ISO camera axes → OpenCV (X right, Y down, Z forward)
        IsoCamToOpenCV = np.array(
            [
                [0, -1, 0, 0],  # Xc = -Y_iso (= right)
                [0, 0, -1, 0],  # Yc = -Z_iso (= down)
                [1, 0, 0, 0],  # Zc =  X_iso (= forward)
                [0, 0, 0, 1],
            ],
            dtype=np.float64,
        )
        return IsoCamToOpenCV @ W2C_iso

    # Back-compat alias used by older make_extrinsics call sites
    def make_extrinsics(self, xyz, hpr):
        xyz = np.asarray(xyz, dtype=np.float64)
        if xyz.shape[0] >= 3 and abs(xyz[2]) > abs(xyz[0]) + 0.5:
            # Likely world position mistaken for local — treat as height-only
            xyz_iso = np.array([0.0, 0.0, float(xyz[2])], dtype=np.float64)
        else:
            xyz_iso = panda_vehicle_to_iso(xyz) if abs(xyz[1]) > abs(xyz[0]) else xyz
        return self.make_extrinsics_iso(xyz_iso, np.asarray(hpr, dtype=np.float64))


class CameraGeometry:
    def __init__(self, K, Rt):
        self.K = K
        self.Rt = Rt

    def xyz_to_uv(self, xyz):
        xyz_homo = np.stack((xyz[:, 0], xyz[:, 1], xyz[:, 2], np.ones_like(xyz[:, 0])))
        uvw = self.K @ self.Rt[:3, :] @ xyz_homo
        u = uvw[0, :] / uvw[2, :]
        v = uvw[1, :] / uvw[2, :]
        return np.stack((u, v)).T

    def project_polyline_iso(
        self,
        poly: np.ndarray,
        *,
        x_min: float = 0.5,
        margin: float = 80.0,
        image_size: tuple[int, int] | None = None,
    ) -> list[tuple[int, int]]:
        """ISO ego polyline (X fwd, Y left[, Z up]) → image pixels. Z defaults to 0 (road)."""
        a = np.asarray(poly, dtype=np.float64)
        if a.ndim != 2 or a.shape[0] < 2 or a.shape[1] < 2:
            return []
        n = a.shape[0]
        xyz = np.zeros((n, 3), dtype=np.float64)
        xyz[:, 0] = a[:, 0]
        xyz[:, 1] = a[:, 1]
        if a.shape[1] >= 3:
            xyz[:, 2] = a[:, 2]
        ok = np.isfinite(xyz).all(axis=1) & (xyz[:, 0] >= x_min)
        if not np.any(ok):
            return []
        xyz = xyz[ok]
        # Front of camera (OpenCV Z forward after extrinsics)
        hom = np.concatenate([xyz, np.ones((xyz.shape[0], 1))], axis=1)
        cam = (self.Rt[:3, :] @ hom.T).T
        front = cam[:, 2] > 0.2
        if not np.any(front):
            return []
        uv = self.xyz_to_uv(xyz[front])
        w = h = None
        if image_size is not None:
            w, h = image_size
        pts: list[tuple[int, int]] = []
        for u, v in uv:
            if w is not None and h is not None:
                if u < -margin or u >= w + margin or v < -margin or v >= h + margin:
                    continue
            pts.append((int(round(u)), int(round(v))))
        return pts

    def uv_to_xyz(self, uv):
        uv_coords = np.stack([uv[:, 0], uv[:, 1], np.ones_like(uv[:, 0])])
        K_inv = np.linalg.inv(self.K)
        xyz_camera = (K_inv @ uv_coords.T).T
        Rt_inv = np.linalg.inv(self.Rt)
        xyz_world = Rt_inv[:3, :] @ xyz_camera
        return xyz_world
