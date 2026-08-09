#!/usr/bin/env python3
"""Interactive ADAS bag visualizer (Android session layout).

Replay only: everything shown comes out of the bag. No runtime model, no online camera
calibration, no live controller — the camera mounting is read from the session's own
``calib_rpy.json`` (assets priors when it is absent) and never written back.

Uses AndroidBagPlayer + extractors from vis.visualizer.

UI:
  - trajectory (odom / GPS / IMU / EKF) over the OSM road graph
  - Zoom slider: whole route → a ±5 m window, following the timeline position;
    close in it adds the lane markings, the OSM lanes, the model plan and the planner δ
  - every line is a switch: OSM / Surf / Lanes / Names / Road,
    Cam (camera flattened onto the road) / Ego / Marks / Mid (detector lane centre) /
    Plan / Tgt / δ / Inc, and the four trajectories
  - marks the operator dictated during the drive, from the session transcription;
    "◀ inc" / "inc ▶" jump the timeline between them
  - synced camera frame with the logged vision/lanes and control/lane_keep
  - timeline scrub + play/pause

The map is anchored by the first GNSS fix of the run, so a bag without GNSS shows no map.
Lane counts come from the ``.admap.lanes.npz`` sidecar (``python3 -m mapmatch.osm_lanes``).

Usage:
  python3 vis/interactive_visualizer.py /path/to/2026_07_18_09_45_15
  python3 vis/interactive_visualizer.py -i /path/to/session.zip
  python3 vis/interactive_visualizer.py <bag> --osm-map maps/Moscow.osm.admap
"""

from __future__ import annotations

import _path  # noqa: F401

import argparse
import json
from pathlib import Path
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: F401
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.font_manager import FontProperties
from matplotlib.transforms import Affine2D
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from vis.android_bag_player import AndroidBagPlayer
from vis import incidents as incident_marks
from vis.osm_layer import OsmLayer, lane_counts, lane_offsets, transform_to_world
from vis.road_bev import warp_to_road
from core.frames import DRAW_Y_SIGN
from core.gps_utils import calculate_initial_heading_from_gps, gps_to_local_coords
from core.imu_utils import process_imu_for_odometry
from core.lane_projection import (
    CameraIntrinsics,
    intrinsics_from_messages,
)
from core.path_fusion import (
    DEFAULT_CAMERA_OFFSET_M,
    path_bundle_from_bag_lanes,
    plan_orientation_from_lanes,
)
from core.supercombo_compare import draw_bag_lanes, make_overlay_geometry
from vis.trajectory_calculators import (
    calculate_trajectory,
    calculate_trajectory_ekf,
    calculate_trajectory_imu,
)
from core.lane_keep import (
    DEFAULTS,
    LaneKeepResult,
    format_lane_keep_status,
    lane_keep_result_from_bag,
)
from core.lane_keep_viz import draw_lane_keep_overlay
from core.pure_pursuit import draw_steering_wheel
from core.viz_params_ui import load_camera_priors
from vis.visualizer import (
    _UNIT_TO_DEG,
    extract_gps,
    extract_imu,
    extract_vehicle_series,
)


def _nearest_row(series: Optional[np.ndarray], t_ms: float) -> Optional[np.ndarray]:
    """Nearest row in series[:,0] timestamps (ms)."""
    if series is None or len(series) == 0:
        return None
    ts = series[:, 0]
    idx = int(np.searchsorted(ts, t_ms))
    if idx <= 0:
        return series[0]
    if idx >= len(ts):
        return series[-1]
    if abs(ts[idx - 1] - t_ms) <= abs(ts[idx] - t_ms):
        return series[idx - 1]
    return series[idx]


def _format_ms(ms: float) -> str:
    """Seconds from the start of the bag, one decimal.

    Not mm:ss on purpose: the same recording gets opened in PlotJuggler, whose x axis is seconds, and
    a timestamp read here has to be usable there without arithmetic in the head.
    """
    return f"{max(0.0, float(ms) / 1000.0):.1f} s"


class InteractiveVisualizer:
    """Interactive trajectory + camera viewer for ADAS bag sessions."""

    def __init__(
        self,
        master: tk.Tk,
        bag_path: Optional[str] = None,
        osm_map: Optional[str] = None,
    ):
        self.master = master
        self.master.title("ADAS Interactive Visualizer")
        self.master.geometry("1600x950")

        self.player: Optional[AndroidBagPlayer] = None
        self.trajectories = {}
        self.timestamps = np.array([], dtype=np.float64)
        self.current_index = 0

        self.imu_data: Optional[np.ndarray] = None
        self.gps_data: Optional[np.ndarray] = None
        self.wheel_data: Optional[np.ndarray] = None
        self.steering_data: Optional[np.ndarray] = None
        self.gear_data: Optional[np.ndarray] = None
        # (timestamp_ms, jpeg_bytes, optional cam_msg for K)
        self.camera_frames: List[Tuple[int, bytes, Any]] = []
        # (timestamp_ms, LaneLines proto from vision/lanes)
        self.bag_lane_frames: List[Tuple[int, Any]] = []
        # (timestamp_ms, LaneKeepState from control/lane_keep)
        self.bag_lane_keep_frames: List[Tuple[int, Any]] = []
        self.image_timestamps = np.array([], dtype=np.int64)
        self.lane_timestamps = np.array([], dtype=np.int64)
        self.lane_keep_timestamps = np.array([], dtype=np.int64)
        self.intrinsics: Optional[CameraIntrinsics] = None
        self.intr_msg = None
        self.bag_dir: Optional[Path] = None

        # Camera mounting: assets priors, overridden by the session's own calib_rpy.json.
        ap = self._asset_priors = load_camera_priors()
        self._play_last_wall_ms: Optional[float] = None
        self._overlay_pitch_deg = ap.pitch_deg
        self._overlay_yaw_deg = ap.yaw_deg
        self._overlay_roll_deg = ap.roll_deg
        self._overlay_height_m = ap.height_m
        self._overlay_cam_x = ap.cam_x
        self._overlay_cam_y = ap.cam_y_left

        self.playing = False
        self.play_speed = 1.0

        # OSM background + segment zoom
        self.osm = OsmLayer(Path(osm_map) if osm_map else None)
        self.osm_lane_counts: dict = {}
        self._osm_line = None
        self._osm_wide_line = None
        self._osm_texts: List[Any] = []
        self._detail_artists: List[Any] = []
        self.traj_lines: dict = {}
        self._view_busy = False
        self._view_pending = False
        self._map_view: Optional[Tuple[float, float, float, float]] = None
        self._map_span = 0.0
        self._last_view: Optional[Tuple[float, float, float, float]] = None
        self._full_extent: Optional[Tuple[float, float, float, float]] = None
        self.ref_heading: Optional[np.ndarray] = None
        self.incidents: List[Any] = []
        self.lane_blend_scale = 0.0
        self.camera_offset_m = DEFAULT_CAMERA_OFFSET_M

        self.create_ui()
        if bag_path:
            self.load_bag(bag_path)

    def create_ui(self) -> None:
        control = ttk.Frame(self.master)
        control.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        ttk.Button(control, text="Open bag", command=self.open_bag).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(control, text="Play", command=self.play).pack(side=tk.LEFT, padx=5)
        ttk.Button(control, text="Pause", command=self.pause).pack(side=tk.LEFT, padx=5)
        ttk.Button(control, text="Restart", command=self.restart).pack(
            side=tk.LEFT, padx=5
        )
        # A 22-minute run has dozens of dictated marks; scrubbing to them by hand is hopeless.
        ttk.Button(control, text="◀ inc", command=lambda: self.jump_incident(-1)).pack(
            side=tk.LEFT, padx=(12, 2)
        )
        ttk.Button(control, text="inc ▶", command=lambda: self.jump_incident(+1)).pack(
            side=tk.LEFT, padx=(0, 5)
        )

        self.lanes_overlay_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            control,
            text="Lanes overlay",
            variable=self.lanes_overlay_var,
            command=lambda: self.update_display(self.current_index),
        ).pack(side=tk.LEFT, padx=10)
        self.lk_overlay_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            control,
            text="LK HUD",
            variable=self.lk_overlay_var,
            command=lambda: self.update_display(self.current_index),
        ).pack(side=tk.LEFT, padx=6)
        self.calib_status = ttk.Label(control, text="", foreground="gray")
        self.calib_status.pack(side=tk.LEFT, padx=8)
        self.lk_status = ttk.Label(control, text="", foreground="gray")
        self.lk_status.pack(side=tk.LEFT, padx=8)

        ttk.Label(control, text="Speed:").pack(side=tk.LEFT, padx=(20, 5))
        self.speed_var = tk.DoubleVar(value=1.0)
        ttk.Scale(
            control,
            from_=0.1,
            to=5.0,
            orient=tk.HORIZONTAL,
            variable=self.speed_var,
            length=100,
        ).pack(side=tk.LEFT, padx=5)
        self.speed_label = ttk.Label(control, text="1.0x")
        self.speed_label.pack(side=tk.LEFT, padx=5)
        self.speed_var.trace_add("write", self.on_speed_change)

        self.status_label = ttk.Label(control, text="Ready", foreground="green")
        self.status_label.pack(side=tk.RIGHT, padx=10)

        main = ttk.Frame(self.master)
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        # Equal halves: map left, camera right, whatever the widgets ask for.
        main.columnconfigure(0, weight=1, uniform="halves")
        main.columnconfigure(1, weight=1, uniform="halves")
        main.rowconfigure(0, weight=1)
        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew")
        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew")

        self.fig_trajectory = Figure(figsize=(6, 6), dpi=100)
        self.ax_trajectory = self.fig_trajectory.add_subplot(111)
        self._bare_axes()
        self.canvas_trajectory = FigureCanvasTkAgg(self.fig_trajectory, left)
        self.canvas_trajectory.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        (self.current_pos_marker,) = self.ax_trajectory.plot([], [], "ro", markersize=10)
        self._create_map_controls(left)

        self.camera_canvas = tk.Canvas(right, bg="black", width=320, height=240)
        self.camera_canvas.pack(fill=tk.BOTH, expand=True)

        timeline = ttk.LabelFrame(self.master, text="Timeline", padding=10)
        timeline.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        slider_row = ttk.Frame(timeline)
        slider_row.pack(fill=tk.X)
        self.time_var = tk.IntVar(value=0)
        self.timeline_slider = ttk.Scale(
            slider_row,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.time_var,
            command=self.on_timeline_change,
        )
        self.timeline_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.time_label = ttk.Label(
            slider_row, text="00:00:00 / 00:00:00", font=("Courier", 10)
        )
        self.time_label.pack(side=tk.RIGHT, padx=10)

        sensors = ttk.LabelFrame(self.master, text="Sensors", padding=10)
        sensors.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
        imu_f = ttk.Frame(sensors)
        imu_f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        gps_f = ttk.Frame(sensors)
        gps_f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        odom_f = ttk.Frame(sensors)
        odom_f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)

        ttk.Label(imu_f, text="IMU", font=("Arial", 11, "bold")).pack(anchor=tk.W)
        self.imu_text = tk.Text(imu_f, height=4, width=35, font=("Courier", 9))
        self.imu_text.pack(fill=tk.BOTH, expand=True)
        self.imu_text.config(state=tk.DISABLED)

        ttk.Label(gps_f, text="GPS", font=("Arial", 11, "bold")).pack(anchor=tk.W)
        self.gps_text = tk.Text(gps_f, height=4, width=35, font=("Courier", 9))
        self.gps_text.pack(fill=tk.BOTH, expand=True)
        self.gps_text.config(state=tk.DISABLED)

        ttk.Label(odom_f, text="Odometry", font=("Arial", 11, "bold")).pack(anchor=tk.W)
        self.odom_text = tk.Text(odom_f, height=4, width=35, font=("Courier", 9))
        self.odom_text.pack(fill=tk.BOTH, expand=True)
        self.odom_text.config(state=tk.DISABLED)

        self.master.protocol("WM_DELETE_WINDOW", self.on_close)

    def open_bag(self) -> None:
        path = filedialog.askdirectory(title="Select ADAS bag session directory")
        if not path:
            path = filedialog.askopenfilename(
                title="Or select .zip / .tar.gz",
                filetypes=[
                    ("Archives", "*.zip *.tar.gz *.tgz"),
                    ("All", "*.*"),
                ],
            )
        if path:
            self.load_bag(path)

    def on_close(self) -> None:
        self.playing = False
        if self.player is not None:
            self.player.close()
            self.player = None
        self.master.destroy()

    def load_bag(self, bag_path: str) -> None:
        try:
            self.status_label.config(text="Loading…", foreground="orange")
            self.master.update()
            self.playing = False

            if self.player is not None:
                self.player.close()

            print(f"Loading bag: {bag_path}")
            # quiet: the player's own summary decodes every topic, and half of them
            # (can/rx, localization/pose, controls/steer, …) are never read here.
            self.player = AndroidBagPlayer(bag_path, quiet=True)
            print("Topics: " + ", ".join(self.player.topics))
            self.bag_dir = Path(bag_path)
            if self.bag_dir.is_file():
                self.bag_dir = self.bag_dir.parent

            wheel, steering, gear = extract_vehicle_series(self.player)
            self.wheel_data = np.asarray(wheel, dtype=np.float64) if wheel else None
            self.steering_data = (
                np.asarray(steering, dtype=np.float64) if steering else None
            )
            # gear has mixed types (str name) — keep as object array / list
            self.gear_data = np.array(gear, dtype=object) if gear else None
            self.imu_data = extract_imu(self.player)
            self.gps_data = extract_gps(self.player)
            self._load_camera_frames()
            self._load_bag_lanes()
            self._load_bag_lane_keep()
            self._load_lane_keep_params()
            self._load_intrinsics()
            self.load_camera_calibration()

            self.trajectories = {}
            self.calculate_trajectories()
            # Before plotting: anchoring the map re-projects the trajectories into its frame.
            self._ensure_osm()
            self._compute_headings()
            self.plot_trajectories()

            if self.wheel_data is not None and len(self.wheel_data) > 0:
                self.timestamps = self.wheel_data[:, 0]
                self.timeline_slider.config(to=max(0, len(self.timestamps) - 1))
                self.time_var.set(0)
            elif self.camera_frames:
                self.timestamps = self.image_timestamps.astype(np.float64)
                self.timeline_slider.config(to=max(0, len(self.timestamps) - 1))
                self.time_var.set(0)
            else:
                self.timestamps = np.array([], dtype=np.float64)
                messagebox.showwarning(
                    "No vehicle/state",
                    "Bag has no vehicle/state — trajectory/odometry unavailable.\n"
                    "Camera/GPS/IMU still load if present.",
                )

            # After the timeline exists: incidents are placed by bag timestamp.
            self._load_incidents()
            self.refresh_map_layer(draw=False)
            self.update_display(0)
            n_img = len(self.camera_frames)
            n_imu = 0 if self.imu_data is None else len(self.imu_data)
            n_gps = 0 if self.gps_data is None else len(self.gps_data)
            n_wh = 0 if self.wheel_data is None else len(self.wheel_data)
            n_lanes = len(self.bag_lane_frames)
            n_lk = len(self.bag_lane_keep_frames)
            self.status_label.config(
                text=f"wheel {n_wh} · imu {n_imu} · gps {n_gps} · cam {n_img} · lanes {n_lanes} · lk {n_lk}",
                foreground="green",
            )
        except Exception as e:
            import traceback

            traceback.print_exc()
            self.status_label.config(text=f"Error: {e}", foreground="red")
            try:
                messagebox.showerror("Error", f"Failed to load bag:\n{e}")
            except tk.TclError:
                pass

    def _load_camera_frames(self) -> None:
        self.camera_frames = []
        if self.player is None or "sensors/camera/image" not in self.player.topics:
            self.image_timestamps = np.array([], dtype=np.int64)
            return
        for ts, msg in self.player.get_topic_msgs("sensors/camera/image"):
            data = getattr(msg, "image_data", b"") or b""
            if data:
                self.camera_frames.append((int(ts), data, msg))
        self.image_timestamps = np.array(
            [t for t, _, _ in self.camera_frames], dtype=np.int64
        )
        print(f"Camera frames: {len(self.camera_frames)}")

    def _load_bag_lanes(self) -> None:
        self.bag_lane_frames = []
        if self.player is None or "vision/lanes" not in self.player.topics:
            self.lane_timestamps = np.array([], dtype=np.int64)
            return
        for ts, msg in self.player.get_topic_msgs("vision/lanes"):
            self.bag_lane_frames.append((int(ts), msg))
        self.lane_timestamps = np.array(
            [t for t, _ in self.bag_lane_frames], dtype=np.int64
        )
        print(f"Bag lane messages: {len(self.bag_lane_frames)}")

    def _load_bag_lane_keep(self) -> None:
        self.bag_lane_keep_frames = []
        if self.player is None or "control/lane_keep" not in self.player.topics:
            self.lane_keep_timestamps = np.array([], dtype=np.int64)
            return
        for ts, msg in self.player.get_topic_msgs("control/lane_keep"):
            self.bag_lane_keep_frames.append((int(ts), msg))
        self.lane_keep_timestamps = np.array(
            [t for t, _ in self.bag_lane_keep_frames], dtype=np.int64
        )
        print(f"Bag lane_keep messages: {len(self.bag_lane_keep_frames)}")

    def _meas_road_steer_rad(self, index: int) -> Optional[float]:
        """Measured road-wheel angle [rad] from bag steering series (device right+)."""
        if self.steering_data is None or index < 0:
            return None
        if index >= len(self.steering_data):
            index = len(self.steering_data) - 1
        st = self.steering_data[index]
        deg = float(st[1]) * _UNIT_TO_DEG
        if st[2] != 0:
            deg = -deg
        return float(np.deg2rad(deg))

    def _find_bag_lane_keep_by_time(self, target_time: float) -> Optional[Any]:
        if len(self.lane_keep_timestamps) == 0:
            return None
        idx = int(np.searchsorted(self.lane_keep_timestamps, target_time))
        if idx <= 0:
            return self.bag_lane_keep_frames[0][1]
        if idx >= len(self.lane_keep_timestamps):
            return self.bag_lane_keep_frames[-1][1]
        if abs(self.lane_keep_timestamps[idx - 1] - target_time) <= abs(
            self.lane_keep_timestamps[idx] - target_time
        ):
            return self.bag_lane_keep_frames[idx - 1][1]
        return self.bag_lane_keep_frames[idx][1]

    def _find_bag_lanes_by_time(self, target_time: float) -> Optional[Any]:
        """Latest vision/lanes message with ts <= target camera/state time."""
        if len(self.lane_timestamps) == 0:
            return None
        idx = int(np.searchsorted(self.lane_timestamps, target_time, side="right")) - 1
        if idx < 0:
            idx = 0
        return self.bag_lane_frames[idx][1]

    def _load_lane_keep_params(self) -> None:
        """Fusion parameters the run actually used, from ``control/lane_keep_debug``.

        The python defaults are not them: this firmware ships ``lane_blend_scale`` 0.6 and
        ``camera_offset`` 0.05 m, so a target path drawn with the defaults is not the path
        the controller followed.
        """
        self.lane_blend_scale = 0.0
        self.camera_offset_m = DEFAULT_CAMERA_OFFSET_M
        if self.player is None or "control/lane_keep_debug" not in self.player.topics:
            return
        for _, msg in self.player.get_topic_msgs("control/lane_keep_debug"):
            blend = float(getattr(msg, "p_lane_blend_scale", 0.0) or 0.0)
            offset = float(getattr(msg, "p_camera_offset_m", 0.0) or 0.0)
            if blend > 0.0 or offset > 0.0:
                self.lane_blend_scale = blend
                self.camera_offset_m = offset
                break
        print(
            f"Path fusion from the bag: lane_blend_scale={self.lane_blend_scale:.2f} "
            f"camera_offset={self.camera_offset_m:.3f} m"
        )

    def _lane_path(self, lanes_msg: Any) -> Optional[np.ndarray]:
        """Target path as the run's own parameters produce it."""
        bundle = path_bundle_from_bag_lanes(
            lanes_msg,
            lane_blend_scale=self.lane_blend_scale,
            camera_offset_m=self.camera_offset_m,
        )
        return None if bundle is None else bundle["polyline"]

    def _load_intrinsics(self) -> None:
        self.intr_msg = None
        if self.player is None:
            return

        if "camera/intrinsics" in self.player.topics:
            msgs = self.player.get_topic_msgs("camera/intrinsics")
            if msgs:
                self.intr_msg = msgs[0][1]

        cam0 = self.camera_frames[0][2] if self.camera_frames else None
        self.intrinsics = intrinsics_from_messages(cam0, self.intr_msg)
        print(
            f"Intrinsics: fx={self.intrinsics.fx:.1f} fy={self.intrinsics.fy:.1f} "
            f"cx={self.intrinsics.cx:.1f} cy={self.intrinsics.cy:.1f} "
            f"{self.intrinsics.width}x{self.intrinsics.height}"
        )

    def calculate_trajectories(self) -> None:
        print("\nCalculating trajectories…")
        if self.wheel_data is None or len(self.wheel_data) == 0:
            print("No wheel data")
            return

        initial_yaw = 0.0
        if self.gps_data is not None and len(self.gps_data) >= 2:
            initial_yaw = calculate_initial_heading_from_gps(self.gps_data, n_points=5)
            print(f"Initial GPS heading: {np.degrees(initial_yaw):.1f}°")

        if self.gps_data is not None and len(self.gps_data) > 0:
            x_gps, y_gps = gps_to_local_coords(self.gps_data, origin_idx=0)
            self.trajectories["GPS"] = (x_gps, y_gps)

        wheel = self.wheel_data
        steering = self.steering_data
        gear = self.gear_data

        x_odom, y_odom = calculate_trajectory(
            wheel, steering, gear, wheelbase=DEFAULTS.wheelbase, initial_yaw=initial_yaw
        )
        self.trajectories["Odometry"] = (x_odom, y_odom)
        print(f"Odometry: {len(x_odom)} pts")

        if self.imu_data is not None and len(self.imu_data) > 0:
            try:
                imu_processed = process_imu_for_odometry(
                    self.imu_data,
                    wheel,
                    speed_threshold_orientation=0.1,
                    speed_threshold_bias=0.5,
                    time_window_sec=20.0,
                    invert_yaw_rate=True,
                )
                x_imu, y_imu = calculate_trajectory_imu(
                    wheel,
                    imu_processed["yaw_rate"],
                    imu_processed["imu_calibrated"][:, 0],
                    initial_yaw=initial_yaw,
                )
                self.trajectories["IMU"] = (x_imu, y_imu)
                print(f"IMU: {len(x_imu)} pts")

                if self.gps_data is not None and len(self.gps_data) > 0:
                    x_ekf, y_ekf, _ = calculate_trajectory_ekf(
                        wheel,
                        steering,
                        gear,
                        imu_processed["yaw_rate"],
                        imu_processed["imu_calibrated"][:, 0],
                        self.gps_data,
                        wheelbase=DEFAULTS.wheelbase,
                        initial_yaw=initial_yaw,
                    )
                    self.trajectories["EKF"] = (x_ekf, y_ekf)
                    print(f"EKF: {len(x_ekf)} pts")
            except Exception as e:
                print(f"IMU/EKF failed: {e}")
                import traceback

                traceback.print_exc()

    def plot_trajectories(self) -> None:
        self.ax_trajectory.clear()
        # Axes.clear() drops the callback registry, so the map hook is re-armed here.
        self.ax_trajectory.callbacks.connect("xlim_changed", self._on_axes_limits)
        self._bare_axes()

        styles = {
            "Odometry": {"color": "blue", "linestyle": "-", "linewidth": 2, "alpha": 0.7},
            "GPS": {"color": "red", "linestyle": "--", "linewidth": 2, "alpha": 0.7},
            "IMU": {"color": "green", "linestyle": "-.", "linewidth": 1.5, "alpha": 0.6},
            "EKF": {"color": "orange", "linestyle": "-", "linewidth": 2.5, "alpha": 0.9},
        }
        self.traj_lines = {}
        for name, (x, y) in self.trajectories.items():
            if x is None or len(x) == 0:
                continue
            (line,) = self.ax_trajectory.plot(x, y, label=name, **styles.get(name, {}))
            self.traj_lines[name] = line
            if name in getattr(self, "traj_vars", {}):
                line.set_visible(self.traj_vars[name].get())

        (self.current_pos_marker,) = self.ax_trajectory.plot(
            [],
            [],
            "ro",
            markersize=12,
            label="Current",
            zorder=10,
            markeredgecolor="white",
            markeredgewidth=2,
        )
        if self.trajectories:
            self.ax_trajectory.legend(
                loc="upper right", fontsize=8, framealpha=0.85, borderpad=0.3
            )

        self._osm_line = None
        self._osm_wide_line = None
        self._osm_texts = []
        self._detail_artists = []
        self._map_view = None
        self._full_extent = self._extent_of_trajectories()
        self._apply_zoom(draw=False)
        self.refresh_map_layer(draw=False)
        self.canvas_trajectory.draw()

    # ---------------------------------------------------------------- OSM layer

    def _bare_axes(self) -> None:
        """No ticks, labels or title: the map should use the whole panel; scale bar instead."""
        ax = self.ax_trajectory
        # datalim, not box: with a box aspect the square plot leaves the panel half empty.
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.margins(0)
        self.fig_trajectory.subplots_adjust(left=0, right=1, top=1, bottom=0)

    def _draw_scale_bar(self) -> None:
        """Bar of a round length in the corner — the only thing left saying how far things are."""
        half = self._view_half_m()
        if half <= 0.0:
            return
        from mpl_toolkits.axes_grid1.anchored_artists import AnchoredSizeBar

        nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]
        length = min(nice, key=lambda v: abs(v - half * 0.5))
        label = f"{length} m" if length < 1000 else f"{length // 1000} km"
        bar = AnchoredSizeBar(
            self.ax_trajectory.transData,
            length,
            label,
            "lower left",
            pad=0.4,
            borderpad=0.6,
            sep=3,
            frameon=False,
            size_vertical=max(0.5, half * 0.004),
            color="#2f3640",
            fontproperties=FontProperties(size=8),
        )
        bar.set_zorder(12)
        self.ax_trajectory.add_artist(bar)
        self._detail_artists.append(bar)

    def _create_map_controls(self, parent: tk.Widget) -> None:
        """Toolbar + OSM background and segment-zoom controls under the trajectory plot."""
        bar = ttk.Frame(parent)
        bar.pack(fill=tk.X)
        self.traj_toolbar = NavigationToolbar2Tk(
            self.canvas_trajectory, bar, pack_toolbar=False
        )
        self.traj_toolbar.update()
        self.traj_toolbar.pack(side=tk.LEFT)

        layers = ttk.Frame(parent)
        layers.pack(fill=tk.X, pady=(2, 0))
        self.layer_vars = {}
        for key, text in self.MAP_LAYERS:
            self._layer_check(layers, key, text)
        ttk.Separator(layers, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        for key, text in self.OVERLAY_LAYERS:
            self._layer_check(layers, key, text)
        self.osm_status = ttk.Label(layers, text="", foreground="gray")
        self.osm_status.pack(side=tk.RIGHT, padx=6)
        # Kept as attributes: the map background reads them on every redraw.
        self.osm_var = self.layer_vars["osm"]
        self.osm_names_var = self.layer_vars["streets"]

        opts = ttk.Frame(parent)
        opts.pack(fill=tk.X, pady=(2, 0))
        self.traj_vars = {}
        for name in ("GPS", "Odometry", "IMU", "EKF"):
            var = tk.BooleanVar(value=True)
            self.traj_vars[name] = var
            ttk.Checkbutton(
                opts,
                text=name,
                variable=var,
                command=self._apply_trajectory_visibility,
            ).pack(side=tk.LEFT, padx=(4, 2))
        ttk.Separator(opts, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=6)
        self.follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            opts, text="Follow", variable=self.follow_var, command=self._on_zoom_change
        ).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(opts, text="Zoom:").pack(side=tk.LEFT)
        self.zoom_var = tk.DoubleVar(value=0.0)
        ttk.Scale(
            opts,
            from_=0.0,
            to=100.0,
            orient=tk.HORIZONTAL,
            variable=self.zoom_var,
            command=lambda _v: self._on_zoom_change(),
            length=220,
        ).pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        self.zoom_label = ttk.Label(opts, text="whole route", width=13)
        self.zoom_label.pack(side=tk.LEFT, padx=4)

        self.ax_trajectory.callbacks.connect("xlim_changed", self._on_axes_limits)

    MAP_LAYERS = (
        ("osm", "OSM"),
        ("bands", "Surf"),
        ("osm_lanes", "Lanes"),
        ("streets", "Names"),
        ("road", "Road"),
    )
    OVERLAY_LAYERS = (
        ("cam", "Cam"),
        ("ego", "Ego"),
        ("marks", "Marks"),
        ("centre", "Mid"),
        ("plan", "Plan"),
        ("target", "Tgt"),
        ("steer", "δ"),
        ("incidents", "Inc"),
    )

    def _layer_check(self, parent: tk.Widget, key: str, text: str) -> None:
        var = tk.BooleanVar(value=True)
        self.layer_vars[key] = var
        ttk.Checkbutton(
            parent, text=text, variable=var, command=self.refresh_map_layer
        ).pack(side=tk.LEFT, padx=(4, 2))

    def _layer_on(self, key: str) -> bool:
        var = self.layer_vars.get(key) if hasattr(self, "layer_vars") else None
        return True if var is None else bool(var.get())

    def _apply_trajectory_visibility(self) -> None:
        for name, line in self.traj_lines.items():
            line.set_visible(self.traj_vars[name].get())
        self.canvas_trajectory.draw_idle()

    def _extent_of_trajectories(self) -> Optional[Tuple[float, float, float, float]]:
        xs, ys = [], []
        for x, y in self.trajectories.values():
            if x is not None and len(x):
                xs.append(np.asarray(x))
                ys.append(np.asarray(y))
        if not xs:
            return None
        x = np.concatenate(xs)
        y = np.concatenate(ys)
        pad = 0.05 * max(float(np.ptp(x)), float(np.ptp(y)), 50.0)
        return (
            float(x.min()) - pad,
            float(y.min()) - pad,
            float(x.max()) + pad,
            float(y.max()) + pad,
        )

    def _zoom_half_m(self) -> Optional[float]:
        """Half-size of the view in meters; None means the whole route."""
        if self._full_extent is None:
            return None
        level = float(self.zoom_var.get()) if hasattr(self, "zoom_var") else 0.0
        if level <= 0.5:
            return None
        x0, y0, x1, y1 = self._full_extent
        full = max(x1 - x0, y1 - y0) * 0.5
        near = 5.0
        if full <= near:
            return full
        return full * (near / full) ** (level / 100.0)

    def _panel_half(self, half_m: float) -> Tuple[float, float]:
        """Half-extents that fill the panel at 1:1 scale; ``half_m`` is the shorter side."""
        bbox = self.ax_trajectory.get_window_extent()
        w, h = max(bbox.width, 1.0), max(bbox.height, 1.0)
        return (half_m * w / h, half_m) if w >= h else (half_m, half_m * h / w)

    def _on_zoom_change(self) -> None:
        self._apply_zoom(draw=False)
        self.refresh_map_layer()

    def _apply_zoom(self, draw: bool = True) -> None:
        if self._full_extent is None:
            return
        half = self._zoom_half_m()
        if half is None:
            x0, y0, x1, y1 = self._full_extent
            cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
            half = 0.5 * max(x1 - x0, y1 - y0)
            if hasattr(self, "zoom_label"):
                self.zoom_label.config(text="whole route")
        else:
            pos = self._ego_position(self.current_index)
            if pos is None or not self.follow_var.get():
                cx, cy = (
                    0.5
                    * (
                        self.ax_trajectory.get_xlim()[0]
                        + self.ax_trajectory.get_xlim()[1]
                    ),
                    0.5
                    * (
                        self.ax_trajectory.get_ylim()[0]
                        + self.ax_trajectory.get_ylim()[1]
                    ),
                )
            else:
                cx, cy = pos
            self.zoom_label.config(text=f"±{half:.0f} m")

        # Shape the limits like the panel: an equal aspect on datalim would otherwise stretch
        # them at draw time, and the resulting limit change re-armed a second refresh per frame.
        hx, hy = self._panel_half(half)
        self._view_busy = True
        try:
            self.ax_trajectory.set_xlim(cx - hx, cx + hx)
            self.ax_trajectory.set_ylim(cy - hy, cy + hy)
        finally:
            self._view_busy = False
        if draw:
            self.canvas_trajectory.draw_idle()

    def _on_axes_limits(self, _ax) -> None:
        """Pan/zoom with the toolbar refetches the map for the new window."""
        if self._view_busy or self._view_pending:
            return
        # An equal aspect on datalim rewrites the limits on every draw, so a refetch is
        # armed only when the window really moved — otherwise draw and refetch ping-pong.
        if not self._view_moved():
            return
        self._view_pending = True
        self.master.after_idle(self._refresh_after_view_change)

    def _refresh_after_view_change(self) -> None:
        self._view_pending = False
        self.refresh_map_layer(force=False)

    def _current_view(self) -> Tuple[float, float, float, float]:
        x0, x1 = self.ax_trajectory.get_xlim()
        y0, y1 = self.ax_trajectory.get_ylim()
        return x0, y0, x1, y1

    def _view_moved(self) -> bool:
        view = self._current_view()
        if self._last_view is None:
            return True
        span = max(view[2] - view[0], 1e-6)
        return any(abs(a - b) > 0.02 * span for a, b in zip(view, self._last_view))

    def _ensure_osm(self) -> bool:
        """Anchor the map to the run and re-project the trajectories into the map frame."""
        if self.osm.ready:
            return True
        if self.gps_data is None or len(self.gps_data) == 0:
            self._set_osm_status(
                "no GNSS", "bag has no GNSS — the map cannot be anchored"
            )
            return False
        if not self.osm.set_origin(
            float(self.gps_data[0, 1]), float(self.gps_data[0, 2])
        ):
            self._set_osm_status("no map", self.osm.error or "map unavailable")
            return False
        self.osm_lane_counts = lane_counts(self.osm.map_path)
        for name, (x, y) in list(self.trajectories.items()):
            if x is not None and len(x):
                self.trajectories[name] = self.osm.to_map(x, y)
        return True

    def _set_osm_status(self, text: str, detail: str = "") -> None:
        """Short text next to the layer switches; anything that needs room goes to stdout."""
        if hasattr(self, "osm_status"):
            self.osm_status.config(text=text)
        if detail:
            print(f"OSM layer: {detail}")

    def refresh_map_layer(self, draw: bool = True, force: bool = True) -> None:
        """Redraw the OSM background (when the view left it) and the per-frame overlays."""
        if force or self._osm_stale():
            self._draw_osm_background()
        self._last_view = self._current_view()
        self._draw_frame_layer()
        if draw:
            self.canvas_trajectory.draw_idle()

    def _osm_stale(self) -> bool:
        """True when the view left the fetched window, or the zoom changed enough that the
        road bands — sized in screen points from a ground width — would be wrong."""
        if self._map_view is None:
            return True
        x0, y0, x1, y1 = self._current_view()
        fx0, fy0, fx1, fy1 = self._map_view
        if x0 < fx0 or y0 < fy0 or x1 > fx1 or y1 > fy1:
            return True
        ratio = (x1 - x0) / max(self._map_span, 1e-6)
        return ratio < 0.7 or ratio > 1.4

    def _clear_background(self) -> None:
        if self._osm_line is not None:
            self._osm_line.remove()
        for band in self._osm_wide_line or []:
            band.remove()
        self._osm_line = None
        self._osm_wide_line = None
        for text in self._osm_texts:
            text.remove()
        self._osm_texts = []

    def _clear_frame_layer(self) -> None:
        for artist in self._detail_artists:
            artist.remove()
        self._detail_artists = []

    def _clear_osm_artists(self) -> None:
        self._clear_background()
        self._clear_frame_layer()
        self._map_view = None

    OSM_FETCH_PAD = 1.5

    def _draw_osm_background(self) -> None:
        """Roads, lanes and street names — fetched for more than the view, so playback
        does not re-read and re-lay-out the map on every frame."""
        self._clear_background()
        if not (hasattr(self, "osm_var") and self.osm_var.get() and self._ensure_osm()):
            self._map_view = None
            return

        x0, y0, x1, y1 = self._current_view()
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        span = max(x1 - x0, y1 - y0)
        # Padding buys silence during playback; over a whole city it only buys work.
        pad = self.OSM_FETCH_PAD if span <= 2000.0 else 0.05
        half = 0.5 * span * (1.0 + 2.0 * pad)
        fx0, fy0, fx1, fy1 = cx - half, cy - half, cx + half, cy + half
        self._map_view = (fx0, fy0, fx1, fy1)
        self._map_span = x1 - x0

        xs, ys = self.osm.polylines(fx0, fy0, fx1, fy1)
        if not len(xs):
            self._set_osm_status("no roads")
            return

        view_half = 0.5 * (x1 - x0)
        if view_half <= 200.0 and (
            self._layer_on("bands") or self._layer_on("osm_lanes")
        ):
            self._draw_road_bands(fx0, fy0, fx1, fy1, view_half)
        (self._osm_line,) = self.ax_trajectory.plot(
            xs, ys, color="#9aa4b0", linewidth=0.8, alpha=0.9, zorder=1
        )
        n_edges = int(np.isnan(xs).sum()) + 1
        self._set_osm_status(f"{n_edges} edges")

        if self._layer_on("streets") and view_half <= 1500.0:
            for lx, ly, ang, name in self.osm.street_labels(x0, y0, x1, y1, limit=14):
                self._osm_texts.append(
                    self.ax_trajectory.text(
                        lx,
                        ly,
                        name,
                        rotation=ang,
                        rotation_mode="anchor",
                        fontsize=7,
                        color="#5a6472",
                        ha="center",
                        va="bottom",
                        zorder=2,
                        clip_on=True,
                    )
                )

    LANE_WIDTH_M = 3.5
    OSM_LANE_WIDTH_PT = 1.8

    def _draw_road_bands(
        self, x0: float, y0: float, x1: float, y1: float, half_m: float
    ) -> None:
        """Every road in view as a surface of its own width, with its lane boundaries."""
        by_lanes: dict = {}
        rails_x: List[float] = []
        rails_y: List[float] = []
        with_lanes = half_m <= 120.0 and self._layer_on("osm_lanes")
        with_bands = self._layer_on("bands")
        for eid in self.osm.edges(x0, y0, x1, y1):
            px, py = self.osm.edge_polyline(eid)
            if len(px) < 2:
                continue
            n = int(self.osm_lane_counts.get(eid, 2))
            if with_bands:
                xs, ys = by_lanes.setdefault(n, ([], []))
                xs.extend(px.tolist() + [np.nan])
                ys.extend(py.tolist() + [np.nan])
            if with_lanes and n > 1:
                for lx, ly in lane_offsets(px, py, n, self.LANE_WIDTH_M):
                    rails_x.extend(lx.tolist() + [np.nan])
                    rails_y.extend(ly.tolist() + [np.nan])
        self._osm_wide_line = []
        for n, (xs, ys) in by_lanes.items():
            (band,) = self.ax_trajectory.plot(
                xs,
                ys,
                color="#dcdfe4",
                linewidth=self._meters_to_points(n * self.LANE_WIDTH_M, half_m),
                solid_capstyle="round",
                zorder=0,
            )
            self._osm_wide_line.append(band)
        if rails_x:
            (rails,) = self.ax_trajectory.plot(
                rails_x,
                rails_y,
                color="#8b97a5",
                linewidth=self.OSM_LANE_WIDTH_PT,
                alpha=0.95,
                zorder=2,
                label="OSM lanes",
            )
            self._osm_wide_line.append(rails)

    def _meters_to_points(self, width_m: float, half_m: float) -> float:
        """Line width in points for a band that should span ``width_m`` on the ground."""
        bbox = self.ax_trajectory.get_window_extent()
        px_per_m = max(bbox.width, 1.0) / max(2.0 * half_m, 1e-6)
        return max(2.0, width_m * px_per_m * 72.0 / self.fig_trajectory.dpi)

    # ------------------------------------------------------------ detail layer

    def _reference_trajectory(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        return self.trajectories.get("EKF") or self.trajectories.get("Odometry")

    def _ego_position(self, index: int) -> Optional[Tuple[float, float]]:
        traj = self._reference_trajectory()
        if traj is None:
            return None
        x, y = traj
        if index >= len(x):
            return None
        return float(x[index]), float(y[index])

    def _compute_headings(self) -> None:
        """Course along the reference trajectory; the detail overlays are drawn in it."""
        self.ref_heading = None
        traj = self._reference_trajectory()
        if traj is None:
            return
        x, y = np.asarray(traj[0]), np.asarray(traj[1])
        if len(x) < 3:
            return
        # Chord over a fixed straight-line distance, not a per-sample derivative and not a
        # window of path length: standing at a red light the position jitters, which adds
        # path without moving the car, and the overlays would spin on the spot.
        ahead, behind = self._chord_neighbours(x, y, self.HEADING_BASE_M)
        self.ref_heading = np.unwrap(
            np.arctan2(y[ahead] - y[behind], x[ahead] - x[behind])
        )

    HEADING_BASE_M = 3.0

    @staticmethod
    def _chord_neighbours(x: np.ndarray, y: np.ndarray, dist_m: float):
        """For every sample, the nearest samples ahead and behind at least ``dist_m`` away."""
        n = len(x)
        d2 = dist_m * dist_m
        ahead = np.empty(n, dtype=np.int64)
        behind = np.empty(n, dtype=np.int64)
        j = 0
        for i in range(n):
            if j < i:
                j = i
            while j < n - 1 and (x[j] - x[i]) ** 2 + (y[j] - y[i]) ** 2 < d2:
                j += 1
            ahead[i] = j
        k = 0
        for i in range(n):
            while k + 1 <= i and (x[i] - x[k + 1]) ** 2 + (y[i] - y[k + 1]) ** 2 >= d2:
                k += 1
            behind[i] = k
        return ahead, behind

    def _ego_pose(self, index: int) -> Optional[Tuple[float, float, float]]:
        pos = self._ego_position(index)
        if pos is None or self.ref_heading is None or index >= len(self.ref_heading):
            return None
        return pos[0], pos[1], float(self.ref_heading[index])

    def _view_half_m(self) -> float:
        """Half-size of what is on screen, shorter side — the toolbar zooms without the slider."""
        x0, x1 = self.ax_trajectory.get_xlim()
        y0, y1 = self.ax_trajectory.get_ylim()
        return 0.5 * min(x1 - x0, y1 - y0)

    def _detail_active(self) -> bool:
        return self._view_half_m() <= 150.0

    def _draw_frame_layer(self) -> None:
        """Everything that follows the timeline: the pose overlays and the scale bar."""
        self._clear_frame_layer()
        self._draw_detail()
        self._draw_incidents()
        self._draw_scale_bar()
        if self.trajectories:
            self.ax_trajectory.legend(
                loc="upper right", fontsize=8, framealpha=0.85, borderpad=0.3
            )

    def _draw_detail(self) -> None:
        """Lane markings, the road under the car, model plan and planner output."""
        if not self._detail_active():
            return
        pose = self._ego_pose(self.current_index)
        if pose is None:
            return
        x, y, heading = pose
        ax = self.ax_trajectory

        if self._layer_on("cam"):
            self._detail_artists.extend(self._draw_road_image(x, y, heading))
        if self._layer_on("ego"):
            self._detail_artists.extend(self._draw_ego_box(x, y, heading))
        if self._layer_on("road"):
            self._detail_artists.extend(self._draw_current_road(x, y))

        t_ms = float(self.timestamps[self.current_index]) if len(self.timestamps) else 0.0
        lanes_msg = self._find_bag_lanes_by_time(t_ms)
        if lanes_msg is not None:
            if self._layer_on("marks"):
                self._detail_artists.extend(
                    self._draw_lane_marks(lanes_msg, x, y, heading)
                )
            if self._layer_on("centre"):
                self._detail_artists.extend(
                    self._draw_lane_centre(lanes_msg, x, y, heading)
                )
            plan_xy, _, _ = plan_orientation_from_lanes(lanes_msg)
            if self._layer_on("plan") and plan_xy is not None and len(plan_xy):
                pts = transform_to_world(plan_xy, x, y, heading)
                if pts is not None:
                    (line,) = ax.plot(
                        pts[:, 0],
                        pts[:, 1],
                        color="#0ea5e9",
                        linewidth=1.4,
                        alpha=0.8,
                        zorder=7,
                        label="model plan",
                    )
                    self._detail_artists.append(line)

            path = self._lane_path(lanes_msg) if self._layer_on("target") else None
            if path is not None:
                pts = transform_to_world(path, x, y, heading)
                if pts is not None:
                    (line,) = ax.plot(
                        pts[:, 0],
                        pts[:, 1],
                        color="#1f9d55",
                        linewidth=2.0,
                        zorder=7,
                        label="LK target path",
                    )
                    self._detail_artists.append(line)

        lk_msg = (
            self._find_bag_lane_keep_by_time(t_ms) if self._layer_on("steer") else None
        )
        if lk_msg is not None:
            self._detail_artists.extend(self._draw_planner(lk_msg, x, y, heading))

    def _draw_ego_box(self, x: float, y: float, heading: float) -> List[Any]:
        half_l, half_w = 2.3, 0.9
        box = np.array(
            [
                [half_l, -half_w],
                [half_l, half_w],
                [-half_l, half_w],
                [-half_l, -half_w],
                [half_l, -half_w],
            ]
        )
        pts = transform_to_world(box, x, y, heading)
        if pts is None:
            return []
        (line,) = self.ax_trajectory.plot(
            pts[:, 0], pts[:, 1], color="crimson", linewidth=1.4, zorder=9
        )
        return [line]

    def _draw_current_road(self, x: float, y: float) -> List[Any]:
        """Highlight the road the car is on; the rest of the network is in the background."""
        if not self.osm.ready:
            return []
        found = self.osm.nearest_edge(x, y, radius_m=40.0)
        if found is None:
            return []
        eid, _, name = found
        px, py = self.osm.edge_polyline(eid)
        if len(px) < 2:
            return []
        out: List[Any] = []
        (line,) = self.ax_trajectory.plot(
            px,
            py,
            color="#3b82f6",
            linewidth=1.6,
            linestyle="--",
            zorder=3,
            label="OSM road",
        )
        out.append(line)
        n_lanes = self.osm_lane_counts.get(eid)
        label = name or "OSM"
        if n_lanes:
            label = f"{label} · {n_lanes} lanes"
        out.append(
            self.ax_trajectory.text(
                px[len(px) // 2],
                py[len(py) // 2],
                label,
                fontsize=7,
                color="#3b82f6",
                zorder=4,
                clip_on=True,
            )
        )
        return out

    def _draw_lane_marks(
        self, lanes_msg: Any, x: float, y: float, heading: float
    ) -> List[Any]:
        xs = list(getattr(lanes_msg, "x", []) or [])
        lanes = list(getattr(lanes_msg, "lanes", []) or [])
        if not lanes:
            return []
        out: List[Any] = []
        styles = ["#c084fc", "#f59e0b", "#f59e0b", "#c084fc"]
        for i, lane in enumerate(lanes[:4]):
            ys = list(getattr(lane, "y", []) or [])
            if len(ys) < 2:
                continue
            prob = float(getattr(lane, "prob", 0.0) or 0.0)
            if prob < 0.3:
                continue
            grid = np.asarray(xs[: len(ys)] if xs else np.arange(len(ys)), dtype=float)
            if len(grid) != len(ys):
                grid = np.linspace(0.0, 60.0, len(ys))
            pts = transform_to_world(
                np.stack([grid, np.asarray(ys, dtype=float)], axis=1), x, y, heading
            )
            if pts is None:
                continue
            (line,) = self.ax_trajectory.plot(
                pts[:, 0],
                pts[:, 1],
                color=styles[i % len(styles)],
                linewidth=1.4,
                alpha=min(1.0, 0.4 + prob),
                zorder=6,
                label="lane marks" if not out else None,
            )
            out.append(line)
        return out

    def _near_lane_lines(self, lanes_msg: Any):
        """Grid and the two near lane lines (proto order 1 = left, 2 = right), or None."""
        lanes = list(getattr(lanes_msg, "lanes", []) or [])
        if len(lanes) < 3:
            return None
        xs = list(getattr(lanes_msg, "x", []) or [])
        out = []
        for i in (1, 2):
            ys = list(getattr(lanes[i], "y", []) or [])
            prob = float(getattr(lanes[i], "prob", 0.0) or 0.0)
            if len(ys) < 2 or prob < self.LANE_PROB_MIN:
                return None
            out.append(np.asarray(ys, dtype=float))
        grid = np.asarray(xs[: len(out[0])], dtype=float) if xs else None
        if grid is None or len(grid) != len(out[0]):
            grid = np.linspace(0.0, 60.0, len(out[0]))
        return grid, out[0], out[1]

    LANE_PROB_MIN = 0.3

    def _draw_lane_centre(
        self, lanes_msg: Any, x: float, y: float, heading: float
    ) -> List[Any]:
        """Middle between the detector's near lane lines — the reference the offset is
        measured from; without it the target path alone does not say whether the planner
        shifted the car or the lane moved."""
        near = self._near_lane_lines(lanes_msg)
        if near is None:
            return []
        grid, y_left, y_right = near
        mid = 0.5 * (y_left + y_right)
        pts = transform_to_world(np.stack([grid, mid], axis=1), x, y, heading)
        if pts is None:
            return []
        (line,) = self.ax_trajectory.plot(
            pts[:, 0],
            pts[:, 1],
            color="#7c3aed",
            linewidth=1.6,
            linestyle=(0, (6, 3)),
            zorder=7,
            label="lane centre",
        )
        return [line]

    def _draw_road_image(self, x: float, y: float, heading: float) -> List[Any]:
        """Camera frame flattened onto the road plane, under everything else."""
        if not self.camera_frames or self._view_half_m() > 80.0:
            return []
        t_ms = float(self.timestamps[self.current_index]) if len(self.timestamps) else 0.0
        img_index = self.find_closest_image_by_time(t_ms)
        if img_index is None:
            return []
        _, jpeg, cam_msg = self.camera_frames[img_index]
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return []
        h, w = img.shape[:2]
        if self.intrinsics is not None:
            K = intrinsics_from_messages(cam_msg, self.intr_msg)
            fx, fy, cx, cy = K.fx, K.fy, K.cx, K.cy
            if abs(fx / max(fy, 1e-6) - 1.0) > 0.15:
                fy = fx
        else:
            fx = fy = 0.5 * w / np.tan(np.radians(35.0))
            cx, cy = 0.5 * w, 0.5 * h
        geom = make_overlay_geometry(
            fx,
            fy,
            cx,
            cy,
            w,
            h,
            camera_height=self._overlay_height_m,
            pitch_deg=self._overlay_pitch_deg,
            yaw_deg=self._overlay_yaw_deg,
            roll_deg=self._overlay_roll_deg,
            cam_x=self._overlay_cam_x,
            cam_y_left=self._overlay_cam_y,
        )
        x0, x1 = self.BEV_RANGE_M
        half = self.BEV_HALF_WIDTH_M
        patch = warp_to_road(img, geom, (x0, x1), half, self.BEV_PX_PER_M)
        if patch is None:
            return []
        transform = (
            Affine2D().rotate(heading).translate(x, y) + self.ax_trajectory.transData
        )
        im = self.ax_trajectory.imshow(
            patch,
            extent=(x0, x1, -half, half),
            transform=transform,
            interpolation="bilinear",
            zorder=2.5,
        )
        im.set_clip_path(None)
        return [im]

    BEV_RANGE_M = (3.0, 40.0)
    BEV_HALF_WIDTH_M = 12.0
    BEV_PX_PER_M = 8.0

    def _load_incidents(self) -> None:
        self.incidents = incident_marks.load(self.bag_dir)
        if self.incidents:
            print(f"Incidents dictated in this run: {len(self.incidents)}")
            for inc in self.incidents:
                rel = (
                    (inc.t_ms - float(self.timestamps[0])) / 1000.0
                    if len(self.timestamps)
                    else 0.0
                )
                print(f"  {_format_ms(1000.0 * rel)}  {inc.text}")

    def jump_incident(self, direction: int) -> None:
        """Move the timeline to the next/previous dictated mark."""
        if not self.incidents or len(self.timestamps) == 0:
            self._set_lk_status("no dictated incidents in this bag")
            return
        now = float(self.timestamps[self.current_index])
        times = [inc.t_ms for inc in self.incidents]
        if direction > 0:
            picks = [i for i, t in enumerate(times) if t > now + 500.0]
            which = picks[0] if picks else len(times) - 1
        else:
            picks = [i for i, t in enumerate(times) if t < now - 500.0]
            which = picks[-1] if picks else 0
        idx = self._incident_index(times[which])
        if idx is None:
            return
        self.playing = False
        self.time_var.set(idx)
        self.update_display(idx)
        self._set_lk_status(
            f"incident {which + 1}/{len(times)}: {self.incidents[which].text}"
        )

    def _incident_index(self, t_ms: float) -> Optional[int]:
        if len(self.timestamps) == 0:
            return None
        idx = int(np.searchsorted(self.timestamps, t_ms))
        return int(np.clip(idx, 0, len(self.timestamps) - 1))

    def _draw_incidents(self) -> None:
        """Where the operator dictated a complaint — the ground truth for 'it drove wrong'."""
        if not self._layer_on("incidents") or not self.incidents:
            return
        traj = self._reference_trajectory()
        if traj is None:
            return
        tx, ty = traj
        x0, y0, x1, y1 = self._current_view()
        near = self._view_half_m() <= 150.0
        xs, ys = [], []
        for inc in self.incidents:
            idx = self._incident_index(inc.t_ms)
            if idx is None or idx >= len(tx):
                continue
            px, py = float(tx[idx]), float(ty[idx])
            xs.append(px)
            ys.append(py)
            if near and x0 <= px <= x1 and y0 <= py <= y1:
                self._detail_artists.append(
                    self.ax_trajectory.text(
                        px,
                        py + 2.0,
                        inc.text[:40],
                        fontsize=7,
                        color="#b91c1c",
                        ha="left",
                        va="bottom",
                        zorder=11,
                        clip_on=True,
                    )
                )
        if not xs:
            return
        (marks,) = self.ax_trajectory.plot(
            xs,
            ys,
            linestyle="none",
            marker="v",
            markersize=7,
            color="#b91c1c",
            markeredgecolor="white",
            markeredgewidth=0.8,
            zorder=11,
            label=f"incidents ({len(xs)})",
        )
        self._detail_artists.append(marks)

    def _draw_planner(self, lk_msg: Any, x: float, y: float, heading: float) -> List[Any]:
        lk = lane_keep_result_from_bag(
            lk_msg, speed_mps=self._ego_speed_mps(self.current_index)
        )
        out: List[Any] = []
        # Steering command as the arc the car will follow: bicycle model at the logged δ.
        steer = float(lk.steer_rad)
        wheelbase = float(DEFAULTS.wheelbase)
        s = np.linspace(0.0, 30.0, 40)
        curv = np.tan(steer) / wheelbase if abs(steer) > 1e-6 else 0.0
        if abs(curv) < 1e-9:
            arc = np.stack([s, np.zeros_like(s)], axis=1)
        else:
            r = 1.0 / curv
            ang = s * curv
            arc = np.stack([r * np.sin(ang), r * (1.0 - np.cos(ang))], axis=1)
        pts = transform_to_world(arc, x, y, heading)
        if pts is not None:
            (line,) = self.ax_trajectory.plot(
                pts[:, 0],
                pts[:, 1],
                color="#ef4444",
                linewidth=1.8,
                linestyle=":",
                zorder=8,
                label="planner δ",
            )
            out.append(line)
        return out

    def update_display(self, index: int) -> None:
        if len(self.timestamps) == 0:
            self.update_camera_view(0)
            return

        self.current_index = int(np.clip(index, 0, len(self.timestamps) - 1))
        traj = self._reference_trajectory()
        if traj is not None:
            x, y = traj
            if self.current_index < len(x):
                self.current_pos_marker.set_data(
                    [x[self.current_index]], [y[self.current_index]]
                )
                if self._zoom_half_m() is not None:
                    self._apply_zoom(draw=False)
                    self.refresh_map_layer(draw=False, force=False)
                self.canvas_trajectory.draw_idle()

        self.update_camera_view(self.current_index)
        self.update_sensor_data(self.current_index)
        self.update_time_label(self.current_index)

    def _ego_speed_mps(self, index: int) -> float:
        """Best-effort ego speed (m/s) from wheels or GPS."""
        if self.wheel_data is not None and index < len(self.wheel_data):
            wh = self.wheel_data[index]
            # km/h average of four wheels
            v_kmh = float(np.mean(wh[1:5]))
            return max(0.0, v_kmh / 3.6)
        if len(self.timestamps) > 0 and index < len(self.timestamps):
            gps = _nearest_row(self.gps_data, float(self.timestamps[index]))
            if gps is not None:
                return max(0.0, float(gps[4]))
        return 0.0

    def load_camera_calibration(self) -> None:
        """Camera mounting for the overlays: assets priors, then the session's own file.

        ``calib_rpy.json`` is what the run itself was calibrated with, so replaying a bag
        shows the geometry the car actually used.
        """
        ap = self._asset_priors
        self._overlay_pitch_deg = ap.pitch_deg
        self._overlay_yaw_deg = ap.yaw_deg
        self._overlay_roll_deg = ap.roll_deg
        self._overlay_height_m = ap.height_m
        self._overlay_cam_x = ap.cam_x
        self._overlay_cam_y = ap.cam_y_left
        source = "assets"

        path = self.bag_dir / "calib_rpy.json" if self.bag_dir is not None else None
        if path is not None and path.is_file():
            try:
                data = json.loads(path.read_text())

                def angle(key: str, default: float) -> float:
                    if f"{key}_deg" in data:
                        return float(data[f"{key}_deg"])
                    if key in data:
                        return float(np.rad2deg(data[key]))
                    return default

                pitch = angle("pitch", ap.pitch_deg)
                # Windshield mount looks slightly down; a positive pitch is Hough junk.
                if pitch <= 2.0:
                    self._overlay_pitch_deg = pitch
                    self._overlay_yaw_deg = angle("yaw", ap.yaw_deg)
                    self._overlay_roll_deg = angle("roll", ap.roll_deg)
                    self._overlay_height_m = float(data.get("camera_height", ap.height_m))
                    self._overlay_cam_x = float(data.get("cam_x", ap.cam_x))
                    self._overlay_cam_y = float(data.get("cam_y_left", ap.cam_y_left))
                    source = path.name
                else:
                    print(f"Ignored {path} pitch={pitch:.1f}° (expect ≤2°)")
            except (OSError, ValueError) as e:
                print(f"Failed to load {path}: {e}")

        print(
            f"Camera calib from {source}: "
            f"R/P/Y={self._overlay_roll_deg:.1f}/{self._overlay_pitch_deg:.1f}/"
            f"{self._overlay_yaw_deg:.1f}°  h={self._overlay_height_m:.2f} "
            f"x={self._overlay_cam_x:.2f} y={self._overlay_cam_y:.2f}"
        )
        if hasattr(self, "calib_status"):
            self.calib_status.config(
                text=(
                    f"RPY {self._overlay_roll_deg:.1f}/{self._overlay_pitch_deg:.1f}/"
                    f"{self._overlay_yaw_deg:.1f}°  ({source})"
                )
            )

    def update_camera_view(self, index: int) -> None:
        if not self.camera_frames:
            return
        if len(self.timestamps) > 0 and index < len(self.timestamps):
            t = float(self.timestamps[index])
            img_index = self.find_closest_image_by_time(t)
        else:
            t = float(self.camera_frames[0][0])
            img_index = 0
        if img_index is None:
            return

        ts_cam, jpeg, cam_msg = self.camera_frames[img_index]
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return

        h, w = img.shape[:2]
        if self.intrinsics is not None:
            K = intrinsics_from_messages(cam_msg, self.intr_msg)
            fx, fy, cx, cy = K.fx, K.fy, K.cx, K.cy
            if abs(fx / max(fy, 1e-6) - 1.0) > 0.15:
                fy = fx
        else:
            fx = fy = 0.5 * w / np.tan(np.radians(35.0))
            cx, cy = 0.5 * w, 0.5 * h

        roll_deg = self._overlay_roll_deg
        pitch_deg = self._overlay_pitch_deg
        yaw_deg = self._overlay_yaw_deg
        height_m = self._overlay_height_m
        cam_x, cam_y = self._overlay_cam_x, self._overlay_cam_y

        if self.lanes_overlay_var.get():
            geom = make_overlay_geometry(
                fx,
                fy,
                cx,
                cy,
                w,
                h,
                camera_height=height_m,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                roll_deg=roll_deg,
                cam_x=cam_x,
                cam_y_left=cam_y,
            )
            bag_lanes = self._find_bag_lanes_by_time(float(ts_cam))
            if bag_lanes is not None:
                # Bag lanes/plan are device Y-right (same as flowpilot / ONNX).
                draw_bag_lanes(img, bag_lanes, geom, w, h, y_sign=DRAW_Y_SIGN)
                cv2.putText(
                    img,
                    f"vision/lanes  plan#{getattr(bag_lanes, 'plan_hyp', -1)}",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    img,
                    "vision/lanes: no sync for this frame",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 200, 255),
                    1,
                    cv2.LINE_AA,
                )

            if self.lk_overlay_var.get():
                poly = self._lane_path(bag_lanes) if bag_lanes is not None else None
                meas_rad = self._meas_road_steer_rad(index)
                lk_msg = self._find_bag_lane_keep_by_time(float(ts_cam))
                if poly is not None and poly.shape[0] >= 2:
                    if lk_msg is not None:
                        lk = lane_keep_result_from_bag(
                            lk_msg,
                            poly,
                            speed_mps=self._ego_speed_mps(index),
                            waypoint_shift=DEFAULTS.pp_shift,
                            wheel_base=DEFAULTS.wheelbase,
                        )
                        status = format_lane_keep_status(lk)
                    else:
                        # Lanes without logged control/lane_keep: show the path and the wheel.
                        lk = LaneKeepResult(
                            mode="pure_pursuit",
                            steer_rad=float(meas_rad or 0.0),
                            steer_norm=0.0,
                            throttle=0.0,
                            brake=0.0,
                            polyline=poly,
                            status="bag",
                            controller="bag",
                        )
                        status = "lanes (no control/lane_keep)"
                    img = draw_lane_keep_overlay(
                        img,
                        lk,
                        fx=fx,
                        fy=fy,
                        cx=cx,
                        cy=cy,
                        w=w,
                        h=h,
                        geom=geom,
                        pitch_deg=pitch_deg,
                        yaw_deg=yaw_deg,
                        roll_deg=roll_deg,
                        camera_height=height_m,
                        waypoint_shift=DEFAULTS.pp_shift,
                        y_sign=DRAW_Y_SIGN,
                        draw_bev=False,
                        draw_footer=False,
                        draw_wheel=False,
                    )
                    if meas_rad is not None:
                        status += f"  meas={np.rad2deg(meas_rad):+.1f}°"
                    self._set_lk_status(status)
                else:
                    self._set_lk_status("no path")

                if meas_rad is not None:
                    draw_steering_wheel(img, meas_rad)
                    cv2.putText(
                        img,
                        "meas SWA",
                        (img.shape[1] - 110, img.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (180, 180, 180),
                        1,
                        cv2.LINE_AA,
                    )
        elif self.lk_overlay_var.get():
            self._set_lk_status("LK HUD needs Lanes overlay")

        # Sync dt readout for debugging lag
        if len(self.timestamps) > 0 and index < len(self.timestamps):
            dt = ts_cam - float(self.timestamps[index])
            cv2.putText(
                img,
                f"cam-state dt={dt:+.0f}ms  img#{img_index}  "
                f"RPY={roll_deg:.1f}/{pitch_deg:.1f}/{yaw_deg:.1f}  "
                f"h={height_m:.2f} x={cam_x:.2f} y={cam_y:.2f}",
                (8, h - 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )

        from PIL import Image, ImageTk

        canvas_width = max(1, self.camera_canvas.winfo_width())
        canvas_height = max(1, self.camera_canvas.winfo_height())
        h, w = img.shape[:2]
        scale = min(canvas_width / w, canvas_height / h)
        new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
        img_rgb = cv2.cvtColor(cv2.resize(img, (new_w, new_h)), cv2.COLOR_BGR2RGB)

        photo = ImageTk.PhotoImage(image=Image.fromarray(img_rgb))
        self.camera_canvas.delete("all")
        self.camera_canvas.create_image(
            (canvas_width - new_w) // 2,
            (canvas_height - new_h) // 2,
            image=photo,
            anchor=tk.NW,
        )
        self.camera_canvas.image = photo

    def _set_lk_status(self, text: str) -> None:
        if hasattr(self, "lk_status"):
            self.lk_status.config(text=text)

    def update_sensor_data(self, index: int) -> None:
        if len(self.timestamps) == 0:
            return
        t = float(self.timestamps[index])

        imu = _nearest_row(self.imu_data, t)
        if imu is not None:
            text = (
                f"Accel: ax={imu[1]:6.2f} ay={imu[2]:6.2f} az={imu[3]:6.2f} m/s²\n"
                f"Gyro:  gx={imu[4]:6.3f} gy={imu[5]:6.3f} gz={imu[6]:6.3f} rad/s\n"
                f"Mag:   mx={imu[7]:6.1f} my={imu[8]:6.1f} mz={imu[9]:6.1f}"
            )
            self._set_text(self.imu_text, text)

        gps = _nearest_row(self.gps_data, t)
        if gps is not None:
            text = (
                f"Lat:   {gps[1]:12.8f}°\n"
                f"Lon:   {gps[2]:12.8f}°\n"
                f"Alt:   {gps[3]:8.2f} m\n"
                f"Speed: {gps[4]:6.2f} m/s"
            )
            self._set_text(self.gps_text, text)

        if self.wheel_data is not None and index < len(self.wheel_data):
            wh = self.wheel_data[index]
            # stored as vr,vl,hr,hl km/h (= fr,fl,rr,rl)
            text = (
                f"Wheels km/h: FR={wh[1]:5.1f} FL={wh[2]:5.1f}\n"
                f"             RR={wh[3]:5.1f} RL={wh[4]:5.1f}\n"
            )
            if self.steering_data is not None and index < len(self.steering_data):
                st = self.steering_data[index]
                deg = st[1] * _UNIT_TO_DEG
                if st[2] != 0:
                    deg = -deg
                text += f"Steer: {deg:7.1f} deg\n"
            if self.gear_data is not None and index < len(self.gear_data):
                name = str(self.gear_data[index][1]).upper()
                short = {
                    "PARK": "P",
                    "REVERSE": "R",
                    "NEUTRAL": "N",
                    "DRIVE": "D",
                    "SPORT": "S",
                    "ECO": "E",
                }.get(name, name[:1] if name else "?")
                text += f"Gear:  {short} ({name})"
            self._set_text(self.odom_text, text)

    @staticmethod
    def _set_text(widget: tk.Text, text: str) -> None:
        widget.config(state=tk.NORMAL)
        widget.delete(1.0, tk.END)
        widget.insert(1.0, text)
        widget.config(state=tk.DISABLED)

    def update_time_label(self, index: int) -> None:
        if len(self.timestamps) == 0:
            return
        elapsed = self.timestamps[index] - self.timestamps[0]
        duration = self.timestamps[-1] - self.timestamps[0]
        self.time_label.config(text=f"{_format_ms(elapsed)} / {_format_ms(duration)}")

    def find_closest_image_by_time(self, target_time: float) -> Optional[int]:
        """Latest camera frame with ts <= target (avoids showing a future image)."""
        if len(self.image_timestamps) == 0:
            return None
        idx = int(np.searchsorted(self.image_timestamps, target_time, side="right")) - 1
        if idx < 0:
            return 0
        if idx >= len(self.image_timestamps):
            return len(self.image_timestamps) - 1
        return idx

    def on_timeline_change(self, value) -> None:
        try:
            self.update_display(int(float(value)))
        except Exception as e:
            print(f"timeline error: {e}")

    def on_speed_change(self, *args) -> None:
        self.play_speed = float(self.speed_var.get())
        self.speed_label.config(text=f"{self.play_speed:.1f}x")

    def play(self) -> None:
        if not self.trajectories and len(self.timestamps) == 0:
            messagebox.showwarning("Warning", "Load a bag first")
            return
        self.playing = True
        self._play_last_wall_ms = None
        self.animate()

    def pause(self) -> None:
        self.playing = False
        self._play_last_wall_ms = None

    def restart(self) -> None:
        self.playing = False
        self._play_last_wall_ms = None
        self.time_var.set(0)
        self.update_display(0)

    def animate(self) -> None:
        if not self.playing:
            return
        import time

        wall_ms = time.monotonic() * 1000.0
        tick_ms = 50.0
        if self._play_last_wall_ms is None:
            self._play_last_wall_ms = wall_ms
            bag_dt = tick_ms * self.play_speed
        else:
            wall_dt = max(1.0, wall_ms - self._play_last_wall_ms)
            self._play_last_wall_ms = wall_ms
            # Real-time: 1.0x → advance bag by wall_dt milliseconds
            bag_dt = wall_dt * self.play_speed

        current = int(self.time_var.get())
        max_val = int(float(self.timeline_slider.cget("to")))
        if current >= max_val or len(self.timestamps) == 0:
            self.playing = False
            self.status_label.config(text="Playback finished", foreground="blue")
            return

        t_now = float(self.timestamps[current])
        t_target = t_now + bag_dt
        # Advance timeline index to cover bag time (vehicle/state clock)
        nxt = current
        while nxt < max_val and float(self.timestamps[nxt]) < t_target:
            nxt += 1
        if nxt == current and current < max_val:
            nxt = current + 1
        nxt = min(nxt, max_val)
        self.time_var.set(nxt)
        self.update_display(nxt)
        self.master.after(int(tick_ms), self.animate)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", nargs="?", help="Session dir or .zip/.tar.gz")
    ap.add_argument("-i", "--input", help="Alias for bag path")
    ap.add_argument("--osm-map", help="Compact OSM map (.admap) for the background layer")
    args = ap.parse_args()
    bag = args.bag or args.input

    root = tk.Tk()
    InteractiveVisualizer(root, bag_path=bag, osm_map=args.osm_map)
    print("Interactive visualizer started (Open bag / Play / Pause / Restart)")
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
