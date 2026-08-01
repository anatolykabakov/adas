#!/usr/bin/env python3
"""Bag-style Tk live UI for MetaDrive sim (camera + trajectory + live params)."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Optional

import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

from core.viz_params_ui import OverlayUiParams, RpyPpControlBar

if TYPE_CHECKING:
    from sim.main import MetaDriveSimulator


class SimLiveUi:
    """Tk layout mirrored from ``vis.interactive_visualizer.InteractiveVisualizer``."""

    def __init__(self, sim: "MetaDriveSimulator") -> None:
        self.sim = sim
        self.args = sim.args
        self._alive = True
        self._quit_requested = False
        self._photo_cam: Optional[ImageTk.PhotoImage] = None
        self._photo_traj: Optional[ImageTk.PhotoImage] = None

        self.root = tk.Tk()
        self.root.title("ADAS Sim — camera / trajectory / params")
        self.root.geometry("1400x900")
        self.root.protocol("WM_DELETE_WINDOW", self.request_quit)

        # --- toolbar ---
        control = ttk.Frame(self.root)
        control.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        self.supercombo_var = tk.BooleanVar(value=bool(self.args.draw_supercombo))
        ttk.Checkbutton(
            control, text="Supercombo overlay", variable=self.supercombo_var
        ).pack(side=tk.LEFT, padx=10)

        lane_src = ttk.LabelFrame(control, text="Lanes", padding=(4, 0))
        lane_src.pack(side=tk.LEFT, padx=(4, 8))
        self.lane_source_var = tk.StringVar(value=str(self.args.lanes))
        ttk.Radiobutton(
            lane_src,
            text="GT",
            value="gt",
            variable=self.lane_source_var,
            command=self._on_lane_source,
        ).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(
            lane_src,
            text="Supercombo",
            value="supercombo",
            variable=self.lane_source_var,
            command=self._on_lane_source,
        ).pack(side=tk.LEFT, padx=2)

        self.vp_calib_var = tk.BooleanVar(value=bool(sim.vp_enabled))
        ttk.Checkbutton(
            control,
            text="VP calib (AAD)",
            variable=self.vp_calib_var,
            command=self._on_vp_toggle,
        ).pack(side=tk.LEFT, padx=6)
        self.vp_debug_var = tk.BooleanVar(value=bool(sim.vp_debug))
        ttk.Checkbutton(
            control,
            text="VP debug",
            variable=self.vp_debug_var,
            command=self._on_vp_debug,
        ).pack(side=tk.LEFT, padx=4)
        self.pp_var = tk.BooleanVar(value=self.args.controller != "straight")
        ttk.Checkbutton(
            control, text="Pure pursuit", variable=self.pp_var, command=self._on_pp_toggle
        ).pack(side=tk.LEFT, padx=6)
        self.gt_lanes_var = tk.BooleanVar(value=bool(self.args.draw_gt_lanes))
        ttk.Checkbutton(
            control,
            text="GT lanes",
            variable=self.gt_lanes_var,
            command=self._on_gt_lanes,
        ).pack(side=tk.LEFT, padx=4)
        self.bev_var = tk.BooleanVar(value=False)  # bag-style default
        ttk.Checkbutton(control, text="BEV inset", variable=self.bev_var).pack(
            side=tk.LEFT, padx=4
        )

        ttk.Button(control, text="Reset VP", command=self._on_reset_vp).pack(
            side=tk.LEFT, padx=4
        )
        self.vp_status = ttk.Label(control, text="RPY —")
        self.vp_status.pack(side=tk.LEFT, padx=8)

        self.status_label = ttk.Label(control, text="Running", foreground="green")
        self.status_label.pack(side=tk.RIGHT, padx=10)

        # --- RPY / PP (shared with bag) ---
        initial = OverlayUiParams(
            roll_deg=sim.roll_deg,
            pitch_deg=sim.pitch_deg,
            yaw_deg=sim.yaw_deg,
            height_m=sim.camera_height,
            pp_k_dd=float(self.args.pp_k_dd),
            pp_ld_min=float(self.args.pp_ld_min),
            pp_ld_max=float(self.args.pp_ld_max),
            wheelbase=float(self.args.wheelbase),
            pp_shift=float(self.args.pp_shift),
        )
        self.params_bar = RpyPpControlBar(
            self.root,
            initial,
            on_rpy=self._on_rpy,
            on_pp=self._on_pp,
        )
        self.params_bar.set_rpy_defaults_from_current()

        # --- main: trajectory | camera ---
        main = ttk.Frame(self.root)
        main.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        ttk.Label(
            left, text="Trajectory (GT / Odom / EKF)", font=("Arial", 12, "bold")
        ).pack(pady=4)
        self.traj_canvas = tk.Canvas(left, bg="black", width=480, height=480)
        self.traj_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        ttk.Label(right, text="Camera", font=("Arial", 14, "bold")).pack(pady=5)
        self.camera_canvas = tk.Canvas(right, bg="black", width=640, height=480)
        self.camera_canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- sensors / status strip ---
        sensors = ttk.LabelFrame(self.root, text="Status", padding=8)
        sensors.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)
        self.info_text = tk.Text(sensors, height=3, font=("Courier", 9))
        self.info_text.pack(fill=tk.BOTH, expand=True)
        self.info_text.config(state=tk.DISABLED)

        hint = ttk.Label(
            self.root,
            text="Sliders update overlay + controller live  ·  close window or Ctrl+C to quit",
            foreground="gray",
        )
        hint.pack(side=tk.BOTTOM, pady=(0, 4))

    @property
    def alive(self) -> bool:
        return self._alive and not self._quit_requested

    def request_quit(self) -> None:
        self._quit_requested = True
        self._alive = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def pump(self) -> bool:
        if not self.alive:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self._alive = False
            return False
        return self.alive

    def draw_supercombo(self) -> bool:
        return bool(self.supercombo_var.get())

    def draw_gt_lanes(self) -> bool:
        return bool(self.gt_lanes_var.get())

    def draw_bev(self) -> bool:
        return bool(self.bev_var.get())

    def pp_enabled(self) -> bool:
        return bool(self.pp_var.get())

    def show_frame(
        self,
        camera_bgr: np.ndarray,
        traj_bgr: Optional[np.ndarray] = None,
        status: str = "",
    ) -> None:
        if not self.alive:
            return
        self._set_canvas_image(self.camera_canvas, camera_bgr, which="cam")
        if traj_bgr is not None:
            self._set_canvas_image(self.traj_canvas, traj_bgr, which="traj")
        self._set_info(status)
        self._update_vp_status()

    def sync_rpy_from_sim(self) -> None:
        """After VP commit / reset — keep sliders aligned with sim state."""
        s = self.sim
        self.params_bar.set_rpy(
            roll_deg=s.roll_deg,
            pitch_deg=s.pitch_deg,
            yaw_deg=s.yaw_deg,
            height_m=s.camera_height,
            notify=False,
        )
        self._update_vp_status()

    def _set_canvas_image(self, canvas: tk.Canvas, bgr: np.ndarray, which: str) -> None:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        cw = max(int(canvas.winfo_width()), 320)
        ch = max(int(canvas.winfo_height()), 240)
        h, w = rgb.shape[:2]
        scale = min(cw / w, ch / h)
        nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
        if (nw, nh) != (w, h):
            rgb = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_AREA)
        photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        if which == "cam":
            self._photo_cam = photo
        else:
            self._photo_traj = photo
        canvas.delete("all")
        canvas.create_image(cw // 2, ch // 2, image=photo, anchor=tk.CENTER)

    def _set_info(self, text: str) -> None:
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete("1.0", tk.END)
        self.info_text.insert(tk.END, text)
        self.info_text.config(state=tk.DISABLED)

    def _update_vp_status(self) -> None:
        s = self.sim
        ok = s.vp_calib.calibration_success
        n = len(s.vp_calib.pitch_yaw_history)
        tag = "OK" if ok else f"…{n}/{s.vp_calib.history_len}"
        self.vp_status.config(
            text=(f"VP {tag}  R={s.roll_deg:.1f} P={s.pitch_deg:.1f} Y={s.yaw_deg:.1f}°"),
            foreground="green" if ok else "orange",
        )

    def _on_rpy(self, p: OverlayUiParams) -> None:
        self.sim.apply_overlay_rpy(p)

    def _on_pp(self, p: OverlayUiParams) -> None:
        self.sim.apply_pp_params(p)
        self.params_bar.set_pp_status(
            f"K={p.pp_k_dd:.2f} Ld=[{p.pp_ld_min:.1f},{p.pp_ld_max:.1f}] shift={p.pp_shift:.2f}"
        )

    def _on_lane_source(self) -> None:
        self.sim.set_lane_source(self.lane_source_var.get())

    def _on_vp_toggle(self) -> None:
        self.sim.vp_enabled = bool(self.vp_calib_var.get())

    def _on_vp_debug(self) -> None:
        self.sim.vp_debug = bool(self.vp_debug_var.get())

    def _on_gt_lanes(self) -> None:
        self.args.draw_gt_lanes = bool(self.gt_lanes_var.get())

    def _on_pp_toggle(self) -> None:
        self.sim.set_pp_enabled(bool(self.pp_var.get()))

    def _on_reset_vp(self) -> None:
        self.sim._reset_vp()
        self.sync_rpy_from_sim()
