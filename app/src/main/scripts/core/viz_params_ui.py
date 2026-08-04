#!/usr/bin/env python3
"""Shared RPY / Pure Pursuit Tk controls for bag visualizer and MetaDrive sim."""

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Optional

import tkinter as tk
from tkinter import ttk

from .lane_keep import DEFAULTS


@dataclass
class OverlayUiParams:
    """Live overlay / controller params (same ranges as bag InteractiveVisualizer).

    Defaults match ``app/src/main/assets/config.json`` camera priors.
    """

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    height_m: float = 1.40
    cam_x: float = 1.50
    cam_y_left: float = 0.0
    pp_k_dd: float = DEFAULTS.pp_k_dd
    pp_ld_min: float = DEFAULTS.pp_ld_min
    pp_ld_max: float = DEFAULTS.pp_ld_max
    wheelbase: float = DEFAULTS.wheelbase
    pp_shift: float = DEFAULTS.pp_shift

    def clamped_pp(self) -> "OverlayUiParams":
        ld_min, ld_max = self.pp_ld_min, self.pp_ld_max
        if ld_min > ld_max:
            ld_min, ld_max = ld_max, ld_min
        return replace(self, pp_ld_min=ld_min, pp_ld_max=ld_max)


def assets_config_path() -> Path:
    """``app/src/main/assets/config.json`` relative to this package."""
    # core/viz_params_ui.py → scripts → main → assets
    return Path(__file__).resolve().parents[2] / "assets" / "config.json"


def load_camera_priors(path: Optional[Path] = None) -> OverlayUiParams:
    """Load camera RPY / mount priors from assets config (fallback = dataclass defaults)."""
    base = OverlayUiParams()
    cfg = Path(path) if path is not None else assets_config_path()
    try:
        data = json.loads(cfg.read_text())
        cam = (data.get("calibration") or {}).get("camera") or {}
        pos = cam.get("position_m") or {}
        rpy = cam.get("rpy_deg") or {}
        return OverlayUiParams(
            roll_deg=float(rpy.get("roll", base.roll_deg)),
            pitch_deg=float(rpy.get("pitch", base.pitch_deg)),
            yaw_deg=float(rpy.get("yaw", base.yaw_deg)),
            height_m=float(pos.get("z_up", base.height_m)),
            cam_x=float(pos.get("x_forward", base.cam_x)),
            cam_y_left=float(pos.get("y_left", base.cam_y_left)),
        )
    except Exception:
        return base


class RpyPpControlBar:
    """Two rows of sliders: Roll/Pitch/Yaw/Height/X/Y and K_dd/Ld_min/Ld_max/L_wb/shift."""

    def __init__(
        self,
        master: tk.Misc,
        initial: OverlayUiParams,
        *,
        on_rpy: Optional[Callable[[OverlayUiParams], None]] = None,
        on_pp: Optional[Callable[[OverlayUiParams], None]] = None,
        pack: bool = True,
    ) -> None:
        self._on_rpy = on_rpy
        self._on_pp = on_pp
        self._suppress = False

        self.rpy_frame = ttk.Frame(master)
        self.pp_frame = ttk.Frame(master)
        if pack:
            self.rpy_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=(0, 4))
            self.pp_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=(0, 4))

        self.roll_var = tk.DoubleVar(value=initial.roll_deg)
        self.pitch_var = tk.DoubleVar(value=initial.pitch_deg)
        self.yaw_var = tk.DoubleVar(value=initial.yaw_deg)
        self.height_var = tk.DoubleVar(value=initial.height_m)
        self.cam_x_var = tk.DoubleVar(value=initial.cam_x)
        self.cam_y_var = tk.DoubleVar(value=initial.cam_y_left)

        self.roll_label = self._add_rpy_slider(
            self.rpy_frame, "Roll", self.roll_var, -15.0, 15.0
        )
        self.pitch_label = self._add_rpy_slider(
            self.rpy_frame, "Pitch", self.pitch_var, -20.0, 10.0
        )
        self.yaw_label = self._add_rpy_slider(
            self.rpy_frame, "Yaw", self.yaw_var, -20.0, 20.0
        )
        self.height_label = self._add_rpy_slider(
            self.rpy_frame, "Height", self.height_var, 0.40, 2.20, unit="m", length=120
        )
        self.cam_x_label = self._add_rpy_slider(
            self.rpy_frame, "X", self.cam_x_var, 0.0, 3.0, unit="m", length=120
        )
        self.cam_y_label = self._add_rpy_slider(
            self.rpy_frame, "Y", self.cam_y_var, -1.0, 1.0, unit="m", length=120
        )
        ttk.Button(self.rpy_frame, text="Reset RPY", command=self.reset_rpy).pack(
            side=tk.LEFT, padx=12
        )

        self.pp_kdd_var = tk.DoubleVar(value=initial.pp_k_dd)
        self.pp_ld_min_var = tk.DoubleVar(value=initial.pp_ld_min)
        self.pp_ld_max_var = tk.DoubleVar(value=initial.pp_ld_max)
        self.pp_wb_var = tk.DoubleVar(value=initial.wheelbase)
        self.pp_shift_var = tk.DoubleVar(value=initial.pp_shift)

        # Algorithm first — PP row is wide; combo at the end was easy to miss/clip.
        self.controller_var = tk.StringVar(value="pure_pursuit")
        ttk.Label(self.pp_frame, text="LK").pack(side=tk.LEFT, padx=(4, 2))
        self.controller_pp_rb = ttk.Radiobutton(
            self.pp_frame,
            text="PP",
            value="pure_pursuit",
            variable=self.controller_var,
            command=self._fire_pp,
        )
        self.controller_pp_rb.pack(side=tk.LEFT)
        self.controller_mpc_rb = ttk.Radiobutton(
            self.pp_frame,
            text="MPC",
            value="mpc",
            variable=self.controller_var,
            command=self._fire_pp,
        )
        self.controller_mpc_rb.pack(side=tk.LEFT, padx=(0, 8))
        # Keep combo API for any callers that still touch it.
        self.controller_combo = self.controller_pp_rb

        self.pp_kdd_label = self._add_pp_slider(
            self.pp_frame, "K_dd", self.pp_kdd_var, 0.05, 1.5, "{:.2f}"
        )
        self.pp_ld_min_label = self._add_pp_slider(
            self.pp_frame, "Ld_min", self.pp_ld_min_var, 1.0, 15.0, "{:.1f}"
        )
        self.pp_ld_max_label = self._add_pp_slider(
            self.pp_frame, "Ld_max", self.pp_ld_max_var, 5.0, 40.0, "{:.1f}"
        )
        self.pp_wb_label = self._add_pp_slider(
            self.pp_frame, "L_wb", self.pp_wb_var, 2.0, 3.5, "{:.2f}"
        )
        self.pp_shift_label = self._add_pp_slider(
            self.pp_frame, "shift", self.pp_shift_var, 0.0, 3.0, "{:.2f}"
        )
        ttk.Button(self.pp_frame, text="Reset PP", command=self.reset_pp).pack(
            side=tk.LEFT, padx=10
        )
        self.pp_status = ttk.Label(self.pp_frame, text="PP —")
        self.pp_status.pack(side=tk.LEFT, padx=8)

        self._lk_widgets = [
            self.controller_pp_rb,
            self.controller_mpc_rb,
            self.pp_kdd_label,
            self.pp_ld_min_label,
            self.pp_ld_max_label,
            self.pp_wb_label,
            self.pp_shift_label,
        ]
        # Scale widgets are children created in _add_pp_slider; collect Scales + Reset.
        for child in self.pp_frame.winfo_children():
            if isinstance(child, (ttk.Scale, ttk.Button, ttk.Radiobutton, ttk.Label)):
                if child is self.pp_status:
                    continue
                if child not in self._lk_widgets:
                    self._lk_widgets.append(child)

        self._rpy_defaults = OverlayUiParams(
            roll_deg=initial.roll_deg,
            pitch_deg=initial.pitch_deg,
            yaw_deg=initial.yaw_deg,
            height_m=initial.height_m,
            cam_x=initial.cam_x,
            cam_y_left=initial.cam_y_left,
        )
        self._pp_defaults = OverlayUiParams(
            pp_k_dd=initial.pp_k_dd,
            pp_ld_min=initial.pp_ld_min,
            pp_ld_max=initial.pp_ld_max,
            wheelbase=initial.wheelbase,
            pp_shift=initial.pp_shift,
        )

    def _add_rpy_slider(
        self,
        parent: tk.Misc,
        name: str,
        var: tk.DoubleVar,
        lo: float,
        hi: float,
        unit: str = "°",
        length: int = 160,
    ) -> ttk.Label:
        ttk.Label(parent, text=f"{name}:").pack(side=tk.LEFT, padx=(8, 2))
        ttk.Scale(
            parent,
            from_=lo,
            to=hi,
            orient=tk.HORIZONTAL,
            variable=var,
            length=length,
            command=lambda _v: self._fire_rpy(),
        ).pack(side=tk.LEFT, padx=2)
        text = f"{var.get():.2f}{unit}" if unit == "m" else f"{var.get():.1f}{unit}"
        lab = ttk.Label(parent, text=text, width=8)
        lab.pack(side=tk.LEFT, padx=2)
        return lab

    def _add_pp_slider(
        self,
        parent: tk.Misc,
        name: str,
        var: tk.DoubleVar,
        lo: float,
        hi: float,
        fmt: str,
    ) -> ttk.Label:
        ttk.Label(parent, text=f"{name}:").pack(side=tk.LEFT, padx=(8, 2))
        ttk.Scale(
            parent,
            from_=lo,
            to=hi,
            orient=tk.HORIZONTAL,
            variable=var,
            length=120,
            command=lambda _v: self._fire_pp(),
        ).pack(side=tk.LEFT, padx=2)
        lab = ttk.Label(parent, text=fmt.format(var.get()), width=7)
        lab.pack(side=tk.LEFT, padx=2)
        return lab

    def params(self) -> OverlayUiParams:
        return OverlayUiParams(
            roll_deg=float(self.roll_var.get()),
            pitch_deg=float(self.pitch_var.get()),
            yaw_deg=float(self.yaw_var.get()),
            height_m=float(self.height_var.get()),
            cam_x=float(self.cam_x_var.get()),
            cam_y_left=float(self.cam_y_var.get()),
            pp_k_dd=float(self.pp_kdd_var.get()),
            pp_ld_min=float(self.pp_ld_min_var.get()),
            pp_ld_max=float(self.pp_ld_max_var.get()),
            wheelbase=float(self.pp_wb_var.get()),
            pp_shift=float(self.pp_shift_var.get()),
        ).clamped_pp()

    def set_rpy(
        self,
        *,
        roll_deg: Optional[float] = None,
        pitch_deg: Optional[float] = None,
        yaw_deg: Optional[float] = None,
        height_m: Optional[float] = None,
        cam_x: Optional[float] = None,
        cam_y_left: Optional[float] = None,
        notify: bool = False,
    ) -> None:
        self._suppress = True
        try:
            if roll_deg is not None:
                self.roll_var.set(roll_deg)
            if pitch_deg is not None:
                self.pitch_var.set(pitch_deg)
            if yaw_deg is not None:
                self.yaw_var.set(yaw_deg)
            if height_m is not None:
                self.height_var.set(height_m)
            if cam_x is not None:
                self.cam_x_var.set(cam_x)
            if cam_y_left is not None:
                self.cam_y_var.set(cam_y_left)
            self._refresh_rpy_labels()
        finally:
            self._suppress = False
        if notify and self._on_rpy is not None:
            self._on_rpy(self.params())

    def set_pp_status(self, text: str) -> None:
        self.pp_status.config(text=text)

    def set_lane_keep_controls_enabled(self, enabled: bool) -> None:
        """Enable PP/MPC tuning widgets (Live); disable in Bag playback."""
        state = tk.NORMAL if enabled else tk.DISABLED
        for w in getattr(self, "_lk_widgets", []):
            try:
                w.configure(state=state)
            except tk.TclError:
                pass

    def controller_mode(self) -> str:
        return str(self.controller_var.get())

    def set_controller_mode(self, mode: str) -> None:
        if mode in ("pure_pursuit", "mpc"):
            self.controller_var.set(mode)

    def reset_rpy(self) -> None:
        d = self._rpy_defaults
        self.set_rpy(
            roll_deg=d.roll_deg,
            pitch_deg=d.pitch_deg,
            yaw_deg=d.yaw_deg,
            height_m=d.height_m,
            cam_x=d.cam_x,
            cam_y_left=d.cam_y_left,
            notify=True,
        )

    def reset_pp(self) -> None:
        d = self._pp_defaults
        self._suppress = True
        try:
            self.pp_kdd_var.set(d.pp_k_dd)
            self.pp_ld_min_var.set(d.pp_ld_min)
            self.pp_ld_max_var.set(d.pp_ld_max)
            self.pp_wb_var.set(d.wheelbase)
            self.pp_shift_var.set(d.pp_shift)
            self._refresh_pp_labels()
        finally:
            self._suppress = False
        if self._on_pp is not None:
            self._on_pp(self.params())

    def set_rpy_defaults_from_current(self) -> None:
        p = self.params()
        self._rpy_defaults = OverlayUiParams(
            roll_deg=p.roll_deg,
            pitch_deg=p.pitch_deg,
            yaw_deg=p.yaw_deg,
            height_m=p.height_m,
            cam_x=p.cam_x,
            cam_y_left=p.cam_y_left,
        )

    def set_rpy_defaults(self, defaults: OverlayUiParams) -> None:
        """Replace Reset-RPY baseline (e.g. after loading session calib_rpy.json)."""
        self._rpy_defaults = OverlayUiParams(
            roll_deg=defaults.roll_deg,
            pitch_deg=defaults.pitch_deg,
            yaw_deg=defaults.yaw_deg,
            height_m=defaults.height_m,
            cam_x=defaults.cam_x,
            cam_y_left=defaults.cam_y_left,
        )

    def update_rpy_defaults(self, **kwargs: float) -> None:
        """Patch Reset-RPY baseline fields (e.g. pitch/yaw after VP commit)."""
        d = self._rpy_defaults
        self._rpy_defaults = OverlayUiParams(
            roll_deg=float(kwargs.get("roll_deg", d.roll_deg)),
            pitch_deg=float(kwargs.get("pitch_deg", d.pitch_deg)),
            yaw_deg=float(kwargs.get("yaw_deg", d.yaw_deg)),
            height_m=float(kwargs.get("height_m", d.height_m)),
            cam_x=float(kwargs.get("cam_x", d.cam_x)),
            cam_y_left=float(kwargs.get("cam_y_left", d.cam_y_left)),
        )

    def _refresh_rpy_labels(self) -> None:
        self.roll_label.config(text=f"{float(self.roll_var.get()):.1f}°")
        self.pitch_label.config(text=f"{float(self.pitch_var.get()):.1f}°")
        self.yaw_label.config(text=f"{float(self.yaw_var.get()):.1f}°")
        self.height_label.config(text=f"{float(self.height_var.get()):.2f}m")
        self.cam_x_label.config(text=f"{float(self.cam_x_var.get()):.2f}m")
        self.cam_y_label.config(text=f"{float(self.cam_y_var.get()):.2f}m")

    def _refresh_pp_labels(self) -> None:
        self.pp_kdd_label.config(text=f"{float(self.pp_kdd_var.get()):.2f}")
        self.pp_ld_min_label.config(text=f"{float(self.pp_ld_min_var.get()):.1f}")
        self.pp_ld_max_label.config(text=f"{float(self.pp_ld_max_var.get()):.1f}")
        self.pp_wb_label.config(text=f"{float(self.pp_wb_var.get()):.2f}")
        self.pp_shift_label.config(text=f"{float(self.pp_shift_var.get()):.2f}")

    def _fire_rpy(self) -> None:
        if self._suppress:
            return
        self._refresh_rpy_labels()
        if self._on_rpy is not None:
            self._on_rpy(self.params())

    def _fire_pp(self) -> None:
        if self._suppress:
            return
        self._refresh_pp_labels()
        if self._on_pp is not None:
            self._on_pp(self.params())
