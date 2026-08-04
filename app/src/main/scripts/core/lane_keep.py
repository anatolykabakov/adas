"""Lane keeping for bag/sim visualizers via Simulated ``pyadas.AdasApp``.

``pure_pursuit`` / ``mpc`` / ``fp`` / ``straight`` — publish chassis/lanes, ``step``, read state.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
from pyadas import require_core

pyadas = require_core()

from .pure_pursuit import PurePursuitResult, pp_result_from_output

CONFIG_JSON = Path(__file__).resolve().parents[2] / "assets" / "config.json"


def load_vehicle_config(path: Optional[Path] = None) -> Dict[str, Any]:
    """``vehicle`` block of the shipped config.json — what actually runs on the phone."""
    p = Path(path) if path is not None else CONFIG_JSON
    with p.open(encoding="utf-8") as f:
        return json.load(f).get("vehicle", {})


@dataclass(frozen=True)
class LaneKeepDefaults:
    speed_mps: float = 12.0
    max_steer_deg: float = 8.0
    wheelbase: float = 2.636
    pp_k_dd: float = 0.4
    pp_ld_min: float = 3.0
    pp_ld_max: float = 20.0
    pp_shift: float = 1.40
    speed_kp: float = 0.08
    mpc_Lf: float = 2.67
    mpc_max_steer_deg: float = 25.0


DEFAULTS = LaneKeepDefaults()


@dataclass
class LaneKeepResult:
    mode: str
    steer_rad: float
    steer_norm: float
    throttle: float
    brake: float
    polyline: Optional[np.ndarray]
    e_y: float = 0.0
    e_psi: float = 0.0
    curvature: float = 0.0
    pure_pursuit: Optional[PurePursuitResult] = None
    status: str = "ok"
    controller: str = "pp"


def build_centerline_polyline(
    left_road: np.ndarray,
    right_road: np.ndarray,
    x_min: float = 1.0,
    x_max: float = 40.0,
    n: int = 48,
) -> Optional[np.ndarray]:
    left = np.asarray(left_road, dtype=np.float64)
    right = np.asarray(right_road, dtype=np.float64)
    if left.ndim != 2 or right.ndim != 2 or left.shape[0] < 2 or right.shape[0] < 2:
        return None

    x_lo = max(x_min, float(max(left[:, 0].min(), right[:, 0].min())))
    x_hi = min(x_max, float(min(left[:, 0].max(), right[:, 0].max())))
    if x_hi <= x_lo + 1.0:
        return None

    xs = np.linspace(x_lo, x_hi, n)
    y_left = np.interp(xs, left[:, 0], left[:, 1])
    y_right = np.interp(xs, right[:, 0], right[:, 1])
    return np.stack([xs, 0.5 * (y_left + y_right)], axis=1)


def lane_keep_result_from_bag(
    msg: Any,
    polyline: Optional[np.ndarray] = None,
    *,
    speed_mps: float = 0.0,
    waypoint_shift: float = 1.4,
    wheel_base: float = 2.636,
) -> LaneKeepResult:
    """Build a viz ``LaneKeepResult`` from logged ``control/lane_keep`` (LaneKeepState)."""
    status_raw = str(getattr(msg, "status", "") or "")
    controller = "pp"
    status = status_raw
    if ":" in status_raw:
        status, tail = status_raw.split(":", 1)
        controller = tail.strip() or "pp"
    elif status_raw in ("pp", "mpc"):
        controller = status_raw
        status = "ok"

    # Heuristic for bags recorded before status:"ok:pp|mpc"
    if controller == "pp" and abs(float(getattr(msg, "lookahead_m", 0.0) or 0.0)) < 1e-6:
        if abs(float(getattr(msg, "target_x", 0.0) or 0.0)) < 5.0 and getattr(
            msg, "has_target", False
        ):
            controller = "mpc"

    steer_rad = float(getattr(msg, "steer_rad", 0.0) or 0.0)
    steer_norm = float(getattr(msg, "steer_norm", 0.0) or 0.0)
    cte = 0.0
    epsi = 0.0
    pp = None
    mode = "mpc" if controller == "mpc" else "pure_pursuit"
    if controller == "mpc":
        cte = float(getattr(msg, "target_x", 0.0) or 0.0)
        epsi = float(getattr(msg, "target_y", 0.0) or 0.0)
    else:
        pp = pp_result_from_output(
            msg,
            polyline if polyline is not None else np.zeros((0, 2), dtype=np.float64),
            speed_mps,
            waypoint_shift=waypoint_shift,
            wheel_base=wheel_base,
        )

    return LaneKeepResult(
        mode=mode,
        steer_rad=steer_rad,
        steer_norm=steer_norm,
        throttle=float(getattr(msg, "throttle", 0.0) or 0.0),
        brake=float(getattr(msg, "brake", 0.0) or 0.0),
        polyline=polyline,
        e_y=cte,
        e_psi=epsi,
        curvature=float(getattr(msg, "curvature", 0.0) or 0.0),
        pure_pursuit=pp,
        status=status or "ok",
        controller=controller,
    )


def format_lane_keep_status(lk: LaneKeepResult) -> str:
    if lk.mode == "mpc" or lk.controller == "mpc":
        return (
            f"MPC δ={np.rad2deg(lk.steer_rad):+.1f}° CTE={lk.e_y:+.2f}m "
            f"epsi={np.rad2deg(lk.e_psi):+.1f}° κ={lk.curvature:.4f}/m  {lk.status}"
        )
    if lk.mode == "pure_pursuit" and lk.pure_pursuit is not None:
        pp = lk.pure_pursuit
        tgt = (
            f"({pp.target_ego[0]:.1f},{pp.target_ego[1]:.1f})"
            if pp.target_ego is not None
            else "none"
        )
        return (
            f"PP Ld={pp.lookahead_m:.1f}m δ={np.rad2deg(lk.steer_rad):.1f}° "
            f"κ={lk.curvature:.4f}/m tgt={tgt}"
        )
    return f"{lk.mode} δ={np.rad2deg(lk.steer_rad):+.1f}°  {lk.status}"


class LaneKeepController:
    """Sim/viz glue: publish inputs → AdasApp.step → pop_messages(LaneKeepOutput)."""

    MODES = ("straight", "pure_pursuit", "mpc", "fp")

    # Sim/viz mode → C++ LaneKeepService controller key.
    _CONTROLLER = {"straight": "pp", "pure_pursuit": "pp", "mpc": "mpc", "fp": "fp"}

    def __init__(
        self,
        mode: str = "pure_pursuit",
        desired_speed: float = DEFAULTS.speed_mps,
        max_steer_deg: float = DEFAULTS.max_steer_deg,
        wheelbase: float = DEFAULTS.wheelbase,
        pp_k_dd: float = DEFAULTS.pp_k_dd,
        pp_ld_min: float = DEFAULTS.pp_ld_min,
        pp_ld_max: float = DEFAULTS.pp_ld_max,
        pp_shift: float = DEFAULTS.pp_shift,
        speed_kp: float = DEFAULTS.speed_kp,
        mpc_max_steer_deg: float = DEFAULTS.mpc_max_steer_deg,
        cam_y_left_m: float = 0.0,
        dt_s: float = 0.05,
        vehicle_config: Optional[Dict[str, Any]] = None,
        app: Any = None,
    ):
        if mode not in self.MODES:
            raise ValueError(f"Unknown mode {mode!r}, expected one of {self.MODES}")
        self.mode = mode
        self.desired_speed = float(desired_speed)
        self.max_steer_rad = float(np.deg2rad(max_steer_deg))
        self.mpc_max_steer_rad = float(np.deg2rad(mpc_max_steer_deg))
        self.wheelbase = float(wheelbase)
        self.pp_shift = float(pp_shift)
        self.speed_kp = float(speed_kp)
        self.dt_s = float(dt_s)
        self._t_us = 0
        self._owns_app = app is None
        self._app = app or pyadas.AdasApp(wheelbase=float(wheelbase))
        self.apply_pp_params(
            pp_k_dd=pp_k_dd,
            pp_ld_min=pp_ld_min,
            pp_ld_max=pp_ld_max,
            pp_shift=pp_shift,
            max_steer_deg=max_steer_deg,
        )
        self.set_cam_y_left_m(cam_y_left_m)
        if vehicle_config:
            self.apply_vehicle_config(vehicle_config)
        self.set_mode(mode)
        self.last_result: Optional[LaneKeepResult] = None
        self.last_warn: Any = None

    def apply_vehicle_config(self, cfg: Dict[str, Any]) -> None:
        """Push the ``vehicle`` block of config.json into the C++ service.

        Without this the sim runs the C++ struct defaults, not the tuned values that ship
        in the APK, and any sim result says nothing about the car.
        """
        app = self._app
        get = cfg.get

        self.max_steer_rad = float(
            np.deg2rad(get("max_steer_deg", DEFAULTS.max_steer_deg))
        )
        self.mpc_max_steer_rad = float(
            np.deg2rad(get("mpc_max_steer_deg", DEFAULTS.mpc_max_steer_deg))
        )
        self.pp_shift = float(get("pp_shift", DEFAULTS.pp_shift))
        app.set_lane_keep_pp(
            float(get("pp_k_dd", DEFAULTS.pp_k_dd)),
            float(get("pp_ld_min", DEFAULTS.pp_ld_min)),
            float(get("pp_ld_max", DEFAULTS.pp_ld_max)),
            self.pp_shift,
        )
        app.set_lane_keep_pp_ld_curv_gain(float(get("pp_ld_curv_gain", 0.0)))
        app.set_lane_keep_mpc_kappa_yaw_blend(
            float(get("mpc_kappa_yaw_blend", 0.0)),
            float(get("mpc_kappa_yaw_min_speed", 3.0)),
        )
        app.set_lane_keep_mpc_ema_alphas(
            float(get("mpc_kappa_ema_alpha", 1.0)),
            float(get("mpc_epsi_ema_alpha", 1.0)),
            float(get("mpc_cte_ema_alpha", 1.0)),
        )
        app.set_lane_keep_steer_slew_limit_deg(float(get("steer_slew_limit_deg", 8.0)))
        app.set_lane_keep_vehicle_model(
            bool(get("lat_use_vehicle_model", True)),
            float(get("tire_stiffness_factor", 0.64)),
        )
        app.set_lane_keep_fp_steer_delay_s(float(get("fp_steer_delay_s", 0.35)))
        app.set_lane_keep_fp_steering_rate_weight(
            float(get("fp_steering_rate_weight", 400.0))
        )

        # Module-level VisionPilot knobs (the `mpc` seed/cost).
        pyadas.set_mpc_warm_start_gains(
            float(get("mpc_epsi_gain", 0.5)), float(get("mpc_ff_scale", 2.0))
        )
        pyadas.set_mpc_cost_weights(
            float(get("mpc_cte_weight_base", 20.0)),
            float(get("mpc_cte_quartic_scale", 5.0)),
        )
        pyadas.set_mpc_cte_gain_base(float(get("mpc_cte_gain_base", 0.6)))
        pyadas.set_mpc_cte_gain_floor(float(get("mpc_cte_gain_floor", 0.0)))

    def set_cam_y_left_m(self, m: float) -> None:
        self.cam_y_left_m = float(m)
        if hasattr(self._app, "set_lane_keep_cam_y_left_m"):
            self._app.set_lane_keep_cam_y_left_m(self.cam_y_left_m)

    def set_mode(self, mode: str) -> None:
        if mode not in self.MODES:
            raise ValueError(f"Unknown mode {mode!r}")
        self.mode = mode
        if hasattr(self._app, "set_lane_keep_controller"):
            self._app.set_lane_keep_controller(self._CONTROLLER[mode])
        # Only the PP clamp is settable from here; mpc/fp clamp lives in mpc_max_steer_deg.
        self._app.set_lane_keep_max_steer_deg(float(np.rad2deg(self.max_steer_rad)))

    def apply_pp_params(
        self,
        *,
        pp_k_dd: float,
        pp_ld_min: float,
        pp_ld_max: float,
        pp_shift: float,
        max_steer_deg: float,
    ) -> None:
        self.pp_shift = float(pp_shift)
        self.max_steer_rad = float(np.deg2rad(max_steer_deg))
        self._app.set_lane_keep_pp(
            float(pp_k_dd), float(pp_ld_min), float(pp_ld_max), float(pp_shift)
        )
        if self.mode != "mpc":
            self._app.set_lane_keep_max_steer_deg(float(max_steer_deg))

    @property
    def waypoint_shift(self) -> float:
        return self.pp_shift

    @property
    def app(self) -> Any:
        return self._app

    def _speed_action(self, speed_mps: float) -> tuple[float, float]:
        err = self.desired_speed - max(0.0, float(speed_mps))
        if err > 0.5:
            return min(0.85, self.speed_kp * err), 0.0
        if err < -1.0:
            return 0.0, min(0.5, 0.05 * (-err))
        return 0.0, 0.0

    def _active_max_steer_rad(self) -> float:
        """Mirrors C++ LaneKeepService::activeMaxSteerRad (mpc family capped at 25°)."""
        if self.mode in ("mpc", "fp"):
            return min(self.mpc_max_steer_rad, float(np.deg2rad(25.0)))
        return self.max_steer_rad

    def compute_from_polyline(
        self,
        speed_mps: float,
        polyline: Optional[np.ndarray],
        yaw_rate: float = 0.0,
        dt_s: Optional[float] = None,
        lane_anchored: bool = False,
    ) -> LaneKeepResult:
        throttle, brake = self._speed_action(speed_mps)

        if self.mode == "straight" or polyline is None:
            result = LaneKeepResult(
                mode=self.mode,
                steer_rad=0.0,
                steer_norm=0.0,
                throttle=throttle,
                brake=brake,
                polyline=None
                if polyline is None
                else np.asarray(polyline, dtype=np.float64),
                status="straight" if self.mode == "straight" else "no_polyline",
                controller=self._CONTROLLER[self.mode],
            )
            self.last_result = result
            return result

        poly = np.asarray(polyline, dtype=np.float64)
        if poly.ndim != 2 or poly.shape[0] < 2:
            result = LaneKeepResult(
                mode=self.mode,
                steer_rad=0.0,
                steer_norm=0.0,
                throttle=throttle,
                brake=brake,
                polyline=None,
                status="no_polyline",
                controller=self._CONTROLLER[self.mode],
            )
            self.last_result = result
            return result

        pairs = [(float(x), float(y)) for x, y in poly]
        # Real step: the C++ planner measures its solve period from these stamps and scales
        # the x0 advance, jerk clips and slew guards by it (docs/FRAME_DT_FIX_0801.md).
        self._t_us += int(round(float(dt_s if dt_s is not None else self.dt_s) * 1e6))
        self._app.publish_chassis(self._t_us, float(speed_mps), 0.0, float(yaw_rate))
        self._app.publish_lanes(self._t_us, pairs, lane_anchored=lane_anchored)
        self._app.step(self._t_us)
        out = None
        for msg in self._app.pop_messages():
            if isinstance(msg, pyadas.LaneKeepOutput):
                out = msg
            elif isinstance(msg, pyadas.SafetyWarnState):
                # Same drain: whoever wants the warnings has to read them here, or they are gone.
                self.last_warn = msg
        if out is None:
            result = LaneKeepResult(
                mode=self.mode,
                steer_rad=0.0,
                steer_norm=0.0,
                throttle=throttle,
                brake=brake,
                polyline=poly,
                status="no_output",
                controller=self._CONTROLLER[self.mode],
            )
            self.last_result = result
            return result

        steer_rad = float(out.steer_rad)
        lim = self._active_max_steer_rad()
        steer_norm = float(np.clip(steer_rad / lim, -1.0, 1.0) if lim > 1e-6 else 0.0)
        controller = getattr(out, "controller", None) or self._CONTROLLER[self.mode]
        cte = float(getattr(out, "cte_m", 0.0) or 0.0)
        epsi = float(getattr(out, "epsi_rad", 0.0) or 0.0)
        if controller == "mpc" and abs(cte) < 1e-12 and abs(epsi) < 1e-12:
            cte = float(out.target_x)
            epsi = float(out.target_y)

        pp = None
        if self.mode == "pure_pursuit":
            pp = pp_result_from_output(
                out,
                poly,
                speed_mps,
                waypoint_shift=self.pp_shift,
                wheel_base=self.wheelbase,
            )

        result = LaneKeepResult(
            mode=self.mode,
            steer_rad=steer_rad,
            steer_norm=steer_norm,
            throttle=throttle,
            brake=brake,
            polyline=poly,
            e_y=cte,
            e_psi=epsi,
            curvature=float(out.curvature),
            pure_pursuit=pp,
            status=str(out.status),
            controller=str(controller),
        )
        self.last_result = result
        return result

    def compute(
        self, speed_mps: float, lanes: Optional[Dict[str, Any]]
    ) -> LaneKeepResult:
        """``left_road`` / ``right_road`` must already be **device Y-right**."""
        if self.mode == "straight" or lanes is None:
            return self.compute_from_polyline(speed_mps, None)
        poly = build_centerline_polyline(lanes.get("left_road"), lanes.get("right_road"))
        return self.compute_from_polyline(speed_mps, poly)

    def get_control(
        self, speed_mps: float, lanes: Optional[Dict[str, Any]] = None
    ) -> list[float]:
        """Returns [steer_norm, throttle, brake] in **device** frame (right+)."""
        result = self.compute(speed_mps, lanes)
        return [result.steer_norm, result.throttle, result.brake]
