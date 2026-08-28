"""Longitudinal harness for MetaDrive: a scripted lead, the pedal actuator, the metrics.

The C++ services produce an acceleration request; a car realises it through its motor and brake
controllers. Here `AccelActuator` plays that part: a measured feedforward map from the MetaDrive ego
(``sim.longitudinal --calibrate``) plus a small inner loop on measured acceleration, the way an ACC
ECU closes its own loop on the request.

The lead is kinematic — it is moved along the lane centre at the scripted speed, not driven — so a
scenario is exactly reproducible and the gap is a function of our controller alone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

# MetaDrive DefaultVehicle; the lead is the same model.
VEHICLE_LENGTH_M = 4.515
# The planner's `long_plan.lead_origin_offset_m`: model lead distances are from the camera, the
# harness measures bumper to bumper, so it adds this back before publishing.
LEAD_ORIGIN_OFFSET_M = 1.4


@dataclass
class LeadScenario:
    name: str
    gap0_m: float          # initial bumper-to-bumper gap
    v0_mps: float          # lead speed at the start
    event_t_s: float = 1e9  # when the lead changes speed
    a_event_ms2: float = 0.0
    v_end_mps: Optional[float] = None  # the lead holds this once reached
    description: str = ""


SCENARIOS: Dict[str, Optional[LeadScenario]] = {
    "none": None,
    "lead_const": LeadScenario("lead_const", 60.0, 15.0, description="steady lead 15 m/s, ego set faster"),
    "lead_brake": LeadScenario(
        "lead_brake", 45.0, 20.0, event_t_s=8.0, a_event_ms2=-2.0, v_end_mps=5.0,
        description="lead 20 m/s brakes at −2 m/s² to 5 m/s",
    ),
    "lead_stop": LeadScenario(
        "lead_stop", 60.0, 15.0, event_t_s=8.0, a_event_ms2=-2.5, v_end_mps=0.0,
        description="lead 15 m/s brakes to a stop",
    ),
    "stationary": LeadScenario("stationary", 120.0, 0.0, description="stopped car 120 m ahead"),
}


class ScriptedLead:
    """A lead moved along the ego's lane chain at a scripted speed."""

    def __init__(self, env: Any, net: Any, scenario: LeadScenario, next_lane: Any):
        from metadrive.component.vehicle.vehicle_type import DefaultVehicle

        self.env = env
        self.net = net
        self.scenario = scenario
        self._next_lane = next_lane
        ego = env.agent
        self.lane = ego.lane
        s, _ = self.lane.local_coordinates(ego.position)
        self.s = s + scenario.gap0_m + VEHICLE_LENGTH_M  # centre to centre
        self._advance_lane()
        self.v = float(scenario.v0_mps)
        self.a = 0.0
        self.t = 0.0
        self.at_end = False
        pos = self.lane.position(self.s, 0.0)
        heading = self.lane.heading_theta_at(self.s)
        vc = dict(env.config["vehicle_config"])
        vc.update({"spawn_lane_index": self.lane.index, "spawn_longitude": float(self.s), "spawn_lateral": 0.0})
        self.obj = env.engine.spawn_object(DefaultVehicle, vehicle_config=vc, position=pos, heading=heading)
        self.obj.set_static(True)
        self._place()

    def _advance_lane(self) -> None:
        while self.s > self.lane.length:
            nxt = self._next_lane(self.net, self.lane)
            if nxt is None:
                # The track ends here: the lead would freeze on the last metre and the run would score
                # a phantom rear-end. The harness stops the scenario instead.
                self.s = self.lane.length
                self.at_end = True
                return
            self.s -= self.lane.length
            self.lane = nxt

    def _place(self) -> None:
        pos = self.lane.position(self.s, 0.0)
        heading = self.lane.heading_theta_at(self.s)
        self.obj.set_position(pos)
        self.obj.set_heading_theta(heading)
        self.obj.set_velocity([self.v * math.cos(heading), self.v * math.sin(heading)])

    def step(self, dt: float) -> None:
        self.t += dt
        sc = self.scenario
        self.a = 0.0
        if self.t >= sc.event_t_s and sc.v_end_mps is not None:
            if (sc.a_event_ms2 < 0 and self.v > sc.v_end_mps) or (sc.a_event_ms2 > 0 and self.v < sc.v_end_mps):
                self.a = sc.a_event_ms2
                self.v = max(0.0, self.v + self.a * dt)
                if (sc.a_event_ms2 < 0 and self.v <= sc.v_end_mps) or (sc.a_event_ms2 > 0 and self.v >= sc.v_end_mps):
                    self.v = sc.v_end_mps
        self.s += self.v * dt
        self._advance_lane()
        self._place()

    def relative(self, ego: Any) -> Dict[str, float]:
        """Lead as the model would report it: from the camera, y right-positive (device frame)."""
        x, y_left = ego.convert_to_local_coordinates(self.obj.position, ego.position)
        gap = float(x) - VEHICLE_LENGTH_M
        return {
            "gap_m": gap,
            "d_rel_camera": gap + LEAD_ORIGIN_OFFSET_M,
            "y_rel": -float(y_left),
            "v_lead": self.v,
            "a_lead": self.a,
        }

    def destroy(self) -> None:
        try:
            self.env.engine.clear_objects([self.obj.id])
        except Exception:
            pass


class AccelActuator:
    """Acceleration request → MetaDrive's signed throttle/brake axis.

    Feedforward from ``--calibrate`` on the straight track (DefaultVehicle, MAX_ENGINE_FORCE 2600):
    throttle t gives ≈ 9.45·t m/s²; zero pedal coasts at −0.73; the brake axis is nonlinear,
    −0.2 → −6.8, −0.5 → −11.5, −1.0 → −17. The inner loop takes up the rest.
    """

    THROTTLE_GAIN = 9.45
    COAST_ACCEL = -0.73
    BRAKE_AXIS = np.array([-1.0, -0.5, -0.2, 0.0])
    BRAKE_ACCEL = np.array([-17.0, -11.5, -6.8, -0.73])

    def __init__(self, dt_s: float, kp: float = 0.03, ki: float = 0.02):
        self.dt = float(dt_s)
        self.kp = kp
        self.ki = ki
        self.integral = 0.0
        self.a_meas = 0.0
        self._v_prev: Optional[float] = None

    def measure(self, v_mps: float) -> float:
        if self._v_prev is not None:
            a = (v_mps - self._v_prev) / self.dt
            self.a_meas = 0.6 * self.a_meas + 0.4 * a
        self._v_prev = float(v_mps)
        return self.a_meas

    def feedforward(self, a_req: float) -> float:
        if a_req >= self.COAST_ACCEL:
            return float(np.clip(a_req / self.THROTTLE_GAIN, 0.0, 1.0))
        return float(np.interp(a_req, self.BRAKE_ACCEL, self.BRAKE_AXIS))

    def act(self, a_req: Optional[float], v_mps: float) -> float:
        """One control step. ``None`` means no request: coast (pedals released)."""
        a_meas = self.measure(v_mps)
        if a_req is None:
            self.integral = 0.0
            return 0.0
        err = a_req - a_meas
        self.integral = float(np.clip(self.integral + self.ki * err * self.dt, -0.3, 0.3))
        u = self.feedforward(a_req) + self.kp * err + self.integral
        if v_mps < 0.05 and a_req <= 0.0:
            u = min(u, 0.0)  # do not creep while holding a stop
        return float(np.clip(u, -1.0, 1.0))


@dataclass
class LongSample:
    t_s: float
    v_ego: float
    v_cruise: float
    accel_cmd: float
    accel_meas: float
    long_status: str
    long_state: int
    source: str
    gap_m: float = float("nan")
    v_lead: float = float("nan")


@dataclass
class LongMetrics:
    scenario: str
    min_gap_m: float = float("nan")
    min_ttc_s: float = float("nan")
    final_gap_m: float = float("nan")
    desired_gap_m: float = float("nan")
    accel_min: float = 0.0
    accel_max: float = 0.0
    jerk_p95: float = 0.0
    v_final: float = 0.0
    v_cruise: float = 0.0
    time_to_cruise_s: float = float("nan")
    active_pct: float = 0.0
    crash: bool = False
    failures: List[str] = field(default_factory=list)


def desired_follow_distance(v_ego: float, v_lead: float, t_follow: float = 1.45, stop_distance: float = 6.0,
                            comfort_brake: float = 2.5) -> float:
    """upstream's desired_follow_distance — the gap the plan settles at."""
    return v_ego * v_ego / (2 * comfort_brake) + t_follow * v_ego + stop_distance - v_lead * v_lead / (2 * comfort_brake)


def long_metrics(samples: List[LongSample], scenario: str, dt_s: float, crash: bool) -> LongMetrics:
    m = LongMetrics(scenario=scenario, crash=crash)
    if not samples:
        m.failures.append("no samples")
        return m
    a = np.array([s.accel_cmd for s in samples])
    active = np.array([s.long_status == "ok" for s in samples])
    m.accel_min = float(a[active].min()) if active.any() else 0.0
    m.accel_max = float(a[active].max()) if active.any() else 0.0
    if len(a) > 1:
        m.jerk_p95 = float(np.percentile(np.abs(np.diff(a)) / dt_s, 95))
    m.active_pct = float(100.0 * active.mean())
    m.v_final = float(np.mean([s.v_ego for s in samples[-10:]]))
    m.v_cruise = float(samples[-1].v_cruise)
    gaps = np.array([s.gap_m for s in samples])
    if np.isfinite(gaps).any():
        m.min_gap_m = float(np.nanmin(gaps))
        m.final_gap_m = float(np.nanmean(gaps[-10:]))
        v_lead = np.array([s.v_lead for s in samples])
        v_ego = np.array([s.v_ego for s in samples])
        closing = v_ego - v_lead
        ttc = np.where(closing > 0.1, gaps / np.maximum(closing, 0.1), np.inf)
        m.min_ttc_s = float(np.min(ttc))
        m.desired_gap_m = desired_follow_distance(float(v_ego[-1]), float(v_lead[-1]))
    if m.v_cruise > 0 and np.isnan(gaps).all():
        reached = [s.t_s for s in samples if s.v_cruise > 0 and abs(s.v_ego - s.v_cruise) < 0.5]
        m.time_to_cruise_s = float(reached[0]) if reached else float("nan")

    # Acceptance: no contact, the envelope respected, and the plan's own settle points.
    if crash or (np.isfinite(m.min_gap_m) and m.min_gap_m <= 0.0):
        m.failures.append("contact with the lead")
    if m.accel_min < -3.5 - 0.05 or m.accel_max > 2.0 + 0.05:
        m.failures.append(f"acceleration outside the envelope: {m.accel_min:.2f} … {m.accel_max:.2f}")
    if np.isfinite(m.min_ttc_s) and m.min_ttc_s < 2.0:
        m.failures.append(f"TTC dipped to {m.min_ttc_s:.1f} s")
    if scenario == "lead_const" and np.isfinite(m.final_gap_m):
        if abs(m.final_gap_m - m.desired_gap_m) > 0.25 * m.desired_gap_m + 3.0:
            m.failures.append(f"settled gap {m.final_gap_m:.1f} m vs desired {m.desired_gap_m:.1f} m")
    if scenario in ("lead_stop", "stationary") and np.isfinite(m.final_gap_m):
        if not (3.0 < m.final_gap_m < 10.0):
            m.failures.append(f"standstill gap {m.final_gap_m:.1f} m, expected the 6 m stop distance")
        if m.v_final > 0.3:
            m.failures.append(f"did not stop: v_final {m.v_final:.2f} m/s")
    if scenario == "none" and m.v_cruise > 0 and abs(m.v_final - m.v_cruise) > 0.7:
        m.failures.append(f"did not hold the set speed: {m.v_final:.1f} vs {m.v_cruise:.1f} m/s")
    return m


def print_long_report(m: LongMetrics) -> None:
    gap = "—" if math.isnan(m.min_gap_m) else f"min gap {m.min_gap_m:.1f} m, final {m.final_gap_m:.1f} (desired {m.desired_gap_m:.1f}), TTC min {m.min_ttc_s:.1f} s"
    ttc = "" if math.isnan(m.time_to_cruise_s) else f", set speed reached at {m.time_to_cruise_s:.1f} s"
    print(
        f"  long[{m.scenario}] active {m.active_pct:.0f}% | a {m.accel_min:.2f} … {m.accel_max:.2f} m/s², "
        f"jerk p95 {m.jerk_p95:.2f} m/s³ | v_final {m.v_final:.1f} / set {m.v_cruise:.1f} m/s{ttc} | {gap}"
    )
    for f in m.failures:
        print(f"  FAIL {f}")
