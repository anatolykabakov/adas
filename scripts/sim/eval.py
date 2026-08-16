#!/usr/bin/env python3
"""Closed-loop controller test on a generated track.

Drives the real C++ lane-keep service (via ``pyadas``) around a MetaDrive track and scores it
against ground truth: the metric is the vehicle's distance from the lane centerline, split by
what the road is doing at that moment — straight, or an arc of a known radius. Same idea as the
AAD control test (distance to the reference polyline), on top of the physics and the map the
project already uses.

The controller reads the shipped ``config.json``, so a run tests what the APK would do.

By default the reference is delayed and roughened to match the car: the vision stack needs ~90 ms
from shutter to path, and the model's lateral estimate is noisy. Without those, the loop is easier
than the road and every controller looks perfect (``--vision-latency-ms 0 --vision-noise-m 0``
gives the ideal-perception behaviour).

  python3 -m sim.eval --controllers fp,pp --track highway
  python3 -m sim.eval --track curvy --speed 20 --seeds 1,2,3 --csv out.csv
  python3 -m sim.eval --track tight --controllers fp --report-only

Exit code is non-zero when a controller misses the thresholds for the track, so this doubles as
a regression test.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

import _path  # noqa: F401

from core.frames import METADRIVE_STEER_FROM_DEVICE
from core.lane_keep import LaneKeepController, load_vehicle_config
from core.path_fusion import iso_left_polyline_to_device
from sim.track import LANE_WIDTH_M, TRACKS, env_config, resolve

# MetaDrive maps action[0] to road-wheel angle: steering_deg = action * max_steering.
# Normalising by the controller's own clamp instead (8° for pp) multiplies the command by 5.
MAX_STEERING_DEG = 40.0

CAR_HALF_WIDTH_M = 0.93
LANE_DEPART_M = LANE_WIDTH_M / 2.0 - CAR_HALF_WIDTH_M  # wheel touches the line

# Per-class pass thresholds, taken from what the car does on the road (docs/CONTROLLER_LIMITS.md).
# Arcs below 100 m are documented as out of scope — measured, never asserted.
THRESHOLDS: Dict[str, Dict[str, float]] = {
    "straight": {"median": 0.20, "p95": 0.50},
    "arc R>=400": {"median": 0.30, "p95": 0.70},
    "arc R 200-400": {"median": 0.30, "p95": 0.70},
    "arc R 100-200": {"median": 0.50, "p95": 1.00},
}


@dataclass
class Sample:
    t_s: float
    seg: str
    radius_m: float
    speed_mps: float
    cte_m: float
    steer_deg: float
    steer_limit_deg: float
    ctrl_cte_m: float
    curvature: float
    lat_accel_ms2: float


@dataclass
class RunResult:
    controller: str
    track: str
    seed: int
    speed_mps: float
    samples: List[Sample] = field(default_factory=list)
    distance_m: float = 0.0
    duration_s: float = 0.0
    end_reason: str = "max_steps"
    ldw_episodes: int = 0
    fcw_episodes: int = 0
    latency_ms: float = 0.0
    noise_m: float = 0.0


def _segment_class(lane: Any) -> tuple[str, float]:
    radius = float(getattr(lane, "radius", 0.0) or 0.0)
    if radius <= 0.0:
        return "straight", 0.0
    if radius >= 400.0:
        return "arc R>=400", radius
    if radius >= 200.0:
        return "arc R 200-400", radius
    if radius >= 100.0:
        return "arc R 100-200", radius
    return "arc R<100", radius


def _next_lane(net: Any, lane: Any) -> Optional[Any]:
    """Follow the forward chain. Test tracks have no forks, so the successor is unambiguous."""
    _from, to_node, idx = lane.index
    for nxt, lanes in (net.graph.get(to_node) or {}).items():
        if nxt.startswith("-"):  # oncoming direction of the same road
            continue
        if idx < len(lanes):
            return lanes[idx]
    return None


def centerline_ahead(
    agent: Any, net: Any, dist_m: float = 45.0, step_m: float = 1.5
) -> np.ndarray:
    """Lane centre sampled ahead of the car, in ego frame (x forward, y left+ / ISO).

    This is the perfect-perception input: what the vision stack would deliver if the model and
    the calibration were exact. A controller that cannot hold the lane on this has a control
    problem, not a perception problem.
    """
    lane = agent.lane
    s, _ = lane.local_coordinates(agent.position)
    pts: List[Any] = []
    travelled = 0.0
    while travelled < dist_m:
        if s > lane.length:
            nxt = _next_lane(net, lane)
            if nxt is None:
                break
            s -= lane.length
            lane = nxt
            continue
        pts.append(
            agent.convert_to_local_coordinates(lane.position(s, 0.0), agent.position)
        )
        s += step_m
        travelled += step_m
    if len(pts) < 4:
        return np.zeros((0, 2), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


class Perception:
    """What the controller is allowed to see: a delayed, noisy copy of the lane centre.

    Latency is the measured capture→path time on the phone (69 ms of it before the planner even
    starts, ~90 ms median end to end). Noise stands in for the model's lateral scatter — it is the
    reason Pure Pursuit weaves on the road while it looks flawless on ground truth.

    The default 0.15 m at 20 m is the frame-to-frame scatter of the model path measured on bags
    2026_07_26_20_55_20 / _20_52_53 (0.10–0.15 m at 20 m, 0.21–0.23 m at 40 m), which is why it
    is scaled by distance here. That measurement still contains real motion between frames, so it
    is an upper bound on pure noise rather than a clean estimate.
    """

    def __init__(
        self,
        latency_ms: float,
        noise_m: float,
        dt_s: float,
        seed: int,
        jump_rate_hz: float = 0.0,
        jump_m: float = 0.0,
    ):
        self.delay_steps = int(round(max(0.0, latency_ms) / 1000.0 / dt_s))
        self.noise_m = max(0.0, noise_m)
        # Rare hypothesis switches: on the bags, 5 % of frames move the path at 20 m by more than
        # 1.6 m, which a Gaussian never produces. Those are the events a controller has to ride out.
        self.jump_prob = max(0.0, jump_rate_hz) * dt_s
        self.jump_m = max(0.0, jump_m)
        self.rng = np.random.default_rng(seed)
        self.queue: List[Optional[np.ndarray]] = []

    def see(self, poly: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if poly is not None and self.noise_m > 0.0:
            poly = np.array(poly, dtype=np.float64, copy=True)
            # One offset per frame plus per-point scatter: a bias the controller chases, and
            # jitter it should not. Both grow with distance, like the model's own uncertainty.
            reach = np.maximum(1.0, poly[:, 0]) / 20.0
            poly[:, 1] += self.noise_m * self.rng.normal() * reach
            poly[:, 1] += 0.5 * self.noise_m * self.rng.normal(size=len(poly)) * reach
            if self.jump_prob > 0.0 and self.rng.random() < self.jump_prob:
                poly[:, 1] += self.jump_m * self.rng.choice([-1.0, 1.0]) * reach

        self.queue.append(poly)
        if len(self.queue) <= self.delay_steps:
            return None
        return self.queue.pop(0)


def run_once(
    controller_mode: str,
    track_name: str,
    seed: int,
    speed_mps: Optional[float],
    max_steps: int,
    warmup_s: float,
    offset_m: float = 0.0,
    tire_stiffness: Optional[float] = None,
    kinematic: bool = False,
    overrides: Optional[Dict[str, float]] = None,
    latency_ms: float = 90.0,
    noise_m: float = 0.15,
    jump_rate_hz: float = 0.0,
    jump_m: float = 1.6,
) -> RunResult:
    from metadrive.envs.metadrive_env import MetaDriveEnv

    track = resolve(track_name)
    target_speed = float(speed_mps if speed_mps is not None else track.speed_mps)

    cfg = env_config(track, seed)
    if abs(offset_m) > 1e-6:
        # spawn_lateral is right-positive like lane.local_coordinates.
        # At speed, too: by the time a standing start reaches LDW speed the controller has
        # already recovered, so the departure would never be observable.
        cfg["vehicle_config"] = {
            **cfg.get("vehicle_config", {}),
            "spawn_lateral": float(offset_m),
            "spawn_velocity": [float(target_speed), 0.0],
            "spawn_velocity_car_frame": True,
        }
        # Starting on the line is the whole point of an offset run; MetaDrive would otherwise end
        # the episode before the controller has taken a single step.
        cfg["out_of_road_done"] = False
        cfg["on_continuous_line_done"] = False
    env = MetaDriveEnv(cfg)
    env.reset()
    net = env.current_map.road_network
    dt_s = float(env.config["physics_world_step_size"] * env.config["decision_repeat"])

    veh_cfg = dict(load_vehicle_config())
    # The κ→δ understeer model is fitted to the Golf (0.61 of kinematic curvature at 22 m/s); the
    # MetaDrive ego measures 0.98–1.01 (sim.vehicle_calib). Keeping the Golf model here would
    # over-turn every arc by ~1.6× and show up as a steady offset that is the car, not the
    # controller — so sim runs default to kinematic and --vehicle-model opts back in.
    if kinematic:
        veh_cfg["lat_use_vehicle_model"] = False
    elif tire_stiffness is not None:
        veh_cfg["tire_stiffness_factor"] = float(tire_stiffness)
    veh_cfg.update(overrides or {})

    ctrl = LaneKeepController(
        mode=controller_mode,
        desired_speed=target_speed,
        dt_s=dt_s,
        vehicle_config=veh_cfg,
    )

    perception = Perception(latency_ms, noise_m, dt_s, seed, jump_rate_hz, jump_m)
    result = RunResult(
        controller=controller_mode, track=track_name, seed=seed, speed_mps=target_speed
    )
    result.latency_ms = float(latency_ms)
    result.noise_m = float(noise_m)
    ldw_prev = [False]
    fcw_prev = [False]
    agent = env.agent
    prev_pos = np.array(agent.position, dtype=np.float64)

    try:
        for step in range(max_steps):
            agent = env.agent
            lane = agent.lane
            speed = float(agent.speed)
            # lane.local_coordinates is right-positive (verified: a car 1 m left of the centre
            # reads −1), which is already the C++ `cte_m` convention: + = ego right of path.
            # convert_to_local_coordinates, in contrast, is ISO left+ — hence the flip below.
            _, lat = lane.local_coordinates(agent.position)
            cte = float(lat)

            poly_iso = centerline_ahead(agent, net)
            poly = iso_left_polyline_to_device(poly_iso) if poly_iso.size else None
            poly = perception.see(poly)
            # GT centreline *is* both lane lines, so the departure warning is armed here.
            lk = ctrl.compute_from_polyline(
                speed,
                poly,
                yaw_rate=float(getattr(agent, "yaw_rate", 0.0)),
                lane_anchored=poly is not None,
            )

            steer_deg = float(np.rad2deg(lk.steer_rad))
            action = [
                METADRIVE_STEER_FROM_DEVICE
                * float(np.clip(steer_deg / MAX_STEERING_DEG, -1.0, 1.0)),
                float(lk.throttle),
                float(lk.brake),
            ]

            t_s = step * dt_s
            if t_s >= warmup_s:
                # Warnings are counted on the same window as the metrics: the spin-up transient
                # is not the assistant's behaviour.
                warn = ctrl.last_warn
                if warn is not None:
                    ldw_now = bool(warn.lldw or warn.rldw)
                    result.ldw_episodes += int(ldw_now and not ldw_prev[0])
                    result.fcw_episodes += int(bool(warn.fcw) and not fcw_prev[0])
                    ldw_prev[0], fcw_prev[0] = ldw_now, bool(warn.fcw)

                seg, radius = _segment_class(lane)
                lat_acc = speed * speed / radius if radius > 0 else 0.0
                result.samples.append(
                    Sample(
                        t_s=t_s,
                        seg=seg,
                        radius_m=radius,
                        speed_mps=speed,
                        cte_m=cte,
                        steer_deg=steer_deg,
                        steer_limit_deg=float(np.rad2deg(ctrl._active_max_steer_rad())),
                        ctrl_cte_m=float(lk.e_y),
                        curvature=float(lk.curvature),
                        lat_accel_ms2=lat_acc,
                    )
                )

            _, _, terminated, truncated, info = env.step(action)
            pos = np.array(env.agent.position, dtype=np.float64)
            result.distance_m += float(np.linalg.norm(pos - prev_pos))
            prev_pos = pos
            result.duration_s = t_s

            if terminated or truncated:
                if info.get("arrive_dest"):
                    result.end_reason = "arrive_dest"
                elif info.get("out_of_road"):
                    result.end_reason = "out_of_road"
                elif info.get("crash"):
                    result.end_reason = "crash"
                else:
                    result.end_reason = "terminated"
                break
    finally:
        env.close()

    return result


def summarize(result: RunResult) -> Dict[str, Dict[str, float]]:
    out: Dict[str, Dict[str, float]] = {}
    by_seg: Dict[str, List[Sample]] = {}
    for s in result.samples:
        by_seg.setdefault(s.seg, []).append(s)

    for seg, rows in by_seg.items():
        cte = np.abs(np.array([r.cte_m for r in rows]))
        steer = np.array([r.steer_deg for r in rows])
        limit = np.array([r.steer_limit_deg for r in rows])
        radii = np.array([r.radius_m for r in rows])
        out[seg] = {
            "n": float(len(rows)),
            "seconds": float(
                len(rows) * (result.duration_s / max(1, len(result.samples)))
            ),
            "radius_min": float(radii.min()) if radii.size and radii.max() > 0 else 0.0,
            "radius_max": float(radii.max()),
            "cte_median": float(np.median(cte)),
            "cte_p95": float(np.percentile(cte, 95)),
            "cte_max": float(cte.max()),
            "depart_pct": float(100.0 * np.mean(cte > LANE_DEPART_M)),
            "sat_pct": float(100.0 * np.mean(np.abs(steer) >= 0.99 * limit)),
            "steer_hf_deg": float(np.median(np.abs(np.diff(steer))))
            if len(steer) > 1
            else 0.0,
            "speed_mean": float(np.mean([r.speed_mps for r in rows])),
            "lat_accel_p95": float(np.percentile([r.lat_accel_ms2 for r in rows], 95)),
        }
    return out


def check(summary: Dict[str, Dict[str, float]]) -> List[str]:
    failures = []
    for seg, stats in sorted(summary.items()):
        limits = THRESHOLDS.get(seg)
        if not limits:
            continue
        if stats["cte_median"] > limits["median"]:
            failures.append(
                f"{seg}: median {stats['cte_median']:.2f} m > {limits['median']:.2f}"
            )
        if stats["cte_p95"] > limits["p95"]:
            failures.append(f"{seg}: p95 {stats['cte_p95']:.2f} m > {limits['p95']:.2f}")
    return failures


SEG_ORDER = ["straight", "arc R>=400", "arc R 200-400", "arc R 100-200", "arc R<100"]


def perception_gap_m(result: RunResult) -> float:
    """|estimate − truth| for CTE. With a perfect-perception polyline this must stay near zero;
    anything else means the sim feeds the controller a different world than it scores it on."""
    if not result.samples:
        return 0.0
    return float(np.median([abs(s.ctrl_cte_m - s.cte_m) for s in result.samples]))


def print_report(
    result: RunResult, summary: Dict[str, Dict[str, float]], failures: List[str]
) -> None:
    head = (
        f"{result.controller:>4s} | {result.track} seed={result.seed} "
        f"v={result.speed_mps:.0f} m/s | {result.distance_m:.0f} m / {result.duration_s:.0f} s | "
        f"end: {result.end_reason} | vision {result.latency_ms:.0f} ms / noise {result.noise_m:.2f} m | "
        f"LDW {result.ldw_episodes} / FCW {result.fcw_episodes}"
    )
    print(head)
    print(
        f"{'segment':<15}{'sec':>6}{'R, m':>12}{'|CTE| med':>11}{'p95':>8}{'max':>8}"
        f"{'off-road %':>9}{'sat %':>9}{'HF °':>7}{'a_lat p95':>11}"
    )
    for seg in SEG_ORDER:
        if seg not in summary:
            continue
        s = summary[seg]
        rng = (
            "—"
            if s["radius_max"] <= 0
            else f"{s['radius_min']:.0f}–{s['radius_max']:.0f}"
        )
        print(
            f"{seg:<15}{s['seconds']:>6.0f}{rng:>12}{s['cte_median']:>11.2f}{s['cte_p95']:>8.2f}"
            f"{s['cte_max']:>8.2f}{s['depart_pct']:>9.1f}{s['sat_pct']:>9.1f}"
            f"{s['steer_hf_deg']:>7.2f}{s['lat_accel_p95']:>11.1f}"
        )
    if failures:
        for f in failures:
            print(f"  FAIL {f}")
    else:
        print("  threshold passed")
    print()


def write_csv(path: Path, results: List[RunResult]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "controller",
                "track",
                "seed",
                "t_s",
                "seg",
                "radius_m",
                "speed_mps",
                "cte_m",
                "steer_deg",
                "ctrl_cte_m",
                "curvature",
                "lat_accel_ms2",
            ]
        )
        for r in results:
            for s in r.samples:
                w.writerow(
                    [
                        r.controller,
                        r.track,
                        r.seed,
                        f"{s.t_s:.2f}",
                        s.seg,
                        f"{s.radius_m:.0f}",
                        f"{s.speed_mps:.2f}",
                        f"{s.cte_m:.3f}",
                        f"{s.steer_deg:.2f}",
                        f"{s.ctrl_cte_m:.3f}",
                        f"{s.curvature:.5f}",
                        f"{s.lat_accel_ms2:.2f}",
                    ]
                )


def plot(path: Path, results: List[RunResult]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    for r in results:
        t = [s.t_s for s in r.samples]
        axes[0].plot(
            t, [s.cte_m for s in r.samples], lw=1.0, label=f"{r.controller} ({r.track})"
        )
        axes[1].plot(t, [s.steer_deg for s in r.samples], lw=1.0, label=r.controller)
    if results:
        t = [s.t_s for s in results[0].samples]
        curv = [1.0 / s.radius_m if s.radius_m > 0 else 0.0 for s in results[0].samples]
        ax2 = axes[0].twinx()
        ax2.fill_between(t, 0, curv, color="0.85", zorder=0)
        ax2.set_ylabel("track curvature, 1/m")
    for lim in (LANE_DEPART_M, -LANE_DEPART_M):
        axes[0].axhline(lim, color="r", ls="--", lw=0.8)
    axes[0].set_ylabel("CTE, m (+ right)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].set_ylabel("δ command, °")
    axes[1].set_xlabel("time, s")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"plot → {path}")


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--controllers",
        default="fp",
        help="comma-separated: fp, mpc, pure_pursuit, straight",
    )
    p.add_argument("--track", default="highway", help=f"one of: {', '.join(TRACKS)}")
    p.add_argument("--list-tracks", action="store_true", help="list tracks and exit")
    p.add_argument("--seeds", default="7", help="comma-separated")
    p.add_argument(
        "--speed",
        type=float,
        default=None,
        help="target speed m/s (default from track)",
    )
    p.add_argument("--max-steps", type=int, default=2500, help="control step cap per run")
    p.add_argument("--warmup-s", type=float, default=6.0, help="warmup seconds to skip")
    p.add_argument(
        "--offset",
        type=float,
        default=0.0,
        help="initial offset from center, m (+ right)",
    )
    p.add_argument(
        "--vehicle-model",
        action="store_true",
        help="κ→δ via Golf understeer model from config.json; default kinematic, "
        "because simulator vehicle is kinematic (sim.vehicle_calib)",
    )
    p.add_argument(
        "--tire-stiffness",
        type=float,
        default=None,
        help="custom tire_stiffness_factor instead of config.json value",
    )
    p.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override param from vehicle block in config.json (repeatable)",
    )
    p.add_argument(
        "--vision-latency-ms",
        type=float,
        default=90.0,
        help="reference delay, ms (0 = perfect perception; phone median 90)",
    )
    p.add_argument(
        "--vision-noise-m",
        type=float,
        default=0.15,
        help="lateral estimate noise at 20 m, m (0 = clean reference; 0.15 measured from bags)",
    )
    p.add_argument(
        "--vision-jump-hz",
        type=float,
        default=0.0,
        help="plan jump rate (model hypothesis change), Hz; ~0.5 from bags, "
        "default 0 — fp fails on them, not pp, i.e. jump model "
        "does not reproduce road ranking (docs/SIM_CONTROLLER_TEST.md)",
    )
    p.add_argument(
        "--vision-jump-m",
        type=float,
        default=1.6,
        help="jump magnitude at 20 m, m (p95 |Δy| from bags)",
    )
    p.add_argument("--csv", type=Path, default=None)
    p.add_argument("--json", type=Path, default=None)
    p.add_argument("--plot", type=Path, default=None)
    p.add_argument(
        "--report-only", action="store_true", help="do not fail run on thresholds"
    )
    return p


def main() -> int:
    args = build_arg_parser().parse_args()

    if args.list_tracks:
        for name, t in TRACKS.items():
            lo, hi = t.lateral_accel_ms2()
            print(
                f"{name:9s} {t.sequence:9s} R {t.radius_m[0]:.0f}–{t.radius_m[1]:.0f} m  "
                f"v {t.speed_mps:.0f} m/s  a_lat {lo:.1f}–{hi:.1f} m/s²  — {t.description}"
            )
        return 0

    overrides: Dict[str, float] = {}
    for item in args.set:
        key, _, value = item.partition("=")
        if not _:
            raise SystemExit(f"--set expects KEY=VALUE, got {item!r}")
        overrides[key.strip()] = float(value)

    modes = [m.strip() for m in args.controllers.split(",") if m.strip()]
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]

    results: List[RunResult] = []
    report: Dict[str, Any] = {
        "track": args.track,
        "overrides": overrides,
        "vision_latency_ms": args.vision_latency_ms,
        "vision_noise_m": args.vision_noise_m,
        "runs": [],
    }
    all_failures: List[str] = []

    for mode in modes:
        for seed in seeds:
            r = run_once(
                mode,
                args.track,
                seed,
                args.speed,
                args.max_steps,
                args.warmup_s,
                args.offset,
                args.tire_stiffness,
                kinematic=not args.vehicle_model,
                overrides=overrides,
                latency_ms=args.vision_latency_ms,
                noise_m=args.vision_noise_m,
                jump_rate_hz=args.vision_jump_hz,
                jump_m=args.vision_jump_m,
            )
            s = summarize(r)
            failures = check(s)
            if r.end_reason in ("out_of_road", "crash"):
                failures.append(f"run aborted: {r.end_reason}")
            print_report(r, s, failures)
            results.append(r)
            report["runs"].append(
                {
                    "controller": mode,
                    "seed": seed,
                    "speed_mps": r.speed_mps,
                    "distance_m": r.distance_m,
                    "duration_s": r.duration_s,
                    "end_reason": r.end_reason,
                    "segments": s,
                    "failures": failures,
                }
            )
            all_failures += [f"{mode}/{seed}: {f}" for f in failures]

    if args.csv:
        write_csv(args.csv, results)
        print(f"CSV → {args.csv}")
    if args.json:
        args.json.write_text(
            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        print(f"JSON → {args.json}")
    if args.plot:
        plot(args.plot, results)

    if all_failures and not args.report_only:
        print("failed:")
        for f in all_failures:
            print(f"  {f}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
