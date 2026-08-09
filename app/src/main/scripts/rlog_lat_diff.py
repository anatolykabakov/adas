#!/usr/bin/env python3
"""Push a recorded openpilot/dragonpilot route through *our* lateral stack and diff the commands.

This is the sharpest test available to a port: same car, same road, same reference path, same measured
steering angle — and the only thing that differs is whose code turns that into a steering command. Synthetic
tests say the code does what it was written to do; this says whether it does what upstream does.

The replay goes through the middleware, not through internal classes. `AdasApp` is the whole app: publish on
the topics the phone publishes on, call `step()`, read what comes out. That is deliberate — the middleware
exists to hide the implementation, so a comparison that reached inside it would be measuring a wiring diagram
we invented for the test rather than the one that drives the car.

## Two reference modes, and what each isolates

`--reference plan` (default) feeds `lateralPlan.dPathPoints`: their finished reference, *after* their lane
blending, camera offset and centering term. Their `y_pts` are sampled at `v_ego · t_idxs`, so the x
coordinates are reconstructed the same way. Perception and lane fusion are excluded on purpose, and our own
blending, camera offset and centering are switched off for the same reason — otherwise the comparison would
measure two different setpoints. What remains under test is path → curvature → angle → torque.

`--reference model` feeds `modelV2` instead: their four lane lines, road edges, and the plan, published as a
`LaneLines` message so that **our** `laneLinesToPath` runs on their model output. That extends the comparison
one stage upstream, to include our lane blending, width gate, σ gate and camera offset. Two things about it
are worth knowing before reading its numbers:

* their `laneLineStds` is **one scalar per line**; ours is a per-point `y_std` whose median over 5–20 m is
  what the σ gate reads. The scalar is broadcast to every point, so model mode exercises our gate on *their*
  definition of σ. That is informative — it is half of what task #23 has to settle — but it is not the same
  quantity our own model produces;
* their lateral sign matches ours (measured, not assumed: on a confident frame the near lines sit at −1.64
  and +1.82 m, i.e. y is right-positive exactly as `lanes.proto` documents), so the polylines are copied
  across without a flip.

## What is compared, and why not the steering angle first

Four quantities, in the order the signal travels. Keeping them apart is the point: a planning disagreement, a
vehicle-model disagreement and a controller disagreement have different fixes.

1. **curvature** — their `controlsState.desiredCurvature` against our `LaneKeepOutput.curvature`. This is the
   planner's own output on both sides, before either stack's steer ratio or understeer term touches it;
2. **setpoint angle** — their `actuators.steeringAngleDeg` against ours. Curvature → angle goes through each
   side's vehicle model, so this is where a `steer_ratio` or `tire_stiffness_factor` disagreement shows up
   rather than a planning one;
3. **torque before the limiter** — their `actuators.steer · STEER_MAX` against our `SteerCommand.torque_cnm`;
4. **torque after the limiter** — their `actuatorsOutput.steerOutputCan` (already in cNm, so no round trip
   through the normalisation) against our command run through `applyDriverSteerTorqueLimits` on a 20 ms grid,
   which is what `CarController` does at `STEER_STEP = 2`. Comparing (3) alone flatters both sides: the
   200 cNm/s rate limit turns a requested 300 into a median 187 applied, and the limiter is also where the
   command resets to zero whenever the assist drops.

**Their setpoint angle is not usable everywhere, and that is why (1) exists.** Measured on this route:
`actuators.steeringAngleDeg` reaches **649.7°** and has a p90 of **130.8°** even restricted to `latActive`
frames, because at low speed the plan is a few metres long, the curvature it implies is large, and their
vehicle model turns that into an angle no rack has. `desiredCurvature` over the same frames stays physical —
p90 0.049 1/m, i.e. a 20 m radius. So the angle comparison is gated on `|their angle| ≤ max_steer_deg ·
steer_ratio`, the largest setpoint our own planner can produce; past it our side is clamped by construction
and the comparison would measure our clamp. The number of frames this removes is reported, not hidden.

The limiter comes from the binding, not from a Python copy of it, so there is one implementation of the
asymmetric up/down logic. The EPS keepalive dither (±1 cNm every 1.9 s) is **not** modelled; it is a
liveness trick, not control.

## The one unfairness that cannot be fixed, only accounted for

**The replay is open loop, and that biases the torque comparison and nothing else.** The measured steering
angle comes from their log, so it is the angle *their* torque produced. Our command never moves the wheel, so
whatever error remains, our integrator keeps accumulating it — while theirs was logged while actively closing
the same error. Measured over 102 000 frames of this route set: our setpoint angle agrees with theirs to a
median of **0.08°** with a correlation of **0.965**, yet our torque before the limiter sits at a median of
**229 cNm against their 69** and hits the ±300 ceiling in 35 % of frames against their 10 %. Two stages
agreeing that closely cannot produce a 3× torque disagreement — the integrator can, and does.

So `--no-integrator` runs our PID with `ki = 0`, which makes the torque comparison a comparison of
instantaneous response to the same error, and is the only version of it worth reading. The default keeps the
integrator so the bias stays visible rather than quietly corrected, and both numbers are printed with the
difference between them named.

What this means for interpretation: **curvature and setpoint angle are trustworthy in this harness, torque
is trustworthy only with `--no-integrator`**, and steady-state offset is not answerable here at all — that
needs a closed loop, which is what the simulator is for and what `docs/BACKLOG.md` §3 says it cannot rank
controllers with either.

Sign conventions are reported, not assumed: their angle is openpilot's, ours is the CAN convention with
`vehicle.steer_sign` already applied, so the script measures the relationship instead of hard-coding it.

  OPENPILOT_ROOT=/path/to/openpilot python3 rlog_lat_diff.py <route dir> [<route dir> ...]
  python3 rlog_lat_diff.py <parent of routes> --reference model
  python3 rlog_lat_diff.py <route> --segments 6 --controller fp --steer-ratio 16.12
"""

from __future__ import annotations

import argparse
import bz2
import json
import os
import sys
from pathlib import Path

import numpy as np

import _path  # noqa: F401

# openpilot's model time grid. The first LAT_MPC_N+1 entries are what `dPathPoints` is sampled on, and
# hard-coding them beats importing openpilot: this must keep working when their tree is not on the path.
T_IDXS = [
    0.0, 0.00976562, 0.0390625, 0.08789062, 0.15625, 0.24414062, 0.3515625, 0.47851562,
    0.625, 0.79101562, 0.9765625, 1.18164062, 1.40625, 1.65039062, 1.9140625, 2.19726562,
    2.5, 2.82226562, 3.1640625, 3.52539062, 3.90625, 4.30664062, 4.7265625, 5.16601562,
    5.625, 6.10351562, 6.6015625, 7.11914062, 7.65625, 8.21289062, 8.7890625, 9.38476562, 10.0,
]
LAT_MPC_N = 16
LIMITER_DT_S = 0.020          # STEER_STEP = 2 on a 10 ms timer
ALIGN_TOL_MS = 30.0


def route_dirs(paths: list[Path]) -> list[Path]:
    """Accept route directories or a parent holding several. A route is a directory of numbered segments."""
    out = []
    for p in paths:
        if not p.is_dir():
            continue
        if any(c.is_dir() and c.name.isdigit() for c in p.iterdir()):
            out.append(p)
        else:
            out.extend(sorted(c for c in p.iterdir() if c.is_dir() and
                              any(g.is_dir() and g.name.isdigit() for g in c.iterdir())))
    return out


def read_segment(f: Path, capnp_log):
    """Events from one segment, tolerating a damaged or truncated tail.

    Both failure modes are normal in recordings and both used to take down the whole run. A recording that
    ended mid-write leaves the last `rlog.bz2` unpacking partially and its capnp stream stopping mid-message;
    one segment of this route set is corrupt outright and raised `OSError: Invalid data stream` from
    `bz2.decompress`, which killed a four-route sweep at the second route. So: decompress in chunks and keep
    whatever came out, then read events until the stream stops making sense. Returns (events, note) where
    note is non-empty when something was lost, because a silently shortened route is how a sweep comes to
    claim coverage it does not have.
    """
    raw = f.read_bytes()
    d = bz2.BZ2Decompressor()
    data, lost = b"", False
    step = 1 << 18
    for k in range(0, len(raw), step):
        try:
            data += d.decompress(raw[k:k + step])
        except (OSError, EOFError):
            lost = True
            break
        if d.eof:
            break
    if not data:
        return [], "не распаковался"
    evts, note = [], ""
    try:
        for evt in capnp_log.Event.read_multiple_bytes(data):
            evts.append(evt)
    except Exception:
        note = "capnp обрезан"
    if lost:
        note = (note + ", " if note else "") + f"bz2 битый, взято {len(data) / 1e6:.1f} МБ"
    return evts, note


def load_route(route: Path, max_segments: int | None, want_model: bool):
    """Read the events we need, in time order, as (timestamp_ms, kind, payload) tuples."""
    # cereal живёт в чужом дереве (openpilot/dragonpilot), путь к нему у каждого свой.
    root = os.environ.get("OPENPILOT_ROOT")
    if not root:
        raise SystemExit(
            "нужен OPENPILOT_ROOT — путь к дереву openpilot/dragonpilot, откуда берётся cereal:\n"
            "  OPENPILOT_ROOT=/path/to/openpilot python3 rlog_lat_diff.py <route dir>"
        )
    if root not in sys.path:
        sys.path.insert(0, root)
    from cereal import log as capnp_log

    segs = sorted([p for p in route.iterdir() if p.is_dir()], key=lambda p: int(p.name))
    if max_segments:
        segs = segs[:max_segments]

    events = []
    car_params = None
    damaged = []
    for seg in segs:
        f = seg / "rlog.bz2"
        if not f.exists():
            continue
        seg_events, note = read_segment(f, capnp_log)
        if note:
            damaged.append(f"{seg.name}: {note}")
        for evt in seg_events:
            w = evt.which()
            t = evt.logMonoTime * 1e-6
            if w == "carParams" and car_params is None:
                cp = evt.carParams
                car_params = {
                    "fingerprint": str(cp.carFingerprint),
                    "wheelbase": float(cp.wheelbase),
                    "steer_ratio": float(cp.steerRatio),
                    "mass": float(cp.mass),
                    "center_to_front": float(cp.centerToFront),
                    "tuning": cp.lateralTuning.which(),
                }
            elif w == "carState":
                cs = evt.carState
                # `steeringTorque` is the driver's torque and it is not decoration either: it widens the
                # limiter's clamp by STEER_DRIVER_ALLOWANCE, so leaving it at zero would make our limited
                # stream tighter than the one the car actually produced.
                events.append((t, "car", (float(cs.vEgo), float(cs.steeringAngleDeg), bool(cs.steeringPressed),
                                          float(cs.yawRate) if hasattr(cs, "yawRate") else 0.0,
                                          float(cs.steeringTorque))))
            elif w == "lateralPlan" and not want_model:
                lp = evt.lateralPlan
                events.append((t, "plan", (np.asarray(list(lp.dPathPoints), dtype=np.float64),
                                           np.asarray(list(lp.psis), dtype=np.float64),
                                           np.asarray(list(lp.curvatures), dtype=np.float64))))
            elif w == "modelV2" and want_model:
                mv = evt.modelV2
                events.append((t, "model", {
                    "x": np.asarray(list(mv.laneLines[1].x), dtype=np.float64),
                    "lanes": [np.asarray(list(ln.y), dtype=np.float64) for ln in mv.laneLines],
                    "probs": np.asarray(list(mv.laneLineProbs), dtype=np.float64),
                    "stds": np.asarray(list(mv.laneLineStds), dtype=np.float64),
                    "edges": [np.asarray(list(e.y), dtype=np.float64) for e in mv.roadEdges],
                    "plan_x": np.asarray(list(mv.position.x), dtype=np.float64),
                    "plan_y": np.asarray(list(mv.position.y), dtype=np.float64),
                    "plan_z": np.asarray(list(mv.position.z), dtype=np.float64),
                    "plan_yaw": np.asarray(list(mv.orientation.z), dtype=np.float64),
                    "plan_yaw_rate": np.asarray(list(mv.orientationRate.z), dtype=np.float64),
                }))
            elif w == "controlsState":
                # Their planner's own output. Kept separate from `carControl` because it is the one lateral
                # quantity of theirs that stays physical at low speed — see the module docstring.
                events.append((t, "curv", float(evt.controlsState.desiredCurvature)))
            elif w == "carControl":
                cc = evt.carControl
                out = cc.actuatorsOutput
                # `steerOutputCan` is the limited value in cNm. Falling back to `steer · STEER_MAX` only
                # matters for logs old enough to lack the field.
                applied = float(out.steerOutputCan) if hasattr(out, "steerOutputCan") \
                    else float(out.steer) * 300.0
                events.append((t, "cmd", (float(cc.actuators.steeringAngleDeg), float(cc.actuators.steer),
                                          applied, bool(cc.latActive))))
    events.sort(key=lambda e: e[0])
    return events, car_params, len(segs), damaged


def build_app(core, cfg_veh, cp, args):
    """Configure the app as it *ships*, not as its constructor defaults.

    Those defaults are not the car the phone drives: `max_steer_deg` is 8 in the constructor and 20 in
    `assets/config.json`, and the controller is `pp` there and `fp` in the file. A replay on constructor
    defaults measures a car that does not exist — the first run of this script had our torque at its ceiling
    in 85 % of frames purely because 8 degrees of road wheel normalises to 1.0 two and a half times sooner.
    """
    wheelbase = cp["wheelbase"] if cp else 2.62
    model_mode = args.reference == "model"
    app = core.AdasApp(wheelbase, -1.8, 0.5, 1.10, topic_convert=model_mode)
    app.set_lane_keep_controller(args.controller or cfg_veh.get("lane_keep_controller", "fp"))
    app.set_lane_keep_max_steer_deg(float(cfg_veh.get("max_steer_deg", 20.0)))
    app.set_lane_keep_steer_slew_limit_deg(float(cfg_veh.get("steer_slew_limit_deg", 8.0)))
    app.set_lane_keep_vehicle_model(bool(cfg_veh.get("lat_use_vehicle_model", True)),
                                    args.stiffness if args.stiffness is not None
                                    else float(cfg_veh.get("tire_stiffness_factor", 0.64)))
    app.set_lane_keep_fp_steer_delay_s(float(cfg_veh.get("fp_steer_delay_s", 0.35)))
    app.set_lane_keep_recompute_setpoint(bool(args.recompute_setpoint))
    if args.no_integrator:
        # See "the one unfairness" in the module docstring: with no feedback path our integrator winds up on
        # an error it cannot influence, so only ki = 0 compares controllers rather than replay artefacts.
        app.set_lane_keep_pid_gains(float(cfg_veh.get("pid_kp", 0.6)), 0.0, float(cfg_veh.get("pid_kf", 6e-5)))
    ratio = args.steer_ratio if args.steer_ratio else (cp["steer_ratio"] if cp else float(cfg_veh["steer_ratio"]))
    app.set_param("steer_ratio", ratio)

    if model_mode:
        # Our own fusion runs here, so it must run with shipped numbers — the same trap as above, one stage up.
        for k in ("path_lane_blend_scale", "path_camera_offset_m", "center_force_gain",
                  "lane_std_good_m", "lane_std_bad_m", "lane_std_range_m"):
            if k in cfg_veh:
                app.set_param(k, float(cfg_veh[k]))
        app.set_param("cam_y_left_m", float(cfg_veh.get("cam_y_left_m", 0.0)))
    else:
        # Their path is already the finished reference; ours must not add a second camera offset on top of
        # it, or the two setpoints differ before the controller is even reached.
        app.set_lane_keep_cam_y_left_m(0.0)
    return app, ratio


def lane_lines_message(pb, m, ts_ms, frame_id):
    """Their `modelV2` as our `LaneLines`. Per-point `y_std` is their scalar broadcast — see the docstring."""
    ll = pb.LaneLines()
    ll.timestamp = ts_ms
    ll.capture_ts_ms = ts_ms
    ll.frame_id = frame_id
    ll.x.extend(m["x"].tolist())
    for y, prob, std in zip(m["lanes"], m["probs"], m["stds"]):
        p = ll.lanes.add()
        p.y.extend(y.tolist())
        p.prob = float(prob)
        p.y_std.extend([float(std)] * len(y))
    for y in m["edges"]:
        e = ll.edges.add()
        e.y.extend(y.tolist())
    ll.plan_x.extend(m["plan_x"].tolist())
    ll.plan_y.extend(m["plan_y"].tolist())
    ll.plan_z.extend(m["plan_z"].tolist())
    ll.plan_yaw.extend(m["plan_yaw"].tolist())
    ll.plan_yaw_rate.extend(m["plan_yaw_rate"].tolist())
    ll.plan_hyp = 0
    return ll.SerializeToString()


def replay(core, events, cp, args, cfg_veh):
    """Run one route through the stack. Returns (ours, theirs, ratio) as float arrays."""
    app, ratio = build_app(core, cfg_veh, cp, args)
    app.start()
    if args.reference == "model":
        import lanes_pb2

    t0 = events[0][0]
    v_ego, driver_tq = 0.0, 0.0
    last_swa = last_curv = float("nan")
    ours, theirs, their_curv = [], [], []
    n_ref = 0
    for t, kind, payload in events:
        ts_us = int((t - t0) * 1000)
        if kind == "car":
            v_ego, ang, pressed, yaw, driver_tq = payload
            app.publish_chassis(ts_us, v_ego, 0.0, yaw, ang, pressed)
        elif kind == "plan":
            dpath, psis, curvs = payload
            n = min(len(dpath), LAT_MPC_N + 1, len(T_IDXS))
            if n < 6:
                continue
            xs = np.asarray(T_IDXS[:n]) * max(v_ego, 0.1)
            poly = [(float(x), float(y)) for x, y in zip(xs, dpath[:n])]
            plan_yaw = [float(p) for p in psis[:n]] if len(psis) >= n else []
            plan_yaw_rate = [float(c) * max(v_ego, 0.1) for c in curvs[:n]] if len(curvs) >= n else []
            app.publish_lanes(ts_us, poly, n_ref, poly, plan_yaw, plan_yaw_rate, True)
            n_ref += 1
        elif kind == "model":
            app.publish_lane_lines(lane_lines_message(lanes_pb2, payload, ts_us // 1000, n_ref))
            n_ref += 1
        elif kind == "cmd":
            theirs.append((t - t0, *payload))
        elif kind == "curv":
            their_curv.append((t - t0, payload))
        app.step(ts_us)
        for msg in app.pop_messages():
            if isinstance(msg, core.LaneKeepOutput):
                # `desired_swa_deg` is only filled on the chassis path, so the setpoint is reconstructed from
                # `steer_rad`, which the vision path does fill: SWA = road-wheel angle x steer ratio. That is
                # arithmetic on a published field, not a peek inside the service.
                if msg.has_target:
                    last_curv = msg.curvature
                    last_swa = msg.steer_rad * 180.0 / np.pi * ratio
                else:
                    last_curv = last_swa = float("nan")
            elif isinstance(msg, core.SteerCommand):
                ours.append((ts_us / 1000.0, last_swa, float(msg.torque_cnm),
                             1.0 if msg.enabled else 0.0, v_ego, driver_tq, last_curv))
    app.stop()
    return (np.asarray(ours, dtype=np.float64) if ours else np.zeros((0, 7)),
            np.asarray(theirs, dtype=np.float64) if theirs else np.zeros((0, 5)),
            np.asarray(their_curv, dtype=np.float64) if their_curv else np.zeros((0, 2)),
            ratio, n_ref)


def limited_stream(core, O):
    """Our command stream as the rack would have received it: zero-order held onto a 20 ms grid and run
    through the MQB limiter, exactly as `CarController` does when `frame_ % STEER_STEP == 0`.

    The zero-order hold is not a smoothing choice — it is what the actuator timer does. It picks up whatever
    the last published command was, and republishing the same value is how ~32 % of `controls/steer` frames
    behave on the road.
    """
    if len(O) == 0:
        return np.zeros((0, 4))
    grid = np.arange(O[0, 0], O[-1, 0], LIMITER_DT_S * 1000.0)
    idx = np.clip(np.searchsorted(O[:, 0], grid, side="right") - 1, 0, len(O) - 1)
    out = np.empty((len(grid), 4))
    last = 0
    for k, i in enumerate(idx):
        want = int(round(O[i, 2])) if O[i, 3] > 0.5 else 0
        # Not lat_active means apply_steer = 0 *and* the ramp restarts from zero on re-engagement, because
        # `apply_steer_last_` is assigned unconditionally. That reset is a real part of the behaviour.
        last = core.apply_driver_steer_torque_limits(want, O[i, 5], last) if O[i, 3] > 0.5 else 0
        out[k] = (grid[k], last, O[i, 3], O[i, 4])
    return out


def align(A, T, tol_ms=ALIGN_TOL_MS):
    """Index of the nearest `theirs` row for each row of A, plus a validity mask."""
    if len(A) == 0 or len(T) == 0:
        return np.zeros(len(A), dtype=int), np.zeros(len(A), dtype=bool)
    i = np.clip(np.searchsorted(T[:, 0], A[:, 0]), 0, len(T) - 1)
    j = np.clip(i - 1, 0, len(T) - 1)
    i = np.where(np.abs(T[j, 0] - A[:, 0]) < np.abs(T[i, 0] - A[:, 0]), j, i)
    return i, np.abs(T[i, 0] - A[:, 0]) <= tol_ms


def signed(ours, theirs):
    """Their sign convention is not ours, so measure the relationship rather than assume it."""
    c = float(np.corrcoef(ours, theirs)[0, 1]) if len(ours) > 2 else 0.0
    return ours * (-1.0 if c < 0 else 1.0), c


def report_torque(title, ours, theirs):
    o, c = signed(ours, theirs)
    d = o - theirs
    print(f"\n{title} (знак: корр {c:+.3f}), n={len(o)}")
    print(f"  |их|  медиана {np.median(np.abs(theirs)):6.0f}   p90 {np.percentile(np.abs(theirs), 90):6.0f}"
          f"   в упоре 300: {100 * np.mean(np.abs(theirs) >= 299):5.1f}%")
    print(f"  |наш| медиана {np.median(np.abs(o)):6.0f}   p90 {np.percentile(np.abs(o), 90):6.0f}"
          f"   в упоре 300: {100 * np.mean(np.abs(o) >= 299):5.1f}%")
    print(f"  расхождение: медиана {np.median(d):+.0f}, ск.кв {np.std(d):.0f}, "
          f"p90 |·| {np.percentile(np.abs(d), 90):.0f} cNm, наклон наш/их {np.polyfit(theirs, o, 1)[0]:.3f}")
    return d


def report_angle(oa_raw, ta, vv):
    oa, ca = signed(oa_raw, ta)
    da = oa - ta
    print(f"\nуставной угол руля на одном и том же пути, n={len(oa)} (знак: корр {ca:+.3f}):")
    print(f"  |их|  медиана {np.median(np.abs(ta)):6.2f}°  p90 {np.percentile(np.abs(ta), 90):6.2f}°")
    print(f"  |наш| медиана {np.median(np.abs(oa)):6.2f}°  p90 {np.percentile(np.abs(oa), 90):6.2f}°")
    print(f"  наш = {np.polyfit(ta, oa, 1)[0]:.3f} x их, корр {abs(ca):.3f}")
    print(f"  расхождение: медиана {np.median(da):+.2f}°, ск.кв {np.std(da):.2f}°, "
          f"p90 |·| {np.percentile(np.abs(da), 90):.2f}°")
    print(f"  доля кадров с расхождением больше 1.67° (порога, за которым наш PID сразу в упоре): "
          f"{100 * np.mean(np.abs(da) > 1.667):.0f}%")
    print("\n  по скорости:      n   |их| мед   |наш| мед   наклон наш/их   расхожд. ск.кв")
    for lo, hi in ((0, 5), (5, 10), (10, 15), (15, 20), (20, 30)):
        s_ = (vv >= lo) & (vv < hi)
        if s_.sum() < 50:
            continue
        note = "  ← сетка x = t·v вырождена, эталон короче машины" if hi <= 5 else ""
        print(f"    {lo:2d}-{hi:2d} м/с {s_.sum():7d}   {np.median(np.abs(ta[s_])):6.2f}°   "
              f"{np.median(np.abs(oa[s_])):6.2f}°        {np.polyfit(ta[s_], oa[s_], 1)[0]:5.2f}         "
              f"{np.std(da[s_]):6.2f}°{note}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("routes", type=Path, nargs="+", help="route dirs, or one parent holding them")
    ap.add_argument("--segments", type=int, default=6, help="how many minute-segments per route (0 = all)")
    ap.add_argument("--reference", choices=("plan", "model"), default="plan",
                    help="plan: their finished dPathPoints. model: their modelV2 through our lane fusion")
    ap.add_argument("--controller", default=None, help="override lane_keep controller (fp | pp)")
    ap.add_argument("--steer-ratio", type=float, default=None, help="override, e.g. their learned 16.12")
    ap.add_argument("--stiffness", type=float, default=None, help="tire_stiffness_factor override")
    ap.add_argument("--min-speed", type=float, default=5.0,
                    help="speed gate; what it removes is reported, not silently dropped")
    ap.add_argument("--recompute-setpoint", action="store_true",
                    help="recompute the setpoint between frames as upstream does at 100 Hz (lat_recompute_setpoint)")
    ap.add_argument("--no-integrator", action="store_true",
                    help="run our PID with ki=0 — the only fair way to compare torque in an open-loop replay")
    args = ap.parse_args()

    from pyadas import core

    routes = route_dirs(args.routes)
    if not routes:
        print("не нашёл ни одного маршрута (каталог с нумерованными сегментами)")
        return 1
    cfg = json.loads((Path(__file__).resolve().parents[1] / "assets" / "config.json").read_text())
    veh = cfg["vehicle"]
    print(f"маршрутов: {len(routes)}, эталон: {args.reference}, "
          f"контроллер {args.controller or veh.get('lane_keep_controller')}, "
          f"жёсткость {args.stiffness if args.stiffness is not None else veh.get('tire_stiffness_factor')}, "
          f"интеграл {'ВЫКЛ (честный момент)' if args.no_integrator else 'вкл (момент смещён открытым контуром)'}, "
          f"пересчёт уставки между кадрами {'ВКЛ' if args.recompute_setpoint else 'выкл'}")

    pool = {k: [] for k in ("pre_o", "pre_t", "post_o", "post_t",
                            "ang_o", "ang_t", "ang_v", "curv_o", "curv_t")}
    per_route = []
    angle_ceiling = float(veh.get("max_steer_deg", 20.0)) * (args.steer_ratio or float(veh["steer_ratio"]))
    for route in routes:
        events, cp, n_seg, damaged = load_route(route, args.segments or None, args.reference == "model")
        for d in damaged:
            print(f"  {route.name} / {d}")
        if not events:
            print(f"  {route.name}: событий нет, пропускаю")
            continue
        O, T, C, ratio, n_ref = replay(core, events, cp, args, veh)
        if len(O) == 0 or len(T) == 0:
            print(f"  {route.name}: наш стек не выдал команд (эталонов подано {n_ref})")
            continue

        # Before the limiter, at our own command rate.
        i, dt_ok = align(O, T)
        both_on = (O[:, 3] > 0.5) & (T[i, 4] > 0.5)
        fast = O[:, 4] > args.min_speed
        m = dt_ok & both_on & fast
        drops = {
            "slow": int((dt_ok & both_on & ~fast).sum()),
            "off": int((dt_ok & ~both_on).sum()),
            "align": int((~dt_ok).sum()),
        }

        # After the limiter, on the actuator's own 20 ms grid.
        L = limited_stream(core, O)
        li, ldt_ok = align(L, T)
        lm = ldt_ok & (L[:, 2] > 0.5) & (T[li, 4] > 0.5) & (L[:, 3] > args.min_speed)

        # Curvature: their planner against ours, on their own event stream rather than ours.
        ci, cdt_ok = align(O, C)
        cm = m & cdt_ok & np.isfinite(O[:, 6])

        # Angle: only where their setpoint is one our planner could have produced at all.
        am = m & np.isfinite(O[:, 1]) & (np.abs(T[i, 1]) <= angle_ceiling)
        drops["angle_wild"] = int((m & np.isfinite(O[:, 1]) & (np.abs(T[i, 1]) > angle_ceiling)).sum())

        per_route.append({
            "name": route.name, "segs": n_seg, "refs": n_ref, "ours": len(O),
            "matched": int(m.sum()), "matched_post": int(lm.sum()), "matched_curv": int(cm.sum()),
            "matched_ang": int(am.sum()), "fast_share": 100.0 * float(np.mean(fast)),
            "car": cp["fingerprint"] if cp else "?", "ratio": ratio, **drops,
        })
        if m.sum() >= 50:
            pool["pre_o"].append(O[m, 2])
            pool["pre_t"].append(T[i[m], 2] * core.STEER_MAX)
        if am.sum() >= 50:
            pool["ang_o"].append(O[am, 1])
            pool["ang_t"].append(T[i[am], 1])
            pool["ang_v"].append(O[am, 4])
        if cm.sum() >= 50:
            pool["curv_o"].append(O[cm, 6])
            pool["curv_t"].append(C[ci[cm], 1])
        if lm.sum() >= 50:
            pool["post_o"].append(L[lm, 1])
            pool["post_t"].append(T[li[lm], 3])

    if not per_route:
        print("ни один маршрут не дал данных")
        return 1

    print("\nпо маршрутам:")
    print("  маршрут                                сегм  наших   v>гейта   сопост.  кривизна  угол  "
          "после огр.    отброшено: медл / выкл / дикий угол")
    for r in per_route:
        print(f"  {r['name']:38s} {r['segs']:4d} {r['ours']:6d} {r['fast_share']:8.0f}% {r['matched']:9d} "
              f"{r['matched_curv']:9d} {r['matched_ang']:5d} {r['matched_post']:11d}    "
              f"{r['slow']:6d} / {r['off']:5d} / {r['angle_wild']:5d}")
    cars = {r["car"] for r in per_route}
    print(f"  машина: {', '.join(sorted(cars))}, передаточное {per_route[0]['ratio']:.2f}, "
          f"потолок уставного угла для сравнения {angle_ceiling:.0f}°")

    P = {k: (np.concatenate(v) if v else np.zeros(0)) for k, v in pool.items()}
    if len(P["pre_o"]) < 200:
        print("\nмало сопоставленных кадров для выводов")
        return 0

    if len(P["curv_o"]) >= 200:
        co, cc = signed(P["curv_o"], P["curv_t"])
        dc = co - P["curv_t"]
        print(f"\nкривизна, 1/м — выход планировщика до любой модели машины, n={len(co)} "
              f"(знак: корр {cc:+.3f})")
        print(f"  |их|  медиана {np.median(np.abs(P['curv_t'])):.5f}  p90 {np.percentile(np.abs(P['curv_t']), 90):.5f}")
        print(f"  |наш| медиана {np.median(np.abs(co)):.5f}  p90 {np.percentile(np.abs(co), 90):.5f}")
        print(f"  наш = {np.polyfit(P['curv_t'], co, 1)[0]:.3f} x их, корр {abs(cc):.3f}, "
              f"расхождение ск.кв {np.std(dc):.5f}")

    if not args.no_integrator:
        print("\nвнимание: интеграл включён, а контур разомкнут — момент ниже завышен нашим виндапом.")
        print("           честное сравнение момента: --no-integrator (угол и кривизна выше от этого не зависят)")
    d_pre = report_torque("момент ДО ограничителя, cNm", P["pre_o"], P["pre_t"])
    if len(P["post_o"]) >= 200:
        d_post = report_torque("момент ПОСЛЕ ограничителя, cNm — то, что доходит до рейки",
                               P["post_o"], P["post_t"])
        print(f"\n  ограничитель меняет вывод: ск.кв расхождения {np.std(d_pre):.0f} → {np.std(d_post):.0f} cNm, "
              f"p90 |·| {np.percentile(np.abs(d_pre), 90):.0f} → {np.percentile(np.abs(d_post), 90):.0f}")
        print("  (сравнение только до ограничителя льстит обеим сторонам — темповый предел срезает "
              "и запрос, и расхождение)")
    else:
        print("\nпосле ограничителя сопоставить не удалось — мало кадров")

    print("\nпо величине их момента (до ограничителя):")
    print("  |их| cNm       n    |наш| медиана   расхождение ск.кв   упор их / наш")
    o_pre, _ = signed(P["pre_o"], P["pre_t"])
    for lo, hi in ((0, 50), (50, 150), (150, 250), (250, 299), (299, 1000)):
        s_ = (np.abs(P["pre_t"]) >= lo) & (np.abs(P["pre_t"]) < hi)
        if s_.sum() < 30:
            continue
        print(f"  {lo:3d}-{hi:4d} {s_.sum():8d}      {np.median(np.abs(o_pre[s_])):6.0f}         "
              f"{np.std(d_pre[s_]):7.0f}          {100 * np.mean(np.abs(P['pre_t'][s_]) >= 299):4.0f}% / "
              f"{100 * np.mean(np.abs(o_pre[s_]) >= 299):4.0f}%")

    # The planner half decides whether the torque comparison above means anything — a controller fed a
    # different setpoint will disagree no matter how faithfully it was ported.
    if len(P["ang_o"]) > 200:
        report_angle(P["ang_o"], P["ang_t"], P["ang_v"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
