# `paramsd`: what it computes and what transfers to us

Analysis of `selfdrive/locationd/paramsd.py` and `models/car_kf.py` from dragonpilot
(`COMMA_VERSION 2023.04.13`). Needed for step 4 of the plan in
`BENCHMARK_COMMA2.md`: we set steer ratio, tire stiffness, and steering offset as constants,
and have no road grade at all — they learn all of that on the fly.

What it gives on our car, measured from logs (`BENCHMARK_COMMA2.md`): `steerRatio 16.27 ± 0.10`,
`stiffnessFactor 1.319 ± 0.007`, steering offset +0.094°, road grade median −0.78° (p10 −2.55°).

## Architecture

Kalman filter with 9 states on a linear bicycle model. Dynamics:

    ẋ = A·x + B·(δ − offset − offset_fast) − C·θ,   x = [v_y, ω]

where `δ` is steering angle, `offset`/`offset_fast` are slow and fast offsets, `θ` is road
grade. Matrices `A`, `B`, `C` are built from mass, wheelbase, weight distribution, and tire stiffnesses.

## States

| state | initial | process noise (σ) | published as | meaning |
|---|---|---|---|---|
| `STIFFNESS` | 1.0 | 0.05 %/step | `stiffnessFactor` | multiplier on tire stiffnesses from the port |
| `STEER_RATIO` | 15.0 | 0.01/step | `steerRatio` | steering gear ratio |
| `ANGLE_OFFSET` | 0 | 0.02°/step | `angleOffsetAverageDeg` | slow steering offset (mechanical) |
| `ANGLE_OFFSET_FAST` | 0 | 0.25°/step | part of `angleOffsetDeg` | fast offset: grade, wind, load |
| `VELOCITY` (v_x, v_y) | 10.0, 0 | 0.1 / 0.01 m/s | — | internal |
| `YAW_RATE` | 0 | 0.1 °/s | — | internal |
| `STEER_ANGLE` | 0 | 0.1° | — | internal |
| `ROAD_ROLL` | 0 | 1°/step | `roll` | **road grade** |

Uncertainty is published too: `steerRatioStd`, `stiffnessFactorStd`, `angleOffsetAverageStd`,
`angleOffsetFastStd` — from these you can see whether the estimate has converged.

## Observations

| observation | source | noise (σ) | condition |
|---|---|---|---|
| `STEER_ANGLE` | `carState.steeringAngleDeg` | 0.05° | active |
| `ROAD_FRAME_X_SPEED` | `carState.vEgo` | 0.1 m/s | active |
| `ROAD_FRAME_YAW_RATE` | `liveLocationKalman.angularVelocityCalibrated[2]`, negated | from message | `posenetOK`, σ in (0, 10), \|ω\| < 1 rad/s |
| `ROAD_ROLL` | `liveLocationKalman.orientationNED[0]` | 2× locator σ; if estimate invalid — **observe zero** with σ 10° | constrains state when the locator cannot be trusted |
| `ANGLE_OFFSET_FAST` | constant **0** | 10° | pulls fast offset toward zero |
| `STIFFNESS`, `STEER_RATIO` | **current value itself** | 0.5 and 5.0 | trick against σ growth |

The last row is a non-obvious trick worth porting with the filter: they "observe"
the state with itself at high noise. Without this, on long straights where there is no information
about stiffness and steer ratio, σ grows and estimates start jumping. Their own comment about this
says so directly.

## Gates and limits

| rule | value |
|---|---|
| active | `vEgo > 1` m/s **and** \|steering angle\| < 45° (linear model region) |
| max offset rate | 20 °/s |
| max grade rate | 20 °/s |
| validity flag | \|offset\| < 10°, 0.2 ≤ `stiffnessFactor` ≤ 5.0, `steerRatio` within port bounds, grade and its σ within bounds |
| `sensorValid` | \|v · (ω + ω_measured)\| below lateral-acceleration threshold |
| at standstill | timer reset so uncertainty does not grow |
| persistence | once per minute to parameters, together with car fingerprint |
| load at startup | if fingerprint differs or values are unsanitary — reset to port values |

## What transfers to us, and what it costs

**Portable without a locator** — `steerRatio` and `stiffnessFactor`: they are learned from steering angle,
speed, and yaw rate, all of which we have on CAN. These are the two numbers where we currently
differ from their learning (15.7 vs 16.27 and 0.64 vs 1.319).

**Needs more** — road grade. They take it from `liveLocationKalman.orientationNED`, i.e. from
their locator; we have no direct analogue: we have `localization/pose` and raw IMU. So either
estimate orientation ourselves (accelerometer coupled with a motion model), or on a first step take
grade from the accelerometer in steady driving and settle for a coarse estimate.

**Do not port** — their implementation: the filter uses `rednose` with code generation via sympy.
The filter itself is simple — 9 states, linear model, six observation types — and fits in
about 150 lines on Eigen. But the caveat from the plan remains: "port `paramsd`" is not just the filter,
but also where to get calibrated yaw rate and orientation.

## Link to localization

`paramsd` itself does not localize: it has no coordinates or heading; its `YAW_RATE` state is a model
quantity, not a measurement. In upstream the dependency runs **from** the locator **to** `paramsd`, not the other way.
Still, its outputs relate to our localization, more than it seems.

**Heading in `localization/pose` comes almost entirely from the bicycle model.** `VehicleEKF::predict`
advances heading with prediction `v·tan(δ)/L` and with that same prediction **overwrites** the yaw-rate
state. The measurement correction reaches heading only through cross-covariance in one step, with
weight `dt·(1 − K₄) ≈ 0.12·dt` at our `Q₄₄ = 0.05²` and `R_imu = 0.02²` (verified numerically; weight does not
depend on step size: 0.123 at both 10 ms and 50 ms). So heading = 0.88 · bicycle + 0.12 · measurement.

Hence the cost of wrong parameters: at measured under-steer 0.54 at 22 m/s prediction over-steers
by 1.85×, and heading turns **1.75×** faster than truth — on an arc with ω = 0.15 rad/s over 30 s that is 450°
instead of 258°. On the road this is masked by GNSS heading correction; without GNSS — it is not.

So learned `steerRatio`, `stiffnessFactor`, and steering offset matter here directly. But primary is still
**structural** treatment (yaw rate as a state with random walk, or heading from measured quantity) — see `BACKLOG.md`, "Localization" section. Parameters are needed where there is no measurement:
before GNSS heading seed and when IMU is invalid while CAN reports zero. One steering offset +0.094° gives
curvature 3.8·10⁻⁵ 1/m, i.e. 0.22° per 100 m — one third of the heading-drift allowance in `mapmatch` (0.6°/100 m).

**The `mapmatch` track is unaffected.** `buildTrack` integrates measured yaw rate and speed;
steering angle does not participate at all, so vehicle parameters do not enter the track shape.

**The reverse direction is more useful.** To port `paramsd` we need a source of calibrated yaw rate
instead of their `liveLocationKalman`, and we have two, already wired: camera odometry
(`CameraOdometrySample.rot[2]`, gate `rot_std < 0.5`) and phone IMU. The first is independent of CAN, and an
independent source is required — because:

**Identifiability trap.** From CAN alone, a scale error in the yaw-rate sensor is almost
collinear with `1/steer_ratio` (at low speed — exactly), and the filter silently writes it off as vehicle parameters.
So the under-steer contradiction (plan step 0) **cannot be closed by `paramsd` alone** — an external
reference is needed. We have two: `mapmatch` fits yaw-rate sensor scale against OSM centerlines (5 %
tolerance), and `analyzeYaw` compares CAN with the phone gyro.

Some of this is already known from `MAPMATCH.md`: integrating raw ESP signal gives total turn
+49° vs GNSS reference +47° over 3 km, i.e. scale is correct within a few percent and
a twofold gap is not explained by it. Plus sensor scale error **does not depend on speed**, while my
measurement does (0.80 at 12–15 m/s vs 0.54 at 21–26) — that is tire behavior, not sensor gain.
Suspicion therefore shifts to the κ_kinem side: `steer_ratio` and steering-angle definition.

Conclusion on plan step 0 unchanged: until CAN sensor scale is verified against an independent source,
`tire_stiffness_factor` must not be changed. But we now have what to verify with and what to compare against, and the most
likely suspect is no longer the sensor itself.
