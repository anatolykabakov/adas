# Plan: Bring Steering to Comma-Two Level

Compiled 2026-08-04 from dragonpilot logs with comma-two on **the same car and roads**
(`docs/VS_DRAGONPILOT_0803.md`) and analysis of our runs (`docs/RUN_0802_ARC_OFFSET.md`).

## Target Expressed as a Number

Cross-track offset from lane center at the vehicle, median absolute value:

| segment | comma-two | our ADAS | manual driver | **target** |
|---|---|---|---|---|
| straight | 0.08 | 0.12 | 0.16 | 0.10 |
| left arc | 0.07 | **0.51** | 0.17 | **0.20** |
| right arc | 0.20 | **0.71** | 0.15 | **0.20** |

On straight we are already smoother than the human; gap is only on arcs — there it is 3–7×. Measure with
`bag_arc_offset.py`; engagement flag — `controls_allowed` from `panda/health`.

Breakdown shows where the gap is:

| | comma-two | our ADAS |
|---|---|---|
| tracking error, straight / left / right | +0.00 / +0.03 / −0.08 | +0.07 / +0.35 / −0.23 |
| setpoint offset | −0.06 / −0.06 / −0.08 (constant) | −0.10 / +0.15 / −0.42 |

So their controller sits on its reference, and reference equals lane center plus constant camera
shift. We achieve neither.

## Step 0. Resolve Understeer Contradiction — Before Any Vehicle Model Changes

Two measurements contradict each other; until resolved, do not touch `tire_stiffness_factor`.

* **their learning** on this car: `stiffnessFactor = 1.319 ± 0.007`, i.e. car is **stiffer**
  than nominal. Our formula at `tsf = 1.0` gives exactly their port stiffness (184691 / 238876 N/rad),
  so quantities are directly comparable, and our `0.64` is 2.06× the other way;
* **my measurement from our runs**: κ_fact/κ_kin = 0.80 at 12–15 m/s and 0.54 at 21–26, meaning
  understeer **stronger** than even our aggressive 0.64 (it predicts 0.675 at 22 m/s).

One of the two is wrong. Suspect in my measurement — **yaw rate source**: I used
`vehicle/state.yaw_rate` from CAN. If ESP has different scale, entire measurement shifts. Ready check:
`mapmatch::analyzeYaw` computes correlation and mutual scale of CAN sensor and phone gyro.

Two independent measurements already exist, both saying **not the sensor**: camera odometry vs
ESP gives scale 0.92 at correlation −0.879 (`RUN_0804_PERCEPTION.md`, section 6 — also fixed
57.3 multiplier that made this source unusable), and integrating raw ESP in
`mapmatch` gives full turn +49° vs GPS reference +47°, i.e. +4 %. A few-percent scale error does not explain twofold gap, and it does not depend on speed, while my measurement
does (0.80 at 12–15 m/s vs 0.54 at 21–26). Remains κ_kin side: `steer_ratio` and
steer angle definition.

Order: first verify yaw scale via IMU, then recompute κ_fact/κ_kin with learned
`steer_ratio = 16.27` instead of 15.7, only then decide on stiffness. If after that measurement
still contradicts their learning — trust their learning: spread 0.007 over 9.5k records
on this same car.

**Effort**: hours. **Verification**: `analyzeYaw` on two runs plus understeer table recompute.

## Step 1. Vehicle Parameters (Config Only)

All three numbers from this same car, from comma `liveParameters`, low spread.

| parameter | now | set to | rationale |
|---|---|---|---|
| `steer_ratio` | 15.7 | **16.27** | learned, ±0.10; their port marks 15.6 "let it learn" |
| `tire_stiffness_factor` | 0.64 | **1.319** | learned, ±0.007; our formula at 1.0 gives their stiffness |
| `fp_steer_delay_s` | 0.35 | **0.30** | `steerActuatorDelay = 0.10` plus +0.2 baked in code |
| `wheelbase_m` | 2.636 | 2.62 | their port; if our 2.636 is measured — keep ours |

Expected effect: command multiplier at 22 m/s drops from 1.48 to 1.23, i.e. stop
over-commanding at speed. This also explains why stronger compensation (`tsf 0.50`) in my replay
made offset worse.

**Effort**: minutes of edits, but only after step 0. **Verification**: closed loop
`bag_config_sweep --set arcs --real-vision-rate`, then road run.

## Step 2. Inner Loop at 100 Hz (Done and **Verified on Vehicle 2026-08-04**)

Their `controlsState` runs **100 Hz**, planner 20 Hz. Our angle PID ran at
`vehicle/state` rate, which was 20 Hz due to CAN receive timer 50 ms. Steer angle on MQB is sent on CAN
at 100 Hz, so this was our setting, not a limit.

Done: `rx` timer in `PandaService` 50 ms → 10 ms. HCA TX was already 100 Hz.

Verified in run `2026_08_04_11_15_15`: `vehicle/state` at **10 ms** (p90 11, p99 18, max 88),
`controls/steer` also 10 ms, panda callback 1.1 ms mean, 186 054 messages over 30 min with
no drops. USB and DBC parsing hold.

This run did not measure tracking error: control was blocked by panda `health` packet version
mismatch (`RUN_0804_PERCEPTION.md`, section 1; fixed). Side effect appeared — vision rate
tail became 3× worse (p99 179 → 609 ms); cause between daytime heating and CAN receive cost
not yet separated.

## Step 3. Road Grade Estimate and Compensation (No New Code Yet)

Only found cause of **constant** offset not explained by anything else. From their logs on
your route: grade median −0.78°, p10 −2.55°, max 3.68°. Median 0.8° gives lateral
acceleration 0.14 m/s², i.e. constant curvature 3.4·10⁻⁴ at 20 m/s. Without compensation controller
counters only with cross-track feedback, which is weak for us — hence tens of centimeters.

They estimate grade with `paramsd` and feed it into desired-curvature compensation.

**Effort**: grade estimate is part of step 4; faster path — grade from phone accelerometer at
steady cruise. **Verification**: offset on straight segments with known profile.

## Step 4. Port `paramsd` — Learn Parameters Instead of Constants

9-state Kalman filter on linear bicycle model: tire stiffness, steer ratio, slow and fast steering bias, velocity (x, y), yaw rate, steer angle, **road
grade**. Observations — steer angle, yaw rate, speed.

Closes steps 1 and 3 at once and removes hand-tuning: we set `steer_ratio`,
stiffness, and bias by hand; they learn them. Their implementation on `rednose` with sympy codegen —
not portable, but filter itself is simple and fits in ~150 lines on Eigen.

Full breakdown — states, noise, observations, gates, and what is portable without their localizer —
in [`PARAMSD.md`](PARAMSD.md). Key caveat: `steerRatio` and `stiffnessFactor` learn from
CAN; road grade they take from `liveLocationKalman`, which we lack.

**Effort**: several hours plus tests. **Verification**: convergence to their numbers (16.27 / 1.319 / +0.09°)
on our runs, and estimate stability between runs.

## Step 5. Reference: Lane Blending and σ Threshold

Their reference **equals lane center**: setpoint offset constant (−0.06…−0.08 = their `CAMERA_OFFSET`),
lane paint in reference 92–100 % of arc frames, `d_prob ≈ 1.0`. Their plan offset (+0.12 / −0.25) is fully
absorbed by blending.

We have `path_lane_blend_scale = 0.6`, i.e. 40 % of plan offset flows into reference. Raising to 1.0 —
but **after step 2**: at real command rate (12.5 Hz) difference between 0.3, 0.6, and 1.0 almost vanished,
so benefit appears only with fast loop.

On σ separately: our 0.19–0.93 vs their 0.04–0.13 — **hardware gap, not code**. Input pipeline
matches flowpilot (1280×720, warp to medmodel 910 / cy 47.6), and measured FOV of our
camera (65.6°) matches EON camera (65.2°). Difference is lens and ISP processing.

**Clarified 2026-08-04 by reading their code (previous wording here was wrong).** flowpilot on
Android runs `CameraHandler` (`AndroidLauncher.java:99`), and it **does not touch tone curve,
noise reduction, or sharpening** — only AE metering region, 20 fps, and autofocus. Curve
`TONEMAP_MODE_CONTRAST_CURVE` exists in their `CameraManager` on CameraX, but that branch is **commented out**
(`AndroidLauncher.java:100`). So for ISP we match flowpilot, and "do like
flowpilot" here means nothing — on a phone they would have the same σ.

Hence sub-step:

**5a. AE metering region — the only thing flowpilot has and we lack.** They meter
exposure on a rectangle on the road (`CONTROL_AE_REGIONS`, frame center below horizon: x 0.4–0.6 W,
y 0.5–0.7 H, weight 1000); in commented branch — wide strip 0.05–0.95 W, 0.4–0.8 H. We do not
set region at all, so exposure is over full frame including sky, and road is
systematically underexposed. Verify σ on short recording before/after, **separate run**: one
camera change at a time, otherwise cannot separate from blending.

**5b. Disable ISP processing** (`EDGE_MODE`, `NOISE_REDUCTION_MODE`, `TONEMAP_MODE` with linear
curve) — still worth trying, but as our own hypothesis, not upstream port:
in `TEMPLATE_RECORD` noise reduction and sharpening are on by default for both. Set σ thresholds from **our**
distribution, not copy 0.15/0.30 from upstream: 0.8 was taken from there and on
arcs cut good lane paint.

## What NOT to Do — Verified and Dropped

* **curvature plan correction** (`50.2·κ` or `½κ(vT)²`): not reproducible between runs (T 0.84
  vs 0.56, residual +0.23 m), because mixed with camera calibration. Upstream does not correct plan —
  they do not use it when lanes are visible;
* **centering term** (`center_force`): at real command rate does nothing (0.21 → 0.19) and
  eats stability margin — at 1.2 left arc diverges (0.81 m, p95 2.95). Disabled. Revisit
  only after step 2;
* **lengthen MPC horizon** from 16 to 32 nodes: near zone is the same (11 of 17 nodes in first
  second), lengthening only adds far nodes and costs ~4× more;
* **principal point fix** without online calibration reset: doubles σ (0.25 → 0.55), because
  principal-point shift is equivalent to rotation and already absorbed in pitch/yaw estimate;
* **steer rate penalty** (`fp_steering_rate_weight` 150 or 800): difference ≤ 0.03 m.

## Order and Expected Outcome

1. **step 0** — resolve understeer contradiction (hours, measurements only);
2. **step 1** — three numbers in config (minutes);
3. **step 2** — verify 100 Hz on device (one run);
4. **step 5a** — disable ISP processing, measure σ (one short recording);
5. **step 3/4** — road grade, then full `paramsd` (days);
6. **step 5** — `blend` toward 1.0, remeasure centering.

Steps 1–2 should give most of the gain: correct feedforward plus fast inner loop — that is
exactly how they hold near-zero tracking error. Steps 3–4 remove constant offset. Step 5
removes setpoint offset.

What this plan **does not** promise: comma-two-level σ. Model is trained on automotive camera, and on
a phone it is out of domain; our ceiling is thresholds adequate for our hardware.
