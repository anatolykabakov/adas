# Lateral control pipeline audit on run 2026_08_01_01_14_22

36 minutes, 24 810 planner frames, 10 windows with HCA on (20 % time), rest — driver steering.
First run with fixed time step (`FRAME_DT_FIX_0801.md`).

Segment analyses: [`RUN_0801_STRAIGHTS.md`](RUN_0801_STRAIGHTS.md),
[`RUN_0801_CURVE.md`](RUN_0801_CURVE.md), [`RUN_0801_LOWSPEED.md`](RUN_0801_LOWSPEED.md).

## Summary table

| # | Finding | Severity | Status |
|---|---|---|---|
| 1 | κ→angle computed kinematically, car achieves only 60 % curvature | high | **fixed** |
| 2 | Lateral command active below walking speed (20.8° at 0.9 m/s) | high | **fixed** |
| 3 | PID integrator assumed 50 Hz, called ~31/s from two places | medium | **fixed** |
| 4 | Debug topic published before PID → bags recorded wrong torque | medium | **fixed** |
| 5 | Simulator closed-loop kinematic — overstated response up to 65 % | medium | **fixed** |
| 6 | `mpc_max_steer_deg` / `steer_slew_limit_deg` in wheel degrees → never trigger | low | documented |
| 7 | HCA torque hits 300 cNm on arcs < 50 m, outward up to 0.9 m | — | not fixable by tuning |
| 8 | Camera roll not estimated (always 0), live pitch/yaw drifted 1.0–1.4° from prior | low | proposed |
| 9 | Model plan drifts itself (σ 0.14–0.19 m) and only 45–50 % returns to center | medium | proposed |
| 11 | Model own left bias −0.096 m, loop amplifies twofold → 0.21 m drift | high | **fixed** (`path_camera_offset_m = 0.09`) |
| 10 | 23 TX frames blocked by Panda (command while `controls_allowed = 0`) | low | consequence of #2 |
| 12 | Lane markings blended from single line (center extrapolated 1.6 m) | high | **fixed**: need both lines + width 2.6–4.6 m (filtered) |

## Measured vehicle model (basis for most conclusions)

Regression κ_fact = yaw/v vs κ_kinem = tan(SWA/15.7)/2.636, 19k frames, corr 0.98–0.997:

| v, m/s | 4–8 | 8–12 | 12–16 | 16–20 | 20–26 |
|---|---|---|---|---|---|
| κ_fact/κ_kinem | 0.99 | 0.93 | 0.85 | 0.76 | **0.61** |
| VM openpilot, tsf = 1.0 (stock) | 0.98 | 0.94 | 0.89 | 0.83 | 0.76 |
| VM, tsf = 0.64 (tuned) | 0.97 | 0.90 | 0.84 | 0.75 | 0.66 |

Yaw rate prediction check from recorded steering: RMS 0.32 °/s with model `tsf = 0.64` and
120 ms delay vs 0.52 °/s kinematic; slope by speed bins 0.91–1.03 vs
0.60–0.98. Model in `cpp/include/utils/vehicle_model.h` and `scripts/core/vehicle_model.py`
(same coefficient both sides).

### Pipeline delays

| link | value |
|---|---|
| capture → command publish | 69 ms (p90 95) |
| command → actual rack angle | 40 ms |
| rack angle → yaw rate | 120 ms |
| **capture → vehicle response** | **230 ms** |

## What was fixed

### 1. Curvature to angle via vehicle model

`fp` commanded `atan(L·κ)·ratio`, i.e. knowingly less than needed: at 22 m/s 40 % shortfall.
Now `steerFromCurvature(κ, v, L, sf)` with tuned `tire_stiffness_factor = 0.64`.
Disable with `vehicle.lat_use_vehicle_model = false`.

Closed-loop on measured model with vision delay in loop (window 1360–1520 s, clean markings,
17–22 m/s):

| configuration | \|center\|med | p95 | \|ΔSWA\| |
|---|---|---|---|
| as in run (kinematic) | 0.34 m | 1.24 m | 0.05 |
| **+ vehicle model** | **0.26 m** | **0.79 m** | 0.07 |

Command jitter did not grow — loop gain increase is stable.

### 2. Minimum speed gate

Below 1.5 m/s (hysteresis 0.5) command zeroed, MPC and PID state reset. Removes
+20.8° spikes on launch and −23.7° before stop where 2.5 s horizon collapses to
two meters. Stock analogue — `MIN_LATERAL_CONTROL_SPEED = 0.3` (ours higher due to short
horizon). Test `LaneKeepServiceMpc.LowSpeedGateHoldsZeroAndHasHysteresis`.

### 3. PID integrator rate

`LatControlPid` created with `rate_hz = 50`, but `updateTorqueFromAngle` called from
`onChassis` (20 Hz) and `onLanes` (11 Hz) — ~31/s total. Integral term was
~1.5× high relative to tuning. Now period measured between calls.

### 4. Debug topic published before PID

`control/lane_keep_debug.steer_norm` and `torque_cnm` recorded before torque calc and contained
geometric fraction δ/δ_max, not real command. Hence arc analysis first showed
"torque 55 cNm" when actual was 300 (saturation). Publish moved after PID.

### 5. Closed-loop simulator

`bag_mpc_sim --mode closed` integrated `ψ̇ = v·tan(δ)/L`, so simulator car
responded 65 % faster than real at 22 m/s — exactly hiding finding #1.
Now uses `core.vehicle_model.LateralPlant` (shortfall + two delays). Added
`scripts/bag_config_sweep.py`: config comparison with centering metric vs
lane markings, pose resync and vision delay in loop.

## Checked and OK

* **Camera y offset.** Estimate over 11k frames (line fit to lane midline) gives +0.111 m vs `cam_y_left = 0.10` in config — within 1 cm.
  Method does not separate mount from driver habit (spread across runs ±0.07 m).

  > **Clarified 2026-08-01 on run 13_40_48.** Conclusion "with HCA on car is centered anyway,
  > so `CAMERA_OFFSET` not needed" was wrong: center held only because
  > kinematic under-steer prevented loop reaching plan equilibrium. After vehicle model fix
  > equilibrium exposed — 0.21 m left drift, exactly as plan bias regression predicts.
  > `path_camera_offset_m` restored at measured **0.09 m**. Analysis: [`RUN_0801_LEFT_DRIFT.md`](RUN_0801_LEFT_DRIFT.md).
* **Localization.** `LocalizationService` computes dt from message stamps with fallback only on
  first frame — no same bug as lane keep.
* **Panda / CAN.** `tx_overflow = 0`, `heartbeat_lost = 0`. 23 blocked TX frames —
  consequence of finding #2 (command while `controls_allowed = 0`), after gate should not occur.
* **Feedforward.** Measured 230 ms less than hardcoded 350 ms, but sweeps showed reducing
  cannot: lead must cover vehicle delay too (0.23 → 0.41 m centering).
  Left at 0.35.
* **Steering rate penalty** (`fp_steering_rate_weight`): 200 / 400 / 700 give same
  result (0.26 m). Left at 400.

## Dead limiters (finding #6)

`mpcMaxSteerRad()` and `steer_slew_limit_deg` set in **wheel** degrees but intended as "safe" thresholds:

| parameter | value | steering equivalent | real peak in run |
|---|---|---|---|
| `mpc_max_steer_deg` | 25° wheels | 392° steering | 3.7° wheels (tight arc) |
| `mpc_low_speed_steer_deg` | 8° wheels | 126° steering | — |
| `steer_slew_limit_deg` | 8° wheels/frame | 126° steering/frame | 0.5° wheels/frame |

None triggered once in 36 minutes (`slew_clipped = 0.00` on all segments). Actually
limited only by jerk limit inside `get_lag_adjusted_curvature` (~0.48° wheels/frame at
12 m/s). Proposal: set `mpc_max_steer_deg ≈ 6°`, `steer_slew_limit_deg ≈ 1.5°` — still
2× above any observed value, but real protection. Not done: change
safeties without road check.

## Related documents

* [`CALIBRATION_FROM_BAGS.md`](CALIBRATION_FROM_BAGS.md) — what calibration (roll / y / x /
  scale) is measurable from runs.
* [`CONTROLLER_LIMITS.md`](CONTROLLER_LIMITS.md) — applicability limits: roads, speeds,
  arc radii, marking availability.
* [`RUN_0801_LEFT_DRIFT.md`](RUN_0801_LEFT_DRIFT.md) — left drift and compensation.

## Proposals (not implemented)

1. **Actuator limit indication.** Flag `steer_limited` in debug topic when torque saturated
   longer than ~0.5 s plus UI hint. On R 41 m arc that is the only honest signal to driver.
2. **Speed limit from curvature ahead** in `long_plan` (a_lat ≤ 2.5 m/s²) — then
   car approaches arc at speed HCA can handle.
3. **Lane blending** (done 2026-08-03: **0.6**, σ threshold 0.3/1.5, width up to 4.6 m,
   centering term added — `RUN_0802_ARC_OFFSET.md`). Then proposed `0.5`: centering in sweeps better
   (0.23 vs 0.26 median, offset +0.14 vs +0.21), but p95 worse on some windows because
   we lack stock weight reduction by `laneLineStds` (field not in protocol). Candidate
   for road A/B, not default.
4. **Camera roll.** Calibration estimates only pitch/yaw, roll always 0. Live pitch/yaw over run
   drifted from prior by 1.4° and 1.05° — mechanism works, but phone roll
   not compensated. Roll estimable from lane symmetry on straights.
5. **`v_plan` from model.** `plan_vx` already parsed (`ModelLongParse`), but not in
   `LanePathMsg`; planner holds speed constant over horizon. On accel/brake that is
   noticeable anchor error.

## Verification

```bash
# build + tests (41 pass; MiddlewareTest.SnapshotStatsTracksTimers failed before fixes too)
cmake --build app/src/main/cpp/build-linux -j && cmake --build app/src/main/cpp/build-tests -j
./app/src/main/cpp/build-tests/tests/adas_tests

# closed-loop config comparison on measured model
PYTHONPATH=app/src/main/scripts python3 app/src/main/scripts/bag_config_sweep.py \
  adas_logs/2026_08_01_01_14_22 --t0 1360 --t1 1520

# open loop vs driver
PYTHONPATH=app/src/main/scripts python3 app/src/main/scripts/bag_controller_ab.py \
  adas_logs/2026_08_01_01_14_22 --t0 1360 --t1 1520 --controllers fp,mpc
```

Android build (`./scripts/build_project.sh --cpp-only`) passes.
