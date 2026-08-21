# Benchmark vs comma-two and closing the gap

Compiled 2026-08-03/04 from dragonpilot logs on **the same Golf 7 and roads**, plus our
`bag/bag_arc_offset.py` metrics. Replaces the former `VS_DRAGONPILOT_0803` / `PLAN_TO_COMMA2` pair.

---

## 1. Measurement setup

comma-two + dragonpilot: `dragonpilot_rlog_lite/rlog_lite`, 4 routes, 33 segments, 26.1 min,
42 % usable (`carControl.latActive`, hands off, lanes visible).

Tool: `rlog_arc_offset.py` (retired to git history 2026-08-20 with the rest of the porting-era stands) — same metric
as `bag/bag_arc_offset.py`.

| quantity | source |
|---|---|
| lane center / curvature | `modelV2.laneLines[1,2].y`, fit x∈[0,30], at x=0 |
| model plan | `modelV2.position.y` |
| **reference at vehicle** | **`lateralPlan.dPathPoints[0]`** |
| fast loop | `controlsState.desiredCurvature`, 100 Hz |

Pitfalls: use `latActive` (not `controlsState.active` with `dp_atl`); κ desired is right-positive.

---

## 2. Results (cross-track at vehicle, m; left +)

| segment | **dragonpilot** | our ADAS, blend 0.3 (0802) | **our ADAS, blend 0.61 (0804 night)** | manual | **target** |
|---|---|---|---|---|---|
| straight | **0.08** | 0.12 | **0.04** | 0.16 | **0.10** |
| left arc | **0.07** | 0.51 | **0.30** | 0.17 | **0.20** |
| right arc | **0.20** | 0.71 | **0.00** | 0.15 | **0.20** |

| | comma-two | our ADAS 0802 | **our ADAS 0804 night** |
|---|---|---|---|
| tracking error (str / L / R) | +0.00 / +0.03 / −0.08 | +0.07 / +0.35 / −0.23 | +0.02 / **+0.29** / +0.09 |
| setpoint offset | ≈ camera offset (−0.06…−0.08) | −0.10 / +0.15 / −0.42 | −0.06 / **+0.02** / **−0.10** |

Blending closed the setpoint gap: on arcs it is now +0.02 / −0.10 against +0.15 / −0.42. What remains on
the left arc is pure tracking error (+0.29 of +0.30), and the wheel does reach the commanded angle
(+13.8° asked → +13.5° achieved) — so the command itself is too small, i.e. understeer compensation is
too weak. Right-arc zero is partly luck: σ there is 1.49, blending collapses to d = 0.10, the reference
follows the model plan (−0.12), and tracking error (+0.09) happens to cancel it.

---

## 3. Causes

1. **Fast loop** — they run angle PID at 100 Hz; we did ~12.5 Hz. Done: CAN RX 50→10 ms (verified 2026-08-04).
2. **Lane blending** — their reference ≈ lane center when paint is visible; justifies `blend→1.0` after fast loop. No curvature-plan correction upstream.
3. **Lane σ** — theirs ~0.05, ours 0.2–0.9 on arcs. Warp/FOV match; gap is phone ISP / AE. Try AE road metering (5a) and ISP off (5b). Do not expect 0.05 on a phone.

---

## 4. Pipeline: matches vs diverges

**Matches:** 1280×720, medmodel warp FL910/CY47.6, raw YUV, `fp` MPC math, stiffness at `tsf=1.0` = VW port, angle PID coeffs, panda STEER_MAX/deltas, HCA 100 Hz.

**Diverges / missing:** blend 0.6 vs ~1.0; planner ~12.5 vs 20 Hz; σ thresholds (different definitions); `paramsd` + road grade; lane change.

---

## 5. Work plan

| step | action | status |
|---|---|---|
| 0 | ~~Resolve understeer contradiction before editing tsf~~ — **closed 2026-08-04**: CAN yaw is sound (see below), so our measurement stands and tsf must move **down**, not to 1.319 | **done** |
| 1 | ~~Config: `tsf` **0.50 → 0.40**~~ | **dropped 2026-08-07** — it moves *away* from comma (their learner settled at 1.247–1.319 on this car) on the strength of a fit whose curve is flat, and §5a resolved the disagreement as identifiability rather than tuning. See `BACKLOG.md` §0 |
| 1a | Setpoint recomputed between frames, the one confirmed inner-loop divergence | **closed 2026-08-13**: the control law moved into `Control` and runs at a fixed 100 Hz, so the setpoint follows speed by construction; the `lat_recompute_setpoint` flag and its planner-side mechanism are gone |
| 2 | Inner loop 100 Hz | **done** 2026-08-04 |
| 3 | Road grade estimate | open |
| 4 | Port `paramsd` — see [`PARAMSD.md`](PARAMSD.md) | open |
| 5a/5b | AE metering region; optional ISP off; blend→1.0 after fast loop | open |

### Step 0 closed: the ESP yaw sensor is sound

Fixing the `cameraOdometry.rot` units (they were 57.3× too small) made a three-way comparison possible.
On turns (|ω| > 0.02 rad/s, N = 3565, run `2026_08_04_21_00_18`):

| pair | scale | corr |
|---|---|---|
| phone gyro / ESP | **1.017** | +0.969 |
| camera / ESP | 0.849 | −0.992 |
| camera / gyro | 0.788 | +0.962 |

Gyro vs ESP by speed: 1.012 (8–14 m/s), 1.041 (14–20), 1.025 (20–30) — no speed dependence. Two
independent physical sensors agree within 2–4 %; the outlier is the camera, consistent with its known
metric scale error (`trans_x/v_ego` = 0.893).

So the understeer measurement (κ_actual/κ_kinematic = 0.80 at 12–15 m/s, 0.54 at 21–26) was taken with a
trustworthy yaw source and stands. Their `stiffnessFactor = 1.319` still contradicts it, but the suspect
is no longer the yaw sensor: it is the κ_kinematic side (steering-angle scale, possibly a variable-ratio
rack) or the comparability of their factor with our formula.

**Do not:** curvature plan correction, `center_force` at 12.5 Hz, horizon 16→32, cx/cy fix without calib reset,
`tsf` → 1.319.

Does **not** promise comma-two σ on phone camera.

---

## 5a. Differential replay: their inputs through our stack (2026-08-07)

The (since retired) `rlog_lat_diff.py` pushed their recorded route through `AdasApp` and diffed stage by
stage. 101 975 matched frames, 28 usable segments, `v > 10 m/s`, both sides actuating:

| stage | theirs | ours | agreement |
|---|---|---|---|
| curvature, 1/m | median 0.00042 | 0.00034 | slope 0.897, **corr 0.966** |
| setpoint angle | median 1.40° | 1.12° | slope 0.930, corr 0.965, **2.40° rms** |
| torque, `ki = 0` | median 69 cNm | 78 cNm | median disagreement +10 cNm |
| torque, `ki = 0.2` | median 69 cNm | 229 cNm | **artefact — see below** |

**The port is faithful through the planner and the vehicle model.** That was not previously demonstrable: the
same harness used to report our torque as three times theirs, and the cause was the replay being open loop —
the measured angle is the one *their* torque produced, so our integrator accumulates an error our command
cannot influence. Torque is only meaningful here with `--no-integrator`.

**What is left is a band, not a constant.** With the integrator off we still rail at ±300 in 21.7 % of frames
against their 10.1 %, and the setpoint disagreement that explains it is concentrated at **10–15 m/s: 4.22° rms
against 1.25° at 15–20 and 0.51° at 20–30**. Since 1.67° of error already rails our PID, that band is where
the remaining lateral gap on this car lives — and it is upstream of the controller.

**Their setpoint angle cannot be used below ~10 m/s at all**: `actuators.steeringAngleDeg` reaches 649.7° with
a p90 of 130.8° even when `latActive`, because a short plan implies a large curvature and their vehicle model
turns it into an angle no rack has. `desiredCurvature` stays physical over the same frames, which is why
curvature is now the primary comparison.

## 6. Related

* [`PARAMSD.md`](PARAMSD.md) · [`CONTROLLER_LIMITS.md`](CONTROLLER_LIMITS.md) · [`BACKLOG.md`](BACKLOG.md)
