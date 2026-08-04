# Comparison with dragonpilot on the Same Car (2026-08-03)

Drive on comma-two with dragonpilot on a route with straights and arcs — same Golf 7, same roads as
our runs. Logs: local `dragonpilot_rlog_lite/rlog_lite` dump, 4 routes,
33 segments, 26.1 min, 42 % of frames usable (lateral control active, hands off wheel,
lanes visible).

Purpose: until now we argued about levers from simulation and one run. Here upstream is measured with **the same metric on the same car**, so the argument closes with numbers.

Tool: `app/src/main/scripts/rlog_arc_offset.py` — reads rlog via cereal schema from an
openpilot/dragonpilot checkout (`OPENPILOT_ROOT` / `--op-root`) and computes the same
breakdown as `bag_arc_offset.py`.

## How Their Logs Were Read

| quantity | source |
|---|---|
| lane center and its curvature | `modelV2.laneLines[1,2].y`, quadratic fit over x∈[0,30], value at x=0 |
| model plan | `modelV2.position.y` |
| **reference at vehicle** | **`lateralPlan.dPathPoints[0]`** — plan after lane blending, camera shift, and centering |
| tracking error | same: vehicle is at y = 0 in this frame |
| curvature trajectory | `lateralPlan.curvatures` — 17 nodes |
| fast loop | `controlsState.desiredCurvature`, 100 Hz |

Two pitfalls that made the first measurement zero usable frames:

* **`controlsState.active` true in only 1 %.** Parameter `dp_atl = 1` (always-on
  lateral) is enabled: lateral control works even when openpilot is not "engaged". Correct flag —
  `carControl.latActive`, true in 65 % of frames;
* **`desiredCurvature` is right-positive** (plan frame), while `steeringAngleDeg`
  is left-positive. So corr(κ lane, desiredCurvature) = −0.73 is normal, not sign error;
  corr with steer angle is +0.60.

## Results

Cross-track offset from lane center at the vehicle, meters, "left" +:

| segment | | **dragonpilot** | **our ADAS** | manual driver |
|---|---|---|---|---|
| straight | signed | −0.05 | −0.00 | −0.09 |
| | abs / p90 | **0.08** / 0.23 | 0.12 / — | 0.16 |
| left arc | signed | −0.02 | **+0.51** | +0.14 |
| | abs / p90 | **0.07** / 0.41 | 0.51 | 0.17 |
| right arc | signed | −0.20 | **−0.71** | −0.14 |
| | abs / p90 | **0.20** / 0.66 | 0.71 | 0.15 |

On arcs dragonpilot is 3.5–7× smoother than us and smoother than the human. Their arcs here at 13–17 m/s, ours at
13 — speed does not explain the gap.

## Cause 1: Their Tracking Error Is Near Zero

| | dragonpilot | our ADAS |
|---|---|---|
| straight | **+0.00** | +0.07 |
| left arc | **+0.03** | +0.35 |
| right arc | **−0.08** | −0.23 |

Their controller sits on its reference; ours is ten times farther. This is the fast-loop
difference: they publish curvature trajectory, interpolate it, and run angle PID at **100 Hz**
(`lateralControlState = pidState` in all frames, `actuators.steer` ±1.0), while we compute one number
per vision frame (median 80 ms, p99 161 ms) and hold it for 8 panda ticks unchanged.

## Cause 2: Lane Blending Removes Plan Offset Entirely

| | model plan | reference |
|---|---|---|
| straight | +0.02 | −0.06 |
| left arc | **+0.12** | **−0.06** |
| right arc | **−0.25** | **−0.08** |

Their plan also cuts the corner, but reference stays **constant and equals their `CAMERA_OFFSET`** (−0.06 for
comma-two). Lane paint in reference 92–100 % of arc frames.

Two conclusions for our debates:

* **`blend 1.0` is justified**, and I wrongly rejected it: upstream relies on paint when visible;
* **curvature plan correction is not needed in principle**. Upstream does not correct plan — they do not
  use it when lanes are visible. Our attempt to fit `50.2·κ` (and its failure between runs)
  treated a problem upstream does not have.

## Cause 3, Unexpected: Their Lanes Are 5–10× More Confident

| host-line σ (median) | dragonpilot | our ADAS |
|---|---|---|
| straight | **0.04 / 0.05** | 0.19 |
| left arc | **0.05 / 0.06** | 0.60 |
| right arc | **0.13 / 0.09** | 0.93 |

This is the root. At σ ≈ 0.05 blending stays at unity and lanes are reliable reference.
At our σ = 0.6–0.9 blending was cut by 0.8 threshold; raising to 1.5 treated the symptom: lines
are genuinely less confident; we merely forced their use.

Same model generation (0.8.x, `PLAN_MHP_N = 5`, 33 points), so it is not the model but the input.

**Clarified 2026-08-04, and this was not "approximate calibration".** Verified by measurement:

* warp matches theirs line-for-line (`MED_FL 910`, `MED_CY 47.6`, same formula), tensor layout
  and no normalization — same;
* FOV of our camera, **measured with chessboard**, is 65.6° vs 65.2° on EON camera
  the model was trained on. Geometry is what the model expects; previous 951 and flowpilot 930 assumption were too wide;
* board-found principal-point shift (1.04° yaw, 0.97° pitch) **is already compensated by
  online calibration**: for pinhole model principal-point shift equals rotation. Fixing cx/cy without
  calibration reset **doubles** σ (0.25 → 0.55 on 90 frames, `bag_intrinsics_ab.py`), so only focal length went into config;
* flowpilot input pipeline is identical to ours, including guessed 930/640/360 — on a phone they would have the same σ.

Remaining gap is **hardware**: comma-two has automotive camera with dedicated lens and near-raw
pipeline; we have phone camera with ISP (noise reduction, sharpening, tone curve) that we do not disable —
same as flowpilot, same defaults from `TEMPLATE_RECORD` (verified 2026-08-04;
tone curve in their commented CameraX branch). What they have and we lack — **AE metering region on the road**; we meter over full frame including
sky. Work order — `PLAN_TO_COMMA2.md`, steps 5a and 5b. Do not expect 0.05 level on a phone:
model is out of domain.

Spread of yaw estimate between our runs (+1.12° vs +0.24°) comes from the same principal-point shift
absorbed by calibration — not calibration instability as such.

## What to Do in ADAS

Work order, effort, and verification — in **`PLAN_TO_COMMA2.md`**. Here only what follows directly from
this comparison:

1. **Inner loop at 100 Hz.** Initially I thought we need to publish curvature trajectory and
   interpolate it. Turned out otherwise: their `get_lag_adjusted_curvature` on the fast tick
   recomputes from **the same** published trajectory, and between planner updates their
   `desiredCurvature` changes only from fresh speed. So publishing trajectory is not needed —
   difference is that **angle PID** runs at 100 Hz vs our 20. Done: CAN receive timer 50 → 10 ms (steer angle on MQB already 100 Hz).
2. **`path_lane_blend_scale` → 1.0** when lanes visible: their reference equals lane center. But
   **after** item 1: at real command rate difference between 0.3, 0.6, and 1.0 almost vanished.
3. **Vehicle parameters from their learning** (`liveParameters` on this same car): `steer_ratio` 16.27,
   `stiffnessFactor` 1.319 vs our 15.7 and 0.64. Our `slipFactor` at multiplier 1.0 gives exactly their
   stiffness, so quantities are directly comparable — and we over-command at 22 m/s by 1.48×
   vs their 1.23.
4. **Road grade estimate** — we have none; from their logs on this route grade median 0.78°
   (p10 2.55°), giving constant curvature the controller must counter.
5. **Do not do**: curvature plan correction (not reproducible between runs) and centering term
   (`center_force` disabled: at real rate does nothing and eats stability margin).

## Pipeline Stage Check: What Matches 1:1 and What Diverges

Verified by reading dragonpilot/flowpilot sources against ours, not from memory.
Numbers and paths given for re-check.

### Matches

| stage | evidence |
|---|---|
| input resolution | `flowpilot .../transformations/Camera.java`: `frameSize = {1280, 720}` — same as ours |
| warp | `K · view_from_device · R(rpy) · inv(medmodel_K · view_from_device)`; target `medmodel` FL 910, CY 47.6, input 512×256. Ours `ModelCalibWarp.java`: `MED_FL = 910.0f`, `MED_CY = 47.6f`, same 512×256 |
| tensor layout | 6 planes per frame: four Y subsamples (2×2) + U + V (`yuv_warp.cpp`, `out6[0..5]`), two frames stacked `[prev, cur]` → `input_imgs[1,12,128,256]` |
| **input normalization** | **none for us or upstream** — supercombo 0.8.x eats raw YUV 0..255, scaling in first network layers |
| other inputs | `desire[8]`, `traffic_convention[2]`, `initial_state[512]` with recurrent state feedback (`SupercomboOnnxRunner.java`) |
| output layout | 6472, `PLAN_MHP_N = 5`, 33 points, 4 lines; σ = exp(second 264 of block) |
| lane blending | their formula: `d_prob = l_prob + r_prob − l·r`, path `lll + w/2` and `rll − w/2`, width clamped |
| lateral MPC | our `fp` = dragonpilot `lat_mpc` with `N = 16` (Tf = 2.5 s), grid `T_IDXS[i] = 10·(i/32)²`, equal weights on all nodes (`for i in range(N): cost_set(i,'W',W)`) |
| delay compensation | our `LateralMpc::lagAdjustedCurvature` — line port of `get_lag_adjusted_curvature`: `2·avg_κ − κ₀`, ψ interpolated at delay |
| vehicle model | our `slipFactor()` at `tire_stiffness_factor = 1.0` gives **exactly** their VW port stiffness: 184691 / 238876 N/rad. Our `tire_stiffness_factor` and their `stiffnessFactor` are the same quantity |
| angle PID | their VW port coeffs: kp 0.6, ki 0.2, kf 0.00006 — match ours |
| control point | `rotation_radius = 0` for both: rear axle controlled, though path is from camera |
| panda safety | `STEER_MAX 300`, `STEER_DELTA_UP 4`, `DOWN 10`, `DRIVER_ALLOWANCE 80` — match, covered by tests |
| HCA TX | 100 Hz for both |

### Diverges

| item | theirs | ours | note |
|---|---|---|---|
| lane weight in reference | ≈1.0 (`get_stock_path`), up to 0.8 in `get_nlp_path` | 0.6 | no multiplier in theirs, our addition |
| planner rate | 20 Hz | 12.5 Hz (median 80 ms, p99 161) | phone inference limit, not setting |
| inner loop (angle PID) | 100 Hz (`controlsState`) | was 20 Hz, now 100 (CAN receive 50 → 10 ms) | not verified on vehicle |
| σ threshold | `interp(σ, [0.15, 0.30])` per line scalar | `interp(σ, [0.3, 1.5])` on median of 33 points | **different quantities**, numbers not comparable |
| actuator delay | `steerActuatorDelay 0.10` + baked `+0.2` = 0.30 | `fp_steer_delay_s = 0.35` | |
| `steer_ratio`, stiffness, steer bias | learned by `paramsd` | constants in config | see `PLAN_TO_COMMA2.md` |

### Missing Entirely on Our Side

* **`paramsd`** — learns vehicle parameters (steer ratio, stiffness, steer bias) and **road grade
  estimate**; latter is the only found cause of constant offset not explained otherwise;
* **grade compensation** in desired curvature;
* **lane change** (`DesireHelper`): turn signal only silences LDW for us;
* **torque controller** (`latcontrol_torque`) — but theirs does not work on this car either:
  `liveTorqueParameters` filtered zero, `lateralControlState = pidState` in all frames;
* longitudinal control: plan exists, execution only via cruise buttons.

### Missing on Their Side

Our `mpc` (spatial VisionPilot MPC) and `pp` as alternative controllers, and our
FCW / AEB / LDW implementation — openpilot warnings are structured differently.

### Conclusion

**Perception pipeline and control math match** — to the point that our tire stiffness formula at multiplier 1.0 reproduces their VW values. Arc gap comes not from architecture but from three
numbers (`path_lane_blend_scale`, `tire_stiffness_factor`, `steer_ratio`), two missing mechanisms
(parameter learning and grade estimate), and inner-loop rate that until 2026-08-03 was five
times slower than theirs.
