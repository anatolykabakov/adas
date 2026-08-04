# Real time step instead of DT_MDL=0.05 (2026-08-01)

## Symptom

Adapted `fp` planner commanded steering with **0.83 s** lag relative to the driver
(command recorded on phone — 0.94 s). In closed loop this means late corrections and yawing.

## Cause

In openpilot `plannerd` runs at fixed 20 Hz, so `DT_MDL = 0.05` there equals the
real decision period. In ADAS the planner is called on each `vision/lanes`, which on device is
**89 ms median** (p10 42, p90 170 — triple jitter). The port carried the constant over,
and it was used where the real step is needed:

| Location | What broke |
|---|---|
| `flowpilot/lateral_mpc.cpp` — `x0_[3] = interpAtTime(dt, r_sol)` | next solution initial condition advanced 50 ms instead of ~90–120 ms → each MPC frame started from lagged state; aperiodic link with time constant of several frames |
| `flowpilot/lateral_mpc.cpp` — jerk clip `κ₀ ± max_kappa_rate·dt` | κ change budget per frame was twice as tight as intended and hit exactly on transients |
| `lane_keep_service.cpp` — publish-slew guard and VisionPilot-MPC rate limit | same hardcoded 0.05 |
| `KappaRateFilter::update` | default dt 0.05 for dκ/ds |

Horizon `N=16` (2.5 s instead of stock 10 s) and gradient-descent solver barely relate to lag:
after step fix phase is zero at same `N`.

## What was done

* `LaneKeepService::updateFrameDt` — inter-frame period measured from `capture_ts_us`
  (else `timestamp_us`), clamp `[0.02, 0.5]`, EMA α=0.3; gap > 0.5 s or non-monotonic
  stamp → revert to `vehicle.vision_nominal_dt_s` (0.09) and reset estimate. Called in `step()`,
  so works for direct test/`pyadas` calls too.
* Measured step forwarded to `LateralMpc::update`, `StockLateralPlanner::update` (`fpa`),
  both slew/rate guards and `KappaRateFilter`. Signatures got `dt_s` default = old
  value so direct calls do not silently change behavior.
* `control/lane_keep_debug.frame_dt_ms` — measured step written to bag (on phone frequency
  is its own; without this field it cannot be checked).
* Offline harness (`bag_mpc_sim.py`, `model_ab_sim.py`) no longer feeds synthetic
  `us += 50_000`, but real bag time — otherwise runs would hide exactly this bug.
* Test `LaneKeepServiceMpc.FrameDtComesFromMessageTimestamps`: same maneuver at 20 Hz and 5 Hz
  must give step proportional to period.

## Result (`2026_07_31_19_02_24`, 753–860 s, shadow: driver steering)

| controller | \|err\|med | corr | HF | gain | lag |
|---|---|---|---|---|---|
| fp **before** | 1.05° | 0.890 | 0.05 | 0.77 | **0.83 s** |
| fp **after** (step + no κ-EMA) | **0.83°** | **0.946** | 0.08 | **0.91** | **0.00 s** |
| mpc (VisionPilot) | 0.55° | 0.953 | 0.12 | 0.84 | 0.24 s |
| fpa (stock + acados) | 0.95° | 0.913 | 0.18 | 0.83 | 0.00 s |
| driver | — | — | 0.10 | — | — |

`mpc` here blends 40 % measured yaw (`mpc_kappa_yaw_blend`), so in shadow mode
it partially repeats the driver's steering; without blending \|err\|med 0.79° and lag 0.59 s.

Plot: `docs/mpc_img/52_dtfix_1902.png`. Regression on `2026_07_31_10_33_17` (HCA was on there,
so "rack" = old `fp` command): new `fp` gives HF 0.08 vs 0.10 rack, no blow-ups.

## Other links with hardcoded rate (for fair comparison)

Step fix touched planners; for all controllers the pipeline still had two
constants, both fixed:

* **angle-PID integrated at 50 Hz**, while `updateTorqueFromAngle` is called on each
  `vehicle/chassis` — **20 Hz** on MQB. So `ki` was 2.5× low for all
  controllers at once. Now `LaneKeepService::updateChassisDt` measures chassis period and calls
  `LatControlPid::setRate`.
* **EMA on κ / CTE / epsi were per-frame**: with vision jitter 42–170 ms their physical
  time constant swung fourfold. `LaneKeepService::emaAlpha` recomputes α specified for
  `vision_nominal_dt_s` into α for current period (`α_eff = 1 − exp(−dt/τ)`), so
  smoothing no longer depends on how fast the network runs.

On window 753–860 s metrics barely changed (\|err\|med 0.83 → 0.82°) — period almost
constant there; point of fixes is same behavior at different inference rates.

## Angle divergence at 60–90 s (abs 813–843 s)

There `fp` and `fpa` hold +2…+5° left of driver. This is **not** the controller: frame breakdown gives

| quantity on segment | value |
|---|---|
| model plan in frame @20 m | −0.19 m (left) |
| after shift `cam_y_left = 0.10` | **−0.29 m** |
| plan offset from lane center @20 m | −0.19 m |
| vehicle position in lane @0 | +0.05 m (i.e. centered) |
| required κ = 2·y/x² | 0.00144 1/m → δ 0.22° → **SWA ≈ 3.4°** |

So command matches exactly what the model plan is ~0.3 m left of current path
line. Important: **open-loop** run — car does not respond, offset does not go away,
and command stays biased all 25 s. In closed loop it is chosen: `bag_mpc_sim.py
--mode closed` on same window gives \|cross-track\| med 0.14 m, p95 1.10 m, final +0.09 m.

Where plan offset itself comes from (whole run median −0.10 m, here −0.19 m): regression over 2932
frames with lead

```
plan_offset@20 = −0.62·(vehicle position in lane) + 0.08·(lead y) − 0.060    R² = 0.47
```

— plan mainly continues current vehicle line, not returning to lane center
(−0.62 coefficient on lane position), plus slight pull toward lead lateral position.
Free term −0.06 m — residual left shift, same order as stock `CAMERA_OFFSET`
(0.04–0.08 m right), which we replaced with `cam_y_left`.

Practical consequences:

* cannot evaluate controllers open-loop by "divergence from driver" on long straights —
  static path offset becomes static command error; judge by phase/shape
  (what `lag`/`corr` do) or `--mode closed`;
* `cam_y_left = 0.10` **verified on runs and confirmed** (see below) — constant component
  did not come from mount, but from the model itself; closed with stock `CAMERA_OFFSET`.

## What remains to resolve

* **κ-EMA removed from `fp`** (2026-08-01). It masked command jitter with old
  fixed `DT_MDL`; with measured step it cost 0.12 s phase and gave nothing.
  After removal on `19_02_24` [753–860]: \|err\|med 0.83°, corr 0.946, gain 0.91, **lag 0.00 s**,
  HF 0.08 vs 0.10 driver; on `10_33_17` HF 0.08 vs 0.10 rack — no jitter increase.
  Stock flowpilot has no such filter at all. For VisionPilot `mpc` filter kept: it is noisier
  (HF 0.06 with filter vs 0.12 without), `mpc_kappa_ema_alpha` now applies only to it.
* Internal EMA in `KappaRateFilter` (`RATE_ALPHA = 0.6` per frame) remains per-frame —
  only place smoothing still depends on rate; affects only `mpc`.
* Default controller: with fixed step `fp` beats `fpa` on this window
  (0.83° / 0.944 vs 0.95° / 0.913), so `lane_keep_controller = "fp"` remains reasonable.

Early tuning reports (`MPC_VS_FP_HIGHWAY`, `PARITY_0731_BAGS`, `FLOWPILOT_PARITY_0731`)
referred to state before this fix and were removed from the repo: their `fp` numbers no longer
reproduce. Current measurements — in `PIPELINE_AUDIT_0801.md` and `RUN_0801_*.md`.


## Calibration on y and stock CAMERA_OFFSET (2026-08-01)

Lateral camera offset estimated from run: for frames with both near lane markings
`prob > 0.9` and lane width 2.8–3.8 m fit line `c(x) = a + b·x` to lane midline
on 0–30 m. Intercept `a` — lane center position in camera frame at camera height,
slope `b` — residual yaw error (does not affect `a`).

| run | n | a, m | b, mrad |
|---|---|---|---|
| 10_33_17 | 4129 | +0.066 | +0.7 |
| 16_17_50 | 4412 | +0.168 | −1.0 |
| 16_43_59 | 1028 | −0.038 | −5.3 |
| 16_50_13 | 207 | +0.196 | −1.5 |
| 19_02_24 | 1245 | +0.122 | −3.9 |
| **combined** | **11021** | **+0.111** | |

`cam_y_left = 0.10` confirmed (1 cm difference ≈ 0.07° steering). **Caveat:** method assumes
"on average driver centers in lane" and does not separate mount offset
from driver habit; spread across runs ±0.07 m is exactly style and road contribution. Can only be separated with external reference (tape to longitudinal axis or frame from exactly centered
parking).

Breakdown showed constant left drift comes from **the model**, not mount:

| median over 10.5k frames | value |
|---|---|
| lane center in frame @0 | +0.110 m → vehicle centered at `cam_y = 0.10` |
| plan relative to lane center @20 m | **−0.099 m** (stable across runs) |
| plan in vehicle-center coords @20 m | −0.097 m |

Exactly what stock `lane_planner.CAMERA_OFFSET` is for (flowpilot 0.08 m right),
and what ADAS lacked. Added: `vehicle.path_camera_offset_m = 0.08`, applied in
`laneLinesToPath` to `polyline` and `plan_poly` (and mirrored in `core/path_fusion.py`, else offline
would not match phone). Test `TopicConvert.CameraOffsetShiftsPathRight`.

Effect on `19_02_24` [753–860]:

| controller | \|err\|med | p95 | corr | HF | gain | lag |
|---|---|---|---|---|---|---|
| fp before offset | 0.83° | 2.51 | 0.946 | 0.08 | 0.91 | 0.00 |
| **fp after** | **0.61°** | **2.03** | **0.951** | 0.08 | 0.91 | 0.00 |
| fpa after | 0.79° | 2.24 | 0.921 | 0.18 | 0.83 | 0.00 |
| mpc | 0.55° | 1.90 | 0.954 | 0.13 | 0.84 | 0.24 |

`mpc` unchanged: it has `mpc_cte_gain_base = 0` (FF-only seed), path lateral offset barely affects
its output. For vision-only comparison (mpc blend=0 gives 0.79°) `fp` is now
best: **0.61°**.
