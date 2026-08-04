# Tight arc 17:46–18:16 (run 2026_08_01_01_14_22)

Segment marked by driver as "controller could not handle it". HCA on 1070–1093 s
(17:50–18:13), v ≈ 12.3 m/s, S-curve link radius ~41 m.

Plot: `docs/mpc_img/57_run0801_curve.png`

## What happened

| metric (window 1070–1093 s) | value |
|---|---|
| \|yaw\| max | 17.2 °/s → κ 0.024 1/m → **R ≈ 41 m**, lateral accel 3.7 m/s² |
| HCA torque | median **300 cNm**, saturated **52 % frames** |
| lane position | median **−0.55 m**, max −0.94 m (outside arc) |
| angle tracking error (command−rack) | median 1.7°, p95 7.6°, max 13.9° |
| driver on wheel | 21 % frames (intervened second half) |

## Two independent causes, both confirmed by data

### 1. Controller systematically under-steered (fixed)

Per second on first half of arc:

| t, s | κ required | κ actual | SWA command | SWA actual |
|---|---|---|---|---|
| 1076 | −0.0110 | −0.0087 | −26.5° | −25.6° |
| 1077 | −0.0115 | −0.0097 | −27.6° | −26.7° |
| 1078 | −0.0112 | −0.0095 | −26.8° | −26.4° |

Rack tracked command closely (~1° diff), but **curvature was 16 % below
required**. Exactly measured car shortfall: regression over whole run gives κ_fact/κ_kinem
= 0.85 at 12–16 m/s. `fp` computed angle kinematically (`atan(L·κ)`), i.e. asked for
less than needed, and tried to make up via vision — with delay, so car
drifted outward and error accumulated.

**Done:** κ→angle via measured vehicle model
(`utils/vehicle_model.h`, `tire_stiffness_factor = 0.64`). On this arc command would be
−32.3° instead of −27.6° — exactly what required κ needs.

Closed-loop replay window on measured model: centering error 0.64 → **0.55 m** median.

### 2. Power steering hit limit (not fixable by tuning)

52 % frames torque at 300 cNm — MQB HCA ceiling (openpilot same number).
At 1080 s command was −22.2°, rack gave only −10.4° with saturated torque:
actuator physically could not hold angle.

Required 3.7 m/s² lateral accel. For comparison, openpilot limits comfortable
lateral accel around 2.5–3.0 m/s² and on such arcs routinely asks driver to help.

**Conclusion:** on R 41 m arc at 12 m/s assistant runs at hardware limit. Tuning cannot
fix this; need:

* show driver limit reached (saturation visible only in log now);
* optionally — reduce speed before such arcs via `long_plan`.

## Recommendations

1. **Done:** vehicle model in κ→angle conversion.
2. **Proposed:** flag `steer_limited` in `control/lane_keep_debug` + UI indication when
   torque saturated longer than ~0.5 s. Same value as condition for
   "take the wheel" warning.
3. **Proposed:** lateral speed limit in `long_plan` from curvature ahead
   (a_lat ≤ 2.5 m/s²) — then car approaches arc at speed HCA can hold.
4. Thresholds `mpc_max_steer_deg` / `steer_slew_limit_deg` are in **wheel** degrees (25° and 8°),
   i.e. 392° and 126° steering — they never trigger. On this arc peak was
   3.7° wheels. See `PIPELINE_AUDIT_0801.md`, "dead limiters" section.
