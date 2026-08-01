# Arc Offset: Car Cuts Inside the Turn (Run 2026_08_02_22_02_38)

Driver: "when I drove a left arc, the car hugged the left lane line; it should be in the center."
Verified — confirmed, but offset is not "left" but **inside the turn**: left on left arc,
right on right arc. On straight the car is centered.

Run: 31.6 min, under control 28.4 min, controller `fp`, median 10.3 m/s, max 25.3.
`cam_y_left_m = 0` (camera on centerline), `path_camera_offset_m = 0.08`, `path_lane_blend_scale = 0.3`.
Old APK: σ-weighted blending not yet in recording (verified, see below).

Plots: `docs/mpc_img/0802_offset_arcs.png`, `docs/mpc_img/0802_offset_decomp.png`.

## How It Was Measured (and Why the First Attempt Was Wrong)

Offset is computed **at the vehicle**: lane lines are fit with quadratic over
x ∈ [0, 30] m, value taken at x = 0. Averaging lane center "8–25 m ahead" is wrong —
on an arc it is itself shifted toward the turn by ½κd², up to 1.9 m, and the metric shows the wrong sign.
First measurement was spoiled by exactly this.

Lane frame is right-positive (`topic_convert.cpp`: "Device Y right-positive"), camera
shifted left by `cam_y_left` (`lane_keep_service.cpp:449`: `p.y() -= cam_y_left`), hence

    offset_left(0) = y_right_positive(0) − cam_y_left

Signs and reference reproduction verified independently:

| check | result |
|---|---|
| arc sign | corr(κ lane, chassis yaw) = **+0.875**, corr(κ, SWA) = +0.857 |
| reference reproduced | computed −CTE vs recorded `mpc_cte_m`: median diff **+0.030 m**, p90 0.044 m, corr **+0.986** |

3 cm gap is fit window difference (my 0–30 m vs 1–12 m in C++). It also limits
metric sensitivity to window: an order of magnitude below measured effect.

**Important:** curvature ratios (κ_fact/κ_lane) on these windows are not comparable — they differ by
the same 15 % because on a clothoid fit over 0–30 m gives larger κ than over 1–12 m. Understeer
model was measured separately, from SWA and yaw, without polylines, and that is a **different problem**: on
steady arcs measured transfer κ_fact/κ_kin is 0.97 at 6–9 m/s (model expects 0.96),
0.80 at 12–15 (0.87), and 0.54 at 21–26 (0.69), i.e. at speed command is 14–29 % low and pulls car **outside**. Here it goes inside, and stronger compensation only hurts
(see replay below, tsf 0.50).

## Measurements

Offset decomposition identity (all meters, "left" +):

    offset from lane center = (−CTE) + (reference offset from lane center)
                              ^tracking error  ^setpoint offset

Frames: HCA actually steering, driver hands off, v > 5 m/s, both lines prob > 0.3,
width 2.6–4.2 m.

| segment | n | total | tracking error | setpoint offset | torque saturated |
|---|---|---|---|---|---|
| straight \|κ\|<0.002 | 6088 | **−0.00** | +0.07 | −0.10 | 19 % |
| **left arc κ>0.004** | 310 | **+0.51** | +0.35 | +0.15 | 34 % |
| **right arc κ<−0.004** | 232 | **−0.71** | −0.23 | −0.42 | 41 % |

On straight the two parts cancel (+0.07 and −0.10) — hence straight looks perfect and
`path_camera_offset_m` need not change. On arcs both parts add in the same direction.

**"HCA was steering" flag — `controls_allowed` from `panda/health`, not `steer_output_enabled`.**
Latter is 1 in **all** frames in this run, so it filters nothing; former true in 39 % of frames. First table version used `steer_output_enabled`; conclusions unchanged (straight −0.02 → −0.00, left +0.49 → +0.51, right −0.68 → −0.71 — "hands off" filter already removed almost everything: 310 of 319 left-arc frames under HCA), but mask must be correct or another run will lie.

## Bar: How the Human Drives on the Same Arcs

Frames from same run where HCA did **not** steer, i.e. human at wheel:

| segment | under HCA | driver | n driver |
|---|---|---|---|
| straight | −0.00 (abs 0.12) | −0.09 (abs 0.16) | 3216 |
| left arc | +0.51 | **+0.14** (abs 0.17) | 39 |
| right arc | −0.71 | **−0.14** (abs 0.15) | 69 |

Three conclusions:

* **human also cuts inside on arcs**, symmetrically ±0.14 m — independent confirmation sign is not metric artifact;
* **but magnitude is 0.15–0.17 m on straight and arcs**, while controller on arcs is
  0.5–0.7. Correct target is **not zero but ≈0.15 m**;
* **on straight controller is already smoother than human** (0.12 vs 0.16).

Caveat: driver has only 39 and 69 arc frames, one or two arcs. Confirm bar on other
runs before treating as reference.

Worst episodes: left arc R≈200 m — 22 s with offset up to +0.68 m; right arc R≈157 m — 17 s,
median −0.75 m, max −0.95 m at lane width 3.9 m.

## Reproduced on Another Run

Run 2026_08_01_23_36_40 — different day, camera shifted 0.10 m left (`cam_y_left = 0.1`),
3227 straight frames and 195 left-arc frames (no right arcs in run):

| segment | total | tracking | setpoint | model plan |
|---|---|---|---|---|
| straight | −0.06 | +0.01 | −0.05 | +0.04 |
| left arc | **+0.42** | +0.19 | +0.18 | **+0.37** |

Reference reproduction on this run matches recorded CTE to 0.001 m (p90 0.018, corr 0.990).
Plan offset inside arc is most stable part: +0.33 m on one run, +0.37 m on another.

## Where Setpoint Offset Comes From: Model Plan Cuts the Corner

Reference = `d · lane_center + (1−d) · model_plan + right_shift`, where `d = (pl+pr−pl·pr)·0.3 ≈ 0.3`.

| segment | ref vs lane | model plan vs lane | lane estimate vs lane | shift |
|---|---|---|---|---|
| straight | −0.10 | −0.02 | −0.00 | −0.08 |
| left arc | +0.15 | **+0.33** | −0.00 | −0.08 |
| right arc | −0.42 | **−0.48** | −0.00 | −0.08 |

Two-line lane center estimate is unbiased (−0.00 everywhere), no skew from `prob` or width
clamp. Shifted part is **supercombo plan**: on arcs it goes 0.33–0.48 m inside
turn, on straight it is centered. At `blend = 0.3` 70 % of this offset enters reference.

## Where Tracking Error Comes From

Car sits 0.35 m inside **its own reference** (left arc). Meanwhile:

* own steer-angle clamp never triggers in any frame;
* HCA torque saturated at ±300 cNm in **34–41 %** of arc frames (vs 19 % on straight);
* of saturated frames where car is already ≥0.3 m inside, torque points **into turn in 100 %**
  of cases — controller does not try to return to center, it holds a tighter line;
* desired SWA tracked accurately (12.3° → 12.2° left, −16.4° → −16.1° right), so gap is not rack.

Mechanism is structural, and **not horizon length** (I first wrote that — wrong,
analysis below). Two causes:

* **near nodes are uncontrollable.** Each frame state resets: `x0 = [0, 0, 0, desired_ψ̇]`,
  so vehicle by construction sits at origin of its own frame, and offset sits in
  reference `y_ref`. Residual at zero node equals `y_ref[0]` and does not depend on control; nodes
  1–5 sit 0.1–3.2 m ahead (13 m/s), and lateral motion in that time is negligible. Their cost contribution is constant, and no weight redistribution removes it;
* **no integral term.** Any constant disturbance — torque ceiling, cross slope,
  inaccurate understeer model — gives constant error, like pure P regulator.

When torque also hits ceiling, no near cross-track feedback remains at all.

## How flowpilot and dragonpilot Solve This

Verified against flowpilot and dragonpilot sources (local checkouts).

**Horizon.** 10 s is model plan horizon (33 points `T_IDXS[i] = 10·(i/32)²`). Lateral MPC
takes first `N+1` points from it, and upstream diverged: flowpilot
(`lateral_mpc_lib/lat_mpc.py`) `N = 32`, i.e. `Tf = T_IDXS[32] = 10.0 s`, dragonpilot
`N = 16`, i.e. `Tf = 2.5 s`. We took dragonpilot.

But near cross-track is unaffected, because grid is quadratic, so **11 of our 17
nodes lie in the first second** (0.0, 0.1, 0.5, 1.1, 2.0, 3.2, 4.6, 6.2, 8.1, 10.3, 12.7 m at
13 m/s) — same 11 as flowpilot. Extending to `N = 32` only adds far nodes
(2.5 → 10 s, i.e. 32 → 130 m), and weight in both upstream is equal on all nodes
(`for i in range(N): cost_set(i, 'W', W)`). Long horizon does not strengthen near zone, it
dilutes it. Our short horizon is rather advantageous for this task; change it for far-arc lookahead
and measure separately — we use GD instead of acados, and `N = 32` is roughly
4× cost.

**What flowpilot actually does differently — fixes reference, not MPC.** In
`lane_planner.py:get_nlp_path` there is separate term `center_force`:

    target_centering = rll_y[0] + lll_y[0]          # = 2 × vehicle offset from lane center
    target_centering *= 1.5                          # if not at road edge
    center_force = 0.4 · (3.4 / lane_width) · target_centering
    center_force = clamp(center_force, −0.8, +0.8)   # m
    if sign(center_force) == sign(vcurv[0]): center_force *= 0.7
    path_xyz[:, 1] += CAMERA_OFFSET + center_force

Proportional term on measured cross-track **at the vehicle** with effective coefficient
≈1.2 and clamp ±0.8 m, added to entire reference — exactly the near feedback
we lack. Plus curvature gate (comment in source: "less centering toward direction we already turn — avoid over-steering in existing turn"); `vcurv` sign
must be checked on bench when porting, Python has its own frame.

On our measured arcs this term would shift reference **+0.60 m** on left arc (offset +0.49,
width 3.31) and **−0.71 m** on right (−0.68, 3.91) — same order as the error itself.

**Upstream does not weaken lane blending.** In `get_stock_path`
`d_prob = l_prob + r_prob − l_prob·r_prob`, i.e. with two confident lines ≈1.0. In
`get_nlp_path` lane path share `final_ultimate_path_mix` reaches **0.8** with comment
"always have at least 20 % model path in there". Our 0.3 is our own addition; upstream has
no such multiplier.

**Upstream σ damping is softer and per best line, not worst:**

| | flowpilot | ours |
|---|---|---|
| σ function | `interp(σ, [0, 0.3, 0.9], [1.0, 0.4, 0.0])` | `interp(σ, [0.2, 0.8], [1.0, 0.0])` |
| at σ = 0.6 | ≈0.25 remains | 0.33 remains |
| at σ = 0.9 | 0.0 | 0.0 (already at 0.8) |
| both lines required | no: `lane_trust = clamp(1.2·max(l_vis, r_vis)^0.5, 0, 1)` | yes: `anchored = pl > 0 and pr > 0` |
| road edges | used as extra references (`le_y`, `re_y`) | not used at all |

Hence our arc failure: one bad line is enough for us to disable blending entirely,
while flowpilot keeps reference with one good line.

`CAMERA_OFFSET` in flowpilot is **0.08** — matches our `path_camera_offset_m`.

## Separate Finding: σ-Weighted Blending Kills Lanes Exactly on Arcs

Blending weight from model lane σ (`lane_std_good_m = 0.2`, `lane_std_bad_m = 0.8`)
added 2026-08-02 and was not in the recording APK. Verified against recorded CTE:

| reference variant | straight | left arc | right arc |
|---|---|---|---|
| prob threshold (as in APK) | +0.030 / p90 0.042 | +0.031 / 0.119 | **+0.032 / 0.079** |
| σ weight (new code) | +0.030 / 0.044 | +0.015 / 0.078 | **+0.113 / 0.229** |

On right arc σ variant diverges from recorded CTE 3.5× more — run used prob threshold, and `d = 0.30` held everywhere.

What new code would do if deployed as-is (lane σ, worst of two lines):

| segment | σ median | p75 | share σ > 0.8 | blending enabled |
|---|---|---|---|---|
| straight | 0.19 | 0.38 | 9 % | 91 % frames |
| left arc | 0.60 | 0.90 | 26 % | 74 % |
| right arc | 0.93 | 1.17 | 67 % | **33 %** |

Threshold 0.8 disables lane reference exactly where it is the only defense against
corner-cutting plan: on right arc setpoint offset grows from −0.42 to −0.54 m (recomputed
with same script and `--weight-by-std`), and blending stays alive in only a third of frames.

## Verified in Closed Loop

Same arc windows, real C++ controller vs `core.vehicle_model.LateralPlant`, vision delay
69 ms, pose resync to recorded pose every 20 s. Metric — same cross-track offset from lane center at vehicle.

Columns: "offset" — median signed offset from lane center (left +), "|·|" — absolute,
"steer" — median |ΔSWA| between frames, i.e. steering work.

| config | left arc R≈200 | | | right arc R≈157 | | |
|---|---|---|---|---|---|---|
| | offset | abs p95 | steer | offset | abs p95 | steer |
| as in run (blend 0.3, delay 0.35, tsf 0.64, shift 0.08) | +0.15 | 0.39 | 0.14 | −0.34 | 1.06 | 0.16 |
| shift 0.05 | +0.17 | 0.43 | 0.13 | −0.31 | 1.03 | 0.16 |
| **blend 0.6** | **+0.11** | 0.37 | 0.14 | **−0.22** | 0.92 | 0.15 |
| blend 1.0 | +0.00 | 0.43 | 0.14 | −0.10 | 0.82 | 0.15 |
| delay 0.23 | +0.14 | 0.45 | 0.11 | −0.24 | 1.15 | 0.14 |
| delay 0.23 + blend 0.6 | +0.06 | 0.47 | 0.12 | −0.13 | 1.01 | 0.13 |
| steer rate penalty 150 | +0.16 | 0.40 | 0.14 | −0.36 | 1.05 | 0.16 |
| penalty 800 (stock acados) | +0.14 | 0.39 | 0.13 | −0.32 | 1.07 | 0.15 |
| **tsf 0.50** | **+0.20** | 0.45 | 0.14 | **−0.42** | 1.10 | 0.16 |
| delay 0.23 + blend 0.6 + tsf 0.50 | +0.10 | 0.41 | 0.12 | −0.18 | 1.04 | 0.14 |
| centering 0.7 (blend 0.3) | +0.10 | 0.44 | 0.18 | −0.18 | 0.86 | 0.18 |
| **centering 0.4 + blend 0.6** | **+0.06** | **0.41** | **0.16** | **−0.16** | **0.83** | **0.16** |
| centering 0.55 + blend 0.6 | +0.05 | 0.44 | 0.18 | −0.14 | 0.80 | 0.18 |
| centering 0.7 + blend 0.6 | +0.05 | 0.44 | 0.19 | −0.13 | 0.78 | 0.19 |
| centering 1.2 + blend 0.6 | +0.00 | **0.55** | **0.35** | −0.11 | 0.75 | 0.22 |

Absolute offset (median) on same replays: left arc 0.21 without centering → **0.16** at
0.4 and same 0.16 at 0.55 and 0.7; right 0.34 → **0.16** at 0.4, 0.14 at 0.55 and 0.7, 0.25 at 1.2.

Replay conclusions, some against my own hypotheses:

* **lane weight works and monotonically on both arcs**: 0.3 → 0.6 → 1.0 gives +0.15 → +0.11 → +0.00
  on left and −0.36 → −0.27 → −0.13 on right, and p95 on right drops from 1.06 to 0.84;
* **`tire_stiffness_factor` 0.50 makes both worse** (+0.15 → +0.20 and −0.36 → −0.44). Stronger
  understeer compensation fixes outward drift on fast arcs; here car goes inside.
  Two different problems, do not conflate;
* **delay `fp_steer_delay_s` 0.35 → 0.23 gives at best 0.01–0.04 m median and worsens p95** on
  right arc (1.06 → 1.15). Not a useful lever; hypothesis "extra 0.2 s lookahead enters arc
  earlier" not confirmed;
* **steer rate penalty not a lever**: 150 slightly worse, 800 slightly better, both ≤ 0.03 m;
* **centering term beats lane weight**: 0.7 at old blend 0.3 gives −0.18 vs −0.22 for
  blend 0.6. With blend 0.6 it cuts abs offset on worst arc from 0.34 to 0.16 m
  and p95 from 1.06 to 0.83;
* **gain saturates at 0.4, cost grows monotonically.** 0.4 → 0.55 → 0.7 on right arc gives
  0.16 → 0.14 → 0.14 m abs, i.e. 2 cm, while steering work 0.16 → 0.18 → 0.19. On left arc
  all three give same 0.16 m, but 0.4 has best p95 (0.41 vs 0.44) and least steering work
  (0.16 vs 0.19). At 0.4 steering work does not differ from baseline — offset removed
  for free, hence **0.4** chosen, not upstream 0.72…1.2;
* **stability margin is small.** Top of upstream range (1.2) on right arc buys 2 cm, on left **falls apart**: abs offset grows from 0.16 to 0.25 m, p95 from 0.41 to
  0.55, steering work from 0.16 to 0.35 — 2.5× baseline. That is oscillation. If on road
  steering chatter appears, move coefficient **down**, not up.

Absolute replay numbers do not equal road (+0.15 vs measured +0.51 on left arc): plan
in replay is rebuilt from recorded lines, chassis model is July version. Replay ranks
levers, it does not predict road values.

**Resync fixed.** Previously every 20 s pose was replaced with recorded **entirely**, including
lateral position — controller accumulated offset zeroed and placed exactly where
driver was, and median then included transient. That meant offset was partly measured relative to driver's line. Now only along-road drift and absolute heading drift reset;
lateral offset and heading error accumulated by controller are kept:

    e_y   = lateral offset of simulated vehicle from recorded path
    e_psi = its heading error
    pose := recorded(t) + e_y·normal(heading(t)),   heading := heading(t) + e_psi

Old behavior remains under `--resync-full`. Lever ranking unchanged by fix (on
right arc baseline 0.34 → 0.28, blend 0.6 0.22 → 0.17, centering 0.4 0.16 → 0.12), conclusions
hold, but absolute numbers slightly smoother — opposite of what I expected.

Two more things replay cannot show in principle, worth remembering: lanes in it are **model
estimates**, not ground truth, so systematic perception error is invisible (controller and metric see
same paint); and replay does not re-capture — when simulated vehicle diverges from recorded path,
far polyline points come from a viewpoint where they were not observed.

First table version had two mistakes in config set: baseline row ran delay 0.23 instead of 0.35 (in `build_app` that is default, and I did not set explicitly),
and path shift applied twice — `path_bundle_from_bag_lanes` already adds 0.08. Ranking unchanged,
absolute numbers shifted 0.08 m. Both fixed and commented in `bag_config_sweep.py`.

## Done

All below is in code and `assets/config.json`, APK rebuilt, 81 tests pass. Not verified on vehicle —
next step.

1. **`path_lane_blend_scale` 0.3 → 0.6.** Share of corner-cutting plan in reference drops from 70 % to 40 %.
   Straight unaffected: plan not offset there. `blend 1.0` remains backup but gives all lateral
   planning to lane paint.
2. **`lane_std_good_m` 0.2 → 0.3, `lane_std_bad_m` 0.8 → 1.5.** Without this item 1 fails on arcs:
   43 % of arc frames have σ above 0.8 and blending was fully disabled. Mechanism "do not trust line model is unsure about" is correct, but absolute threshold 0.8 was tuned on straights where σ is
   four times lower, and lane center stays unbiased up to σ ~1.
3. **`lane_width_max_m` 4.2 → 4.6, width filtered** (RC ≈ 10 s, like upstream). Ceiling 4.2
   rejected 31 % of frames on 3.9 m wide arc though road unchanged — estimate jumped.
4. **Ported `center_force`, coefficient 0.4.** Proportional term on cross-track at vehicle,
   added to entire reference, clamp ±0.8 m, ×0.7 attenuation toward current turn. Value
   chosen by replay (see conclusions above), not taken from upstream. Cross-track at vehicle read by
   quadratic lane fit at zero — same metric as run. Computed from vehicle centerline, not camera: config has `cam_y_left_m` from
   `calibration.camera.position_m.y_left`.
5. **`path_camera_offset_m` unchanged** (0.05): straight equilibrium −0.02 m held by it.
6. **`fp_steer_delay_s`, `fp_steering_rate_weight`, `tire_stiffness_factor` unchanged** — replay
   showed they either do not affect this problem or hurt.
7. **Horizon `N = 16` (2.5 s) unchanged.** flowpilot keeps `N = 32` (10 s), but near zone is the same — 11 of 17 nodes in first second; lengthening only adds far nodes and at equal weights dilutes near zone, and we use GD instead of acados with `N = 32` ~4×
   costlier.

## What to Expect on Vehicle

Closed-loop numbers are not forecasts: its baseline is already 2–3× smoother than road
(+0.15 vs measured +0.49 on left arc), so "0.16 m" in table is simulation, not
vehicle. Forecast computed differently: setpoint offset recomputed from recorded perception
with new config (measurement, not guess), tracking error taken measured, centering term — P-loop, so equilibrium per frame gives `o = (tracking + setpoint) / (1 + k)`, where
`k = 0.4·3.4/width` (0.41 in narrow lane, 0.35 in wide).

| segment | now | setpoint becomes | config only | + centering 0.4 |
|---|---|---|---|---|
| straight | −0.02 | −0.06 | +0.01 | **+0.01** |
| left arc | +0.49 | +0.11 | +0.45 | **+0.32** |
| right arc | −0.68 | −0.30 | −0.54 | **−0.40** |

Three conclusions from this table:

* **straight stays centered** — equilibrium was and remains within a couple centimeters;
* **on left arc config fixes alone do almost nothing** (+0.49 → +0.45): its offset is
  70 % tracking error, not setpoint. Centering term does the work;
* **car will not be "lane center".** Offset roughly halved, but 0.3–0.4 m remains.
  Centering divides remainder by 1.41, does not zero it, and zero is unreachable at this coefficient —
  above 0.55 oscillation starts.

Forecast is pessimistic in one direction: tracking error is measured and should
shrink — HCA torque sat 34–41 % of arc frames because too tight a line was requested,
and with less offset setpoint request drops and authority returns. Closed loop that accounts for this gave 2.1× improvement on right arc vs 1.7× from this algebra.

## What Is Needed to Reach Human 0.14

From same decomposition: `o = (tracking + setpoint) / (1 + k)`, require |o| ≤ 0.14.

| arc | setpoint variant | setpoint | forecast | tracking needed | how much better |
|---|---|---|---|---|---|
| left (now |0.51|, tracking +0.35, k 0.41) | blend 0.6, shift 0.05 | +0.11 | +0.32 | ≤ 0.09 | 3.9× |
| | blend 1.0, shift 0.05 | −0.04 | +0.22 | ≤ 0.15 | 2.3× |
| | blend 1.0, shift 0 | +0.01 | +0.26 | ≤ 0.19 | 1.8× |
| right (now |0.71|, tracking −0.23, k 0.35) | blend 0.6, shift 0.05 | −0.31 | −0.42 | — | **unreachable** |
| | blend 1.0, shift 0.05 | −0.11 | −0.25 | ≤ 0.08 | 2.7× |
| | blend 1.0, shift 0 | −0.06 | −0.21 | ≤ 0.13 | 1.7× |

Right arc at `blend 0.6` unreachable at **any** tracking quality: setpoint offset alone
(−0.31) exceeds entire budget 0.14·(1+k) = 0.19. Setpoint offset must go almost to zero,
and tracking must improve 1.7–2.3×. P-term alone cannot do second part: it divides steady error
by (1+k) = 1.4, but 2× is needed.

### Setpoint Offset: Not blend 1.0 but Curvature Plan Correction

Initially I proposed `blend 1.0` — when both lines visible, drive lane center and ignore
model plan. Rejected, because:

* plan is **the only** thing reacting to unmarked road: no free corridor or
  parked-car detector in project;
* upstream goes the other way — openpilot moved from lane blending to pure plan
  (laneless) because lanes fail in hard cases (construction, snow, double markings side by side);
* plan offset is partly normal: driver on same arcs also goes ±0.14 m
  inside. Plan is not broken, it cuts like human, just four times stronger.

### Curvature Plan Correction: Verified on Two Runs and Rejected

At first plan offset looked like pure function of curvature and removable with one
coefficient. On run 22_02_38 (10 506 frames) fit gives `50.2·κ − 0.005 m`, and arc residual drops from +0.32 / −0.35 to **+0.04 / −0.02**. Tempting.

**Coefficient unstable even within one run:**

| slice | coefficient |
|---|---|
| whole run | 50.1·κ |
| v < 12 m/s | **20.2·κ** |
| v ≥ 12 m/s | **77.2·κ** |
| first / second half | 46.4 / 59.0 |

Fourfold spread by speed — "constant distance" model is wrong. Correct
parameterization is **time lookahead**: plan takes lane center at `v·T` ahead, i.e.
offset = ½κ(vT)². Parameter is stable vs speed:

| slice | T |
|---|---|
| v 5–9 / 9–13 / 13–17 / 17–26 m/s | 1.03 / 0.71 / 0.91 / 0.82 s |
| arcs only | 0.83 s |
| whole run | **0.84 s** (at 13 m/s that is 10.9 m) |

But **not reproducible between runs either**:

| run | T | plan: straight / left / right | arc residual after correction |
|---|---|---|---|
| 22_02_38 | 0.84 s | −0.02 / +0.32 / −0.35 | −0.00 / −0.06 |
| 01_23_36_40 | **0.56 s** | +0.04 / +0.37 / **−0.15** | **+0.23 / +0.14** |

On second run plan offset is **asymmetric** (+0.37 left, only −0.15 right), and no symmetric
curvature function closes both arcs: correction from first run leaves +0.23 m on left there, i.e. removes only 38 %.

Runs differ in camera calibration: yaw **+1.12°** vs **+0.24°** (pitch almost same). Calibration
enters input-image warp, network sees different image and outputs different plan — cannot tie difference to geometry with one transform.

**Conclusion: coefficient does not transfer between setups, cannot bake it in** — on another calibration it adds its own offset. Proposal withdrawn. Important consequence favoring two other levers:
lane blending **requires no tunable number** at all, and integral term removes
steady offset regardless of cause, including the part that changes run to run.

### Tracking Error: Integrator

Required 1.7–2.3× improvement not achievable with P-term. Direct path — **integral term** on
cross-track: removes steady error by construction, not by dividing it. Required reference shift computed:
**0.36 m on left arc and 0.28 m on right**, both within ±0.8 clamp — achievable on paper.

Risk stated plainly: integrator on perception quantity turns any calibration error into
real offset. Error in `cam_y_left` or roll of 5 cm — car drives 5 cm off center while log
shows "centered". Hence clamp ±0.25 m, time constant 5–10 s, freeze when no lanes,
reset on override and lane change, and calibration verified **first**. Upstream has nothing like this;
flowpilot lives with P-term.

Separately: **get torque out of saturation**. 34–41 % of arc frames steer at ±300 cNm ceiling, and while there,
controller fixes do not act at all. Part will self-resolve when reference stops demanding
too tight a line — check on next run. Ceiling itself is panda safety contract
(`STEER_MAX = 300`), not config key.

Verified and **does not** help: delay, steer rate penalty, `tire_stiffness_factor`, horizon
length. Details above.

Still open separately: **single line does not hold reference** (flowpilot needs only better of
two), and **road edges not used at all**. Correct order — edges first, then
relax two-line requirement; see `BACKLOG.md`.

## What to Record on Next Run

Run of this build describes itself: `control/lane_keep_debug` now carries `lane_offset_m`
(vehicle cross-track from lane center at its own position), `center_force_m`, `lane_width_m`,
`lane_anchored` and three keys — `p_lane_blend_scale`, `p_camera_offset_m`, `p_center_force_gain`.
Before this analysis had to use Python reference reproduction, i.e. second
instance of `laneLinesToPath`, and they had diverged (see `core/path_fusion.py` in same fix).
`bag_arc_offset.py` now takes recorded quantities; recomputation prints on separate line as
cross-check.

What the drive needs:

* HCA enabled on **urban arcs radius 100–250 m** both directions, several seconds each
  — where measurement was taken; plus straight section to verify equilibrium did not drift;
* hands off wheel during arc: frames with `steering_pressed` dropped from analysis;
* `record_camera_images` can stay off — offset analysis needs no images, and run
  drops from 1.6 GB to tens of MB.

Watch first: **steering chatter on arcs**. Centering is P-loop with small margin (see replay conclusions). Numerically visible in median |ΔSWA|
between frames: was 0.14–0.16, expect same, alarm if 0.25 and above.

## How to Reproduce

Measurement from run — tables, episodes, both plots:

    python3 app/src/main/scripts/bag_arc_offset.py adas_logs/2026_08_02_22_02_38 \
        --cache /tmp/arc.npz --plots docs/mpc_img --prefix 0802

Script prints self-checks first: computed −CTE vs recorded `mpc_cte_m` and arc
sign. If first fails, reference reproduction does not match run and other numbers cannot be trusted. `--weight-by-std` shows what σ weighting would do; `--blend`/`--shift` — if run
recorded with different config.

Closed loop on arc windows:

    python3 app/src/main/scripts/bag_config_sweep.py adas_logs/2026_08_02_22_02_38 \
        --t0 662 --t1 686 --set arcs --cam-y-left 0     # right arc R≈157
    python3 app/src/main/scripts/bag_config_sweep.py adas_logs/2026_08_02_22_02_38 \
        --t0 1050 --t1 1078 --set arcs --cam-y-left 0   # left arc R≈200

`--cam-y-left` required and must match run (`lane_keep_debug.cam_y_left_m`): it goes
into controller and metric; error shifts all numbers by its value. Column
`offset` — median signed offset from lane center; `|center|med` — absolute.
