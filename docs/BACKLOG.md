# Items Requiring Further Work

Compiled 2026-08-02, updated 2026-08-04. Sources: road runs (`RUN_0801_*`,
`RUN_0802_ARC_OFFSET.md`, `PIPELINE_AUDIT_0801.md`), simulator runs
(`SIM_CONTROLLER_TEST.md`), warning analysis (`SAFETY_WARN.md`), and **dragonpilot logs from
comma-two on the same car** (`VS_DRAGONPILOT_0803.md`). Order reflects what blocks progress first.

For lateral control there is a consolidated work plan with targets in meters: **`PLAN_TO_COMMA2.md`**.

## 1. Requires On-Vehicle Verification

Everything below is implemented in code and covered by tests, but **not verified on the road**. One run
with logging enabled closes most of the list: topics `safety/warn`, `vision/model_long`, and
`control/lane_keep_debug` are written to the bag; analysis uses `bag_safety_warn.py` and `bag_controller_ab.py`.

| item | verify |
|---|---|
| `path_camera_offset_m` 0.08 → **0.05** (camera remounted at `y_left=0`) | equilibrium within ±5 cm both with and without lane blending |
| MPC feedback coefficients restored (`epsi 0.3`, `cte_gain_base 0.6`, `floor 0.02`) | MPC controller actually holds the lane; on the phone `fp` is default, so this is not urgent |
| curvature-based speed limit (`long_plan.curv_*`, a_lat ≤ 1.8 m/s²) | κ ahead is computed adequately; execution currently only via cruise buttons |
| turn signal from `Gateway_72` (`0x3DB`) | signal actually arrives and bits are correct: older runs lack this frame, receive filter was dropping it |
| arc offset: `path_lane_blend_scale` 0.6, σ thresholds 0.3/1.5, width up to 4.6 | car cut inside the turn by 0.51 m (left arc) and 0.71 m (right) because the supercombo plan goes 0.33–0.48 m inside. Three config fixes applied. On-road expectation without the centering term (disabled): left +0.51 → **+0.45**, right −0.71 → **−0.54**, straight stays ≈0. This is noticeably more modest than needed; the fast loop and vehicle parameters should provide most of the gain — see `PLAN_TO_COMMA2.md`. **Not verified on road**; measure with `bag_arc_offset.py` |
| `center_force` disabled (0.0) | at real command rate (12.5 Hz, not 20 as in replay) the term does nothing: offset magnitude 0.21 → 0.19 on right arc and 0.26 vs 0.25 on left. It also eats stability margin — at 1.2 the left arc diverges (0.81 m, p95 2.95). Comma-two logs show tracking error ~0 without centering because the loop runs at 100 Hz. Revisit only after speeding up the loop |
| ~~`intrinsics_prior` fx 951 → 993.4 / 995.2~~ | **verified 2026-08-04**: camera odometry scale 0.844 → **0.888** vs expected +4.5 %. Remaining 11 % is not explained by focal length — model domain gap. `RUN_0804_PERCEPTION.md` |
| ~~CAN receive 50 → 10 ms~~ | **verified 2026-08-04**: `vehicle/state` 10 ms (p99 18, max 88), `controls/steer` 10 ms, panda callback 1.1 ms, no drops. But a vision tail appeared — see section 2 |
| **σ threshold analysis on arcs** | run 0804 turned out to be urban: only 38 frames with \|κ\|>0.004, zero arc episodes. Main question (0.51/0.71 m inside arc) still unverified; need a highway run |
| **calibration prior `rpy_deg`** | config has `pitch −1.79 / yaw 0.52`, learned value is `+1.4 / −2.7` in 1.5 min and holds ±0.2°. Camera was remounted, prior is stale; learned value is not persisted, so every drive starts with wrong warp |
| FCW/AEB | no recordings with lead vehicle data yet |
| `STEERING LIMIT` indication | appears when torque is actually at the ceiling |

## 2. Perception

**Model metric scale 0.888** (camera odometry vs wheel speed, run 0804, 6891 frames;
p10 0.848). Was 0.844 at fx 951; focal fix to 993.4 raised it exactly the expected 4.5 %. Remaining
11 % is not focal length: to reach 1.0 would need fx ≈ 1119, while the board gives 993 with RMS 0.38 px. This is the upper bound on accuracy for everything we measure laterally, and controller tuning cannot bypass it.

**The supercombo pipeline stalls for seconds — the most expensive item for the lateral planner on this list.**
If measured by time rather than frames: reference older than 300 ms **15.9 %** of run 0804 time and older than
500 ms — 11.8 %, mean reference age 96 ms vs 56 ms in 0802. Plus one stall of **75 seconds**.
Stalls cluster in two windows (15–20 and 30–36 min), while `vehicle/state` holds 10 ms and `traffic/state`
its own 100 ms — so neither CPU, CAN, nor timers are at fault; the camera → model chain stalls. Battery
52 °C and 16 % charge coincide with stalls, but windows at the same temperature ran clean, so overheating
does not explain everything. Next steps: log camera frame arrival separately from inference completion, capture
`logcat`, repeat at night.

**Safety gap closed with reference-age gate (2026-08-04).** HCA 250 ms timeout did not help:
`hca_cmd_ts_ms_` updates on any `controls/steer`, and the fast angle PID publishes at 100 Hz
regardless of reference age. In the 30–36 min stall `controls/steer` ran at mean dt 9.9 ms while
the plan was up to 75 s old; with control enabled the car would have steered on a minute-old plan (did not
trigger in this run only because panda blocked steering). Now `LaneKeepService` computes age from
frame capture timestamp and at `> lane_max_age_s` (0.3 s) sets status `stale`: command cleared, torque
zero, PID reset — integral does not accumulate, and recovery starts from zero. On run 0804 this means
control withdrawal 16 % of the time, which is correct behavior. Two tests via simulated
middleware. **Remaining**: separate HUD warning (currently visible only via assist indicator and status line) and root-cause analysis of the stalls themselves.

**Intrinsics measured with board (2026-08-03) and partially applied.** `fx` 951 → 993.4: measured field
of view 65.6° matched the EON camera (65.2°) the model was trained on, while 951 and flowpilot assumption
930 gave 67.9° and 69.1°. **Principal point intentionally left at center**: its shift
(1.04° yaw, 0.97° pitch) is equivalent to rotation and is already absorbed by online calibration; fixing
it without reset doubles σ (0.25 → 0.55, verified with `bag_intrinsics_ab.py` on 90 frames). Fixing
cx/cy only makes sense together with calibration reset — then the estimate stops being a mix of true
mount angles and principal-point error; it does not improve σ.

**Lane σ: 0.19 on straight and 0.60–0.93 on arcs vs 0.04–0.13 on comma-two.** This is a hardware
gap, not code: our input pipeline matches flowpilot line-for-line (1280×720, warp to `medmodel` 910 / 47.6,
six YUV planes, no normalization), and flowpilot on a phone would have the same.

**What actually differs in camera settings (verified 2026-08-04).** flowpilot on Android runs
`CameraHandler` (`AndroidLauncher.java:99`; CameraX branch with tone curve is commented out) and
sets only three things: **AE metering region on the road**, 20 fps, and autofocus. We set focus at
infinity, disable OIS, video stabilization, and distortion correction — they do not have this and it helps us
(frame geometry stays constant), — but **we do not set metering region at all**: exposure is computed over
the full frame including sky, and the road is systematically underexposed. This is the only setting
they have and we lack; verify σ with a separate run, one camera change at a time.

Previous wording "flowpilot sets `TONEMAP_MODE_CONTRAST_CURVE`" is **wrong** — the curve is in an
unused branch. For ISP processing (`EDGE_MODE`, `NOISE_REDUCTION_MODE`, tone curve) we match
flowpilot: both use defaults from `TEMPLATE_RECORD`, i.e. noise reduction and sharpening enabled. Disabling them is still worth trying, but as our own hypothesis, not a port. Set σ thresholds from
our distribution, not copy 0.15/0.30 from upstream.

**Our model is `sc_v0.8.12`, and updating it "like flowpilot" is not possible — they have a different artifact.**
Verified against flowpilot and dragonpilot checkouts:

* both ship `supercombo.thneed` (51 and 47 MB) — precompiled program for
  Snapdragon 845 GPU, tied to driver and their runtime. We run ONNX via onnxruntime, nothing to load
  it with;
* flowpilot base is openpilot 0.8.x (fork of commit `05b3755`, `FLOWPILOT_VERSION 0.2.3`), and its
  `driving.h` keeps the same layout as our model: `PLAN_MHP_N = 5`, 33 points, four lines with y/z.
  **So their code already fits our outputs** — that is why `center_force` ported without
  parser changes. No need to update the model to "rely on their code": we already do;
* dragonpilot is newer (`COMMA_VERSION 2023.04.13`, ≈0.9.3) — two-camera generation.

What we have and what a switch would require (verified by loading in onnxruntime):

| model | output | additionally required |
|---|---|---|
| `sc_v0.8.5` | 6409 | different output layout |
| **`sc_v0.8.12` (shipped) / `sc_v0.8.13`** | 6472 | — (files byte-identical, one copy redundant) |
| `supercombo_op089` | 6609 | different output layout |
| `sc_v0.9.0` | 6108 | **`big_input_imgs`** — second wide camera; `features_buffer[1,99,128]` instead of `initial_state[1,512]`; `desire[1,100,8]` |
| `supercombo_op094` | 6120 | same + **`nav_features[1,256]`** from navigation model |

0.9.x generation requires **second camera** and recurrent feature buffer instead of 512-dim
state, and `op094` also needs the navigation model. On a single-camera phone this is not a config swap,
but separate work with unclear outcome: feeding one image to both inputs is not what
openpilot does.

**Why change the model at all.** Plan offset inside the arc (+0.12 / −0.25 on comma-two,
+0.32 / −0.35 for us) exists on both, and tuning it with a coefficient failed — see section 4.
Whether another model cuts less, **is measured offline on recorded frames** (`bag_intrinsics_ab.py`
is built exactly for this — just swap the ONNX path), without a drive. Do this before any
replacement. But priority is lower: upstream removes plan offset via lane blending, not model choice.

**Perception cannot be verified in the simulator.** supercombo is trained on a real camera; on MetaDrive
render lane probabilities drop to 0.03–0.13 in the first arc and the car leaves the lane. Either full-pipeline
testing lives only on recordings (as now), or domain adaptation is separate major work.

## 3. Simulator and Offline Metrics: What They Cannot Do

**Fixed 2026-08-03 so replays can be trusted:**

* pose resync no longer replaces **lateral** position with the recorded one. Previously every 20 s the car
  was placed exactly where the driver was, and the metric partly measured offset relative to the driver's line.
  Now only along-road drift and absolute heading drift reset; old behavior under `--resync-full`;
* `core/path_fusion.py` (Python mirror of `laneLinesToPath` that the whole simulator uses) did not
  weight blending by σ at all, checked width with sign, and kept old bounds — i.e. replays tested different code than what drives. Brought in line;
* replay config set had two mistakes: default delay 0.23 instead of 0.35 in the run and
  double application of path shift. Ranking unchanged; absolute numbers shifted by 0.08 m;
* "HCA was steering" flag in offline analysis — `controls_allowed` from `panda/health`, not
  `steer_output_enabled`: the latter was 1 in all frames on the 02.08 run and filters nothing.

**Upstream log reading pitfalls** (`rlog_arc_offset.py`), each of which made the first measurement
zero or nonsense:

* `controlsState.active` true in only 1 % — on comma `dp_atl` (always-on lateral) is enabled, and
  lateral control works when openpilot is not "engaged". Use `carControl.latActive`;
* their `desiredCurvature` is right-positive (plan frame), while `steeringAngleDeg` is left-positive,
  so correlations with lane curvature have **opposite signs** — as expected;
* last route segment is truncated mid-recording: read only what unpacked;
* Python model parser **did not read lane σ at all** (only first 264 of 528) — added,
  σ = exp(second 264). Without this offline analysis misses the main quantity.

**What replay cannot show in principle:** lanes in it are model estimates, not ground truth, so
systematic perception error is invisible (controller and metric see the same paint); and replay
does not re-capture — when diverging from recorded trajectory, far polyline points come from a viewpoint
where they were not observed.

**Open loop (compare command to driver steering) does not answer offset**: it measures command agreement,
offset is the integral of command, and steady error is invisible.

Vision delay (90 ms) and reference noise (0.15 m at 20 m) are set and measured from runs, but
**replay does not reproduce controller ranking**: PP holds lane better than `fp` at any
degradation level tried, while on the road it is the opposite. Plan jumps (`--vision-jump-hz`,
p95 |Δy| ≈ 1.6 m on runs) hit `fp`, not PP, so disabled by default.

So for controller choice replay is unsuitable — only as regression on a fixed
controller. To change that, need a perception error model from frame-by-frame run analysis
(model plan vs lane-marking path), not inter-frame spread. On available runs
lane markings are too poor for such comparison.

## 4. Lateral Control: Open Items

**Consolidated plan — `PLAN_TO_COMMA2.md`.** Individual items below; work order and expected effect
there, with numbers from comma-two logs on the same car.

**Gap measured on the same car.** Offset magnitude on arcs: comma-two 0.07 and 0.20 m, us 0.51 and
0.71, manual driver 0.15–0.17. On straight we are already smoother than the human (0.12 vs 0.16). Breakdown:
their tracking error +0.00 / +0.03 / −0.08 and **constant** setpoint offset (−0.06…−0.08 = their
`CAMERA_OFFSET`), ours +0.07 / +0.35 / −0.23 and setpoint wandering from −0.10 to −0.42.

**Three vehicle parameters differ from values learned on this same car** (`liveParameters` on comma, low
spread):

| parameter | ours | comma learned | note |
|---|---|---|---|
| `steer_ratio` | 15.7 fixed | **16.27 ± 0.10** | their port marks 15.6 "let it learn" |
| `tire_stiffness_factor` | 0.64 | **1.319 ± 0.007** | our `slipFactor` at 1.0 gives exactly their VW stiffness (184691 / 238876), so quantities are directly comparable |
| steering bias | no mechanism | +0.094° | |

Consequence: at 22 m/s we over-command by **1.48×** vs their **1.23**. This also explains
why stronger understeer compensation (`tsf 0.50`) made offset worse in replay.

**But resolve the contradiction first.** My measurement from our runs gives κ_fact/κ_kin 0.54 at 22 m/s,
i.e. understeer **stronger** than even our aggressive 0.64, while their learning says the opposite. Suspect —
`yaw_rate` scale from CAN; ready check (`mapmatch::analyzeYaw` computes mutual scale of CAN and
phone gyro). Until verified — do not touch `tire_stiffness_factor`.

**We have no road grade estimate at all.** From comma logs on this route grade median −0.78°,
p10 −2.55°, max 3.68°. Median 0.8° gives lateral acceleration 0.14 m/s², i.e. constant
curvature 3.4·10⁻⁴ at 20 m/s that the controller must counter with cross-track feedback. This is
the only found cause of **constant** offset not explained by anything else.

**`paramsd` not ported.** 9-state Kalman filter learns stiffness, steer ratio, two
steering biases, and road grade. Closes three items above at once and removes hand-tuning constants. Their
implementation on `rednose` with sympy codegen is not portable, but the filter itself fits in roughly
150 lines on Eigen. State, observation, and gate breakdown — `docs/PARAMSD.md`; boundary from there:
steer ratio and stiffness learn from CAN; grade needs its own orientation estimate.

**Target is 0.15 m, not zero.** On the same arcs the driver without controller also goes ±0.14 m inside
the turn, and holds 0.15–0.17 m magnitude on straight and arcs. Controller on straight is already smoother
than human (0.12 vs 0.16), on arcs 3.6–5× worse. Bar measured on 39 and 69 frames — one or two
arcs; confirm on other runs.

**Curvature plan correction — verified on two runs and rejected.** Idea was to remove plan offset with one
coefficient. Failed:

* "constant distance" (`50.2·κ`) unstable even within one run: 20.2 at v < 12 m/s vs
  77.2 at v ≥ 12 — fourfold spread;
* correct parameterization is **time lookahead** (`½κ(vT)²`), and T is stable vs speed:
  0.71–1.03 s, 0.83 on arcs, 0.84 s on run (10.9 m at 13 m/s);
* but **not reproducible between runs either**: 0.84 s vs 0.56 s, and plan offset on second run
  is asymmetric (+0.37 left, −0.15 right), and correction from first run leaves +0.23 / +0.14 m there,
  i.e. removes 38 %;
* runs differ in camera calibration (yaw +1.12° vs +0.24°), which goes into input-image
  warp — network sees different image and outputs different plan.

Cannot bake in such a coefficient: on another calibration it adds its own offset. Details and
tables — `RUN_0802_ARC_OFFSET.md`.

**Consequence favoring two other levers:** lane blending requires no tunable number
at all, and integral term removes steady offset regardless of cause, including the part that
changes run to run. That is why integrator is now the main candidate, not correction.

**`blend 1.0` remains fallback**: it discards the plan where lane paint is visible, while upstream goes
the other way (openpilot moved to pure plan because lanes fail in construction, snow, and
double markings). But it is parameter-free.

**Integral cross-track term — without it 0.15 is unreachable.** P-term (`center_force`) divides
steady error by (1+k) = 1.4, but 1.7–2.3× is needed. Required reference shift 0.36 m (left arc) and
0.28 m (right) — within ±0.8 clamp, i.e. achievable on paper. Conditions: clamp ±0.25 m, τ 5–10 s,
freeze without lanes, reset on override and lane change, camera calibration verified **first**.
Otherwise integrator turns calibration error into real offset while logs show "centered".
Upstream has nothing like this.

**Right arc at `blend 0.6` up to 0.14 is unreachable at any tracking quality** — setpoint offset
(−0.31) exceeds entire budget 0.14·(1+k) = 0.19. Plan correction is mandatory, not optional.

**`fp` does not hold near cross-track offset on arcs — closed by porting `center_force`.** Car sat
0.35 m inside its own reference, and this is not tuning: state resets every frame
(`x0 = [0,0,0,ψ̇]`), so residual at zero node does not depend on control; nodes 1–5 sit at
0.1–3.2 m ahead where lateral motion is impossible; plus no integral term. Closed loop confirmed
neither `fp_steering_rate_weight` (150 and 800 — no change) nor `fp_steer_delay_s` (0.23 vs
0.35) affects this.

Ported `center_force` from `flowpilot/selfdrive/controls/lib/lane_planner.py` — proportional
term on cross-track at the vehicle, added to entire reference, clamp ±0.8 m, attenuation toward
current turn direction. Lives in `laneLinesToPath`, coefficient `center_force_gain = 0.4` tuned by
replay (upstream 0.72…1.2 oscillate). Still to verify on road.

**Single lane does not hold reference, and this is intentional.** We require both host lines for `anchored`,
flowpilot needs only the better one (`lane_trust = clamp(1.2·max(l_vis, r_vis)^0.5, 0, 1)`). Did not relax yet:
with one line center is invented from assumed width, and `laneLinesToPath` documents where that ends —
drift into parked cars or shoulder. flowpilot backs up with
road edges (`roadEdges` with σ correction), which we do not read at all. Measured need was partly closed by σ threshold: with `lane_std_bad_m = 1.5` share of arc frames without reference drops from 26 %
(left) and 67 % (right) to 23 % and 9 %. Correct order — road edges first, then
relax two-line requirement.

**Horizon change not needed.** flowpilot keeps `N = 32` (10 s), we took dragonpilot `N = 16`
(2.5 s), but grid `T_IDXS` is quadratic: 11 of our 17 nodes in the first second — same as
flowpilot. Lengthening only adds far nodes and at equal weights dilutes the near zone,
and costs roughly 4× more (we use GD instead of acados). Details — `RUN_0802_ARC_OFFSET.md`.

**HCA torque hits ±300 cNm ceiling in 34–41 % of arc frames** (19 % on straight). While torque
is saturated, there is no feedback at all. This is MQB HCA limit, not fixable in config; reduce
requested curvature (item 1: lane weight), not coefficients.

**Understeer compensation is low at speed.** From steady arcs measured transfer
κ_fact/κ_kin: 0.97 at 6–9 m/s (model expects 0.96), 0.80 at 12–15 (0.87), 0.54 at 21–26 (0.69).
On fast arcs command is 14–29 % low and pulls car **outside**. This is a **separate** task from
cutting inside on urban arcs: stronger compensation (`tire_stiffness_factor` 0.50) in replay on urban arc
made it worse (−0.44 → −0.52 m). Touch only for highway run and measure on that.

## 5. Localization

**Heading in `localization/pose` comes almost entirely from bicycle model, not measured yaw rate.** In
`VehicleEKF::predict` heading grows from prediction `v·tan(δ)/L`, and yaw-rate state is **overwritten** by the same
prediction. Correction from measurement reaches heading only through cross-covariance in one step, weight `dt·(1 − K₄) ≈ 0.12·dt`:
heading becomes 0.88 · bicycle + 0.12 · measurement (verified numerically on our Q/R:
`Q₄₄ = 0.05²`, `R_imu = 0.02²`).

Cost: with measured understeer (κ_fact/κ_kin = 0.54 at 22 m/s) prediction over-turns by 1.85×,
so heading rotates **1.75×** faster than truth. On arc with ω = 0.15 rad/s for 30 s that is 450°
vs 258°. On road this is masked by GPS heading correction (`updateGpsYaw`, hard snap above
>0.5 rad), so runs did not show it; without GPS heading drifts.

Proper fix is structural: yaw rate should be state with its own random walk
(or heading should integrate measured rate), not be overwritten by prediction. Learned
vehicle parameters (`docs/PARAMSD.md`) improve the prediction itself where measurements are absent:
before GPS heading seed and when IMU invalid but CAN gives zero.

`mapmatch` track is **not** affected: `buildTrack` integrates measured yaw rate directly,
steer angle does not participate.

## 6. Missing from the System Entirely

* **lane change** — no `DesireHelper`; turn signal is decoded but used only to
  silence LDW;
* **urban** — no free corridor (drivable space), no parked-car detector;
* **interchanges, exits, intersections** — model plan is not designed for them.

## 7. Minor Items and Hygiene

* `models/sc_v0.8.12.onnx` and `models/sc_v0.8.13.onnx` are **byte-identical** (md5
  `d95f5d06…`), same as `assets/supercombo.onnx`. One of three copies is redundant.
* `maps/Moscow.osm.pbf` — 81 MB of 87 MB in `maps/`. Needed only to rebuild
  `Moscow.osm.admap` (4.9 MB), re-downloaded by `mapmatch/fetch_map.py`. Candidate for deletion,
  your call.
* Dated reports `RUN_0801_*` and `PIPELINE_AUDIT_0801.md` contain suggestions since implemented
  differently. Not rewritten: history with measurements. Where a later measurement refined the
  conclusion, explicit footnote — e.g. `RUN_0801_LEFT_DRIFT.md` (plan offset turned out to be function of
  curvature, not constant left pull), `RUN_0801_LOWSPEED.md`, `PIPELINE_AUDIT_0801.md`.

* `models/` — ~250 MB weights with no references (`sc_v0.8.12`, `sc_v0.9.0`,
  `supercombo_op089`, `supercombo_op094`, `yolov8n_traffic_192`, `yolov8n_coco_256`,
  `ru_signs_best.pt`). Not in git. **Intentionally not deleted**: artifacts, not junk, and
  cannot be restored from repo — decide what you need.
* `.tools/gradle-home` — 1.4 GB local cache inside project. Not in git; deletion costs
  re-download.
* `docs/calib/` — two chessboards (1920×1080 and 2560×1440), in git intentionally: needed for
  repeat calibration.
* `docs/FRAME_DT_FIX_0801.md` — five mentions of removed `fpa` controller. Dated report with
  measured numbers, left as-is.
* Book build gives 4 warnings: links from chapters to docs outside `docs/book` (`../README`,
  `../IMAGE_TO_CAN_PIPELINE`) sphinx does not treat as documents. Links work in editor and on GitHub,
  not in html.
* CI (`.github/workflows/ci.yml`) is written but **never ran** — no access to
  GitHub Actions. First run will likely need fixes.

## Done 2026-08-04

* **Runtime knobs instead of per-parameter JNI.** Service declares a knob by name
  (`registerParameter` / `setParameter` / `updateParams`): writes from other threads are queued,
  middleware applies them on the service thread between callbacks. Six `nativeSet*` replaced by one
  `nativeUpdateParams(json)`,
  `AdasApp` no longer calls services directly, goes through mw. **Lane blend** slider in
  parameter panel: compare 0.6 vs 1.0 on same arc in same run; restarting app between drives gives different calibration and lane. Four tests on mechanism.
* **Container build no longer breaks install.** `~/.android` was not mounted, so
  container created new debug keystore each run: APK signed with different key each time,
  `adb install -r` failed with `INSTALL_FAILED_UPDATE_INCOMPATIBLE`, had to uninstall app with
  calibration and granted permissions. Host keystore mounted.
* **supercombo actually runs on NNAPI** (`NNAPI EP enabled`, 278/280 and 372/374 nodes),
  no silent CPU fallback. So ~80 ms inference is NNAPI speed on this phone; vision stalls
  must be sought elsewhere.

* **Run 0804 analysis** — `RUN_0804_PERCEPTION.md`: perception after focal fix, planner reference,
  100 Hz loop confirmation, and why steering was silent.
* **Panda with comma-two firmware no longer breaks control.** Its `pandad` reflashes its
  firmware, which returns `health` version 11 instead of 16: everything after `rx_buffer_overflow` shifts by
  5 bytes, ignition reads zero, VW mode never confirmed, HCA not sent. Now
  packet version is queried on connect (`0xdd`) and v11 parsed with its own struct. Three tests,
  including reverse check — old code on same bytes gives exactly values from run.
* **Camera angular velocity was 57.3× too small**: `CameraOdometry.java` applied
  `toRadians` to quantity already in rad/s (as in cereal and flowpilot `driving.cc`). Because of this
  "driving straight" gate in `PoseCalibrator` never fired in the run, and localizer got
  near-zero turn rate as measurement. Impact on this run's calibration was small
  (7 % accepted samples in turn, heading 0.07° off), but after fix there is a **third
  independent yaw scale source**: camera vs ESP gives 0.92 at correlation −0.879.
* Comparison with dragonpilot on **same car and roads** from its logs: where exactly the gap
  (`VS_DRAGONPILOT_0803.md`), plus stage-by-stage pipeline check — what matches 1:1 and what diverges.
  Tool `rlog_arc_offset.py` reads rlog via cereal schema and computes same breakdown as
  `bag_arc_offset.py`.
* Consolidated work plan with meter targets — `PLAN_TO_COMMA2.md`.
* Intrinsics calibration with chessboard: `camera_calib_chessboard.py` (parameters like flowpilot —
  9×6, subpixel 7×7, view selection by diversity), board generator `make_chessboard.py`, boards
  in `docs/calib/`. Verified on synthetic with known fx: 700 → 700.2 at error 0.083 px.
  Result on run: 35 views, error 0.383 px, fx 993.4 (taken into config), principal point intentionally
  not taken.
* `bag_intrinsics_ab.py` — run model on bag frames with two geometries and compare σ,
  probabilities, lane width, and plan offset. Also caught principal-point error.
* Lane σ reading in Python model parser (previously not read at all).
* Inner loop: CAN receive timer 50 → 10 ms, i.e. angle PID now at 100 Hz like theirs.
* Board capture mode `calibration.camera.chessboard_capture` (autofocus instead of fixed infinity),
  off by default — cannot drive with it.
* Bug fix in `CameraHandler`: OnePlus returns `LENS_INTRINSIC_CALIBRATION` as non-empty zero array,
  and check `!= null && length >= 4` overwrote correctly computed focal with zero — hence
  `focal_length_px = 0` in all previous runs.
* `push_config.sh`: delivery to device did not work at all. Three causes — relative path,
  SELinux block on redirect inside `run-as` (`cat > ...`), and same on `wc -c < ...`. Write
  via `/data/local/tmp` and `run-as cp`. Added `--set path=value` on top of calibration key protection: without it new calibration could not be delivered to device.
* `center_force` disabled per measurement (see section 4).

## Done 2026-08-03

* Analyzed arc offset: car cuts **inside** turn by +0.51 m (left) and −0.71 m
  (right), centered on straight. Decomposed into tracking error and setpoint offset, cause of
  each found — `RUN_0802_ARC_OFFSET.md`. Reproduced on second run.
* Measured bar: driver without controller also goes ±0.14 m inside, magnitude 0.15–0.17 m. On straight
  controller already smoother than human.
* Config: `path_lane_blend_scale` 0.3 → 0.6, σ thresholds 0.2/0.8 → 0.3/1.5, width up to 4.6 m with filter.
* Ported `center_force` from flowpilot (`center_force_gain = 0.4`, chosen by replay; upstream
  0.72…1.2 oscillate). Plus dead zone on curvature gate — without it term lost 30 % on straight from
  numerical fit noise.
* Run is self-documenting: `lane_offset_m`, `center_force_m`, `lane_width_m`, `lane_anchored` and
  three config keys written to `control/lane_keep_debug`.
* New tool `bag_arc_offset.py` — offset breakdown on run with self-checks and plots.
* Fixed three offline-method items (pose resync, `path_fusion.py` mirror, "HCA steering" flag) and two replay config mistakes — section 3.
* Test `AdasConfigFile`: key added to `config.json` and forgotten in `adas_config.cpp` now
  fails test instead of silently lost. Tests 82.
* `scripts/push_config.sh` — deliver config to device without losing camera calibration.
* `MAPMATCH.md` — OSM map localization breakdown: how it works, results, what remains.

## Done 2026-08-02

Below — items that were on this list and closed. Details in corresponding documents.

* FCW/AEB switched from IDM desired decel to threat (TTC and required braking); target —
  only `lead0` and only in own lane. LDW got gates on speed, lanes, turn signal, outward drift
  plus debounce. 131 false triggers on run → 0 (`SAFETY_WARN.md`).
* Curvature-based speed limit ahead in `long_plan` + 6 tests on curvature preview.
* `laneLineStds` in protocol, Java parser, and blend weight + 4 tests.
* Turn signal decoded from `Gateway_72`, gates LDW on its side.
* `STEERING LIMIT` in app when torque at ceiling for ~0.5 s.
* Simulator: controller replay on tracks with set radii, ground-truth metrics, delay
  and perception noise, window with same tracks (`SIM_CONTROLLER_TEST.md`).
* Docker: both images built and verified, `tests` / `host` / `sim` / `apk` inside container.
* Tests: 71 vs 48; removed three dead RPC stubs, working ZMQ integration test instead of skipped one;
  fixed long-failing `SnapshotStatsTracksTimers`.
* `jupyter-book` pinned to 1.x (2.x is mystmd with different CLI, build failed on it).
