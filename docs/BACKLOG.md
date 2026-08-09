# Items Requiring Further Work

Compiled 2026-08-02, updated 2026-08-07. Sources: road bags, simulator (`SIM_CONTROLLER_TEST.md`), warnings (`SAFETY_WARN.md`), and **dragonpilot logs from
comma-two on the same car** (`BENCHMARK_COMMA2.md`). Priority order is in §0; the sections below are grouped
by subsystem, not by rank.

For lateral control there is a consolidated work plan with targets in meters: **`BENCHMARK_COMMA2.md`**.
The divergence catalogue against their tree and their logs is **`DIFF_FROM_DP_RU.md`**.

## 0. Priority: ranked by measured divergence from comma on this car

The ranking criterion is deliberate and narrow: **on the same car, on the same route, dragonpilot drives
well and we do not.** So the ordering is not by how interesting an item is, nor by how cheap — it is by how
large the *measured* gap to their stack is, and every item names their value, ours, and the number that
closes. Three things make this criterion usable rather than rhetorical: we have their code
(`/home/anatoly/atom/dragonpilot`), their rlogs from this car and route
(`7b63cf08132829d3_2026-08-03--19-01-52`), and a harness that pushes their recorded inputs through our stack
(`app/src/main/scripts/rlog_lat_diff.py`). Divergences are enumerated in `DIFF_FROM_DP_RU.md`; this section
ranks them.

**What the criterion changes.** Two items that ranked high on the old ordering drop, and one that was not on
it rises to the top — see «Deliberately demoted» below.

| # | divergence | comma | us | rank rests on | cost |
|---|---|---|---|---|---|
| ~~**P0**~~ | ~~actuation state is not fed back~~ | resets the controller when `not active` | ~~`steer_output_enabled` stays true regardless~~ | **done 2026-08-07**, see below | — |
| ~~**P0**~~ | ~~harness is not yet fair (#26)~~ | — | — | **done 2026-08-07**, and it retired a divergence — see below | — |
| ~~**P1**~~ | ~~gate to steer: ATL/ALKA (#24)~~ | `latActive` on cruise **available** | ~~on cruise **engaged**~~ | **built 2026-08-07, shipped off** — assist was present in 2.6 % of frames at 5–8 m/s and 18.3 % at 8–12; see below | one drive |
| ~~**P2**~~ | setpoint recompute rate (#25) | 100 Hz, median step **0.0067°**/10 ms | once per frame, held 78 ms, median jump **0.107°**, p99 **2.48°** (3.78° at 5–12 m/s) | **built 2026-08-07, shipped off** — the only one of the three that was a real divergence | done |
| ~~**P2**~~ | ~~friction feed-forward (#25)~~ | `latcontrol_torque.py` only — **not run on this car** | angle PID, no friction term | **withdrawn 2026-08-07: misattributed, and measurement argues against it** — see below | — |
| ~~**P2**~~ | ~~`freeze_integrator` (#25)~~ | `latcontrol_torque.py` only — **not passed in `latcontrol_pid.py`** | same as theirs: `override=steeringPressed` and nothing else | **withdrawn 2026-08-07: misattributed** — our PID already matches theirs | — |
| **P3** | σ gate (#23) | scalar `laneLineStds[i]`, ramp **0.15 → 0.30** | median `y_std[]` over 5–20 m, ramp **0.30 → 1.50** | their gate is binary in practice: σ median **0.086** but **39 % of frames zeroed** — see the measurement below. The two σ are different quantities and copying their thresholds would discard most of our frames | offline |
| **P4** | lane σ itself | 0.04–0.13 | 0.19 straight, 0.60–0.93 arcs | bounds everything measured laterally; the one camera setting they have and we lack is **AE metering on the road** | one line + one run |
| **P5** | blend scale | `d_prob` as-is → reference is the lane centre | ×0.6 → 60 % centre, 40 % model plan | their setpoint offset is a **constant** −0.06…−0.08 (their `CAMERA_OFFSET`); ours wandered −0.10…−0.42 before blending and +0.02 / −0.10 after | config |

**The first P0 item is done (2026-08-07).** `LaneKeepService` now subscribes to `panda/health`, and the
angle PID is gated on whether torque is actually reaching the rack (`lat_require_assist`, shipped on).
Three things came out of building it:

* **the windup was real and it is measurable.** With the gate off, resuming after one second of withheld
  torque commands **283 cNm** where the honest command is **203** — 74 cNm the wheel never earned, and the
  step lands the moment the assist returns, which is exactly the reported «на выходе из дуги приходится
  подкручивать». Gated, resumption is 203 cNm regardless of how long the torque was withheld. The test that
  pins this carries its own control group: the same comparison with the gate off, so it cannot pass for the
  wrong reason;
* **«we do not know» needed two different answers**, and getting this wrong would have silenced every
  offline harness. Never having heard from a panda means there is no panda in the loop — a bag replay, the
  Python bindings, a bench run — so the gate stays open. Having heard from one and then losing it means we
  are on the car and the device stopped talking, in which case it is not passing torque either, so the gate
  closes. `dbg.assist_known` distinguishes the two in the recording;
* **reporting is deliberately not tied to the gate.** `assist_allowed` and `assist_known` are published on
  `control/lane_keep_debug` even with `lat_require_assist = false`, because separating actuated frames from
  the rest is what future analysis needs whether or not the gate is doing anything. Existing bags do not
  carry the fields, so analysis of past runs still has to go through `panda/health.controls_allowed` — the
  scripts were left alone rather than changed for a field no recorded bag has yet.

Six tests, `171` in the suite. **Not verified on road**: this withdraws control in about 70 % of the frames
of a drive like 08-06, which is correct behaviour but a large change in what the car does.

**The second P0 item is done (2026-08-07), and its result was to delete a divergence rather than measure
one.** `rlog_lat_diff.py` now runs all routes, feeds either their finished plan or their raw `modelV2`,
compares after the limiter as well as before, and survives damaged segments. Over **101 975** matched frames
of the 28 usable segments:

| stage | theirs | ours | agreement |
|---|---|---|---|
| curvature (planner output, before any vehicle model) | median 0.00042 1/m | 0.00034 | slope **0.897**, corr **0.966** |
| setpoint angle | median 1.40°, p90 12.84° | 1.12°, p90 13.26° | slope 0.930, corr 0.965, disagreement **2.40° rms**, median +0.08° |
| torque before the limiter, integrator **on** | median 69 cNm, 10.1 % at the ceiling | **229 cNm, 35.1 %** | slope 0.51, corr 0.31 |
| torque before the limiter, integrator **off** | median 69 cNm | **78 cNm, 21.7 %** | median disagreement **+10 cNm** |
| torque after the limiter (`steerOutputCan`) | median 53 cNm | 60 cNm | rms 143 against 170 before it |

**The 3× torque gap was the harness, not the car.** Two stages agreeing to 0.08° of median and 0.966 of
correlation cannot produce a threefold torque disagreement. The cause is that the replay is open loop: the
measured steering angle comes from their log, so it is the angle *their* torque produced, our command never
moves the wheel, and our integrator accumulates an error it cannot influence while theirs was logged closing
it. With `ki = 0` the torque medians are 78 against 69. So **every torque number this harness produced before
today was an artefact**, and the honest reading is: curvature and setpoint angle are trustworthy here, torque
only with `--no-integrator`, and steady-state offset not at all.

**Running the same sweep in `--reference model` localises what is left.** Feeding their raw `modelV2` through
*our* lane fusion instead of feeding their finished plan, over the identical segments:

| | reference = their plan | reference = their model, our fusion |
|---|---|---|
| curvature slope / corr | 0.897 / 0.966 | **0.976** / 0.967 |
| setpoint angle slope | 0.930 | **1.011** (by band 0.99 / 1.14 / 0.97) |
| angle disagreement rms | 2.40° | 2.50° |
| frames past the 1.67° rail | 18 % | 25 % |
| angle rms at 10–15 m/s | **4.22°** | **4.23°** |

So our lane fusion is **not** a source of systematic scale error — on their model output it reproduces their
setpoint scale to within 1 %. And the residual scatter is the same number in both modes, which places it
between the reference polyline and the setpoint angle, i.e. in our path → curvature → angle stage rather than
in fusion or in the controller.

What survives as a real residual, then: we hit the ±300 ceiling in **21.7 %** of frames against their 10.1 %
even with the integrator off, because 18–25 % of frames exceed 1.67° — the error at which our PID output rails
— and that tail is concentrated at **10–15 m/s (4.2° rms against 1.25 at 15–20 and 0.51 at 20–30)**. That band
is the one to look at next. It is also where the setpoint is held as a 78 ms step rather than recomputed
(§0 P2), which is the first hypothesis to test rather than a conclusion.

Three smaller things the work turned up, each of which had been silently degrading the comparison:

* **their setpoint angle is unusable at low speed** — `actuators.steeringAngleDeg` reaches 649.7° with a p90
  of 130.8° even restricted to `latActive`, because a short plan implies a large curvature and their vehicle
  model turns it into an angle no rack has. Their `desiredCurvature` over the same frames stays physical
  (p90 0.049 1/m, a 20 m radius), which is why curvature is now the primary comparison and the angle
  comparison is gated on what our own planner could produce;
* **one segment is corrupt and two routes are empty**, and `bz2.decompress` raising `OSError` killed a
  four-route sweep at the second route. Reads are chunked and per-segment now, and what was lost is printed —
  a silently shortened sweep is how a harness comes to claim coverage it does not have;
* **`actuatorsOutput.steerOutputCan`** carries their post-limiter value in cNm directly, so the comparison no
  longer round-trips through the ±1 normalisation.

**P0 first, and not because it is cheap.** Both P0 items are about the measurement, not the car, and the
reason they outrank the gate is `4a`: the assist was absent 70.7 % of the time and *nothing in our stack said
so*, so the achieved-angle column that made our controller look badly tuned was mixing frames where no torque
was applied at all. Any P1–P5 change measured before P0 lands gets the same contamination. The harness item
matters for the same reason from the other side — their rlogs contain the inputs and their own outputs, so a
fair harness answers «did this change move us toward comma» without a road slot.

**P2 was three items and only one of them was real — corrected 2026-08-07 by reading the controller this car
actually runs.** `lateralTuning = pid`, so the code path is `latcontrol_pid.py`, and both
`freeze_integrator = steer_limited or …` and `friction_compensation` live **only** in `latcontrol_torque.py`.
Consequences, each of which would have sent a change to the wrong place:

* **`freeze_integrator` is not a gap.** `latcontrol_pid.py` calls `pid.update(error,
  override=CS.steeringPressed, feedforward=ff, speed=CS.vEgo)` — exactly what we call. Their `steer_limited`
  feeds `_check_saturation`, a counter that raises a UI event, not the integrator. Freezing on the rate limit
  remains a defensible idea on physics, but it is **our** hypothesis and has to be tested closed-loop, since
  the open-loop harness manufactures that very windup (§3);
* **friction compensation is not a port either**, and there is now a measurement against it. Their friction
  term acts in a *lateral-acceleration* loop; ours is an angle loop, so the equivalent would be our own
  invention. And with the assist actually on, our achieved fraction of the commanded angle is **0.92–0.98** —
  the wheel reaches the angle asked for, so there is no stiction deficit visible to compensate. Adding 57 cNm,
  19 % of full range, to a loop that already tracks that well is more likely to introduce a bias than remove
  one. The 0.192 stays useful as a measurement of this rack's friction, not as a controller change;
* **the setpoint rate was the real one**, and it is the one the differential replay independently pointed at:
  the residual disagreement sits between the reference polyline and the setpoint angle, concentrated at
  10–15 m/s. Built, see below.

A fidelity detail found while reading their PID and worth keeping in mind: their feed-forward uses the desired
angle **without** the learned offset ("offset does not contribute to resistive torque"), ours includes it. It
is latent today because `use_learned_params` ships `false` and the offset is ~0.01–0.09° anyway.

**Deliberately demoted by this criterion:**

* **`tire_stiffness_factor` 0.50 → 0.40** — `BENCHMARK_COMMA2.md` step 1 still says «next», and under this
  criterion it drops, because it moves *away* from comma: their learner settled at **1.247–1.319** on this
  car while we ship 0.64 and the proposal is 0.40. §5a established that this is an identifiability finding
  rather than a tuning disagreement — stiffness and steer ratio both scale predicted curvature, and one
  drive's speed range cannot separate them — and a direct yaw-residual fit on our own bag put the optimum at
  0.55 and called 0.64 near-optimal. Do not spend a road slot moving further from their value on the strength
  of a fit that flat;
* **roll compensation in κ→δ** — they have it, we do not, so it is a genuine divergence, but the estimate is
  ~0.00013 1/m of curvature per degree of bank, i.e. **0.03° of steering wheel**. Real and negligible;
* **discrete laneless switching, lane-width speed prior, `LOW_SPEED_FACTOR`** — divergences with no measured
  cost yet. Port them when something else is already open in that file, not on their own;
* **model 0.9.x** — needs a second camera. Not a divergence we can close on this hardware.

**The ALKA bit is settled, 2026-08-07, and it was settled by their drive rather than by their firmware.**
dragonpilot ships **no panda board sources** — `panda/board/` holds only `obj/panda.bin.signed` — so the
"which branch defines bit 16" question has no answer in the tree. Their python side is the readable half:
`panda/python/__init__.py:130` declares `ALTERNATIVE_EXPERIENCE.ALKA = 16` and line 186 declares
`HEALTH_PACKET_VERSION = 11` (`CAN_HEALTH_PACKET_VERSION = 4`). The flowpilot checkout that named bit 16
`ALT_EXP_ALLOW_AEB` is a **different generation** — its `board/health.h` is version **16** — which is the whole
reason the two readings looked like a contradiction. Our own `include/panda/health.h` already carries both
layouts and auto-detects, and the v11 struct's tail (`gmlan_send_errs`, `gas_interceptor_detected`,
`usb_power_mode`, `torque_interceptor_detected`) is the older/forked shape.

The behaviour on the flashed firmware, measured across all their recorded routes:

| quantity | value |
|---|---|
| `pandaStates.alternativeExperience` | **17** = bit 1 (`DISABLE_DISENGAGE_ON_GAS`) + bit **16** |
| `safetyModel` | `volkswagen` |
| `safety_tx_blocked` | **0**, never incremented once across every route |
| `carControl.latActive` with `controls_allowed` **false** | **64.3 %** of frames |
| of those, non-zero applied torque | **96.3 %**, median **53 cNm**, max 300 |
| `controls_allowed` over the whole drive | **1.1 %** — and `cruiseState.enabled` is the same 1.1 % |

So the panda accepted HCA torque with `controls_allowed` false and blocked nothing. **On this firmware bit 16
is ALKA.** It also reframes the comparison: on that drive dragonpilot steered the car for two thirds of the
time with the stock cruise essentially never engaged — the exact condition under which our stack steers 0 % of
the time. `controlsState.dpLateralAltActive` is 0 % throughout, so that field is a different dp feature and
not the mechanism.

**Built 2026-08-07, and it ships `false`.** `lat_always_on` in `assets/config.json` drives both halves from
one switch: `PandaService` sends `alternative_experience = 17` and `assistAllowed()` opens the gate on the
cruise **main switch** instead of on `controls_allowed`. Five tests, 177 in the suite. Three things about the
implementation are worth knowing:

* **the flag had to reach `LaneKeepService` too, and via a new signal rather than the old one.** The assist
  gate built earlier the same day read `panda/health.controls_allowed` — which stays *false* under ALKA. Left
  alone, turning on always-on lateral would have made the lane-keep service withdraw the command and reset the
  PID in exactly the 64 % of frames where the assist finally worked, and the two changes would have cancelled
  out. `PandaService` is the only place that knows both halves, so it now publishes
  `panda/health.lat_actuation_allowed` and `LaneKeepService` reads that. Consequence to remember: bags recorded
  before today carry `false` in that field, so they cannot be replayed through the gate;
* **two of upstream's four conditions are deliberately not repeated.** `CarController::update` already
  enforces standstill (`CS.standstill || |vEgo| < 0.3` — the same 0.3 m/s as their
  `MIN_LATERAL_CONTROL_SPEED`) and the EPS check (`epsHcaAllowsSteer`, Ready or Active only) on every frame.
  Duplicating them in the service would create two places to keep in step. What is new is `cruiseAvailable`
  (`TSK_Status != 0/1`, already decoded) and `gearReverse` (`GE_Fahrstufe == 6` per the DBC value table);
* **their calibration-valid condition is intentionally absent, and this is the one open judgement.** The camera
  moves every mount and the estimator converges in 30–60 s at a measured cost of 0.02 m of offset on straights
  (§1), so gating on calibration would withhold the assist for a minute to avoid two centimetres. But until
  now the stock cruise would not have been engaged during that warm-up anyway, so the choice was never
  exercised. With ATL on, the car steers while calibration is still converging. Worth watching on the drive
  rather than deciding in a comment.

A false alarm was removed on the way: the supervisor logged `health packet version mismatch … fields will be
wrong` for version 11, which `Panda::get_state` handles correctly and which is the version dragonpilot's panda
declares. Anyone debugging this gate would have been sent to the wrong place.

**Ported verbatim from `controlsd.py:677`:**

```python
standstill = CS.vEgo <= max(CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill  # 0.3 m/s; this car's minSteerSpeed is 0
if not standstill and CS.cruiseState.available and dpAtl > 0:
    if calStatus != CALIBRATED: pass
    elif CS.steerFaultTemporary or CS.steerFaultPermanent: pass
    elif gearShifter == reverse: pass
    else: CC.latActive = True
```

plus `alternativeExperience |= ALKA` under the same flag — both halves keyed on one switch, which is the right
shape for us too: a single `lat_always_on` that sets `alternative_experience = 17` and opens the gate, so the
two can never disagree. Our equivalents of their gates: calibration validity from `calibration/camera`, EPS
fault from `EPS_HCA_Status` (already checked as `eps_ok`), reverse from the gear signal, and `assist_max_age_s`
from the work above still applies — the point is to stop requiring `controls_allowed`, not to stop knowing
about it.

**The setpoint recompute, built 2026-08-07 and shipped off (`lat_recompute_setpoint`).** `LateralMpc` keeps
the last solved trajectory and answers `curvatureAtSpeed(v, dt)`; `LaneKeepService::recomputeSetpoint` reruns
the whole `stepFlowpilot` tail from it — vehicle model, speed-dependent angle ceiling, slew guard — so a
recomputed tick cannot command something the once-per-frame path never would. Nothing is re-solved: only the
two terms speed enters, `psi/(v·delay)` and the curvature-rate ceiling `MAX_LATERAL_JERK/v²`. `fp` only.

Two things make it safe to call at any rate, and both are pinned by tests:

* **the clamp inside `lagAdjustedCurvature` is referenced to the plan's own `kappa0`, not to the previous
  output**, so repeated calls at one speed return an identical value instead of ratcheting the command away.
  Upstream relies on the same property — they clamp against `curvatures[0]` and use `DT_MDL`, the *plan*
  period, not the tick period, which is why our recompute passes `frame_dt_s_` and not the 10 ms tick;
* **the reported setpoint is the one the PID chased.** The first version wrote its result into `last_`, which
  `updateTorqueFromAngle` had already copied and then assigned back — so the command changed while
  `control/lane_keep_debug` kept showing the once-per-frame value. Silent, and precisely the class of
  mismatch that made earlier lateral analysis wrong. It now writes into the caller's `out`, and a test
  changes the speed between vision frames and fails if the reported `steer_rad` does not move.

**Vision rate: the camera is the bound, and two attempts at it both failed — 2026-08-08.**

Measured on run `2026_08_07_19_04_05`: pipeline work 7.4 + 44.7 = **52 ms** against a **67 ms** camera
interval with **94 % of cycles dropping nothing**. A work-bound pipeline drops frames; ours does not, so the
camera sets the rate. Two changes were made on that reading and both went the wrong way on
`2026_08_08_10_47_41`:

| change | expected | measured |
|---|---|---|
| `nnapi_fp16` | inference 44.7 → ~25 ms | **64.9 ms**, +45 % |
| AE fps range: prefer a *fixed* range | camera 67 → 33 ms | **100 ms**, pinned at exactly 10 fps |

The second was a defect in the rule, not in the idea. Preferring "the highest **fixed** range at or below the
target" is wrong whenever the device's only fixed range is a low one — this phone's evidently is, and the
interval landed on a 100 ms mode, i.e. 10.00 fps. The correct order is **highest achievable upper bound
first**, and only among equals the highest lower bound: `[30,30]` beats `[15,30]` beats `[10,10]`. Fixed;
`nnapi_fp16` is back off.

**The drive still confirmed the mechanism, backwards.** Rate fell 13.46 → 10.06 Hz, and the arc setpoint step
grew **0.49 → 0.91°** median with the share above 1.67° going **7.9 % → 21.7 %**. Interval ×1.5, step ×1.9.
So the step size really is set by the frame interval, which is what the whole "raise the vision rate"
argument rested on — it just now rests on evidence instead of on an expectation.

**And a gap in instrumentation that invalidated a claim in `PREDRIVE.md`.** That file said the setpoint
recompute would be attributable because it moves `desired_swa_deg` between vision frames. It is not:
`control/lane_keep_debug` is published **once per frame**, from the vision path, while the recompute lives in
`updateTorqueFromAngle` at chassis rate and publishes only torque. So the between-frame setpoint is never
recorded and `lat_recompute_setpoint` remains unverified on the road despite having been enabled for a whole
drive. The fix is to carry the setpoint on `controls/steer`, which is published at the tick rate.

**Their σ gate measured on their own log, 2026-08-07 — and it is not the shape the comparison assumed.**
`laneLineStds` for the two host lines over 9 473 frames of this route: median **0.086**, p10 0.034, p90
**1.165**. So the distribution is not a spread around a threshold, it is two populations — confident frames
far below their 0.15 knee, and a tail far above their 0.30 cut-off. Consequences:

* **their gate acts as a switch, not a ramp.** Under their own 0.15 → 0.30 the median line weight is 1.00 and
  **39 % of frames are zeroed outright**. Their `d_prob` after the σ modifier has a median of 0.979 but sits
  below 0.05 in **37 %** of frames — i.e. on this road dragonpilot drives on the model plan alone about a
  third of the time. That is the discrete laneless behaviour documented in `DIFF_FROM_DP_RU.md` arriving
  through the σ gate rather than through the hysteresis switch;
* **under our ramp the same numbers zero only 5 % of frames**, so our gate really is much softer — the
  earlier «at σ = 0.25 their weight is 0.33, ours 1.00» was right in direction and understated in degree;
* **but the scales are not comparable, and this is the finding.** Their median 0.086 is a *scalar the model
  emits*; our 0.19 straight / 0.60–0.93 on arcs is a *median of per-point `y_std` over 5–20 m*. Both the
  definition and the camera differ. Moving our ramp to their 0.15/0.30 with our σ distribution would zero the
  majority of our frames, not 39 % of them. So #23 keeps its shape: establish what our own quantity
  corresponds to first, and the candidate to compare against is the controller mirror's 0.13, not the
  per-point median's 0.58.

**Not to be done, restated here because both are tempting under a «be like comma» framing:** raising
`STEER_DELTA_UP` (panda enforces the same 4 cNm per frame for them, so it is a safety-firmware edit and not a
setting), and copying their σ thresholds before #23 says the two σ are the same quantity.

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
| ~~arc offset: `path_lane_blend_scale` 0.6, σ thresholds 0.3/1.5, width up to 4.6~~ | **verified 2026-08-04 night** (`2026_08_04_21_00_18`, 23.5 min, first run with control actually engaged — `controls_allowed` 44.7 %): right arc −0.71 → **−0.00**, left +0.51 → **+0.30**, straight −0.04. Beat the expectation (+0.45 / −0.54). Setpoint offset on arcs is now +0.02 / −0.10 against +0.15 / −0.42, i.e. blending did its job. See `BENCHMARK_COMMA2.md` §2 |
| `center_force` disabled (0.0) | at real command rate (12.5 Hz, not 20 as in replay) the term does nothing: offset magnitude 0.21 → 0.19 on right arc and 0.26 vs 0.25 on left. It also eats stability margin — at 1.2 the left arc diverges (0.81 m, p95 2.95). Comma-two logs show tracking error ~0 without centering because the loop runs at 100 Hz. Revisit only after speeding up the loop |
| ~~`intrinsics_prior` fx 951 → 993.4 / 995.2~~ | **verified 2026-08-04**: camera odometry scale 0.844 → **0.888** vs expected +4.5 %. Remaining 11 % is not explained by focal length — model domain gap. 2026-08-04 bag |
| ~~CAN receive 50 → 10 ms~~ | **verified 2026-08-04**: `vehicle/state` 10 ms (p99 18, max 88), `controls/steer` 10 ms, panda callback 1.1 ms, no drops. But a vision tail appeared — see section 2 |
| **σ window 5–40 → 5–20 m** | the fix for the right-arc collapse, and the one lateral change for the next drive. Predicted: blending weight on arcs 0.19–0.20 → 0.24–0.26, share of frames with blending off 33 %/32 % → 22 %/25 %. Measure with `bag_arc_offset.py` and watch the `d` and `σ worst` columns, not just the totals. Raising `lane_std_bad_m` was swept and rejected (see Done 08-06) |
| **cruise-button actuation after the fix** | four plan defects and the actuator ceiling are fixed with tests, and replay over run 08-06 gives 1.5 tips/min against 73.1 — but no drive yet. Enable deliberately (`--set vehicle.cruise_buttons=true`), not by default. Watch: tips per minute in single digits, `brake_needed` share, and that the set speed never rises above where the driver put it |
| ~~calibration prior `rpy_deg`~~ | **superseded 2026-08-06**: the camera moves every mount, and the learned yaw went +1.67° (08-04) → +0.10° (08-06), so a prior from the previous drive is no better than the default. Convergence takes 30–60 s and costs 0.02 m of offset on straights, so no prior edit and no calibration gate is needed. Config carries `pitch −1.8 / yaw +0.5`. What the finding does mean is that **plan offsets are not comparable across runs with different learned yaw**, since yaw enters the input warp |
| **left-arc tracking error +0.29 m** | what remains of the arc problem. The wheel *does* reach the commanded angle (+13.8° asked → +13.5° achieved), so the command is too small: understeer compensation is too weak. Step 0 no longer blocks this (see `BENCHMARK_COMMA2.md`) — try `tire_stiffness_factor` 0.50 then 0.40 and re-measure on the same road |
| **torque saturation on right arcs 76 %** | and 80 % within one 12.3 s episode at R = 130 m. This is the MQB HCA ceiling, not a coefficient: reduce requested curvature, not gains |
| ~~FCW/AEB | no recordings with lead vehicle data yet~~ | **measured 2026-08-04 night**: 5 forward (4 FCW + 1 AEB) and 7 lane warnings, all false positives. Forward ones were all stop-and-go (median 4.7 m/s, max 8.5) → speed gate 3 → 8 m/s. Lane ones were 82 % with our own steering engaged at \|cte\| 0.54 m → LDW now suppressed while we steer. Both fixed with tests; `SAFETY_WARN.md` |
| **vision rate quantised by camera period** | pipeline work is 59 ms (prep 9 + infer 45.5) against a 44 ms camera period, so it can never take every frame and lands on exactly half the camera rate: 11.3 Hz, 2.02 camera frames per processed frame. The pipeline itself has no idle wait (one-slot latest frame, picks up immediately). `TARGET_FPS` raised 20 → 30 so the quantum shrinks to 33 ms; ceiling at any camera rate is ~15 Hz, and upstream's 20 Hz needs prep+infer under one camera period |
| **LKA hands the wheel back on a turn signal** | new gate `lka_suppress_on_blinker` (on by default, resume delay 1 s): while a blinker is on we stop steering — there is no lane-change planner, so holding the current lane fights the driver. Signal verified present on the night run (left 7 episodes, right 11). Deliberately not keyed on `steering_pressed`: 520 episodes in 23.5 min with a 30 ms median would drop the assist every few seconds. Two tests; **not verified on road** |
| **setpoint recompute (`lat_recompute_setpoint`)** | ships `false`; `fp` only. Changes the command *between* vision frames, so the actuator sees a ramp where it used to see a 78 ms step. Measurable offline first with `rlog_lat_diff.py --recompute-setpoint`, which is the intended order: the harness reports our setpoint against theirs at their own 100 Hz cadence, so the 10–15 m/s band (4.2° rms) is where the number should move |
| **always-on lateral (`lat_always_on`)** | ships `false`; enable deliberately for one drive (`--set vehicle.lat_always_on=true`). This is the largest behaviour change in the whole backlog: the car will steer where the stock cruise is not engaged, i.e. below ~30 km/h, on brake, and during the camera-calibration warm-up. Watch, in order: `panda/health.tx_blocked` stays at zero (if it climbs, bit 16 is not ALKA on this unit and the assist must go back off immediately), `lat_actuation_allowed` tracks `cruiseAvailable` rather than `controls_allowed`, and the share of frames with real torque at 5–12 m/s rises from the measured 2.6–18.3 % |
| **assist gate (`lat_require_assist`)** | the largest behaviour change on this list: on a drive like 08-06 it withdraws the command in ~70 % of frames, which is correct — the panda was discarding them anyway — but it must be seen on the road. Watch `assist_allowed` against `panda/health.controls_allowed` (they should agree), that `no_assist` appears and clears rather than sticking, and that the first command after the assist returns is no longer a step. This is also the row that makes the drive worth doing before #24: it establishes what the honest baseline is |
| `STEERING LIMIT` indication | appears when torque is actually at the ceiling |

## 2. Perception

**Night-vs-night σ, matched subset** (v 8–15 m/s, straight, both lines): 0.344 → **0.273** median
(fx 951 → 993.4), but p90 (0.864 → 0.878) and the share of σ > 0.8 (12.2 % both) are unchanged. The
middle of the distribution improved by 21 %, the tail did not move. The 0.226 measured on the daytime run
was therefore mostly daylight, not focal length.

**Vision stalls do not reproduce at night.** Night run: `frame_dt` median 84.4 ms, p99 110.7, max 119,
and the new staleness gate never fired (`stale` status: zero frames). Daytime run had p99 189.7 and max
388.8 with a 75 s hole. Heat remains the leading explanation.

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

**And it does not answer torque either, which cost us a wrong conclusion — 2026-08-07.** An open-loop replay
feeds the controller a measured angle that *someone else's* torque produced, so an integrator accumulates an
error our command cannot influence. In `rlog_lat_diff.py` this inflated our torque from a median of 78 cNm to
**229** against upstream's 69, and the ceiling share from 21.7 % to 35.1 %, while the setpoint the same run
computed agreed with theirs to 0.08° at a correlation of 0.965. The rule this establishes: **in an open-loop
replay, compare quantities upstream of the integrator, or turn it off.** Curvature and setpoint angle are
safe; torque needs `--no-integrator`; steady offset needs a closed loop, which this section already says the
simulator cannot rank controllers with.

Vision delay (90 ms) and reference noise (0.15 m at 20 m) are set and measured from runs, but
**replay does not reproduce controller ranking**: PP holds lane better than `fp` at any
degradation level tried, while on the road it is the opposite. Plan jumps (`--vision-jump-hz`,
p95 |Δy| ≈ 1.6 m on runs) hit `fp`, not PP, so disabled by default.

So for controller choice replay is unsuitable — only as regression on a fixed
controller. To change that, need a perception error model from frame-by-frame run analysis
(model plan vs lane-marking path), not inter-frame spread. On available runs
lane markings are too poor for such comparison.

## 4. Lateral Control: Open Items

**Consolidated plan — `BENCHMARK_COMMA2.md`.** Individual items below; work order and expected effect
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
tables — arc-offset bags.

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
current turn direction. **Shipped `center_force_gain = 0.0`** (disabled at ~12.5 Hz command rate). Replay once used 0.4.

**Single lane does not hold reference, and this is intentional.** We require both host lines for `anchored`,
flowpilot needs only the better one (`lane_trust = clamp(1.2·max(l_vis, r_vis)^0.5, 0, 1)`). Did not relax yet:
with one line center is invented from assumed width, and `laneLinesToPath` documents where that ends —
drift into parked cars or shoulder. Measured need was partly closed by σ threshold: with
`lane_std_bad_m = 1.5` share of arc frames without reference drops from 26 % (left) and 67 % (right) to
23 % and 9 %.

**The road-edge backup flowpilot uses is not available to us — measured 2026-08-06.** Their fallback is
`roadEdges` with σ correction; ours are recorded now and are far too noisy to substitute for the lines
they would replace: σ **1.17 m median over 5–20 m against 0.14 for the lane lines**, 7.5× worse, only
68 % of frames under the 1.5 m cut-off, and the two edges bracket 12.8 m — a corridor, not our
carriageway. So relaxing the two-line requirement has no cheap backup, and would have to stand on the
single line's own σ plus an assumed width. Not attempted.

**Horizon change not needed.** flowpilot keeps `N = 32` (10 s), we took dragonpilot `N = 16`
(2.5 s), but grid `T_IDXS` is quadratic: 11 of our 17 nodes in the first second — same as
flowpilot. Lengthening only adds far nodes and at equal weights dilutes the near zone,
and costs roughly 4× more (we use GD instead of acados). Details — arc-offset bags.

**HCA torque hits ±300 cNm ceiling in 34–41 % of arc frames** (19 % on straight). While torque
is saturated, there is no feedback at all. This is MQB HCA limit, not fixable in config; reduce
requested curvature (item 1: lane weight), not coefficients.

**Understeer compensation is low at speed.** From steady arcs measured transfer
κ_fact/κ_kin: 0.97 at 6–9 m/s (model expects 0.96), 0.80 at 12–15 (0.87), 0.54 at 21–26 (0.69).
On fast arcs command is 14–29 % low and pulls car **outside**. This is a **separate** task from
cutting inside on urban arcs: stronger compensation (`tire_stiffness_factor` 0.50) in replay on urban arc
made it worse (−0.44 → −0.52 m). Touch only for highway run and measure on that.

## 5. Pipeline speed: measured budget and what is cheap

Measured on run `2026_08_04_21_00_18` (night, 23.5 min): capture → steering command on CAN is **93 ms**
(83 ms to the command plus ~10 ms panda TX), and the vision loop runs at **11.3 Hz**.

| stage | now | cheap to improve? |
|---|---|---|
| frame prep (warp) | 9 ms median, **mean 15.3, p95 37.6** | jitter yes, magnitude no |
| supercombo inference | **45.5 ms** (p95 55.3) | yes — FP16, see below |
| Java → C++ → command | 7 ms (of which ~5 is the 10 ms `zmq_bridge` poll) | marginal |
| panda CAN TX | ~10 ms (own 10 ms timer, mean phase 5) | no — HCA needs strict 100 Hz cadence |
| wait for next camera frame | ~20 ms | yes — camera fps |

Everything except inference and the frame wait adds up to 26 ms; there is no slack there.

**Rate is quantised by the camera period, not by the work.** Camera ran at 44 ms (22.7 Hz) while a cycle
takes 59 ms of work, so the pipeline can only take every second frame: 2.02 camera frames per processed
one, 11.3 Hz. The pipeline itself never idles by design (one-slot latest frame, immediate pickup —
`VisionPipeline.drainYuvLatest`).

Ordered by effort:

1. **`TARGET_FPS` 20 → 30** — done. Quantum 44 → 33 ms, wait ~29 → ~7. Expect **13–15 Hz**. Verify on
   road; if the gain is smaller, the residual is the camera→`submitYuv` delivery path, not the wait
   (see the instrumentation item below);
2. ~~**NNAPI FP16 for supercombo**~~ — **tried on the road 2026-08-08 and reverted: it is slower.** Inference
   went **44.7 → 64.9 ms** median (p95 53.2 → 75.6), i.e. **+45 % where −45 % was expected**. On this SoC the
   `USE_FP16` flag does not accelerate; the plausible reason is that NNAPI partitions the graph differently or
   inserts precision conversions and part of it lands on a less suitable accelerator, but that is a guess and
   the only honest next step is an on-device NNAPI profile rather than another log.

   Worth separating what was and was not verified beforehand. `bag_fp16_ab.py` checked **accuracy** — lane
   centre moved 0.027 m, plan offset 0.037 m, line probabilities rose, the σ tail shrank — and that result
   stands. It simply never answered the question about **speed**, and speed was the only reason to enable it.
   An offline A/B that measures the wrong quantity reads exactly like one that measures the right one.

   Original reasoning kept:
   the pattern already existed in the tree —
   `SupercomboOnnxRunner.java:103` called `opts.addNnapi()` with no flags, while
   `TrafficYoloRunner.java:206` already has `addNnapi(EnumSet.of(NNAPIFlags.USE_FP16))` with fallback to
   plain NNAPI and then CPU. If it gives the usual 1.5–2× on this SoC: inference 45.5 → 25–30 ms, cycle
   ~40 ms → **20–22 Hz at ~65–70 ms to CAN**, i.e. upstream's planner rate. Risk: FP16 changes model
   output; check offline on recorded frames the way `bag_intrinsics_ab.py` does (swap the session,
   compare σ and plan on the same frames) — no drive needed;
3. **Frame prep jitter** (mean 15.3 vs median 9, p95 37.6) — a few ms of median, much more of the tail,
   and the tail is exactly the frames where the reference ages past 150 ms;
4. **`zmq_bridge` poll 10 → 5 ms** — saves 2.5 ms mean for 200 extra wakeups/s. Noise next to
   inference; do it only if something else touches that service;
5. ~~**Instrument frame arrival**~~ — **done**: `VisionPipeline` records the moment a frame reaches
   `submitYuv` and `LaneLines` carries it alongside the capture stamp, so camera-to-app delivery is separable
   from inference. It was needed twice — to attribute the ~20 ms wait, and to tell camera starvation from slow
   inference in the daytime stalls (see section 2) — and **neither question is answered yet**, because that
   needs a run with the field recorded plus `logcat`.

**Ceiling with minimal effort: ~20 Hz and ~70 ms capture→CAN.** Beyond that it is model surgery
(smaller or quantised net, DSP), which is not minimal.

**Not a bottleneck, verified:** middleware never delays a message on a timer — publishing wakes the
subscriber thread immediately (`cv.notify_one()`), and only the two boundaries with the outside world are
polled (ZMQ ingress, CAN egress). Timer dt is exactly 10.00 ms mean over 143 700 firings; worst dt 44.8
(panda, likely a USB stall) and 21.8 (zmq bridge).

## 5a. Feeding the localizer into the planners

Two related items, and they are related through the same dependency: **`paramsd` is a consumer of the
localizer, not an alternative to it.** In `dragonpilot/selfdrive/locationd/paramsd.py` the filter takes
its yaw rate and its **road roll** from `liveLocationKalman` — the fused output — and only the steering
angle and `vEgo` from `carState`. So "use the localizer in the planners" and "port `paramsd`" are one
chain, not two choices.

### What upstream actually does with speed, measured against what we do

Worth stating plainly because it is the opposite of what one might assume: **upstream does not fuse
GPS or IMU into the speed the controllers use.** `CarInterfaceBase.update_speed_kf`
(`flowpilot/selfdrive/car/interfaces.py:346`) runs a fixed-gain two-state `KF1D` over the four-wheel
average — `A = [[1, dt], [0, 1]]`, `C = [1, 0]`, `K = [0.174, 1.659]` at 100 Hz — and both `vEgo` and
`aEgo` come out of it. The localizer is used for orientation and for `paramsd`, not for `vEgo`.

We measured why that choice is defensible, and where it leaves a real gap
(`bag_speed_sources.py`, GNSS Doppler as the reference, two runs):

| quantity | run 08-06 | run 08-04 |
|---|---|---|
| CAN wheel / GNSS Doppler, scale | **1.0117** | **1.0120** |
| residual after removing the scale, median | 0.101 m/s | 0.066 m/s |
| `localization/pose.v` / GNSS, scale | 1.0109 | 1.0112 |

Two conclusions:

* **wheel speed reads 1.2 % high, and the scale is flat across speed bands** (1.005–1.022, no trend) —
  the signature of a wheel-radius constant, not slip or a sensor fault. Once that constant is removed
  the residual is 0.07–0.10 m/s, which is about what Doppler noise and the 1 Hz sampling explain. So
  there is little left for fusion to win on *speed* — the win is one calibration number.
  Knob exists now (`wheel_speed_factor`, default 1.0, correction 0.988), queued as its own change;
* **the current filter adds nothing to the scale**, and the reason is more interesting than the one first
  written here. `updateGpsVel` **is** called (`online_localizer.cpp`, inside the course-valid branch) —
  an earlier note in this document claiming otherwise was wrong. What defeated it was two things, both
  now fixed and both measured:
  * `predict` did `state_(3) = v_measured` on every 10 ms tick, so a GPS velocity update at 0.2 s
    intervals was assigned over about twenty times before the next one arrived. Speed is a state now
    (`VehicleEKF::setSpeedIsAState`), and the correction survives the next tick;
  * the update was given `R = 1.0`, i.e. an assumed 1 m/s of Doppler noise, which made it irrelevant even
    in the tick it arrived in. Measured, the residual between Doppler and scale-corrected wheel speed is
    0.066–0.101 m/s on two runs, so `gps_vel_R` is now `0.1²`.

  **And it still does not learn the scale**, which is the useful finding. The wheel measurement arrives
  twenty times more often at the same assumed noise, so between GPS samples it drags the estimate back to
  the biased value — pinned by a test (`ButTheWheelSpeedDragsItStraightBack`). No pair of unbiased-noise
  assumptions can separate a constant bias on the frequent measurement from the truth. That needs the
  **scale itself as a state**, which is exactly what `paramsd` does for the vehicle parameters. So task
  #18 as originally framed was the wrong shape; the structural work is done, and the scale belongs in
  #20.

Why it matters at all, given the numbers are small: speed enters the understeer term squared,
`κ = δ / (L·(1 + K·v²))`, and it sets the curvature speed limit `sqrt(a_lat/κ)`. A 1.2 % bias is
consistent, in the same direction, in everything lateral. It also revises a documented number — the
model metric scale 0.888 was measured against wheel speed, so against ground truth it is 0.899.

**Acceleration was worse than a bias, it was noise — fixed 2026-08-06.** `a_ego` was a finite difference
of the wheel average over the CAN interval: p5/p95 of ±3.8 m/s², extremes −61.5 and +74.5, RMS step
4.16 m/s² between consecutive samples. Nothing read it, which is the only reason it did no harm. Now a
two-state filter with `dt` from real timestamps (`utils/speed_filter.h`, eight tests): on the same
recorded data p5/p95 ±0.6, extremes −4.0 and +3.1, RMS step **0.062**, and speed differs from raw by
0.010 m/s median. `a_ego` is a usable signal for the first time.

### What is still open

* ~~make GNSS velocity a measurement~~ — **done, and it was not enough**: see above. Speed is a state,
  the Doppler noise is measured rather than assumed, and the scale is still not identifiable without a
  scale state. Fold that into the `paramsd` work;
* **give the planners a choice of speed source** and A/B it. Right now `LaneKeepService` and
  `LongPlanService` both read `chassis_.speed_mps`. The filtered CAN speed is probably right for
  control — it is what upstream uses, and it has no GNSS latency — but that should be measured, not
  assumed, and the harness for it is `bag_speed_sources.py` plus a replay;
* ~~road roll has no source at all~~ — **built 2026-08-06**, see below. What remains open is grade
  (longitudinal), which needs the same treatment with `f_x` and is not yet done.

### `paramsd`: ported, measured, and correctly left switched off

**Done 2026-08-06.** `utils/params_learner.h`, twelve tests, published on `localization/pose`, replayable
offline with `bag_params_learner.py` — and both flags ship `false`, because the replay said so. The result
is worth more than a working feature would have been, since two of the three premises that motivated the
task turned out to be wrong, and one of them was a premise about the estimator built the day before.

Measured on run 2026_08_06_00_36_42, 98 684 accepted samples:

* **the speed dependence is real, and it is not the constant's job.** `κ_fact/κ_kin` from the *actual*
  steering angle falls 0.89 → 0.84 → 0.84 → 0.65 → 0.58 → 0.46 across 5–9, 9–12, 12–16, 16–21, 21–26 and
  26–32 m/s, which reproduces the earlier 0.97 / 0.80 / 0.54 and is why this task existed. But apply the
  vehicle model with the shipped 0.64 and the residual `κ_fact/κ_model` is 0.95 / 0.94 / 0.99 / 0.87 / 0.89
  / 0.80 — flat around 0.90, no monotone trend. The `1/(1 − slip·v²)` denominator *is* the speed dependence.
  "One constant cannot represent this" confused the constant with the model it sits inside;
* **what is left is a flat ~10 % shortfall, and it is degenerate.** Stiffness and steer ratio both scale
  predicted curvature, and their speed signatures differ too little over one drive's range for a yaw-rate
  measurement to separate them. With the ratio's prior at 0.5 the filter slid along the ridge to 14.06 and
  dragged stiffness to 0.374, then predicted the yaw rate **3 % worse** than the shipped constants while
  `valid()` returned true. Tightening the prior to 0.1 — a rack's ratio is a mechanical fact known to a few
  percent, and that knowledge belongs in the prior — gives 0.517 ± 0.026, still drifting 0.462 → 0.368
  across the four quarters of the drive, for a 0 % gain;
* **the alternative is a config edit, and it is not worth a road slot.** The best single
  `tire_stiffness_factor` on this drive is 0.55, worth 4.1 % of yaw-rate residual against 0.64, and the
  curve is flat: 0.50 → 0.00636, 0.55 → 0.00625, 0.64 → 0.00652 rad/s. The shipped value is near-optimal;
* **the road-bank input is an order of magnitude too coarse, which is a negative result about §6's roll
  estimator.** The term is `g·sin(roll)/v`: at 15 m/s one degree of bank is 0.0114 rad/s, while the whole
  yaw-rate residual of the flat model is 0.0065 rad/s. Our bank estimate is good to 0.65° at its floor.
  Feeding it in raised the residual 0.0065 → 0.0104 rad/s, a 60 % degradation, so `params_use_roll` ships
  `false`. `RoadRollEstimator` measures what it claims to; what it measures is too coarse for *this*
  consumer, and fixing that needs an IMU on the chassis rather than a phone on glass. The synthetic
  banked-road tests still exercise the term, because on a road with a *known* bank it is correct and they are
  what prove its sign;
* **two defects the replay caught that no synthetic test could have.** The port negated the yaw rate on the
  way in (ISO → z-down, right) but never applied this car's `vehicle.steer_sign`, which the controller has
  always applied. On real data positive CAN angle is a *left* turn — measured ISO yaw against the kinematic
  prediction, slope +0.824 at correlation 0.987 over 28 636 cornering samples — so prediction opposed
  measurement and the filter ran to whichever bounds shrink the predicted magnitude: stiffness on its 0.200
  floor, ratio on its 20.0 ceiling, identical in all four quarters. And `valid()` did not object to the
  saturated ratio, because it checked the stiffness bounds and not the ratio's — saturation is the failure
  mode that most resembles convergence, since a state that has stopped moving has a small sigma. Both fixed,
  both now have a test.

**What comma's own learner says on this car, from the captured rlogs** (`liveParameters`, 19.6 Hz, `valid`
true 100 % of a 25-minute drive):

| parameter | median | its own σ | movement across the drive |
|---|---|---|---|
| `steerRatio` | 16.12 | 0.099 | 15.74 → 16.43 = **7σ** |
| `stiffnessFactor` | 1.247 | 0.0066 | 1.187 → 1.318 = **20σ** |
| `angleOffsetDeg` (fast) | −0.017° | 0.011 | −0.56 → +0.26 |
| `angleOffsetAverageDeg` (slow) | +0.010° | 0.0037 | −0.27 → +0.10 |
| `roll` | −0.98° | — | spread ±0.91° |

Three things follow, and they matter more than the numbers.

**Their filter's stated uncertainty is 7–20× smaller than its own movement.** `stiffnessFactor` claims a σ of
0.0066 while walking 0.13 over the drive, and reports `valid` throughout. That is upstream's own log
confirming the defect this port had to design around: "observe the state with its own current value at high
noise" shrinks the covariance without any information arriving, so both the reported σ and the validity flag
stop meaning anything. Our covariance ceiling exists precisely so this cannot happen — see
`stiffness_std_max` in `utils/params_learner.h`.

**The baseline is comparable and our constant is below even their starting point.** Their `carParams` carries
`tireStiffnessFront = 184691`, and our `slipFactor` at `tire_stiffness_factor = 1.0` computes 184 731 from the
same Civic-derived construction — the same quantity, so the numbers can be compared directly. Their default
is 1.0, their learner settles near 1.25, and we ship 0.64. Meanwhile our own yaw-residual fit on our own bag
put the optimum at 0.55 and called 0.64 near-optimal. Two learners and one direct fit on one car giving 0.55,
0.64 and 1.25 is not a tuning disagreement; it is the identifiability finding of §5a stated a third way.

**`liveTorqueParameters` is computed on this car but not used** — `lateralTuning` is `pid`, so `torqued`'s
output goes nowhere. Kept here because the numbers are the ones we would need if the lateral stack ever moves
to torque control: `latAccelFactorRaw` 0.92, `latAccelOffsetRaw` 0.075, and **`frictionCoefficientRaw`
0.192** — nineteen percent of the full ±300 cNm range, about 57 cNm, is pure stiction compensation on this
car. Our angle PID has no friction term at all; its only feed-forward is `k_f · SWA · v²`, which measures
3 cNm at 6 m/s and 16 cNm at 25 m/s. Also from their `carParams`: `maxLateralAccel = 1.58` m/s²,
`steerActuatorDelay = 0.1`, `minSteerSpeed = 0`, `wheelSpeedFactor = 1.0`, mass 1533, wheelbase 2.62,
`steerRatio` 15.6.

**What the two flags are for.** `localization.learn_vehicle_params` runs the estimator;
`lane_keep.use_learned_params` lets the controller read it. Separate on purpose — the learner can publish
for a whole drive while the controller keeps its constants, which is how a learned value earns the right to
be used. One flag for both would make the first drive that tests the estimator also the first drive that
trusts it.

The original reasoning, kept because the conclusion changed and the reasoning is why:

Full state and observation breakdown: `docs/PARAMSD.md`. The gates worth copying verbatim from
`paramsd.py`: active only above 1 m/s and below 45° of wheel, yaw-rate std in (0, 10) and |yaw rate| < 1
rad/s, roll std bounded, and the self-observations of stiffness and steer ratio that keep their
uncertainty from growing without bound on long straights. Their `rednose`/sympy codegen is not portable;
the filter itself is roughly 150 lines of Eigen.

**Order of work, given the dependencies:** GNSS velocity into the EKF → an orientation/roll estimate →
`paramsd`. Doing `paramsd` first would leave its roll input pinned at zero with a 10° std, which is what
`paramsd.py` itself falls back to when the localizer is untrustworthy — usable, but it gives up the one
thing that separates road bank from vehicle understeer.

That order was followed and it was still the right order — but the roll estimate it insisted on turned out
to be unusable *for this consumer*, and only building it revealed that. Worth keeping as a lesson about
dependency arguments: "A needs B" is a claim about interfaces, and it says nothing about whether B is
accurate enough for A. That question has an answer only after B exists and is measured against A's error
budget. Here the budget was 0.0065 rad/s and the input carried 0.0114.

## 6. Localization

**Fixed 2026-08-06: yaw rate is a state now, not a value overwritten every step.**

The defect: `VehicleEKF::predict` advanced heading with `v·tan(δ)/L` and then **overwrote** the yaw-rate
state with the same prediction, so whatever the gyro had taught the filter was discarded. Measurement
reached heading only through cross-covariance in one step, weight `dt·(1 − K₄) ≈ 0.12·dt` on our
`Q₄₄ = 0.05²` and `R_imu = 0.02²` — heading came out 0.88 · bicycle + 0.12 · measured.

That would be fine if the bicycle model were right. It is not: with measured understeer
(κ_fact/κ_kin = 0.54 at 22 m/s) it over-turns by 1.85×, so heading rotated **1.75×** faster than truth.
On an arc at ω = 0.15 rad/s for 30 s that is 450° against 258°. GPS heading masked it on the road
(`updateGpsYaw` hard-snaps above 0.5 rad), which is why runs never showed it; without GPS it drifted.

Now (`setYawRateIsAState`, on by default, off restores the old behaviour exactly for A/B): yaw rate is a
state with its own random walk, heading integrates the state, and the bicycle model enters as a *weak*
measurement with `R_model = 0.15²` against the gyro's `0.02²` — weak because that is roughly how far
apart they are at 20 m/s and 10° of wheel. The gyro is trusted; the model fills gaps, so a failed IMU
degrades to the model instead of freezing the heading. Model updates are counted separately
(`model_update_count`) so diagnostics still say which sensor produced the heading.

Six tests in `tests/test_vehicle_ekf.cpp` — there were none before — pinning the arc case (new filter
lands on the gyro within 10 %, old one on the model within 15 %), the gyro-less fallback, the
straight-line hold, that a measurement survives the next predict (old: heading barely moved), the
counters, and that a GPS course fix still pulls heading back.

**Blast radius is small and worth knowing:** nothing in the control path reads `localization/pose` — the
only subscriber is `internal_subscriber` for the debug egress, and the lateral chain takes yaw rate
straight from `vehicle/chassis`. So this changes the recorded pose and mapmatch input, not steering.

Learned vehicle parameters (`docs/PARAMSD.md`) would improve the prediction itself where measurements
are absent: before the GPS heading seed, and when the IMU is invalid but CAN still reports steering.

`mapmatch` track was never affected: `buildTrack` integrates measured yaw rate directly.

`mapmatch` track is **not** affected: `buildTrack` integrates measured yaw rate directly,
steer angle does not participate.

## 6a. Course (`docs/book/`)

**Snippets are executed, not trusted.** `docs/book/check_snippets.py` runs every ```python block — 51 of
them, all passing. The contract it enforces is **a chapter is one program, read top to bottom**: blocks
share a namespace within a chapter, each chapter starts clean, and `--isolated` lists the blocks that would
not survive being copied out alone. Ten of them would not, all in chapters that define shared constants
once at the top — which is fine for a reader going in order and a trap for one arriving from a search.

**Chapters are being restructured as progressive builds** rather than descriptions. The pattern, set by
`Localization/Overview.md`: numbered steps where each adds one sensor or one idea, states what it fixes,
and ends on what it cannot do — which motivates the next step. Every step corresponds to a config switch,
so the failure the text predicts can be reproduced on the vehicle.

| chapter | state |
|---|---|
| `Localization/Overview.md` | **done** — steps 1–5, from dead reckoning to what `paramsd` needs. Includes a 20-line runnable EKF where one flag reproduces the 38°-in-5-seconds heading defect |
| `Control/Overview.md` | **done** — the four control chapters framed as "each exists because the previous one fails at *this measured number*", carried by the understeer table (0.97 / 0.80 / 0.54) |
| `Vision/Overview.md` | **done** — the four conditions that must hold before a controller can follow anything, in order of how often they fail |
| `Architecture/Overview.md` | **done** — the order the system was actually built in, which is not the order of the layer table |
| `Control/BicycleModel.md` | **done** — ends by measuring the model against the car (exact below 9 m/s, 0.54 at 21–26, and below the textbook gradient at every speed) and converting the shortfall into metres of open-loop drift |
| `Control/PurePursuit.md` | **done** — a runnable sweep showing the *information* limit: at an 8–12 m look-ahead three paths that diverge later produce identical commands, spread 0.00°. Distinct from the understeer argument, and the reason a horizon exists |
| `Safety/Warnings.md` | **done** — three rounds of being wrong: desired acceleration as a hazard proxy (an empty road and an empty bend both demand −3 m/s²), `lead1`/`lead2` as targets that are predictions, and LDW warning about the assistant's own tracking error in 82 % of frames |
| `Vision/Supercombo.md` | **done** — five steps: warp into the model's geometry (handing off to the calibration chapter), run it and decide the precision (with the fp16 evidence), read the output vector (the two parsing traps), the rate you actually get (camera-period quantisation and the one-slot drop policy), what control consumes. Stale metrics replaced by the measured 7 / 45.6 / 54 / 79 ms and 13.24 Hz |
| `Calibration/IntrinsicsAndWarp.md` | **new chapter** — the three things "calibrate the camera" means, as steps: intrinsics from a chessboard (and why the field of view matching the EON's 65.2° mattered more than the absolute focal), why the principal point is deliberately left at frame centre (its offset is 1.04° of yaw, which the online estimator already absorbs — correcting both doubled σ), the vanishing-point estimator with its three refusals and its sign convention, the warp `K·V·R·(K_med·V)⁻¹` and the 27 px of sampling shift a remount causes, and what stays unobservable (roll, height, the model's metric scale) |
| `Calibration/Overview.md`, `Latency/`, `Logging/` | examples added; `Overview` now points at the chapter above |
| `Control/VehicleModel.md` | **done** — the calibration detective story: our 0.54 against comma's learned 1.319 on the same car, resolved by a third sensor (gyro/ESP 1.017, camera/ESP 0.849) rather than by argument, then what 0.64 → 0.50 actually bought (3–9 % more angle, a quarter of the tracking error), then why a constant cannot be the end. Latency table updated to the measured 54 / 79 / ~89 ms and the transport-versus-dynamics reason the lookahead exceeds the sum |
| `Control/MPC_and_FP.md` | **done** — already built as seven steps for each controller; what it lacked was an ending. New section on the four kinds of limit a horizon does *not* remove: structural (a steady offset is nearly free in the cost function, because over the first four nodes the car cannot move sideways by more than 3.7 cm), a non-result (N=16 already puts 11 of 17 nodes inside the first second, same as N=32 — lengthening dilutes), upstream (the plan's own ±0.33 m arc bias, coupled to camera calibration, which is why the curvature coefficient was not reproducible between runs), and physical (65 % torque saturation, where feedback stops existing) |
| `Architecture/Middleware.md` | **done** — a measured section answering "is the bus the bottleneck": publish calls `cv.notify_one()` so a subscriber is runnable at once, only the two boundaries with the outside world poll, and timer dt was 10.00 ms mean over 143 700 firings (worst 44.8, a USB stall). With a runnable comparison of polling against notification that also shows how picking a commensurate period flatters polling. Plus the queue-policy asymmetry: the bus drops the newest, `VisionPipeline` drops the oldest, and both are right |
| `Architecture/Pipeline.md` | **done** — a runnable trace of one frame with an accumulating clock, arranged to land on the two cumulative figures `latency.py` reports (54 ms to model output, 79 to the command, 89 to the wire, 1.96 m of travel at 22 m/s). Plus "what happens between frames": 7.6 actuator ticks per vision frame, why 32 % of `controls/steer` are republishes on an older reference, and why "the command is fresh" and "the picture is fresh" are different claims — the distinction the 250 ms HCA timeout failed to make |
| `Architecture/JavaLayer.md` | has examples; not yet restructured |

**The course exists in Russian too**, at `/ru/`, with an **ENG/RU** button in the navbar that maps to the same
page in the other language. All 23 chapters are translated; `docs/build_book.sh` builds both into `docs/_site`
and verifies that every one of the 50 page transitions resolves, so the button cannot land on a 404.

Two source trees rather than gettext — `.po` files are worse than markdown for prose full of formulas,
tables and code — with the drift that invites caught mechanically. `book/sync_translation.py` requires the
same pages, the same heading structure and **byte-identical code blocks**, and `--fix` restores a drifted
block from the English original, because code is never translated and copying it back is a repair rather than
a judgement call. Writing the translations against that checker found two real defects in the English
chapters as a side effect: `Safety/Warnings` still documented the pre-fix 3 m/s speed gate, and
`Localization/Overview` claimed `updateGpsVel` was never called in one section while explaining in another
that it is. The heading check itself was also wrong — it counted `#` comments inside code blocks as headings.

**Road bank is estimated now** — the input `paramsd` needs to tell a banked road from an understeering car,
and the last hard blocker on it. Measured first (`bag_road_roll.py`), built second.

The relation is one line: in the vehicle frame with the road banked by φ the accelerometer reads
`f_y = a_y − g·sin(φ)` with `a_y = −v·yaw_rate`, so `sin(φ) = (a_y − f_y)/g`. Everything interesting is
around it:

* **that minus caught a sign trap worth remembering.** `chassis.yaw_rate` is decoded exactly as flowpilot
  decodes `ESP_02`, i.e. openpilot's ISO convention with z up and positive for a *left* turn, while the
  estimator's frame is z down. With the sign flipped the estimator does not look broken — it reports a
  body-roll gradient of **116 °/g** with a correlation of 0.95, and only being seventeen times outside the
  physical range for a car gave it away. Checked rather than fitted afterwards: regressing `f_y` on
  `v·yaw_rate` gives slope −0.986, correlation −0.92, i.e. the accelerometer and the yaw sensor agree to
  1.4 % and only the convention differs;
* **the body-roll confound is real and measured.** The phone is bolted to the body, which rolls on its
  springs away from the turn, and that tilt is indistinguishable from road bank in one sample. Restricted to
  real cornering where the regression has any lever arm (|a_y| 1–2 m/s²) it is **+3.2 and +2.8 °/g** on the
  two runs — the low end of the physical range. On near-straight samples the same regression returns
  −7 °/g at a correlation of −0.07, which is a good reminder to gate a slope on having signal to slope
  against. Subtracted, with the gradient configurable and 0 meaning "leave it in", which is the honest
  default for a car where nobody measured it;
* **accuracy, and where it stops improving.** 2.1–2.3° per sample — the phone is on a windscreen mount and
  the IMU logs at **15 Hz**, not 100 — falling to 0.9° over one second and **0.65° over ten**, and then
  flat. That floor is the road's own camber changing along the route, so the filter is deliberately slow
  (τ = 10 s) and reports its own uncertainty, starting at the same 10° `paramsd` uses for "no usable roll".
  Median over a whole drive is −0.07° and −0.21° on the two runs, agreeing to 0.14°.

Nine tests. Published on `localization/pose` as `road_roll_deg` / `road_roll_std_deg` / `road_roll_valid` —
**gate on the std, not on presence**.

**And a defect found on the way there.** `ImuCalibrator::tryLockOrientation` replaced the whole rotation
with one derived from gravity. Gravity fixes two angles of three and says nothing about heading, so the lock
silently discarded the mount prior's heading, leaving the horizontal axes pointing nowhere in particular.
Harmless for yaw rate — only the z component is read, and gravity fixes z — and fatal for anything lateral,
which is exactly what this work needed. The lock now keeps the heading it had and replaces only what gravity
measured (`rotationFromGravityKeepingHeading`), and `sensors/imu_yaw` carries `lat_accel` with a validity
flag that is false until a heading exists.

**Localization sources are switchable per measurement**, which is what makes the course experiments real:
`use_gps_position`, `use_gps_course`, `use_gps_velocity`, `use_imu_yaw_rate`, `use_chassis_yaw_rate`,
`use_camera_odometry`, `use_bicycle_model`.

## 7. Missing from the System Entirely

* **lane change** — no `DesireHelper`; turn signal is decoded but used only to
  silence LDW;
* **urban** — no free corridor (drivable space), no parked-car detector;
* **interchanges, exits, intersections** — model plan is not designed for them.

## 8. Minor Items and Hygiene

* `models/sc_v0.8.12.onnx` and `models/sc_v0.8.13.onnx` are **byte-identical** (md5
  `d95f5d06…`), same as `assets/supercombo.onnx`. One of three copies is redundant.
* `maps/Moscow.osm.pbf` — 81 MB of 87 MB in `maps/`. Needed only to rebuild
  `Moscow.osm.admap` (4.9 MB), re-downloaded by `mapmatch/fetch_map.py`. Candidate for deletion,
  your call.
* Dated reports dated August bags and pipeline audit notes contain suggestions since implemented
  differently. Not rewritten: history with measurements. Where a later measurement refined the
  conclusion, explicit footnote — e.g. left-drift bags (plan offset turned out to be function of
  curvature, not constant left pull), low-speed bags, pipeline audit notes.

* `models/` — ~250 MB weights with no references (`sc_v0.8.12`, `sc_v0.9.0`,
  `supercombo_op089`, `supercombo_op094`, `yolov8n_traffic_192`, `yolov8n_coco_256`,
  `ru_signs_best.pt`). Not in git. **Intentionally not deleted**: artifacts, not junk, and
  cannot be restored from repo — decide what you need.
* `.tools/gradle-home` — 1.4 GB local cache inside project. Not in git; deletion costs
  re-download.
* `docs/calib/` — two chessboards (1920×1080 and 2560×1440), in git intentionally: needed for
  repeat calibration.
* frame-dt fix notes (removed) — five mentions of removed `fpa` controller. Dated report with
  measured numbers, left as-is.
* ~~Book build warnings~~ — **fixed 2026-08-06**, build is clean. Links to documents outside
  `docs/book` are repo-relative code paths now (sphinx cannot resolve a path outside its source tree, so
  they rendered as broken references), and `ATTRIBUTION.md` has a title, which accounted for seven of the
  eighteen warnings on its own.
* CI (`.github/workflows/ci.yml`) is written but **never ran** — no access to
  GitHub Actions. First run will likely need fixes.

## 4a. Lateral assist is absent half the time the controller thinks it is steering

**Found 2026-08-06 in run `2026_08_06_18_27_12`, reported as "рулёжка запоздалая, не хватает момента на
выходе из крутой дуги, приходится подкручивать".** It is not a tuning problem. The assist is simply not
there, and nothing in our stack says so.

The chain, every link measured:

* our `CarController` steers only when `cc.latActive = cmd_fresh && hca_cmd_enabled_ &&
  safety_.lastControlsAllowed()` (`panda_service.cpp:167`);
* `controls_allowed` for VW MQB is set by panda from `pcm_cruise_check(cruise_engaged)` with
  `cruise_engaged = TSK_06.TSK_Status ∈ {3,4,5}` — the stock cruise **actually engaged**, not merely
  switched on;
* on this drive `TSK_Status` was *main on but not engaged* **70.7 %** of the time. `controls_allowed` was
  true 29.3 %;
* so where our controller wanted to steer, the assist was present **2.6 %** of frames at 5–8 m/s, **18.3 %**
  at 8–12, 65 % at 12–16, 87 % at 16–22. The stock VW cruise drops below ~30 km/h and on brake — exactly a
  slow sharp bend and its exit;
* where `|angle error| > 5°`, the assist was on in **12.9 %** of frames; where the error was under 1°, in
  78.5 %. The error *is* the missing assist.

**But the assist is not the whole of the reported symptom — measured 2026-08-07, and this narrows what the
ALKA drive can deliver.** The complaint was specifically "на выходе из крутой дуги не хватает момента,
приходится подкручивать". Decomposing the bend exits of that run (|κ| falling from above 0.006, v > 3 m/s,
1340 frames):

| | assist off | assist on | assist on **> 3 s continuously** |
|---|---|---|---|
| frames | 992 (74 %) | 348 (26 %) | 319 |
| \|angle error\| median | 16.88° | **6.91°** | **6.40°** |
| achieved fraction of commanded angle | 0.15 | 0.83 | 0.84 |
| torque at the ±300 ceiling | 69.6 % | 77.0 % | **75.2 %** |

**Saturation on arcs is not windup and not a resumption transient.** It reproduces with the assist
continuously on for more than three seconds, so the gate and the integrator reset cannot remove it. The
contrast with straights is the whole finding:

| segment, assist on > 1 s | \|angle error\| median | p90 | at ceiling |
|---|---|---|---|
| straight (\|κ\| < 0.002) | **0.36°** | 0.85° | **1.9 %** |
| arc (\|κ\| > 0.006) | 6.46° | 19.56° | **81.0 %** |
| bend exit | 6.40° | 18.32° | 76.1 % |

So the loop is excellent where it is not saturated and open where it is. The causality runs error → torque,
not the other way: with `kp = 0.6` and a ±1 output limit, **1.67° of error already rails the PID**, so a
standing 6.4° error necessarily means a railed request.

**"The rack cannot deliver it" was wrong, and the correction matters.** Those ceiling shares are the
*pre-limiter request* (`lane_keep_debug.torque_cnm` is `steer_norm · 300`), not what reached the wheel.
Decoding `HCA_01.Assist_Torque` off `can/rx` — the actual bus value — for the same frames:

| segment, assist on | request median | request at 300 | **delivered** median | delivered at 300 |
|---|---|---|---|---|
| straight | 76 | 2.1 % | **60** | 0.0 % |
| arc | 300 | 81.3 % | **255** | 41.8 % |
| bend exit | 300 | 77.0 % | **207** | 31.3 % |

So the rack does deliver 255 cNm on our arcs. dragonpilot on the same car delivers a median of **51 cNm**
(`actuatorsOutput.steerOutputCan`) and rails 1.4 % of the time. **We ask for five times their torque on the
same roads** — that is the finding, and it is not an actuator limit.

**But the two populations are not comparable, and that is the honest end of this analysis.** On our bend
frames the driver was on the wheel: driver torque median **37** against 14.5 on straights, and
`steering_pressed` fired in **32.5 %** of bend-exit frames against 4.4 % on straights. Comma's numbers come
from hands-off frames by construction (`BENCHMARK_COMMA2.md` §1: "42 % usable — `latActive`, hands off").
Hands-off frames with the assist on exist in our bags for straights (551 of them, error 0.40°, identical to
the 0.36° with hands on) and **do not exist for arcs at all** — because the assist was absent so often that
the driver never let go on a bend.

There is also a mechanism that makes co-steering self-reinforcing, and it is upstream's too: driver torque
sets `override = true`, which unwinds the integrator, which weakens the assist, which invites more driver
torque. Upstream's driver is not in that loop because their assist is present.

**So the population that would answer "why five times the torque" has to be created**, and the ALKA drive is
the first one that can: with the assist present below 30 km/h, hands can come off on a bend. Until then, the
6.4° is a mixture of our command lagging and the wheel being where the driver put it, in unknown proportion.

**Two hypotheses eliminated by the same measurement**, so nobody spends a drive on them:

* **the speed-dependent angle ceiling is not binding**: `mpcMaxSteerRad` clipped **0.03 %** of frames overall
  and 0.12 % at 3–6 m/s, where the ceiling is 12.4° of road wheel against a requested p99 of 8.2°;
* **the held setpoint is not the standing error either.** The step is worst exactly in this band (p99 4.08° at
  3–6 m/s against a 0.107° median), so `lat_recompute_setpoint` should smooth the transients — but a 78 ms
  step cannot explain an error that persists for seconds.

**What this leaves open, and it is now the first lateral question:** why the commanded angle on tight low-speed
bends exceeds what the rack delivers. The differential harness cannot answer it — below ~10 m/s upstream's own
`actuators.steeringAngleDeg` is unusable (p90 130.8°) — so this needs either their `desiredCurvature` compared
against ours at low speed, or the admission that MQB HCA cannot hold tight bends at 3 N·m and the reference
must ask for less curvature there.

**When the assist is on, the lateral stack is as good as upstream's.** Same car, same controller, same gains
— `dragonpilot` runs Golf 7 MQB on the identical angle PID (kp 0.6, ki 0.2, kf 6e-5, confirmed from its own
`carParams` in the captured rlogs). Achieved fraction of the commanded steering-wheel angle:

| commanded | ours, assist on | ours, all frames | dragonpilot |
|---|---|---|---|
| 0–2° | 0.95 | 0.97 | 0.90 |
| 2–5° | 0.92 | 0.81 | 0.94 |
| 5–10° | 0.96 | 0.87 | 0.96 |
| 10–15° | 0.98 | 0.60 | 0.98 |
| 15–25° | 0.93 | 0.54 | 0.96 |

Median angle error with the assist on is 0.34–1.47° against dragonpilot's 0.16–0.79°. The middle column is
what every previous lateral measurement of ours has been reporting — frames with no actuation mixed in.

**Ruled out, each by measurement, so nobody re-litigates them:**

* *torque rate limit*: real but not the cause. `STEER_DELTA_UP = 4` cNm per 20 ms frame = 200 cNm/s, so full
  torque needs 1.5 s and 75 % of full-torque episodes are shorter than that. Requested 300 arrives as a
  median 187 applied. But panda enforces the same limit for upstream, and upstream tracks fine;
* *oscillation*: ours flips torque sign 0.03–0.10 /s, dragonpilot 0.12–0.21 /s. We are calmer, not noisier;
* *sign convention*: positive applied torque raises the CAN steering angle in both stacks (slope +0.011 and
  +0.038 °/s per cNm) — correct in ours;
* *HCA_01 encoding*: bit-for-bit identical to `opendbc/vw_mqb_2010.dbc` and to dragonpilot's `mqbcan.py`;
* *EPS refusing*: `EPS_HCA_Status` was `Ready` 71.6 % / `Active` 28.4 % and **never** `Fault` or `Rejected`;
* *vehicle model / learned parameters*: comma's learned values on this car are steerRatio 16.12,
  stiffnessFactor 1.247, angleOffset −0.02°. Against our 15.7 / 0.64 that changes the commanded angle by
  1.7 % at 6 m/s — nothing at the speeds where the problem lives.

**What we did not port: dragonpilot's always-on lateral (ATL / ALKA).** Two halves:

1. `controlsd.py:682` — when `dpAtl > 0`, `CC.latActive = True` whenever the car is moving and
   `CS.cruiseState.available` (the cruise **main switch**, which was on 100 % of our drive), gated on
   calibration valid, no `steerFaultTemporary`/`steerFaultPermanent`, and not in reverse;
2. `alternativeExperience |= ALKA` so the safety layer passes HCA torque while `controls_allowed` is false.

We send `alternative_experience = 1` (`DISABLE_DISENGAGE_ON_GAS` only) — confirmed in the bag's
`panda/health`. `ALKA = 16` in dragonpilot's panda while bit 16 in the flowpilot panda source is
`ALT_EXP_ALLOW_AEB`; **that apparent contradiction is resolved — the two trees are different panda
generations (health packet 11 against 16), and on the flashed firmware bit 16 is ALKA.** Proven from their own
recordings rather than from firmware source, which dragonpilot does not ship: they run
`alternativeExperience = 17`, `safety_tx_blocked` never incremented once, and `latActive` was true with
`controls_allowed` false in 64.3 % of frames with real torque applied in 96.3 % of those (median 53 cNm). Full
numbers and the exact gate to port are in §0.

**A separate bug, independent of ATL and worth fixing either way — fixed 2026-08-07.** The lane-keep service
never learned that its output was discarded: `steer_output_enabled` stayed true, the debug message said the
system was steering, and the angle PID kept integrating against an error it could not influence. Upstream
resets the controller when `not active` (`latcontrol_pid.py`).

`LaneKeepService` now subscribes to `panda/health` and gates the PID on it (`lat_require_assist`, on by
default; `assist_max_age_s = 0.5`, five panda periods). Measured on the bench, the windup this removes is
**74 cNm** after one second of withheld torque — resumption commands 283 cNm against the honest 203, and it
arrives as a step exactly when the assist returns. `assist_allowed` and `assist_known` go out on
`control/lane_keep_debug` regardless of the gate, so no future analysis silently averages actuated and
non-actuated frames together — which is what happened to every lateral number in this backlog before now.
Details and the two-answers-for-unknown design in §0.

## Done 2026-08-06 (backlog pass on top of the run analysis)

**A recorded comma route can now be pushed through our own stack and diffed against what upstream commanded**
— `app/src/main/scripts/rlog_lat_diff.py`. Same car, same road, same reference path, same measured steering
angle; the only difference is whose code turns that into a command. It runs through the middleware
(`AdasApp.publish_*` → `step()` → `pop_messages()`), not through internal classes, because the middleware is
the thing that hides the implementation and a comparison that reached around it would be measuring a wiring
diagram invented for the test.

**It paid for itself before producing a single comparison, by finding two defects that only a replay can see.**

* `LaneKeepService` read the **wall clock** from inside the service — `utils::getCurrentTimestamp()` — instead
  of `Service::now()`, which is the middleware clock and is simulated under `Mode::Simulated`. Off-vehicle
  that made the path age out by hours, so `status` became `stale` and *every* command was cleared; it also fed
  the angle PID a rate derived from wall-clock deltas rather than the replay cadence, and ran the blinker
  suppression timer on a different clock from the messages it gates. Fixed, and the two stale-gate tests now
  drive `mw.setTime` instead of matching the wall clock — which is what made them unable to catch this.
* `controls/steer` was not observable from the host API. `pop_messages()` returned the plan and the pose but
  not the actuation command, so an offline consumer could see the curvature and not the torque produced from
  it. `SteerCommand` is now part of `HostOutMsg`. Note that `LaneKeepOutput.steer_norm` is not a substitute:
  on the vision path it carries the geometric normalisation of the road-wheel angle and only the chassis path
  fills it with the angle-PID output — which is why the first comparison silently diffed two different
  quantities.

**First result, and it needs its caveat read first.** The harness feeds their `lateralPlan.dPathPoints`, whose
x grid is `v_ego · t_idxs` — at 6 m/s that is a 15-metre path, and our distance-based controller wants 8–25 m
of lookahead. So the low-speed half of the comparison is not yet fair, and it shows: on 3899 matched frames
above 5 m/s the setpoint correlation is 0.897 with a slope of 0.925, but split by speed it is slope 0.82 with
**2.58° RMS** at 10–15 m/s against slope 0.92 with **12.94° RMS** at 5–10 m/s. Where the path is long enough
for our controller, we agree with upstream to a couple of degrees. Fixing the harness means feeding
`modelV2.position` instead — 33 points to 74 m, speed-independent — and that is the next step.

What already survives the caveat: **our torque saturates far more than theirs on the same inputs** — 72.6 % of
frames at ±300 against their 46.9 %, and 52–60 % of frames at the ceiling even where *their* torque is under
150 cNm. The mechanism is the one §4a/#25 measured on our own bag from the other direction: `kp = 0.6` with
the error in degrees means the proportional term alone reaches full scale at 1.67° of error, and 68 % of these
frames differ from their setpoint by more than that. A porting-faithful controller fed a setpoint that differs
by two degrees behaves like a relay.

**The ENG/RU button led nowhere from the intermediate build tree — fixed.** Reported as "книга крашится при
нажатии ru", and the browser's actual words were "Your file couldn't be accessed", which is the symptom of a
`file://` URL that does not exist.

The cause was not in the URL mapping: replayed against a stub DOM, `lang-switch.js` produced the correct
counterpart for every page shape — nested, root, hash-suffixed, `file://` and `http://` alike. The cause was
*where the page was opened from*. `jupyter-book build` leaves its output in `docs/book/_build/html/`, that
tree is directly openable, it carries the switch script because the script is part of the English book's
static files — and it has no `ru/` inside it, because the two languages only meet when `build_book.sh`
assembles `docs/_site/`. The build script's own 404 check walked `_site` only, so it never saw the tree where
the button was broken.

Fixed so that the button cannot exist where it cannot work: `build_book.sh` stamps `data-adas-bilingual` onto
every page of `_site` *after* both languages are copied in, and the script draws nothing without that marker.
The intermediate trees are now honestly monolingual — no button at all. Verified in both directions: in
`_site` the button appears and its target file exists (50 transitions), in `book/_build/html` and
`book_ru/_build/html` no button is created. Two guards added: the stamping pass fails the build if any page
with a navbar lacks the marker, and the checks refuse to run if `lang-switch.js` stops testing for it —
because the gate lives in one file and the stamp in another, and neither is obviously load-bearing on its own.

**`paramsd` ported and left switched off, with the numbers that say why** — `utils/params_learner.h`, a
three-state EKF for stiffness / steer ratio / steering bias measured through the controller's own
`curvatureFromSteer`, twelve tests, published on `localization/pose`, replayable with
`bag_params_learner.py`, and gated behind two independent flags that both ship `false`. The full account is
in §5a; the four things worth carrying forward:

* the speed-dependent understeer that motivated the task is already handled by the vehicle model's
  `1/(1 − slip·v²)`, not by the constant inside it — residual `κ_fact/κ_model` is flat at ~0.90 across
  5–32 m/s with the shipped 0.64;
* stiffness and steer ratio are near-degenerate against a yaw-rate measurement; a loose prior on the ratio
  produced a self-declared-converged estimate that was **3 % worse** than the constants it replaced;
* the road-bank input is 0.0114 rad/s of error per degree at 15 m/s against a 0.0065 rad/s residual, so
  `params_use_roll` ships `false` — a negative result about the roll estimator built the day before, and the
  reason "A needs B" was never an argument that B is accurate enough for A;
* saturation looks exactly like convergence. `valid()` checked the stiffness bounds and not the ratio's, and
  a flipped steering sign (the port applied ISO → z-down but not `vehicle.steer_sign`) pinned both
  parameters to bounds for an entire drive while reporting itself valid. Both fixed and tested.

**σ is summarised over the wrong range — fixed, and it is the one lateral change for the next drive.**
The blending confidence took the median σ over a fixed 5–40 m. Measured on run 2026_08_06_00_36_42:

| segment | 5–20 m | 5–40 m | 20–40 m | 40–80 m |
|---|---|---|---|---|
| straight | 0.11 | 0.14 | 0.20 | 0.37 |
| left arc | 0.58 | 0.89 | 1.49 | 4.32 |
| right arc | 0.34 | 0.50 | 0.78 | 1.66 |

σ roughly doubles from the near half to the far half and quadruples past 40 m, because on a bend the
inner line leaves the frame and its far samples are extrapolation. The old window therefore let "I have
not seen that far" veto a line whose near half is fine — the 1.5 m cut-off rejected 20 % of left-arc
frames instead of 5 %. New knob `lane_std_range_m = 20.0` (C++, the Python mirror, and a runtime
parameter), four tests.

**The prediction attached to this was wrong, and re-measuring it is an open item.** It read "median blending
weight on arcs 0.19–0.20 → 0.24–0.26, share of frames with blending effectively off 33 %/32 % → 22 %/25 %".
Scored during the 2026-08-06 pre-drive with one script applied identically to both windows
(`bag_arc_offset.py --recompute --std-range {40,20}`, same bag), the effect is far smaller: σ p90 on arcs
0.70 → 0.63 and 0.76 → 0.70, and the share of frames where σ alone zeroes a lane line 15 % → 14 % on left
arcs and 4 % → 2 % on right. The median blending weight does not move, because the median σ on this bag is
0.13–0.32 against a 1.5 cut-off — the whole effect is in the tail.

Worse, the σ-by-segment table above (left arc 0.58 over 5–20 m, 0.89 over 5–40 m) is a **factor of four**
above what the same run measures through the analysis that mirrors the controller (0.13 and 0.14). One of
those two aggregations is not the quantity `laneLinesToPath` weights by. Finding out which is the open item;
until it is resolved, the σ table supports the *shape* of the argument — σ grows steeply with distance on
arcs — and not the magnitude of any predicted gain. The change itself stays, because a near-field median is
the more defensible summary either way, but it ships as a regression check rather than as an expected win.

**Raising `lane_std_bad_m` was the wrong lever — swept and rejected.** 1.5 → 2.0 leaves right-arc
blending at exactly 0.00 (σ there is 2.48, above any sane threshold) and 2.5 buys 0.05 m. This closes
the "raise the threshold" line of the σ item; the window above is what actually moves it.

**Road edges are not a usable fallback — measured and closed.** Their σ was parsed by nobody, in Java
or in Python, so no bag could answer the question. Now recorded (`edges[].y_std`, `EDGE_STDS_START`,
plus the Python mirror). First measurement, 555 frames: **edge σ 1.17 median at 5–20 m against 0.14 for
the lane lines — 7.5× worse** — with only 68 % of frames under the 1.5 m cut-off, and the two edges
bracket 12.8 m (p10 10.5, p90 15.3), i.e. a wide corridor rather than our carriageway. Substituting a
7× noisier estimate for the lines it would replace cannot help, so "road edges first, then relax the
two-line requirement" is dropped. The relaxation itself now has no cheap backup.

**NNAPI FP16 wired, off by default, checked offline first.** `vision.nnapi_fp16` with the fallback chain
fp16 → plain NNAPI → CPU, mirroring `TrafficYoloRunner`. Verified with `bag_fp16_ab.py` on 200 frames
where both lines are visible: converting the whole model to fp16 — stricter than NNAPI, which keeps a
float32 graph and may only relax per node — moved the lane centre 0.027 m and the plan offset 0.037 m,
while line probabilities rose (right line 0.37 → 0.82) and the σ tail shrank (p90 0.69 → 0.33). Not a
degradation. Not free either: per-frame disagreement 0.05 m median, 0.20 m p95 on the lane centre. Left
off so the σ-window change gets a clean drive; worth ~15–20 ms of the 45.6 ms inference when enabled.

**Speed sources measured against GNSS Doppler, `a_ego` fixed, and speed made a state.** New section 5a
with the numbers and the dependency chain for using the localizer in the planners. Short version: wheel
speed reads **1.2 % high with a flat scale across speed bands** on both runs — a wheel-radius constant,
knob added (`wheel_speed_factor`, default 1.0) and queued. `a_ego` was quantisation noise (RMS step
4.16 m/s², extremes ±74), now a two-state filter with real `dt` (RMS step **0.062**, extremes −4.0/+3.1,
speed unchanged to 0.010 m/s). And two defects of the same family as the yaw-rate one: `predict` assigned
the wheel speed to the state every tick, erasing the GPS velocity update about twenty times per GPS
sample, and that update was given an assumed 1 m/s of noise when Doppler measures at 0.1. Both fixed;
the scale is still not identifiable, and now there is a test saying why. Thirteen tests.

**Localization sources are configurable per measurement.** `localization.use_gps_position`,
`use_gps_course`, `use_gps_velocity`, `use_imu_yaw_rate`, `use_chassis_yaw_rate`,
`use_camera_odometry`, `use_bicycle_model`. A fused estimate does not say which sensor carries it, and
when several agree a broken one hides behind the others — switching them off one at a time is the only
cheap way to find out what the filter would do without each. It is also what turns "disable GPS and watch
the heading drift" from a thought experiment into a line of config, which the course needs.

**Build fails on a malformed `config.json`.** `loadFromFile` reacts to bad JSON with one log line and
built-in defaults, which on the phone means driving on the wrong parameters. Two missing commas slipped
in on 2026-08-06 alone, so `build_cpp.sh` now parses the shipped config before compiling.

**Localization heading no longer comes from the bicycle model** — section 6 rewritten with the fix, the
reasoning, and six tests where there were none. Nothing in the control path reads `localization/pose`,
so this changes the recorded pose and mapmatch input, not steering.

**Frame arrival instrumented.** `submit_ts_ms`, `pickup_ts_ms`, `frames_dropped` on `vision/lanes`, read
by `latency.py`. Until now `capture_ts_ms` was the earliest thing known, so a frame arriving 40 ms late
and a frame waiting 40 ms in the queue looked identical — and the daytime stalls (reference older than
300 ms in 15.9 % of run 08-04, one 75 s hole) could not be attributed. Many drops with short delivery
means inference is too slow; few drops with long delivery means the camera is late.

## Done 2026-08-06 (first run with longitudinal actuation — and what it exposed)

Run `2026_08_06_00_36_42`, 27.9 min night, 22 170 vision frames.

**The pipeline got faster, as predicted.** `TARGET_FPS` 20 → 30 delivered: vision **11.29 → 13.24 Hz**,
capture → CAN command **83 → 79 ms** median (p95 115 → 111), frame prep mean 15.3 → 11.4 ms, capture →
capture median 87 → 68 ms. Inference is untouched at 45.6 ms, so the remaining lever is still FP16
(section 5). No `stale` frames in either run; `low_speed` share 21.9 → 12.9 %.

**Lateral, left arcs: `tire_stiffness_factor` 0.50 works.** Total offset +0.30 → **+0.23** m, tracking
error +0.29 → **+0.21**. Half the predicted gain, right direction. Next step down (0.40) only after the
right arcs stop hitting the torque ceiling — a bigger command there is not executed anyway.

**Lateral, right arcs: −0.00 → −0.29 m, and the decomposition says this is perception, not control.**
Reference offset −0.10 → −0.21 because worst-line σ on those arcs reached **2.48** against the
`lane_std_bad_m = 1.5` cut-off, so blending fell from 0.10 to **0.00** and the reference became the raw
model plan, which sits at −0.33 m from lane centre. Two things make the comparison weaker than it looks:
the night run's −0.00 was cancellation of −0.10 reference and +0.09 tracking over 134 frames, and this
run's right arcs are far tighter (R 71–134 m against 130–273). Raising the σ threshold is now the top
lateral item.

**Matched σ comparison, not raw.** `bag_lane_sigma_ab.py` buckets by |κ| and speed so route differences
cannot masquerade as perception changes. Overall σ improved (share above the 1.5 cut-off 27.4 → 14.5 %,
median 0.54 → 0.40), but that is the route mix — this run spent far more time fast and straight. Matched
by condition it is mixed: worse on straights at every speed (×1.04 to ×1.57) and at κ 0.004–0.008 above
14 m/s (×3.46), better on tight slow arcs (×0.53 to ×0.70). Frames are brighter (road band luma 99 →
112) but less sharp (Laplacian variance 350 → 199) — consistent with shorter exposure at higher gain and
the ISP's noise reduction working harder. Not enough to call it a regression; worth one run with
`TARGET_FPS` back at 20 to separate.

**The camera moves every drive, so persisting calibration is not the fix it looked like.** Learned yaw
+1.67° (08-04) → **+0.10°** (08-06), pitch −1.11 → −0.67. A prior from the previous mount is no better
than the default. Convergence is fast — 30–60 s, and on straights the offset in the first 30 s is 0.02 m
— so the warm-up costs little and no calibration gate is needed. What this does mean: **plan offsets are
not comparable between runs with different learned yaw**, because yaw goes into the input warp. That
weakens every cross-run perception comparison in this document, including the right-arc one above.

**Longitudinal actuation was a regression, root-caused and fixed.** With `cruise_buttons` on for the
first time, the plan asked for a set speed **4.81 m/s below actual** at the median (p5 −8.87) and
demanded more than 0.5 m/s² of braking in 66.9 % of ticks. The bus shows **715 rising edges of
`cruise_decel` and 690 of `cruise_accel` in 28 minutes** — the set speed was being chattered
continuously. Four separate defects, all now fixed with tests:

| defect | evidence | fix |
|---|---|---|
| `plan_v` used as an absolute speed target | `plan_v0 / v_ego` = **0.678** median over 18 916 frames (0.756 at 5–10 m/s falling to 0.669 at 20–30) — worse than the camera odometry's own 0.888, and speed-dependent, so no single constant repairs it | `plan_v_enabled = false`; free flow now means hold the current speed |
| plan picked the most probable of `lead0/lead1/lead2` | `lead1`/`lead2` are the model's +2 s and +4 s predictions. The warning path was fixed this way on 2026-08-02; the plan kept the old behaviour | `lead0` only |
| no in-lane gate on the lead | lead used in 38.2 % of ticks, **30.9 % of those reported it nearly stationary** — parked cars at night, and `v_target = lead_v` then demanded a stop | `lead_max_offset_m = 2.0` against our own path, plus `lead_min_speed_ms = 2.0` |
| `v_target = lead_v` announced the whole deficit at once | a lead 100 m ahead and 5 m/s slower needs no action, but the actuator reads any gap past its 0.7 m/s deadband as "tip down" | target derived from `a_target` over a 3 s horizon, floored at the lead's speed |

Plus the actuator itself: **the speed the driver had at engage is now a ceiling** for the whole
engagement (`cruise_v_set_ceiling_`). Without a radar we have no business asking for more speed than was
chosen, and it removes the up-tip half of the hunting.

Replayed over the same run (`bag_long_replay.py`): median `v_target − v_ego` −4.81 → **+0.00** m/s,
ticks demanding over 0.5 m/s² of braking 66.9 → **0.0 %**, and cruise tips **73.1 → 1.5 per minute**.

**This car cannot brake, and now the plan knows it.** Measured with `bag_coast_decel.py` over four
2026-08-04 bags (125 intervals, both pedals up, speed falling monotonically): median **−0.28 m/s²**, p10
−0.51, strongest −0.89, and nearly flat with speed (−0.25 at 0–5 m/s, −0.26 at 14–20, −0.44 at 20–30).
Flat with speed at that magnitude is road load, not engine drag — the DSG is sailing. The signal that
would change that, `ACC_Freilauf_Anf` ("request DSG sailing", `ACC_07`), is sent only by the
radar-equipped ACC. So `a_coast_ms2 = -0.30` is the whole deceleration budget, and anything past it is
published as `status = "brake_needed"` — a request to the driver, not a command. Consequence worth
stating plainly: at 20 m/s, reaching a 14.7 m/s corner speed needs 307 m of coasting while the path
preview is 80 m, so the curvature limiter can only trim speed at city pace, never make a highway corner.

**A header default is not a decision until the shipped config agrees.** The FCW/AEB speed gate was
raised 3 → 8 m/s on 2026-08-04 in `safety_planner.hpp`, but `assets/config.json` kept 3.0 and the file
wins — so run 08-06 reproduced the same three false warnings against a stationary object at 4.8–5.5 m/s.
Config fixed, `ldw_suppress_on_lat_active` was not even parsed (added), and three `ShippedConfig` tests
now assert the shipped file matches the decisions taken on road. LDW is silent on this run (0 episodes),
so that gate did hold. The other 10 forward warnings are genuine closing situations (TTC 1.9–2.7 s,
closing 5–11 m/s, one at a 0.78 s headway) — whether `fcw_ttc_s = 2.5` is too eager for city following
is a separate judgement call, not a defect.

## Done 2026-08-04 (night run + fixes on top of it)

* **Arc offset measured with control actually engaged** — right −0.71 → 0.00, left +0.51 → +0.30,
  straight −0.04; setpoint offset on arcs +0.02 / −0.10. Blending did its job; what remains on the left
  arc is tracking error. Section 1 and `BENCHMARK_COMMA2.md` §2.
* **Step 0 closed: the ESP yaw sensor is sound.** Phone gyro / ESP = 1.017 (corr +0.969, no speed
  dependence), camera / ESP = 0.849, camera / gyro = 0.788. Two physical sensors agree; the camera is the
  outlier, consistent with its metric scale 0.893. So `tire_stiffness_factor` may move — **down** from
  0.64, not to the comma's 1.319.
* **Warning false positives fixed at the source.** 5 forward warnings, all stop-and-go (median 4.7 m/s,
  max 8.5) → FCW/AEB speed gate 3 → 8 m/s. 7 lane warnings, 82 % of frames with our own steering engaged
  at |cte| 0.54 m → LDW suppressed while we steer. Both with tests. `SAFETY_WARN.md`.
* **LKA hands the wheel back on a turn signal** (`lka_suppress_on_blinker`, resume delay 1 s). Not keyed
  on `steering_pressed`: 520 episodes in 23.5 min, 30 ms median — gating on it would drop the assist every
  few seconds.
* **Camera `TARGET_FPS` 20 → 30** — the vision rate is quantised by the camera period; see section 5.
* **No vision stalls at night** (`frame_dt` max 119 ms, zero `stale` frames) against p99 190 / max 389 and
  a 75 s hole in daylight — heat remains the explanation.
* **Localization verified against GNSS**: heading matches the GNSS course to 0.3° median (p99 5.4°),
  speed to 0.17 m/s, fix type 3 throughout.

## Done 2026-08-04 (earlier)

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

* **Run 0804 analysis** — 2026-08-04 bag: perception after focal fix, planner reference,
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
  (`BENCHMARK_COMMA2.md`), plus stage-by-stage pipeline check — what matches 1:1 and what diverges.
  Tool `rlog_arc_offset.py` reads rlog via cereal schema and computes same breakdown as
  `bag_arc_offset.py`.
* Consolidated work plan with meter targets — `BENCHMARK_COMMA2.md`.
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
  each found — arc-offset bags. Reproduced on second run.
* Measured bar: driver without controller also goes ±0.14 m inside, magnitude 0.15–0.17 m. On straight
  controller already smoother than human.
* Config: `path_lane_blend_scale` 0.3 → 0.6, σ thresholds 0.2/0.8 → 0.3/1.5, width up to 4.6 m with filter.
* Ported `center_force` from flowpilot (replay chose 0.4; later shipped default **0.0**).
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
