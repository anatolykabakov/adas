# Run 2026-08-04: Perception After Intrinsics Fix, 100 Hz Loop, Panda Without Control

Run `adas_logs/2026_08_04_11_15_15`, 30.1 min, **urban**: speed median 11.3 m/s, p90 14.8,
never above 20 m/s. 15 409 model frames, 33 % usable for metrics. First drive on
build with measured focal length (fx 993.4 / fy 995.2), blending 0.6, σ thresholds 0.3/1.5, camera
shift 0.05, and CAN receive at 10 ms.

Three short catalogs same day (`10_58_04`, `11_14_56`, `11_15_11`) — failed starts, no
CAN or almost no data.

**Vehicle was not controlled** — cause found and it is not the controller (section 1). Planner still computed
as usual, so reference and perception can be measured; tracking error cannot.

## 1. Why Steering Was Silent: Panda with Comma-Two Firmware

Panda sends `health` packet **version 11**, host expects 16. v11 has two fields absent in 16 —
`gmlan_send_errs` before `faults` and `gas_interceptor_detected` before `car_harness_status`, —
so everything after `rx_buffer_overflow` shifts by 5 bytes. This happens after connecting panda to comma-two:
its `pandad` reflashes its firmware (`dragonpilot/panda/board/obj/panda.bin.signed`, there
`HEALTH_PACKET_VERSION = 11`).

Diagnosis rests on all fields matching at once, not guesswork:

| our code read | actually this | confirmation from run |
|---|---|---|
| `interrupt_load = 2.3509887e-38` | bytes `[fault_status 0, power_save 0, heartbeat_lost 0, alt_exp 1]` | value **exactly** `0x01000000` in 100 % of messages, and `alt_exp = 1` our code also sets |
| `safety_mode = 1` | `ignition_line = 1` | ignition was on entire trip |
| `safety_param = 256` (1.5 % frames) | `controls_allowed = 1`, 13 episodes | **panda itself allowed control** |
| `heartbeat_lost = true` always | `safety_mode = 19` (NOOUTPUT), any nonzero → true | mode we set at startup |
| `power_save_enabled = true` always | `car_harness_status ≠ 0` | harness detected |
| `sbu2 = 2` | `usb_power_mode = 2` (CDP) | panda without SPI lacks these fields entirely |

Then failure mechanics are unambiguous: supervisor sees `ignition_line = 0` and `ignition_can = 0`, while
voltage 12.6 V > 11.5 V, so "sticky" ignition still turns on and VW mode is set —
and panda **does** enter it (else no `controls_allowed`). But reading it back fails:
instead of 15 we read 1. Supervisor thinks mode "did not stick", retries with
rollback, and `carControllerCallback` requires `lastSafetyMode() == kVolkswagen` and sends no HCA for a
single frame. Panda and car were fine.

**Fixed** (`panda/health.h`, `panda.cc`): on connect query packet version (`0xdd`) and
for 11 parse with own struct mapped to v16. Unknown version — loud log error.
Tests: layout of both versions, v11 mapping and reverse check — old code on same bytes
gives **exactly** values seen in run.

No need to reflash panda, but if desired — use a comma-two / flowpilot `panda.bin.signed`
and this repo's `scripts/panda_flash.py` (if present) or the upstream panda flash tool.

## 2. Perception: Noticeably Better, but Part of Gain Is Lighting

Comparison with run `2026_08_02_22_02_38` on **comparable sample** (v ∈ [8, 15] m/s, straight
|κ| < 0.002, both lines visible):

| | 0802, night, fx 951 | 0804, day, fx 993.4 |
|---|---|---|
| host-line σ, median | 0.344 | **0.226** |
| p90 | 0.864 | **0.591** |
| share σ > 0.8 | 12.2 % | **4.5 %** |
| line probability, median | 0.88 | **0.97** |

Caveat that cannot be omitted: 0802 is night, 0804 is day, different conditions for the model. σ alone
does not prove fx difference.

**What does prove it — metric scale.** Ratio `cameraOdometry.trans[0]` to wheel speed
does not depend on lighting or route:

| | 0802 (fx 951) | 0804 (fx 993.4) |
|---|---|---|
| `trans_x / v_ego`, median | 0.844 | **0.888** |

Expected +4.5 % (993.4/951), measured +5.2 %. Focal fix worked exactly as intended,
and distance underestimation dropped from 15.6 % to 11.2 %. Remainder not explained by fx: to reach 1.0,
fx ≈ 1119 would be needed; board gives 993 with RMS 0.38 px. Remainder is model domain gap.

Consequence for blending: at σ ≈ 0.23 and thresholds 0.3/1.5 σ weight **no longer cuts lane paint** —
`d` on straights equals 0.60, i.e. exactly set `blend`. Previously at σ ≈ 0.34 and thresholds 0.2/0.8
it fell to ≈0.77 of nominal.

## 3. Planner: Reference Landed on Lane Center

Analysis `bag_arc_offset.py` (cross-check with recorded fields converges: median 0.000 m,
p90 |·| 0.018 m).

| segment | n | setpoint offset | plan·(1−d) | lane·d | shift | d |
|---|---|---|---|---|---|---|
| straight \|κ\|<0.002 | 4879 | **−0.04** | +0.01 | +0.00 | −0.05 | 0.60 |
| left weak 0.002–0.004 | 62 | −0.06 | −0.02 | −0.00 | −0.05 | 0.50 |
| right weak | 110 | −0.07 | −0.02 | +0.00 | −0.05 | 0.60 |

Was in 0802: −0.10 on straight, +0.15 on left arc, **−0.42** on right. Now setpoint offset on
straight is practically camera shift alone (−0.05), plan contribution down to centimeters — same as
comma-two where reference equals lane center plus constant `CAMERA_OFFSET`.

Two quantities from recorded fields not available before:

* **`lane_offset_m`** — where the driver actually was: median +0.038 m, abs 0.103, p90 0.281.
  Lanes reliable (`lane_anchored`) in 75 % of moving frames, lane width median 3.04 m;
* **`cte` = 0.093 m abs** — how far the driver was from **our** reference. Reference
  passed where the human steered.

**What this run did not verify: arcs.** Only 38 frames with |κ| > 0.004, zero episodes longer than 2.5 s.
Route was urban with almost no arcs, and that is where the open problem lies (0.51 m left,
0.71 m right). Expectation after fixes remains unverified; need highway run with arcs like
`0802`.

## 4. Inner Loop 100 Hz: Confirmed on Vehicle

Step 2 of plan `PLAN_TO_COMMA2.md` closed on rates:

| | 0802 | 0804 |
|---|---|---|
| `vehicle/state`, median dt | 50 ms | **10 ms** (p90 11, p99 18, max 88) |
| `controls/steer` | 50 ms | **10 ms** (p99 13) |
| panda `rx` timer | 50 ms | 10 ms, mean dt 10.8, max 96 |
| parse in `panda`, callback | — | 1.1 ms mean, 51 ms max |

USB and DBC parsing hold at 10 ms: 186 054 messages over 30 min, no drops.

**But vision regressed:**

| | 0802 | 0804 |
|---|---|---|
| `vision/lanes`, median dt | 87 ms | 88 ms |
| p90 | 169 ms | 174 ms |
| **p99** | 179 ms | **609 ms** |
| max | 216 ms | **939 ms** |

Median unchanged, tail 3× worse. Softer wording: by time, not frames,

| how old reference is | time share, 0802 | time share, 0804 |
|---|---|---|
| older than 150 ms | 19.9 % | 42.2 % |
| older than 300 ms | 0.0 % | **15.9 %** |
| older than 500 ms | 0.0 % | **11.8 %** |
| mean reference age | 56 ms | **96 ms** |

Plus one stall of **75 seconds** (excluded from percentages above: gaps > 5 s dropped).

**Not vision in general, only supercombo pipeline.** In five-minute windows stalls cluster in two
chunks (15–20 min: 82 % of time reference older than 0.3 s, mean dt 333 ms; 30–36 min: 85 %, mean dt
657 ms, i.e. 1.5 Hz), absent elsewhere. In same windows `vehicle/state`
holds 10 ms, `traffic/state` — its 100 ms. So neither CPU, CAN, nor timers:
camera → model chain stalls.

**And not silent CPU fallback.** Verified on device after run: `NNAPI EP enabled`, 278 of 280
and 372 of 374 graph nodes on NNAPI. So ~80 ms is NNAPI speed on OnePlus 7T, not
CPU fallback (~107 ms).

Battery in stalls 52 °C, little cores 1.56–1.63 GHz, charge at end 16 %. But windows 20–30 min at same
temperature and frequency ran without stalls, so overheating does not explain all. Next: log
camera frame arrival and inference completion separately (now only result visible), capture
`logcat` for trip and repeat at night. Cannot separate daytime sun and 100 Hz CAN receive from one
run, but stalls this long did not appear in any previous run.

### Safety Gap: 250 ms HCA Timeout Does Not Protect Against This

`PandaService` stops sending HCA if steering command was not updated for 250 ms. But `hca_cmd_ts_ms_`
updates on **any** `controls/steer` message, published by fast angle PID — with
`vehicle/state` arrival, i.e. 100 Hz, regardless of reference age. Measured in stalls:

| window | `vision/lanes`, mean dt | `controls/steer`, mean dt |
|---|---|---|
| 15–20 min | 333 ms (max 939) | 10.8 ms |
| 30–36 min | 657 ms (max **74 848**) | 9.9 ms |

So with control enabled the car would keep steering to desired angle computed from plan
up to 75 seconds old, and no gate noticed. This run did not trigger only because
steering was blocked by panda.

Need reference-age gate in `LaneKeepService`: older than ~250–300 ms — status `stale`, command
cleared (or published with `enabled = false`), HUD warning. On this run that would mean
control withdrawal 16 % of the time — which is correct: cannot steer on half-second-old plan.

## 5. Camera Calibration: Converged Fast, but Config Prior Is Stale

Estimate settled in 1.5 min and held 30 min: pitch +1.02 → +1.40°, yaw −2.95 → −2.70°. Spread
±0.2° — more stable than spread between previous runs.

Config still has **old** prior: `rpy_deg {roll 0, pitch −1.79, yaw 0.52}`. Camera was
remounted, prior is 3.2° off in pitch and 3.2° in yaw. Learned value is not persisted, so every drive first ~1.5 min runs with wrong prior — and it enters model input
warp. Set prior to learned value:

```bash
./scripts/push_config.sh --apply \
    --set calibration.camera.rpy_deg='{"roll":0,"pitch":1.4,"yaw":-2.7}'
```

`--set` required here: without it script preserves calibration from device as-is.

## 6. Found Defect: Camera Angular Velocity Was 57.3× Too Small

`CameraOdometry.java` applied `Math.toRadians` to `rot`, although model output is **already rad/s** — as in cereal (`rot @1 # rad/s in device frame`) and flowpilot `driving.cc`
writes `setRot({r_mean.x, r_mean.y, r_mean.z})` without conversion. Same for `rotStd`.

What broke:

* **"driving straight" gate in `PoseCalibrator`** (`|rot.z| < 2 °/s`) never fired **once** in run:
  true in 100 % of frames, rejection reason `yaw_rate` never appears. Max |rot.z| was
  0.0125 vs threshold 0.0349 — with correct scale that is 0.72 rad/s, and gate would work;
* **camera yaw correction in localizer** got near-zero turn rate as measurement.

Honest about impact on this run: calibration barely affected. Angle spread gate
(`angle_std < 0.25°`) already rejected turns — among accepted samples in turn only 7 %, and
observed yaw differed from straights by 0.07°. Defect is real, but not explanation for
previous calibration spread between runs.

After removing multiplier, camera angular velocity vs ESP sensor:

    corr = −0.879 (sign — different frame conventions),   scale = 0.92

Camera sees turn 8 % weaker than ESP shows. Third independent source
alongside phone gyro and `mapmatch` fit to OSM (+49° vs GPS reference +47°, i.e.
ESP high by 4 %). Both point same way — ESP slightly over-reads — and both too small
to explain twofold understeer gap.

## Next Steps

1. **highway run with arcs** on this same build — only thing that closes main question;
2. set calibration prior `pitch +1.4 / yaw −2.7` (section 5);
3. verify vision tail with night run (section 4);
4. recompute yaw scale from fixed camera odometry and close step 0 (section 6).
