# Pre-drive checklist

One drive answers one question. This file is the record of which question, what was changed to ask it, and
what would count as an answer — written *before* driving, because a criterion invented after looking at the
data is not a criterion.

Everything here is verified against the artefact that actually goes to the car, not against the source tree.

---

## This drive (prepared 2026-08-09 — lateral feedforward)

### The question

**Does a correctly sized feedforward fix the under-steer the driver keeps reporting on bends?**

The previous drive answered the vision-rate question and it answered it well — see below. What it did
not fix was the complaint itself, and the voice notes recorded on that drive said so twelve times.

### What the last drive established, so this one does not re-ask it

Vision runs at **29.86 Hz** on the road (interval median 33.0 ms, capture→CAN 58 ms against 93 before).
The setpoint step halved exactly as predicted, and tracking error fell with it:

| segment | step median 13.5 Hz → 29.9 Hz | share > PID rail | \|error\| median |
|---|---|---|---|
| gentle 167–500 m | 0.25° → **0.19°** | 2.0 → **1.4 %** | 0.70 → 0.62° |
| medium 83–167 m | 0.49° → **0.28°** | 8.0 → **3.2 %** | 1.87 → **1.13°** |
| sharp R < 83 m | 1.01° → **0.83°** | 32.8 → **14.9 %** | 8.21 → **5.87°** |

**The mechanism was right and the rate is no longer the limiting factor.** The complaint that remains is
a different fault, and the driver's own voice notes located it.

### What the voice notes located

Every note saying «недокрут», «не справился», «рывками» lands on a tick where torque is at **300 cNm,
the panda ceiling**, with 15–55° of angle error. Over the drive, 11.7 % of ticks above 23 km/h sit at the
ceiling. But lateral acceleration at those moments is **0.58 m/s² median, 1.27 p95** — nothing. The torque
is not fighting the corner; it is fighting the tyres to turn the wheel at low speed.

The feedforward could not help, because of its shape:

```cpp
const double ff = desired_swa_deg * v_ego_mps * v_ego_mps;   // was
```

It scales with v², so at 26 km/h it delivered 0.16 of the ±1 range. Torque could only rise **after** an
error appeared, through the integrator, which is slow by construction.

### The change

| what | from | to |
|---|---|---|
| `vehicle.lat_pid_kf` | 0.00006 | **0.00015** |
| `vehicle.lat_pid_ff_floor_mps` | — (absent) | **9.8** |

Feedforward becomes `kf · SWA · (v² + v₀²)`. Same shape as comma's above the floor; below it a
speed-independent demand takes over.

**Both numbers are measured, not chosen.** On three drives (2026_08_04, 08-07, 08-08), over steady-state
frames with the proportional term explicitly subtracted, the required coefficient on SWA fits `a + b·v²`:

| drive | a | b |
|---|---|---|
| 2026_08_08_23_00_28 | 0.0193 | 0.000119 |
| 2026_08_07_19_04_05 | 0.0140 | 0.000146 |
| 2026_08_04_21_00_18 | 0.0049 | 0.000173 |

`a/b` is the floor squared — 9.8 m/s, about 35 km/h. The old gain under-delivered **4–8× at every speed**,
not only at low ones; that part of the diagnosis was a surprise and is the larger correction of the two.

### What would count as an answer

* **the complaint itself**: record voice notes again. Fewer «недокрут» notes, or the same notes at
  tighter radii, is the result. This is the primary criterion and it is deliberately the driver's;
* **share of ticks at 300 cNm** above 23 km/h, against 11.7 %. Expect it to *rise* — feedforward reaches
  the ceiling sooner. That is not a failure by itself;
* **\|angle error\| median on arcs 83–167 m**, against 1.13°, and on R < 83 m against 5.87°. This is the
  number that must fall. If error does not fall while ceiling occupancy rises, the change is wrong;
* **overshoot**: `|error|` p90 on gentle arcs, against 3.96°. Feedforward that is too large shows up here
  first, as the car turning in more than asked.

### What could not be verified before driving, and why

`bag/bag_feedforward_ab.py` replays the inner loop over the recorded drive, but **open loop**: the recorded
actual SWA is what the car did under the *old*, insufficient torque. With more torque it would have turned
further and the error would have been smaller, so the replay **over-estimates** the new command. Its
ceiling-occupancy numbers (49 → 72 % on 83–167 m) are an upper bound, not a prediction, and the harness
reproduces the recorded old torque only to ±42 cNm, so they are not decision-grade either.

What the replay does show, and what does not depend on its accuracy: at steady state the feedforward's
share of the command goes **19 % → 55 %**, i.e. the work moves off the integrator and onto a term that is
instant. On bend entry, at the same small error, commanded torque goes 192 → 266 cNm.

Proving the tracking improvement offline would need a validated model of how the rack responds to torque.
We do not have one. That is the honest reason this needs a drive.

### Reverting

`vehicle.lat_pid_kf` → 0.00006 and `lat_pid_ff_floor_mps` → 0. Nothing else in the control path changed;
`vision.model_runner` stays on `thneed`.

---

## Previous drive (prepared 2026-08-08, evening — the vision model)

### The question

**Does driving the vision model at 30 Hz instead of 15 fix the late, jerky steering on bends?**

Everything before this drive said the mechanism is the vision rate, not the controller. The size of a
setpoint step is set by the interval between frames, and that was measured twice:

| run | rate | setpoint step on arcs, median | share of steps > 1.67° |
|---|---|---|---|
| 2026_08_07_19_04_05 | 13.46 Hz | 0.49° | 7.9 % |
| 2026_08_08_10_47_41 | 10.06 Hz | **0.91°** | **21.7 %** |

1.67° of error alone rails the angular PID (kp = 0.6, limit ±1). At 10 Hz every fifth frame on a bend
delivers a jump larger than the controller's whole linear range, in one step.

### The change

| what | value |
|---|---|
| `vision.model_runner` | **`onnx` → `thneed`** |

That is the whole drive. One switch, one question.

**thneed is the flowpilot model (generation 0.9.x) executed on the GPU through OpenCL.** Measured on this
phone: **15.9 ms** inference against 44.7 for ours through ONNX and 64.9 through ONNX with fp16. Work now
fits inside the camera period, and the pipeline holds **30.0 Hz** — verified by frame counts against
log timestamps, 100 frames per 3.33 s, over 2400 frames.

The switch is now in Parameters (Vision model → ONNX / THNEED) and rebuilds the pipeline live, without a
restart. Flip it on a straight, not in a bend: while the pipeline rebuilds, the camera delivers to nobody
and lateral control has no target for that fraction of a second.

### What would count as an answer

Same criterion as the previous drives, so the numbers are comparable:

* **rate**: `interval_ms` median ≤ 40 ms. Below that the whole premise is untested;
* **the actual question**: setpoint step median on arcs R 83–167 m, against 0.49° at 13.5 Hz and 0.91° at
  10 Hz. Halving the interval should roughly halve the step;
* **the driver's answer**: does it still steer late out of a bend.

### What is NOT verified, and must be read with the result

**The lane geometry of this model has never been measured against ours.** Its speed is proven; its
accuracy is not. The second camera input is fed the same narrow frame warped with a wide matrix
(focal 455 against 910), so there is no real wide field there — the periphery is simply outside the source
frame. flowpilot ships it that way and it drives, but "drives" and "steers well" are different claims.

Consequences to expect on `thneed`:

* **no longitudinal plan.** Their output layout is parsed for lanes and pose only; leads and meta sit at
  different offsets and are not read. `vision/model_long` stops being published, so lead detection and
  curve-speed targets are absent for the whole drive. Lateral is unaffected;
* **camera pose is published and calibration runs.** The offset was derived from their `driving.h`
  (4955 + 536 + 264 + 105 + 88 = 5948) and confirmed on the bench: five of six pose components read zero
  on a stationary phone. The sixth, forward velocity, reads ~10 m/s while parked, which is not physical.
  Calibration is robust to that — it uses the *direction* of the velocity vector, `atan2(z, x)` and
  `atan2(y, x)`, and gates on wheel speed from CAN — but the scale itself is unverified. Comparing
  `model__camera_odometry` against `vehicle__state` on this bag settles it (task #37).

### Also new on this build

* **`vEgo` reaches the model.** Generation 0.9.x takes speed as an input; ours never did, so the vision
  pipeline had no reason to know it. It now comes from `vehicle/state` at 100 Hz. Until this drive the
  input was a constant zero, which was out-of-distribution on every single frame;
* **microphone audio is recorded with the bag.** One button: while logging runs, `audio_<t>.m4a` is
  written into the bag directory, AAC mono 64 kbit/s, ~28 MB per hour. The name carries the start time on
  the same monotonic clock as every message in the bag, so aligning a sound to a frame is a subtraction.
  Say what you feel out loud — a spoken "вот здесь дёрнуло" is timestamped better than the memory of it.

### Reverting

`vision.model_runner` → `onnx`, or the radio button in Parameters. Nothing else changed in the control path.

---

## Previous drive (prepared 2026-08-07, evening — camera)

### The question

**Is the vision rate held down by the camera's auto-exposure, and does metering the road instead of the sky
give the rate back?** The previous drive answered ALKA (it works, see below) and left one complaint standing:
late and jerky steering on sharper bends. Measuring that drive found the mechanism, and it is not in the
controller.

### What the last drive established, so this one does not re-ask it

Always-on lateral (ALKA) is unconditional in the code — not a variable. On run `2026_08_07_19_04_05`:
`alternative_experience = 17`, `tx_blocked` never incremented over 50.2 min, and `controls_allowed` was
**0.0 %** for the whole drive, so every actuated frame came from bit 16. Assist presence went 2.6 → **79.1 %**
at 5–8 m/s, 18.3 → **91.9 %** at 8–12, 65 → **99.9 %** at 12–16.

Steering quality on that drive, above 23 km/h, excluding frames we withheld:

| segment | \|angle error\| median | p90 | setpoint step median | steps > 1.67° |
|---|---|---|---|---|
| straight | 0.30° | 0.81° | 0.09° | 0.1 % |
| gentle arc R > 167 m | 0.69° | 4.27° | 0.26° | 2.0 % |
| arc R 83–167 m | 1.88° | 8.70° | 0.49° | 7.9 % |
| sharp R < 83 m | 8.50° | 25.87° | 0.99° | **32.5 %** |

1.67° of error alone rails the PID, so on a sharp bend a third of the frames deliver a setpoint jump larger
than the controller's whole linear range, in one step. That is "рывками и запоздало", and its size is set by
how often the plan is refreshed.

### The changes, and why three of them share one drive

| flag | value | what it changes |
|---|---|---|
| camera AE (code, unconditional) | on | fixed fps range + metering region on the road |
| `vision.nnapi_fp16` | **false → true** | inference in half precision |

**The camera fix and fp16 are one change with two halves, and testing either alone gives a null result.**
Measured on run `2026_08_07_19_04_05`: pipeline work is 7.4 ms prep + 44.7 ms inference = **52 ms**, the camera
interval is **67 ms**, and **94 % of cycles drop nothing**. A work-bound pipeline drops frames; ours does not,
so the camera is the bound. But fixing the camera to 33 ms leaves 52 ms of work — we would take every *second*
frame and land at 15 Hz, barely better than today. Only with fp16 does work fall to ~32 ms, just inside one
camera period, and the pipeline can take every frame:

| | work | camera period | frames taken | rate |
|---|---|---|---|---|
| now | 52 ms | 67 ms | every | 13.5 Hz |
| camera fix alone | 52 ms | 33 ms | **every second** | **15 Hz** |
| camera fix + fp16 | ~32 ms | 33 ms | every | **~25 Hz** |

fp16 itself was verified offline before today (`bag_fp16_ab.py`, 200 frames): lane centre moved 0.027 m, plan
offset 0.037 m, line probabilities *rose* and the σ tail shrank. Not free — frame-to-frame divergence is
0.05 m median, 0.20 m p95 — but no degradation.

**The lateral control law now runs in `Control` at a fixed 100 Hz** (upstream `Ratekeeper(100)`), so the setpoint follows current speed between vision frames by construction. The old `lat_recompute_setpoint` flag, which emulated this inside the planner, no longer exists.

**What still cannot share a drive:** two changes to the command for the *same* reference. `use_learned_params`
therefore stays off.

### The camera change in detail

Two edits in `CameraHandler.java`:

1. **`pickTargetFpsRange` preferred any range *covering* the target.** With no `[30,30]` available it accepted
   e.g. `[15,30]`, and a lower bound of 15 lets the AE halve the frame rate to lengthen exposure. It now
   prefers a **fixed** range (lower == upper), highest at or below the target, and logs every range the device
   offers. flowpilot pins `Range(20,20)` for the same reason;
2. **`CONTROL_AE_REGIONS` on the road.** Exposure was computed over the whole frame including sky, so the road
   was underexposed and the AE lengthened exposure to compensate.

**The port is deliberately not literal.** flowpilot computes its rectangle from the *frame* size (1280×720),
but the field is in **active-array** coordinates — on a 4000×3000 sensor their rectangle lands in the top-left
corner, i.e. in the sky, the exact thing being avoided. `roadMeteringRegions` finds the crop the sensor uses
for our aspect ratio and takes the same fractions inside it (x 0.40–0.60, y 0.50–0.70), preferring the
pre-correction active array because we disable distortion correction.

### What to watch, and what would falsify each part

The exposure telemetry was added for exactly this — the mechanism is falsifiable from `logcat` alone:

```
AE fps ranges available: ...          ← what this phone actually offers
Set target FPS range 30-30            ← must be FIXED; if it logs 15-30, the device has no fixed range
AE metering on road: array ... -> region x,y w×h
AE: exposure 8.3 ms, frame duration 33.3 ms (30.0 fps), iso 400     ← every 5 s
```

| part | trace in the bag | falsified if |
|---|---|---|
| camera AE | `interval_ms`, exposure in logcat | exposure falls but the interval does not — then something else holds the camera and this reasoning is wrong |
| fp16 | `infer_ms` | it does not drop below ~30 ms; then the NNAPI path silently fell back to CPU |
| setpoint recompute | fraction of chassis ticks where `desired_swa_deg` moved with no new vision frame | that fraction stays ~0 — the flag did not reach the service |

**The assist gate is unconditional**, and with always-on lateral it should almost never close. The
ten-second reverse-gear check below is still the only thing that exercises it end to end.

## Artefact verification (done, 2026-08-07 23:35)

| check | result |
|---|---|
| C++ tests | **188 passed**, 1 skipped (`ZMQIMUTest`, needs a running app) |
| drive-discipline test | `ShippedConfig.AtMostOneCommandChangeIsEnabledAtATime` passes — nothing in the command-changing set is on |
| native lib | rebuilt **23:35**; the binding added for the offline harness was the only source newer than the previous lib, and functionally irrelevant to the app — rebuilt anyway, because "irrelevant, probably" is how a stale lib ships |
| lib inside the APK | sha256 `b0a678ae6c0383b46a869635…`, byte-identical to disk |
| camera change inside the APK | 3 marker strings in `classes*.dex` (`AE metering on road`, `AE fps ranges available`, `AE: exposure`) |
| `config.json` inside the APK | `nnapi_fp16 **true**`, `cruise_buttons false`. Always-on lateral, the assist gate, the parameter learner and the controller reading it are no longer switches: they are unconditional in the code as of 2026-08-13, and the config no longer carries them |
| APK signature | signer SHA-256 `79836abfcbc278ce270d2a68…`, matches `~/.android/debug.keystore` |

That signature row is not paranoia: a container build that created its own debug keystore once forced an
uninstall. What that costs is listed under «Install» — and it is less than this file used to claim.

**The config changed this time, so the `run-as rm files/config.json` step below is mandatory.** Without it the
phone keeps its copy from the ALKA drive, where both new flags are `false`, and the drive would test the
camera change alone — which the section above explains is a null result by construction.

## Install

```bash
adb devices                     # the phone must appear; it was not connected when this was prepared
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

If `install -r` fails with `INSTALL_FAILED_UPDATE_INCOMPATIBLE` the signature check above was wrong — stop
and re-check rather than uninstalling.

**What an uninstall actually costs — corrected 2026-08-08, the old claim here was wrong.** This file used to
say uninstalling "discards the calibration". It does not: the camera calibration is **never persisted**. The
only writes to `filesDir` in the whole Java layer are asset copies (`AdasAppHandler:205`, `RuntimeParams:60`
and the two model caches) — no `SharedPreferences`, no calibration file, in Java or in C++. That is
deliberate: the camera moves at every mount, the learned yaw went +1.67° → +0.10° between drives, so a prior
from last time is no better than the default, and convergence takes 30–60 s at a cost of 0.02 m of offset on
straights.

| uninstall removes | cost |
|---|---|
| `config.json` | none — it is the one guaranteed way to get the new config, stronger than the `run-as rm` below |
| the `.dbc` | none, re-copied |
| extracted `supercombo.onnx` / `traffic_yolo.onnx` | ~250 MB unpacked again, so the first launch is slow |
| granted permissions (camera, storage, USB) | **must be re-granted by hand** |

Bags are **not** affected: they are written to `Environment.getExternalStorageDirectory()/adas_logs`, outside
the app sandbox.

So `install -r` is still preferred — not to save calibration, but to keep permissions and avoid re-unpacking
the models. Uninstalling is a legitimate fallback when you want certainty that the config is fresh.

### The config in the APK is not the config that runs — do this or the drive tests nothing

`AdasAppHandler.java:89` calls `ensureAssetCopied(this, AdasConfig.ASSET, /*force=*/false)`, and that helper
returns early when `filesDir/config.json` already exists. So the app reads the copy made the **first** time it
ever ran, and `install -r` never refreshes it. Every "config.json inside the APK" row in the table above
describes the asset, not what the phone will load.

Delete the stale copy and let the app re-copy it:

```bash
adb shell am force-stop ai.flow.adas
adb shell run-as ai.flow.adas rm -f files/config.json
adb shell run-as ai.flow.adas ls -l files/          # config.json must be gone
# start the app, then confirm it copied the new one:
adb logcat -d | grep "Copied asset config.json"
```

`run-as` works because the build is debuggable; the app's own files are otherwise unreachable without root.
This touches only `config.json` — the learned calibration lives elsewhere in `filesDir` and is left alone.

Then confirm the values actually loaded, which is the only check that means anything:

```bash
adb logcat -d | grep -E "alt_exp=|LaneKeepService: controller=|learn_params="
```

`alt_exp=17` is the one that proves `lat_always_on` reached the native side.

To rebuild the APK on this machine, note that `~/.gradle/caches` is owned by **root** (something once ran
gradle under sudo), so the wrapper cannot write its lock. Either fix it once with

```bash
sudo chown -R anatoly:anatoly ~/.gradle
```

or use a writable copy, which is what this build did:

```bash
GRADLE_USER_HOME=/path/to/writable/gradle_home \
  ~/.gradle/wrapper/dists/gradle-8.10.2-bin/*/gradle-8.10.2/bin/gradle assembleDebug
```

---

## Before moving

1. **Mount and camera.** The phone in its usual windscreen position — the intrinsics and the mount prior
   assume it. A moved phone invalidates the comparison against the previous run, which is the whole point of
   changing one variable.
2. **Let the calibration converge** before judging anything lateral. Online VP/pose calibration needs
   straight road first; the saved calibration is reloaded, so this is faster than the first ever run but not
   instant.
3. **Confirm the panda took the ALKA bit — this is the check that gates the whole drive.** In `adb logcat`:
   ```
   Panda alt_exp=17 set before VW safety (disengage_on_gas=1 alka=1)
   Panda initial safety want=15 got=15 ... alt=17 ...
   ```
   `alt=17` is the value read back **from the device**, not what we asked for. If it reads `1`, the bit did not
   stick and the drive is the old behaviour — not dangerous, just not the experiment.
4. **Then, with the ignition on and the cruise main switch on but not engaged, watch `tx_blocked`.** It is in
   `panda/health` and in the same logcat line. It must stay **0**. If it climbs while the assist is trying to
   steer, bit 16 is not ALKA on this unit: stop, set `lat_always_on` back to `false`. Zero here with the
   assist steering is the positive result.
5. **Verify the feedback path end to end, stationary, in ten seconds.** With the ignition and the cruise main
   switch on, shift into **reverse** and watch `adb logcat`:
   ```
   LaneKeep: panda is not passing torque (controls_allowed=0, known=1) — command cleared, PID reset
   ```
   then shift back to drive:
   ```
   LaneKeep: torque reaching the rack again
   ```
   That single pair exercises the whole chain — `PandaService` computing the gate, publishing
   `lat_actuation_allowed` on `panda/health`, `LaneKeepService` receiving it, clearing the command and
   resetting the PID — which nothing else on this drive will, because with always-on lateral the gate is open
   almost all the time. If the lines never appear, the feedback is not reaching the lane-keep service and the
   `no_assist` status in the bag will be meaningless.

6. **Confirm the log line** for the observer:
   ```
   LocalizationService → localization/pose  sources: gps[...] ... learn_params=1
   ```
   If it reads `0`, the APK on the phone is not this one.
7. **Confirm recording.** `logging.record_camera_images` is `true`, so the bag is large — check free space
   before a long route.
8. **Hands on the wheel from the start.** The car now steers below 30 km/h and while braking, and for the
   first 30–60 s the camera calibration is still converging.

---

## What the route must contain

Two questions, two requirements, and they are compatible.

**For always-on lateral:** at least one bend taken **hands off** where it is safe to do so — that is the
population no bag we have contains, and it is what decides whether the 6.4° arc error is ours or the driver's.
Beyond that, the route must spend real time **below 30 km/h with the cruise main switch on but not engaged** — city streets, roundabouts, the approach and exit of slow bends. That is the regime the assist
has never covered and the whole reason for the drive; a motorway run would show almost nothing, because there
`controls_allowed` was already true. Include the same kind of slow sharp bend that produced the original
complaint on run `2026_08_06_18_27_12`. Arcs in *both* directions, and straight sections for the baseline.

**For the learner (the observer):** corners at **several different speeds**. This is the specific lesson from
today's replay: the bank term enters the yaw prediction as `g·sin(φ)/v` and stiffness through
`1/(1 − slip·v²)`, so speed variety is what separates the parameters from each other and from the road. A
long stretch of motorway at one speed produces a parameter fitted to that stretch's camber. It needs ≥ 500
accepted samples, and a sample is accepted at v ≥ 5 m/s with |SWA| ≤ 45° in steady state.

---

## After the drive, in this order

```bash
cd scripts
NEW=../adas_logs/<new_run>
OLD=../adas_logs/2026_08_06_18_27_12      # the baseline for assist presence

# 0. Did the pipeline stay healthy at all — frame rate, drops, stage latency.
python3 bag/bag_middleware_stats.py $NEW
python3 tools/latency.py $NEW

# 1. The gate question, and it comes first because everything else is conditional on it.
#    tx_blocked must be flat at 0; alternative_experience must read 17; and the assist must be present in
#    the 5-12 m/s bands where run 08-06 had 2.6 % and 18.3 %.
python3 bag/bag_topdown_video.py $NEW --list-assist
python3 bag/bag_override_episodes.py $NEW      # HCA_01.HCA_Active from the bus, independent of our own flags

# 2. Steering quality restricted to frames that were actually actuated — the comparison that was never
#    honest before. Same script on both bags; on the old one it will cover far fewer frames, which is
#    itself the result.
python3 bag/bag_controller_ab.py $NEW
python3 bag/bag_controller_ab.py $OLD

# 3. Arc offset, at the shipped window. Read it as this drive's baseline rather than as a comparison:
#    with the assist present in frames that used to be excluded, it is not measuring the same population.
python3 bag/bag_arc_offset.py $NEW --std-range 20 --blend 0.6 --std-good 0.3 --std-bad 1.5 \
    --center-force 0.0 --weight-by-std --cache /tmp/new20.npz

# 4. Perception σ — unaffected by actuation, so this one *is* comparable across drives.
python3 bag/bag_lane_sigma_ab.py $NEW $OLD

# 5. The observer: what did it learn live, and does it agree with the offline replay?
python3 bag/bag_params_learner.py $NEW
```

Step 1 carries the weight, and it is the one step where a negative answer ends the analysis: if the panda was
blocking frames, the rest of the bag describes the old behaviour with extra logging.

Steps 2 and 3 are deliberately not framed as before/after. The assist gate changes *which frames exist* in the
actuated population, so a drive-to-drive delta mixes "steers better" with "steers more often". The clean
comparison is within this bag: actuated frames at 5–12 m/s against actuated frames at 16–22 m/s, where the old
drive already had 87 % presence and therefore a trustworthy number.

`bag/bag_arc_offset.py --std-range` matters here even though the window is not the variable this time: it used to
take the median σ over the *whole* lane line, which is a different number from the one the controller gates on,
and an analysis that disagrees with the running code cannot be used to judge that code. Pass 20 to match what
ships.

Step 5 has two halves worth keeping apart. `bag/bag_params_learner.py` replays the *Python mirror* of the filter;
the bag also carries what the *C++* filter published live, in
`localization/pose.learned_stiffness_factor`. They should agree closely — if they do not, one of the two
transcriptions is wrong and that is a finding in itself.

---

## Caveat added 2026-08-06 after run `2026_08_06_18_27_12`

**Every lateral tracking number in this file, and in the backlog before that run, mixes actuated and
non-actuated frames.** The assist is only allowed while the stock cruise is engaged (panda's
`controls_allowed`), and on that drive it was engaged 29.3 % of the time while the lane-keep service reported
itself steering throughout — 49 % of "steering" frames had no torque on the bus at all. See §4a of the
backlog.

Consequences for the criteria below:

* the **perception** numbers — σ, σ p90, σ veto, blending weight — are unaffected. They are computed from
  `vision/lanes` and do not depend on actuation;
* the **arc-offset / tracking** numbers are contaminated and must be recomputed restricted to frames where
  `HCA_01.HCA_Active` is set on the bus. Until `bag/bag_arc_offset.py` gates on that, treat its tracking columns
  as a lower bound on how well the controller actually steers;
* so a drive that shows "worse arcs" may only be showing "less cruise engaged", which is a property of the
  route and the driver's right foot, not of any knob in this file.

---

## What counts as an answer

**The camera — and the criterion is the frame interval, not a feeling.**

| quantity | drive 08-07 19:04 | what would count as working |
|---|---|---|
| `infer_ms` median | 44.7 | **≤ 30** — fp16 took effect |
| `interval_ms` median | **67.0** | **≤ 40** — the camera stopped stretching exposure |
| `frames_dropped` per processed frame | 0.06 | staying near zero means work fits inside the camera period; a rise means we are work-bound again |
| vision rate | 13.46 Hz | **≥ 20 Hz** |
| exposure (logcat, every 5 s) | not recorded before | should fall; if it does not, the sky was not the cause |
| lane σ, matched subset | 0.273 median / 0.878 p90 | must not get *worse* — fp16 moved the plan 0.037 m offline, so this is the guard on it |

**Revert the camera change if** the road is visibly blown out or crushed in the recorded frames, or σ gets
worse. Correct exposure on the road is the point; a pinned frame rate bought with an unusable image is not.

**Then the arc numbers, and this is the reason for the drive.** Re-run the table from the last drive above
23 km/h excluding withheld frames. The prediction is specific: **the setpoint step should shrink roughly in
proportion to the rate**, so the share of steps above 1.67° on sharp bends should fall from 32.5 % toward
20 % if the rate reaches 18–20 Hz. If the rate improves and the step share does not, then the step is set by
the scene changing rather than by the interval, and the whole "vision rate is the root" argument is wrong —
which would send this back to the reference itself, not to the loop.

**Route:** the same roads as `2026_08_07_19_04_05`, and at a comparable time of day. Exposure is the variable,
so a bright-daylight drive would answer a different question. Include sharp bends, and **hands off through at
least one bend** where it is safe — that population still does not exist in any bag.

## Caveat that this drive is meant to retire## Caveat that this drive is meant to retire

The section above dated 2026-08-06 says every lateral tracking number mixes actuated and non-actuated frames.
Two things changed since:

* `control/lane_keep_debug` now carries `assist_allowed` and `assist_known`, so the split is recorded in the
  bag rather than reconstructed from `panda/health` timing;
* with `lat_always_on` on, most frames should be actuated, so the mixture stops being the dominant term.

`bag/bag_arc_offset.py` still does not gate on actuation. Until it does, use `assist_allowed` from the debug topic
to filter before reading its tracking columns — and the analysis helper `vis.bag_io.lateral_actuation_on`
exists for exactly this, taking `lat_actuation_allowed` when present and falling back to `controls_allowed`
for older bags.
