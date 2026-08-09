# From Frame to CAN

![Simplified pipeline diagram](figures/pipeline_simple.png)

Engineering version with implementation details: `docs/IMAGE_TO_CAN_PIPELINE.md`.

## Follow one frame, with a clock

The stages below are easier to hold onto if you watch a single frame move through them. Each number is a
measured median from a 28-minute night run. Per-stage medians do not have to add up to cumulative medians —
these are chosen to land on the two cumulative figures `latency.py` actually reports, 54 ms to the model
output and **79 ms to the command**, so the arithmetic is checkable rather than decorative.

```python
# Measured medians, run 2026_08_06_00_36_42. "own" is the stage's own cost; the clock accumulates.
STAGES = (
    ("capture (Camera2 YUV, capture_ts stamped)", 0.0),
    ("delivery to VisionPipeline", 0.0),          # instrumented from 2026-08; was invisible before
    ("geometric warp to 512x256", 7.0),
    ("Supercombo inference (OrtSession.run)", 45.6),
    ("parse heads -> vision/lanes", 1.0),
    ("Java -> ZMQ -> ZmqBridgeService", 5.0),      # of which ~5 is the 10 ms ingress poll
    ("TopicConvert -> vision/path", 1.0),
    ("LaneKeepService -> controls/steer", 19.4),
    ("PandaService -> HCA_01 on the wire", 10.0),  # its own 10 ms TX timer
)

clock = 0.0
print(f"{'stage':>44} {'own':>6} {'clock':>7} {'car has moved':>15}")
for name, own in STAGES:
    clock += own
    print(f"{name:>44} {own:>5.1f} {clock:>6.1f} {22.0 * clock * 1e-3:>13.2f} m")
print(f"\nAt 22 m/s the command that reaches the rack was computed for a road position {22.0 * clock * 1e-3:.2f} m back.")
print("That is what fp_steer_delay_s compensates, and why it is a feedforward input rather than a nicety.")
```

Two of those lines deserve attention.

**Delivery is 0.0 because it was invisible.** Until 2026-08 `capture_ts_ms` was the earliest timestamp on a
frame, so a frame that arrived 40 ms late and a frame that waited 40 ms for a busy inference thread looked
identical. `submit_ts_ms`, `pickup_ts_ms` and `frames_dropped` now separate them — see
[Latency](../Latency/Overview.md).

**The ZMQ hop costs about half a poll interval.** It is one of only two places in the whole chain that polls,
and the other is the CAN egress. Everything between them is notified, not polled —
[Middleware](./Middleware.md) has the measurement.

## What happens between frames

Vision runs at 13.24 Hz. The actuator runs at 100 Hz, because HCA needs a strict cadence or the EPS drops the
command. So for roughly seven ticks out of eight, the panda transmits a command derived from a *reference it
has already used*.

That is correct behaviour and it creates a trap for anyone reading logs:

```python
VISION_HZ = 13.24
TX_HZ = 100.0
FRESH_WINDOW_MS = 50.0        # latency.py keeps commands within this of their vision timestamp

vision_period_ms = 1000.0 / VISION_HZ
per_frame = TX_HZ / VISION_HZ
print(f"vision period {vision_period_ms:.1f} ms, {per_frame:.1f} actuator ticks per vision frame")
print(f"only the first tick uses a brand-new reference; the rest reuse it")
print()
# The freshness filter is a time window, not "one tick per frame" — so what it keeps is the fraction of
# the vision period that falls inside the window.
kept_predicted = min(1.0, FRESH_WINDOW_MS / vision_period_ms)
print(f"predicted share passing a {FRESH_WINDOW_MS:.0f} ms freshness filter: {100 * kept_predicted:.0f} %")
print(f"measured on the run: 113 723 kept of 168 107 = {100 * 113723 / 168107:.0f} %")
print()
print("Compute e2e latency over the raw stream instead and the republishes drag the number up, because")
print("their vision_ts is older than the command that carries it.")
```

The fast loop is not idle work — it is the angle PID closing on the *measured* steering angle at 100 Hz,
which is what makes the rack follow smoothly. But it means "the command is fresh" and "the picture is fresh"
are different claims, and only the second one matters for safety. That distinction is why the staleness gate
is keyed on the frame's capture timestamp rather than on the command's age, and why a 250 ms HCA command
timeout did not protect against a 75-second-old plan.

## Stage by stage

### 1. Capture

`CameraHandler` (Camera2):

* YUV buffer, nominally **1280×720**;
* `capture_ts = TimeUtil.nowMs()` (BOOTTIME);
* `acquireLatestImage()` — drops stale frames.

Frame branches: preview UI; VisionPipeline queue; when logging — JPEG preview in bag (**not** warped 512×256 network input).

### 2. Geometric warp

`ModelCalibWarp` brings frame to medmodel **512×256** using intrinsic prior and calibration RPY. Pitch/yaw error shifts metric lane interpretation and causes systematic CTE.

### 3. Supercombo inference

`SupercomboOnnxRunner` / `VisionPipeline` on separate thread:

* temporal stack (2 frames) + RNN state;
* output → parse → topic **`vision/lanes`** (+ odometry / model_long);
* stamps `infer_ts_ms`, `infer_duration_ms`.

**13.24 Hz** measured at 30 fps camera (dt ≈ 68 ms median, 75 mean). Under thermal throttle or with the
traffic YOLO competing for the SoC it drops to single-digit Hz and the lateral metronome breaks. The rate is
quantised by the camera period rather than set by the work — [Latency](../Latency/Overview.md).

### 4. Java → native bridge

Protobuf `ZMQMessage` PUB on `:5555`. `ZmqBridgeService` reads IN and publishes to internal Middleware. Split: Java — camera/ORT/log, C++ — control/calib/panda.

### 5. Lane-keep

`LaneKeepService` consumes path polyline and chassis ($v$, actual SWA). Controller from config:

* `pp` — Pure Pursuit + angle PID;
* `mpc` — VisionPilot path MPC;
* `fp` — stock-like time-domain MPC (default on road).

Publications: `control/lane_keep`, `controls/steer`.

### 6. Actuation

`PandaService` TX ~100 Hz, `HCA_01` frames. Gates: ignition, `controls_allowed`, command age ≤ ~250 ms.

## Parallel topics

| topic | role |
|---|---|
| `calibration/camera` | live RPY → warp |
| `phone/stats` | CPU %, thermal |
| `middleware/stats` | native timer lags |
| `vision/traffic_dets` | YOLO (usually off for LK) |

## Rates

| signal | measured |
|---|---|
| camera | 30 fps requested, 33 ms period confirmed |
| `vision/lanes` | **13.24 Hz** (was 11.29 at 20 fps) |
| lane_keep publish | ≈ vision, 13.24 Hz |
| `controls/steer` | ~100 Hz; 68 % pass the 50 ms freshness filter, 32 % are republishes on an older reference |
| panda TX | 100 Hz, its own timer |

Vision rate sets the lateral decision period; the 100 Hz loop only closes the angle. Hz and e2e degradation
must be recorded **before** comparing controllers, because a controller evaluated on a 3 Hz window is not the
controller you shipped.

Safety icons (`safety/warn`) are parallel to lane-keep — see [FCW / AEB / LDW](../Safety/Warnings.md).

<!-- next-chapter -->
---

**Next:** [Vision — overview](../Vision/Overview.md)
