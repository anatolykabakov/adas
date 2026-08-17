# From Frame to CAN

![Simplified pipeline diagram](figures/pipeline_simple.png)

Engineering version with implementation details: `docs/IMAGE_TO_CAN_PIPELINE.md`.

## Follow one frame, with a clock

The stages below are easier to hold onto if you watch a single frame move through them. Each number is a
measured median from a 29-minute night run. Per-stage medians do not have to add up to cumulative medians —
these are chosen to land on the two cumulative figures `tools/latency.py` actually reports, 22 ms to the model
output and **52 ms to the command**, so the arithmetic is checkable rather than decorative.

```python
# Measured medians, run 2026_08_16_23_59_45, OnePlus 7T, thneed runner.
# "own" is the stage's own cost; the clock accumulates.
STAGES = (
    ("capture (Camera2 YUV, capture_ts stamped)", 0.0),
    ("delivery to VisionPipeline", 0.0),           # median 0, p95 0 — instrumented since 2026-08
    ("wait in the inference queue", 0.0),          # median 0, mean 1.0 — nothing is queueing up
    ("geometric warp to 6x128x256 (OpenCL)", 4.6), # prep_ms; was 12.1 on the CPU
    ("Supercombo inference (thneed, GPU)", 17.6),  # was 45.6 through ONNX Runtime
    ("parse heads -> vision/lanes", 0.0),
    ("Java -> ZMQ -> ZmqBridge -> proto_convert", 4.0),
    ("Planner -> control/lat_plan, control/lane_keep", 5.0),
    ("Control -> controls/steer", 21.0),
    ("Platform -> HCA_01 on the wire", 10.0),      # its own 10 ms TX timer
)

clock = 0.0
print(f"{'stage':>46} {'own':>6} {'clock':>7} {'car has moved':>15}")
for name, own in STAGES:
    clock += own
    print(f"{name:>46} {own:>5.1f} {clock:>6.1f} {22.0 * clock * 1e-3:>13.2f} m")
print(f"\nAt 22 m/s the command that reaches the rack was computed for a road position {22.0 * clock * 1e-3:.2f} m back.")
print("That is what fp_steer_delay_s compensates, and why it is a feedforward input rather than a nicety.")
```

Three of those lines deserve attention.

**Inference is 17.6 ms because the model runs on the GPU.** The same network through ONNX Runtime costs
45.6 ms on this phone. Both paths carry supercombo 0.9.7 — the same file, converted — so the choice is a
speed choice and nothing else; see [Supercombo](../Vision/Supercombo.md).

**Delivery and queueing are both 0.0, and that is the interesting part.** They are separate measurements
(`submit_ts_ms`, `pickup_ts_ms`, `frames_dropped`), added in 2026-08 precisely because a frame that arrived
40 ms late and a frame that waited 40 ms for a busy inference thread used to look identical. On this run the
camera delivers on time and inference is never the thing being waited on: **0 dropped frames in 52 690**,
100 % of cycles dropping nothing. When that stops being true, the two numbers tell you which half broke —
many drops with short delivery means inference is too slow, few drops with long delivery means the camera is
arriving late.

**The ZMQ hop costs about half a poll interval.** It is one of only two places in the whole chain that polls,
and the other is the CAN egress. Everything between them is notified, not polled —
[Middleware](./Middleware.md) has the measurement.

## What happens between frames

Vision runs at 30.0 Hz — one frame per camera period, nothing skipped. The actuator runs at 100 Hz, because
HCA needs a strict cadence or the EPS drops the command. So for roughly two ticks out of three, the panda
transmits a command derived from a *reference it has already used*.

That is correct behaviour and it creates a trap for anyone reading logs:

```python
VISION_HZ = 30.01
TX_HZ = 100.0
FRESH_WINDOW_MS = 50.0        # tools/latency.py keeps commands within this of their vision timestamp

vision_period_ms = 1000.0 / VISION_HZ
per_frame = TX_HZ / VISION_HZ
print(f"vision period {vision_period_ms:.1f} ms, {per_frame:.1f} actuator ticks per vision frame")
print(f"only the first tick uses a brand-new reference; the rest reuse it")
print()
# The freshness filter is a time window, not "one tick per frame" — so what it keeps is the fraction of
# the vision period that falls inside the window.
kept_predicted = min(1.0, FRESH_WINDOW_MS / vision_period_ms)
print(f"predicted share passing a {FRESH_WINDOW_MS:.0f} ms freshness filter: {100 * kept_predicted:.0f} %")
print(f"measured on the run: 171 097 kept of 175 573 = {100 * 171097 / 175573:.0f} %")
print()
print("Compute e2e latency over the raw stream instead and the republishes drag the number up, because")
print("their vision_ts is older than the command that carries it.")
```

At 13 Hz — the rate this chapter reported before the model moved to the GPU — a third of the commands failed
that filter, and the freshness window itself was doing visible work. At 30 Hz the vision period is shorter
than the window, so the filter now drops only the genuinely stale republishes: 4 476 of 175 573. The lesson
survives the improvement: "the command is fresh" and "the picture is fresh" are different claims, and only
the second one matters for safety. That is why the staleness gate is keyed on the frame's capture timestamp
rather than on the command's age, and why a 250 ms HCA command timeout did not protect against a
75-second-old plan.

The fast loop is not idle work — it is the angle PID closing on the *measured* steering angle at 100 Hz,
which is what makes the rack follow smoothly.

## Stage by stage

### 1. Capture

`CameraHandler` (Camera2):

* YUV buffer, nominally **1280×720**;
* `capture_ts = TimeUtil.nowMs()` (BOOTTIME);
* `acquireLatestImage()` — drops stale frames;
* the lens is pinned: autofocus off, `LENS_FOCUS_DISTANCE = 0` (infinity). A drive lost to a defocused
  lens is in [Calibration](../Calibration/Overview.md).

Frame branches: preview UI; VisionPipeline queue; when logging — JPEG preview in bag (**not** the warped
network input).

### 2. Geometric warp

`ModelCalibWarp` brings the frame to the model's input — two frames of six planes at 128×256 — using the
intrinsics and the calibration RPY. Pitch/yaw error shifts metric lane interpretation and causes systematic
CTE.

The warp runs as an OpenCL kernel on the GPU (4.6 ms; 12.1 ms when it ran on the CPU) and falls back to the
CPU where OpenCL is unavailable. The first frame after startup is computed **both** ways and compared
bit-for-bit: a wrong warp does not crash, it hands the network a plausible picture of the wrong road.

### 3. Supercombo inference

`VisionPipeline` on a separate thread, with two interchangeable runners:

* `SupercomboThneedRunner` — supercombo 0.9.7 in fp16 on the GPU, **17.6 ms**, the default;
* `SupercomboOnnxRunner` — the same network in fp32 through ONNX Runtime, 48.5 ms, the fallback.

Either way: temporal stack of two frames plus the recurrent state, output parsed into topic
**`vision/lanes`** (+ odometry / model_long), stamped with `infer_ts_ms` and `infer_duration_ms`.

**30.0 Hz** measured at a 30 fps camera (dt 33.0 ms median) — the rate is now quantised by the camera period
and nothing else. Before the model moved to the GPU it was 13.24 Hz, and the lateral metronome broke under
thermal throttle or when the traffic YOLO competed for the SoC. See [Latency](../Latency/Overview.md).

Both runners verify themselves at load: the network is run on zero inputs and the output signature is
compared with a reference measured offline. A runner that computes something else refuses to start rather
than reporting plausible numbers about the wrong road.

### 4. Java → native bridge

Protobuf `ZMQMessage` PUB on `:5555`. `ZmqBridge` reads IN and publishes to the internal middleware;
`utils/proto_convert.cpp` turns the wire messages into internal ones. Split: Java — camera / model / log,
C++ — planning, control, calibration, panda.

### 5. Plan and command

Three services divide the lateral loop, and the split is the point: each one is testable without the other
two.

* **`Planner`** (`services/planner.cpp`) consumes the path polyline and the chassis and produces a plan
  **in curvature**, never in steering angle. Which algorithm produces it is a config choice —
  `pp` (pure pursuit), `mpc` (VisionPilot path MPC), `fp` (stock-like time-domain MPC, the default on the
  road) — and the interface is identical whichever runs. Publishes `control/lat_plan`, `control/long_plan`,
  `control/lane_keep`.
* **`Control`** (`services/control.cpp`) is the control law and nothing else: curvature plus the chassis
  become a torque command, engagement, HUD pictograms, and the wish to press a cruise button. It knows no
  CAN address, no signal, no frame counter. Publishes `controls/steer`.
* **`Platform`** (`services/platform.cpp`) carries that intent onto the bus, and it names no brand — which
  car is behind the CAN is decided once, by `vehicle.name`, and reached through `platform::CarPlatform`.
  See [Porting](../../PORTING.md).

### 6. Actuation

`Platform` transmits `HCA_01` at ~100 Hz. Gates: ignition, `controls_allowed`, command age ≤ ~250 ms.

## Parallel topics

| topic | role |
|---|---|
| `calibration/camera` | live RPY → warp |
| `camera/intrinsics` | focal length and principal point, in full-frame units |
| `phone/stats` | CPU %, thermal |
| `middleware/stats` | native timer lags |
| `vision/traffic_dets` | YOLO (usually off for LK) |

## Rates

| signal | measured |
|---|---|
| camera | 30 fps requested, 33.0 ms period confirmed |
| `vision/lanes` | **30.01 Hz**, 0 frames dropped in 52 690 (was 13.24 Hz through ONNX) |
| `control/lane_keep` | 30.01 Hz, 31 ms after capture |
| `controls/steer` | ~100 Hz; 97 % pass the 50 ms freshness filter (was 68 %) |
| panda TX | 100 Hz, its own timer |

Vision rate sets the lateral decision period; the 100 Hz loop only closes the angle. Hz and e2e degradation
must be recorded **before** comparing controllers, because a controller evaluated on a 3 Hz window is not the
controller you shipped.

Safety icons (`safety/warn`) are parallel to lane-keep — see [FCW / AEB / LDW](../Safety/Warnings.md).

<!-- next-chapter -->
---

**Next:** [Vision — overview](../Vision/Overview.md)
