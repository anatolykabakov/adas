# From Frame to CAN

![Simplified pipeline diagram](figures/pipeline_simple.png)

Engineering version with implementation details: [`IMAGE_TO_CAN_PIPELINE.md`](../../IMAGE_TO_CAN_PIPELINE.md).

## 1. Capture

`CameraHandler` (Camera2):

* YUV buffer, nominally **1280×720**;
* `capture_ts = TimeUtil.nowMs()` (BOOTTIME);
* `acquireLatestImage()` — drops stale frames.

Frame branches: preview UI; VisionPipeline queue; when logging — JPEG preview in bag (**not** warped 512×256 network input).

## 2. Geometric warp

`ModelCalibWarp` brings frame to medmodel **512×256** using intrinsic prior and calibration RPY. Pitch/yaw error shifts metric lane interpretation and causes systematic CTE.

## 3. Supercombo inference

`SupercomboOnnxRunner` / `VisionPipeline` on separate thread:

* temporal stack (2 frames) + RNN state;
* output → parse → topic **`vision/lanes`** (+ odometry / model_long);
* stamps `infer_ts_ms`, `infer_duration_ms`.

Nominally ~**9–11 Hz** (dt ≈ 90 ms). Under thermal throttle or competing YOLO rate drops to single-digit Hz — lateral metronome breaks.

## 4. Java → native bridge

Protobuf `ZMQMessage` PUB on `:5555`. `ZmqBridgeService` reads IN and publishes to internal Middleware. Split: Java — camera/ORT/log, C++ — control/calib/panda.

## 5. Lane-keep

`LaneKeepService` consumes path polyline and chassis ($v$, actual SWA). Controller from config:

* `pp` — Pure Pursuit + angle PID;
* `mpc` — VisionPilot path MPC;
* `fp` — stock-like time-domain MPC (default on road).

Publications: `control/lane_keep`, `controls/steer`.

## 6. Actuation

`PandaService` TX ~100 Hz, `HCA_01` frames. Gates: ignition, `controls_allowed`, command age ≤ ~250 ms.

## Parallel topics

| topic | role |
|---|---|
| `calibration/camera` | live RPY → warp |
| `phone/stats` | CPU %, thermal |
| `middleware/stats` | native timer lags |
| `vision/traffic_dets` | YOLO (usually off for LK) |

## Rates

| signal | typical |
|---|---|
| preview | ~20–30 Hz |
| `vision/lanes` | **~9–11 Hz** |
| lane_keep publish | ≈ vision |
| panda TX | ~100 Hz (between vision frames — stale vision_ts) |

Vision rate sets lateral decision period. Hz/e2e degradation must be recorded **before** comparing controllers.

Safety icons (`safety/warn`) are parallel to lane-keep — see [FCW / AEB / LDW](../Safety/Warnings.md).

<!-- next-chapter -->
---

**Next:** [Vision — overview](../Vision/Overview.md)
