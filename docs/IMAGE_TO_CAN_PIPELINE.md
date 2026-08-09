# Chain: camera → CAN (HCA)

Detailed path from phone camera frame to Volkswagen MQB `HCA_01` frames on the bus via Panda.

Configuration this chain describes:

- model = `supercombo.onnx` (v0.8.13) + Java ONNX Runtime, NNAPI with CPU fallback;
- lateral control = `fp` (flowpilot MPC port) → κ → steering angle via vehicle model →
  angle-PID → HCA torque. All three available controllers are covered below;
- target vehicle = VW Golf 7 MQB.

Mechanism-level walkthrough of the **lateral** half of this chain, in Russian, with the measured numbers and
the three places a command dies silently: [`LATERAL_CHAIN_RU.md`](LATERAL_CHAIN_RU.md). This file stays the
topology map — where the messages go; that one explains why each step is shaped the way it is.

Related overviews: root [`README.md`](../README.md),
[`vision/README.md`](../app/src/main/java/ai/flow/adas/vision/README.md),
[`cpp/README.md`](../app/src/main/cpp/README.md), applicability limits —
[`CONTROLLER_LIMITS.md`](CONTROLLER_LIMITS.md).

---

## Flow overview

```
Camera2 YUV 1280×720
        │
        ├─► preview TextureView
        ├─► grayscale JPEG @ 640×360 ──► bag (camera)          [if logging]
        └─► ARGB Bitmap ──► VisionPipeline (background thread)
                              │
                              ├─ ModelCalibWarp (512×256)
                              ├─ SupercomboOnnxRunner (ORT)
                              ├─ parse → LaneLines (+ CameraOdometry)
                              ├─ LaneOverlayView (UI)
                              ├─ bag: vision/lanes (+ model_out)
                              └─ ZMQ PUB :5555  vision/lanes  (no model_out)
                                                 model/camera_odometry
        │
        ▼
 Native ZmqBridgeService  SUB :5555
        │
        ▼
 TopicConvertService
        ├─ vision/lanes  → vision/path   (LanePathMsg polyline)
        └─ vehicle/state → vehicle/chassis
        │
        ▼
 LaneKeepService
        ├─ PurePursuit(polyline, v) → δ_road (device Y right+)
        ├─ desired_SWA = steer_sign × δ × steer_ratio
        ├─ LatControlPID(desired, actual_SWA) → steer_norm
        ├─ control/lane_keep   (PP geometry)
        └─ controls/steer      (torque_cNm, enabled)
        │
        ▼
 PandaService @ 100 Hz TX
        ├─ safety: ignition + controls_allowed + cmd age ≤ 250 ms
        ├─ CarController → HCA_01 + LDW HUD
        └─ panda.can_send(...)
        │
        ▼
 CAN PT bus → EPS HCA
```

Parallel branches (not on the critical torque path, but affect RPY / overlay / bag):

| Branch | Input | Output |
|-------|------|--------|
| Panda RX | CAN | `vehicle/state`, `panda/health`, `can/rx` |
| GPS / IMU (Java) | sensors | `sensors/gps/location`, `sensors/imu` → localization / imu_calib |
| Camera calib | `model/camera_odometry` (+ optional UV) | `calibration/camera` → UI → Java warp |
| ZMQ OUT `:5556` | native algorithms | bag + overlay (`control/lane_keep`, `controls/steer`, …) |

---

## Stage 1. Frame capture

**Code:** `CameraHandler.java`

| Parameter | Value |
|----------|----------|
| API | Camera2 |
| Buffer format | `ImageFormat.YUV_420_888` |
| Resolution | `W×H = 1280×720` |
| Session outputs | `ImageReader` + `TextureView` preview |

On each `onImageAvailable`:

1. `acquireLatestImage()` (drop stale frames).
2. If `VisionPipeline` enabled (`nodes.vision_supercombo` and init OK):
   - `captureTs = TimeUtil.nowMs()` (BOOTTIME ms);
   - pack `YuvFrame` from `Image` planes;
   - `visionPipeline.submitYuv(yuv, captureTs)` (preferred; `submitBitmap` is legacy);
3. For bag (only if `Logger.isRunning()`):
   - grayscale bitmap → scale `SCALE_FACTOR=2` → **640×360**;
   - JPEG quality 70;
   - topic camera image + bag intrinsics (`bagFx/bagFy/…` from CameraCharacteristics, fy separate from fx).

**Important:** bag gets preview JPEG, **not** warped 512×256 model input. For offline ONNX re-run need either JPEG+intrinsics+RPY, or `vision/lanes.model_out`.

---

## Stage 2. VisionPipeline (inference queue)

**Code:** `vision/VisionPipeline.java`

- Separate `HandlerThread` `"SupercomboInfer"`.
- `busy` flag: while a frame is processing, new frames are **dropped** (not queued).
- ARGB copy → `SupercomboOnnxRunner.run` → `Result{lanes, pose}`.

After successful result:

| Action | Where | Note |
|----------|------|------------|
| `overlay.setLanes(lanes)` | UI | immediate |
| `ProtoUtils.createLaneLinesMessage(lanes, true)` | `Logger` / bag | **with** `model_out` (~6409 float) |
| `ProtoUtils.createLaneLinesMessage(lanes, false)` | `ZMQBridgeService.publishToNative` | **without** `model_out` (light control) |
| `createCameraOdometryMessage` | ZMQ + bag | if `publishPose` and pose.valid |

Topic names: `vision/lanes`, `model/camera_odometry`.

Warp calibration: `MainActivity.applyParamsToVision` / live RPY / inbound `calibration/camera` → `runner.setCalib(...)`.
Pose-calib gate internals: topic `calibration/camera_debug` (`CameraCalibDebug`, every cam-odom frame, including rejects).

---

## Stage 3. Calib warp (preprocess)

**Code:** `vision/ModelCalibWarp.java`

Goal — same as flowpilot `getWrapMatrix` / TransformCL for **medmodel**:

\[
M_{\text{model→cam}} = K_{\text{cam}} \cdot V \cdot R(\text{rpy}) \cdot (K_{\text{med}} \cdot V)^{-1}
\]

| Constant | Value |
|-----------|----------|
| Model size | 512×256 |
| Med focal | 910 |
| Med cy | 47.6 |
| `VIEW_FROM_DEVICE` | device (x fwd, y right, z down) → view |

`warpToModel`: for each model pixel sample from camera bitmap via \(M\) (out-of-bounds = black).

**Height / cam X/Y** in `RuntimeParams` do **not** affect network input — only overlay / C++ priors.

---

## Stage 4. ONNX Supercombo

**Code:** `vision/SupercomboOnnxRunner.java`

| Input | Size / meaning |
|------|----------------|
| Image tensor | 12×128×256 = 2 frames × 6 ch (YUV-like layout after warp+resize) |
| desire | 8 |
| traffic | 2 |
| rnn state | 512 (recurrent between frames) |

First useful output — after **2** frames (`hasPrev`).

Model: `/sdcard/adas_models/supercombo.onnx` → filesDir cache → `assets/supercombo.onnx` (`AdasConfig`).

### Output parsing (layout driving.cc, out≈6409)

| Slice | Content |
|------|------------|
| `[0:4955)` | PLAN — 5 MHP × (33×15 mean+std) + logit |
| `[4955:5483)` | 4 lanes × 33 × (y,z) + stds |
| `[5483:5491)` | lane probs — `sigmoid(prob[i*2+1])` |
| `[5491:5755)` | 2 road edges × 33 × (y,z) |
| further | pose / other → `CameraOdometry` |

Coordinate system **device**: X forward, **Y right+**, Z up (as flowpilot Parser).
Lane order in proto: leftFar, leftNear, rightNear, rightFar (near = indices 1 and 2).
Path on overlay — **best PLAN**, not mid-lane.

C++ path fusion (next stage) may blend mid-lane `lll+w/2`, `rll−w/2`
(simplified `get_stock_path`: no std-downweight / `CAMERA_OFFSET` / NLP).

---

## Stage 5. Publish to native (ZMQ IN)

**Code:** `ZMQBridgeService.java` + `ZmqBridgeService` (C++)

| Socket | Role |
|-------|------|
| Java PUB → `tcp://127.0.0.1:5555` | sensors / vision → native |
| Native PUB → `tcp://127.0.0.1:5556` | algorithms / panda → Java bag + UI |

Multipart format: `[topic UTF-8][ZMQMessage protobuf]`.

Java `publishToNative` writes to IN. Native `ZmqBridgeService` on ~10 ms timer reads IN and `publish` to internal Middleware bus with same topic.

Inbound from phone (typical):

- `vision/lanes`
- `model/camera_odometry`
- `sensors/imu`, `sensors/gps/location`
- (camera JPEG **not** required in native for lane-keep)

Outbound topics (native → Java), see `kZmqOutboundTopics`:

`can/rx`, `panda/health`, `vehicle/state`, `control/lane_keep`, `control/lane_keep_debug`, `localization/pose`, `calibration/camera`, `calibration/camera_debug`, `controls/steer`, `middleware/stats`, `control/long_plan`, `safety/warn`.

---

## Stage 6. TopicConvert: lanes → path, carState → chassis

**Code:** `topic_convert_service.cpp`, `utils/topic_convert.cpp`

### `vision/lanes` → `vision/path` (`laneLinesToPath`)

1. Base polyline from **plan_x / plan_y** (points with \(x \ge 1\)).
2. If near L/R lanes exist (indices 1 and 2) with soft-prob:
   - mid: `from_l = yl + w/2`, `from_r = yr − w/2`, \(w\in[2.6,4]\);
   - blend by probabilities (flowpilot `lane_planner` style, Y right+).
3. If both plan and lanes exist:
   `y = d_prob * y_lane + (1-d_prob) * y_plan`.
4. Else — lanes mid only or plan only.

Timestamps: `capture_ts` / `infer_ts` forwarded in us for latency in `control/lane_keep`.

### `vehicle/state` → `vehicle/chassis` (`carStateToChassis`)

From MQB decode (Panda):

- `speed_mps = v_ego`
- `steering_angle_deg`, `steering_pressed`
- `steer_rad = SWA_rad / steer_ratio`
- `yaw_rate`

Without correct SWA/pressed LatPID and driver override on MQB work incorrectly.

---

## Stage 7. LaneKeep: planner → steering angle → torque

**Code:** `lane_keep_service.cpp`, `flowpilot/lateral_mpc.cpp`, `utils/vehicle_model.h`,
`pure_pursuit.cpp`, `lat_control_pid.h`

Controller choice — `vehicle.lane_keep_controller` in `config.json`:

| key | planner | κ → wheel angle |
|---|---|---|
| `fp` (default) | flowpilot LatMpc port (N=16) + `get_lag_adjusted_curvature` | vehicle model with curvature shortfall (`vehicle_model.h`) |
| `mpc` | VisionPilot spatial MPC | kinematics |
| `pp` | Pure Pursuit (below) | pure pursuit geometry |

Common to all: below `min_control_speed_mps` command zeroed, then angle limited by
speed, then angle-PID converts to torque.

### 7.1 Pure Pursuit (controller `pp`)

Historical path without MPC; internal angle/torque loop (LatPID → HCA) shared with all.

On `vision/path` polyline + speed from chassis:

- lookahead \(L_d = \mathrm{clamp}(K_{dd}\cdot v, L_{d,\min}, L_{d,\max})\);
- rear-axle shift `pp_shift` (m back along X);
- target = lookahead circle intersection with polyline (\(x>0\));
- \(\delta = \arctan(2\,L\,\sin\alpha / L_d)\) → `steer_rad` (wheel angle, device frame).
  Code: `pure_pursuit.cpp` — `atan((2 * wheel_base * sin(α)) / lookahead)` (not `atan2(..., L_d²)`).

Live params from UI: `RuntimeParams` / sliders → `AdasAppHandler.applyLaneKeepParams` → JNI
`nativeUpdateParams` → `Middleware::setParameter` (ParamBag). Per-knob `nativeSetLaneKeepPp` /
`nativeSetSteerRatio` were removed.

### 7.2 VW sign

Device Y right+ → positive δ "right".
MQB EPS / HCA: torque **left-positive**.

```
desired_swa_deg = steer_sign * (steer_rad * 180/π) * steer_ratio
```

In `config.json`: `"steer_sign": -1.0`.

### 7.3 LatControlPID

On each chassis update and after lanes:

- `active` = `steer_output_enabled` ∧ target exists ∧ status ok ∧ chassis present;
- input: `desired_swa_deg`, `actual` SWA, `steering_pressed` (unwind I);
- output: `steer_norm ∈ [-1,1]` → `torque_cNm = round(steer_norm * max_torque_cnm)`.

Publications:

| Topic | Content |
|-------|------------|
| `control/lane_keep` | δ, Ld, target, κ, status, latency stamps (`steer_norm` geometric before PID overwrite on steer) |
| `controls/steer` | `torque_cNm`, `enabled` |

`control/lane_keep` goes to `:5556` → UI overlay (PP arc, HUD).
`controls/steer` subscribed by PandaService.

---

## Stage 8. Panda → CAN HCA

**Code:** `panda_service.cpp`, `volkswagen/carcontroller.cpp`, `mqbcan.cpp`

### RX / state

- ~50 ms timer: CAN RX → decode → `vehicle/state`, filtered `can/rx`.
- ~100 ms: `panda/health` (ignition, `controls_allowed`, safety mode, heartbeat).

### TX (`carControllerCallback` @ 10 ms)

Conditions for `latActive`:

1. Panda connected / comms healthy;
2. safety mode Volkswagen + ignition;
3. last `controls/steer` not older than **250 ms**;
4. `enabled` and torque ≠ 0;
5. `controls_allowed` (usually needs stock ACC engage).

Additionally CarController/decoder account for EPS HCA status, standstill and driver-torque
rate limits (see `carcontroller.cpp`, `mqb_car_state_decoder.cpp`).

Then `CarController::update`:

- rate/driver torque limits;
- `create_steering_control` → **HCA_01** (`0x126`) with CRC/counter;
- LDW HUD frames.

Without Panda USB / without `controls_allowed` chain through stage 7 is alive (vision + PP on UI), nothing goes to bus.

---

## Stage 9. Feedback to UI

**Code:** `MainActivity.onOutboundMessage`, `LaneOverlayView`

| Message | UI |
|-----------|-----|
| `control/lane_keep` | PP target / arc / status |
| `controls/steer` | torque bar, enabled |
| `panda/health` | HCA line (ignition / controls_allowed / ok) |
| `calibration/camera` | on success or `cal_percent≥50` → params RPY/K → warp + overlay |

CAN online LED: `panda/health` freshness ≤ 500 ms.

---

## Config and flags

`app/src/main/assets/config.json` (copied to filesDir on first start, **no force overwrite** — RuntimeParams edits persist):

| Key | Role |
|------|------|
| `nodes.panda` / `zmq_bridge` / `lane_keep` / `localization` / `camera_calib` | native services |
| `nodes.vision_supercombo` | Java ONNX pipeline |
| `vehicle.steer_ratio` / `steer_sign` / `wheelbase_m` | LaneKeep |
| `calibration.camera.*` | priors K / RPY / height |
| `supercombo_asset` | ONNX name in assets |

`RuntimeParams` (UI sliders) merged into same JSON for C++ and pushed live to PP via JNI.

---

## Bag / pull / offline

| Component | Purpose |
|-----------|------------|
| `BagLogger` + `Logger` | session under `/sdcard/adas_logs/...` |
| `pull_bags.sh` | `adb pull` → `./adas_logs/`, clear on device |
| `./scripts/run_bag_vis.sh` | `vis/interactive_visualizer.py` |
| `bag_overlay_lanes.py` | lane overlay on JPEG |
| `bag_lane_keep_offline.py` | PP via `pyadas.AdasApp` |
| `export_to_plotjuggler.py` | topic time series |

Typical bag topics: camera JPEG, intrinsics, IMU/GPS, `vision/lanes`(+model_out), odometry, outbound `vehicle/state`, `control/lane_keep`, `controls/steer`, `calibration/camera`, `panda/health`, …

Bag lane projection: **Y right+** → in visualizer often `y_sign=-1` for ISO left+ drawing (`supercombo_compare.py`).

---

## Sim (MetaDrive)

`./scripts/run_sim.sh` → `scripts/sim/main.py`:

1. MetaDrive RGB camera;
2. lanes = GT centerline **or** host `supercombo.onnx`;
3. `LaneKeepController` → Simulated `AdasApp.publish_chassis/lanes` → `step` → PP from **same** C++ `LaneKeepService`;
4. steer in MetaDrive; optional overlay.

No Panda/CAN in sim — tests vision→path→PP, not HCA.

---

## File map (critical path)

| File | Role |
|------|------|
| `CameraHandler.java` | capture |
| `VisionPipeline.java` | thread + publish |
| `ModelCalibWarp.java` | homography |
| `SupercomboOnnxRunner.java` | ONNX + parse |
| `LaneLines.java` | DTO ego xyz |
| `ProtoUtils.java` | protobuf ZMQMessage |
| `ZMQBridgeService.java` | Java ↔ ZMQ |
| `zmq_bridge_service.cpp` | ZMQ ↔ Middleware |
| `topic_convert*.cpp` | lanes→path, carState→chassis |
| `lane_keep_service.cpp` | PP + PID |
| `pure_pursuit.cpp` | geometry |
| `panda_service.cpp` | RX/TX orchestration |
| `carcontroller.cpp` / `mqbcan.cpp` | HCA_01 |
| `LaneOverlayView.java` | visualization |
| `MainActivity.java` | wiring params / calib / HCA UI |
