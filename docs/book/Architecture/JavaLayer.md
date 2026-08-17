# Java layer (phone shell)

C++ owns algorithms. **Java** owns the Android world: camera, the model runners, bags, UI, and the ZMQ bridge
into the middleware.

Package root: `app/src/main/java/adas/app/`, split by what a class talks to:

| package | contents |
|---|---|
| `adas.app` | `AdasAppHandler` (JNI), `AdasConfig`, `RuntimeParams`, `Logger`, `TimeUtil` |
| `adas.app.sensors` | `CameraHandler`, `IMUHandler`, `GPSHandler`, `PhoneStatsHandler` |
| `adas.app.vision` | `VisionPipeline`, both runners, `ModelCalibWarp`, `LaneLines`, `IntrinsicsCalibrator` |
| `adas.app.bridge` | `ZMQBridgeService`, `ProtoUtils` |
| `adas.app.record` | `BagLogger`, `AudioRecorder` |
| `adas.app.ui` | `MainActivity`, `ParamSlider`, the two bench activities |

## Split of responsibility

| Stays in **Java** | Lives in **C++** |
|---|---|
| Camera2 / IMU / GPS | planning (`pp` / `mpc` / `fp`) and the control law |
| Supercombo (thneed or ORT) + optional YOLO | SafetyWarn, long plan, `proto_convert` |
| `Logger` / `BagLogger` (session bags) | CAN TX/RX and HCA, behind `CarPlatform` |
| Preview HUD (`LaneOverlayView`) | Middleware bus + timers |
| `ZMQBridgeService` PUB/SUB sockets | Native ZMQ bridge service |

```{admonition} Design rule
:class: tip
If it decides **how to steer or warn**, prefer C++ (testable with `pyadas`).
If it needs **Camera2, Activity, or ORT Android**, it stays in Java.
```

## Frame path (camera → native)

```text
CameraHandler (YUV 1280×720, capture_ts)
    → VisionPipeline.submitYuv   (latest-frame drop if busy)
        → ModelCalibWarp.warpPair → 6×128×256 ×2   (OpenCL kernel, CPU fallback)
        → SupercomboThneedRunner.run (GPU)  |  SupercomboOnnxRunner.run (ORT)
        → parse LaneLines / model_long / odometry
            → ProtoUtils.create…Message(...)
            → ZMQBridgeService.publishToNative(msg)   // :5555
            → Logger.logZMQMessage(...)               // bag copy
```

```java
// Shape of the control publish (names simplified)
Messages.ZMQMessage lanes =
    ProtoUtils.createLaneLinesMessage(laneLines, /*forBag=*/ false);
ZMQBridgeService.publishToNative(lanes);

Messages.ZMQMessage modelLong =
    ProtoUtils.createModelLongPlanMessage(ts, frameId, modelLong, pose);
ZMQBridgeService.publishToNative(modelLong);
```

```{warning}
Bag JPEG is usually the **preview** image, not the warped network input.
Offline re-inference must rebuild the warp — see [Supercombo](../Vision/Supercombo.md).
```

## ZMQ bridge

| direction | endpoint (default) | content |
|---|---|---|
| Java → C++ | `tcp://127.0.0.1:5555` | `vision/lanes`, `vision/model_long`, sensors, … |
| C++ → Java | `tcp://127.0.0.1:5556` | `safety/warn`, steer debug, stats, … |

Config: `zmq.endpoint_in` / `zmq.endpoint_out` in `config.json`.

```python
# Student checklist when "native never moves"
checks = [
    "nodes.zmq_bridge == true",
    "the runner passed its zero-input check at load (logcat: 'accepted')",
    "VisionPipeline actually finishing inference (infer_ms finite)",
    "publishToNative called (logcat)",
    "C++ ZmqBridge polling IN",
    "Planner producing control/lat_plan for Control",
]
for c in checks:
    print("-", c)
```

## Config and live parameters

1. **Shipped JSON:** `app/src/main/assets/config.json` (copied to `filesDir` on first run — **not**
   overwritten on upgrade, which is a trap worth knowing: new keys added to the asset never reach an
   existing install. Clear it with the reset button, or
   `adb shell run-as adas.app rm -f files/config.json` — but see the warning below before you do).

   ```{warning}
   That file also holds the **learned** camera calibration. Deleting it throws away mounting angles the
   system spent a drive learning, and the app will re-learn them from whatever it sees next — including,
   once, a desk. Saving is now gated on `VisionPipeline.seesRoad()` for that reason.
   ```
2. **`AdasConfig`** — which nodes are on (`nodes.lane_keep`, `nodes.safety_warn`, …).
3. **`RuntimeParams` + UI sliders** — PP / calib-style knobs while driving.
4. **`AdasAppHandler.applyLaneKeepParams`** → JNI → `Middleware::setParameter`.

```python
# What a slider change means on the bus
ui_value = 0.8
# Java sends string "0.8" for name "pp_k_dd"
# ParamBag on the Planner's thread parses → double pp_k_dd_
assert float("0.8") == ui_value
```

Push a new file to the phone without rebuilding APK: `./scripts/push_config.sh` (see repo scripts).

## Bags

`Logger` creates a session directory under the app’s storage; each topic is a protobuf stream.
Students pull with `./scripts/pull_bags.sh`, then:

```bash
cd scripts
python3 tools/latency.py /path/to/adas_logs/SESSION
python3 -m vis.export_to_plotjuggler /path/to/SESSION -o /tmp/out
```

Enable `phone_stats` in config for CPU / thermal at 1 Hz — required for latency homework.

## UI outbound

`MainActivity` listens on ZMQ OUT. Example: `safety/warn` →

```java
laneOverlay.setSafetyWarn(fcw, aeb, lldw, rldw);
```

HUD text is **display only**. It does not brake or steer.

## Class map (read in this order)

| class | role |
|---|---|
| `MainActivity` | lifecycle, wire handlers |
| `CameraHandler` | frames + timestamps |
| `VisionPipeline` | runner selection, drop policy, `seesRoad()` |
| `SupercomboThneedRunner` / `SupercomboOnnxRunner` | the network, on GPU or through ORT |
| `ModelCalibWarp` | warp to model geometry, GPU-vs-CPU verification, focus metric |
| `IntrinsicsCalibrator` | chessboard intrinsics, when you shoot them |
| `ZMQBridgeService` / `ProtoUtils` | into native |
| `Logger` | bags |
| `AdasConfig` / `RuntimeParams` | knobs |
| `AdasAppHandler` | JNI / start native |
| `LaneOverlayView` | HUD |

## Exercise

1. Trace one frame: which class drops it if the runner is busy?
2. Flip `nodes.vision_supercombo` to `false` in a mental experiment — which topics disappear?
3. From logcat or code, list three messages Java publishes on `:5555`.

<!-- next-chapter -->
---

**Next:** [From frame to CAN](./Pipeline.md)
