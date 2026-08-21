# Vision: supercombo on the phone

Output layout follows the driving model `driving.cc`. Two runners sit behind one interface, so the
rest of the package does not care which one produced the vector.

## Classes

| Class | Role |
|-------|------|
| `ModelRunner` | The interface both runners implement |
| `SupercomboOnnxRunner` | Warp preprocess + ONNX Runtime + parse plan / lanes / edges |
| `SupercomboThneedRunner` | The same model on the GPU through thneed; output vector 6504 |
| `ModelCalibWarp` | K + RPY → 3×3 warp (flowpilot `getWrapMatrix` / medmodel) |
| `LaneLines` | 4 lanes + 2 edges + best PLAN path at ego xyz |
| `LaneOverlayView` | Yellow lanes / red edges / **green PLAN** on camera |
| `CameraOdometry` | Pose slice from the model output (for C++ calibration) |
| `ModelLongParse` | Lead and PLAN velocity for the longitudinal side |
| `YuvFrame` | Camera frame buffer handed to a runner |
| `VisionPipeline` | Background thread wiring |
| `TrafficVisionPipeline`, `TrafficYoloRunner`, `SpeedLimitOcr` | Signs and traffic lights: a separate model and its OCR |

**The pose offset is computed, not hardcoded.** `CameraOdometry.poseIdx(length)` derives it from the
vector length, because the pose sits last before the features and generations differ: the same formula
that gives the right address for our layout gives 5980 instead of 5948 on 0.9.x. The pose struct
itself — velocity_mean, rotation_mean, velocity_std, rotation_std — is identical across them; only its
address moves. Pass an explicit offset to `parse(out, poseIdx)` when the layout is known.

## Calib warp

Android params **Roll/Pitch/Yaw** (and `intrinsics_prior` from `config.json`) rebuild the model warp live. Height/X/Y affect overlay / C++ only, not the network input.

## Frames / overlay

- Model outputs kept in **device frame** (X fwd, **Y right+**, Z up) — same as flowpilot `Parser`.
- Overlay projects with the **same R(rpy)** as the model warp (`ModelCalibWarp`), via
  `Rt = V·R·V⁻¹` after remap `(Y,Z,X)`. Do **not** use LibGDX
  `setFromEulerAnglesRad(-pitch,-yaw,-roll)` here — it swaps pitch/yaw vs the warp.
- Path lift **+1.28 m** on camera-up (after remap), like flowpilot `OnRoadScreen`.
- C++ `laneLinesToPath` is comma stock `LanePlanner.get_d_path` (mid-lane `lll+w/2`, `rll-w/2`).

## Output parse (important)

Do **not** use the GitHub demo’s `ll_t` / `ll_t2` grouping. Official layout:

| Slice | Meaning |
|-------|---------|
| `[0:4955)` | PLAN — 5 MHP trajectories (33×15 mean + std + logit) |
| `[4955:5483)` | LANES — 4×33×**(y,z)** means, then stds |
| `[5483:5491)` | lane probs — `sigmoid(prob[i*2+1])` |
| `[5491:5755)` | ROAD EDGES — 2×33×(y,z) means, then stds |

Green overlay is the **best PLAN hypothesis**, not the midpoint of near lanes.

## Build / install APK

From the repository root:

```bash
./scripts/build_project.sh              # debug
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

## Model

Load order: `/sdcard/adas_models/supercombo.onnx` → app filesDir cache → assets.

```bash
adb shell mkdir -p /sdcard/adas_models
adb push /path/to/supercombo.onnx /sdcard/adas_models/supercombo.onnx
```

## Notes

- Needs 2 frames before first result (temporal stack).
- Bag `vision/lanes.model_out` stores the full flat output vector for offline re-parse; its length
  depends on the runner, so parse against the length rather than a constant.
