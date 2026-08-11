# Vision: supercombo ONNX (simple wrapper)

Based on driving-model `driving.cc` output layout (`supercombo.onnx`, out=6409).

## Classes

| Class | Role |
|-------|------|
| `vision/ModelCalibWarp` | K + RPY → 3×3 warp (flowpilot `getWrapMatrix` / medmodel) |
| `vision/SupercomboOnnxRunner` | Warp preprocess + ONNX Runtime + parse plan / lanes / edges |
| `vision/LaneLines` | 4 lanes + 2 edges + best PLAN path at ego xyz |
| `vision/LaneOverlayView` | Yellow lanes / red edges / **green PLAN** on camera |
| `vision/CameraOdometry` | Pose slice from model output (for C++ calib) |
| `vision/VisionPipeline` | Background thread wiring |

## Calib warp

Android params **Roll/Pitch/Yaw** (and `intrinsics_prior` from `config.json`) rebuild the model warp live. Height/X/Y affect overlay / C++ only, not the network input.

## Frames / overlay

- Model outputs kept in **device frame** (X fwd, **Y right+**, Z up) — same as flowpilot `Parser`.
- Overlay projects with the **same R(rpy)** as the model warp (`ModelCalibWarp`), via
  `Rt = V·R·V⁻¹` after remap `(Y,Z,X)`. Do **not** use LibGDX
  `setFromEulerAnglesRad(-pitch,-yaw,-roll)` here — it swaps pitch/yaw vs the warp.
- Path lift **+1.28 m** on camera-up (after remap), like flowpilot `OnRoadScreen`.
- C++ `laneLinesToPath` uses flowpilot `lane_planner` mid-lane (`lll+w/2`, `rll-w/2`).

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
- Bag `vision/lanes.model_out` stores the full ONNX flat vector (~6409) for offline re-parse.
