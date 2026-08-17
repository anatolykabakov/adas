# Model long / CIPV / SafetyWarn — bag findings (2026-07-31)

## Topics (live → bag)

| Topic | Source | Bag |
|-------|--------|-----|
| `vision/lanes` | Java | `lead_d/y/v/prob`, `plan_v0`, `lead_valid` (+ full `model_out`) |
| `vision/model_long` | Java | full lead0/1/2 + plan_v_* |
| `control/long_plan` | C++ LongPlanService | via ZMQ OUT → BagLogger |
| `safety/warn` | C++ SafetyWarnService | via ZMQ OUT → BagLogger |

## Index check vs flowpilot F2

`LEAD_IDX=5755`, `LEAD_PROB_IDX=5857`, group=51 — matches `CommonModelF2` / `driving.h`.

## Current assets (`supercombo.onnx`, out=6409)

- Same as `openpilot-supercombo-model` / OP **v0.8.5** (md5 `1fe98451…`).
- Lead presence logits ≈ **−55 even on zero input** → head dead in weights (not just camera).
- `plan_v` / pose OK.

## Alternate models (`adas/models/`, downloaded)

| File | out | Drop-in? | Lead @5857 (zeros) |
|------|-----|----------|--------------------|
| assets / **v0.8.5** | 6409 | yes | **dead (−53)** |
| **v0.8.12 = v0.8.13** | **6472** | same I/O; set `OUTPUT_SIZE=6472` | **alive (~0.05–0.10)** |
| v0.8.9 | 6609 | same I/O; parse size differs | layout shifted |
| v0.9.0 / v0.9.4 | 6108/6120 | **no** (2 cams + feat buf, often fp16) | other API |

**Preferred on device:** `models/sc_v0.8.13.onnx` (full F2).
Gradle `preBuild` copies it to `assets/supercombo.onnx` (`app/build.gradle` → `syncSupercomboModel`).
Pose index auto-detects from `flat.length` in Java (`CameraOdometry.poseIdx`).

```bash
# ensure model present (gitignored downloads/)
ls models/sc_v0.8.13.onnx
# then normal APK build — assets get synced automatically
# override: ./gradlew :app:assembleDebug -PsupercomboModel=/path/to.onnx

# or skip rebuild:
adb push models/sc_v0.8.13.onnx /sdcard/adas_models/supercombo.onnx
```

### Bag A/B (`model_ab_sim.py`, `2026_07_31_10_33_17`, 30 frames)

| Model | lead_frac | FCW/AEB* | FP Δswa_p95 | MPC Δswa | PP Δswa |
|-------|-----------|----------|------------|----------|---------|
| v0.8.5 (6409) | 0.00 | 0 / 0 | ~0 | +0.8 | +11 |
| **v0.8.13 (6472)** | **0.70** | 0 / high* | **~0** | **+0.6** | +13 |
| v0.8.9 (6609) | 1.00† | noisy† | ~0 | — | +9 |

\* IDM fixed to classic `Δv = v_ego − v_lead` + `v_lim=27.778` (was absolute lead_v → perpetual AEB). On v0.8.13 lead is real (~55–65 m); steady follow no longer fires AEB.
† Layout shifted — do not trust long/lead indices.

Lateral: **FP/MPC no regression** across 6409↔6472 vs bag `lane_keep_debug`; PP stays noisy offline.

```bash
python model_ab_sim.py ../adas_logs/<bag> \
  --models ../../../models/sc_v0.8.5.onnx ../../../models/sc_v0.8.13.onnx \
  --n-frames 40 --controllers pp,fp,mpc -o /tmp/model_ab.csv
python bag/bag_long_sim.py ../adas_logs/<bag> -o /tmp/long.csv
```
