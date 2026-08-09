# ADAS — phone-based LKA / AEB / FCW / LDW / ACC (Android + C++)

Phone-based ADAS for VW Golf 7 (MQB): camera → supercombo ONNX → lateral MPC (LKA) and
longitudinal/safety functions (ACC, FCW, AEB, LDW) → HCA / related CAN via Panda. Plus bag
recording, offline analysis tools, and MetaDrive sim.

**All algorithms are in C++** (services inside `AdasApp`). Java handles the camera, ONNX, UI, and
logging; Python is offline only: bag visualizer, simulator, analysis (`publish → step →
pop_messages` via `pyadas`).

**Status:** MVP drives on the highway. Vision now runs at the full camera rate — 29.86 Hz measured on
the road, against 13.5 before — and the lateral feedforward has been resized from three drives'
worth of data; that change has **not been verified on the road yet**, and the criterion for the next
drive is written down in [`docs/PREDRIVE.md`](docs/PREDRIVE.md) before it happens. Latest run — 58 minutes recorded, **35.6 min under
control**: lane-center tracking **0.16 m** median (p95 0.53 m) at 15–27 m/s, 0.14 m on straights,
0.19–0.21 m on arcs R 200–1000 m; driver touched the wheel 3% of the time. On the same
road a human averaged 0.17 m. Numbers from the 2026-08-01 highway bag;
applicability limits and what the system cannot do —
[`docs/CONTROLLER_LIMITS.md`](docs/CONTROLLER_LIMITS.md).

**Links:** [Course book (GitHub Pages)](https://anatolykabakov.github.io/adas/) ·
[Latest APK](https://github.com/anatolykabakov/adas/releases/latest/download/app-debug.apk)
([Releases](https://github.com/anatolykabakov/adas/releases))

---

## Quick start

No host install — use the container (docker only):

```bash
./scripts/docker.sh build                                  # image with C++/python environment
./scripts/docker.sh tests                                  # unit tests
./scripts/docker.sh host                                   # host build + pyadas
./scripts/docker.sh sim --track highway --controllers fp    # controller run on track
```

On the host:

```bash
./scripts/install_dependencies.sh        # SDK/NDK/Conan/Python — if needed
./scripts/build_project.sh               # debug APK + native arm64
./scripts/build_project.sh --cpp-only    # libadas_app_android.so only

# offline stack: host build, pyadas, and unit tests in one command
./app/src/main/cpp/build_cpp.sh -t linux --test
pip install -r app/src/main/scripts/sim/requirements.txt   # + MetaDrive
```

APK: `app/build/outputs/apk/debug/app-debug.apk`, native lib: `app/libs/arm64-v8a/`.
C++ builds land in `app/src/main/cpp/build/<target>/<BuildType>` — one root, split by target.

**Gradle does not build the native library.** It is copied from `app/libs/`, so `assembleDebug`
finishes in three seconds and will happily package a stale one; `scripts/prepare_drive.sh` refuses to
install when the C++ sources are newer and compares the sha1 inside the APK against the built one.
Requires `ANDROID_HOME` / NDK (`local.properties`), Java 17+, Conan 2. The host build puts
`pyadas/core*.so` into `app/src/main/scripts/`, which every offline tool runs on.

## Project layout

```
app/src/main/
├── java/ai/flow/adas/      UI, camera, ONNX + thneed runners, ZMQ bridge, bag + audio recording
│   └── vision/             calibration warp, model output parsing, overlay
├── cpp/                    AdasApp: all control and processing
│   ├── include/adas/       public headers; the include path mirrors the namespace
│   └── src/
│       ├── services/       lane_keep, panda, localization, camera_calib, map_data, …
│       ├── lateral/        the three lateral controllers: flowpilot_mpc, visionpilot_mpc, pp
│       ├── platform/       the car behind an interface: volkswagen/ is the only make so far
│       ├── mapmatch/       OSM road graph: localization, curvature ahead
│       ├── panda/          USB driver and CAN framing
│       ├── thneed/         vendored flowpilot GPU runner (MIT)
│       └── utils/          vehicle model, PID, filters, calibrations
├── proto/                  protobuf: bag format and ZMQ exchange
├── assets/                 config.json, supercombo.onnx, supercombo.thneed, DBC
└── scripts/                Python: bag analysis, simulator, visualizer, pyadas
docs/                       engineering reports + Jupyter Book course
scripts/                    build, dependency install, adb helpers
```

Namespaces follow directories: `adas/services/lane_keep.h` declares `adas::services::LaneKeep`,
`adas/middleware/manager.hpp` declares `adas::middleware::Manager`. Shared data types stay in
`adas` — they belong to the system, not to the service that produces them.

## Control pipeline

```
camera 30 Hz → calibration warp → supercombo (thneed 15.9 ms on GPU, or ONNX 44.7 ms)
        │
        └─► vision/lanes ─ZMQ→ TopicConvert ─► vision/path (plan + lane markings)
                                                    │
                                   LaneKeep: MPC → κ → steer angle → angle-PID
                                                    │
                                   controls/steer ─► Panda 100 Hz ─► HCA_01 on bus
```

Full chain with all fields and timings:
[`docs/IMAGE_TO_CAN_PIPELINE.md`](docs/IMAGE_TO_CAN_PIPELINE.md).

Measured on run 2026_08_08_23_00_28 with the thneed model: frame interval 33.0 ms (**29.86 Hz**),
capture → steering command on CAN **58 ms** against 93 before. The vision rate matters because it sets
the size of a setpoint step: halving the interval roughly halved it on arcs (0.49° → 0.28° on
R 83–167 m), and tracking error fell with it (1.87° → 1.13°). Discussion —
[`docs/VISION_RATE.md`](docs/VISION_RATE.md).

### Controllers

| key | description | status |
|---|---|---|
| `fp` | flowpilot lateral MPC port (Eigen, N=16) + κ→angle via vehicle model | **default** |
| `mpc` | spatial MPC from VisionPilot | alternative |
| `pp` | Pure Pursuit + angle-PID | legacy, for debugging |

Switch in `config.json` (`vehicle.lane_keep_controller`) or in the in-app parameter panel.

## Key parameters (`app/src/main/assets/config.json`)

| key | value | meaning |
|---|---|---|
| `lane_keep_controller` | `fp` | active lateral controller |
| `vision.model_runner` | `onnx` | which model runs vision: `onnx` or `thneed` (GPU, 30 Hz) |
| `lat_pid_kf` / `lat_pid_ff_floor_mps` | `0.00015` / `9.8` | feedforward `kf·SWA·(v² + v₀²)`, both fitted over three drives |
| `lat_always_on` | `true` | steer where stock cruise is not engaged (ALKA) |
| `tire_stiffness_factor` | `0.64` | κ→angle understeer correction, measured from bags |
| `path_lane_blend_scale` | `0.6` | lane-center share in the reference, when both lines are visible |
| `localization.use_camera_odometry` | `false` | off: the model yaw rate is sign-inverted against CAN |

Every key in `config.json` carries a `comment_*` next to it saying what was measured and why the
value is what it is — that file, not this table, is the reference.



The file is copied to `filesDir` on first launch and **is not overwritten** by APK install: to
pick up new defaults, use `scripts/push_config.sh --apply` (preserves camera calibration)
or clear app data.

## Simulator

Closed loop on a generated track with straights and arcs of known radius, scored against ground
truth. The controller reads the same `config.json` that ships in the APK, and a non-zero exit code
means the thresholds were missed — usable as a regression test.

```bash
cd app/src/main/scripts
python3 -m sim.eval --list-tracks                              # highway / curvy / tight / straight
python3 -m sim.eval --track curvy --controllers fp,pure_pursuit
./scripts/run_sim.sh --track curvy --controller fp --show      # same run with a window
```

What this proves and what it does not — an ideal reference on a different vehicle, and supercombo
breaks on synthetic curves so closed loop needs `--lanes gt`:
[`docs/SIM_CONTROLLER_TEST.md`](docs/SIM_CONTROLLER_TEST.md).

## Offline tools

Start with one command — it prints the vision rate, the setpoint step, where torque hits the panda
ceiling, lane confidence, model pose against the wheels, and the driver's own spoken notes:

```bash
./scripts/pull_bags.sh                                   # pull bags from the phone into ./adas_logs
cd app/src/main/scripts
python3 bag_report.py ../../../../adas_logs/<session>     # 95 s cold, 0.3 s from cache
python3 bag_report.py <A> <B> --only step                 # compare two drives
```

Audio is recorded alongside the bag, and `bag_voice_notes.py` pulls the driver's remarks out of it
with the numbers from the same second. What each of the remaining scripts answers —
[`app/src/main/scripts/README.md`](app/src/main/scripts/README.md); how the cache and the voice notes
work — [`docs/BAG_ANALYSIS.md`](docs/BAG_ANALYSIS.md).

```bash
./scripts/run_bag_vis.sh adas_logs/<session>              # interactive bag viewer
python3 bag_controller_ab.py <bag> --controllers fp,mpc   # controllers against the driver's steering
python3 bag_safety_warn.py <bag>                          # replay the warning chain, count triggers
```

## Tests

```bash
./app/src/main/cpp/build_cpp.sh -t linux --test     # 200 tests, 1 skipped (needs a ZMQ socket)
./scripts/docker.sh tests                           # same in container
```

Coverage: lateral controllers (speed gate, limits, lane-marking blend, feedforward), pure pursuit,
PID, VW CAN (HCA/counter/CRC), middleware, FCW/AEB/LDW warnings, learned vehicle parameters, road
roll, speed filter, EKF, map route matching, and config wiring.

After a structural change rebuild from scratch — an incremental build silently reuses object files
whose source did not change even when a header did, and reports green on code that no longer
compiles.

## Documentation

[`docs/README.md`](docs/README.md) is the index and says which document answers which question.
If you are new: the [course book](docs/book/Introduction/intro.md), then
[`docs/IMAGE_TO_CAN_PIPELINE.md`](docs/IMAGE_TO_CAN_PIPELINE.md) for the whole chain and
[`docs/CONTROLLER_LIMITS.md`](docs/CONTROLLER_LIMITS.md) for where the assistant works.
