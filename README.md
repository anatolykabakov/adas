# ADAS — phone-based LKA / AEB / FCW / LDW / ACC (Android + C++)

> **Research code that steers a car.** Not a product, not certified, no warranty. The driver keeps
> hands on the wheel and supervises at all times; complying with local law is on whoever runs it.
> What the assistant can and cannot do is written down —
> [`docs/CONTROLLER_LIMITS.md`](docs/CONTROLLER_LIMITS.md). Tight bends are outside it today.

Phone-based ADAS for VW Golf 7 (MQB): camera → supercombo 0.9.7 on the phone GPU → lateral MPC (LKA)
and longitudinal/safety functions (ACC, FCW, AEB, LDW) → HCA / related CAN through a panda. Plus bag
recording, offline analysis tools, and a MetaDrive sim. No comma hardware beyond the panda.

**All algorithms are in C++** (services inside `AdasApp`). Java handles the camera, inference, UI and
logging; Python is offline only: bag visualizer, simulator, analysis (`publish → step →
pop_messages` via `pyadas`).

**Status: MVP, drives on the highway.** Latest drive 2026-08-21, 25.7 minutes on a Xiaomi 14 — every
number here is measured on the road, not estimated:

| | |
|---|---|
| supercombo inference | **9.7 ms** median, p95 17.1 (17.6 ms on a OnePlus 7T) |
| capture → plan | **22 ms** median, p95 53 |
| vision rate | 23.8 Hz at the camera's pinned rate, 0 messages lost across 9 services |
| lane-centre offset, straights (R > 500 m) | **+0.036 m** median, angle tracking error **0.32°** |
| gentle arcs (R 167–500 m) | +0.083 m median, 0.74° |

**Three caveats those numbers do not carry.** The offset is measured only where the lane was
recognised — **51 %** of controlled ticks — and on 21.8 % of frames both lines sit below 0.3
confidence without the system saying so ([task #40](docs/BACKLOG.md)). Torque reaches the panda's
300 cNm ceiling on 6.7 % of ticks overall, but **28 % on gentle arcs and 82 % on medium ones**
(R 83–167 m), where the tracking error grows to 2.9° — that is the honest edge of this assistant.
And this phone's lens calibration disagrees with the datasheet by 11 %, which is the leading suspect
behind lane-line σ of 0.37 / 0.51 ([task #51](docs/BACKLOG.md)).

**Frame spacing is the thing to know about supercombo.** Every velocity it reports — the lead, the
pose — lives in the interval between its two input frames, and it is trained at comma's 50 ms. At a
33 ms pairing lead velocity reads 0.59× true and pose 0.64×; at 66 ms, 1.22×. The correction is
`50/dt`, applied in `ModelLongParse`, and it is what explains a pose scale that had looked like a
mystery constant: this drive paired at 42 ms and the pose slope came out 0.825, against 42/50 = 0.84.
The yaw sign is still inverted ([task #37](docs/BACKLOG.md)), which is why localisation does not use
the model's pose.

Which question the next drive asks, and what would count as an answer, is written down before it
happens — [`docs/PREDRIVE.md`](docs/PREDRIVE.md).

**Links:** [Course book (GitHub Pages)](https://anatolykabakov.github.io/adas/) ·
[Latest APK](https://github.com/anatolykabakov/adas/releases/latest/download/app-debug.apk)
([Releases](https://github.com/anatolykabakov/adas/releases))

**License:** code Apache-2.0, book and docs CC BY 4.0 — [`LICENSE`](LICENSE),
[`LICENSE.md`](LICENSE.md) for what it does not cover (the shipped model and DBCs are comma's, MIT).
Reviews and pull requests are welcome.

---

## Quick start

No host install — use the container (docker only):

```bash
git lfs install && git lfs pull                            # models and map are LFS objects
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
pip install -r scripts/sim/requirements.txt   # + MetaDrive
```

Models and the map can be kept out of the repository instead of pulled from LFS:
`./scripts/fetch_models.sh` brings them into `models/` and `maps/` (both gitignored) from the places
entitled to distribute them, checking sha256 against `scripts/models.manifest`; gradle packages from there.
Why that matters beyond repo size — [`THIRD_PARTY.md`](THIRD_PARTY.md), [`LICENSE.md`](LICENSE.md).

APK: `app/build/outputs/apk/debug/app-debug.apk`, native lib: `app/libs/arm64-v8a/`.
C++ builds land in `app/src/main/cpp/build/<target>/<BuildType>` — one root, split by target.

**Gradle does not build the native library.** It is copied from `app/libs/`, so `assembleDebug`
finishes in three seconds and will happily package a stale one; `scripts/prepare_drive.sh` refuses to
install when the C++ sources are newer and compares the sha1 inside the APK against the built one.
Requires `ANDROID_HOME` / NDK (`local.properties`), Java 17+, Conan 2. The host build puts
`pyadas/core*.so` into `scripts/`, which every offline tool runs on.

## Project layout

```
app/src/main/
├── java/adas/app/          UI, camera, ONNX + thneed runners, ZMQ bridge, bag + audio recording
│   ├── vision/             calibration warp, model output parsing, overlay
│   ├── sensors/            camera, GNSS, IMU
│   ├── record/             bag and audio writers
│   ├── bridge/             ZMQ to the native side
│   └── ui/                 screens and overlays
├── cpp/                    AdasApp: all control and processing
│   ├── include/adas/       public headers; the include path mirrors the namespace
│   ├── tests/              gtest: 230 cases
│   └── src/
│       ├── services/       planner, control, platform, localization, camera_calib, map_data, …
│       ├── lateral/        the three lateral strategies behind IPlanner: fp, vp, pp, plus the solvers
│       ├── platform/       the car behind CarPlatform: volkswagen/
│       ├── mapmatch/       OSM road graph: localization, curvature ahead
│       ├── panda/          USB driver and CAN framing
│       ├── python/         pybind11 module: the same C++ the phone runs, importable as pyadas
│       ├── thneed/         vendored flowpilot GPU runner (MIT)
│       └── utils/          vehicle model, PID, filters, calibrations
├── proto/                  protobuf: bag format and ZMQ exchange
└── assets/                 config.json, supercombo.onnx, supercombo.thneed, DBC
docs/                       engineering reports + Jupyter Book course
scripts/                    everything runnable, one directory
├── *.sh                    build, dependency install, adb helpers, docker
├── bag/                    bag analysis: bag_report and the drill-downs behind it
├── rlog/                   the same against comma logs, for differential comparison
├── sim/                    MetaDrive: eval on tracks, interactive run
├── vis/                    bag viewer, top-down render, PlotJuggler export
├── mapmatch/               OSM graph tooling: build, locate, score
├── tools/                  calibration, latency, panda flashing
├── core/                   the shared Python library the above import
├── cpp/                    clang-tidy: runners and priority reports
└── pyadas/                 built C++ module, copied here by the build
```

Namespaces follow directories: `adas/services/planner.h` declares `adas::services::Planner`,
`adas/middleware/manager.hpp` declares `adas::middleware::Manager`. Shared data types stay in
`adas` — they belong to the system, not to the service that produces them.

## Control pipeline

```
camera 30 Hz → calibration warp (GPU, 4.6 ms) → supercombo (thneed 17.6 ms on GPU, or ONNX 48.5 ms)
        │
        └─► vision/lanes ─ZMQ→ ZmqBridge ─► vision/path (plan + lane markings)
                                                    │
                              Planner: MPC → κ → steer angle ─► control/lat_plan
                                                    │
                              Control: angle-PID → torque ─► controls/steer
                                                    │
                                        Platform 100 Hz ─► HCA_01 on bus
```

Full chain with all fields and timings:
[`docs/IMAGE_TO_CAN_PIPELINE.md`](docs/IMAGE_TO_CAN_PIPELINE.md).

Measured on run 2026_08_16_23_59_45 with the thneed model, and worth keeping the three stages apart
because they are routinely conflated: capture → model output **22 ms**, capture → plan
(`control/lane_keep`) **31 ms**, capture → steering command (`controls/steer`) **52 ms**, plus the
panda's 10 ms transmit timer to the wire. The vision rate matters because it sets the size of a
setpoint step: halving the interval roughly halved it on arcs, and tracking error fell with it. How
that was established — `VISION_RATE.md` (удалён, история в git).

The model runs the same network on either path — `assets/supercombo.onnx` is the source, and
`assets/supercombo.thneed` is generated from it. Both runners check themselves against a zero-input
reference before accepting a frame; how the thneed is built and verified, and what it takes on a
phone that is not this one — [`docs/THNEED.md`](docs/THNEED.md), [`docs/NEW_PHONE.md`](docs/NEW_PHONE.md).

### Controllers

| key | description | status |
|---|---|---|
| `fp` | flowpilot lateral MPC port, N=16, + κ→angle via vehicle model; `fp_solver` picks the numerical method: `grad` (Eigen) or `acados` | **default** |
| `pp` | Pure Pursuit + angle-PID | legacy, for debugging |

Switch in `config.json` (`vehicle.lane_keep_controller`) or in the in-app parameter panel.

## Key parameters (`app/src/main/assets/config.json`)

| key | value | meaning |
|---|---|---|
| `vehicle.lane_keep_controller` | `fp` | active lateral controller |
| `vehicle.fp_solver` | `acados` | numerical method inside `fp`: `grad` or `acados` (the key's own `comment_*` argues for `grad` on a measurement — the shipped value and that comment disagree, see below) |
| `vision.model_runner` | `thneed` | which model runs vision: `thneed` (GPU) or `onnx` |
| `vehicle.lat_pid_kf` | `6e-05` | feedforward `kf·SWA·(v² + v₀²)`, fitted over three drives |
| `vehicle.tire_stiffness_factor` | `1.0` | κ→angle understeer correction; `lat_use_vehicle_model` off leaves plain kinematics |
| `vehicle.path_lane_blend_scale` | `1.0` | lane-center share in the reference, when both lines are visible |
| `vehicle.lane_max_age_s` | `0.3` | a plan older than this withdraws the command — 8 m of road at 100 km/h |
| `vehicle.assist_max_age_s` | `0.5` | how long a panda health report stays usable; five publish periods |
| `localization.use_camera_odometry` | `false` | off: the model yaw rate is sign-inverted against CAN |

`vehicle.fp_solver` is shipped as `acados` while its `comment_fp_solver` records that on run
2026_08_10_02_07_55 acados differed from a converged `grad` by 0.36° median and jittered more between
frames, concluding "therefore grad by default". The code default is `grad`; the shipped config
overrides it. One of the two should move — that is a driving decision, not a documentation one.

The values above drift as tuning continues — `config.json` itself is the reference, not this table.
Eight of its keys carry a `comment_*` next to them spelling out what was measured and why the value
is what it is; those are the ones where the reasoning was not obvious from the name.

The file is copied to `filesDir` on first launch and **is not overwritten** by APK install: to
pick up new defaults, use `scripts/push_config.sh --apply` (preserves camera calibration)
or clear app data.

## Simulator

Closed loop on a generated track with straights and arcs of known radius, scored against ground
truth. The controller reads the same `config.json` that ships in the APK, and a non-zero exit code
means the thresholds were missed — usable as a regression test.

```bash
cd scripts
python3 -m sim.eval --list-tracks                              # highway / curvy / tight / straight
python3 -m sim.eval --track curvy --controllers fp,pp
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
cd scripts
python3 bag/bag_report.py ../adas_logs/<session>  # 95 s cold, 0.3 s from cache
python3 bag/bag_report.py <A> <B> --only step     # compare two drives
```

Audio is recorded alongside the bag, and `bag/bag_voice_notes.py` pulls the driver's remarks out of it
with the numbers from the same second. What each of the remaining scripts answers —
[`scripts/README.md`](scripts/README.md); how the cache and the voice notes
work — [`docs/BAG_ANALYSIS.md`](docs/BAG_ANALYSIS.md).

```bash
./scripts/run_bag_vis.sh adas_logs/<session>              # interactive bag viewer
python3 bag/bag_report.py <bag>                              # the whole drive in one cached report
python3 bag/bag_safety_warn.py <bag>                         # replay the warning chain, count triggers
```

## Tests

```bash
./app/src/main/cpp/build_cpp.sh -t linux --test     # 230 tests
./scripts/docker.sh tests                           # same in container
```

Coverage: lateral controllers (speed gate, limits, lane-marking blend, feedforward), pure pursuit,
PID, VW CAN (HCA/counter/CRC), middleware, FCW/AEB/LDW warnings, learned vehicle parameters, road
roll, speed filter, EKF, map route matching, and config wiring.

After a structural change rebuild from scratch — an incremental build silently reuses object files
whose source did not change even when a header did, and reports green on code that no longer
compiles.

Static analysis is clang-tidy, run either over everything or over the lines a change touched; the
report sorts files by how much the findings can actually hurt. How to run it and what the priorities
mean — [`scripts/cpp/README.md`](scripts/cpp/README.md), last pass —
`CLANG_TIDY_2026_08_15.md` (удалён, история в git).

## Documentation

[`docs/README.md`](docs/README.md) is the index and says which document answers which question.
If you are new: the [course book](docs/book/Introduction/intro.md), then
[`docs/IMAGE_TO_CAN_PIPELINE.md`](docs/IMAGE_TO_CAN_PIPELINE.md) for the whole chain and
[`docs/CONTROLLER_LIMITS.md`](docs/CONTROLLER_LIMITS.md) for where the assistant works.
