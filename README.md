# ADAS — phone-based LKA / AEB / FCW / LDW / ACC (Android + C++)

Phone-based ADAS for VW Golf 7 (MQB): camera → supercombo ONNX → lateral MPC (LKA) and
longitudinal/safety functions (ACC, FCW, AEB, LDW) → HCA / related CAN via Panda. Plus bag
recording, offline analysis tools, and MetaDrive sim.

**All algorithms are in C++** (services inside `AdasApp`). Java handles the camera, ONNX, UI, and
logging; Python is offline only: bag visualizer, simulator, analysis (`publish → step →
pop_messages` via `pyadas`).

**Status:** MVP drives on the highway. Latest run — 58 minutes recorded, **35.6 min under
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
Tagged releases (`v*`) upload that APK to GitHub Releases — use the Latest APK link above.
Requires `ANDROID_HOME` / NDK (`local.properties`), Java 17+, Conan 2.
Host build puts `pyadas/core*.so` in `app/src/main/scripts/` — all offline tools run on that.

## Project layout

```
app/src/main/
├── java/ai/flow/adas/      UI, camera, ONNX (supercombo + YOLO), ZMQ bridge, bag recording
│   └── vision/             calibration warp, model output parsing, overlay
├── cpp/                    AdasApp: all control and processing
│   ├── services/           lane_keep, panda, topic_convert, localization, camera_calib, map_data, …
│   ├── mapmatch/           OSM road graph: localization, curvature ahead
│   ├── flowpilot/          flowpilot lateral planner port
│   ├── visionpilot/        alternative spatial MPC
│   └── utils/              vehicle model, PID, pure pursuit, calibrations
├── proto/                  protobuf: bag format and ZMQ exchange
├── assets/                 config.json, supercombo.onnx, DBC
└── scripts/                Python: bag analysis, simulator, visualizer, pyadas
docs/                       engineering reports + Jupyter Book course
scripts/                    build, dependency install, adb helpers
```

## Control pipeline

```
camera 20 Hz → calibration warp → supercombo ONNX (11.4 Hz, 88 ms, p90 103)
        │
        └─► vision/lanes ─ZMQ→ TopicConvert ─► vision/path (plan + lane markings)
                                                    │
                                   LaneKeepService: MPC → κ → steer angle → angle-PID
                                                    │
                                   controls/steer ─► PandaService 100 Hz ─► HCA_01 on bus
```

Full chain with all fields and timings:
[`docs/IMAGE_TO_CAN_PIPELINE.md`](docs/IMAGE_TO_CAN_PIPELINE.md).

Measured latencies: capture → command publish 69 ms, command → rack 40 ms, rack → yaw
120 ms, **total capture → vehicle response ≈ 230 ms**. The planner uses the actual frame step,
not a hard-coded constant — after that fix, vision jitter stopped affecting the command
(measured vision `frame_dt`, not fixed `DT_MDL`).

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
| `lat_use_vehicle_model` / `tire_stiffness_factor` | `true` / `0.64` | κ→angle with understeer correction (measured from bags) |
| `fp_steer_delay_s` | `0.35` | feedforward in `get_lag_adjusted_curvature` |
| `min_control_speed_mps` | `1.5` | below this, lateral command is zeroed |
| `path_lane_blend_scale` | `0.6` | lane-center share in reference (only when **both** lines visible) |
| `lane_std_good_m` / `lane_std_bad_m` | `0.3` / `1.5` | line weight by model σ: fully used up to `good`, ignored from `bad` |
| `lane_width_min_m` / `lane_width_max_m` | `2.6` / `4.6` | plausible lane-width bounds; width is filtered |
| `center_force_gain` | `0.4` | centering term: reference shift by vehicle offset from lane center |
| `path_camera_offset_m` | `0.05` | path shift right: compensates model plan left bias |
| `calibration.camera.position_m` | — | camera mount: `y_left`, `z_up`, `x_forward` |

Lateral block values tuned in closed loop on arcs.

The file is copied to `filesDir` on first launch and **is not overwritten** by APK install: to
pick up new defaults, use `scripts/push_config.sh --apply` (preserves camera calibration)
or clear app data.

### Live tunables

Some parameters can be changed **without restart** — in-app parameter panel, `set_param` from
the Python host, or JSON via JNI. A service registers a knob by name (`registerParameter`);
writes from another thread only queue the value, and middleware applies it on that service's
thread between callbacks. A new knob needs neither its own JNI method nor race reasoning —
just a `registerParameter` line in the service.

```cpp
// in service
registerParameter<double>("path_lane_blend_scale",
                          [this](const double& v) { setLaneBlendScale(v); },
                          [this] { return path_cfg_.lane_blend_scale; });
```

```python
app.set_param("path_lane_blend_scale", 1.0)   # replay on bag
app.get_param("steer_ratio")
```

One name can be registered in multiple services — then the value reaches each; that is how
`steer_ratio` works, needed by both the controller and steer-angle conversion. Registered:
`steer_ratio`, `max_steer_deg`, `cam_y_left_m`, `lane_keep_controller`, `pp_*`, `steer_slew_limit_deg`,
`tire_stiffness_factor`, `lat_use_vehicle_model`, `fp_steer_delay_s`, `fp_steering_rate_weight`,
`path_lane_blend_scale`, `path_camera_offset_m`, `center_force_gain`, `lane_std_good_m`,
`lane_std_bad_m`.

## Simulator: controller run

Closed loop on a generated track with straights and arcs of known radius; evaluation uses
ground truth, separately on straights and arcs. The controller reads parameters from `config.json`, so
what would ship in the APK is what gets tested. Non-zero exit code = failed thresholds; usable as a
regression test.

```bash
cd app/src/main/scripts
python3 -m sim.eval --list-tracks                                  # highway / curvy / tight / straight
python3 -m sim.eval --track highway --controllers fp,pure_pursuit
python3 -m sim.eval --track curvy --controllers fp --plot out.png
python3 -m sim.eval --track straight --controllers fp --offset 1.1  # return to center from offset
python3 -m sim.vehicle_calib                                       # simulator vehicle understeer
```

Example output:

```
  fp | highway seed=7 v=25 m/s | 3043 m / 126 s | end: max_steps | LDW 0 / FCW 0
segment           sec        R, m  |CTE| med     p95    max  exit %  sat %   HF °  a_lat p95
straight           39           —       0.05    0.20    0.26      0.0      0.0   0.00        0.0
arc R>=400         77     508–699       0.03    0.16    0.24      0.0      0.0   0.00        1.2
  threshold passed
```

What this test proves and what it does not (ideal reference, different vehicle) —
[`docs/SIM_CONTROLLER_TEST.md`](docs/SIM_CONTROLLER_TEST.md).

### Visual inspection


Same tracks, same 90 ms step and same reference as `sim.eval` — with a window:

```bash
./scripts/run_sim.sh --track highway --controller fp                  # MetaDrive 3D window
./scripts/run_sim.sh --track curvy --controller fp --show             # + camera window with overlay
./scripts/run_sim.sh --track curvy --lanes supercombo --show          # model overlay on image
./scripts/run_sim.sh --track straight --offset 1.1 --show             # lane departure and return
./scripts/run_sim.sh --track random --controller fp                   # random MetaDrive map
```

`--track` defaults to `highway`. Status line shows segment (straight / arc R), speed,
true CTE and steer command; on exit — median |CTE| for the run, comparable to `sim.eval`.
Camera window (`--show`) closes on `q` or `Esc`; `--max-frames N` stops the run automatically.

`--lanes supercombo` is fine to see what the model sees on the image, but **not** to drive on:
supercombo is trained on a real camera and breaks on MetaDrive synthetic curves
(line probabilities drop to 0.03–0.13, vehicle leaves the lane). Run closed loop on
`--lanes gt` — details in [`docs/SIM_CONTROLLER_TEST.md`](docs/SIM_CONTROLLER_TEST.md).

## Offline tools

```bash
./scripts/pull_bags.sh                                  # pull bags from phone to ./adas_logs
./scripts/run_bag_vis.sh adas_logs/<session>             # interactive bag viewer

PYTHONPATH=app/src/main/scripts python3 \
  app/src/main/scripts/bag_controller_ab.py adas_logs/<session> --controllers fp,mpc --plot out.png
PYTHONPATH=app/src/main/scripts python3 \
  app/src/main/scripts/bag_config_sweep.py adas_logs/<session> --t0 100 --t1 260
PYTHONPATH=app/src/main/scripts python3 \
  app/src/main/scripts/bag_safety_warn.py adas_logs/<session>
```

`bag_controller_ab.py` — open-loop controller comparison against driver steering;
`bag_config_sweep.py` — closed-loop config comparison on measured vehicle model
(understeer + delays), with lane-marking centering metric;
`bag_safety_warn.py` — replay bag through the real warning chain and count triggers
(on a normal bag any trigger is a false positive).

## Tests

```bash
./app/src/main/cpp/build_cpp.sh -t linux --test     # 59 tests, 5 skipped (ZMQ harness needed)
./scripts/docker.sh tests                           # same in container
```

Coverage: lateral controllers (speed gate, limits, lane-marking blend), pure pursuit,
PID, VW CAN (HCA/counter/CRC), middleware, FCW/AEB/LDW warnings.

## Documentation

Engineering reports and course — [`docs/README.md`](docs/README.md).

Start with the [course book](docs/book/Introduction/intro.md), then
[`docs/IMAGE_TO_CAN_PIPELINE.md`](docs/IMAGE_TO_CAN_PIPELINE.md) and
[`docs/CONTROLLER_LIMITS.md`](docs/CONTROLLER_LIMITS.md). Index: [`docs/README.md`](docs/README.md).
