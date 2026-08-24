# Reading List and Internal Docs

## External

* [Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) (CC BY 4.0)
* Coursera *Introduction to Self-Driving Cars* (lateral control weeks)
* openpilot / flowpilot — Supercombo and stock lateral

**Background for the gaps the chapters open** (read the named one first if that chapter is hard):

* camera model / pinhole / homography — Szeliski, *Computer Vision*, ch. 2 (for [IntrinsicsAndWarp](../Calibration/IntrinsicsAndWarp.md));
* IDM car-following — Treiber, Hennecke & Helbing 2000, *Congested traffic states…* (for [Safety](../Safety/Warnings.md));
* Kalman/EKF — Labbe, *Kalman and Bayesian Filters in Python* (for [Localization](../Localization/Overview.md));
* protobuf & ZMQ — the official protobuf "Basics" tutorial and the ZeroMQ guide (for [Bags](../Logging/Bags.md), [Latency](../Latency/Overview.md)).

## Internal (`docs/`)

| file | topic |
|---|---|
| `IMAGE_TO_CAN_PIPELINE.md` | camera→HCA |
| `CONTROLLER_LIMITS.md` / `BACKLOG.md` | limits / open work |
| `BENCHMARK_COMMA2.md` | vs comma-two + closing plan |
| `SAFETY_WARN.md` | FCW / AEB / LDW |
| `THNEED.md` | the GPU path: what a thneed is, how ours is generated and checked |
| `NEW_PHONE.md` | bringing up a new phone, and what to measure before driving |
| `PORTING.md` | adding a car behind `CarPlatform` |
| `PARAMSD.md` | params learning |
| book `Localization/Overview.md` | GPS ENU, phone IMU, live EKF |
| `TRAFFIC_VISION.md` | the traffic-light vertical |
| `SIM_CONTROLLER_TEST.md` | MetaDrive eval |

## Code anchors

| path | why |
|---|---|
| `include/adas/middleware/manager.hpp` | Service / ParamBag |
| `tests/test_middleware.cpp` | smallest examples |
| `VisionPipeline.java` | runner selection, inference queue |
| `src/services/planner.cpp` | which lateral strategy runs |
| `src/services/control.cpp` | the control law, no CAN in sight |
| `src/services/platform.cpp` + `src/platform/` | the bus, and the only place a brand is named |
| `src/services/safety_warn.cpp` | FCW / AEB / LDW |
| `scripts/tools/thneed_from_onnx.py` | ONNX → GPU run, and the checks around it |
| `online_localizer` / `imu_calibrator` / `gps_local_projector` | ENU pose / IMU lock / GPS |
| `assets/config.json` | all levers |

**End of course.** [Back to start](../Introduction/intro.md)
