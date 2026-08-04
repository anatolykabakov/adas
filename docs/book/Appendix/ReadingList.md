# Reading List and Internal Docs

## External

* [Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) (CC BY 4.0)
* Coursera *Introduction to Self-Driving Cars* (lateral control weeks)
* openpilot / flowpilot — Supercombo and stock lateral

## Internal (`docs/`)

| file | topic |
|---|---|
| `IMAGE_TO_CAN_PIPELINE.md` | camera→HCA |
| `CONTROLLER_LIMITS.md` / `BACKLOG.md` | limits / open work |
| `BENCHMARK_COMMA2.md` | vs comma-two + closing plan |
| `SAFETY_WARN.md` | FCW / AEB / LDW |
| `MPC_EXPLAINED.md` | VisionPilot MPC |
| `PARAMSD.md` / `MAPMATCH.md` | params learning / offline map match |
| book `Localization/Overview.md` | GPS ENU, phone IMU, live EKF |
| `TRAFFIC_VISION.md` / `MODEL_LONG_PLAN.md` / `CRUISE_BUTTONS.md` | adjacent |
| `SIM_CONTROLLER_TEST.md` | MetaDrive eval |

## Code anchors

| path | why |
|---|---|
| `middleware/middleware.hpp` | Service / ParamBag |
| `test_middleware.cpp` | smallest examples |
| `VisionPipeline.java` | inference queue |
| `lane_keep_service.cpp` | controller selection |
| `safety_warn_service.cpp` | FCW / AEB / LDW |
| `online_localizer` / `imu_calibrator` / `gps_local_projector` | ENU pose / IMU lock / GPS |
| `assets/config.json` | all levers |

**End of course.** [Back to start](../Introduction/intro.md)
