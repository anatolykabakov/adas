# Reading List and Internal Docs

## External

* [Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) — camera, IPM, bicycle, Pure Pursuit (CC BY 4.0).
* Coursera *Introduction to Self-Driving Cars* (weeks on lateral control) — alongside AAD Control.
* openpilot / flowpilot — Supercombo and stock lateral context.

## Internal primary sources (`docs/`)

| file | topic |
|---|---|
| `SAFETY_WARN.md` | FCW / AEB / LDW rules and FP history |
| `IMAGE_TO_CAN_PIPELINE.md` | full camera→HCA path |
| `MPC_EXPLAINED.md` | VisionPilot MPC breakdown |
| `PIPELINE_AUDIT_0801.md` | pipeline audit + vehicle model |
| `RUN_0801_STRAIGHTS.md` / `CURVE` / `LOWSPEED` | window analyses |
| `TRAFFIC_VISION.md` | YOLO / COCO |
| `MODEL_LONG_PLAN.md` | lead / long |
| `CONTROLLER_LIMITS.md` | applicability limits |
| `CALIBRATION_FROM_BAGS.md` | what is calibrated from runs |
| `RUN_0801_LEFT_DRIFT.md` | left drift and its cause |

Book **does not replace** these notes: it builds a teaching path and links to them.

## Code "anchors"

| path | why open |
|---|---|
| `…/cpp/.../middleware/middleware.hpp` | Service / ParamBag / timers |
| `…/cpp/.../test_middleware.cpp` | smallest working examples |
| `…/java/.../VisionPipeline.java` | inference queue |
| `…/java/.../ZMQBridgeService.java` | Java ↔ native |
| `…/cpp/.../lane_keep_service.cpp` | controller selection |
| `…/cpp/.../safety_warn_service.cpp` | FCW / AEB / LDW |
| `…/cpp/.../vehicle_model.h` | understeer |
| `…/scripts/latency.py` | delays |
| `…/scripts/bag_safety_warn.py` | warning episodes on bag |
| `…/assets/config.json` | all levers |

<!-- next-chapter -->
---

**End of course.** [Back to start](../Introduction/intro.md)
