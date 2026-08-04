# System Overview

We want a phone on the windshield to keep a Golf 7 near lane center by commanding MQB **HCA** through a USB **Panda**.
This chapter is the map of the repository before we dive into vision and control math.

## Purpose

The ADAS app:

1. captures the road with the camera (and supporting IMU / GPS);
2. runs **Supercombo** inference (ONNX Runtime on device);
3. computes a lateral lane-keep command in native **C++**;
4. publishes `HCA_01` on VW MQB CAN via Panda;
5. writes a full **bag** for offline analysis.

This is a research / teaching lane-keep stack — not a full ACC / "autopilot" product.

```{figure} figures/pipeline_simple.png
---
width: 95%
---
End-to-end path you will learn to trace: sensors → model → path → control → CAN.
```

## Layers

| layer | components | role |
|---|---|---|
| **Java** | Camera / IMU / GPS, VisionPipeline, Logger, ZMQ | sensors, ORT, UI, bag I/O |
| **C++** | `AdasApp`, LaneKeep, Calib, Panda, … | algorithms and actuation |
| **Python** | vis, latency, sweeps, MetaDrive, `pyadas` | analysis and host-sim |

**Design rule:** lateral algorithms live in C++. Python either analyzes a bag or drives the **same** native code through `pyadas` (`publish → step → pop_messages`). That keeps lab sweeps honest relative to the phone.

Next in this part: [Middleware](./Middleware.md) (native bus) → [Java layer](./JavaLayer.md) (camera / ORT / ZMQ) → [Pipeline](./Pipeline.md) (frame → HCA). Also [FCW / AEB / LDW](../Safety/Warnings.md) for warnings without actuation.

## Configuration you will touch

`app/src/main/assets/config.json` (optional override in app `filesDir`).

Vehicle knobs (`vehicle.*`):

```json
"lane_keep_controller": "fp",
"lat_use_vehicle_model": true,
"tire_stiffness_factor": 0.64,
"fp_steer_delay_s": 0.35
```

Feature flags (`nodes.*`), e.g. `"vision_traffic": false`, `"phone_stats": true`,
`"safety_warn": true`.

| key | purpose |
|---|---|
| `vehicle.lane_keep_controller` | `pp` \| `mpc` \| `fp` |
| `vehicle.lat_use_vehicle_model` | $\kappa\to$ SWA via understeer model |
| `vehicle.fp_steer_delay_s` | state lookahead for pipeline delay |
| `nodes.vision_traffic` | YOLO; keep `false` when measuring lane-keep |
| `nodes.phone_stats` | 1 Hz CPU / thermal into the bag |

## Typical student workflow

1. Record or download a session under `adas_logs/...`.
2. Run `latency.py`, bag visualizer, PlotJuggler — establish Hz / e2e.
3. Sweep parameters (`bag_config_sweep.py`) at a **fixed** assumed vision latency.
4. Only then open `LaneKeepService` / `PurePursuit` source with Control chapters in hand.

```{tip}
If CTE looks terrible, ask three questions in order: Is vision ≥ ~9 Hz? Is $y$ / `steer_sign` consistent? Is calib sane? Gains come fourth.
```

<!-- next-chapter -->
---

**Next:** [Middleware](./Middleware.md)
