# Algorithms for Phone ADAS

This (mini-)book guides you through the **Android ADAS** stack used in our project: a phone on the windshield, USB Panda, Volkswagen MQB (Golf 7), and lateral lane keeping via HCA.

The teaching arc follows [Algorithms for Automated Driving (AAD)](https://github.com/thomasfermi/Algorithms-for-Automated-Driving): **geometry → model → algorithm → measurement on data**.
Bicycle-model and Pure Pursuit chapters reuse AAD figures and derivations (CC BY 4.0). Architecture, Supercombo, `fp` / MPC, latency, and bag tooling are specific to Phone ADAS.

## Course goals

After working through the book you should be able to:

* trace a $1280\times 720$ camera frame to CAN frame `HCA_01`;
* explain Middleware pub/sub, timers, and live parameters;
* locate the Java shell (camera, ORT, ZMQ, bags) vs C++ algorithms;
* distinguish wheel steer angle $\delta$ from steering-wheel angle (SWA) on the bus;
* implement / tune Pure Pursuit and explain how `mpc` / `fp` differ from one-step geometry;
* account for understeer (`tire_stiffness_factor`) and transport delay (`fp_steer_delay_s`);
* project GPS to local ENU, explain phone IMU lock, and read `localization/pose`;
* compute FCW/AEB TTC / $a_{\mathrm{req}}$ and gated LDW conditions;
* separate vision degradation (thermal, competing YOLO) from controller error when reading a bag.

## Pipeline at a glance

```{figure} ../Architecture/figures/pipeline_simple.png
---
width: 95%
---
Camera → Supercombo → metric path → lane-keep → Panda / HCA.
```

1. Phone camera captures a frame.
2. Supercombo (on the GPU, ONNX as fallback) publishes lane polylines / plan in meters.
3. `Planner` (`pp` | `mpc` | `fp`) produces a plan in curvature; `Control` turns it into SWA / torque.
4. `Platform` sends `HCA_01`; EPS applies torque (supervised use only).

## Difference from AAD

| | AAD | This course |
|---|---|---|
| Environment | CARLA + Python exercises | Android + native C++ + offline Python |
| Lane detection | you build segmentation + IPM | Supercombo (openpilot family) as a fixed sensor |
| Actuation | simulator | MQB HCA (instructor-supervised on road) |
| Data | simulator | road bags + MetaDrive on host |

Use AAD when you want to implement a detector and Pure Pursuit **from scratch**.
Use this book when you want an **industry-near phone pipeline** and discipline around real logs.

## Recommended order

1. [Architecture](../Architecture/Overview.md) → [Middleware](../Architecture/Middleware.md) → [Java](../Architecture/JavaLayer.md) → [Pipeline](../Architecture/Pipeline.md)
2. [Vision](../Vision/Overview.md) → [coordinates](../Vision/Coordinates.md) → [Supercombo](../Vision/Supercombo.md)
3. [Localization](../Localization/Overview.md) (GPS ENU, phone IMU, EKF)
4. [Bicycle](../Control/BicycleModel.md) → [Pure Pursuit](../Control/PurePursuit.md) → [Vehicle model](../Control/VehicleModel.md) → [MPC / fp](../Control/MPC_and_FP.md)
5. [FCW / AEB / LDW](../Safety/Warnings.md)
6. [Calibration](../Calibration/Overview.md) → [Latency](../Latency/Overview.md) → [Bags](../Logging/Bags.md)
7. [Exercises](../Exercises/StudentProjects.md)

Road experiments with HCA require an instructor. Default course mode: **bag + scripts**.

## Prerequisites

* Linear algebra and planar rigid-body kinematics; trigonometry.
* Python (`numpy`), reading protobuf / CSV; ability to read C++ is desirable.
* CNN / ONNX at user level (you will not train Supercombo).

Team engineering notes live under `docs/*.md`. This book is the **teaching path** into that material.

<!-- next-chapter -->
---

**Next:** [System overview](../Architecture/Overview.md)
