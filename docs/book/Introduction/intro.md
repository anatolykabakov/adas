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
* implement / tune Pure Pursuit and explain how `fp` differs from one-step geometry;
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
3. `Planner` (`pp` | `fp`) produces a plan in curvature; `Control` turns it into SWA / torque.
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

Chapters are built the AAD way: each one goes **from a small working thing to the real one** — you
build a toy inside the chapter, watch it break with a number attached, then meet the shipped version of
the same idea. The order below is therefore also a build order, and what you construct accumulates:

1. The lateral loop, split like the code into three parts. **Planner** (what shape to drive):
   [Bicycle model](../Planner/BicycleModel.md) — simulate it, try the obvious controller, watch it
   oscillate, fix it with heading; [Pure Pursuit](../Planner/PurePursuit.md) — your own pursuit on a
   polyline, the trade-off measured; [Lane path](../Planner/LanePath.md) — fuse two noisy lines and the
   model plan into one reference, meet σ and its lies; [MPC / fp](../Planner/MPC_and_FP.md) — a
   path-domain MPC as a teaching toy, then the shipped **time-domain** `fp`, closed-loop in MetaDrive;
   [Longitudinal planner](../Planner/Longitudinal.md) — a P law on the gap that rings, then upstream's
   jerk-limited MPC on the same cost in numpy, closed-loop against a scripted lead. **Control** (what command):
   [Vehicle model](../Control/VehicleModel.md) — sabotage the toy with slip and delay, then measure
   $G(v)$ on the real car; [Angle control](../Control/AngleControl.md) — the inner loop against a toy
   rack: friction, the panda rate limiter, the winding integrator. **Platform** (onto the bus):
   [Platform](../Platform/Overview.md) — decode CAN through a DBC, build a legal HCA_01 with counter and
   checksum secret, and the panda supervisor that gates it.
2. [Vision](../Vision/Overview.md) — build a reference path from two noisy lines, meet σ and its lies;
   [coordinates](../Vision/Coordinates.md) → [Supercombo](../Vision/Supercombo.md).
3. [Middleware](../Architecture/Middleware.md) — write a bus in sixty lines, reproduce the backlog
   disease, then read the real one. [Architecture](../Architecture/Overview.md) →
   [Java](../Architecture/JavaLayer.md) → [Pipeline](../Architecture/Pipeline.md) — ending with the
   same services running on your laptop through `pyadas`.
4. [Bags](../Logging/Bags.md) — build the format in forty lines, **record your own bag** (no car
   needed), run your own controller on your own frames.
5. [Localization](../Localization/Overview.md) — an estimator built one sensor at a time, then repeated
   on your own recording.
6. [FCW / AEB / LDW](../Safety/Warnings.md), [Calibration](../Calibration/Overview.md) →
   [Latency](../Latency/Overview.md).
7. [Exercises](../Exercises/StudentProjects.md) — capstones; the small labs live inside the chapters.

Road experiments with HCA require an instructor. Default course mode: **bag + scripts**.

## Prerequisites

* Linear algebra and planar rigid-body kinematics; trigonometry.
* Python (`numpy`), reading protobuf / CSV; ability to read C++ is desirable.
* CNN / ONNX at user level (you will not train Supercombo).

Unfamiliar acronym? The [Glossary](../Appendix/Glossary.md) collects every term the chapters lean on.

Team engineering notes live under `docs/*.md`. This book is the **teaching path** into that material.

<!-- next-chapter -->
---

**Next:** [System overview](../Architecture/Overview.md)
