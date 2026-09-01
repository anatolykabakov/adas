# System overview

A phone on the windshield keeps a Golf 7 near the lane centre. The camera sees the road, a neural network turns
the picture into lane lines and a path, C++ code turns the path into a steering torque, and a small USB
adapter to the car's CAN bus — the **panda** — puts that torque on the wire as `HCA_01`, the same frame the
factory lane assist would have sent, so the car obeys. This chapter is the map of the project: what the app
does, what it is made of, and how to work with it.

## What the app does

For every camera frame, the app:

1. captures the road, and reads the phone's IMU and GPS alongside;
2. runs the **Supercombo** network on the phone's GPU through a generated `.thneed` (ONNX Runtime is the
   slower fallback);
3. fuses the network's lane lines and plan into one "drive here" line, computes the curvature and speed to
   follow it, and then the commands — a steering torque and an acceleration — all in C++;
4. packs the commands into the car's own CAN frames (`HCA_01` for steering, `ACC_06/07` for speed) and hands
   them to the panda, which checks them against its safety model before anything reaches the bus;
5. writes everything it saw and decided into a **bag** — the drive's journal file, for analysis on a computer.

One honest caveat. This is a research and teaching stack, not an "autopilot": steering is proven on the road
on this very Golf; speed control (plan → acceleration → the car's ACC frames) is proven in the simulator
only, because this car has no radar and its motor and brakes do not accept the request.

```{figure} figures/pipeline_simple.png
---
width: 95%
---
The end-to-end path you will learn to trace: sensors → network → path → control → CAN.
```

## What it is made of

Three languages, each with its own job:

| layer | what lives there | responsible for |
|---|---|---|
| **Java** | camera, IMU, GPS, `VisionPipeline`, logger, ZMQ | sensors, running the network, UI, writing bags |
| **C++** | `AdasApp`, `Planner`, `Control`, `Platform`, localization, calibration | every algorithm, and driving the car |
| **Python** | bag viewer, `tools/latency.py`, parameter sweeps, MetaDrive, `pyadas` | analysis and simulation on a computer |

The main rule: **all algorithms are written in C++, and only there**. Python never re-implements a
controller — it either reads a bag or runs the *same* C++ code through the `pyadas` module (feed data →
step → collect results). The controller itself sees only two inputs, a path polyline in the car's frame and
a speed, and cannot tell whether they came from the live camera, a recording or the simulator — so a
parameter sweep on a laptop means exactly what it would mean on the phone, and most of this book's work
happens on recordings, with no car needed.

Inside the C++, control is split into three services, each answering its own question:

- `Planner` — *what trajectory to drive*: a curvature and a speed plan;
- `Control` — *what command produces it*: a steering torque and an acceleration;
- `Platform` — *how to put the command on the bus*. This is the only service that knows the car is a
  Volkswagen: CAN addresses, counters, checksums and the panda's safety model live here and nowhere else.

The interface between `Control` and `Platform` has seventeen methods and, today, one implementation — the
VW MQB platform with its seventeen car models. A second make would change neither `Planner` nor `Control`
(`docs/PORTING.md` describes the procedure). The book's parts follow the same cut: Vision → Localization →
Planner → Control → Platform.

Everything is configured by one file, `app/src/main/assets/config.json`: which car (`vehicle.name`), which
controller steers (`vehicle.lane_keep_controller`: `pp` or `fp`), which runner executes the network
(`vision.model_runner`), whether to take the longitudinal axis (`vehicle.long_control`). Each key is
introduced in the chapter where it matters, not listed here.

One habit this book insists on: measure before you touch code. On one real drive the car did not steer at
all while the app logged no error — the panda was sending a 57-byte health packet where the code expected
58, so every field after the third, the "controls allowed" flag among them, was read from the wrong offset.
No amount of controller reading would have found that; comparing two independent measurements of the same
thing found it in an evening, and the [Bags](../Logging/Bags.md) chapter teaches exactly that skill.

