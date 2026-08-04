# ADAS documentation

## Start here

| document | topic |
|---|---|
| [`PIPELINE_AUDIT_0801.md`](PIPELINE_AUDIT_0801.md) | control pipeline audit: measured vehicle model, latencies, found and fixed defects |
| [`CONTROLLER_LIMITS.md`](CONTROLLER_LIMITS.md) | where the assistant works and where it does not: roads, speeds, arc radii, lane-marking availability |
| [`IMAGE_TO_CAN_PIPELINE.md`](IMAGE_TO_CAN_PIPELINE.md) | full path camera frame → HCA CAN command, step by step |
| [`SIM_CONTROLLER_TEST.md`](SIM_CONTROLLER_TEST.md) | controller run on simulator track: tracks, metrics, results, limits of the test itself |
| [`PLAN_TO_COMMA2.md`](PLAN_TO_COMMA2.md) | plan to bring steering to comma-two level, from dragonpilot logs on the same vehicle |
| [`VS_DRAGONPILOT_0803.md`](VS_DRAGONPILOT_0803.md) | comparison with dragonpilot on the same vehicle: where the gap is, plus stage-by-stage pipeline diff — what matches 1:1 and what diverges |
| [`BACKLOG.md`](BACKLOG.md) | open work in priority order: from pre-drive blockers to repo hygiene |

## Road bag analyses

| document | segment |
|---|---|
| [`RUN_0801_HIGHWAY.md`](RUN_0801_HIGHWAY.md) | one hour of highway, 35.6 min under control — main system numbers |
| [`RUN_0801_STRAIGHTS.md`](RUN_0801_STRAIGHTS.md) | straights and gentle arcs — where the assistant matches the driver |
| [`RUN_0801_CURVE.md`](RUN_0801_CURVE.md) | tight arc R≈41 m: controller understeer and power-steering limit |
| [`RUN_0801_LOWSPEED.md`](RUN_0801_LOWSPEED.md) | before stop: yaw wander and command spikes at low speed |
| [`RUN_0801_LEFT_DRIFT.md`](RUN_0801_LEFT_DRIFT.md) | left drift: model plan bias and its compensation |
| [`RUN_0802_ARC_OFFSET.md`](RUN_0802_ARC_OFFSET.md) | city, 31.6 min: vehicle cuts inside on arcs — model plan vs lane markings |
| [`RUN_0804_PERCEPTION.md`](RUN_0804_PERCEPTION.md) | city, 30.1 min: perception after focus fix, planner reference on lane center, 100 Hz loop confirmed, panda on comma-two firmware |

## Design and calibration

| document | topic |
|---|---|
| [`FRAME_DT_FIX_0801.md`](FRAME_DT_FIX_0801.md) | planner time step: why the command lagged 0.83 s and how it was fixed |
| [`CALIBRATION_FROM_BAGS.md`](CALIBRATION_FROM_BAGS.md) | what camera calibration is measurable from a bag (roll / y / x / scale) and what is not |
| [`MPC_EXPLAINED.md`](MPC_EXPLAINED.md) | how spatial MPC works (controller `mpc`) |
| [`MAPMATCH.md`](MAPMATCH.md) | global localization by track shape and OSM map, without GNSS: how it works, results, remaining work |
| [`PARAMSD.md`](PARAMSD.md) | upstream `paramsd`: what it learns (steer ratio, tire stiffness, steer offset, road roll) and what transfers to us |

## Adjacent subsystems

| document | topic |
|---|---|
| [`TRAFFIC_VISION.md`](TRAFFIC_VISION.md) | YOLO for signs and traffic lights, speed-limit HUD |
| [`MODEL_LONG_PLAN.md`](MODEL_LONG_PLAN.md) | longitudinal plan and lead vehicle from model output |
| [`SAFETY_WARN.md`](SAFETY_WARN.md) | FCW / AEB / LDW: rules, gates, and false-positive checks |
| [`CRUISE_BUTTONS.md`](CRUISE_BUTTONS.md) | stock cruise via GRA buttons (no ACC) |

## Course

[`book/`](book/) — Jupyter Book for students: pipeline, coordinates, bicycle model,
Pure Pursuit, MPC, delays, calibration. Live site:
[anatolykabakov.github.io/adas](https://anatolykabakov.github.io/adas/).
Partly inspired by
[AAD](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) (CC BY 4.0, see
[`book/ATTRIBUTION.md`](book/ATTRIBUTION.md)).

```bash
cd docs/book && pip install -r requirements.txt && jupyter-book build .
```

Deploy: push to `main` under `docs/book/**` (workflow `deploy-book` → branch `gh-pages`).
Enable Pages once: repo **Settings → Pages → Deploy from a branch → `gh-pages` / root**.

Chapter figures use plain Markdown, so preview works directly in the editor.
Start: [`book/Introduction/intro.md`](book/Introduction/intro.md).

---

Reports are dated by the bags they analyze (`RUN_0801_*` — drives on 1 August 2026). Older
tuning notes were removed: their numbers predate the time-step and vehicle-model fixes and
are no longer reproducible.
