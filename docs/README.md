# ADAS documentation

## Start here

| document | topic |
|---|---|
| [`book/`](book/) | course — [live site](https://anatolykabakov.github.io/adas/) |
| [`IMAGE_TO_CAN_PIPELINE.md`](IMAGE_TO_CAN_PIPELINE.md) | camera frame → HCA CAN |
| [`MAPMATCH.md`](MAPMATCH.md) | offline track-shape localization (complement to live EKF) |
| [`CONTROLLER_LIMITS.md`](CONTROLLER_LIMITS.md) | where the assistant works |
| [`BACKLOG.md`](BACKLOG.md) | open work in priority order |
| [`BENCHMARK_COMMA2.md`](BENCHMARK_COMMA2.md) | vs comma-two + plan to close the gap |
| [`SIM_CONTROLLER_TEST.md`](SIM_CONTROLLER_TEST.md) | MetaDrive closed-loop eval |

## Design and subsystems

| document | topic |
|---|---|
| [`SAFETY_WARN.md`](SAFETY_WARN.md) | FCW / AEB / LDW |
| [`MPC_EXPLAINED.md`](MPC_EXPLAINED.md) | VisionPilot MPC (`mpc`) |
| [`PARAMSD.md`](PARAMSD.md) | upstream `paramsd` |
| [`MAPMATCH.md`](MAPMATCH.md) | track-shape localization |
| [`TRAFFIC_VISION.md`](TRAFFIC_VISION.md) | YOLO signs / lights |
| [`MODEL_LONG_PLAN.md`](MODEL_LONG_PLAN.md) | long plan / lead |
| [`CRUISE_BUTTONS.md`](CRUISE_BUTTONS.md) | stock cruise via GRA |

## Course build

```bash
cd docs/book && pip install -r requirements.txt && jupyter-book build .
```

Deploy: push `docs/book/**` → `gh-pages`. Start: [`book/Introduction/intro.md`](book/Introduction/intro.md).
