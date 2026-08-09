# ADAS documentation

## Start here

| document | topic |
|---|---|
| [`book/`](book/) | course, English source — [live site](https://anatolykabakov.github.io/adas/) |
| [`book_ru/`](book_ru/) | course, Russian, all 23 chapters — [live site](https://anatolykabakov.github.io/adas/ru/) |
| [`IMAGE_TO_CAN_PIPELINE.md`](IMAGE_TO_CAN_PIPELINE.md) | camera frame → HCA CAN |
| [`MAPMATCH.md`](MAPMATCH.md) | offline track-shape localization (complement to live EKF) |
| [`CONTROLLER_LIMITS.md`](CONTROLLER_LIMITS.md) | where the assistant works |
| [`BACKLOG.md`](BACKLOG.md) | open work in priority order |
| [`PREDRIVE.md`](PREDRIVE.md) | what ships on the next drive, what it asks, and what would count as an answer |
| [`LATERAL_CHAIN_RU.md`](LATERAL_CHAIN_RU.md) | поперечный тракт по этапам: механика, измеренные числа, где команда умирает (на русском) |
| [`DIFF_FROM_DP_RU.md`](DIFF_FROM_DP_RU.md) | отличия от dragonpilot/flowpilot: что перенесено дословно, где разошлись сознательно, что недонесли (на русском) |
| [`BENCHMARK_COMMA2.md`](BENCHMARK_COMMA2.md) | vs comma-two + plan to close the gap |
| [`SIM_CONTROLLER_TEST.md`](SIM_CONTROLLER_TEST.md) | MetaDrive closed-loop eval |
| [`BAG_ANALYSIS.md`](BAG_ANALYSIS.md) | разбор заезда одной командой поверх кэша, включая голосовые пометки водителя |

## Design and subsystems

| document | topic |
|---|---|
| [`SAFETY_WARN.md`](SAFETY_WARN.md) | FCW / AEB / LDW |
| [`MPC_EXPLAINED.md`](MPC_EXPLAINED.md) | VisionPilot MPC (`mpc`) |
| [`PARAMSD.md`](PARAMSD.md) | upstream `paramsd` |
| [`MAPMATCH.md`](MAPMATCH.md) | track-shape localization |
| [`MAP_CURVATURE.md`](MAP_CURVATURE.md) | road curvature ahead from OSM (`map_data`) |
| [`TRAFFIC_VISION.md`](TRAFFIC_VISION.md) | YOLO signs / lights |
| [`MODEL_LONG_PLAN.md`](MODEL_LONG_PLAN.md) | long plan / lead |
| [`CRUISE_BUTTONS.md`](CRUISE_BUTTONS.md) | stock cruise via GRA |

## Course build

```bash
cd docs && pip install -r book/requirements.txt
./build_book.sh            # both languages: EN into _site, RU into _site/ru
./build_book.sh --check     # snippets run + translation in sync, no build
```

Two source trees rather than gettext: `.po` files are worse than markdown for prose full of formulas,
tables and code. The drift that this invites is caught mechanically — `book/sync_translation.py` requires
the same pages, the same heading structure, and **byte-identical code blocks**, because prose is translated
and code never is. The build also verifies that every page has a counterpart in the other language, so the
ENG/RU button cannot land on a 404.

Deploy: push `docs/book/**` → `gh-pages`. Start: [`book/Introduction/intro.md`](book/Introduction/intro.md).
