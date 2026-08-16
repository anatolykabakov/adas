# Что читать дальше и внутренние документы

## Внешнее

* [Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) (CC BY 4.0)
* Coursera *Introduction to Self-Driving Cars* (недели про поперечное управление)
* openpilot / flowpilot — Supercombo и штатное поперечное управление

## Внутреннее (`docs/`)

| файл | тема |
|---|---|
| `IMAGE_TO_CAN_PIPELINE.md` | камера → HCA |
| `CONTROLLER_LIMITS.md` / `BACKLOG.md` | границы применимости / открытая работа |
| `BENCHMARK_COMMA2.md` | сравнение с comma-two и план сокращения разрыва |
| `SAFETY_WARN.md` | FCW / AEB / LDW |
| `MPC_EXPLAINED.md` | MPC из VisionPilot |
| `THNEED.md` | путь через GPU: что такое thneed, как наш собирается и проверяется |
| `NEW_PHONE.md` | подъём на новом телефоне и что замерить до выезда |
| `PORTING.md` | подключение машины за `CarPlatform` |
| `PARAMSD.md` / `MAPMATCH.md` | обучение параметров / офлайн-привязка к карте |
| глава книги `Localization/Overview.md` | ENU из GPS, IMU телефона, живой EKF |
| `TRAFFIC_VISION.md` / `MODEL_LONG_PLAN.md` / `CRUISE_BUTTONS.md` | смежное |
| `SIM_CONTROLLER_TEST.md` | оценка в MetaDrive |

## Опорные точки в коде

| путь | зачем |
|---|---|
| `include/adas/middleware/manager.hpp` | Service / ParamBag |
| `tests/test_middleware.cpp` | самые маленькие примеры |
| `VisionPipeline.java` | выбор раннера, очередь инференса |
| `src/services/planner.cpp` | какая поперечная стратегия работает |
| `src/services/control.cpp` | закон управления, ни одного упоминания CAN |
| `src/services/platform.cpp` и `src/platform/` | шина и единственное место, где названа марка |
| `src/services/safety_warn.cpp` | FCW / AEB / LDW |
| `scripts/tools/thneed_from_onnx.py` | ONNX → прогон на GPU и проверки вокруг него |
| `online_localizer` / `imu_calibrator` / `gps_local_projector` | поза в ENU / захват IMU / GPS |
| `assets/config.json` | все рычаги |

**Конец курса.** [В начало](../Introduction/intro.md)
