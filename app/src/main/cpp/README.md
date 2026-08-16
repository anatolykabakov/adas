# ADAS C++

Всё управление и обработка: разметка → поперечный план → угол на руле, продольный план, локализация,
калибровка камеры, предупреждения, карта. Один и тот же код работает на телефоне (Android, arm64),
на хосте под тестами и как модуль Python в офлайн-разборах — разница только в источнике данных и в
том, кто крутит время.

## Структура

```
cpp/
├── include/adas/            публичные заголовки; путь включения повторяет пространство имён
│   ├── middleware/          шина: сервисы, типизированный pub/sub, таймеры, реестр параметров
│   ├── services/            сервисы — единицы работы на шине
│   ├── lateral/             поперечные регуляторы и общая для них модель автомобиля
│   ├── mapmatch/            дорожный граф OSM: привязка к дороге, кривизна впереди
│   ├── platform/            автомобиль за интерфейсом; volkswagen/ — пока единственная марка
│   ├── panda/               USB-драйвер и упаковка CAN
│   ├── traffic/             разбор выходов детектора знаков
│   ├── thneed/              вендоренный GPU-раннер из flowpilot (MIT)
│   └── utils/               модель автомобиля, PID, фильтры, калибровки, конвертация протобуфов
├── src/                     реализация, зеркало include/adas
│   └── python/              модуль pybind11: тот же C++, импортируемый как pyadas
├── tests/                   gtest, 223 случая
├── profiles/                профили conan
├── build_cpp.sh             conan + cmake, сборка и тесты
├── .clang-tidy              конфигурация статического анализа
├── CMakeLists.txt
└── conanfile.py
```

Пространства имён следуют каталогам: `adas/services/planner.h` объявляет `adas::services::Planner`,
`adas/middleware/manager.hpp` — `adas::middleware::Manager`. Общие типы данных остаются в `adas`:
они принадлежат системе, а не сервису, который их произвёл.

## Архитектура

Шина `middleware` даёт сервисам типизированный pub/sub, таймеры и живые параметры. В режиме
`RealTime` у каждого сервиса свой поток, просыпающийся на публикации или сроке таймера; в режиме
`Simulated` потоков нет вовсе — хост сам двигает время через `setTime` и `step`. Именно поэтому
офлайн-разбор бега и телефон исполняют один код.

```
внешний мир                    шина                          сервисы
──────────────────────────────────────────────────────────────────────────
Java (камера, GNSS, IMU) ──┐
                           ├─→ ZmqBridge ──→ типизированные топики ──→ Planner ──→ Control
бег / офлайн-стенд ────────┘        ↑                                Localization
                                    │                                CameraCalib
panda (CAN, USB) ────→ Platform ────┘                                MapData, SafetyWarn, …
```

`ZmqBridge` — единственное место, где внутренняя шина встречается с внешним миром. Входящие кадры
разбираются в схемные сообщения и публикуются на шине; исходящие заворачиваются в конверт
`ZMQMessage`. Сам конверт не существует больше нигде: сервисы обмениваются схемными сообщениями —
теми же, что лежат в записанном заезде.

`Platform` — только драйвер панды: байты в обе стороны плюс надзор за безопасностью. Разбор
`CarState` намеренно снаружи: это знание о марке, а не об USB, поэтому добавление автомобиля не
трогает этот сервис.

## Сборка

```bash
./build_cpp.sh -t android          # arm64-v8a, копирует .so в ../libs/arm64-v8a/
./build_cpp.sh -t linux            # хост
./build_cpp.sh -t linux --test     # хост + 223 теста; без --test тесты не собираются вовсе
./build_cpp.sh -c -t linux         # начисто
```

Сборка под Linux всегда включает биндинги Python и кладёт `pyadas/core*.so` в `scripts/pyadas/`.

После структурных изменений собирайте начисто: инкрементальная сборка молча переиспользует
объектные файлы, чей исходник не менялся, даже если менялся заголовок, и показывает зелёный свет на
коде, который уже не компилируется.

## Сервисы

| сервис | что делает |
|---|---|
| `Planner` | поперечный план: разметка → кривизна → требуемый угол колёс |
| `Control` | угол → момент на руле: PID, ограничение скорости изменения, гейты актюации |
| `Platform` | панда: приём и передача CAN, здоровье, надзор за безопасностью |
| `ZmqBridge` | граница с внешним миром в обе стороны |
| `Localization` | поза: EKF по колёсам, IMU и ГНСС |
| `CameraCalib` | онлайн-калибровка камеры по точке схода |
| `MapData` | OSM: где мы на дороге и что за поворот впереди |
| `SafetyWarn` | FCW, AEB, LDW |
| `TrafficSign` | состояние по знакам |
| `MiddlewareStats` | телеметрия самой шины раз в секунду |
| `InternalSubscriber` | доступ к топикам из офлайн-стенда |

Регистрируются в `adas_app.cpp::setupRealtimeServices()` (и в упрощённом наборе для
`Simulated`). Что именно поднимать, решает конфигурация.

## Топики

Канонический список — `include/adas/utils/adas_topics.h`. Основные:

| топик | тип | источник |
|---|---|---|
| `sensors/imu`, `sensors/imu_raw`, `sensors/imu_yaw` | IMU | Java → ZmqBridge → калибровка |
| `sensors/gps/location`, `sensors/gps/data` | ГНСС | Java |
| `vision/lanes` → `vision/path` | разметка и план пути | зрение → разбор |
| `vehicle/state`, `vehicle/chassis` | состояние автомобиля | Platform / бег |
| `can/rx`, `panda/health` | CAN и здоровье панды | Platform |
| `control/lat_plan`, `control/lane_keep`, `control/lane_keep_debug` | поперечный план и отладка | Planner |
| `controls/steer` | команда на руль | Control → Platform |
| `control/long_plan`, `vision/model_long` | продольный план | продольная часть |
| `localization/pose` | поза | Localization |
| `calibration/camera`, `calibration/camera_debug`, `calibration/lane_uv` | калибровка | CameraCalib |
| `map/local` | положение на дороге, кривизна впереди | MapData |
| `safety/warn`, `traffic/state` | предупреждения и знаки | SafetyWarn, TrafficSign |
| `middleware/stats` | телеметрия шины | MiddlewareStats |

Что уходит наружу и попадает в бег — список `kZmqOutboundTopics` в `include/adas/services/zmq_bridge.h`.

## Новый сервис

```cpp
#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"

namespace adas {
namespace services {

/// \brief Одна строка о том, за что сервис отвечает.
class MyService : public adas::middleware::Service {
public:
  void configure() override
  {
    subscribe<ChassisSample>(topics::kVehicleChassis, [this](const ChassisSample& s) { onChassis(s); });
    scheduleTimer(
        50, [this] { tick(); }, "my_service");
  }

  void reset() override { /* состояние в исходное */ }
  std::string_view getName() const override { return "my_service"; }
};

}  // namespace services
}  // namespace adas
```

Дальше: файл в `src/services/`, заголовок в `include/adas/services/`, регистрация в
`adas_app.cpp::setupRealtimeServices()`, тест в `tests/`. Публичный API документируется в стиле
doxygen — `\brief`, `\param[in]`, и каждое поле `Config` с `///<`.

## Тесты

```bash
./build_cpp.sh -t linux --test                     # все 223

# выборочно: conanrun.sh кладёт в LD_LIBRARY_PATH библиотеки acados,
# без него бинарь не найдёт libhpipm.so
cd build/linux/Release && . ./conanrun.sh
./tests/adas_tests --gtest_filter='Planner*'
```

Покрыто: поперечные регуляторы (гейт по скорости, ограничения, смешивание разметки, упреждение),
чистое преследование, PID, CAN VW (HCA, счётчик, CRC), шина, предупреждения FCW/AEB/LDW, выученные
параметры автомобиля, крен дороги, фильтр скорости, EKF, привязка к дорожному графу, конфигурация.

Статический анализ — clang-tidy, [`scripts/cpp/README.md`](../../../../scripts/cpp/README.md).

## Зависимости

Через conan (`conanfile.py`): protobuf 3.21.12, cppzmq 4.10.0, libusb 1.0.26, jsoncpp 1.9.6,
eigen 3.4.0, acados 0.1.8; под тесты gtest 1.14.0, под биндинги pybind11 2.11.1.

## Платформы

**Android**: arm64-v8a, minSdk 24, compileSdk 34, приоритеты через `setpriority`, логи в logcat.
**Linux**: x86_64, SCHED_RR (нужен `CAP_SYS_NICE`), логи в stdout, полный набор тестов и биндинги.

## Наблюдаемость

Не гадать, а смотреть: `MiddlewareStats` раз в секунду публикует время обратных вызовов, дрейф
таймеров, отставания и потери. Это уходит в бег и разбирается через
[`scripts/bag/bag_middleware_stats.py`](../../../../scripts/bag/bag_middleware_stats.py).
