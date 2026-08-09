# Слой Java (обвязка телефона)

Алгоритмы принадлежат C++. **Java** принадлежит мир Android: камера, ONNX Runtime, беги, интерфейс и мост ZMQ в Middleware.

Корень пакета: `app/src/main/java/ai/flow/adas/`.

## Разделение ответственности

| Остаётся в **Java** | Живёт в **C++** |
|---|---|
| Camera2 / IMU / GPS | удержание полосы (`pp` / `mpc` / `fp`) |
| Supercombo и опциональный YOLO (ORT) | SafetyWarn, LongPlan, TopicConvert |
| `Logger` / `BagLogger` (беги сессии) | приём и передача CAN панды, HCA |
| Превью и наложение (`LaneOverlayView`) | шина Middleware и таймеры |
| Сокеты PUB/SUB в `ZMQBridgeService` | нативный сервис моста ZMQ |

```{admonition} Правило устройства
:class: tip
Если это решает, **как рулить или предупреждать**, лучше C++ (тестируется через `pyadas`).
Если это требует **Camera2, Activity или ORT для Android**, оно остаётся в Java.
```

## Путь кадра (камера → нативный код)

```text
CameraHandler (YUV 1280×720, capture_ts)
    → VisionPipeline.submitYuv   (latest-frame drop if busy)
        → ModelCalibWarp → 512×256
        → SupercomboOnnxRunner.run (ORT)
        → parse LaneLines / model_long / odometry
            → ProtoUtils.create…Message(...)
            → ZMQBridgeService.publishToNative(msg)   // :5555
            → Logger.logZMQMessage(...)               // bag copy
```

```java
// Shape of the control publish (names simplified)
Messages.ZMQMessage lanes =
    ProtoUtils.createLaneLinesMessage(laneLines, /*forBag=*/ false);
ZMQBridgeService.publishToNative(lanes);

Messages.ZMQMessage modelLong =
    ProtoUtils.createModelLongPlanMessage(ts, frameId, modelLong, pose);
ZMQBridgeService.publishToNative(modelLong);
```

```{warning}
JPEG в беге — это обычно **превью**, а не развёрнутый вход ORT $512\times 256$.
Офлайн-переинференс должен пересобрать warp — см. [Supercombo](../Vision/Supercombo.md).
```

## Мост ZMQ

| направление | адрес (по умолчанию) | содержимое |
|---|---|---|
| Java → C++ | `tcp://127.0.0.1:5555` | `vision/lanes`, `vision/model_long`, датчики, … |
| C++ → Java | `tcp://127.0.0.1:5556` | `safety/warn`, отладка руления, статистика, … |

Конфиг: `zmq.endpoint_in` / `zmq.endpoint_out` в `config.json`.

```python
# Student checklist when "native never moves"
checks = [
    "nodes.zmq_bridge == true",
    "VisionPipeline actually finishing ORT (infer_ms finite)",
    "publishToNative called (logcat)",
    "C++ ZmqBridgeService polling IN",
    "TopicConvert producing vision/path for LaneKeep",
]
for c in checks:
    print("-", c)
```

## Конфиг и живые параметры

1. **Поставляемый JSON:** `app/src/main/assets/config.json` (копируется в `filesDir` при первом запуске и **не** перезаписывается при обновлении).
2. **`AdasConfig`** — какие узлы включены (`nodes.lane_keep`, `nodes.safety_warn`, …).
3. **`RuntimeParams` и слайдеры интерфейса** — ручки в духе PP и калибровки прямо на ходу.
4. **`AdasAppHandler.applyLaneKeepParams`** → JNI → `Middleware::setParameter`.

```python
# What a slider change means on the bus
ui_value = 0.8
# Java sends string "0.8" for name "pp_k_dd"
# ParamBag on LaneKeep's thread parses → double pp_k_dd_
assert float("0.8") == ui_value
```

Залить новый файл на телефон без пересборки APK: `./scripts/push_config.sh` (см. скрипты репозитория).

## Беги

`Logger` создаёт каталог сессии в хранилище приложения; каждый топик — поток protobuf.
Студенты забирают их через `./scripts/pull_bags.sh`, дальше:

```bash
cd app/src/main/scripts
python3 latency.py /path/to/adas_logs/SESSION
python3 -m vis.export_to_plotjuggler /path/to/SESSION -o /tmp/out
```

Включите `phone_stats` в конфиге, чтобы получать процессор и температуры раз в секунду — это нужно для домашнего задания про задержки.

## Исходящее в интерфейс

`MainActivity` слушает выход ZMQ. Пример: `safety/warn` →

```java
laneOverlay.setSafetyWarn(fcw, aeb, lldw, rldw);
```

Текст на экране — **только отображение**. Он не тормозит и не рулит.

## Карта классов (читать в этом порядке)

| класс | роль |
|---|---|
| `MainActivity` | жизненный цикл, связывание обработчиков |
| `CameraHandler` | кадры и метки времени |
| `VisionPipeline` / `SupercomboOnnxRunner` | ORT |
| `ZMQBridgeService` / `ProtoUtils` | в нативный код |
| `Logger` | беги |
| `AdasConfig` / `RuntimeParams` | ручки |
| `AdasAppHandler` | JNI, запуск нативной части |
| `LaneOverlayView` | наложение на экране |

## Задания

1. Проследите один кадр: какой класс его отбросит, если ORT занят?
2. Мысленно переключите `nodes.vision_supercombo` в `false` — какие топики исчезнут?
3. По logcat или по коду перечислите три сообщения, которые Java публикует на `:5555`.

<!-- next-chapter -->
---

**Дальше:** [От кадра до CAN](./Pipeline.md)
