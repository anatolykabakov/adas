# Слой Java (обвязка телефона)

Алгоритмы принадлежат C++. **Java** принадлежит мир Android: камера, ONNX Runtime, беги, интерфейс и мост ZMQ в Middleware.

Корень пакета: `app/src/main/java/adas/app/`, разложенный по тому, с чем класс разговаривает:

| пакет | содержимое |
|---|---|
| `adas.app` | `AdasAppHandler` (JNI), `AdasConfig`, `RuntimeParams`, `Logger`, `TimeUtil` |
| `adas.app.sensors` | `CameraHandler`, `IMUHandler`, `GPSHandler`, `PhoneStatsHandler` |
| `adas.app.vision` | `VisionPipeline`, оба раннера, `ModelCalibWarp`, `LaneLines`, `IntrinsicsCalibrator` |
| `adas.app.bridge` | `ZMQBridgeService`, `ProtoUtils` |
| `adas.app.record` | `BagLogger`, `AudioRecorder` |
| `adas.app.ui` | `MainActivity`, `ParamSlider`, две стендовые активности |

## Разделение ответственности

| Остаётся в **Java** | Живёт в **C++** |
|---|---|
| Camera2 / IMU / GPS | удержание полосы (`pp` / `fp`) |
| Supercombo (thneed или ORT) и опциональный YOLO | SafetyWarn, продольный план, `proto_convert` |
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
        → ModelCalibWarp.warpPair → 6×128×256 ×2   (OpenCL kernel, CPU fallback)
        → SupercomboThneedRunner.run (GPU)  |  SupercomboOnnxRunner.run (ORT)
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
JPEG в беге — это обычно **превью**, а не развёрнутый вход сети.
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
    "the runner passed its zero-input check at load (logcat: 'accepted')",
    "VisionPipeline actually finishing inference (infer_ms finite)",
    "publishToNative called (logcat)",
    "C++ ZmqBridge polling IN",
    "Planner producing control/lat_plan for Control",
]
for c in checks:
    print("-", c)
```

## Конфиг и живые параметры

1. **Поставляемый JSON:** `app/src/main/assets/config.json` (копируется в `filesDir` при первом запуске и
   **не** перезаписывается при обновлении — ловушка, о которой стоит знать: новые ключи, добавленные в ассет,
   до существующей установки не доходят. Сбрасывается кнопкой сброса параметров или
   `adb shell run-as adas.app rm -f files/config.json` — но сперва прочтите предупреждение ниже).

   ```{warning}
   В этом же файле лежит **выученная** калибровка камеры. Удаляя его, вы выбрасываете углы крепления,
   которые система набирала целый заезд, и она выучит их заново по тому, что увидит следующим, — однажды
   это оказался письменный стол. Именно поэтому сохранение теперь разрешено только при
   `VisionPipeline.seesRoad()`.
   ```
2. **`AdasConfig`** — какие узлы включены (`nodes.lane_keep`, `nodes.safety_warn`, …).
3. **`RuntimeParams` и слайдеры интерфейса** — ручки в духе PP и калибровки прямо на ходу.
4. **`AdasAppHandler.applyLaneKeepParams`** → JNI → `Middleware::setParameter`.

```python
# What a slider change means on the bus
ui_value = 0.8
# Java sends string "0.8" for name "pp_k_dd"
# ParamBag on the Planner's thread parses → double pp_k_dd_
assert float("0.8") == ui_value
```

Залить новый файл на телефон без пересборки APK: `./scripts/push_config.sh` (см. скрипты репозитория).

## Беги

`Logger` создаёт каталог сессии в хранилище приложения; каждый топик — поток protobuf.
Студенты забирают их через `./scripts/pull_bags.sh`, дальше:

```bash
cd scripts
python3 tools/latency.py /path/to/adas_logs/SESSION
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
| `VisionPipeline` | выбор раннера, политика отбрасывания, `seesRoad()` |
| `SupercomboThneedRunner` / `SupercomboOnnxRunner` | сеть, на GPU или через ORT |
| `ModelCalibWarp` | варп к геометрии модели, сверка GPU против CPU, метрика резкости |
| `IntrinsicsCalibrator` | интринсики по шахматной доске, когда вы её снимаете |
| `ZMQBridgeService` / `ProtoUtils` | в нативный код |
| `Logger` | беги |
| `AdasConfig` / `RuntimeParams` | ручки |
| `AdasAppHandler` | JNI, запуск нативной части |
| `LaneOverlayView` | наложение на экране |

## Задания

1. Проследите один кадр: какой класс его отбросит, если раннер занят?
2. Мысленно переключите `nodes.vision_supercombo` в `false` — какие топики исчезнут?
3. По logcat или по коду перечислите три сообщения, которые Java публикует на `:5555`.

<!-- next-chapter -->
---

**Дальше:** [От кадра до CAN](./Pipeline.md)
