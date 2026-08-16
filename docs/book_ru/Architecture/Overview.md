# Обзор системы

Мы хотим, чтобы телефон на лобовом стекле удерживал Golf 7 около центра полосы, командуя MQB **HCA** через USB-**Panda**.
Эта глава — карта репозитория, прежде чем мы полезем в математику зрения и управления.

## Назначение

Приложение ADAS:

1. снимает дорогу камерой (плюс вспомогательные IMU и GPS);
2. запускает инференс **Supercombo** (ONNX Runtime на устройстве);
3. считает команду поперечного удержания полосы в нативном **C++**;
4. публикует `HCA_01` на CAN VW MQB через Panda;
5. пишет полный **бег** для офлайн-анализа.

Это исследовательский и учебный стек удержания полосы — не готовый продукт с ACC и «автопилотом».

```{figure} figures/pipeline_simple.png
---
width: 95%
---
Сквозной путь, который вы научитесь трассировать: датчики → модель → путь → управление → CAN.
```

## Слои

| слой | компоненты | роль |
|---|---|---|
| **Java** | камера / IMU / GPS, VisionPipeline, логгер, ZMQ | датчики, ORT, интерфейс, запись бегов |
| **C++** | `AdasApp`, LaneKeep, Calib, Panda, … | алгоритмы и актуация |
| **Python** | vis, latency, перебор параметров, MetaDrive, `pyadas` | анализ и симуляция на хосте |

**Правило устройства:** поперечные алгоритмы живут в C++. Python либо анализирует бег, либо гоняет **тот же** нативный код через `pyadas` (`publish → step → pop_messages`). Это и делает лабораторные прогоны честными относительно телефона.

Дальше в этой части: [Middleware](./Middleware.md) (нативная шина) → [Слой Java](./JavaLayer.md) (камера / ORT / ZMQ) → [Конвейер](./Pipeline.md) (кадр → HCA). Ещё [FCW / AEB / LDW](../Safety/Warnings.md) — предупреждения без актуации.

## Как это собиралось, в том порядке, в котором собиралось

Таблица слоёв выше — то, как система выглядит сейчас. Это не тот порядок, в котором её кто-либо стал бы
строить, и именно попытка следовать таблице как плану заводит в тупик. Работает другой порядок — и книга
идёт по нему: замкнуть контур как можно раньше, а потом улучшать по одному звену.

| этап | что можно в конце этапа | чего пока нельзя |
|---|---|---|
| 1. реплей бега | гонять настоящий C++ удержания полосы по записанным кадрам | верить числам: запись сделана другим контроллером |
| 2. датчики на телефоне | видеть живую разметку, живой путь, живые задержки | рулить |
| 3. нативная шина | сервисы разговаривают, не зная друг о друге | понимать, почему что-то пришло поздно |
| 4. Panda, только приём | видеть скорость, угол руля и разрешён ли HCA вообще | что-либо отправлять |
| 5. Panda, передача | действительно рулить, с моделью безопасности панды на пути | понимать, *хорошо* ли отрулило |
| 6. измерения | ответить на этот вопрос бегом и скриптом | остановиться: теперь начинается настоящая работа |

Шестой этап — там, где курс обычно заканчивается, и там, где этот проект проводит большую часть времени.
Две иллюстрации почему, обе из настоящих заездов:

* машина не рулила весь заезд, и приложение не залогировало ничего плохого. Панда сообщала пакет здоровья
  в 57 байт там, где код ждал 58, поэтому каждое поле после третьего читалось не с того смещения — включая
  `controls_allowed`;
* первый заезд с включённой продольной актуацией отработал ровно как спроектирован и всё равно был
  регрессией: план весь заезд требовал уставку на 4.8 м/с ниже текущей скорости, и шина собрала 715 нажатий
  кнопок за 28 минут.

Ни то, ни другое не задача теории управления. Оба нашлись сравнением двух независимых измерений — этому и
учит глава [Беги](../Logging/Bags.md).

```python
# The contract that makes stage 1 possible, and why it is worth building first: the controller cannot tell
# where its path came from.
def lane_keep_step(path_xy, speed_ms):
    """Stand-in for the real service: it sees a polyline and a speed. Nothing else."""
    x = [p[0] for p in path_xy]
    y = [p[1] for p in path_xy]
    # Curvature of a quadratic fit at the vehicle, the same quantity the real feedforward uses.
    n = len(x)
    sx = sum(x); sxx = sum(v * v for v in x); sxxx = sum(v ** 3 for v in x)
    sxxxx = sum(v ** 4 for v in x)
    sy = sum(y); sxy = sum(a * b for a, b in zip(x, y)); sxxy = sum(a * a * b for a, b in zip(x, y))
    # Solve the 3x3 normal equations for y = a x^2 + b x + c
    import numpy as np
    A = np.array([[sxxxx, sxxx, sxx], [sxxx, sxx, sx], [sxx, sx, n]], dtype=float)
    a, b, c = np.linalg.solve(A, np.array([sxxy, sxy, sy], dtype=float))
    return {"cte_m": float(c), "epsi_rad": float(b), "kappa": float(2 * a), "speed": speed_ms}

live = [(i * 1.5, 0.5 * 0.004 * (i * 1.5) ** 2 + 0.20) for i in range(20)]     # from the camera
replay = list(live)                                                             # from a bag
simulated = list(live)                                                          # from MetaDrive

for name, path in (("live", live), ("bag replay", replay), ("simulator", simulated)):
    out = lane_keep_step(path, 15.0)
    print(f"{name:>11}: cte {out['cte_m']:+.3f} m, kappa {out['kappa']:.4f} 1/m")
print("\nIdentical, by construction. That is what lets a lab sweep mean something.")
```

## Конфигурация, которую вы будете трогать

`app/src/main/assets/config.json` (можно переопределить в `filesDir` приложения).

Ручки машины (`vehicle.*`):

```json
"lane_keep_controller": "fp",
"lat_use_vehicle_model": true,
"tire_stiffness_factor": 0.64,
"fp_steer_delay_s": 0.35
```

Флаги узлов (`nodes.*`), например `"vision_traffic": false`, `"phone_stats": true`,
`"safety_warn": true`.

| ключ | назначение |
|---|---|
| `vehicle.lane_keep_controller` | `pp` \| `mpc` \| `fp` |
| `vehicle.lat_use_vehicle_model` | $\kappa\to$ SWA через модель недокрута |
| `vehicle.fp_steer_delay_s` | упреждение состояния на задержку конвейера |
| `nodes.vision_traffic` | YOLO; держите `false`, когда измеряете удержание полосы |
| `nodes.phone_stats` | процессор и температуры раз в секунду в бег |

## Типичный ход работы студента

1. Записать или скачать сессию в `adas_logs/...`.
2. Прогнать `tools/latency.py`, визуализатор бега, PlotJuggler — установить темп и сквозную задержку.
3. Перебрать параметры (`bag/bag_config_sweep.py`) при **фиксированной** предполагаемой задержке зрения.
4. И только потом открывать исходники `LaneKeepService` / `PurePursuit` с главами по управлению в руках.

```{tip}
Если CTE выглядит ужасно, задайте три вопроса по порядку: темп зрения ≥ ~9 Гц? Знаки $y$ и `steer_sign` согласованы? Калибровка разумна? Коэффициенты идут четвёртыми.
```

<!-- next-chapter -->
---

**Дальше:** [Middleware](./Middleware.md)
