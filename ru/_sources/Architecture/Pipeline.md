# От кадра до CAN

![Упрощённая схема конвейера](figures/pipeline_simple.png)

Инженерная версия с деталями реализации: `docs/IMAGE_TO_CAN_PIPELINE.md`.

## Проследите один кадр, с часами

Этапы ниже держать в голове проще, если смотреть, как через них движется один кадр. Каждое число — измеренная
медиана с ночного заезда длиной 29 минут. Медианы по этапам не обязаны складываться в накопленные медианы —
эти подобраны так, чтобы попасть в две накопленные величины, которые `tools/latency.py` реально печатает: 22 мс до
выхода модели и **52 мс до команды**, — чтобы арифметику можно было проверить, а не принять на слово.

```python
# Measured medians, run 2026_08_16_23_59_45, OnePlus 7T, thneed runner.
# "own" is the stage's own cost; the clock accumulates.
STAGES = (
    ("capture (Camera2 YUV, capture_ts stamped)", 0.0),
    ("delivery to VisionPipeline", 0.0),           # median 0, p95 0 — instrumented since 2026-08
    ("wait in the inference queue", 0.0),          # median 0, mean 1.0 — nothing is queueing up
    ("geometric warp to 6x128x256 (OpenCL)", 4.6), # prep_ms; was 12.1 on the CPU
    ("Supercombo inference (thneed, GPU)", 17.6),  # was 45.6 through ONNX Runtime
    ("parse heads -> vision/lanes", 0.0),
    ("Java -> ZMQ -> ZmqBridge -> proto_convert", 4.0),
    ("Planner -> control/lat_plan, control/lane_keep", 5.0),
    ("Control -> controls/steer", 21.0),
    ("Platform -> HCA_01 on the wire", 10.0),      # its own 10 ms TX timer
)

clock = 0.0
print(f"{'stage':>46} {'own':>6} {'clock':>7} {'car has moved':>15}")
for name, own in STAGES:
    clock += own
    print(f"{name:>46} {own:>5.1f} {clock:>6.1f} {22.0 * clock * 1e-3:>13.2f} m")
print(f"\nAt 22 m/s the command that reaches the rack was computed for a road position {22.0 * clock * 1e-3:.2f} m back.")
print("That is what fp_steer_delay_s compensates, and why it is a feedforward input rather than a nicety.")
```

Три строки в этом списке заслуживают внимания.

**Инференс занимает 17.6 мс, потому что модель считается на GPU.** Та же сеть через ONNX Runtime стоит на
этом телефоне 45.6 мс. Оба пути везут supercombo 0.9.7 — один и тот же файл, переведённый в другой формат, —
так что выбор пути это выбор скорости и ничего больше; см. [Supercombo](../Vision/Supercombo.md).

**Доставка и очередь обе равны нулю, и это самое интересное.** Это два разных измерения (`submit_ts_ms`,
`pickup_ts_ms`, `frames_dropped`), заведённые в августе 2026 именно потому, что кадр, пришедший на 40 мс
позже, и кадр, простоявший 40 мс в ожидании занятого потока инференса, раньше выглядели одинаково. На этом
заезде камера отдаёт вовремя, а инференс ни разу не оказывается тем, чего ждут: **ноль потерянных кадров из
52 690**, 100 % циклов не потеряли ничего. Когда это перестанет быть правдой, два числа покажут, какая
половина сломалась: много потерь при короткой доставке — инференс не успевает, мало потерь при долгой
доставке — камера приходит поздно.

**Переход через ZMQ стоит примерно половину интервала опроса.** Это одно из двух мест во всей цепочке, где
вообще есть опрос, второе — выход на CAN. Всё между ними работает по извещению, а не по опросу; измерение в
главе [Middleware](./Middleware.md).

## Что происходит между кадрами

Зрение идёт на 30.0 Гц — по кадру на период камеры, без пропусков. Актуатор — на 100 Гц, потому что HCA нужен
строгий такт, иначе EPS отбрасывает команду. Значит примерно два тика из трёх панда передаёт команду,
построенную на *эталоне, который уже использовался*.

Это правильное поведение, и оно создаёт ловушку для того, кто читает логи:

```python
VISION_HZ = 30.01
TX_HZ = 100.0
FRESH_WINDOW_MS = 50.0        # tools/latency.py keeps commands within this of their vision timestamp

vision_period_ms = 1000.0 / VISION_HZ
per_frame = TX_HZ / VISION_HZ
print(f"vision period {vision_period_ms:.1f} ms, {per_frame:.1f} actuator ticks per vision frame")
print(f"only the first tick uses a brand-new reference; the rest reuse it")
print()
# The freshness filter is a time window, not "one tick per frame" — so what it keeps is the fraction of
# the vision period that falls inside the window.
kept_predicted = min(1.0, FRESH_WINDOW_MS / vision_period_ms)
print(f"predicted share passing a {FRESH_WINDOW_MS:.0f} ms freshness filter: {100 * kept_predicted:.0f} %")
print(f"measured on the run: 171 097 kept of 175 573 = {100 * 171097 / 175573:.0f} %")
print()
print("Compute e2e latency over the raw stream instead and the republishes drag the number up, because")
print("their vision_ts is older than the command that carries it.")
```

На 13 Гц — темпе, который эта глава приводила до переезда модели на GPU, — этот фильтр не проходила треть
команд, и само окно свежести делало заметную работу. На 30 Гц период зрения короче окна, поэтому фильтр
отбрасывает только по-настоящему несвежие переиздания: 4 476 из 175 573. Урок улучшение пережил: «команда
свежая» и «картинка свежая» — разные утверждения, и для безопасности важно только второе. Ровно поэтому гейт
устаревания привязан к метке съёмки кадра, а не к возрасту команды, и поэтому таймаут команды HCA в 250 мс не
защитил от плана возраста 75 секунд.

Быстрый контур — не пустая работа: это угловой PID, замыкающийся на *измеренный* угол руля 100 раз в секунду,
и именно он делает движение рейки плавным.

## По этапам

### 1. Съёмка

`CameraHandler` (Camera2):

* буфер YUV, номинально **1280×720**;
* `capture_ts = TimeUtil.nowMs()` (BOOTTIME);
* `acquireLatestImage()` — отбрасывает несвежие кадры;
* объектив зафиксирован: автофокус выключен, `LENS_FOCUS_DISTANCE = 0` (бесконечность). Заезд, потерянный
  из-за расфокуса, разобран в главе [калибровки](../Calibration/Overview.md).

Кадр ветвится: превью в интерфейсе; очередь VisionPipeline; при включённом логировании — JPEG превью в бег
(**не** развёрнутый вход сети).

### 2. Геометрический warp

`ModelCalibWarp` приводит кадр ко входу модели — два кадра по шесть плоскостей 128×256, — используя
интринсики и калибровочные RPY. Ошибка тангажа или рыска сдвигает метрическую трактовку разметки и даёт
систематическую CTE.

Варп считается ядром OpenCL на GPU (4.6 мс против 12.1 на процессоре) и откатывается на процессор там, где
OpenCL недоступен. Первый кадр после запуска считается **обоими** способами и сверяется побитово: неверный
варп не падает — он отдаёт сети правдоподобную картинку не той дороги.

### 3. Инференс Supercombo

`VisionPipeline` в отдельном потоке, с двумя взаимозаменяемыми раннерами:

* `SupercomboThneedRunner` — supercombo 0.9.7 в fp16 на GPU, **17.6 мс**, путь по умолчанию;
* `SupercomboOnnxRunner` — та же сеть в fp32 через ONNX Runtime, 45.6 мс (медиана), запасной путь.

В обоих случаях: временной стек из двух кадров и рекуррентное состояние, выход разбирается в топик
**`vision/lanes`** (плюс одометрия и `model_long`), ставятся `infer_ts_ms` и `infer_duration_ms`.

**30.0 Гц** измерено при камере 30 к/с (dt 33.0 мс медианно) — темп теперь квантован периодом камеры и больше
ничем. До переезда модели на GPU было 13.24 Гц, и поперечный метроном ломался при тепловом троттлинге или
когда за SoC конкурировал YOLO. См. [задержки](../Latency/Overview.md).

Оба раннера проверяют себя при загрузке: сеть прогоняется на нулевых входах, и подпись выхода сверяется с
эталоном, снятым офлайн. Раннер, который считает не то, отказывается стартовать, а не отдаёт правдоподобные
числа про другую дорогу.

### 4. Мост Java → нативный код

Protobuf `ZMQMessage` публикуется PUB на `:5555`. `ZmqBridge` читает вход и публикует во внутренний
middleware; `utils/proto_convert.cpp` переводит сообщения провода во внутренние. Разделение: Java — камера,
модель, логи; C++ — планирование, управление, калибровка, панда.

### 5. План и команда

Поперечный контур разделён на три сервиса, и в разделении весь смысл: каждый проверяем без двух остальных.

* **`Planner`** (`services/planner.cpp`) берёт полилинию пути и шасси и выдаёт план **в кривизне**, никогда
  не в угле руля. Каким алгоритмом он построен — выбор конфига: `pp` (pure pursuit) или `fp` (MPC в
  области времени, как у штатных систем, по умолчанию на дороге), — и интерфейс
  одинаков, какой бы ни работал. Публикует `control/lat_plan`, `control/long_plan`, `control/lane_keep`.
* **`Control`** (`services/control.cpp`) — закон управления и ничего больше: кривизна и шасси превращаются в
  момент, признак включения, пиктограммы приборки и желание нажать кнопку круиза. Он не знает ни адреса CAN,
  ни сигнала, ни счётчика кадра. Публикует `controls/steer`.
* **`Platform`** (`services/platform.cpp`) выносит это намерение на шину и не называет ни одной марки: какая
  машина за шиной, решается один раз по `vehicle.name` и достигается через `platform::CarPlatform`. См.
  [подключение машины](../../PORTING.md).

### 6. Актуация

`Platform` передаёт `HCA_01` около 100 раз в секунду. Гейты: зажигание, `controls_allowed`, возраст команды
не более ~250 мс.

## Параллельные топики

| топик | роль |
|---|---|
| `calibration/camera` | живые RPY → warp |
| `camera/intrinsics` | фокус и главная точка, в единицах полного кадра |
| `phone/stats` | процессор, температуры |
| `middleware/stats` | отставание нативных таймеров |
| `vision/traffic_dets` | YOLO (для удержания полосы обычно выключен) |

## Темпы

| сигнал | измерено |
|---|---|
| камера | запрошено 30 к/с, период 33.0 мс подтверждён |
| `vision/lanes` | **30.01 Гц**, ноль потерянных кадров из 52 690 (было 13.24 Гц через ONNX) |
| `control/lane_keep` | 30.01 Гц, через 31 мс после съёмки |
| `controls/steer` | ~100 Гц, из них 97 % проходят фильтр свежести 50 мс (было 68 %) |
| передача панды | 100 Гц, свой таймер |

Темп зрения задаёт период поперечных решений; контур 100 Гц только замыкает угол. Деградацию темпа и сквозной
задержки надо фиксировать **до** сравнения контроллеров, потому что контроллер, оценённый на окне 3 Гц, — это
не тот контроллер, который вы поставили.

Иконки предупреждений (`safety/warn`) идут параллельно удержанию полосы — см. [FCW / AEB / LDW](../Safety/Warnings.md).

## Тот же конвейер на вашем ноутбуке

Сервисы выше работают и без телефона: `pyadas` открывает нативный `AdasApp` в симулированном режиме —
вы публикуете protobuf-сообщения, зовёте `step()`, читаете, что опубликовали сервисы:

```bash
./app/src/main/cpp/build_cpp.sh -t linux --test    # host build + ~230 gtest cases; pyadas core into scripts/
```

```python
# not-runnable — needs the host build; run from scripts/. core/lane_keep.py is the full version.
from pyadas import require_core
pyadas = require_core()

app = pyadas.AdasApp()                      # simulated mode: no threads, you own the clock
app.set_param("lane_keep_controller", "fp")

for ts, lanes_msg, raw in lanes:            # your session, from step 8
    app.publish("vision/lanes", raw.SerializeToString())
    app.publish("vehicle/state", chassis_at(ts))    # your synthetic chassis, or a real one
    app.step()
    for topic, payload in app.pop_messages():
        if topic == "control/lane_keep":
            record(ts, payload)             # the shipped stack's steering, frame by frame
```

`scripts/core/lane_keep.py` оборачивает ровно этот цикл, им уже пользуются симулятор и визуализатор —
прочтите его раз, и офлайновый инструментарий перестанет быть магией. Это же честный способ сравнить
собственный регулятор с боевыми `pp` и `fp` на одинаковых кадрах: один бег на входе, три команды на
выходе, один график.

<!-- next-chapter -->
---

**Дальше:** [Зрение — обзор](../Vision/Overview.md)
