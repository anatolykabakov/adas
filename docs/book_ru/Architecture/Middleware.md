# Middleware (шина на C++)

После зрения и до HCA почти весь нативный код общается через один объект —
**`adas::middleware::Manager`**. Разберётесь в нём — и сможете читать любой сервис (`Planner`, `Control`,
`Platform`, `SafetyWarn`, …), не утопая в Android.

Источник истины: `app/src/main/cpp/include/adas/middleware/manager.hpp`.
Тесты, которые читаются как учебник: `app/src/main/cpp/tests/test_middleware.cpp`.

## Сначала постройте свою: шина в шестьдесят строк

Легче всего читается та шина, маленькую версию которой вы уже написали сами. Важных свойств три, и все они
умещаются на один экран: **топики** отвязывают говорящего от слушающих; **доставка
детерминирована** — опубликованное во время тика приходит на следующем, поэтому прогон
воспроизводим; и **очередь наблюдаема**, ведь невидимая очередь — как раз то место, где системы
умирают:

```python
class Bus:
    """Topics, deferred delivery, a visible queue. The real one adds threads and timers."""
    def __init__(self):
        self.subs = {}
        self.queue = []
        self.max_backlog = 0

    def subscribe(self, topic, fn):
        self.subs.setdefault(topic, []).append(fn)

    def publish(self, topic, msg):
        self.queue.append((topic, msg))

    def step(self):
        """Deliver everything queued so far; what handlers publish waits for the next step."""
        batch, self.queue = self.queue, []
        self.max_backlog = max(self.max_backlog, len(batch))
        for topic, msg in batch:
            for fn in self.subs.get(topic, []):
                fn(msg)
        return len(batch)
```

Сервис владеет состоянием, подписывается, публикует — и никто никого не вызывает напрямую. Теперь переселим на
шину игрушечный планировщик (слияние линий по обратной дисперсии) и игрушечный контроллер (pure pursuit в
связанной системе координат):

```python
import math
import numpy as np

L, HALF = 2.636, 1.75

class PlannerService:
    """lanes -> path: the step-3 fusion, one message at a time."""
    def __init__(self, bus):
        self.bus = bus
        bus.subscribe("vision/lanes", self.on_lanes)

    def on_lanes(self, m):
        w_l, w_r = 1.0 / m["sig_l"] ** 2, 1.0 / m["sig_r"] ** 2
        centre = (w_l * (m["y_l"] - HALF) + w_r * (m["y_r"] + HALF)) / (w_l + w_r)
        self.bus.publish("vision/path", {"x": m["x"], "y": centre})

class ControlService:
    """path + chassis -> steer: the step-2 pursuit, ego frame."""
    def __init__(self, bus, ld=8.0):
        self.bus, self.ld = bus, ld
        self.path = None
        bus.subscribe("vision/path", lambda m: setattr(self, "path", m))
        bus.subscribe("vehicle/state", self.on_chassis)

    def on_chassis(self, m):
        if self.path is None:
            return
        x, y = self.path["x"], self.path["y"]
        i = int(np.argmax(np.hypot(x, y) >= self.ld))
        alpha = math.atan2(y[i], x[i])
        delta = math.atan2(2.0 * L * math.sin(alpha), self.ld)
        self.bus.publish("controls/steer", {"delta": delta})
```

Критерий приёмки для этого архитектурного шага — *равенство*: сервисы на шине обязаны воспроизвести прямые
вызовы бит в бит:

```python
rng = np.random.default_rng(0)
x_grid = np.arange(0.0, 60.0, 2.0)
sig = 0.05 + 0.01 * x_grid

def one_frame():
    c = 0.5 * 0.004 * x_grid**2
    return {"x": x_grid, "y_l": c + HALF + rng.normal(0, sig), "sig_l": sig,
            "y_r": c - HALF + rng.normal(0, sig), "sig_r": sig}

bus = Bus()
PlannerService(bus)
ControlService(bus)
got = []
bus.subscribe("controls/steer", lambda m: got.append(m["delta"]))

frames = [one_frame() for _ in range(50)]
for f in frames:
    bus.publish("vision/lanes", f)
    bus.step()                            # deliver lanes -> planner publishes path
    bus.publish("vehicle/state", {"v": 10.0})
    bus.step()                            # deliver path and the chassis tick
    bus.step()                            # deliver steer
# Publish the chassis before the path has been delivered and every steer command quietly
# computes on the PREVIOUS frame's path — off by one frame, no error raised. On the phone the
# two arrive on independent threads, which is why every plan carries capture_ts and Control
# gates on its age instead of trusting arrival order.

# the same math, called directly
def direct(f, ld=8.0):
    w_l, w_r = 1.0 / f["sig_l"] ** 2, 1.0 / f["sig_r"] ** 2
    centre = (w_l * (f["y_l"] - HALF) + w_r * (f["y_r"] + HALF)) / (w_l + w_r)
    i = int(np.argmax(np.hypot(f["x"], centre) >= ld))
    alpha = math.atan2(centre[i], f["x"][i])
    return math.atan2(2.0 * L * math.sin(alpha), ld)

ref = [direct(f) for f in frames]
worst = max(abs(a - b) for a, b in zip(got, ref))
print(f"{len(got)} commands, worst |bus - direct| = {worst:.2e} rad, "
      f"max backlog {bus.max_backlog}")
assert len(got) == len(frames) and worst == 0.0, "acceptance: the bus must change nothing"
```

А теперь посмотрите, как это ломается на самом деле. Мост ZMQ на телефоне когда-то вычерпывал **по одному
сообщению за 10-мс тик** — потолок в 100 сообщений/с. Третий топик на 28 Гц вытолкнул поток за этот потолок,
очередь росла, план старел, и гейт свежести молча снимал поперечное управление через минуту после начала
каждого заезда. Воспроизведём:

```python
bus2 = Bus()
delivered = {"n": 0}
bus2.subscribe("t", lambda m: delivered.__setitem__("n", delivered["n"] + 1))

backlog = []
for tick in range(200):
    for _ in range(3):                      # 3 messages arrive per tick...
        bus2.publish("t", tick)
    batch, bus2.queue = bus2.queue[:1], bus2.queue[1:]   # ...but we deliver only 1
    for topic, msg in batch:
        for fn in bus2.subs[topic]:
            fn(msg)
    backlog.append(len(bus2.queue))

print(f"inflow 3/tick, drain 1/tick: backlog after 200 ticks = {backlog[-1]}")
print(f"age of the message being delivered now ≈ {backlog[-1] // 3} ticks")
assert backlog[-1] == 400, "a fixed drain below inflow does not lag — it diverges"
```

Очередь не выходит на какое-то постоянное отставание — она **расходится линейно**, а вместе с ней растёт и
возраст того, на что вы реагируете. Лекарство (вычерпывать досуха, с разумной границей) здесь укладывается в
одну строку, а на телефоне — в одну константу, `kMaxPerTick = 32`. Всё, что ниже, — это та же игрушка, только
повзрослевшая: потоки, таймеры, параметры, а `middleware/stats` — ваш `max_backlog`, вышедший в люди.

## Мысленная модель

```text
┌──────────── Service A ────────────┐     ┌──────────── Service B ────────────┐
│ worker thread                     │     │ worker thread                     │
│  • subscribe callbacks            │     │  • subscribe callbacks            │
│  • scheduleTimer ticks            │pub──▶│  • scheduleTimer ticks            │
│  • ParamBag (own knobs)         │     │  • ParamBag                      │
└───────────────────────────────────┘     └───────────────────────────────────┘
                 ▲
                 │ Middleware::setParameter / publish
```

* **Service** — единица параллельности: у него либо свой поток (`RealTime`), либо общий `step()` (`Simulated`).
* **Топики** — это типизированные строковые имена (`vision/path`, `vehicle/chassis`, …).
* **Публикация** копирует сообщение в очередь каждого подписчика; колбэк выполняет **владелец** этой очереди.
* **Параметры** можно задавать из любого потока (интерфейс, JNI); а **применяются** они на потоке сервиса, между колбэками.

```{admonition} Почему не вызывать сервисы напрямую?
:class: tip
Java, реплей бега и MetaDrive нужны одни и те же алгоритмы. Шина вместе с `Mode::Simulated` позволяют Python и `pyadas` гонять **тот же самый** C++ без Camera2.
```

## Два режима

| режим | кто крутит ручку | когда студент с ним встретится |
|---|---|---|
| `RealTime` | поток каждого сервиса и системное время | телефон, APK |
| `Simulated` | `setTime(t_us)` и `step()` | юнит-тесты, `pyadas`, офлайн по бегу |

```python
# Conceptual host loop (pyadas / tests do this for you)
# mw = Middleware(Mode.Simulated)
# mw.register_services(...)
# for t in bag_timestamps_us:
#     mw.set_time(t)
#     mw.publish("vehicle/chassis", chassis_sample)
#     mw.publish("vision/path", path_msg)
#     mw.step()
#     msgs = mw.pop_messages()  # outbound for logging / asserts
```

## Как написать крошечный сервис

Настоящие имена API (упрощённое эхо):

```cpp
#include "adas/middleware/manager.hpp"
#include "adas/utils/adas_topics.h"

class EchoService : public adas::middleware::Service {
 public:
  std::string_view getName() const override { return "echo"; }

  void configure() override {
    registerParameter<double>("gain", gain_);

    subscribe<adas::ChassisSample>(
        adas::topics::kVehicleChassis,
        [this](const adas::ChassisSample& m) {
          last_v_ = m.speed_mps;  // ChassisSample / VehicleState use speed_mps
        });

    scheduleTimer(50, [this] { tick(); }, "tick");  // 50 ms
  }

 private:
  void tick() {
    // publish something other services / ZMQ OUT can see
    (void)gain_;
    (void)last_v_;
  }

  double gain_ = 1.0;
  float last_v_ = 0.f;
};
```

Зарегистрировать:

```cpp
adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
mw.registerService(std::make_shared<EchoService>());
mw.setParameter("gain", "2.5");  // string in, parsed to double
mw.setTime(/*us=*/0);
mw.step();  // flush params + drain queues + fire due timers
```

### Что можно вызывать из `configure()`

| API | смысл |
|---|---|
| `subscribe<T>(topic, cb)` | типизированный входящий ящик |
| `publish(topic, msg)` | типизированный исходящий (тип должен совпадать с подписчиками) |
| `scheduleTimer(ms, cb, name)` | периодический вызов на этом сервисе |
| `registerParameter<T>(name, ref)` | живая ручка |
| `now()` | время middleware, мкс |

## Топики, которые встречаются чаще всего

Объявлены в `utils/adas_topics.h`:

| топик | типичный производитель → потребитель |
|---|---|
| `vision/lanes` | Java (ZMQ) → `ZmqBridge` → `proto_convert` |
| `vision/path` | `Planner` → `SafetyWarn`, бег |
| `vision/model_long` | Java → `SafetyWarn`, `LongPlan` |
| `vehicle/chassis` | декодер панды или Java → многие |
| `control/lat_plan`, `control/long_plan` | `Planner` → `Control` |
| `control/lane_keep` | `Planner` → бег, интерфейс |
| `controls/steer` | `Control` → `Platform`, бег |
| `panda/health`, `can/rx` | `Platform` → надзор, бег |
| `safety/warn` | SafetyWarn → ZMQ OUT → интерфейс, бег |
| `middleware/stats` | сервис статистики → бег |

## ParamBag (живые ручки)

Задача: поток интерфейса не должен состязаться с потоком управления за общие данные.

Решение:

1. `setParameter("pp_k_dd", "0.8")` **кладёт строку в очередь** на владеющем сервисе.
2. Перед следующим колбэком или таймером `ParamBag::flush()` разбирает её и записывает в переменную C++.

```python
# Student intuition (not the real class, but the timing)
pending = {"pp_k_dd": "0.8"}

def flush(params, pending):
    for k, text in pending.items():
        params[k] = float(text)  # or bool/int/...
    pending.clear()

params = {"pp_k_dd": 0.4}
flush(params, pending)
assert params["pp_k_dd"] == 0.8
```

Поддерживаемые типы: `bool`, `int`, `long long`, `float`, `double`, `string`.
Android NDK потребовал, чтобы вспомогательные функции разбора были свободными (см. `detail::parseParamText` в заголовке): статический шаблонный метод класса не слинковался.

## Шина ли виновата? Измерьте, а не предполагайте

Когда конвейер тормозит, шина сообщений — первый подозреваемый, и подозревать её ничего не стоит. Но мы её
измерили, и от ответа зависит, как читать любое другое число задержки в книге.

**Публикация будит подписчика немедленно.** `publish` копирует сообщение в очередь каждого подписчика и
вызывает `cv.notify_one()`, поэтому владеющий поток сразу готов к исполнению — он не ждёт своего
следующего тика таймера. Опрос остаётся ровно в двух местах, на границах с внешним миром: вход ZMQ от Java и
выход CAN к панде, оба по таймеру 10 мс, — потому что то, что с другой стороны, разбудить нас не умеет.

Разницу между этими двумя схемами стоит увидеть своими глазами, а не принимать на слово:

```python
import math

# Two ways to hand a message from producer to consumer. The producer emits vision frames at the measured
# 68.3 ms interval; the consumer either polls on its own 10 ms timer or is woken on publish.
#
# 68.3 and not 90: a period that is an exact multiple of the poll interval always lands on a tick and the
# wait comes out zero, which flatters polling for a reason that has nothing to do with the design.
PRODUCE_MS = 68.3
POLL_MS = 10.0
N = 500

polled = []
for i in range(N):
    t_pub = i * PRODUCE_MS
    next_tick = POLL_MS * math.ceil(t_pub / POLL_MS)
    polled.append(next_tick - t_pub)

print(f"polled  : mean {sum(polled) / N:5.2f} ms, worst {max(polled):5.2f} ms")
print(f"notified: mean {0.0:5.2f} ms, worst {0.0:5.2f} ms")
print()
print("Half a poll interval on average and a full one at worst, per hop. With four hops between camera")
print("and CAN that is around 20 ms of pure waiting — well over a third of the whole 52 ms budget, spent")
print("on nothing. Now try PRODUCE_MS = 90.0 and watch the cost vanish, which is how a benchmark lies.")
```

**Таймеры срабатывают тогда, когда обещали.** За 143 700 срабатываний на настоящем заезде средний интервал
составил **10.00 мс** против номинальных 10. Худшие случаи — 44.8 мс в сервисе панды (скорее всего затык USB,
то есть внешний мир, а не планировщик) и 21.8 мс в мосте ZMQ.

Итог измерений: **middleware никогда не задерживает сообщение на таймере**, и та часть задержки конвейера,
которая не инференс, приходится не на шину. Из примерно 8 мс между выходом модели и опубликованным планом
около 5 — это опрос входа ZMQ; вот настоящая цена, и единственная, которую стоит называть.

При этом доля опроса выросла безо всякого замедления шины. Когда инференс занимал 45.6 мс, опрос составлял 6 %
бюджета и упоминания не стоил; при 17.6 мс это уже 10 % куда меньшего бюджета. Постоянные издержки выходят на
первый план, когда убрали переменную часть, — это нормальный конец оптимизации, а не новая беда.

```{admonition} Что это даёт диагностически
:class: tip
Если бег показывает устаревший эталон, шину можно сразу исключить. Это ценнее, чем кажется: область поиска
сужается до тракта камера → модель или до внешних границ, а `manager.hpp` можно не открывать.
```

## Очереди и потери

Ёмкость очереди подписчика по умолчанию конечна (около 100). Если медленный сервис её не вычитывает:

* при переполнении отбрасываются **старые** сообщения (`pop_front`), в очереди остаются последние ~100;
* `middleware/stats` показывает отстающие таймеры и потери, и он есть в каждом беге.

Это сделано намеренно: устаревшее измерение контуру ни к чему, а издателя нельзя тормозить из-за медленного
подписчика. Потери видны в статистике и не прячутся в растущей задержке.

```{warning}
«Плохой контроллер» на беге, где `middleware/stats` показывает огромное отставание таймеров, — это чаще
история про **планирование**, а не про коэффициенты. Проверьте это, прежде чем трогать веса: хватит одного графика.
```

У конвейера зрения политика **та же по сути** — и он, и шина отбрасывают старое, чтобы оставить свежее, — но
разная по **глубине**. Шина держит до сотни сообщений (логгеру и разбору бега нужна серия); а `VisionPipeline`
держит ровно **один** слот и перезаписывает ожидающий кадр. Контроллеру запас несвежих кадров ни к чему,
поэтому зрение действует резче; шина мягче, ведь один и тот же топик могут слушать и контур, и запись.

## Как это изучать в репозитории

1. Пробежать `tests/test_middleware.cpp` (`PubService`, `SubService`, `TimerService`).
2. Открыть `src/services/planner.cpp` → `configure()`: параметры и подписки.
3. Открыть `src/services/zmq_bridge.cpp`: сообщения из Java входят в ту же шину.
4. Прогнать тесты на хосте: `./scripts/docker.sh tests` (или `build_cpp.sh -t linux --test`).

## Задания

1. Нарисуйте путь одного `ChassisSample` от `Platform` (или бега) до `Control`.
2. Добавьте гипотетический параметр `echo.gain` — какой поток его пишет, какой читает?
3. Почему в режиме `Simulated` нужно вызвать `setTime` до `step`, чтобы таймеры сработали?

<!-- next-chapter -->
---

**Дальше:** [Слой Java](./JavaLayer.md)
