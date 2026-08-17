# Middleware (шина на C++)

После зрения и до HCA почти всё в нативном коде разговаривает через один объект:
**`adas::middleware::Manager`**. Если вы понимаете его, то сможете читать любой сервис (`Planner`, `Control`,
`Platform`, `SafetyWarn`, …), не утопая в Android.

Источник истины: `app/src/main/cpp/include/adas/middleware/manager.hpp`.
Тесты, которые читаются как учебник: `app/src/main/cpp/tests/test_middleware.cpp`.

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

* **Service** — одна единица параллельности: свой поток (`RealTime`) или общий `step()` (`Simulated`).
* **Топики** — типизированные строковые имена (`vision/path`, `vehicle/chassis`, …).
* **Публикация** копирует сообщение в очередь каждого подписчика; колбэк выполняет **владелец** этой очереди.
* **Параметры** можно задавать из любого потока (интерфейс, JNI); **применяются** они на потоке сервиса между колбэками.

```{admonition} Почему не вызывать сервисы напрямую?
:class: tip
Java, реплей бега и MetaDrive нуждаются в одних и тех же алгоритмах. Шина плюс `Mode::Simulated` позволяют Python и `pyadas` гонять **тот же** C++ без Camera2.
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

Задача: поток интерфейса не должен гоняться за поток управления.

Решение:

1. `setParameter("pp_k_dd", "0.8")` **кладёт в очередь** строку на владеющем сервисе.
2. Перед следующим колбэком или таймером `ParamBag::flush()` разбирает её и пишет в переменную C++.

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
Android NDK требует, чтобы разбирающие вспомогательные функции были свободными (см. `detail::parseParamText` в заголовке) — статический шаблон класса не слинковался.

## Шина ли виновата? Измерьте, а не предполагайте

Шина сообщений — естественный подозреваемый, когда конвейер медленный, и подозревать дёшево. Эту измерили, и
ответ определяет, как читать любое другое число задержки в книге.

**Публикация будит подписчика немедленно.** `publish` копирует в очередь каждого подписчика и вызывает
`cv.notify_one()`, поэтому владеющий поток становится готовым к исполнению сразу — он не ждёт своего
следующего тика таймера. Опрос есть ровно в двух местах, на границах с внешним миром: вход ZMQ от Java и
выход CAN к панде, оба по таймеру 10 мс, потому что то, что с другой стороны, разбудить нас не умеет.

Разницу между этими двумя схемами стоит увидеть, а не принять на слово:

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

**Таймеры срабатывают тогда, когда обещают.** За 143 700 срабатываний на настоящем заезде средний интервал
был **10.00 мс** против номинальных 10. Худшие случаи — 44.8 мс в сервисе панды (скорее всего затык USB, то
есть внешний мир, а не планировщик) и 21.8 мс в мосте ZMQ.

Итого измеренный вывод: **middleware никогда не задерживает сообщение на таймере**, и та задержка конвейера,
которая не инференс, — это не шина. Из примерно 8 мс между выходом модели и опубликованным планом около 5 —
опрос входа ZMQ; это настоящая цена и единственная, которую стоит называть.

Доля эта выросла безо всякого замедления шины. Когда инференс занимал 45.6 мс, опрос был 6 % бюджета и не
стоил упоминания; при 17.6 мс он уже 10 % куда меньшего бюджета. Постоянные издержки становятся сюжетом,
когда переменную убрали, — это нормальный конец оптимизации, а не новая беда.

```{admonition} Что это даёт диагностически
:class: tip
Когда бег показывает устаревший эталон, шина уже исключена. Это дороже, чем звучит: область поиска сужается
до тракта камера → модель или до внешних границ, а `manager.hpp` можно не читать.
```

## Очереди и потери

Ёмкость очереди подписчика по умолчанию конечна (около 100). Если медленный сервис не вычитывает:

* при переполнении отбрасываются **старые** сообщения (`pop_front`), в очереди остаются последние ~100;
* `middleware/stats` показывает отстающие таймеры и потери, и он есть в каждом беге.

Так сделано намеренно: устаревшее измерение контуру не нужно, а издателя нельзя стопорить медленным
подписчиком. Потери видны в статистике, а не прячутся в растущей задержке.

```{warning}
«Плохой контроллер» на беге, где `middleware/stats` показывает огромное отставание таймеров, — это чаще
история про **планирование**, а не про коэффициенты. Проверьте это прежде, чем трогать вес: нужен один график.
```

С конвейером зрения политика **та же по направлению** — оба отбрасывают старое, чтобы оставить свежее, —
но разная по **глубине**. Шина держит до сотни сообщений (логгеру и разбору бега нужна серия); `VisionPipeline`
держит ровно **один** слот и перезаписывает ожидающий кадр. Контроллеру запас несвежих кадров бесполезен,
поэтому зрение агрессивнее; шина мягче, потому что один и тот же топик могут слушать и контур, и запись.

## Как это изучать в репозитории

1. Пробежать `tests/test_middleware.cpp` (`PubService`, `SubService`, `TimerService`).
2. Открыть `src/services/planner.cpp` → `configure()`: параметры и подписки.
3. Открыть `src/services/zmq_bridge.cpp`: сообщения из Java входят в ту же шину.
4. Прогнать тесты на хосте: `./scripts/docker.sh tests` (или `build_cpp.sh -t linux --test`).

## Задания

1. Нарисуйте путь одного `ChassisSample` от `Platform` (или бега) до `Control`.
2. Добавьте гипотетический параметр `echo.gain` — какой поток его пишет, какой читает?
3. В режиме `Simulated` почему нужно вызвать `setTime` до `step`, чтобы таймеры сработали?

<!-- next-chapter -->
---

**Дальше:** [Слой Java](./JavaLayer.md)
