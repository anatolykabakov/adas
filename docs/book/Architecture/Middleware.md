# Middleware (C++ bus)

After vision and before HCA, almost everything in native code talks through one object: **`adas::Middleware`**.
If you understand Middleware, you can read any service (`LaneKeep`, `SafetyWarn`, `Panda`, …) without drowning in Android.

Source of truth: `app/src/main/cpp/include/middleware/middleware.hpp`.
Tests that read like a tutorial: `app/src/main/cpp/tests/test_middleware.cpp`.

## Mental model

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

* A **Service** is one concurrent unit: its own thread (Realtime) or shared `step()` (Simulated).
* **Topics** are typed string names (`vision/path`, `vehicle/chassis`, …).
* **Publish** copies the message into each subscriber queue; the **owner** of that queue runs the callback.
* **Parameters** can be set from any thread (UI / JNI); they are **applied** on the service thread between callbacks.

```{admonition} Why not call services directly?
:class: tip
Java, bag replay, and MetaDrive all need the same algorithms. A bus + `Mode::Simulated` lets Python/`pyadas` drive the **same** C++ without Camera2.
```

## Two modes

| mode | who turns the crank | when students meet it |
|---|---|---|
| `Realtime` | each service thread + OS time | phone / APK |
| `Simulated` | `setTime(t_us)` + `step()` | unit tests, `pyadas`, bag offline |

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

## Writing a tiny service

Real API names (simplified Echo):

```cpp
#include "middleware/middleware.hpp"
#include "utils/adas_topics.h"

class EchoService : public adas::Service {
 public:
  std::string_view getName() const override { return "echo"; }

  void configure() override {
    registerParameter<double>("gain", gain_);

    subscribe<adas::ChassisSample>(
        adas::topics::kVehicleChassis,
        [this](const adas::ChassisSample& m) {
          last_v_ = m.speed_ms;  // field names: see ChassisSample
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

Register it:

```cpp
adas::Middleware mw(adas::Middleware::Mode::Simulated);
mw.registerService(std::make_shared<EchoService>());
mw.setParameter("gain", "2.5");  // string in, parsed to double
mw.setTime(/*us=*/0);
mw.step();  // flush params + drain queues + fire due timers
```

### What you are allowed to call from `configure()`

| API | meaning |
|---|---|
| `subscribe<T>(topic, cb)` | typed inbox |
| `publish(topic, msg)` | typed outbox (type must match subscribers) |
| `scheduleTimer(ms, cb, name)` | periodic on this service |
| `registerParameter<T>(name, ref)` | live knob |
| `now()` | middleware time [µs] |

## Topics students see often

Defined in `utils/adas_topics.h`:

| topic | typical producer → consumer |
|---|---|
| `vision/lanes` | Java (ZMQ) → `TopicConvert` |
| `vision/path` | `TopicConvert` → `LaneKeep`, `SafetyWarn` |
| `vision/model_long` | Java → `SafetyWarn`, `LongPlan` |
| `vehicle/chassis` | Panda decode / Java → many |
| `control/lane_keep`, `controls/steer` | LaneKeep → Panda / bag |
| `safety/warn` | SafetyWarn → ZMQ OUT → UI / bag |
| `middleware/stats` | stats service → bag |

## ParamBag (live knobs)

Problem: UI thread must not race the control thread.

Solution:

1. `setParameter("pp_k_dd", "0.8")` **queues** a string on the owning service.
2. Before the next callback / timer, `ParamBag::flush()` parses and writes the C++ variable.

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

Supported types: `bool`, `int`, `long long`, `float`, `double`, `string`.
Android NDK needs the parse helpers as free functions (see `detail::parseParamText` in the header) — a class-static template failed to link.

## Queues and drops

Default subscriber capacity is finite (~100). If a slow service does not drain:

* new messages can be **dropped**;
* `middleware/stats` shows lagging timers / drops.

```{warning}
A “bad controller” on a bag where `middleware/stats` shows huge timer lag is often a **scheduling** story, not a gain story.
```

## How to study this in the repo

1. Skim `test_middleware.cpp` (`KnobService`).
2. Open `lane_keep_service.cpp` → `configure()`: parameters + subscriptions.
3. Open `zmq_bridge_service.cpp`: Java messages enter the same bus.
4. Run host tests: `./scripts/docker.sh tests` (or `build_cpp.sh -t linux --test`).

## Exercise

1. Draw the path of one `ChassisSample` from Panda (or bag) to `LaneKeepService`.
2. Add a hypothetical parameter `echo.gain` — which thread writes it, which thread reads it?
3. In Simulated mode, why must you call `setTime` before `step` for timers to fire?

<!-- next-chapter -->
---

**Next:** [Java layer](./JavaLayer.md)
