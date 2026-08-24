# Middleware (C++ bus)

After vision and before HCA, almost everything in native code talks through one object:
**`adas::middleware::Manager`**. If you understand it, you can read any service (`Planner`, `Control`,
`Platform`, `SafetyWarn`, …) without drowning in Android.

Source of truth: `app/src/main/cpp/include/adas/middleware/manager.hpp`.
Tests that read like a tutorial: `app/src/main/cpp/tests/test_middleware.cpp`.

## Build one first: a bus in sixty lines

The fastest way to read the real bus is to have written a small one. Three properties matter, and they
fit on one screen: **topics** decouple who talks from who listens; **delivery is deterministic** —
messages published during a tick are delivered on the next, so a run is reproducible; and **the queue
is observable**, because a queue you cannot see is where systems die:

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

A service owns state, subscribes, publishes — nobody calls anybody. Here a toy planner (inverse-variance
lane fusion) and a toy controller (pure pursuit in the ego frame) move onto the bus:

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

The acceptance for an architecture step is *equality* — services on a bus must reproduce the plain
function calls bit for bit:

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

Now watch it die the real way. The ZMQ bridge on the phone once drained **one message per 10 ms tick**
— a ceiling of 100 msg/s. A third 28 Hz topic pushed the inflow past it, the queue grew, the plan aged,
and the staleness gate silently dropped lateral control a minute into every drive. Reproduce it:

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

The queue does not stabilise at some lag — it **diverges linearly**, and the age of what you act on
diverges with it. The fix (drain until empty, with a sane bound) is one line here and one constant on
the phone, `kMaxPerTick = 32`. Everything below is this toy grown up: threads, timers, parameters, and
`middleware/stats` — which is your `max_backlog` with a uniform.

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
| `RealTime` | each service thread + OS time | phone / APK |
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

Register it:

```cpp
adas::middleware::Manager mw(adas::middleware::Manager::Mode::Simulated);
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

Defined in `adas/utils/adas_topics.h`:

| topic | typical producer → consumer |
|---|---|
| `vision/lanes` | Java (ZMQ) → `ZmqBridge` → `proto_convert` |
| `vision/path` | `Planner` → `SafetyWarn`, bag |
| `vision/model_long` | Java → `SafetyWarn`, long plan |
| `vehicle/chassis`, `vehicle/state` | `Platform` (CAN decode) / Java → many |
| `control/lat_plan`, `control/long_plan` | `Planner` → `Control` |
| `control/lane_keep` | `Planner` → bag / UI |
| `controls/steer` | `Control` → `Platform` / bag |
| `panda/health`, `can/rx` | `Platform` → supervisor, bag |
| `safety/warn` | `SafetyWarn` → ZMQ OUT → UI / bag |
| `middleware/stats` | stats service → bag |

Read that table as the shape of the lateral loop: the plan is published in curvature by `Planner`, turned
into a command by `Control`, and only `Platform` ever touches CAN. Each arrow is a place a test can cut.

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

## Is the bus the bottleneck? Measure, do not assume

A message bus is the natural suspect when a pipeline is slow, and suspicion is cheap. This one was measured,
and the answer shapes how you read every latency number elsewhere in the book.

**Publishing wakes the subscriber immediately.** `publish` copies into each subscriber queue and calls
`cv.notify_one()`, so the owning thread is runnable at once — it does not wait for its next timer tick. The
only two places where anything polls are the boundaries with the outside world: the ZMQ ingress from Java and
the CAN egress to the panda, both on 10 ms timers because the thing on the other side has no way to wake us.

The difference between those two designs is worth seeing rather than believing:

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

**Timers fire when they say they do.** Over 143 700 firings on a real drive, the mean interval was
**10.00 ms** against a nominal 10. The worst cases were 44.8 ms in the panda service — most likely a USB
stall, i.e. the outside world, not the scheduler — and 21.8 ms in the ZMQ bridge.

So the measured conclusion is: **the middleware never delays a message on a timer**, and the non-inference
latency in the pipeline is not the bus. Of the ~8 ms between model output and the published plan, about 5 is
the ZMQ ingress poll — a real cost, and the only one worth naming.

That share grew without the bus getting any slower. When inference took 45.6 ms, the poll was 6 % of the
budget and not worth a sentence; at 17.6 ms it is 10 % of a much smaller budget. Fixed costs become the
story once you remove the variable one — which is the normal end state of optimisation, not a new problem.

```{admonition} What this buys you diagnostically
:class: tip
When a bag shows a stale reference, the bus is already ruled out. That is worth more than it sounds: it means
the search space is the camera → model chain or the outside boundaries, and you can stop reading
`manager.hpp`.
```

## Queues and drops

Default subscriber capacity is finite (~100). If a slow service does not drain:

* when full, the **oldest** messages are dropped (`pop_front`), so the queue keeps the newest ~100;
* `middleware/stats` shows lagging timers and drops, and it is in every bag.

That is deliberate: a stale measurement is useless to the loop, and a slow subscriber must not stall the
publisher. The loss shows up in the stats instead of as a growing delay.

```{warning}
A "bad controller" on a bag where `middleware/stats` shows huge timer lag is often a **scheduling** story, not
a gain story. Check it before touching a weight — it takes one plot.
```

With the vision pipeline the policy is the **same in direction** — both drop the oldest to keep the newest —
but different in **depth**. The bus holds up to ~100 messages (a logger or bag analysis wants a series);
`VisionPipeline` keeps exactly **one** slot and overwrites the pending frame. A control loop has no use for a
backlog of stale frames, so vision is stricter; the bus is softer because the same topic may feed both the
loop and the recorder.

## How to study this in the repo

1. Skim `tests/test_middleware.cpp` (`PubService`, `SubService`, `TimerService`).
2. Open `src/services/planner.cpp` → `configure()`: parameters + subscriptions.
3. Open `src/services/zmq_bridge.cpp`: Java messages enter the same bus.
4. Run host tests: `./scripts/docker.sh tests` (or `build_cpp.sh -t linux --test`).

## Exercise

1. Draw the path of one `ChassisSample` from `Platform` (or a bag) to `Control`.
2. Add a hypothetical parameter `echo.gain` — which thread writes it, which thread reads it?
3. In Simulated mode, why must you call `setTime` before `step` for timers to fire?

<!-- next-chapter -->
---

**Next:** [Java layer](./JavaLayer.md)
