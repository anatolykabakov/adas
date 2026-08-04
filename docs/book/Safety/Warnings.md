# Warnings: FCW, AEB, LDW

Service **`safety_warn`** raises **icons / HUD text only** — it does **not** brake or steer.
That is intentional: students can study threat logic without actuation risk.

Canonical eng doc: [`SAFETY_WARN.md`](../../SAFETY_WARN.md).
Code: `safety_planner.hpp`, `safety_warn_service.cpp`, tests `test_safety_warn.cpp`.

## What the acronyms mean here

| flag | meaning in this project |
|---|---|
| **FCW** | Forward Collision Warning — “you are closing too fast” |
| **AEB** | stronger forward threat (UI: red `BRAKE!`) — **still no auto-brake** in our stack |
| **LLDW / RLDW** | Lane Departure Warning left / right |

AEB overrides FCW when both would fire.

## Data path

```text
vision/lanes (Java) → TopicConvert → vision/path   ─┐
vision/model_long (lead0, …)                       ─┼→ SafetyWarnService @ 50 ms
vehicle/chassis (v, blinker, steering_pressed)     ─┘
        → safety/warn → ZMQ OUT → HUD / bag
```

```{note}
`LongPlanService` may also read `model_long` for ACC-like **desired accel**.
Warnings do **not** use that IDM accel anymore (it false-triggered on empty highways).
```

## Longitudinal threat math (FCW / AEB)

Need a **present lead** (`lead0` only — not future hypotheses lead1/2):

* probability ≥ `lead_prob_thresh` (0.5);
* distance $d$ in $(1,\ 150)$ m;
* ego speed $v \ge$ `warn_min_speed_ms` (3 m/s);
* lead roughly **in path**: $|y_{\mathrm{lead}} - y_{\mathrm{path}}(d)| \le$ `lead_max_offset_m`;
* closing: $\Delta v = v - v_{\mathrm{lead}} \ge$ `min_closing_speed_ms`.

Then

$$
\mathrm{ttc} = \frac{\mathrm{gap}}{\Delta v},
\qquad
a_{\mathrm{req}} = \frac{(\Delta v)^{2}}{2\cdot\mathrm{gap}}.
$$

With shipped thresholds:

| | TTC | required decel |
|---|---:|---:|
| FCW | $\le 2.5$ s | or $a_{\mathrm{req}} \ge 3.5$ m/s² |
| AEB | $\le 1.4$ s | or $a_{\mathrm{req}} \ge 5.5$ m/s² |

```python
def forward_warning(gap_m, v_ego, v_lead, *,
                    min_close=0.5, fcw_ttc=2.5, aeb_ttc=1.4,
                    fcw_a=3.5, aeb_a=5.5):
    dv = v_ego - v_lead
    if gap_m <= 1.0 or dv < min_close:
        return None, float("nan"), float("nan")
    ttc = gap_m / dv
    a_req = (dv * dv) / (2.0 * gap_m)
    if ttc <= aeb_ttc or a_req >= aeb_a:
        return "AEB", ttc, a_req
    if ttc <= fcw_ttc or a_req >= fcw_a:
        return "FCW", ttc, a_req
    return None, ttc, a_req

# Slow close from 30 m at 5 m/s relative
print(forward_warning(30.0, 20.0, 15.0))
# Fast close — expect AEB
print(forward_warning(12.0, 25.0, 10.0))
# Not closing
print(forward_warning(30.0, 15.0, 16.0))
```

### Why path-relative $y$ matters

In an arc, a lead car on **your** lane has nonzero $y$ in the vehicle frame.
Comparing to the **axis** false-triggers “adjacent lane”. Comparing to $y_{\mathrm{path}}(d)\approx \mathrm{CTE}+\tfrac12\kappa d^2$ keeps your own leader in-lane.

```python
def path_y(d, cte=0.1, kappa=0.01):
    return cte + 0.5 * kappa * d * d

d, y_lead = 25.0, 0.8
print("in path?", abs(y_lead - path_y(d)) <= 2.0)
```

## Lateral LDW math

Gates (all required unless noted):

* $v \ge$ `ldw_min_speed_ms` (12.5 m/s ≈ 45 km/h);
* `lane_anchored` — path built from **two** plausible lane lines;
* blinker on that side off (`ldw_suppress_on_blinker`);
* driver not holding wheel if `ldw_suppress_on_driver_steer`.

Then fire if

$$
|\mathrm{CTE}| > 0.5~\mathrm{m}
\ \textbf{and}\
\text{outward rate} > 0.05~\mathrm{m/s}
\quad\textbf{or}\quad
|\mathrm{CTE}| > 0.8~\mathrm{m}.
$$

```python
def ldw(cte, cte_rate, side_right, *,
        v=15.0, anchored=True, blinker=False, hands_on=False):
    if v < 12.5 or not anchored or hands_on:
        return False
    if blinker:
        return False
    outward = (cte_rate > 0.05) if side_right else (cte_rate < -0.05)
    # simplify: |cte| with outward OR hard threshold
    return (abs(cte) > 0.5 and outward) or abs(cte) > 0.8

print("hold offset in arc:", ldw(0.55, 0.0, True))   # False (no outward rate)
print("drifting out:", ldw(0.55, 0.1, True))         # True
print("hard offset:", ldw(0.85, 0.0, True))           # True
```

Old rule `|cte|>0.5` alone caused **hundreds** of false episodes on city bags; gated LDW dropped them — see `SAFETY_WARN.md`.

## Debounce (`WarningLatch`)

Tick = 50 ms. Raise after **3** true ticks (150 ms); clear after **10** false (500 ms).

```python
class Latch:
    def __init__(self, set_n=3, hold_n=10):
        self.set_n, self.hold_n = set_n, hold_n
        self.on = False
        self.cnt = 0

    def update(self, raw: bool) -> bool:
        if raw:
            self.cnt = self.cnt + 1 if not self.on else self.hold_n
            if self.cnt >= self.set_n:
                self.on = True
        else:
            if self.on:
                self.cnt -= 1
                if self.cnt <= 0:
                    self.on = False
                    self.cnt = 0
            else:
                self.cnt = 0
        return self.on

L = Latch()
seq = [0, 1, 1, 1, 1, 0, 0, 0]  # spikes then quiet start
print([int(L.update(bool(x))) for x in seq])
```

(Real latch semantics are in `WarningLatch` — use tests as ground truth; this sketch shows *why* debounce exists.)

## Config block

`config.json` → `safety_warn`:

```json
"fcw_ttc_s": 2.5,
"aeb_ttc_s": 1.4,
"fcw_decel_ms2": 3.5,
"aeb_decel_ms2": 5.5,
"warn_min_speed_ms": 3.0,
"cte_ldw_threshold_m": 0.5,
"cte_ldw_hard_m": 0.8,
"ldw_min_speed_ms": 12.5,
"warn_set_frames": 3,
"warn_hold_frames": 10
```

Node switch: `nodes.safety_warn: true`.

## How to reproduce / verify

**Unit tests** (no phone):

```bash
./scripts/docker.sh tests
# or filter: test_safety_warn
```

**Bag episodes** (real Java→convert→service chain via `pyadas`):

```bash
PYTHONPATH=app/src/main/scripts \
  python3 app/src/main/scripts/bag_safety_warn.py adas_logs/<session>
```

Report **episodes**, not raw frame counts.

```{warning}
Many road bags lack usable `vision/model_long` → FCW/AEB stay silent. LDW still needs `lane_anchored` and speed.
```

## Exercise

1. Run the `forward_warning` examples; find a `(gap, Δv)` that is FCW but not AEB.
2. Explain in one sentence why IDM accel was a bad FCW trigger.
3. On a bag, run `bag_safety_warn.py` and record LDW / FCW / AEB episode counts + vision Hz.

<!-- next-chapter -->
---

**Next:** [Calibration](../Calibration/Overview.md)
