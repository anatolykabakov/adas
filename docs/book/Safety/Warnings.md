# Warnings: FCW, AEB, LDW

Service **`safety_warn`** raises **icons / HUD text only** — it does **not** brake or steer.
That is intentional: students can study threat logic without actuation risk.

Canonical eng doc: `docs/SAFETY_WARN.md`.
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

## How these rules were arrived at: three rounds of being wrong

Nothing in this chapter was designed on paper. Each rule replaced a simpler rule that fired when there was
no danger, and the sequence is worth following because the failures are more instructive than the formulas.

### Round 1: acceleration as a proxy for danger

The first version raised FCW when the IDM desired acceleration fell below −3 m/s², and AEB below −5. It
seemed reasonable: hard braking wanted means danger ahead.

It is wrong because IDM acceleration includes terms that have nothing to do with a lead car.

```python
def idm_accel(v_ego, v_lim, gap=None, v_lead=None, a_max=1.5, b=3.0, T=1.5, s0=2.0, expn=4.0):
    """Treiber IDM. The free-road term alone can demand hard braking with nothing in front."""
    free = (v_ego / max(v_lim, 0.1)) ** expn
    inter = 0.0
    if gap is not None and v_lead is not None:
        dv = v_ego - v_lead
        s_star = s0 + max(0.0, v_ego * T + v_ego * dv / (2.0 * (a_max * b) ** 0.5))
        inter = (s_star / max(gap, 0.5)) ** 2
    return a_max * (1.0 - free - inter)

MU_G = 0.5 * 9.81
print(f"{'situation':>38} {'a_idm':>8}  would fire?")
# Empty road, over the assumed limit
print(f"{'empty road at 37 m/s, limit 27.8':>38} {idm_accel(37.0, 27.8):>8.2f}  FCW")
# Empty arc: the curvature speed limit becomes v_lim
v_lim_arc = (MU_G / (1.0 / 50.0)) ** 0.5
print(f"{'empty R=50 m arc at 22 m/s':>38} {idm_accel(22.0, v_lim_arc):>8.2f}  FCW")
# An actual lead car, closing
print(f"{'lead 15 m ahead at 13 m/s, we do 20':>38} {idm_accel(20.0, 27.8, 15.0, 13.0):>8.2f}  FCW")
```

The first two lines have no car in them. An empty road above the assumed limit and an empty tight bend both
demand hard braking, and both used to light the collision warning. The lesson generalises past this
project: **a controller's desired output is not a hazard signal.** It contains everything the controller
cares about, and a warning must be built from the hazard itself.

So the rule became threat-based — time to collision and required deceleration, computed from a target that
exists — and IDM acceleration stayed in the message as a debug field only.

### Round 2: the target that is not there yet

The second version took the most probable of `lead0`, `lead1`, `lead2`. Those are not three candidate cars:
`lead1` and `lead2` are the model's predictions of the lead **at +2 s and +4 s**. Warning about a car that
the model thinks will be there in four seconds is warning about the future, which is a different product.

Fixed by using `lead0` only. Worth flagging because the same defect survived in the longitudinal planner
for two more months, and was found only when that planner was first allowed to act.

### Round 3: warning about ourselves

With threat-based rules and the right target, a 23-minute night drive still produced 5 forward and 7 lane
warnings, all false. Two distinct causes, and both are visible only in a measurement:

| class | what the numbers said | the gate that followed |
|---|---|---|
| forward | all 5 episodes were stop-and-go: median 4.7 m/s, max 8.5. Worst case 4.3 m/s with a nearly stationary lead 9.5 m ahead — TTC 1.9 s arithmetically, trivial in practice | `warn_min_speed_ms` 3 → 8 m/s |
| lane | of 144 LDW frames, **82 % had our own lateral control engaged**, and the driver was steering in only 6 %. \|CTE\| 0.54 m with 0.15 m/s of drift | suppress LDW while we steer (`ldw_suppress_on_lat_active`) |

The second one is the interesting failure. LDW exists to warn a *drifting driver*. With the assistant
engaged, the offset it measures is the assistant's own tracking error on an arc — so the system was
warning about itself, and it did so 82 % of the time it spoke.

```{admonition} A header default is not a decision
:class: warning
The speed gate above was raised in the code and **not** in `assets/config.json`, which keeps its own value
and wins. So the fix was dead, and the next run reproduced the same three false warnings at 4.8–5.5 m/s.
Three tests now assert that the shipped config matches the decisions taken on the road, and the build
refuses to compile a config it cannot parse.
```

## Longitudinal threat math (FCW / AEB)

Need a **present lead** (`lead0` only — not future hypotheses lead1/2):

* probability ≥ `lead_prob_thresh` (0.5);
* distance $d$ in $(1,\ 150)$ m;
* ego speed $v \ge$ `warn_min_speed_ms` (**8 m/s** — raised from 3 after the stop-and-go false positives in round 3);
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
* driver not holding wheel if `ldw_suppress_on_driver_steer`;
* **we are not steering ourselves** (`ldw_suppress_on_lat_active`) — 82 % of the false lane warnings were the
  assistant measuring its own tracking error on an arc.

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
"warn_min_speed_ms": 8.0,
"cte_ldw_threshold_m": 0.5,
"cte_ldw_hard_m": 0.8,
"ldw_min_speed_ms": 12.5,
"ldw_suppress_on_lat_active": true,
"warn_set_frames": 3,
"warn_hold_frames": 10
```

Two of those values are the outcome of round 3 above, and both live in the shipped config rather than only in
the header defaults — a header default the config overrides is not a decision, which is how the speed gate
stayed dead for two days.

Node switch: `nodes.safety_warn: true`.

## How to reproduce / verify

**Unit tests** (no phone):

```bash
./scripts/docker.sh tests
# or filter: test_safety_warn
```

**Bag episodes** (real Java→convert→service chain via `pyadas`):

```bash
PYTHONPATH=scripts \
  python3 scripts/bag/bag_safety_warn.py adas_logs/<session>
```

Report **episodes**, not raw frame counts.

```{warning}
Many road bags lack usable `vision/model_long` → FCW/AEB stay silent. LDW still needs `lane_anchored` and speed.
```

## Exercise

1. Run the `forward_warning` examples; find a `(gap, Δv)` that is FCW but not AEB.
2. Explain in one sentence why IDM accel was a bad FCW trigger.
3. On a bag, run `bag/bag_safety_warn.py` and record LDW / FCW / AEB episode counts + vision Hz.

<!-- next-chapter -->
---

**Next:** [Calibration](../Calibration/Overview.md)
