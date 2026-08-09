# Vehicle Model (Understeer)

## Kinematics vs reality

From the bicycle chapter, **kinematic** curvature for wheel angle $\delta$ is

$$
\kappa_{\mathrm{kin}} = \frac{\tan\delta}{L}.
$$

If the car truly followed that, yaw rate would satisfy $\dot\psi = v\,\kappa_{\mathrm{kin}}$.
Define the **measured** curvature from sensors:

$$
\kappa_{\mathrm{fact}} = \frac{\dot\psi}{v}.
$$

At parking-lot speeds $\kappa_{\mathrm{fact}} \approx \kappa_{\mathrm{kin}}$.
On the highway the tires slip: for the **same** $\delta$ you get **less** curvature,

$$
\frac{\kappa_{\mathrm{fact}}}{\kappa_{\mathrm{kin}}} < 1.
$$

![Understeer: kinematics vs fact](figures/understeer_idea.png)

If feed-forward uses $\kappa_{\mathrm{kin}}$ only, it **under-orders** SWA in arcs → outward drift; feedback fights late, after vision delay.

## Simple understeer formula

A one-parameter model used in openpilot-style stacks (and our `tire_stiffness_factor`) looks like

$$
\kappa_{\mathrm{fact}} \approx \frac{\kappa_{\mathrm{kin}}}{1 - c\, v^{2}}
\quad\text{with understeer coefficient } c < 0 \text{ in the usual slip convention,}
$$

or equivalently: to **achieve** a desired $\kappa_{\mathrm{des}}$ you must command a larger wheel angle than $\arctan(L\kappa_{\mathrm{des}})$.

Operationally in code:

$$
\delta_{\mathrm{kin}} = \arctan(L\,\kappa_{\mathrm{des}}),
\qquad
\delta_{\mathrm{cmd}} = f_{\mathrm{VM}}(\kappa_{\mathrm{des}}, v) > \delta_{\mathrm{kin}} \text{ at high } v.
$$

```{admonition} Intuition
:class: tip
Tires make the car "longer" in curvature space: same $\delta$, smaller $\kappa$.
The vehicle model **scales the command up** so the mental bicycle matches the real Golf.
```

## Measurement recipe (bag)

On frames where the **driver** steers (or any known SWA):

1. $\delta = \mathrm{SWA} / (\texttt{steer\_sign}\cdot\texttt{steer\_ratio})$.
2. $\kappa_{\mathrm{kin}} = \tan\delta / L$.
3. $\kappa_{\mathrm{fact}} = \dot\psi / v$ (need $v$ not tiny).
4. Bin by $v$, report median ratio $\kappa_{\mathrm{fact}}/\kappa_{\mathrm{kin}}$.

Reference table from our Golf:

| $v$, m/s | $\kappa_{\mathrm{fact}}/\kappa_{\mathrm{kin}}$ |
|---|---:|
| 4–8 | ~0.99 |
| 12–16 | ~0.85 |
| 20–26 | **~0.61** |

At ~80 km/h kinematics overstates curvature by ~$1/0.61 \approx 1.6\times$.

```python
import math
import numpy as np

L = 2.636
STEER_RATIO = 15.7
STEER_SIGN = -1

def delta_from_swa_deg(swa_deg: float) -> float:
    return math.radians(swa_deg / (STEER_SIGN * STEER_RATIO))

def ratio_kin_vs_fact(swa_deg, yaw_rate_rad_s, v_mps):
    """Return kappa_fact / kappa_kin (NaN if unstable)."""
    if abs(v_mps) < 1.0:
        return float("nan")
    delta = delta_from_swa_deg(swa_deg)
    k_kin = math.tan(delta) / L
    k_fact = yaw_rate_rad_s / v_mps
    if abs(k_kin) < 1e-6:
        return float("nan")
    return k_fact / k_kin

# At 22 m/s the measured ratio on this car is 0.54 — not a guess, see the table below.
v = 22.0
swa = -60.0  # deg on CAN
delta = delta_from_swa_deg(swa)
k_kin = math.tan(delta) / L
k_fact = 0.54 * k_kin
yaw = k_fact * v
print(f"delta={math.degrees(delta):.2f} deg, k_kin={k_kin:.4f}, "
      f"ratio={ratio_kin_vs_fact(swa, yaw, v):.2f}")
```

How much extra SWA does feed-forward need? To recover the $\kappa_{\mathrm{fact}}$ that kinematics
promised at low speed you would scale by $1/0.54 \approx 1.85$ at 22 m/s. Hold that number lightly: it is
what a *naive* inversion asks for, and the real vehicle model spreads the correction smoothly over speed
instead of applying a single factor. The gap between 1.85 and the 3–9 % the shipped parameter actually
changes is the subject of the rest of this chapter.

```python
k_des = 0.02  # 1/m, mild arc
delta_kin = math.atan(L * k_des)
# crude high-speed correction: inflate the kappa request by the measured shortfall
delta_vm = math.atan(L * k_des / 0.54)
print(f"|SWA| kin={abs(math.degrees(delta_kin)*STEER_RATIO):.1f} deg, "
      f"VM~{abs(math.degrees(delta_vm)*STEER_RATIO):.1f} deg")
```

## Project knobs

* C++ `vehicle_model.h`, Python `scripts/core/vehicle_model.py`
* `vehicle.lat_use_vehicle_model = true`
* `tire_stiffness_factor` — **0.50 shipped**, and the rest of this chapter is about how that number was
  arrived at, because the route to it is more useful than the value.

## Calibrating one coefficient, and finding out it cannot be one coefficient

Here is where a course normally stops: you have a formula with one free parameter, you measure the ratio
on a bag, you solve for the parameter. We did that. It produced a contradiction that took two runs and a
third sensor to resolve, and the contradiction is the lesson.

### The contradiction

Two independent sources disagreed about the *direction* of the correction:

| source | says | implies |
|---|---|---|
| our bag measurement | $\kappa_{\text{fact}}/\kappa_{\text{kin}} = 0.54$ at 22 m/s | understeer **stronger** than our shipped 0.64 assumes → lower the factor |
| comma's `liveParameters`, learned on **this same car** | `tireStiffnessFactor` = 1.319 ± 0.007 | understeer **weaker** → raise it, by a factor of two |

Both cannot be right, and neither could be dismissed: their estimator ran for hours on our roads with a
tiny spread, and our measurement was straightforward arithmetic on our own logs.

### How it was resolved: find a third sensor

The suspect was the input the two calculations share and neither validates — the yaw rate. Ours comes from
the car's ESP sensor over CAN. If that sensor were scaled, our 0.54 would be wrong by exactly that scale
and the argument would be over.

So: compare it against two other things that measure the same physical quantity.

```python
# Three sensors, one quantity. Ratios measured on run 2026_08_04_21_00_18.
PAIRS = {"phone gyro / ESP": 1.017, "camera odometry / ESP": 0.849, "camera odometry / gyro": 0.788}
for name, ratio in PAIRS.items():
    print(f"{name:>24}: {ratio:.3f}")

print("\nTwo physically independent sensors — a MEMS gyro and a wheel-based ESP unit — agree to 1.7 %,")
print("with no dependence on speed. The camera is the outlier against both, consistently, and by")
print("roughly the amount its own metric scale is off (0.888). So the ESP yaw rate is sound, and the")
print("0.54 measurement stands.")
```

That closed the question in the only way it could be closed: not by arguing about which estimator is
better, but by adding a measurement that makes one of them checkable. Until that ran,
`tire_stiffness_factor` was frozen — deliberately, because moving a coefficient while its own input is
under suspicion produces a number that fits one run and nothing else.

```{admonition} The general move
:class: tip
When two estimates of a parameter disagree, do not average them and do not pick the more authoritative
source. Find the input they share and measure it against something outside both.
```

### What the change actually bought

With the input validated, the factor moved 0.64 → 0.50 and the next drive was measured on the same road:

| | before (0.64) | after (0.50) |
|---|---|---|
| left arc, total offset | +0.30 m | **+0.23 m** |
| left arc, tracking error | +0.29 m | **+0.21 m** |

```python
def understeer_command(kappa, v_ms, stiffness_factor, L=2.636, ratio=15.7):
    """Feedforward with the understeer term, in steering-wheel degrees."""
    K = 0.0015 / max(stiffness_factor, 1e-3)
    return math.degrees(math.atan(kappa * L) * (1.0 + K * v_ms * v_ms)) * ratio

print(f"{'radius':>8} {'speed':>7} {'tsf 0.64':>10} {'tsf 0.50':>10} {'change':>8}")
for radius, v in ((231.0, 13.6), (150.0, 13.8), (134.0, 11.8), (123.0, 13.8)):
    a = understeer_command(1.0 / radius, v, 0.64)
    b = understeer_command(1.0 / radius, v, 0.50)
    print(f"{radius:>6.0f} m {v:>5.1f} m/s {a:>9.1f}° {b:>9.1f}° {100 * (b / a - 1):>+7.1f}%")
```

Note the size of it: **3 to 9 % more steering angle**. A trim, not a lever — and it removed a quarter of
the tracking error. That is a good ratio of effect to intervention, and it is also the warning sign: if a
3 % change in the command moves the outcome by 25 %, the loop is operating close to a limit.

### Why this cannot end with a constant

It ends here because the ratio itself is not constant:

| speed | measured ratio |
|---|---|
| 6–9 m/s | 0.97 |
| 12–15 m/s | 0.80 |
| 21–26 m/s | 0.54 |

A single `tire_stiffness_factor` fits one row. Pick it for the city and the highway under-commands; pick it
for the highway and city arcs get cut. And 0.50 was measured to be right on urban arcs but made a
**replayed** urban arc worse (−0.44 → −0.52 m), which is the kind of split you get when one parameter is
carrying two jobs.

The structural answer is to stop fitting a constant and estimate the parameters continuously, with the
$v^2$ dependence in the observation model rather than baked into a number. That is what upstream's
`paramsd` does — a nine-state filter over stiffness, steer ratio, two steering biases and road grade —
and it is on this project's list. The prerequisite it needs, and we do not yet have, is an estimate of
**road roll**: a banked road produces the same lateral signature as an understeering car, and no amount of
yaw-rate accuracy separates them.

## Pipeline delay, and why the compensation exceeds the measurement

The second thing the feedforward has to know is that its picture of the road is old. Medians measured on a
28-minute night run:

| link | ms | measured how |
|---|---:|---|
| capture → model output | 54 | `infer_ts − capture_ts` in `vision/lanes` |
| model output → steering command | 21 | to the `controls/steer` publish |
| panda CAN transmit | ~10 | its own 10 ms timer, not stamped |
| **to the wire** | **~89** | |
| command → rack angle | ~40 | EPS response, from CAN |
| rack → yaw rate | ~120 | vehicle dynamics |
| **to yaw rate** | **~250** | |

`fp_steer_delay_s = 0.35` is deliberately larger than any of those sums, and the reason is worth stating
because it looks like sloppiness. The first three rows are *transport* — pure dead time, and a lookahead
compensates them exactly. The last two are *dynamics*: a first-order-ish response, where compensating with
its nominal time constant leaves no margin for the response being slower than nominal on a cold rack, a
worn tyre, or a rough surface. Under-compensating drifts; over-compensating oscillates, and the loop
tolerates the first far better than the second.

```python
V = 22.0
print(f"{'delay':>8} {'travel at 22 m/s':>18} {'what it covers'}")
for delay, what in ((0.089, "to the wire"), (0.25, "to yaw rate"), (0.35, "shipped lookahead")):
    print(f"{delay:>7.3f}s {V * delay:>15.2f} m   {what}")
print()
print("The car covers 7.7 m during the shipped lookahead. On a 130 m arc that is 3.4 deg of")
print("heading, which is why the lookahead is a feedforward input and not a tuning nicety.")
```

```{admonition} Arc drift diagnostics
:class: warning
Order: (1) vehicle model on; (2) vision Hz / e2e OK; (3) only then CTE / epsi weights.
```

## Exercise

1. Run the synthetic `ratio_kin_vs_fact` snippet; change `0.54` to `0.99` and see the SWA inflation vanish.
2. On a bag with driver steering, plot ratio by $v$ bins; compare to the table.
3. Argue in one sentence why cutting `fp_steer_delay_s` to $0.23$ can still hurt closed loop.

<!-- next-chapter -->
---

**Next:** [MPC and fp](./MPC_and_FP.md)
