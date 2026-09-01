# Longitudinal control — read the plan where the actuator will be

The [longitudinal planner](../Planner/Longitudinal.md) hands over a speed trajectory; this loop turns it into
an acceleration request, the way [angle control](./AngleControl.md) turns a curvature into torque. It is
upstream's `LongControl`: a proportional law with the plan's acceleration as feedforward, a read 0.15 s ahead
to cover the actuator, and a four-state machine that makes a stop a stop.

Code: `longitudinal/long_control.cpp`; the tick lives in `services/control.cpp` next to the lateral law.

Shared constants for the snippet, as in the planner chapter:

```python
import numpy as np

ACCEL_MIN, ACCEL_MAX = -3.5, 2.0   # the panda's envelope for a VW MQB [m/s^2]
```

## The law

The plan is a speed trajectory on the model's 33-point grid, 17 points of it (2.5 s) handed to `Control`.
The car's motor and brake controllers answer an acceleration request about 0.15 s late, so the law reads
the plan **0.15 s ahead** and turns the speed slope over that delay into the acceleration to ask for:

```python
T_MODEL = np.array([10.0 * (i / 32) ** 2 for i in range(33)])[:17]
DELAY = 0.15

def long_control(speeds, accels, v_ego, t_since_plan=0.0, kp=0.1):
    v_now = np.interp(t_since_plan, T_MODEL, speeds)
    a_now = np.interp(t_since_plan, T_MODEL, accels)
    v_target = np.interp(DELAY + t_since_plan, T_MODEL, speeds)
    a_target = 2.0 * (v_target - v_now) / DELAY - a_now       # the slope over the delay, minus what we have
    accel = kp * (v_target - v_ego) + a_target                 # VW: kp 0.1, ki 0, kf 1
    return float(np.clip(accel, ACCEL_MIN, ACCEL_MAX)), v_target, a_target

speeds = np.maximum(0.0, 20.0 - 1.5 * T_MODEL)
accels = np.full(17, -1.5)
cmd, v_t, a_t = long_control(speeds, accels, v_ego=20.0)
print(f"braking plan at -1.5: v_target {v_t:.2f} m/s, feedforward {a_t:.2f}, command {cmd:.2f} m/s^2")
assert abs(a_t + 1.5) < 1e-9, "on a plan of constant acceleration the slope gives back that acceleration"
```

Where the $2(\cdot)/\Delta$ comes from: over the delay $\Delta = 0.15$ s the speed should move from $v_{\mathrm{now}}$
to $v_{\mathrm{target}}$ while the acceleration ramps linearly from what the car has now, $a_{\mathrm{now}}$, to what
we ask, $a_{\mathrm{target}}$. The average acceleration over the ramp is $\tfrac12(a_{\mathrm{now}} + a_{\mathrm{target}})$, so

$$
v_{\mathrm{target}} - v_{\mathrm{now}} = \tfrac{1}{2}\,(a_{\mathrm{now}} + a_{\mathrm{target}})\,\Delta
\quad\Longrightarrow\quad
a_{\mathrm{target}} = \frac{2\,(v_{\mathrm{target}} - v_{\mathrm{now}})}{\Delta} - a_{\mathrm{now}} .
$$

On a plan of constant acceleration the formula gives that acceleration back exactly (the snippet asserts
it). On a plan that *bends* — the car is at $a = 0$ but the plan wants $-1.5$ in 0.15 s — it asks for
$-3.0$ for one tick and relaxes: the request leads the plan by the delay, so the car does not lag it. The
proportional term $0.1\,(v_{\mathrm{target}} - v_{\mathrm{ego}})$ is small on purpose: with kp 0.1 a 2 m/s speed error adds
only $0.2\ \mathrm{m/s^2}$ — the feedforward does the driving, the P term only corrects the drift.

```{figure} figures/long_control_law.png
---
width: 100%
---
Left: the plan read at the actuator delay. Right: the four states — `pid` while moving, `stopping` once
the plan is at rest below 1 m/s (the request ramps to −2 m/s² and holds), `starting` when the plan leaves
(+1 m/s² until 1 m/s), then `pid` again.
```

The **state machine** is what makes a stop a stop: below 1 m/s with a plan at rest the law leaves the
proportional world and ramps to a hold of −2 m/s² at 0.8 m/s³ — upstream's `stopAccel` and
`stoppingDecelRate` — because a P law on a speed of 0.3 m/s would creep. A subtlety that cost an evening:
upstream's `stay_stopped` reads the *stock ACC's* standstill flag, which does not exist once we own the
axis; feeding the plain speed there made `starting` and `stopping` trade places every tick.

## Acceptance

* the snippet gives back a constant plan acceleration exactly, and adds $0.1$ m/s² per m/s of speed error;
* the state machine walks pid → stopping → starting → pid on a stop-and-go plan, holding −2 m/s² at rest;
* one sentence on why `stay_stopped` must not read the plain speed.

## For depth

* [Longitudinal planner](../Planner/Longitudinal.md) — where the speed trajectory comes from.
* [Angle control](./AngleControl.md) — the lateral inner loop this one is the twin of.
* [Platform](../Platform/Overview.md) — how the acceleration becomes `ACC_06`/`ACC_07` on the bus.

<!-- next-chapter -->
---

**Next:** [Platform](../Platform/Overview.md)
