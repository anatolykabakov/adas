# Kinematic Bicycle Model

> Geometric basis of Pure Pursuit and part of MPC. Figures — from [AAD](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) (CC BY 4.0).

With a **constant** front-wheel angle $\delta$, the rear axle traces a **circular arc** in the plane.
Lateral control = choose $\delta$ (then SWA on the bus) so that arc matches the road you want.

## Wheel steer angle vs SWA

| quantity | meaning |
|---|---|
| $\delta$ | front wheel angle relative to the body (model) |
| SWA | steering-column / rack angle on CAN |

They are related by a nearly constant gear ratio and a sign convention:

$$
\mathrm{SWA} = \mathrm{steer\_sign} \cdot \delta \cdot \mathrm{steer\_ratio}.
$$

Golf MQB numbers in `config.json`: $L \approx 2.636$ m, `steer_ratio ≈ 15.7`, `steer_sign = -1`. The hand-worked examples below round $L$ to 2.64 m for legibility; the runnable toy-control blocks use the exact 2.636.

```{admonition} Tiny numerical example
:class: tip
$\delta = +5°$ (wheels turn "model-positive").
Then $|\mathrm{SWA}| = 15.7 \times 5° = 78.5°$, and with `steer_sign = -1` the CAN command is $\mathrm{SWA} = -78.5°$.
One degree at the wheel is about sixteen degrees at the steering wheel — that is why bag SWA looks "large".
```

```python
import math

L = 2.636          # m, wheelbase
STEER_RATIO = 15.7
STEER_SIGN = -1

def swa_from_delta_deg(delta_deg: float) -> float:
    """Wheel angle [deg] → steering-wheel angle on CAN [deg]."""
    return STEER_SIGN * delta_deg * STEER_RATIO

print(swa_from_delta_deg(5.0))   # -78.5
print(swa_from_delta_deg(-5.0))  # +78.5
```

![Definition of front wheel angles](figures/WheelAngle.png)

Left and right road wheels differ (Ackermann). The bicycle model replaces the front axle by **one** equivalent $\delta$.

## Why $\tan\delta = L/R$

![Bicycle model on vehicle body](figures/BicycleModel.png)

![Arc at constant δ](figures/BicycleModelGeometry.png)

Assume rolling **without lateral slip**. The rear wheel velocity is along the body; the front wheel velocity is along the steered wheel.
Instantaneous motion is pure rotation about the **ICR** (intersection of the wheel axes).

Look at the right triangle ICR–rear–front:

* opposite to $\delta$ is the wheelbase $L$;
* adjacent is the arc radius $R$ of the rear axle.

Therefore

$$
\tan\delta = \frac{L}{R}.
$$

Solve for the three equivalent forms students must memorize:

$$
\delta = \arctan\frac{L}{R},
\qquad
R = \frac{L}{\tan\delta},
\qquad
\kappa = \frac{1}{R} = \frac{\tan\delta}{L}.
$$

Here $\kappa$ is **path curvature** of the rear axle [1/m]. Straight road: $\kappa = 0$ $\Rightarrow$ $\delta = 0$.

### Worked example (by hand)

Take $L = 2.64$ m and $\delta = 5° = 5\pi/180 \approx 0.08727$ rad.

$$
\kappa = \frac{\tan(0.08727)}{2.64} \approx \frac{0.0875}{2.64} \approx 0.0331~\mathrm{m}^{-1},
\qquad
R = \frac{1}{\kappa} \approx 30.2~\mathrm{m}.
$$

So a gentle $5°$ at the wheels is already a ~30 m radius — city-arc territory.

```python
import math

def kappa_from_delta(delta_rad: float, L: float = 2.64) -> float:
    return math.tan(delta_rad) / L

def delta_from_radius(R: float, L: float = 2.64) -> float:
    return math.atan(L / R)

delta = math.radians(5)
k = kappa_from_delta(delta)
R = 1.0 / k
print(f"kappa={k:.4f} 1/m,  R={R:.1f} m")

# Inverse check: radius 50 m → wheel angle
print(f"delta for R=50 m: {math.degrees(delta_from_radius(50)):.2f} deg")
```

Expected print roughly: `kappa=0.0331`, `R=30.2`, `delta for R=50 m ≈ 3.02°`.

## State $(x, y, \theta)$

| symbol | meaning |
|---|---|
| $x, y$ | rear-axle position in a planar world frame [m] |
| $\theta$ | yaw (heading) [rad] |
| $v$ | longitudinal speed [m/s] |

![State x, y, θ](figures/BicycleModel_x_y_theta.png)

With no slip, the kinematic ODEs are:

$$
\dot x = v\cos\theta,
\qquad
\dot y = v\sin\theta,
\qquad
\dot\theta = \frac{v}{L}\tan\delta = v\,\kappa.
$$

```{admonition} Discrete step (what a simulator does)
:class: note
For a small $\Delta t$:
$x \leftarrow x + v\cos\theta\,\Delta t$,
$y \leftarrow y + v\sin\theta\,\Delta t$,
$\theta \leftarrow \theta + v\tan\delta / L \cdot \Delta t$.
```

```python
import math

def bicycle_step(x, y, theta, v, delta, L=2.64, dt=0.05):
    """One Euler step of the kinematic bicycle [SI units]."""
    x = x + v * math.cos(theta) * dt
    y = y + v * math.sin(theta) * dt
    theta = theta + v * math.tan(delta) / L * dt
    return x, y, theta

# Drive 2 s at 10 m/s with +3° wheel angle — expect slow left/right turn
x = y = theta = 0.0
delta = math.radians(3)
for _ in range(40):  # 40 * 0.05 s = 2 s
    x, y, theta = bicycle_step(x, y, theta, v=10.0, delta=delta)
print(f"after 2 s: x={x:.2f} m, y={y:.2f} m, yaw={math.degrees(theta):.1f} deg")
```

## Instantaneous center of rotation (ICR)

For planar rigid-body motion there is always an ICR such that

$$
\dot{\mathbf{r}} = \boldsymbol{\Omega}\times(\mathbf{r}-\mathbf{r}_{\mathrm{ICR}}).
$$

![ICR and velocity orthogonality](figures/ICR.png)

![Constructing ICR from two velocity directions](figures/ICR_construction.png)

No-slip ⇒ wheel velocity ⊥ wheel axle. Draw both axle lines; they meet at ICR. Ackermann steering is exactly "make those lines meet at one point".

### Lateral slip (preview)

At high $v$, tires slip sideways. The true ICR moves, and

$$
\kappa_{\mathrm{fact}} = \frac{\dot\psi}{v} < \frac{\tan\delta}{L} = \kappa_{\mathrm{kin}}.
$$

![ICR with slip](figures/ICR_Slip.png)

Highway kinematics **overstate** curvature — next chapter.

## Try to control it — and watch the obvious controller fail

The model is now a plant you can drive. Give it a road and the most obvious controller there is —
steer against the offset — and the chapter earns its next one. Everything below is numpy only:

```python
import math
import numpy as np

DT = 0.05     # control tick [s] — 20 Hz, the HCA frame rate on the real car
L = 2.636     # wheelbase [m] — the Golf's, straight from config.json
V = 10.0      # speed [m/s], constant in this tier

def step(x, y, psi, delta, v=V, dt=DT):
    """One tick of the kinematic bicycle."""
    x += v * math.cos(psi) * dt
    y += v * math.sin(psi) * dt
    psi += v * math.tan(delta) / L * dt
    return x, y, psi
```

The road is a sine: peak curvature $A(2\pi/\lambda)^2 \approx 0.0048$ 1/m, a radius of about 210 m —
the "gentle arc" class from `docs/CONTROLLER_LIMITS.md`:

```python
A, LAM = 1.75, 120.0

def y_ref(x):
    return A * math.sin(2 * math.pi * x / LAM)

def psi_ref(x):
    return math.atan(A * 2 * math.pi / LAM * math.cos(2 * math.pi * x / LAM))
```

Offset to the left → steer right, proportionally. One gain, one line:

```python
MAX_STEER = math.radians(25.0)   # the same ceiling the phone enforces

def p_controller(cte, kp=0.10):
    return float(np.clip(-kp * cte, -MAX_STEER, MAX_STEER))

def simulate(controller, t_end=60.0):
    """Drive the sine; return arrays of time and cross-track error."""
    x, y, psi = 0.0, 1.0, 0.0          # start 1 m off the road
    ts, ctes = [], []
    for k in range(int(t_end / DT)):
        cte = y - y_ref(x)
        delta = controller(x, y, psi, cte)
        x, y, psi = step(x, y, psi, delta)
        ts.append(k * DT)
        ctes.append(cte)
    return np.array(ts), np.array(ctes)

t, cte_p = simulate(lambda x, y, psi, cte: p_controller(cte))
first = np.max(np.abs(cte_p[t < 10]))
last = np.max(np.abs(cte_p[t > 50]))
crossings = int(np.sum(np.diff(np.sign(cte_p)) != 0))
print(f"P only: |CTE| max first 10 s {first:.2f} m, last 10 s {last:.2f} m, "
      f"{crossings} zero crossings in 60 s")
```

Plot `cte_p` against `t` and look at it: the car crosses the road again and again and **the swings do
not shrink**. That is not a tuning problem. Steering sets the *curvature* of the trajectory, curvature
integrates into heading, heading integrates into offset — so P-on-offset is a spring with no damper, an
undamped oscillator. Any `kp` changes the frequency, none adds damping. (Forward-Euler integration
actually pumps energy in, so the swings slowly grow.)

Damping comes from the state you were ignoring: heading. This is the **Stanley controller**: steer to
cancel the heading error $\psi_{\text{ref}}-\psi$ (the nose-vs-road angle), plus a term that folds the
offset in as an *angle*. The offset term is $\arctan(k\,\text{CTE}/v)$ — an angle because that is what a
steering command is, and divided by $v$ so the same offset asks for a gentler correction at speed (a hard
yank at 100 km/h is a crash). Heading is the damping the pure-offset spring lacked:

```python
def heading_controller(x, y, psi, cte, kp=2.0):
    # Stanley: cancel heading error, and steer the offset in as a speed-scaled angle.
    err = (psi_ref(x) - psi) - math.atan2(kp * cte, V)
    return float(np.clip(err, -MAX_STEER, MAX_STEER))

t, cte_h = simulate(heading_controller)
settled = np.abs(cte_h[t > 20])
print(f"heading+offset: |CTE| p95 after settling {np.percentile(settled, 95):.3f} m, "
      f"max {settled.max():.3f} m")
assert np.percentile(settled, 95) < 0.15, "acceptance: the car must hold the sine within 15 cm"
assert last > 0.8 * first, "P-only must NOT settle — that is the lesson"
print("acceptance passed: damped follower holds the gentle road within 15 cm")
```

```{figure} figures/toycar_control.png
---
width: 85%
---
The same plant under both controllers: P-on-offset swings without decay; heading + offset settles.
```

Two things to carry out of this build: the working controller holds the sine within 15 cm — and it has
two hand-tuned gains whose right values change with speed and road. A controller whose geometry
*derives* the steering from the path, with no gains for the nominal case, is the
[next chapter](./PurePursuit.md). And the real stack's inner loop is exactly this "heading + offset"
shape: an angle-PID around the rack.

## Does it hold on the real car? Measure before believing

The model is a page of geometry with no free parameters, which makes it both trustworthy and easy to
over-trust. The honest way to end this chapter is to check it, and the check is available: on steady arcs
we can compare the curvature the car achieved, $\dot\psi / v$ from the ESP yaw sensor, against the
$\tan\delta / L$ the model predicts from the steering angle.

```python
# Measured on this Golf, steady arcs, binned by speed. Model expectation from the bicycle model plus a
# textbook understeer gradient — the point is which column moves and how fast.
MEASURED = ((7.5, 0.97, 0.96), (13.5, 0.80, 0.87), (23.5, 0.54, 0.69))

print(f"{'speed':>8} {'achieved/predicted':>19} {'expected':>9} {'verdict'}")
for v, ratio, expected in MEASURED:
    gap = ratio - expected
    verdict = ("model is fine" if ratio > 0.95 else
               "usable with a correction" if ratio > 0.7 else
               "model over-commands by 2x")
    print(f"{v:>5.1f} m/s {ratio:>17.2f} {expected:>9.2f}   {verdict}")
```

Two things to take from that.

At parking and city speeds the model is essentially exact, which is why every textbook derivation stops
here and why it is the right first thing to learn. At 23.5 m/s it delivers barely half the curvature
asked for.

And notice that the measured ratio is **below the textbook expectation at every speed** — 0.54 against
0.69 at 23.5 m/s. So the slip is not just present, it is worse than a standard understeer gradient
predicts. That gap is not a rounding error to absorb into a coefficient; it is the reason
[the vehicle model chapter](../Control/VehicleModel.md) exists and the reason a learned parameter estimator is on
this project's list rather than a hand-tuned constant.

### What the shortfall costs, in metres

```{figure} figures/understeer_drift.png
---
width: 70%
---
Open-loop, the un-compensated shortfall drifts the car outward as the square of arc length.
```


An abstract ratio is easy to shrug at, so convert it. If the model asks for curvature $\kappa$ and the car
delivers $r\kappa$, the car turns on a larger radius than intended and drifts to the **outside** of the
bend. Over an arc of length $s$ the lateral shortfall is approximately

$$
\Delta y \approx \tfrac{1}{2}(1 - r)\,\kappa\,s^2 .
$$

```python
def outward_drift(radius_m, ratio, arc_length_m):
    """Lateral error from delivering only `ratio` of the commanded curvature over an arc."""
    kappa = 1.0 / radius_m
    return 0.5 * (1.0 - ratio) * kappa * arc_length_m ** 2

# The arc episodes actually driven, with the measured ratio at their speed.
for radius, v, ratio, seconds in ((231.0, 13.6, 0.80, 21.7), (150.0, 13.8, 0.80, 12.2),
                                  (134.0, 11.8, 0.80, 14.8), (123.0, 13.8, 0.80, 10.4)):
    s_arc = v * seconds
    print(f"R {radius:>5.0f} m at {v:>4.1f} m/s for {seconds:>4.1f} s ({s_arc:>5.0f} m of arc): "
          f"open-loop drift {outward_drift(radius, ratio, s_arc):>6.1f} m")
print("\nThose are open-loop numbers — feedback removes most of them. What survives feedback is the\n"
      "0.21-0.29 m of steady tracking error measured on real left arcs, and that is the residue of\n"
      "exactly this effect.")
```

The numbers are absurd on purpose: tens of metres. That is what the geometry alone would do over a long
bend, and it is why no lane-keep system is open loop. But it also shows why the feedforward matters so
much — feedback that has to remove tens of metres of modelling error is feedback with nothing left over
for the road.


## Summary

1. $\kappa = \tan\delta / L$, $R = 1/\kappa$.
2. $\mathrm{SWA} = \mathrm{steer\_sign}\cdot\delta\cdot\mathrm{steer\_ratio}$.
3. ICR explains the geometry; slip breaks the no-slip assumption at speed.
4. **Verified against the car**: exact below 9 m/s, 0.80 at 12–15, 0.54 at 21–26 — and worse than a
   textbook understeer gradient at every speed. Believe the model where it was checked, not everywhere.

```{admonition} Exercise
:class: tip
1. For $L=2.64$ m, $R=50$ m: compute $\delta$ [deg] and $|\mathrm{SWA}|$ with ratio $15.7$.
2. Modify the Python `bicycle_step` loop: what $y$ do you get after 2 s if $\delta=0$? (Should stay ~0.)
3. Convince yourself from the ICR figure that the bottom-left angle equals $\delta$.
```

<!-- next-chapter -->
---

**Next:** [Pure Pursuit](./PurePursuit.md)
