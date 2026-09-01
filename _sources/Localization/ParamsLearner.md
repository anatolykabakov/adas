# Learning the car's parameters — paramsd, the road's bank, and what the measurement said

Everything so far assumed the car's numbers — steering ratio, tyre stiffness, the steering zero — are
known. They are not, and they drift. This chapter is upstream's `paramsd` ported and measured: the
parameters become states of the filter, the road's bank becomes an input it needs, and the drive tells us
which of the assumptions survived. It closes the Localization part with the config checklist and the
exercises for all its steps.

## Step 5: what is still missing


* **Road roll exists now; grade does not.** `RoadRollEstimator` reads it from the lateral accelerometer —
  `sin(φ) = (a_y − f_y)/g` with `a_y = −v·yaw_rate` — subtracts the measured +3 °/g of suspension roll, and
  publishes it on `localization/pose` with its own uncertainty: 2.2° per sample, 0.65° after ten seconds of
  averaging, and no better than that because the residual is the road's camber changing. Gate on
  `road_roll_std_deg`, which starts at the same 10° `paramsd` uses for "no usable roll". Longitudinal grade
  is the same construction with `f_x` and is not built.

  Two things that cost time and are worth carrying: the sign of `chassis.yaw_rate` is ISO (z up,
  left-positive) while the estimator frame is z down, and getting it backwards reports a body-roll gradient
  of 116 °/g rather than an error; and the standstill orientation lock used to overwrite the mount heading
  with a gravity-derived rotation, which is fine for yaw rate and destroys anything lateral.
* **The wheel-speed scale is still hand-entered.** GPS velocity is used and speed is a state, and that was
  not enough (step 4): the frequent biased measurement wins between GPS samples, and no noise assumption
  separates a constant bias from the truth. The scale needs a state of its own.
* **Learned vehicle parameters** — built, and the result is step 6 below. It is not in this list of gaps
  because it is no longer one: the port exists, it is measured, and the measurement is why it ships disabled.

The dependency order is worth stating because it is easy to get backwards: GPS velocity into the filter →
an orientation and roll estimate → `paramsd`. Porting `paramsd` first leaves its roll input pinned at
zero, which works but gives up the one thing that separates road bank from vehicle understeer. That order
was followed, and step 6 is what happened when the roll input it insisted on met the error budget it had to
fit inside.

---

```{figure} figures/understeer_speed.png
---
width: 75%
---
The achieved-vs-commanded curvature ratio slides with speed, so no single `tire_stiffness_factor` fits —
the reason paramsd exists (and why it needs more than one observable).
```

## Step 6: learning the parameters, and what the measurement said about it

Everything so far estimated *where the car is*. This step estimates *what the car is* — and it is in this
chapter, not in the lateral-control one, because the estimator's inputs are the localizer's inputs and its
one non-obvious input is the road bank we built in step 5.

It is also the most honest thing in the book, because it ends with the feature switched off.

### The premise, and how to check a premise

Here is the reasoning that started the work. Comparing the curvature the steering angle implies against the
curvature the yaw rate reveals, on one real drive:

| speed | $\kappa_{\text{fact}}/\kappa_{\text{kin}}$ |
|---|---|
| 5–9 m/s | 0.89 |
| 9–12 | 0.84 |
| 12–16 | 0.84 |
| 16–21 | 0.65 |
| 21–26 | 0.58 |
| 26–32 | 0.46 |

The car turns steadily less than kinematics predicts as it goes faster. `tire_stiffness_factor` is one
number in a config file. One number cannot be 0.89 and 0.46. Therefore, the reasoning goes, it must become a
state — which is what upstream's `paramsd` does.

Every step of that is true except the "therefore". The constant does not carry the speed dependence; the
*model around it* does. Recall from the lateral-control chapter:

$$\kappa = \frac{\tan\delta}{L\,(1 - \text{slip}\cdot v^2)}$$

with $\text{slip} < 0$ for an understeering car, so the denominator grows with $v^2$ and the same steering
angle buys less curvature at speed. That is exactly the observed effect. So the question is not "does the
ratio fall with speed" — it does — but "does it still fall after the model has had its say".

```python
import numpy as np

WHEELBASE_M = 2.636
C2F_FRAC = 0.45
MASS_KG = 1533.0


def slip_factor(tsf: float) -> float:
    """Transcribed from utils/vehicle_model.h. Negative for an understeering car."""
    civic_mass, civic_wb = 1326.0 + 136.0, 2.70
    civic_c2f = civic_wb * 0.4
    a_f = WHEELBASE_M * C2F_FRAC
    a_r = WHEELBASE_M - a_f
    c_f = 192150.0 * tsf * MASS_KG / civic_mass * (a_r / WHEELBASE_M) / ((civic_wb - civic_c2f) / civic_wb)
    c_r = 202500.0 * tsf * MASS_KG / civic_mass * (a_f / WHEELBASE_M) / (civic_c2f / civic_wb)
    return MASS_KG * (C2F_FRAC * c_f - (1.0 - C2F_FRAC) * c_r) / (WHEELBASE_M * c_f * c_r)


# The measured ratio of actual to kinematic curvature, at the mid-speed of each band.
bands = np.array([7.0, 10.5, 14.0, 18.5, 23.5, 29.0])
ratio_kin = np.array([0.89, 0.84, 0.84, 0.65, 0.58, 0.46])

# What the model already explains, with the shipped constant.
sf = slip_factor(0.64)
model_gain = 1.0 / (1.0 - sf * bands ** 2)
ratio_model = ratio_kin / model_gain

print(f"slip_factor(0.64) = {sf:.6f}")
for v, rk, rm in zip(bands, ratio_kin, ratio_model):
    print(f"  {v:4.1f} m/s   fact/kin {rk:.2f}   fact/model {rm:.2f}")
print(f"  spread: fact/kin {ratio_kin.max() - ratio_kin.min():.2f}, "
      f"fact/model {ratio_model.max() - ratio_model.min():.2f}")
```

The spread collapses from 0.43 to 0.16, and what remains has no monotone trend. (That run uses each band's
mid-speed; computed per sample off the bag the residual ratios are 0.95, 0.94, 0.99, 0.87, 0.89, 0.80 — same
picture.) A flat ~10 % shortfall is not a speed-dependent stiffness.
It is a scale error, and a scale error is one number, which is exactly what a constant can be.

That did not stop the port — a filter that reports its own uncertainty is still worth having, and the 10 %
still wants explaining. But it changes what success would look like.

### The estimator

Three states, and the measurement is the yaw rate:

$$
x = [\text{tsf},\ \text{steer ratio},\ \text{angle offset}], \qquad
\dot\psi_{\text{pred}} = v\,\kappa(x) + \frac{g\sin\phi}{v}
$$

Two details carry most of the value.

**Use the controller's own curvature function.** Not a re-derivation of it, the same function. A parameter
learned against a slightly different model than the one that consumes it is worse than a hand-tuned
constant, because it looks principled.

**Differentiate it numerically.** Three states at CAN rate cost nothing, and the analytic derivative through
`slip_factor` — which divides by products of stiffnesses that themselves depend on the state — is the kind
of expression that is wrong for a month before anyone notices. The measurement function is the single source
of truth; a central difference cannot disagree with it.

```python
G = 9.81
STEER_SIGN = -1.0     # positive CAN angle is a LEFT turn on this car; this frame is z-down


def predict_yaw(x, v, swa_deg, roll_deg):
    """Yaw rate in the z-down frame. The controller's own model, evaluated at the state x."""
    tsf, ratio, offset = x
    v = max(v, 1e-3)
    delta = STEER_SIGN * np.radians(swa_deg - offset) / max(ratio, 1e-3)
    kappa = np.tan(delta) / (WHEELBASE_M * (1.0 - slip_factor(max(tsf, 1e-3)) * v * v))
    return v * kappa + G * np.sin(np.radians(roll_deg)) / v


class ParamsLearner:
    def __init__(self, ratio_std=0.1, use_roll=False):
        self.x = np.array([0.64, 15.7, 0.0])
        self.p = np.array([0.5 ** 2, ratio_std ** 2, 1.0 ** 2])
        self.q = np.array([0.005 ** 2, 0.00005 ** 2, 0.02 ** 2])
        self.p_max = np.array([0.5 ** 2, max(ratio_std, 0.3) ** 2, np.inf])
        self.lo = np.array([0.2, 12.0, -10.0])
        self.hi = np.array([5.0, 20.0, 10.0])
        self.r = 0.02 ** 2
        self.use_roll = use_roll
        self.n = 0

    def update(self, v, swa_deg, yaw_rate_can, roll_deg, dt):
        if v < 5.0 or abs(swa_deg) > 45.0:
            return
        yaw = -yaw_rate_can                       # ISO in (left-positive), z-down inside
        roll = roll_deg if self.use_roll else 0.0
        self.p = self.p + self.q * dt

        h = np.zeros(3)
        eps = np.array([1e-4, 1e-3, 1e-3])
        for i in range(3):
            xp, xm = self.x.copy(), self.x.copy()
            xp[i] += eps[i]
            xm[i] -= eps[i]
            h[i] = (predict_yaw(xp, v, swa_deg, roll) - predict_yaw(xm, v, swa_deg, roll)) / (2 * eps[i])

        k = (self.p * h) / (float(h @ (self.p * h)) + self.r)
        self.x = np.clip(self.x + k * (yaw - predict_yaw(self.x, v, swa_deg, roll)), self.lo, self.hi)
        self.p = self.p * (1.0 - k * h)
        self.p = np.minimum(self.p, self.p_max)   # a ceiling, not a pseudo-measurement — see below
        self.n += 1

    def valid(self):
        """Count, sigma, AND bounds. Each catches what the others miss."""
        return (self.n >= 500 and np.sqrt(self.p[0]) < 0.15
                and bool(np.all(self.x > self.lo + 1e-9)) and bool(np.all(self.x < self.hi - 1e-9)))
```

### The trap upstream's code sets for you

`paramsd` keeps its slow parameters' uncertainty from growing on long straights, and writes it as *observe
the state with its own current value, at high noise*. Copy that literally and you have a bug. The innovation
$z - x$ is identically zero, so the update cannot move the estimate — it can only shrink the covariance.
Applied once per localizer message it is a harmless bound. Applied on every CAN sample at 100 Hz it shrinks
the covariance a hundred times a second while no information arrives, the real measurement loses all gain,
and the filter freezes near wherever it started. Measured on the synthetic recovery test: a true stiffness
of 0.45 came out as 0.515, pinned by the 0.64 it began at — and `valid()` read the shrunken sigma as
convergence after ten minutes of straight road.

So it is a ceiling: sigma may never exceed a bound, and only real measurements may reduce it. That is the
`np.minimum` line above.

### Two failures the synthetic tests could not have found

Both were found by replaying the shipped filter over a recorded drive, which is the argument for building
the replay harness at the same time as the filter.

**The steering sign.** The port negated the yaw rate on the way in — ISO to z-down, correct — and never
applied this car's `vehicle.steer_sign`, which the controller had always applied. On this car positive CAN
angle is a *left* turn: regressing the measured ISO yaw rate against the kinematic prediction gives a slope
of $+0.824$ at a correlation of $0.987$ over 28 636 cornering samples. So the prediction opposed the
measurement on every corner.

**And `valid()` said yes anyway.** The filter ran to whichever bounds shrink the predicted magnitude —
stiffness on its 0.200 floor, steer ratio on its 20.0 ceiling, identical in all four quarters of the drive.
A saturated state has stopped moving, so its sigma is small, so it looks exactly like convergence. The
`valid()` of the day checked the stiffness bounds and not the ratio's, and let it through.

```python
def drive(learner, corners, truth_tsf=0.64, truth_ratio=15.7, sign=STEER_SIGN):
    """Feed steady corners generated by a known car, in the ISO convention the CAN decoder produces."""
    for swa, v in corners:
        delta = sign * np.radians(swa) / truth_ratio
        kappa = np.tan(delta) / (WHEELBASE_M * (1.0 - slip_factor(truth_tsf) * v * v))
        yaw_iso = -v * kappa                      # z-down truth back to ISO, as ESP_02 reports it
        for _ in range(60):
            learner.update(v, swa, yaw_iso, 0.0, 0.01)


corners = [(swa, v) for _ in range(12)
           for swa, v in [(6, 12.0), (-6, 12.0), (12, 15.0), (-12, 15.0),
                          (20, 22.0), (-20, 22.0), (3, 25.0), (-3, 25.0)]]

good = ParamsLearner()
drive(good, corners)
print(f"right sign: tsf {good.x[0]:.3f}  ratio {good.x[1]:.2f}  valid {good.valid()}")

# The same data, the filter's model given the wrong steering sign, and the loose ratio prior it shipped with.
flipped = ParamsLearner(ratio_std=0.5)
drive(flipped, corners, sign=-STEER_SIGN)
print(f"wrong sign: tsf {flipped.x[0]:.3f}  ratio {flipped.x[1]:.2f}  valid {flipped.valid()}")
print(f"            sigma {np.sqrt(flipped.p[0]):.3f} — small, because saturation looks like convergence")
```

Note that `valid()` above reports `False` for the flipped run — that is the *fixed* check, the one that
tests every state against its bounds. The check of the day tested only the count and the sigma, and the
third `print` is why that passed: the sigma of the saturated stiffness is 0.010, comfortably under the 0.15
threshold, because a state that has stopped moving has a small covariance.

So bounds are not a safety net added for tidiness. They are the only thing that distinguishes a saturated
estimate from a converged one, and a validity check that omits any state's bounds has a hole in it exactly
where the worst failure lives.

### Why the road bank is not used, after all that work

Step 5 built a road-bank estimate specifically because a banked road and an understeering car produce the
same lateral-acceleration signature, and `paramsd` cannot separate them without it. That argument is sound.
It is also, on this hardware, irrelevant — and the arithmetic that shows it is two lines.

```python
for v in (10.0, 15.0, 25.0):
    print(f"{v:5.1f} m/s: 1 deg of bank = {G * np.sin(np.radians(1.0)) / v:.4f} rad/s of yaw")
print("flat-model yaw residual on the real drive: 0.0065 rad/s")
print("bank estimate accuracy from a windscreen phone, at its floor: 0.65 deg")
```

One degree of bank at 15 m/s is 0.0114 rad/s — nearly twice the entire residual the flat model already has.
Our bank estimate is good to 0.65° after ten seconds of averaging and no better, because past that the
residual is the road's camber genuinely changing. So the input carries more error than the error it was
meant to remove. Feeding it in raised the residual from 0.0065 to 0.0104 rad/s, a 60 % degradation, and
`params_use_roll` ships `false`.

This is worth sitting with. The dependency argument — "the learner needs roll, so build roll first" — was
correct about interfaces and silent about accuracy. Whether $B$ is good enough for $A$ has an answer only
after $B$ exists and is measured against $A$'s error budget. Here the budget was 0.0065 rad/s and the input
carried 0.0114.

### What the drive actually said

| configuration | learned tsf | drift across quarters | yaw residual vs shipped constants |
|---|---|---|---|
| ratio prior 0.5, bank fed in | 0.374 | — | **3 % worse**, and `valid()` said yes |
| ratio prior 0.1, bank fed in | 0.517 ± 0.026 | 0.462 → 0.368 | 0 % |
| ratio prior 0.1, flat road | 0.342 ± 0.022 | 0.411 → 0.360 | 14 % worse |
| best single constant, 0.55 | — | — | 4.1 % better |

The degeneracy is the story of the first two rows. Stiffness and steer ratio both scale predicted curvature,
and over one drive's speed range their signatures differ too little for a yaw-rate measurement to separate
them — so with a loose prior the filter slides along the ridge rather than picking a point on it. Tightening
the prior to 0.1 is not timidity: a steering rack's ratio is a mechanical fact known to a few percent, and
that knowledge belongs in the prior. **A state the data cannot separate from another state does not become
observable by being declared a state.**

And the last row is the humbling one. The best single constant on this drive is 0.55 against the shipped
0.64, worth 4.1 %, and the curve around it is flat: 0.50 → 0.00636, 0.55 → 0.00625, 0.64 → 0.00652 rad/s.
The shipped hand-tuned number was already nearly optimal.

### So it ships behind two flags, both off

`localization.learn_vehicle_params` runs the estimator. `lane_keep.use_learned_params` lets the controller
read it. Separate, on purpose: the learner can publish for an entire drive while the controller keeps its
constants, which is how a learned value earns the right to be used. One flag for both would make the first
drive that tests the estimator also the first drive that trusts it.

Even with both on, the controller takes the estimate only while `learned_params_valid` holds, and falls back
to the configured constants the moment it does not — so losing validity walks the parameters back rather
than freezing them at whatever the estimator last believed.

That is the whole feature: built, tested, instrumented, published, replayable, and disabled. A negative
result you can point at with numbers is a finished piece of work.

---

## How this relates to lane-keep


The lateral MPC and Pure Pursuit **do not** need global ENU to steer — they track `vision/path` in the
device frame. Localization matters for:

* bag maps and track overlays in the visualizer (the offline map-match vertical was deleted on
  2026-08-21; the `.admap` reader survives as the visualizer backdrop);
* sanity checks — GPS against pure odometry divergence;
* any future map-relative feature;
* learning the vehicle parameters the controller uses — step 6, and the one consumer that turned out not to
  want the roll estimate after all.

A wrong IMU sign will not invert the steering while lane-keep ignores pose — but it will poison every
consumer of `localization/pose` and quietly corrupt offline forensics.

---

## Config checklist


| key / node | role |
|---|---|
| `nodes.localization` / `nodes.imu_calib` | enable services |
| `vehicle.wheelbase_m` | bicycle $L$ |
| `vehicle.wheel_speed_factor` | wheel-radius correction (1.0 shipped, 0.988 measured) |
| `vehicle.speed_accel_process_noise` / `speed_measurement_noise` | speed filter tuning |
| `calibration.camera.rpy_deg` | IMU mount prior (and camera warp prior) |
| (code defaults) | GPS $R_{\text{pos}} = 0.5$ m, update every 0.2 s, camera odometry on |

---

## Exercises


1. Project two GPS fixes 1° apart in latitude at your city's $\phi$; compare $y$ with 111 km. Then do it
   in longitude and explain the $\cos\phi_0$.
2. Convert bearings $\{0,90,180,270\}$ to ENU yaw and sketch them. Now deliberately use $b - 90°$ instead
   of $90° - b$ and see what a plot of a real drive looks like.
3. Derive $K_2$ for the yaw-rate update analytically, then run the covariance recursion numerically for
   200 steps with $Q_{44}=0.05^2$, $R=0.02^2$. Explain the factor of seven.
4. In the arc simulation, sweep `r_model` from $0.02^2$ to $1.0^2$ and plot the final heading error. At
   what point does the model stop mattering, and what have you given up when it does?
5. Add a GPS course update every 100 steps to the old (`as_state=False`) filter and show that the heading
   error stops growing. This is the masking effect — reproduce it, then remove GPS and watch it return.
6. On a bag: measure the time from start until `sensors/imu_yaw` first appears, and correlate it with the
   windows where speed is about zero.
7. Take a bag, feed `v_ego_raw` through `SpeedFilter` with `factor = 1.0` and `0.988`, and compare the
   integrated distance against the GPS track length. Which factor gets you closer?

## On your own recording

Everything above runs unchanged on a bag you record yourself (see
[Bags](../Logging/Bags.md), *Record your own bag*) — and gains from it: your GPS has *your* multipath
and your tunnels, your IMU carries the phone's real mounting, and every claim is checked against a
track you can verify from memory. Load your topics and walk the five steps again:

```python
# not-runnable — the loading pattern; the estimator code is the chapter's own
from pathlib import Path
from vis.bag_io import load_topic_messages

session = Path("../adas_logs/<your-session>")
gps = load_topic_messages(session, "sensors/gps/data")
imu = load_topic_messages(session, "sensors/imu")
print(len(gps), "GPS fixes,", len(imu), "IMU samples")
```

Two expectations to calibrate first: a phone GPS in the open is a few metres of scatter at 1 Hz — good
enough to bound drift, useless for lane-level anything. And a passenger recording has no wheel
odometry, so step 1 runs on GPS-differenced speed instead — the substitution this chapter already
discusses when it makes speed a state. If you recorded on foot, the "vehicle frame" assumption is
gloriously violated, and the yaw-lock step shows exactly how the calibrator copes, or refuses.

<!-- next-chapter -->
---

**Next:** [Camera calibration](../Calibration/Overview.md)
