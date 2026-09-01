# GPS — local metres, what it buys, and making a correction survive

Odometry drifts; the only absolute reference this phone has is its GPS. This chapter turns latitude and
longitude into metres on a local plane, looks honestly at what a 1–5 m fix can and cannot correct, and works
out why a correction that lands in the filter can still be gone a second later.

## Step 3a: GPS → local meters


Origin $(\phi_0,\lambda_0)$ is the first valid fix. With Earth radius $R = 6\,371\,000$ m:

$$
\begin{aligned}
\Delta\phi &= (\phi-\phi_0)\tfrac{\pi}{180},&
\Delta\lambda &= (\lambda-\lambda_0)\tfrac{\pi}{180},\\
y &= R\,\Delta\phi && \text{(North)},\\
x &= R\,\Delta\lambda\cos\phi_0 && \text{(East)}.
\end{aligned}
$$

Bearing (deg from North, clockwise) → ENU yaw (from East, counter-clockwise):

$$
\psi = \mathrm{normalize}\Big(\tfrac{\pi}{2} - b_{\deg}\tfrac{\pi}{180}\Big).
$$

That $\tfrac{\pi}{2} - b$ is worth burning into memory. Getting it backwards once produced a reported
heading error of 103° in an analysis of a run whose real error was 0.3°, and an hour went into hunting a
filter bug that did not exist.

Course is trusted only above $v > 2$ m/s, and the standstill quirk ($v < 0.5$ with a near-zero bearing)
is rejected. Then $v_x = v\sin b$, $v_y = v\cos b$.

```python
import math

R_EARTH = 6_371_000.0

def enu_from_ll(lat_deg, lon_deg, lat0_deg, lon0_deg):
    dlat = math.radians(lat_deg - lat0_deg)
    dlon = math.radians(lon_deg - lon0_deg)
    y = dlat * R_EARTH                                       # North
    x = dlon * math.cos(math.radians(lat0_deg)) * R_EARTH     # East
    return x, y

def yaw_enu_from_bearing_deg(bearing_deg):
    return (math.pi / 2 - math.radians(bearing_deg) + math.pi) % (2 * math.pi) - math.pi

print(enu_from_ll(55.001, 37.001, 55.0, 37.0))   # ~(64 m East, 111 m North) at φ₀=55°
for b in (0.0, 90.0, 180.0, 270.0):
    print(f"bearing {b:5.0f}° → ENU yaw {math.degrees(yaw_enu_from_bearing_deg(b)):+7.1f}°")
```

Flat Earth is fine for a drive of a few kilometres. Crossing an origin reset (app restart) starts a
**new** plane — do not stitch bags without re-anchoring.

---

## Step 3b: what GPS buys, and what it hides


Measured against GNSS on the night run (23.5 min, 3D fix throughout):

| quantity | agreement |
|---|---|
| heading vs GNSS course | 0.3° median, 5.4° p99 |
| speed vs GNSS | 0.17 m/s median |

Those look excellent, and they are — with the caveat that the heading number is partly a measure of how
often GPS snapped the heading into place, and the speed number carries the wheel-radius bias described
above. A filter compared against the thing that corrects it will always look good.

The honest test of a filter is what it does when a source is missing. That is what the arc simulation
above is for, and it is why the yaw-rate fix mattered even though no logged run complained.

---

## Step 4: making a correction survive, and finding out it still is not enough

Step 3 gave bounded position and heading. It did not give a trustworthy **speed**, and the reason is the
same shape of mistake as step 2 — worth seeing twice, because it is the most common way a Kalman filter
gets quietly disabled.

`updateGpsVel` was being called. But `predict` did `state_(3) = v_measured` on every tick — about every
10 ms — while GPS velocity arrives every 0.2 s. So the correction was assigned over roughly twenty times
before the next one came. The fused speed was the wheel speed, to six decimals.

There was a second, independent reason: the update was given `R = 1.0`, an assumed 1 m/s of Doppler noise.
Measured, Doppler is an order of magnitude better — the residual against scale-corrected wheel speed is
0.066–0.101 m/s, and that includes both sensors and the 1 Hz sampling.

The two reasons interact in a way that is worth predicting before you run the code. In the old,
assigned mode nothing ever shrank the speed variance, so it grew without bound and the GPS gain
approached 1: the update moved the speed *fully* to the GPS value, and the next tick threw it away. Make
speed a state and the variance is now small — so with `R = 1.0` the update barely registers at all. Only
with both fixed does it both land and persist.

Both are fixed: speed is a state (`setSpeedIsAState`), and `gps_vel_R` is $0.1^2$.

```python
# The two failure modes and the fix, in one experiment. True speed 20.0; CAN reports 20.24 (the measured
# 1.2 % scale); GPS velocity reports the truth once every 20 ticks.
class SpeedState:
    """The speed part of the EKF, with the old behaviour available for comparison."""

    def __init__(self, as_state=True, q=0.5**2, r_wheel=0.1**2):
        self.as_state, self.q, self.r_wheel = as_state, q, r_wheel
        self.v, self.p = 20.0, 1.0

    def predict(self, v_wheel):
        if not self.as_state:
            self.v = v_wheel                      # <-- the assignment that erased everything
            self.p += self.q
            return
        self.p += self.q
        k = self.p / (self.p + self.r_wheel)       # wheel speed as a measurement
        self.v += k * (v_wheel - self.v)
        self.p *= 1 - k

    def observe_gps(self, v_gps, r_gps):
        k = self.p / (self.p + r_gps)
        self.v += k * (v_gps - self.v)
        self.p *= 1 - k


def run(as_state, r_gps, ticks=400, every=20):
    f = SpeedState(as_state=as_state)
    right_after = None
    for i in range(ticks):
        f.predict(20.24)
        if i % every == 0:
            f.observe_gps(20.0, r_gps)
            right_after = f.v
    return right_after, f.v

for label, as_state, r_gps in (("assigned, R=1.0  (as shipped before)", False, 1.0),
                               ("state,    R=1.0  (noise still assumed)", True, 1.0),
                               ("state,    R=0.01 (Doppler as measured)", True, 0.1**2)):
    after, end = run(as_state, r_gps)
    print(f"{label}: right after GPS {after:.4f}, one GPS period later {end:.4f}")
```

Read the third line carefully. The correction now lands — and by the time GPS speaks again, the wheel
speed has dragged the estimate back to 20.24.

That is not a tuning failure, it is an identifiability one. The biased measurement arrives twenty times
more often at the same assumed noise, so it wins between samples; and no pair of unbiased-noise
assumptions can separate a **constant bias on the frequent measurement** from the truth. Lowering the
wheel's trust would throw away its genuinely low noise and make the estimate worse everywhere else.

The way out is to stop pretending the bias is noise and give it a state of its own: estimate the scale.
That is exactly what `paramsd` does for the vehicle parameters, and it is why step 5 is where this chapter
stops and that work begins.

```{admonition} The general lesson
:class: tip
Twice in one filter, a measurement was defeated not by bad maths but by an assignment in the prediction
step. If you inherit an estimator and a sensor "seems to have no effect", check first whether something
downstream is writing its state.
```

---

<!-- next-chapter -->
---

**Next:** [Learning the car's parameters](./ParamsLearner.md)
