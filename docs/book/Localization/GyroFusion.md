# Gyro fusion — the phone's gyroscope and the mistake worth studying

Wheel speed and a steering angle gave the cheapest heading estimate — and the previous chapter ended with
its limit: the bicycle model does not know what the tyres actually did. The phone carries a gyroscope that
measures the yaw rate directly. This chapter makes it usable (bias, mount orientation, the yaw that gravity
cannot see) and fuses it into the filter — and then studies the one mistake in that fusion carefully,
because it is the kind that looks like it works.

## Step 2a: making the phone gyro usable


The phone is clamped at whatever angle the mount allows. Raw gyro is in **phone axes**; the bicycle
model needs **vehicle yaw rate** $\dot\psi$.

### Lock procedure (`ImuCalibrator`)

1. Need chassis speed. **Stationary:** $v < 0.5$ km/h.
2. **Quiet:** $\big|\|a\| - g\big| \le 1.5$ m/s² and $\|\omega\| \le 0.08$ rad/s.
3. Collect ≥ 50 accel and gyro samples →
   * orientation $R$: rotate measured gravity onto vehicle **down** $(-Z)$;
   * bias $b$: mean gyro.
4. While still quiet: bias EMA with $\alpha = 0.02$.
5. Output (default `invert_yaw_rate=true` on Android):

$$
\dot\psi = \pm \big(R(\omega - b)\big)_z.
$$

Note what this procedure can and cannot give you. Gravity fixes two of the three angles — roll and
pitch of the phone relative to the car. **Yaw is not observable from gravity at all**, which is why the
mount prior exists and why the camera and the IMU share it.

### Mount prior

If `calibration.camera.rpy_deg` is set, the same roll/pitch/yaw seeds

$$
R = R_z(\mathrm{yaw})\,R_y(\mathrm{pitch})\,R_x(\mathrm{roll})
$$

and the service can publish yaw rate **before** a standstill lock (`ready = has_prior ∨
orientation_locked`).

```python
import numpy as np

g = 9.81

def rotation_from_gravity(accel):
    """Rotate the measured gravity direction onto vehicle +down (-Z), Rodrigues from two vectors."""
    a = np.asarray(accel, dtype=float)
    a = a / np.linalg.norm(a)
    target = np.array([0.0, 0.0, -1.0])
    v = np.cross(a, target)
    c = float(np.dot(a, target))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))

def yaw_rate_from_gyro(gyro, R, bias, invert=True):
    w = R @ (np.asarray(gyro) - np.asarray(bias))
    return float(-w[2] if invert else w[2])

R = rotation_from_gravity([0.2, 0.0, -g])       # nearly upright, slight pitch
bias = np.array([0.01, -0.02, 0.005])
gyro = bias + np.array([0.0, 0.0, 0.12])
print("R =\n", np.round(R, 3))
print("yaw_rate =", round(yaw_rate_from_gyro(gyro, R, bias), 4), "rad/s")
```

Until `ready`, there is **no** `sensors/imu_yaw` and localization falls back to the chassis yaw rate, or
to pure bicycle if there is none.

```{admonition} Drive-off tip
:class: tip
Sit still for a few seconds after ignition so orientation can lock. A bad prior plus no lock gives a
wrong sign or a huge bias, and then the EKF's agreement gate rejects the IMU outright
($|\omega - v\tan\delta/L| < 0.35$ rad/s) — the filter goes quiet exactly when you needed it.
```

---

```{admonition} If you have not met a Kalman filter
:class: note
From here the estimator is an **EKF**, in two half-steps per tick. **Predict:** push the state forward
with the motion model and *grow* its uncertainty (covariance $P$). **Update:** compare a sensor reading
to what the state predicts (the *innovation*), and correct the state by a fraction of that gap — the
*gain* $K$, which is large when the sensor is trusted more than the prediction and small when less. "EKF"
just means the models are non-linear and are linearised (a Jacobian $F$) at each step. You do not need
the derivation to follow this chapter — only: predict grows uncertainty, a measurement shrinks it, and
the gain is how much you believe the measurement.
```

## Step 2b: fusing the gyro, and the mistake worth studying


State $[x,\ y,\ \psi,\ v,\ \dot\psi]^\top$ in ENU. Ticks on **chassis**, roughly every 10 ms.

### Predict

$$
\begin{aligned}
x &\leftarrow x + v\cos\psi\,dt,\\
y &\leftarrow y + v\sin\psi\,dt,\\
\psi &\leftarrow \psi + \dot\psi\,dt,\\
v &\leftarrow v_{\text{measured}}.
\end{aligned}
$$

The interesting question is: **which $\dot\psi$?**

### What the filter used to do, and why it was wrong

The original code advanced heading with the bicycle model, $\dot\psi_{\text{pred}} = v\tan\delta/L$, and
then **overwrote the yaw-rate state** with that same value. Every step. So whatever the gyro had taught
the filter was thrown away before the next measurement arrived, and the measurement could only reach
heading through the cross-covariance $P_{\psi\dot\psi}$ in a single step.

How much is "only through the cross-covariance"? It is easy to guess $dt$ and be wrong — I did. Work it
out: the yaw-rate update has gain $K_4 = P_{44}/(P_{44}+R)$, and heading receives
$K_2 = P_{24}/(P_{44}+R)$. With $F_{24} = dt$ the predicted cross term is $P_{24} \approx dt\,P_{44}$, so

$$
K_2 \approx dt\,\frac{P_{44}}{P_{44}+R} = dt\,K_4 .
$$

That is the wrong answer, and the reason is that $P_{44}$ is itself shrunk by the previous update. Run
the recursion numerically with the real $Q_{44} = 0.05^2$ and $R_{\text{imu}} = 0.02^2$ and the
steady-state weight is $dt\,(1-K_4) \approx 0.12\,dt$ — so heading came out as

$$
\psi \approx 0.88 \cdot (\text{bicycle}) + 0.12 \cdot (\text{measured}).
$$

**Simulate before you assert.** The analytic hand-wave and the recursion disagree by a factor of seven.

That split would be fine if the bicycle model were right. On this car it is not: the measured understeer
transfer is $\kappa_{\text{fact}}/\kappa_{\text{kin}} = 0.54$ at 22 m/s, so the kinematic prediction
over-turns by $1/0.54 = 1.85$, and heading rotated about **1.75× faster than truth**. On a 30-second arc
at 0.15 rad/s that is 450° instead of 258°.

Why did no run ever show it? Because `updateGpsYaw` hard-snaps the heading whenever the GPS course
disagrees by more than 0.5 rad. The filter was being dragged back into line by GPS several times a
minute, and the logs looked healthy. **A defect masked by a correction is still a defect** — it surfaces
the moment the correction is unavailable, which for GPS means tunnels, urban canyons, and cold starts.

### What it does now

Yaw rate is a state with its own random walk, heading integrates the state, and the bicycle model enters
as a *weak measurement* with $R_{\text{model}} = 0.15^2$ against the gyro's $0.02^2$. Weak because
0.15 rad/s is roughly how far apart the two are at 20 m/s and 10° of wheel. So:

* the gyro is trusted, and its correction survives to the next step;
* the model still fills gaps — if the IMU goes invalid while CAN keeps reporting steering, heading keeps
  moving instead of freezing;
* model updates are counted separately (`model_update_count`) so the diagnostics still say which sensor
  produced the heading.

`setYawRateIsAState(false)` restores the old behaviour exactly, which is how the two were compared.

### Reproduce both, in twenty lines

This is the whole chapter in one runnable block: drive a steady arc where the gyro tells the truth and
the steering angle claims 1.85× as much, and watch the heading.

```python
import math
import numpy as np

L, DT = 2.636, 0.01
UNDERSTEER = 0.54          # measured kappa_fact / kappa_kin at 22 m/s

class Ekf:
    """[x, y, yaw, v, yaw_rate] in ENU. yaw_rate_is_state=False is the old behaviour."""

    def __init__(self, yaw_rate_is_state=True, r_model=0.15**2, r_imu=0.02**2):
        self.x = np.zeros(5)
        self.P = np.diag([1.0, 1.0, 0.05**2, 0.5**2, 0.05**2])
        self.Q = np.diag([0.1**2, 0.1**2, 0.01**2, 0.5**2, 0.05**2])
        self.as_state, self.r_model, self.r_imu = yaw_rate_is_state, r_model, r_imu

    def predict(self, v_meas, steer, dt):
        x, y, yaw, v = self.x[0], self.x[1], self.x[2], self.x[3]
        model_rate = v_meas * math.tan(steer) / L if abs(steer) > 1e-3 and abs(v_meas) > 0.01 else 0.0
        rate = self.x[4] if self.as_state else model_rate

        self.x[0] = x + v * math.cos(yaw) * dt
        self.x[1] = y + v * math.sin(yaw) * dt
        self.x[2] = yaw + rate * dt
        self.x[3] = v_meas
        if not self.as_state:
            self.x[4] = model_rate                       # <-- the mistake

        F = np.eye(5)
        F[0, 2], F[0, 3] = -v * math.sin(yaw) * dt, math.cos(yaw) * dt
        F[1, 2], F[1, 3] = v * math.cos(yaw) * dt, math.sin(yaw) * dt
        F[2, 4] = dt
        self.P = F @ self.P @ F.T + self.Q

        if self.as_state and model_rate != 0.0:
            self.observe_yaw_rate(model_rate, self.r_model)   # model as a weak measurement

    def observe_yaw_rate(self, z, r):
        H = np.zeros((1, 5))
        H[0, 4] = 1.0
        S = float((H @ self.P @ H.T).item()) + r
        K = (self.P @ H.T / S).ravel()
        self.x = self.x + K * (z - self.x[4])
        IKH = np.eye(5) - np.outer(K, H)
        self.P = IKH @ self.P @ IKH.T + r * np.outer(K, K)

    def observe_gyro(self, z):
        self.observe_yaw_rate(z, self.r_imu)


def drive_arc(as_state, seconds=5.0, v=22.0, steer=0.04, feed_gyro=True):
    ekf = Ekf(yaw_rate_is_state=as_state)
    ekf.x[3] = v
    model_rate = v * math.tan(steer) / L
    truth_rate = UNDERSTEER * model_rate
    for _ in range(int(seconds / DT)):
        ekf.predict(v, steer, DT)
        if feed_gyro:
            ekf.observe_gyro(truth_rate)
    return math.degrees(ekf.x[2]), math.degrees(truth_rate * seconds), math.degrees(model_rate * seconds)

now, truth, model = drive_arc(as_state=True)
before, _, _ = drive_arc(as_state=False)
print(f"truth heading after 5 s : {truth:6.2f}°   (gyro, the only honest witness)")
print(f"bicycle model would say : {model:6.2f}°   (over-turns by 1/0.54)")
print(f"yaw rate as a state     : {now:6.2f}°   <- tracks the gyro")
print(f"old, overwritten        : {before:6.2f}°   <- tracks the model")

# And the gyro-less case: the model must still drive heading, not freeze it.
no_gyro, _, model5 = drive_arc(as_state=True, feed_gyro=False)
print(f"\nno gyro, as a state     : {no_gyro:6.2f}° against the model's {model5:6.2f}°")
```

Run it and you get 51.7° of truth, 52.4° from the new filter, and 90.2° from the old one chasing the
model's 95.7°. One flag, **38° of heading error in five seconds** — about 0.13 rad/s of error rate, which
over a minute without GPS is the difference between being on your road and being on the next one. The
gyro-less case comes out at 95.2° against the model's 95.7°, so the fallback works: heading keeps moving
rather than freezing.

### Every gate in one table

Measurements are cheap to add and expensive to trust, so each one is gated. These are the numbers as
shipped:

| measurement | source | gate | noise |
|---|---|---|---|
| yaw rate | `sensors/imu_yaw` | agrees with bicycle within 0.35 rad/s, or $v<0.5$ | $R = 0.02^2$ |
| yaw rate | bicycle model | $\vert\delta\vert>0.001$ and $\vert v\vert>0.01$ | $R = 0.15^2$ |
| yaw rate | `model/camera_odometry` `rot[2]` | small `rot_std`, $v>1$, same agreement gate | $R \approx 0.05^2$ |
| position | GPS ENU | innovation < 25 m; reseed above 50 m; 4 consecutive rejects also reseed | $R = 0.5$ m, softened when innovation is large |
| course | GPS bearing | $v>2$ m/s; reject $\vert\Delta\psi\vert>1.2$ rad unless forced; hard snap above 0.5 rad | $R \approx 0.05$ |
| velocity | GPS $v_x,v_y$ | innovation < 15 m/s | $R \approx 1.0$ |

Two things to notice. First, the reseed path exists because a rejected measurement that keeps being
rejected is not noise, it is a filter that has lost the plot — after four rejects it accepts reality
rather than defending its estimate.

Second, the velocity row is the weakest of the six, and step 4 is about why. Its noise was assumed rather
than measured (1.0 against a Doppler that is good to 0.1), and until speed became a state the prediction
overwrote it about twenty times per GPS sample. Both are fixed now, and the scale is *still* not
learnable — see step 4 for the reason, which is more interesting than the bug.

---

```{figure} figures/frames.png
---
width: 60%
---
Three frames the chapter keeps straight: ENU axes, the GPS bearing (clockwise from North), and the
vehicle heading — related by ψ = π/2 − b.
```

<!-- next-chapter -->
---

**Next:** [GPS](./Gps.md)
