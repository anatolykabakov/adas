# Localization: GPS, phone IMU, and the EKF

Lane-keep lives in the **car frame**. Map pose, bags overlaid on OSM, and some debug
trajectories live in a **local ENU plane**. This chapter is how the phone builds that plane:
project GPS, calibrate the phone gyro, fuse with the bicycle model.

Code anchors:

* `GpsLocalProjector` — lat/lon → East/North meters
* `ImuCalibrator` / `ImuCalibService` — phone IMU → vehicle yaw rate
* `OnlineLocalizer` / `VehicleEKF` / `LocalizationService` — fuse → `localization/pose`

Camera RPY for Supercombo warp is a **different** story ([Calibration](../Calibration/Overview.md));
it only shares the mount prior with the IMU and optionally feeds cam-odo yaw rate into the EKF.

## Pipeline

```text
Android GPS / IMU
    → sensors/gps/location (lat, lon) , sensors/imu
TopicConvert
    → GpsLocalProjector → GpsSample (x East, y North)
    → sensors/imu_raw
ImuCalibService
    → sensors/imu_yaw   (yaw_rate in vehicle frame)
LocalizationService  (tick on vehicle/chassis)
    ← chassis + GPS ENU + imu_yaw (+ optional cam-odo)
    → localization/pose
```

Feature flags: `nodes.localization`, `nodes.imu_calib` (if omitted, IMU calib follows localization).

## Frames (do not mix)

| frame | axes | used for |
|---|---|---|
| **Local ENU** | $x$ East, $y$ North; origin = first GPS fix | `localization/pose`, GPS samples after convert |
| **GPS bearing** | degrees **clockwise from North** (Android) | course → ENU yaw |
| **Bicycle / vehicle** | planar yaw in ENU; $\delta$ at front axle | EKF predict |
| **ISO vehicle** | $y$ **left+** | camera mount prior |
| **Device / Supercombo** | $y$ **right+** | lanes / cam-odo (see [Coordinates](../Vision/Coordinates.md)) |

```{admonition} Same topic name, two payloads
:class: warning
Java publishes `sensors/gps/location` as **lat/lon proto**. After `TopicConvert`, the **same topic
name** carries a typed `GpsSample` in ENU meters. Offline tools must know which side of convert
they read.
```

---

## 1. GPS → local meters

Origin $(\phi_0,\lambda_0)$ = first valid fix. Earth radius $R = 6371000$ m:

$$
\begin{aligned}
\Delta\phi &= (\phi-\phi_0)\tfrac{\pi}{180},&
\Delta\lambda &= (\lambda-\lambda_0)\tfrac{\pi}{180},\\
y &= R\,\Delta\phi && \text{(North)},\\
x &= R\,\Delta\lambda\cos\phi_0 && \text{(East)}.
\end{aligned}
$$

Bearing (deg from North, CW) → ENU yaw (from East, CCW):

$$
\psi = \mathrm{normalize}\Big(\tfrac{\pi}{2} - b_{\deg}\tfrac{\pi}{180}\Big).
$$

Course is trusted only when $v > 2$ m/s (and not the standstill quirk $v<0.5$ with near-zero bearing).
Then $v_x = v\sin b$, $v_y = v\cos b$.

```python
import math
import numpy as np

R_EARTH = 6_371_000.0

def enu_from_ll(lat_deg, lon_deg, lat0_deg, lon0_deg):
    dlat = math.radians(lat_deg - lat0_deg)
    dlon = math.radians(lon_deg - lon0_deg)
    y = dlat * R_EARTH                          # North
    x = dlon * math.cos(math.radians(lat0_deg)) * R_EARTH  # East
    return x, y

def yaw_enu_from_bearing_deg(bearing_deg):
    return (math.pi / 2 - math.radians(bearing_deg) + math.pi) % (2 * math.pi) - math.pi

# ~111 m North, ~71 m East at φ₀=55°
print(enu_from_ll(55.001, 37.001, 55.0, 37.0))
print("bearing 0° (N) → yaw", round(math.degrees(yaw_enu_from_bearing_deg(0.0)), 1), "° (should be +90)")
print("bearing 90° (E) → yaw", round(math.degrees(yaw_enu_from_bearing_deg(90.0)), 1), "° (should be 0)")
```

Flat-Earth is fine for a drive of a few km. Crossing the origin reset (app restart) starts a **new** plane — do not stitch bags without re-anchoring.

---

## 2. Phone IMU calibration

The phone is glued at a weird angle. Raw gyro is in **phone axes**; the bicycle needs
**vehicle yaw rate** $\dot\psi$.

### Lock procedure (`ImuCalibrator`)

1. Need chassis speed. **Stationary:** $v < 0.5$ km/h.
2. **Quiet:** $\big|\|a\| - g\big| \le 1.5$ m/s² and $\|\omega\| \le 0.08$ rad/s.
3. Collect ≥ 50 accel + gyro samples →
   * orientation $R$: rotate measured gravity onto vehicle **down** $(-Z)$;
   * bias $b$: mean gyro.
4. While still quiet: bias EMA with $\alpha = 0.02$.
5. Output (default `invert_yaw_rate=true` on Android):

$$
\dot\psi = \pm \big(R(\omega - b)\big)_z.
$$

### Mount prior

If `calibration.camera.rpy_deg` is set, the same roll/pitch/yaw seeds

$$
R = R_z(\mathrm{yaw})\,R_y(\mathrm{pitch})\,R_x(\mathrm{roll})
$$

and the service can publish yaw rate **before** a standstill lock (`ready = has_prior ∨ orientation_locked`).

```python
import numpy as np

g = 9.81

def rotation_from_gravity(accel):
    """Map measured gravity direction onto vehicle +down (-Z). Toy: Rodrigues via two vectors."""
    a = np.asarray(accel, dtype=float)
    a = a / np.linalg.norm(a)
    target = np.array([0.0, 0.0, -1.0])  # down in vehicle
    v = np.cross(a, target)
    c = float(np.dot(a, target))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))

def yaw_rate_from_gyro(gyro, R, bias, invert=True):
    w = R @ (np.asarray(gyro) - np.asarray(bias))
    return float(-w[2] if invert else w[2])

# Phone nearly upright, slight pitch: gravity mostly in phone -z
R = rotation_from_gravity([0.2, 0.0, -g])
bias = np.array([0.01, -0.02, 0.005])
gyro = bias + np.array([0.0, 0.0, 0.12])   # +0.12 about phone z after bias
print("R:\n", np.round(R, 3))
print("yaw_rate", round(yaw_rate_from_gyro(gyro, R, bias), 4), "rad/s")
```

Until `ready`, there is **no** `sensors/imu_yaw` — localization falls back to chassis yaw rate (if any) or pure bicycle.

```{admonition} Drive-off tip
:class: tip
Sit still a few seconds after ignition so orientation can lock. A bad prior + no lock → wrong sign
or huge bias → EKF yaw-rate gate rejects IMU (`|ω − v tanδ / L| < 0.35` rad/s).
```

---

## 3. Fusion (EKF + bicycle)

State $[x,\ y,\ \psi,\ v,\ \dot\psi]^\top$ in ENU. Tick on **chassis** (~10 ms after CAN RX fix).

### Predict

Same kinematics as [Bicycle](../Control/BicycleModel.md):

$$
\dot\psi_{\mathrm{pred}} = \frac{v\tan\delta}{L},\qquad
\begin{aligned}
x &\leftarrow x + v\cos\psi\,dt,\\
y &\leftarrow y + v\sin\psi\,dt,\\
\psi &\leftarrow \psi + \dot\psi_{\mathrm{pred}}\,dt.
\end{aligned}
$$

$L$ from `vehicle.wheelbase_m` (≈ 2.636 m). A parallel **pure odom** trajectory (no GPS) is kept for debug.

```{admonition} Heading is mostly bicycle
:class: warning
Predict **writes** $\dot\psi$ from $\delta$. IMU only weakly corrects heading through the filter
(~tens of percent per step with current $Q$/$R$). Without GPS course, heading drifts with
understeer / steer-offset error — see [`PARAMSD.md`](../../PARAMSD.md).
```

### IMU update

If calibrated yaw rate agrees with bicycle within **0.35** rad/s (or $v < 0.5$): scalar update on
$\dot\psi$ with $R \approx 0.05^2$.

### Optional cam-odo

`model/camera_odometry` `rot[2]` if `rot_std` small, $v>1$, same agree gate.

### GPS update

Age gate ≈ 2.5 s; rate limit ≈ 0.2 s.

* Position: innovation gate 25 m (reseed 50 m).
* Course valid: hard snap if $|\Delta\psi|>0.5$ rad, else soft yaw update; velocity update.

```python
import math

L = 2.636

def bicycle_step(x, y, yaw, v, delta, dt):
    yaw_rate = v * math.tan(delta) / L if abs(v) > 0.01 and abs(delta) > 1e-3 else 0.0
    x = x + v * math.cos(yaw) * dt
    y = y + v * math.sin(yaw) * dt
    yaw = yaw + yaw_rate * dt
    return x, y, yaw, yaw_rate

def gps_accept(innov_m, soft=25.0, hard=50.0):
    if innov_m > hard:
        return "reseed"
    if innov_m > soft:
        return "reject"
    return "update"

# 1 s open-loop at 15 m/s, 2° wheel
x = y = yaw = 0.0
for _ in range(10):
    x, y, yaw, r = bicycle_step(x, y, yaw, 15.0, math.radians(2.0), 0.1)
print(f"odom after 1 s: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.2f}°  r={r:.3f} rad/s")

# GPS jump
for dx in (5.0, 30.0, 60.0):
    print(f"innov {dx:.0f} m → {gps_accept(dx)}")
```

### Output

`localization/pose`: fused $x,y,\psi,v,\dot\psi$ plus odom_/ekf_ traces for PlotJuggler.

---

## 4. How this relates to lane-keep

Lateral MPC / PP **do not** need global ENU to steer — they track `vision/path` in the device frame.
Localization still matters for:

* bag maps / mapmatch overlays ([`MAPMATCH.md`](../../MAPMATCH.md) is a **separate** offline track-shape matcher);
* sanity checks (GPS vs odom divergence);
* any future map-relative features.

Wrong IMU sign will not immediately invert HCA if lane-keep ignores pose — but it **will** poison
any consumer of `localization/pose` and confuse offline forensics.

---

## Config checklist

| key / node | role |
|---|---|
| `nodes.localization` / `nodes.imu_calib` | enable services |
| `vehicle.wheelbase_m` | bicycle $L$ |
| `calibration.camera.rpy_deg` | IMU mount prior (and camera warp prior) |
| (code defaults) | GPS $R_{\mathrm{pos}}{=}0.5$ m, update every 0.2 s, cam-odo on |

---

## Exercise

1. Project two GPS fixes 1° apart in latitude at your city φ; compare $y$ to $111$ km.
2. Convert bearings $\{0,90,180,270\}$ → ENU yaw; sketch on paper.
3. On a bag: time from start until `sensors/imu_yaw` appears; correlate with speed≈0 windows.
4. Plot `localization/pose` vs pure odom on a highway bag; where does GPS snap heading?

<!-- next-chapter -->
---

**Next:** [Camera calibration](../Calibration/Overview.md)
