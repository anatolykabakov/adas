# Camera Calibration

Supercombo's **input warp** depends on camera extrinsics (roll / pitch / yaw and mounting position).
A few degrees of pitch error do not "look dramatic" in the HUD, but they bias the network input and show up as **systematic CTE** — the car sits habitually left or right of center even when the controller is healthy.

This is the Phone ADAS analogue of AAD's vanishing-point calibration chapter: we still care about extrinsics, but the main live path is the **model odometry head** rather than a chessboard every drive.

Camera RPY for Supercombo is **not** vehicle ENU pose. The same `rpy_deg` prior also seeds
phone IMU mount rotation — see [Localization](../Localization/Overview.md).

```{admonition} This chapter is the map; the next one is the mechanism
:class: note
[Three calibrations, not one](./IntrinsicsAndWarp.md) works through the intrinsics measurement, the online
vanishing-point estimator and the model warp with runnable code and the measured numbers. Read this page for
why calibration shows up as a control symptom, and that one for how each piece actually works.
```

## Prior (shipped config)

`config.json` → `calibration.camera`:

* `position_m` — camera in **ISO vehicle** frame (see [Coordinates](../Vision/Coordinates.md));
* `rpy_deg` — roll, pitch, yaw prior.

Example rig numbers (order of magnitude): ~0.1 m left of centerline, pitch $\approx -1.8°$, yaw $\approx 0.5°$. These are **starting guesses**, not ground truth forever.

## Online calibration

`CameraCalibService` consumes `model/camera_odometry` (and optionally lane UV) and publishes `calibration/camera`.
On success / sufficient confidence, Java updates the warp used by the next Supercombo inputs.

```{admonition} Roll is weakly observable
:class: warning
Roll often stays near $0$ in live estimates. Treat large roll swings as suspicious; do not "tune LK" to compensate a broken roll state.
```

## What a degree of error actually costs

The reason extrinsics matter more than they look is pure geometry: an angular error becomes a lateral
error that grows with distance. A yaw error $\Delta\psi$ puts a point at range $d$ off by
$d\tan\Delta\psi$, and a pitch error $\Delta\theta$ moves the apparent ground intersection, which for a
camera at height $h$ looking at range $d$ scales as roughly $d^2\Delta\theta / h$.

```python
import math

H_CAM = 1.10          # camera height, m
FY = 995.2            # focal length in pixels, from the chessboard measurement

def lateral_error_from_yaw(d_m, yaw_err_deg):
    """A yaw error rotates the whole scene: lateral error grows linearly with range."""
    return d_m * math.tan(math.radians(yaw_err_deg))

def range_error_from_pitch(d_m, pitch_err_deg, h_m=H_CAM):
    """A pitch error moves where the ground appears to be: error grows with range squared."""
    return d_m * d_m * math.radians(pitch_err_deg) / h_m

print(f"{'range':>7} {'yaw 1.5 deg':>13} {'pitch 0.5 deg':>15}")
for d in (10, 20, 30, 50):
    print(f"{d:>5} m {lateral_error_from_yaw(d, 1.5):>11.2f} m "
          f"{range_error_from_pitch(d, 0.5):>13.2f} m")

# And the other direction: how many pixels is a degree?
print(f"\n1 deg of pitch = {math.radians(1.0) * FY:.0f} px of vertical shift in the source frame")
```

Run it: 1.5° of yaw is 0.52 m of lateral error at 20 m — which is three times the 0.15 m the controller
is aiming for. And 1.5° is not a hypothetical. Between two consecutive night runs the learned yaw moved
from +1.67° to +0.10° because the phone was remounted, and the pitch from −1.11° to −0.67°.

```{admonition} Consequence for comparing runs
:class: warning
The learned RPY goes into the **input warp**, so the network literally sees a different image. Two runs
with different learned yaw produce different model plans on the same road. Any cross-run comparison of
plan offset is comparing two perception configurations, not two controllers — a trap this project fell
into before noticing.
```

## Persisting the estimate is not the fix it looks like

The online estimate converges and then is thrown away at shutdown, so the obvious move is to save it and
use it as next drive's prior. Measured, that is nearly worthless: the mount is not repeatable, so a
prior from the previous mount is no better than the default. What matters instead is **how fast the
estimate converges**, and there the news is good — on the 08-06 run it settled within 30–60 s, and the
lateral offset on straights during the first 30 s was 0.02 m.

```python
# Convergence as the online estimator actually behaved, from the bag: a first-order approach to the
# true angle. The question a student should ask is not "what is the final value" but "how much distance
# do I cover before it is good enough".
def converge(prior_deg, truth_deg, tau_s, t_s):
    return truth_deg + (prior_deg - truth_deg) * math.exp(-t_s / tau_s)

prior, truth, tau = 1.67, 0.10, 15.0
print(f"{'t':>5} {'yaw est':>9} {'lateral err at 20 m':>21}")
for t in (0, 5, 15, 30, 60):
    est = converge(prior, truth, tau, t)
    print(f"{t:>4}s {est:>8.2f}° {lateral_error_from_yaw(20.0, est - truth):>19.2f} m")
```

So the practical rule is the boring one: **do not judge lane-keep in the first minute**, and do not chase
a lateral bias measured there.

## Causal chain (bias → CTE)

1. Mount / prior error → wrong warp.
2. Wrong warp → biased lane / plan laterals.
3. Biased path → nonzero CTE on a visually "straight" road.
4. Feedback fights a constant offset → extra SWA activity, still a mean bias.

Before raising Pure Pursuit or MPC gains, compare **prior vs live RPY** in the bag and check `path_camera_offset_m`.

## Practice

* Do not judge lane-keep quality in the first minutes after a cold calib start.
* Persistent lateral bias on a straight **without** HCA engaged is a calib / offset candidate, not a `pp_k_dd` candidate.
* Chessboard intrinsics (`camera_calib_chessboard.py`) refine $f_x,f_y$; extrinsics remain a separate story.

## What bags can and cannot measure

Verified on three 2026-08-01 runs (phone remounted between first and second).

| parameter | from bag | in config | takeaway |
|---|---|---|---|
| **roll** | ≈0° (≤0.2°) | 0.0 | leave roll at 0 |
| **y (lateral)** | +0.14…+0.18 m | 0.10 | IQR large — prefer tape + `path_camera_offset_m` |
| **x (forward)** | not geometric | 1.5 | tape only; `fp` does not use it |
| **height / scale** | 0.80–0.98 | 1.1 m | unstable; use wheel speed, not model `trans[0]` |
| pitch / yaw | live calib | prior −1.8 / +0.5 | works after remount |

## Exercise

1. On a bag, plot live pitch/yaw vs prior.
2. On a straight HCA-off window, estimate mean lane offset.
3. Hypothesize whether offset is better explained by calib, `path_camera_offset_m`, or plan bias — and what measurement would falsify each hypothesis.

```{admonition} Discussion — vs AAD VP calibration
:class: note
AAD teaches yaw/pitch from vanishing points of lane lines in a notebook. We still recommend reading that intuition. In the phone stack the continuous path is **odometry from Supercombo** plus a prior in `config.json`, because we already run the multitask network every frame.
```

<!-- next-chapter -->
---

**Next:** [Three calibrations, not one](./IntrinsicsAndWarp.md)
