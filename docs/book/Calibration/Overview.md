# Camera Calibration

Supercombo's **input warp** depends on camera extrinsics (roll / pitch / yaw and mounting position).
A few degrees of pitch error do not "look dramatic" in the HUD, but they bias the network input and show up as **systematic CTE** — the car sits habitually left or right of center even when the controller is healthy.

This is the Phone ADAS analogue of AAD's vanishing-point calibration chapter: we still care about extrinsics, but the main live path is the **model odometry head** rather than a chessboard every drive.

Camera RPY for Supercombo is **not** vehicle ENU pose. The same `rpy_deg` prior also seeds
phone IMU mount rotation — see [Localization](../Localization/Overview.md).

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

**Next:** [Latency and thermal](../Latency/Overview.md)
