# What camera calibration can be measured from a run, and what cannot

Verified on three runs 2026-08-01 (`01_14_22`, `13_40_48`, `14_13_03`). Between first and second
the phone was remounted — visible in live calibration (yaw −0.55° → +1.03°) and useful as a
natural experiment.

## Summary

| parameter | from measurement | in config | conclusion |
|---|---|---|---|
| **roll** | −0.04° / +0.20° / −0.04° | 0.0 | confirmed, compensation not needed |
| **y (lateral)** | +0.14 / +0.17 / +0.18 m | 0.10 | estimate higher, but large spread — see below |
| **x (forward)** | not extractable | 1.5 | measure with tape; does not affect `fp` |
| **height / scale** | scale 0.80–0.98 | 1.1 m | unstable, no correction applied |
| pitch / yaw | live calibration | prior −1.8 / +0.5 | works: lane heading on straights +0.03…+0.11° |

## Roll — measured well

On flat road all lane points lie in the ground plane, so with roll error φ
the model outputs `z ≈ −y·φ`. Regression z vs y at 5–40 m:

| run | n | dz/dy | roll |
|---|---|---|---|
| 13_40_48 | 6018 | −0.0006 | **−0.04°** |
| 14_13_03 | 3505 | +0.0035 | +0.20° |
| 01_14_22 | 5312 | −0.0007 | **−0.04°** |

Roll under 0.2° in all runs, including after phone remount. So `roll = 0` in
config is correct, and audit item "roll not estimated" is closed: nothing to estimate.
Caveat: road cross-slope (usually 1.5–2 %) would enter this estimate as ~1°; getting
~0 means both roll and slope are small on these roads.

## y — estimable, but with caveat

Method: fit line `c(x) = a + b·x` to lane midline on 0–30 m while driver steering;
intercept `a` — lane center position at camera height.

| run | n | a | p25..p75 |
|---|---|---|---|
| 01_14_22 | 4980 | +0.140 m | +0.03..+0.27 |
| 13_40_48 | 5980 | +0.172 m | +0.05..+0.30 |
| 14_13_03 | 3267 | +0.177 m | +0.05..+0.27 |

Method assumes "on average driver centers in lane" and **does not
separate mount offset from driver habit**: IQR ±0.12 m exceeds the magnitude itself. After remount estimate rose 3 cm — within noise.

Practical conclusion: tape is more reliable; remainder is better removed not by tuning `cam_y_left`, but
with one scalar `path_camera_offset_m` from offset measurement with HCA on — it is expressed
directly in controller coordinates and does not depend on how offset splits between
mount and model plan bias.

## x — not extractable from this data

Idea was to measure forward camera offset via lateral velocity: camera at distance L from rotation axis
in a turn has `v_y = L·ψ̇`. Regression `camera_odometry.trans[1]` vs yaw rate gives
L = 1.0–1.5 m (corr 0.32–0.83), not claimed 2.2 m.

Reason: at speed rotation center does not coincide with rear axle; it shifts forward
from tire slip (in openpilot this is `rotation_radius = wb − c2f − (c2f·m)/(wb·C_r)·v²`, going
to zero around 22 m/s). So what is measured is effective lever at 15–22 m/s,
not geometry. Geometric forward offset must be measured with tape.

Does not affect lateral control with `fp`: it uses only `wheelbase`
and `mpc_Lf`. `x_forward` from config goes to overlay projection and Pure Pursuit shift.

## Scale (indirectly — height) — unstable, no correction applied

Three independent estimates of model metric scale:

| method | 13_40_48 | 14_13_03 | 01_14_22 |
|---|---|---|---|
| visual odometry (`trans[0]` vs v_ego) | 0.954 | 0.811 | 0.801 |
| lane width (if reference 3.5 m) | 0.954 | 0.896 | 0.976 |
| plan curvature vs actual | ~1.00 | — | 0.883 |

Spread 0.80–0.98 too large for a constant factor: part is different roads with different lane widths, part — real visual odometry instability.

Practical consequences:

* **cannot use speed from model** — `trans[0]` is 5–20 % low; for control
  we take wheel speed, correctly;
* lane width reads as 3.14–3.42 m where highway is usually 3.5–3.75 — lateral
  quantities likely ~10 % low, curvature equally high. Partially explains
  why in shadow mode `fp` command was 16–32 % above driver steering;
* to calibrate scale — need segment with exactly known lane width and
  straight geometry, then factor computed in a minute as `3.5 / measured`.

## Live pitch/yaw calibration check

Indirect but convincing: perceived lane heading on straights (|yaw rate| < 0.5 °/s, v > 15)
is +0.03° / +0.05° / +0.11° on three runs. So after phone remount live
calibration converged to new mount and does not skew the scene. Mechanism works.
