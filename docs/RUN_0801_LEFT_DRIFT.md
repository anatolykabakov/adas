# Left drift on runs 13_40_48 / 14_13_03 (2026-08-01)

> **Clarified 2026-08-03.** Here plan offset was computed "20 m ahead", but in an arc lane center itself shifts toward turn by ½κd² — up to 1.9 m at 25 m. Must measure **at the vehicle**
> (extrapolating lines to x=0). After correct measurement plan offset was not constant
> left bias but a function of curvature: `50.2·κ − 0.005 m`, i.e. near zero on straights (−0.02 m), in
> arcs +0.32 / −0.35 m inside the turn. Value `path_camera_offset_m` justified below
> compensates constant part and does not work in arcs — see `RUN_0802_ARC_OFFSET.md`.

First runs with build where κ→angle uses vehicle model. Driver reported
drift; measurement confirmed — drift **left** up to 0.8 m, driver took over.

`2026_08_01_13_40_48`: 27.5 min, HCA on 11 % (windows 237–251, 774–824, 855–899, 1345–1358 s).
`2026_08_01_14_13_03`: 11 min, entirely manual (reference material).

## Symptom

| mode | lane position (+ left of center) | plan offset from center @20 m |
|---|---|---|
| HCA, all windows | **+0.21 m** (p95 0.66) | −0.26 m |
| manual, same run | +0.03 m | −0.11 m |
| previous run 01_14_22, HCA | −0.01 m | −0.06 m |

In window 774–824 s monotonic slide visible: +0.17 → +0.22 → +0.29 → +0.38 → +0.59 → **+0.79 m**
over 45 s, then driver intervention. Confirmed by frames: at 780 s car between lines,
at 823 s — noticeably closer to left.

## Checked and ruled out

* **Camera calibration.** Between runs yaw went −0.55° to +1.03° (phone remounted).
  But perceived lane heading on straights is +0.03…+0.11° on all three runs, so
  calibration correctly tracked new mount and does not skew the scene.
* **Sign and magnitude.** Metric "lane position" built from near lane midline and does not
  depend on line order; frames give same estimate (≈0.6 m left at pos ≈ +0.6…+0.8).
* **Command.** On straights SWA median +0.25° vs rack +0.20° — no constant "cranking" steering,
  car actually follows offset path.

## Cause: amplified model's own left bias

Model plans left of its own lane marking center, and offset depends on vehicle position.
Regression on manual frames (two independent runs):

| run | relation | own centering ability |
|---|---|---|
| 01_14_22 | plan_off = +0.51·x − 0.095 | 49 % |
| 13_40_48 | plan_off = +0.55·x − 0.097 | 45 % |

where `x` — vehicle offset from lane center (right +). Loop goes where plan points,
and plan depends on where vehicle is — positive feedback with equilibrium

```
x_eq = b / (1 − k) = −0.096 / (1 − 0.53) = −0.21 m
```

**Prediction −0.216 m, measured −0.21 m.** Model's own bias (−0.096 m) amplified
twofold by the loop.

### Why it surfaced only now

On run 01_14_22 same plan bias existed (−0.095), but κ→angle was kinematic, and
car achieved only 60 % of curvature. Loop did not reach plan equilibrium and accidentally sat
near center (−0.01 m). Vehicle model fix removed tracking error — and
exposed offset that was in the anchor all along.

Same explains my earlier mistake: stock `CAMERA_OFFSET` first added from
shadow metric, then reverted to 0 because "on HCA car is centered anyway". Center was
artifact of under-steer.

## Fix

`vehicle.path_camera_offset_m = 0.09` — path shift right by measured own bias
(stock flowpilot `CAMERA_OFFSET = 0.08`, we get 0.096 from regression).

Open-loop replay window 765–835 s (same road, same recording, only shift changes):
command becomes smoother **0.47° right** (p05..p95 0.45..0.54), see
`docs/mpc_img/61_leftdrift_before_after.png`.

Closed-loop sweep same window (metric — offset from lane center, from vehicle center):

| configuration | \|center\| med | p95 | offset |
|---|---|---|---|
| kinematics (pre vehicle-model build) | 0.22 m | 0.87 | −0.01 m |
| as in run (vehicle model, no shift) | 0.18 m | 0.79 | +0.05 m |
| + shift 0.09 | 0.18 m | 0.79 | −0.05 m |
| + shift 0.09 and lane blend 0.5 | 0.19 m | 0.79 | −0.13 m |

> **What closed-loop sim cannot do here.** It replays **recorded** plan: model reaction
> to new vehicle position (that +0.53·x loop) is not reproduced because that would require
> re-running network on redrawn frames. So −0.21 m equilibrium does not appear in
> simulator, and its offset (+0.05) reflects only direct shift action, not
> amplification. Proof of mechanism — analytical (regression + fixed point) plus
> road measurement; early version of this report wrongly claimed sim reproduces
> defect: compared metrics differing by `cam_y` = 0.10 m.

## Proposed next steps

1. **Road verify `path_camera_offset_m = 0.09`** — expect equilibrium shift from −0.21 m
   to about −0.05 m.
2. **`path_lane_blend_scale = 0.5`** more justified than before: gives loop
   its own centering instead of relying on 45–50 % plan ability.
   With shift gives offset +0.06 m. Previous risk: we lack stock
   weight reduction by `laneLineStds`.
3. Value 0.09 should be rechecked on new runs: from two runs on one road,
   and model own bias may depend on marking type and lead presence
   (lead y coefficient in regression was +0.08).
