# Lateral control applicability limits

All measured on runs 2026-07-31 / 2026-08-01 (Golf 7 MQB, supercombo 0.8.13, `fp` controller).
Main numeric source — evening highway run `2026_08_01_20_55_15`: 58 minutes recorded,
35.6 minutes under control (2026-08-01 highway bag). This document answers "where the assistant
works normally and where it cannot be relied on".

## Summary

| condition | status | evidence |
|---|---|---|
| Highway, both lane lines visible, 15–27 m/s | **works** | 35.6 min control: centering 0.16 m median / p95 0.53 vs 0.17 m driver on same road |
| Highway arcs R ≥ 200 m | **works** | 0.19–0.21 m median, torque saturated 3.5 % frames |
| One line / lane marking lost | limited | follows model plan: offset up to 0.2 m, yawing period 4–6 s |
| Arc R < 200 m (a_lat > 2 m/s²) | **degrades** | 0.59 m median, p95 0.98; at R ≈ 97 m / 18 m/s departure 1.0 m |
| Arc R < 100 m at 12 m/s | **cannot hold** | HCA torque saturated 52 % time, outward departure up to 0.9 m |
| Speed < 1.5 m/s | disabled | gate: horizon collapses to 2 m, command becomes absurd |
| Intersections, turns, exits | **not supported** | model plan is for lane keeping |
| Narrowings, merges, chevron markings | blending disabled | width filter 2.6–4.6 m (width itself filtered) + weight by line σ |
| City, parked cars | **not designed** | no free-space detection or avoidance |

## Measured quantitatively

### Anchor availability

Both host lines with probability > 0.5 and plausible width:

| run | frames | both lines | width ok | blending available |
|---|---|---|---|---|
| 20_55_15 (highway, 58 min) | 38 279 | 61 % | 86 % | **59 %** |
| 13_40_48 (highway + city) | 14 066 | 54 % | 92 % | **53 %** |
| 14_13_03 (highway) | 6 460 | 78 % | 99 % | **78 %** |
| 01_14_22 (mixed) | 24 809 | 43 % | 77 % | **39 %** |

The rest of the time only the model plan anchors, with centering ability
only 45–50 % and systematic left bias ~0.1 m (plan / camera offset).

### Actuator limit

HCA torque capped at 300 cNm (MQB ceiling, same as openpilot). Measured:

* normal highway driving (35.6 min, `20_55_15`) — median 64 cNm, p95 247, saturation
  3.5 % frames: enough margin, lateral accel p95 only 1.0 m/s²;
* arc R ≈ 97 m at 18 m/s (3.2 m/s²) in same run — only departure to 1.0 m in the whole run;
* arc R ≈ 41 m at 12.3 m/s (3.7 m/s²) — saturation 52 % frames, car departs outward
  0.55–0.94 m, driver intervenes.

Practical threshold: **lateral acceleration above ~2 m/s² already gives noticeable departure, above ~3 m/s²
the assistant cannot hold the arc**. At 22 m/s that is radii about 240 and 160 m, at 12 m/s — 70 and 50 m.
openpilot comfort limit (2.5 m/s²) corresponds to R ≈ 190 m at 22 m/s.

### Speed

* below **1.5 m/s** — lateral control disabled by gate (otherwise 2.5 s horizon becomes
  2 meters and command reaches 20° steering at walking speed);
* verified on road in **10–27 m/s** (in `20_55_15` under control up to 26.9 m/s,
  median 21.4);
* above 27 m/s not tested; curvature shortfall grows with speed (at 22 m/s car gives 0.61 of kinematic), vehicle-model recalculation accounts for this but torque margin is smaller.

### Rate and latency

* vision: median **88 ms** (11.4 Hz), p10 81, p90 103 — after switching to real frame step
  jitter three times lower than before (p90 was up to 170 ms);
* full path capture → vehicle response ≈ 230 ms;
* planner uses real frame step, but below ~6 Hz horizon
  is sampled coarsely (16 nodes over 2.5 s) — degradation not tested.

### Model geometry accuracy

Model metric scale is unstable: 0.80–0.98 across three estimates (visual odometry, lane
width, curvature). In practice lateral quantities may be ~10 % low,
curvature equally high. Model speed (`trans[0]`) is 5–20 % low and
not used for control. See book [Calibration](book/Calibration/Overview.md).

### What remains imperfect

* **18 % under-steer.** Over the highway run actual curvature = 0.82 of commanded, of
  which 0.89 is steering itself (angle-PID, friction, torque cap) and only ~8 % vehicle
  model. Does not affect centering — loop is closed on lane markings — but on tighter
  arcs the actuator hits first.
* **−0.11 m offset right.** After moving camera to `y = 0` compensating
  `path_camera_offset_m` stayed at 0.08, hence the remainder. 2026-08-02 reduced to
  **0.05**; verify on first run — equilibrium should be within ±5 cm with
  and without lane blending.

## What the system lacks entirely

* **Lateral obstacle reaction** — no free-space detection or avoidance;
  only protection against "drifting off" is requiring two lines for lane blending.
* **Interchanges, exits, intersections** — model plan not designed for them; on lane split
  width filter disables blending, but the plan itself may go to the adjacent lane.
* **Lane change** — still no `DesireHelper`; turn signal is now decoded but
  used only to silence LDW.

## Done 2026-08-02

* **Speed limit from curvature ahead** — `long_plan` computes κ on segment from 0.5·v to 4·v
  meters ahead and caps `v_target` for `a_lat ≤ 1.8 m/s²`. First of items below;
  not verified on road yet, execution only via cruise buttons.
* **Limit indication** — when torque at ceiling longer than ~0.5 s app shows
  `STEERING LIMIT`; previously visible only in logs.
* **Lane weight by uncertainty** — `laneLineStds` added to protocol and Java parser;
  blending now fades smoothly from σ 0.3 m to 1.5 m instead of step at `prob ≥ 0.3`.
  Threshold 0.2/0.8 was tuned on straights and suppressed lane markings in arcs where they matter most — see
  arc-offset bags.

## How to safely expand applicability further

1. Arcs: verify speed limit from curvature on road, then discuss tighter
   turns.
2. City: need free corridor (drivable space) and at least parked-car
   detection — neither exists now.
