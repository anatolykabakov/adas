# Lateral control applicability limits

Measured on the 2026-08-21 run (Golf 7 MQB, Xiaomi 14, supercombo 0.9.7 on the GPU via thneed,
`fp` + acados): 25.7 minutes, 35 118 control ticks. Where that run has no equivalent figure, the
older 2026-08-01 highway run is quoted and marked as such — it was OnePlus 7T with supercombo
0.8.13, so its rates and latencies no longer describe this system, only its road behaviour.

Numbers come from `python3 bag/bag_report.py adas_logs/2026_08_21_14_49_52`; the gates come from
`assets/config.json` and `platform/volkswagen/values.h`. This document answers "where the assistant
works and where it cannot be relied on".

## Summary

| condition | status | evidence (2026-08-21) |
|---|---|---|
| Straight, R > 500 m | **works** | 17 253 ticks: offset +0.036 m median, p90 +0.31; angle error 0.32° median; torque at ceiling 4.2 % |
| Gentle arc, R 167–500 m | **works, less margin** | 1 691 ticks: offset +0.083 m, error 0.74°; torque at ceiling 27.6 %, median 195 cNm |
| Medium arc, R 83–167 m | **cannot hold** | 99 ticks: offset +0.468 m, error 2.86° (p90 9.32°); torque at ceiling 81.8 %, median 300 cNm |
| Arc R < 83 m | **not measured** | the 2026-08-01 run held R ≈ 41 m at 12.3 m/s for 52 % of frames at the ceiling and departed 0.55–0.94 m |
| One line lost / marking absent | limited | both σ below 0.3 m in only 21.8 % of frames; lane recognised on 51.1 % of control ticks, the rest anchors on the model plan alone |
| Speed < 1.5 m/s | disabled | gate `min_control_speed_mps`; 25.2 % of ticks reported `low_speed` |
| Intersections, turns, exits | **not supported** | the model plan is for lane keeping |
| Narrowings, merges, chevrons | blending disabled | width filter 2.6–4.6 m plus weight by line σ |
| City, parked cars | **not designed** | no free-space detection or avoidance |

The R 83–167 m row rests on 99 ticks (31 of them with a recognised lane) — it says the arc is not
held, not how badly.

## Hard gates

Everything the command passes through before it becomes torque. Config keys are live knobs
(`vehicle.*`), the panda values are compile-time.

| gate | value | where |
|---|---|---|
| Angle ceiling | 25° | `mpc_max_steer_deg` |
| Ceiling at a standstill, and its ramp | 8°, +1.0° per m/s | `mpc_low_speed_steer_deg`, `mpc_steer_deg_per_mps` |
| Setpoint rate | 6°/s above 2 m/s | `mpc_rate_limit_deg`, `mpc_rate_min_speed` |
| Per-tick slew | 8° | `steer_slew_limit_deg` |
| Lateral jerk | 5 m/s³ | `mpc_max_lateral_jerk` |
| Actuator delay compensated | 0.35 s | `fp_steer_delay_s` |
| Control speed floor | 1.5 m/s, hysteresis 0.5 | `min_control_speed_mps` |
| Plan age | 0.3 s, then `stale` | `lane_max_age_s` |
| Curvature-ahead speed cap | a_lat ≤ 1.8 m/s², preview 0.5·v … 4·v | `long_plan.curv_a_lat_max` |
| Torque at full command | 300 cNm | `max_torque_cnm`, = panda `STEER_MAX` |
| Torque rate | +4 / −10 cNm per 20 ms frame = 200 / 500 cNm/s | `STEER_DELTA_UP` / `_DOWN` |
| Driver override | 80 cNm allowance, ×3 multiplier | `STEER_DRIVER_ALLOWANCE` |
| MPC horizon | 16 nodes, quadratically spaced over 2.5 s | `LAT_N`, `tNode` |

## Measured quantitatively

### Actuator limit

The ceiling is 300 cNm and the run reaches it, in a pattern that follows radius exactly:

| section | ticks | at ceiling | torque median | a_lat median | angle error median |
|---|---|---|---|---|---|
| straight R > 500 m | 17 253 | 4.2 % | 68 cNm | 0.05 m/s² | 0.32° |
| gentle 167–500 m | 1 691 | 27.6 % | 195 cNm | 0.54 m/s² | 0.74° |
| medium 83–167 m | 99 | 81.8 % | 300 cNm | 0.84 m/s² | 2.86° |

Over the whole run 6.7 % of ticks sit at the ceiling (above 23 km/h, with a target, no turn signal).
Lateral acceleration while saturated is only 0.20 m/s² median (p95 0.98) — the torque is going into
turning the wheel against rack friction, not into holding an arc, which is why saturation appears
long before the tyres are the limit.

From the 2026-08-01 run, whose arcs were sharper: departure becomes noticeable above ~2 m/s² and the
arc is lost above ~3 m/s². At 22 m/s those are radii of about 240 and 160 m.

### Position in the lane

Only where the lane was recognised — 17 960 ticks, 51.1 % of those under control:

| section | ticks | offset median | offset p90 | width median |
|---|---|---|---|---|
| straight R > 500 m | 16 469 | +0.036 m | +0.310 m | 3.08 m |
| gentle 167–500 m | 1 423 | +0.083 m | +0.239 m | 3.12 m |
| medium 83–167 m | 31 | +0.468 m | +0.561 m | 3.03 m |

The car sits left of centre in 67 % of ticks and right in 33 %. A bias that steady is camera
placement or the angle zero, not the controller; `path_camera_offset_m` is at 0.05 after the
2026-08-02 correction, and on straights the remaining offset is 3.6 cm — within the ±5 cm that was
the target. On arcs it is an order of magnitude larger, and that is the controller running wide.

### Speed

* below **1.5 m/s** lateral control is off (hysteresis 0.5 m/s): the 2.5 s horizon collapses to a
  couple of metres and the command becomes absurd. `low_speed` on 8 834 of 35 118 ticks (25.2 %);
* `ok` on 26 277 ticks (74.8 %), `no_polyline` on 7;
* above 27 m/s still not tested.

### Rate and latency

| quantity | median | p95 |
|---|---|---|
| Frame interval | 42.0 ms (23.8 Hz) | 62 ms |
| supercombo inference | 9.7 ms | 17.1 ms |
| Frame → inference done | 15 ms | 36 ms |
| Inference → plan published | 7 ms | 16 ms |
| Frame → plan | **22 ms** | 53 ms (max 295) |

The frame step the controller actually saw was 41.9 ms, i.e. the plan is built on the real spacing
rather than a nominal one. Middleware over the run: nothing dropped in any of the nine services,
backlog present in 7.5 % of stats snapshots (worst offenders `platform` 7 %, `zmq_bridge` 3 %).

The frame step matters beyond scheduling: model velocities are measured between its own two frames
and were trained at 50 ms, so they carry `velocityScale = 50/dt` — at 42 ms that is 1.19. `lead_v`
still reaches FCW/AEB without this correction (known, deliberate).

### Lane-marking confidence

* probability: left 0.90 median, right 0.85;
* σ: left **0.37 m**, right **0.51 m** — the right line is measurably worse;
* both lines below σ 0.3 m in only **21.8 %** of frames, so full-weight blending is the exception,
  not the rule;
* blending fades between `lane_std_good_m` 0.15 and `lane_std_bad_m` 0.30 over `lane_std_range_m`
  20 m, matching what comma's own values imply;
* 7 645 frames had neither host line, and no control status says so — the state reads `ok`
  throughout (task #40). Half the off-centre offset on arcs comes from this σ veto (task #39).

### Model geometry accuracy

Pose against wheel speeds on this run: longitudinal slope **0.825** (1.000 would be a correct
scale, correlation 0.854), yaw slope **−0.882** with correlation −0.946 — the magnitude is close and
**the sign is inverted** (task #37). Lateral quantities are therefore roughly 10–18 % low and
curvature correspondingly high; the vehicle-model recalculation absorbs part of it. The camera-lens
estimate for the Xiaomi 14 and its datasheet value disagree by 11 %, which is not yet resolved with
a chessboard (task #51). See the book, [Calibration](book/Calibration/Overview.md).

### What remains imperfect

* **Torque saturates before the tyres do.** On 167–500 m arcs a quarter of the ticks are at 300 cNm
  while lateral acceleration is half a m/s². Feedforward barely participates: measured on this run
  from `pid_f`, it contributes **1.3 cNm below 8 m/s, 3.2 at 8–15, 4.3 at 15–24** — out of 300. Rack
  friction (comma's learned `frictionCoefficientRaw` 0.192, about 57 cNm) is not compensated at all.
* **The feedforward speed floor is off in the shipped config.** The term is `swa · (v² + v0²) · k_f`,
  and `v0` exists so the term does not vanish at low speed; the code default is 9.8 m/s but
  `assets/config.json` sets `lat_pid_ff_floor_mps: 0.0`. The implied `k_f` recovered from the bag is
  5.63e-05 against the configured 6e-05, which is the arithmetic of `v0 = 0` — so the 2026-08-09
  feedforward change was **not active on this drive**.
* **Both lines are rarely confident at once.** 21.8 % of frames; the rest of the time the model plan
  is the only anchor, and it carries the plan/camera offset with it.
* **No status for lost markings.** The screen and the bag both say `ok` while the lines are gone.

## What the system lacks entirely

* **Lateral obstacle reaction** — no free-space detection or avoidance; the only protection against
  drifting off is requiring two lines for blending.
* **Interchanges, exits, intersections** — the model plan is not designed for them; on a lane split
  the width filter disables blending, but the plan itself may follow the adjacent lane.
* **Lane change** — no `DesireHelper`; the turn signal is decoded but only silences LDW.

## Changed since this was last measured

* supercombo **0.9.7** on the phone GPU through thneed, with `vEgo` fed in and lead output parsed;
  inference went from 17–18 ms to 9.7 and the frame interval from 88 to 42 ms;
* the ZMQ bridge drains its socket (up to 32 messages per tick): the old one-per-tick ceiling was
  100 msg/s and silently killed lateral control a minute into a drive;
* the panda survives the phone sleeping — the descriptor is reseated into the running native layer,
  so paramsd, filters and calibration are not reset;
* `vp` was deleted; `fp` + acados drives and `pp` remains as reference and fallback.

## How to safely expand applicability further

1. Arcs: the curvature-ahead speed cap (`a_lat ≤ 1.8`) is computed but was never verified on the
   road; then loosen the σ veto (task #39) and only then discuss sharper turns.
2. Torque: compensate rack friction before asking for more of the ceiling.
3. City: needs drivable-space detection and at least parked-car detection — neither exists.
