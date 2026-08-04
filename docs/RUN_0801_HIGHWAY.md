# One hour on highway under control 21:00–21:55 (run 2026_08_01_20_55_15)

Most representative run to date: 58 minutes recorded, of which **35.6 minutes with HCA on**
(23 segments longer than 20 s), 22.5 minutes above 15 m/s. Controller `fp`, camera
remounted to `y = 0`, build with vehicle model and real frame step.

Driver: "good on straights and in arcs". Numbers confirm — measured below.

## Run summary

| metric | value |
|---|---|
| \|lane position\| median (v > 15 m/s, both lines) | **0.16 m** |
| p95 / p99 / max | 0.53 / 0.78 / 1.14 m |
| systematic offset | **−0.11 m** (right of center) |
| speed under control | median 21.4, p95 24.3, max 26.9 m/s |
| HCA torque | median 64 cNm, p95 247, saturation (> 290) **3.5 %** frames |
| lateral acceleration | p50 0.17, p95 1.01, p99 1.75, max 3.22 m/s² |
| driver on wheel when on | **3.2 %** time |
| steering rate \|ΔSWA\| | 0.62 °/s (driver on same road 1.11 °/s) |
| `status` ≠ `ok` when on | 0 % |
| `frame_dt` | median 88 ms, p10 81, p90 103 (inference 47 ms) |
| both lines + plausible width | 59 % frames |

Comparison with human on same road in same run (manual segments, v > 15 m/s):
assistant 0.16 m median vs 0.17 m driver. Driver sample short (2.3 min vs
23.8) and its p95 tail 1.11 m includes lane changes, so conclusion cautious: **for centering
on straight highway assistant is already no worse than human, with twice calmer steering**.

## Breakdown by curvature

| segment | frame share | \|pos\| median | p95 | a_lat median / max |
|---|---|---|---|---|
| straight, R > 1000 m | 76 % | **0.14 m** | 0.45 | 0.11 / 0.69 m/s² |
| R 400–1000 m | 21 % | **0.21 m** | 0.64 | 0.69 / 1.46 m/s² |
| R 200–400 m | 1.8 % | **0.19 m** | 0.56 | 1.43 / 2.67 m/s² |
| R < 200 m | 0.9 % | **0.59 m** | 0.98 | 2.05 / 3.21 m/s² |

Gentle highway arcs (R from 200 m) hold like straights — main difference from
daytime run analyses. Degradation starts below R ≈ 200 m.

**Worst point of run:** t ≈ 819 s, R ≈ 97 m at 18 m/s (lateral accel 3.2 m/s²) —
1.01 m departure outside arc. Same limit as `RUN_0801_CURVE.md`, but over 35 minutes
control it appeared once.

## What the pipeline shows

**Frame step stabilized.** 88 ms median at p10 81 / p90 103 — jitter three times lower than
before real step switch (42–170 ms, see `FRAME_DT_FIX_0801.md`). Inference 47 ms,
enough time margin.

**Residual 18 % under-steer, mostly on actuator.** Cross-correlation over whole run:

| link | lag | corr | actual/command slope |
|---|---|---|---|
| angle command → actual steering angle | 0.14 s | 0.981 | **0.89** |
| curvature command → vehicle yaw rate | 0.22 s | 0.968 | **0.82** |

Of 18 % curvature shortfall 11 % is steering itself (angle-PID, friction, torque cap), and
only remaining ~8 % — vehicle model. Closed loop on lane markings compensates, does not affect
centering, but tightening arcs hits actuator first.

**Offset changed sign.** Previously left drift (+0.1 m, `RUN_0801_LEFT_DRIFT.md`), now
−0.11 m right. Camera remounted to `y = 0`, compensating `path_camera_offset_m = 0.08`
remained. 11 cm — mount accuracy order (±5 cm) plus offset itself, so candidate to
reduce to 0.04–0.05 exists, but one run is not enough to change parameter: need another run
with same camera position.

## Second run and empty folder

`2026_08_01_21_53_29` — 3.6 minutes, three short engagements at 10 m/s: \|pos\| median 0.23 m,
offset −0.07 m, `frame_dt` 90 ms. Confirms operation at lower speed bound,
no separate conclusions. `2026_08_01_20_38_01` — empty folder.

## Recommendations

1. Leave as is: this run's configuration is the working point. No `status ≠ ok`,
   no `tx_overflow`, `heartbeat_lost` = 0 (`tx_blocked` = 82 frames over 58 minutes).
2. `path_camera_offset_m`: collect another highway run and if −0.1 m reproduces,
   reduce offset to 0.04–0.05.
3. Arcs below R ≈ 200 m remain outside applicability — speed limit from curvature ahead first
   (`RUN_0801_CURVE.md`), before more aggressive controller tuning.
