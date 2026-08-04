# Straights and gentle arcs 9:45–13:35 (run 2026_08_01_01_14_22)

Segment driver said "behaves better". HCA on 585–815 s (plus 987–1055 s),
v ≈ 20–23 m/s, highway.

Plot: `docs/mpc_img/60_run0801_straights.png`

## Measurements

| metric | HCA 585–815 | HCA 987–1055 | driver 1162–1602 |
|---|---|---|---|
| v median | 22.6 m/s | 19.9 m/s | 17.8 m/s |
| \|lane position\| median | 0.15 m | 0.09 m | **0.12 m** |
| p95 | 0.63 m | 0.55 m | 0.36 m |
| systematic offset | −0.01 m | +0.04 m | +0.06 m |
| yaw period | 9.5 s | 8.4 s | 9.6 s |
| \|ΔSWA\| per frame | 0.5 °/s | 0.4 °/s | 0.6 °/s |
| driver on wheel | 9 % | 3 % | 31 % |

First run where lateral control on highway matches human: centering
0.09–0.15 m vs 0.12 m driver, yaw period same (8.4–9.5 s vs 9.6 s),
command calmer than human steering (0.4–0.5 vs 0.6 °/s). "Got better"
feeling confirmed: removed 0.83 s lag (see `FRAME_DT_FIX_0801.md`) — exactly what
gave late corrections before.

Tail p95 0.55–0.63 m vs 0.36 m driver — only noticeable gap. Breakdown
shows large deviations coincide with segments where model plan itself drifts
(σ plan target @10 m = 0.17 m vs σ vehicle position 0.28 m), i.e. model behavior again,
not the loop.

## Time-step fix verification on device

New field `control/lane_keep_debug.frame_dt_ms`:

* median **86.8 ms**, p10 63.8, p90 104.9 — estimate alive, no zeros;
* matches actual `vision/lanes` rate (11.2 Hz), i.e. planner uses real
  step, not hardcoded 20 Hz.

Pipeline latency from timestamps on this run:

| link | median | p90 |
|---|---|---|
| frame capture → inference end | 62 ms | 86 ms |
| inference → command publish | 7 ms | 12 ms |
| command → actual rack angle | 40 ms | — |
| rack angle → vehicle yaw rate | 120 ms | — |
| **total capture → vehicle response** | **≈230 ms** | — |

## Recommendations

1. Keep `fp` as default controller: on this road type it already matches driver.
2. Vehicle model (`lat_use_vehicle_model`) helps most on highway: closed-loop on
   segment 1360–1520 s — centering 0.34 → 0.26 m median, p95 1.24 → 0.79 m.
3. Feedforward `fp_steer_delay_s` kept at 0.35 s: sweeps showed reducing to measured
   0.23 s worsens (0.29 → 0.41 m) because lead must cover own
   vehicle delay (120 ms), not just transport delay.
4. Tail p95 limited by model plan quality. To reduce — look at
   lane blending (`path_lane_blend_scale`, see `RUN_0801_LOWSPEED.md`) or
   newer supercombo version.
