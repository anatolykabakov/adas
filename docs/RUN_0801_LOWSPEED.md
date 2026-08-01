# Before stopping 27:00–29:05 (run 2026_08_01_01_14_22)

Segment marked by driver as "did not stay close to lane center". HCA on in two
windows: 1626–1651 s (v ≈ 15 m/s) and 1711–1741 s (v ≈ 13 m/s).

Plots: `docs/mpc_img/58_run0801_lowspeed_a.png`, `docs/mpc_img/59_run0801_lowspeed_b.png`

## Measurements

| metric | 1626–1651 | 1711–1741 | driver (19:22–26:42) |
|---|---|---|---|
| \|lane position\| median | 0.14 m | 0.07 m | 0.12 m |
| p95 | 0.33 m | 0.42 m | 0.36 m |
| systematic offset | +0.09 m | +0.03 m | +0.06 m |
| yaw period | 4.4 s | 6.1 s | **9.6 s** |
| torque, p95 | 212 cNm | 300 cNm | — |

No systematic offset: over whole run with HCA on vehicle stays within ±5 cm
of lane center at any speed (10–14 m/s: −0.02, 18–22: +0.01, 22–26: −0.05). Complaint
is not offset but **yawing**: controller "wanders" with 4–6 s period where
driver wanders ~10 s.

## Cause of yawing

Anchor noise is not the issue: high-frequency (<1 s) component in both plan and lane center
is equally small, σ ≈ 0.015 m. Yawing is low-frequency and comes from the plan itself:

| window | σ vehicle position | σ plan target @10 m |
|---|---|---|
| 1626–1651 | 0.171 m | 0.143 m |
| 1711–1741 | 0.172 m | 0.194 m |

Vehicle honestly follows the plan, and the plan itself drifts. Cause is structural: regression over
2932 frames shows plan mainly **continues current vehicle line**, not
returning to lane center:

```
plan_offset@20 = −0.62·(vehicle position in lane) + 0.08·(lead y) − 0.060   R² = 0.47
```

Coefficient −0.62 means over 20 m plan keeps only ~38 % of current offset. Controller has no
centering of its own: `path_lane_blend_scale = 0`, i.e. lane markings not blended into
anchor (in stock openpilot they always blend, weighted by probability).

## Separately: spikes on launch and before stop

Plot `59_run0801_lowspeed_b.png` shows two spikes unrelated to yawing:

| moment | v | command | what happened |
|---|---|---|---|
| 1708.5 s (launch) | 0.9 m/s | **+20.8°** | torque 300 cNm saturated 2.5 s, rack did not move |
| 1742.5 s (stop) | 1.8 m/s | **−23.7°** | torque ±300, rack swings ±10° |

Mechanism: MPC horizon — 2.5 s, at 1 m/s that is **2.5 meters**. Any lateral offset
on such horizon requires absurd curvature. Stock avoids this because lateral
control is disabled below `MIN_LATERAL_CONTROL_SPEED = 0.3 m/s`, and horizon there is 10 s.

Both commands did not reach bus (`controls_allowed = 0`, Panda blocked 23 TX frames in run) —
but PID integrator wound up, and UI showed `enabled`.

**Done:** gate `min_control_speed_mps = 1.5` (hysteresis 0.5) — below it command zeroed,
planner and PID state reset. Test `LaneKeepServiceMpc.LowSpeedGateHoldsZeroAndHasHysteresis`.

## Recommendations

1. **Done:** minimum speed gate — removes both spikes.
2. **Done:** vehicle model in κ→angle conversion. Closed-loop on this window: centering
   0.22 → 0.18 m median.
3. **Done differently:** `path_lane_blend_scale` raised to **0.6** (2026-08-03) from arc measurement, not
   this window — see `RUN_0802_ARC_OFFSET.md`. Below — rationale at that time.
   Proposed `0.5`: closed-loop on this window
   gives better median and lower spread (0.17 m / p95 0.35 vs 0.18 / 0.39), on long
   highway window — noticeably better p95. Downside: we lack stock weight reduction by
   `laneLineStds` (field not in protocol), so on bad markings blending may
   pull worse than pure plan. Therefore not enabled by default.
