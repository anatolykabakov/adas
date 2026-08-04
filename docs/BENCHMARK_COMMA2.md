# Benchmark vs comma-two and closing the gap

Compiled 2026-08-03/04 from dragonpilot logs on **the same Golf 7 and roads**, plus our
`bag_arc_offset.py` metrics. Replaces the former `VS_DRAGONPILOT_0803` / `PLAN_TO_COMMA2` pair.

---

## 1. Measurement setup

comma-two + dragonpilot: `dragonpilot_rlog_lite/rlog_lite`, 4 routes, 33 segments, 26.1 min,
42 % usable (`carControl.latActive`, hands off, lanes visible).

Tool: `app/src/main/scripts/rlog_arc_offset.py` (`OPENPILOT_ROOT` / `--op-root`) — same metric
as `bag_arc_offset.py`.

| quantity | source |
|---|---|
| lane center / curvature | `modelV2.laneLines[1,2].y`, fit x∈[0,30], at x=0 |
| model plan | `modelV2.position.y` |
| **reference at vehicle** | **`lateralPlan.dPathPoints[0]`** |
| fast loop | `controlsState.desiredCurvature`, 100 Hz |

Pitfalls: use `latActive` (not `controlsState.active` with `dp_atl`); κ desired is right-positive.

---

## 2. Results (cross-track at vehicle, m; left +)

| segment | | **dragonpilot** | **our ADAS** | manual | **target** |
|---|---|---|---|---|---|
| straight | abs / p90 | **0.08** / 0.23 | 0.12 | 0.16 | **0.10** |
| left arc | abs | **0.07** | **0.51** | 0.17 | **0.20** |
| right arc | abs | **0.20** | **0.71** | 0.15 | **0.20** |

| | comma-two | our ADAS |
|---|---|---|
| tracking error (str / L / R) | +0.00 / +0.03 / −0.08 | +0.07 / +0.35 / −0.23 |
| setpoint offset | ≈ camera offset (−0.06…−0.08) | −0.10 / +0.15 / −0.42 |

---

## 3. Causes

1. **Fast loop** — they run angle PID at 100 Hz; we did ~12.5 Hz. Done: CAN RX 50→10 ms (verified 2026-08-04).
2. **Lane blending** — their reference ≈ lane center when paint is visible; justifies `blend→1.0` after fast loop. No curvature-plan correction upstream.
3. **Lane σ** — theirs ~0.05, ours 0.2–0.9 on arcs. Warp/FOV match; gap is phone ISP / AE. Try AE road metering (5a) and ISP off (5b). Do not expect 0.05 on a phone.

---

## 4. Pipeline: matches vs diverges

**Matches:** 1280×720, medmodel warp FL910/CY47.6, raw YUV, `fp` MPC math, stiffness at `tsf=1.0` = VW port, angle PID coeffs, panda STEER_MAX/deltas, HCA 100 Hz.

**Diverges / missing:** blend 0.6 vs ~1.0; planner ~12.5 vs 20 Hz; σ thresholds (different definitions); `paramsd` + road grade; lane change.

---

## 5. Work plan

| step | action | status |
|---|---|---|
| 0 | Resolve understeer contradiction (CAN yaw vs their `stiffnessFactor` 1.319) before editing tsf | open |
| 1 | Config: `steer_ratio` 16.27, `tsf` 1.319, `fp_steer_delay_s` 0.30 (after 0) | open |
| 2 | Inner loop 100 Hz | **done** 2026-08-04 |
| 3 | Road grade estimate | open |
| 4 | Port `paramsd` — see [`PARAMSD.md`](PARAMSD.md) | open |
| 5a/5b | AE metering region; optional ISP off; blend→1.0 after fast loop | open |

**Do not:** curvature plan correction, `center_force` at 12.5 Hz, horizon 16→32, cx/cy fix without calib reset.

Does **not** promise comma-two σ on phone camera.

---

## 6. Related

* [`PARAMSD.md`](PARAMSD.md) · [`CONTROLLER_LIMITS.md`](CONTROLLER_LIMITS.md) · [`BACKLOG.md`](BACKLOG.md)
