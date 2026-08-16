# Stock cruise via GRA buttons (non-ACC)

Golf without adaptive cruise: no ABS/ESP brake command, no `ACC_06`/`ACC_07`.
Speed authority = **stock PCM cruise** + steering-wheel stalk spoof on **`GRA_ACC_01` (0x12B)**.

## Architecture

```
vision/model_long ──► LongPlanService ──► control/long_plan (v_target)
                                              │
vehicle/state (GRA RX, TSK cruise_engaged)    │
                                              ▼
                                         PandaService
                                    computeCruiseButtons()
                                              │
                                              ▼
                              CarController → GRA_ACC_01 TX
                           Tip_Hoch / Tip_Runter / Cancel
                                              │
                                              ▼
                                         stock PCM CC
```

| Path | Status |
|------|--------|
| GRA RX decode → CarState | done |
| Latch stock GRA bytes | done |
| `create_acc_buttons_control` + CRC | done |
| `long_plan` → Tip± rate control | done (flag **off**) |
| ACC_06/07 accel | **not** used (needs radar ACC + long safety) |

## Enable

`assets/config.json` → `"vehicle.cruise_buttons": true` (default **false**).

Driver must engage stock cruise (Set). Planner then nudges set speed toward `v_target`.
Gas/brake → tips paused. Disengage → stop.

## How much deceleration actually exists (measured, 2026-08-06)

`bag/bag_coast_decel.py` over four 2026-08-04 bags — 125 intervals with both pedals up and speed falling
monotonically, which is exactly the state the car enters when the set speed drops below current speed:

| speed, m/s | n | median | p10 |
|---|---|---|---|
| 0–5 | 41 | −0.25 | −0.37 |
| 5–8.3 | 40 | −0.31 | −0.53 |
| 8.3–14 | 30 | −0.26 | −0.49 |
| 14–20 | 11 | −0.26 | −0.49 |
| 20–30 | 3 | −0.44 | −0.66 |

Median −0.28 m/s², strongest single interval −0.89. **Nearly flat with speed**, and that is the
informative part: engine drag scales with rpm and would be far stronger in a low gear, while road load
looks like this. The DSG is sailing, not engine-braking. The signal that would change it —
`ACC_Freilauf_Anf`, "request DSG sailing" in `ACC_07` — is sent only by the radar-equipped ACC, and
sending `ACC_07` ourselves means becoming the ACC (panda in the gateway position, `ALLOW_DEBUG`
firmware, stock ACC frames replaced). Not reachable from button spoofing.

So `long_plan.a_coast_ms2 = -0.30` is the entire deceleration budget. Consequence worth stating: at
20 m/s, reaching a 14.7 m/s corner speed needs (400 − 216) / (2 · 0.30) = **307 m** of coasting while the
path preview is 80 m. The curvature limiter can trim speed at city pace and nothing more; on a highway
corner it can only ask the driver.

## Plan rules that exist because of the first drive with this on

Run `2026_08_06_00_36_42` had `cruise_buttons` on for 28 minutes. The plan asked for a set speed
4.81 m/s below actual at the median, and the bus collected **715 rising edges of `cruise_decel` and 690 of
`cruise_accel`**. Four defects in `utils/long_planner.hpp`, all now covered by tests in
`tests/test_long_plan.cpp`:

- **`plan_v_enabled = false`** — `plan_v0 / v_ego` measured 0.678 (0.756 at 5–10 m/s → 0.669 at 20–30).
  The model's plan velocity is not a speed target in our metric scale, and the error moves with speed so
  no constant repairs it. Free flow now means hold the current speed.
- **`lead0` only** — `lead1`/`lead2` are +2 s and +4 s predictions.
- **`lead_max_offset_m` / `lead_min_speed_ms`** — the lead must be in our lane (compared against our own
  path, so a bend does not evict it) and moving. 31 % of lead ticks reported a nearly stationary
  object: parked cars at night.
- **target derived from `a_target`**, not set to the lead's speed. A lead 100 m ahead and 5 m/s slower
  needs no action, but the actuator reads any deficit past its deadband as "tip down".

And in `PandaService`: **the speed the driver had at engage is a ceiling** for the whole engagement. The
assistant may hand speed back and restore it, never ask for more than was chosen.

Replayed over the same run with `bag/bag_long_replay.py`: median `v_target − v_ego` −4.81 → +0.00 m/s, ticks
wanting over 0.5 m/s² of braking 66.9 → 0.0 %, tips **73.1 → 1.5 per minute**. Not yet driven.

## Limits (important)

- Stock CC **coasts only** — measured above, roughly −0.3 m/s², and **not AEB**. Keep SafetyWarn as UI-only.
- Panda stock safety: when `!controls_allowed`, Set/Resume TX blocked; Tip± and Cancel allowed.
- Virtual `v_set` latched from `v_ego` on engage (±1 km/h per tip) and **capped at that latch** — see
  above. No ACC_02 HUD set-speed parse yet, so the latch is our only idea of what the driver chose.
- Tip rate: deadband 0.7 m/s (2.5 km/h), cooldown 200 ms, hold until GRA COUNTER edge (~33 Hz).

## Refs

- flowpilot `mqbcan.create_acc_buttons_control` / `carcontroller` pcmCruise cancel-resume
- Hyundai button-spam pattern: `flowpilot/.../hyundai/carcontroller.py`
- Safety TX: `safety_volkswagen_mqb.h` (`MSG_GRA_ACC_01` bus 0/2)
