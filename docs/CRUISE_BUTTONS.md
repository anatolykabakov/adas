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

## Limits (important)

- Stock CC **coasts / mild engine brake** only — **not AEB**. Keep SafetyWarn as UI-only.
- Panda stock safety: when `!controls_allowed`, Set/Resume TX blocked; Tip± and Cancel allowed.
- Virtual `v_set` latched from `v_ego` on engage (±1 km/h per tip). No ACC_02 HUD set-speed parse yet.
- Tip rate: deadband ~2.5 km/h, cooldown 200 ms, hold until GRA COUNTER edge (~33 Hz).

## Refs

- flowpilot `mqbcan.create_acc_buttons_control` / `carcontroller` pcmCruise cancel-resume
- Hyundai button-spam pattern: `flowpilot/.../hyundai/carcontroller.py`
- Safety TX: `safety_volkswagen_mqb.h` (`MSG_GRA_ACC_01` bus 0/2)
