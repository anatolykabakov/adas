# Warnings: FCW / AEB / LDW

Service `safety_warn` — warnings only, no braking or steering assist. Port of
`an earlier FCW` SafetyPlanner, reworked 2026-08-02: previous rules triggered
warnings where there was no danger.

## What was wrong

| was | why this is a false trigger |
|---|---|
| FCW on IDM accel ∈ [−5, −3], AEB on < −5 | IDM desired acceleration includes speed excess and curvature. On an empty road above 132 km/h and on an empty arc R 50 m at 22 m/s acceleration goes below −3.7 m/s² — **warning with no car ahead** |
| target = most probable from `lead0/lead1/lead2` | `lead1` and `lead2` are model predictions at +2 and +4 s. Warning fired for a car that is not there yet |
| LDW on `\|cte\| > 0.5` m | no speed gate, no drift-direction check, no lane-marking requirement, no debounce. Constant offset in an arc (normal controller behavior, see `CONTROLLER_LIMITS.md`) kept the warning on |
| no time filter | a single spike in target track or path fit flashed the icon |

## How it works now

**Longitudinal (FCW/AEB)** — from threat, not desired acceleration. IDM acceleration is still
computed and published as `accel_ms2`, but does not affect warnings.

```
target present (lead0, prob ≥ 0.5, 1 < d < 150 m) and v_ego ≥ 8 m/s
target in its lane:  |y_target − path(d)| ≤ 2.0 m,   path(d) ≈ cte + ½·κ·d²
closing:            Δv = v_ego − v_target ≥ 0.5 m/s
                      ttc  = gap / Δv
                      a_req = Δv² / (2·gap)
FCW: ttc ≤ 2.5 s  or  a_req ≥ 3.5 m/s²
AEB: ttc ≤ 1.4 s  or  a_req ≥ 5.5 m/s²   (AEB overrides FCW)
```

Target is compared to our own path, not the vehicle axis — so in an arc our own lead car does not
cross the lateral threshold.

**Lateral (LDW)**:

```
v_ego ≥ 12.5 m/s  (≈45 km/h)
path anchored on TWO lane lines of plausible width (LanePathMsg::lane_anchored)
turn signal on that side off (Gateway_72: BH_Blinker_li / BH_Blinker_re)
driver not holding wheel (chassis.steering_pressed)
we are not steering ourselves (fresh controls/steer with enabled, 250 ms window)
then:  |cte| > 0.5 m AND outward drift > 0.05 m/s     — or  |cte| > 0.8 m
```

Turn signal silences only its side: left on does not allow drifting right. Previously
the turn signal was not decoded at all and the only sign of intentional maneuver was steering torque
— which appears once the wheel is already moving. Address `0x3DB` was added to allowed receive IDs (read
only; transmit is still gated by `PandaSafetySupervisor`). **Not verified on the car**:
available runs lack this frame because the receive filter did not pass it.

Lane-marking requirement is key. Without it CTE is relative to the model plan, which on interchanges,
narrowings, and single-line segments swings by more than a meter; each such spike used to be
a warning. Outward-drift requirement separates lane departure from holding an offset line.

### Two gates added after the first run with control engaged

Run `2026_08_04_21_00_18` (23.5 min, night, no collisions and no unintended departures) produced 5
forward and 7 lane warnings — false positives by definition. Both classes had a single cause each.

**FCW/AEB speed gate 3 → 8 m/s.** All five episodes were stop-and-go: median 4.7 m/s, maximum 8.5.
Worst case 4.3 m/s with a nearly stationary lead 9.5 m ahead — TTC ≈ 1.9 s arithmetically, trivially
recoverable in practice. The trade-off is deliberate: city rear-end warnings below 29 km/h are given up
to stop crying wolf. `RealThreatAboveTheSpeedGateStillWarns` pins the case the gate must not silence.

**LDW suppressed while we steer** (`ldw_suppress_on_lat_active`, default on). Of 144 LDW frames, **82 %
had lateral control engaged** and the driver was steering in only 6 %; |cte| was 0.54 m median with a
drift rate of 0.15 m/s. That is the assistant's own tracking error on arcs, not a departure — LDW was
warning about itself. Upstream gates LDW the same way: the warning addresses a drifting *driver*.
Freshness window matches the HCA command timeout (250 ms) so LDW speaks again as soon as the assist
lets go.

Note on `threat_valid = 0` seen alongside active FCW/AEB in that run (15 of 74 and 11 of 16 frames):
that is the latch holding, not a missing validity gate — raising a warning already requires
`threat.valid`.

**Debounce** (`WarningLatch`): raise after 3 consecutive ticks (150 ms), clear after 10 quiet
(500 ms). Service tick — 50 ms.

## Verified

**Unit tests** — `app/src/main/cpp/tests/test_safety_warn.cpp`, 17 cases: empty road above
limit, empty arc, slow target far ahead, following without closing, closing
(FCW→AEB), target in adjacent lane, target in arc, parking speed, holding offset in arc,
drift right/left, return to center, no lane markings, driver hands on wheel, no path,
latch, turn signal on own and opposite side.

**On recordings** — `bag/bag_safety_warn.py` runs a bag through the real chain
`vision/lanes → proto_convert → vision/path → SafetyWarn` and counts episodes (not frames):

```bash
PYTHONPATH=scripts python3 scripts/bag/bag_safety_warn.py adas_logs/<session>
```

Run `2026_07_26_20_55_20`, 7.3 min city, |CTE| median 0.16 m / p95 0.98:

| rule | LDW episodes | % time |
|---|---|---|
| old (`\|cte\|>0.5`) | **131** | 11.0 |
| current | **0** | 0.0 |

Caveat without which the number means little: on this run the LDW gate never opens in any
frame — two lines of plausible width never accumulate (data from July 26, before calibration work; median speed 7.2 m/s). So the recording proves false triggers are gone, and
**does not** prove real departure still triggers a warning. That is covered by
unit tests and the simulator.

**In simulator** — `sim.eval` counts warning episodes together with controller metrics:

| run | LDW |
|---|---|
| track `highway`, 81 s normal driving | 0 |
| track `curvy` (torque limit boundary), 81 s | 1 — on real spike 0.58 m to line |
| start with 1.1 m offset at 25 m/s | 2 — departure detected |

FCW/AEB not checked on recordings: available runs lack lead-car data (fields `lead_*`
added to the protocol later; no `vision/model_long` topic in them). A fresh run is needed.

## Inputs / outputs

| topic | purpose |
|---|---|
| `vision/path` | CTE / epsi / κ / `lane_anchored` |
| `vision/model_long` | target: d, v, prob, y → CIPO |
| `vehicle/chassis` | speed, driver hands on wheel |
| **`safety/warn`** | `SafetyWarnState` |

`SafetyWarnState` exposes for debug `ttc_s`, `a_req_ms2`, `threat_valid`, `cte_rate_ms`,
`lane_anchored`, `driver_steering` — from these you can see why a warning fired or stayed silent.

Enabled via `nodes.safety_warn`, configured by the `safety_warn` block in `config.json`.

## Display in the app

`safety/warn` → ZMQ OUT → `MainActivity.onOutboundMessage` → `LaneOverlayView.setSafetyWarn`:

| flag | alertText1 | border |
|---|---|---|
| AEB | `BRAKE!` / Risk of Collision | red (`STATUS_ALERT`) |
| FCW | `BRAKE!` / Risk of Collision | orange (`STATUS_WARNING`) |
| LLDW/RLDW | `Lane Departure Detected` | orange |

Texts match flowpilot `events.py` (`fcw` / `ldw` / `stockAeb`).

## IDM longitudinal plan

IDM acceleration remains as longitudinal reference (and as a debug field). Calibrated on run
2026-07-31 `10_33_17`, offline v0.8.13: classic Δv instead of absolute target speed and
`v_lim` 100 km/h instead of 60 removed constant braking on the highway (`a=1.5`, `b=3.0`, `T=1.5`,
`s0=2.0`). Plot — [`figures/54_lead_long_warn_v0813_103317.png`](figures/54_lead_long_warn_v0813_103317.png).
