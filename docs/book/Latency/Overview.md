# Latency and Thermal

The controller always operates on a **stale** picture of the road.
At $v = 22$ m/s a delay of $0.23$ s is already ~5 m of travel. Without lookahead (`fp_steer_delay_s`) the loop systematically steers for where the car **was**, not where it is.

```{figure} figures/latency_chain.png
---
width: 95%
---
Break the e2e path into measurable segments — then compare sessions fairly.
```

## Metrics

```bash
cd app/src/main/scripts
python3 latency.py /path/to/session
```

| quantity | meaning |
|---|---|
| `infer_ms` | `OrtSession.run` only |
| e2e vision | `infer_ts − capture_ts` |
| capture → lane_keep | until desired path / SWA publish |
| capture → steer (fresh) | until command on actuation topic (stale-filtered) |

```{warning}
Raw `controls/steer` contains chassis republishes with **stale** `vision_ts`. For e2e use the filter in `latency.py` (same idea as PlotJuggler `latency/` columns).
```

## Session comparison

| session | conditions | Hz | capture→LK |
|---|---|---:|---:|
| `01_14_22` | no YOLO | 11.4 | ~69 ms |
| `14_13_03` | no YOLO | 9.8 | ~81 ms |
| `13_40_48` | no YOLO, throttle window | 8.5 / **~3 in dip** | 87 ms, max ~0.8 s |
| `11_49_49` | COCO YOLO | 7.2 | 88 ms, heavy tails |

YOLO on the same SoC **competes** with Supercombo. Separately, multi-minute thermal throttle shows up as rising `infer_ms` and collapsing Hz.

```{figure} figures/60_run0801_straights.png
---
width: 95%
---
Only interpret controller quality on windows where vision stayed healthy.
```

## `phone/stats`

At 1 Hz the bag can carry `cpu_pct`, `cpu_app_pct`, `thermal_status`, temperatures, `cpu0_freq_khz`.
Align them with `latency/vision/*` in PlotJuggler when you claim "thermal", not "bad MPC".

## Report requirements (course)

1. `vision_traffic: false` when comparing controllers.
2. `phone_stats: true` on teaching runs.
3. Always report Hz and e2e vision **alongside** CTE.
4. Do not shrink `fp_steer_delay_s` to the sum of medians without a closed-loop sweep — dynamics need margin ([Vehicle model](../Control/VehicleModel.md)).

## Exercise

Reproduce the table above on your assigned bags (or explain why a session is missing stamps). Circle any window with Hz $< 9$ and ban it from controller Pareto plots.

<!-- next-chapter -->
---

**Next:** [Bag and offline analysis](../Logging/Bags.md)
