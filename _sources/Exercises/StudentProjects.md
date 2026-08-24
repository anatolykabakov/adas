# Projects

Most assignments run on a bag without the car. Road HCA — instructor supervision only.
Every report must include vision **Hz** and **e2e** capture→infer (or say why missing).

The small per-topic labs live inside their chapters now — each chapter builds its own toy and ends
with a measurable check (the bicycle chapter's failed P-controller, Pure Pursuit's trade-off table, the
sixty-line bus in Middleware, your own bag in Bags). This page keeps the larger, cross-chapter
assignments — the capstones.

Work in the AAD spirit: read the chapter → run a small measurement → write what you saw, including failure cases.

```{admonition} Which bag each project needs
:class: important
A bag you record yourself (Bags, *Record your own bag*) has vision, GPS and IMU but **no `vehicle/state`,
CAN or torque** — there was no panda. Projects that compare against the driver's steering or sweep a
controller against actuation (**A2, A3, B1, B2**) therefore need an **instructor-supplied bag** with CAN.
The rest run on your own recording.
```

## A — data analysis

### A1 Latency
Pick 2–3 sessions. Build a table of rate and e2e. If a thermal / throttle window exists, correlate with `infer_ms` and `phone/stats`.
**Deliverable:** plot + one paragraph separating "controller bad" vs "vision starved".

### A2 Understeer
On driver-steered frames, plot $\kappa_{\mathrm{fact}}/\kappa_{\mathrm{kin}}$ by $v$ bins (see [Vehicle model](../Control/VehicleModel.md)).
**Deliverable:** table vs the course reference bins; comment at $v\sim 20$ m/s.

### A3 Driver vs HCA
Reproduce CTE metrics on an instructor-assigned straight/arc bag.
**Deliverable:** |CTE| med/p95 on straight vs arc; state vision Hz on those windows.

### A4 Localization
Read [Localization](../Localization/Overview.md). On a bag: (1) ENU track from GPS, (2) time until `imu_yaw` valid, (3) overlay `localization/pose` vs odom.
**Deliverable:** one plot + note where GPS heading snaps.

## B — offline control

### B1 Pure Pursuit sweep
`bag/bag_config_sweep.py` on `pp_*`. Pareto |CTE| vs |$\Delta$SWA|.
**Deliverable:** knee point you would ship and why.

### B2 Delay sensitivity
Repeat B1 (or `fp`) at `--vision-latency` ∈ $\{0,\ 0.07,\ 0.2\}$.
**Deliverable:** which latency breaks which controller first.

### B3 Arc feed-forward
Using [MPC and fp](../Planner/MPC_and_FP.md) (sections fp-3 and fp-5), mark feed-forward vs feedback on one arc.
**Deliverable:** sketch or annotated plot; one sentence on why raising CTE weight is the wrong first lever.

## C — platform & safety

### C0 Middleware
Read [Middleware](../Architecture/Middleware.md). From `test_middleware.cpp`, list: how a parameter reaches a service variable; what `step()` does in Simulated mode.

### C1 Debug field
Add a field to `lane_keep_debug` → proto → PlotJuggler column.

### C2 Phone stats
Extend `phone/stats` with a counter you care about (drops, YOLO ms, …).

### C3 MetaDrive
Compare `pp` vs `fp` on a fixed scenario (`sim.eval`); report the same metrics as B1.

### C4 FCW / AEB / LDW
Read [Warnings](../Safety/Warnings.md). Run unit tests touching `test_safety_warn`, then:
```bash
PYTHONPATH=scripts \
  python3 scripts/bag/bag_safety_warn.py adas_logs/<session>
```
**Deliverable:** episode table + one false-positive that the gated rules correctly suppress (or explain why the bag cannot show FCW).

## D — research

### D1 YOLO under load
Enable traffic YOLO with a hard throttle so vision stays $\ge 9$ Hz — or document that it cannot.
### D2 Roll
Is roll observable in live calib? Effect on mean CTE bias?

## Rubric

| criterion | weight |
|---|---:|
| Metric and unit correctness | 30% |
| Reproducibility (commands, bag id, window) | 25% |
| Separation of vision / control / thermal | 25% |
| Clarity of presentation | 20% |

<!-- next-chapter -->
---

**Next:** [Setup](../Appendix/Setup.md)
