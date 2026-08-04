# Control — Overview

**Longitudinal** control sets speed (throttle / brake). **Lateral** control sets steering for lane keeping.
This course focuses on lateral control. On MQB Golf, longitudinal often remains stock cruise; our HCA path is a supervised lateral actuator.

## What goes in, what comes out

**Inputs** (after vision + chassis topics): a metric path polyline, speed $v$, actual SWA, timestamps.
**Output:** desired steering (angle / torque command) → Panda → `HCA_01` → EPS.

Between path and rack sit three interchangeable lateral strategies plus a shared angle PID:

| `vehicle.lane_keep_controller` | role in the course |
|---|---|
| `pp` | geometric baseline — closest to AAD Pure Pursuit |
| `mpc` | VisionPilot path-MPC — best for reading cost / $\kappa$ stories |
| `fp` | **default on road** — delay + vehicle model, stock-like |

After desired SWA, `LatControlPID` produces normalized torque for HCA.

## Chapter order (follow this)

1. [Bicycle model](./BicycleModel.md) — $\delta$, ICR, $\kappa = \tan\delta / L$.
2. [Pure Pursuit](./PurePursuit.md) — one-step geometric law + algorithm box.
3. [Vehicle model / delay](./VehicleModel.md) — why kinematics lies on the highway.
4. [MPC and fp](./MPC_and_FP.md) — horizon optimization and phone defaults.

```{tip}
If you only memorize one diagnostic rule: **bad vision Hz or bad calib is not a reason to raise CTE weights.**
```

## Quality target

On healthy HCA windows, median |lane offset| on the order of **0.10–0.15 m** at $v\sim 20$ m/s is comparable to a careful human driver on the same roads.
Meter-scale deviation on that same segment → first check vision Hz / e2e latency and calibration, then controller parameters.

<!-- next-chapter -->
---

**Next:** [Kinematic bicycle model](./BicycleModel.md)
