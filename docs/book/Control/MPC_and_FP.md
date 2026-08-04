# MPC and fp

Pure Pursuit picks **one** $\delta$ that hits a look-ahead point.
**MPC** optimizes a **short future trajectory**, applies the first command, and re-solves next frame.

| `lane_keep_controller` | role |
|---|---|
| `mpc` | VisionPilot path-MPC (best for learning cost / $\kappa$) |
| `fp` | stock-like time-domain MPC (**road default**) |

Appendix with real-arc forensics: [`MPC_EXPLAINED.md`](../../MPC_EXPLAINED.md).

## State

From the path fit:

| symbol | meaning |
|---|---|
| CTE | lateral offset from lane center [m] |
| epsi | heading error vs path tangent [rad] |
| $\kappa$ | path curvature [1/m] |

**Straight:** CTE, epsi, $\kappa\approx 0$ → **feedback** dominates.
**Arc:** feed-forward from $\kappa$ dominates. Bad/delayed $\kappa$ → outward drift. Raising CTE weight is usually the wrong first fix.

```{figure} figures/57_run0801_curve.png
---
width: 95%
---
When planner $\kappa$ lags yaw-derived curvature, SWA undershoots and CTE grows.
```

## Prediction model (toy)

Along path length with step $ds$ (VisionPilot uses path-domain steps; idea is the same):

$$
\begin{aligned}
\mathrm{CTE} &\leftarrow \mathrm{CTE} - \sin(\mathrm{epsi})\, ds, \\
\mathrm{epsi} &\leftarrow \mathrm{epsi} + \big(\kappa - \tfrac{\tan\delta}{L}\big)\, ds.
\end{aligned}
$$

* If you point left of the road ($\mathrm{epsi}>0$ in a chosen sign), CTE grows.
* If $\tan\delta/L$ does not match road $\kappa$, heading error integrates.

```python
import math

def predict(cte, epsi, kappa, delta, L=2.64, ds=0.5, n=10):
    """Roll n steps; return list of CTE."""
    out = []
    for _ in range(n):
        cte = cte - math.sin(epsi) * ds
        epsi = epsi + (kappa - math.tan(delta) / L) * ds
        out.append(cte)
    return out

# Start 0.4 m off center, aligned (epsi=0), mild arc kappa
print("wrong delta=0:   ", [round(c, 3) for c in predict(0.4, 0.0, 0.02, 0.0)])
delta_ff = math.atan(2.64 * 0.02)
print("delta≈atan(Lκ): ", [round(c, 3) for c in predict(0.4, 0.0, 0.02, delta_ff)])
```

You should see CTE drift grow when $\delta=0$, and grow **slower** when $\delta\approx\arctan(L\kappa)$.

## Cost (what "best" means)

A simplified scalar cost for one trial trajectory:

$$
J = \sum_i \Big(
\mathrm{CTE}_i^{2} + \mathrm{epsi}_i^{2}
+ w_{\mathrm{ff}}(\delta - \delta_{\mathrm{ff}})^{2}
+ w_{\Delta}(\delta_i - \delta_{i-1})^{2}
\Big).
$$

* First terms: stay centered and aligned.
* $w_{\mathrm{ff}}$ term: stay near geometric / understeer feed-forward $\delta_{\mathrm{ff}}$ (huge in arcs).
* Last term: don't jerk the wheel.

```python
import math

def cost(ctes, episis, delta, delta_ff, w_ff=100.0, w_d=1.0, delta_prev=None):
    j = sum(c*c + e*e for c, e in zip(ctes, episis))
    j += w_ff * (delta - delta_ff) ** 2
    if delta_prev is not None:
        j += w_d * (delta - delta_prev) ** 2
    return j

kappa = 0.02
delta_ff = math.atan(2.64 * kappa)
# Compare two trials from same start
for delta in (0.0, delta_ff, 1.5 * delta_ff):
    ctes = predict(0.2, 0.0, kappa, delta)
    # fake epsi history as zeros for the demo
    j = cost(ctes, [0.0]*len(ctes), delta, delta_ff)
    print(f"delta={math.degrees(delta):5.2f} deg → J={j:.3f}")
```

Typically the minimum sits near $\delta_{\mathrm{ff}}$ when $w_{\mathrm{ff}}$ is large — matching the road story that **arcs are feed-forward dominated**.

## Feed-forward

$$
\delta_{\mathrm{ff}} \approx f_{\mathrm{VM}}(\kappa, v)
$$

(with vehicle model at speed; kinematics alone if VM off). Then a seed

$$
\delta_{\mathrm{seed}} = \delta_{\mathrm{ff}} + k_{\mathrm{cte}}\mathrm{CTE} + k_{\mathrm{epsi}}\mathrm{epsi}
$$

starts the solver. On many real runs the descent barely moves the seed — so **seed quality = output quality**.

```python
import math

def seed(cte, epsi, kappa, v, k_cte=0.5, k_epsi=0.3, use_vm_scale=0.61):
    # crude VM: inflate kappa at speed (demo only; real code is richer)
    scale = 1.0 / use_vm_scale if v > 15 else 1.0
    delta_ff = math.atan(2.64 * kappa * scale)
    return delta_ff + k_cte * cte + k_epsi * epsi

print("straight:", math.degrees(seed(0.05, 0.01, 0.0, 20)))
print("arc:     ", math.degrees(seed(0.05, 0.01, 0.02, 20)))
```

## Seven steps (VisionPilot mental model)

1. Measure CTE, epsi, $\kappa$.
2. Predict trajectories for trial $\delta$.
3. Score with $J$.
4. Build $\delta_{\mathrm{ff}}$.
5. Solve (warm start + gradient).
6. Apply first sample → limits → SWA → PID → HCA.
7. Repeat next frame.

## Phone knobs (`fp`)

1. `fp_steer_delay_s` — look state ahead by delay.
2. `lat_use_vehicle_model` — understeer in $\kappa\to$ SWA.
3. `min_control_speed_mps` — no lateral near stop.
4. Measured `frame_dt` from stamps (not fixed 20 Hz).

## Fair comparison

One bag window, vision $\gtrsim 9$ Hz: |CTE| med/p95, |$\Delta$SWA|, straight vs arc separately.
Tool: `bag_config_sweep.py --vision-latency ...`.

## Exercise

1. Run the `predict` / `cost` snippets; which $\delta$ wins on the arc?
2. Set `w_ff=0` and rerun: does the optimizer still prefer $\delta_{\mathrm{ff}}$?
3. Bag: compare `pp` vs `fp`; disable VM once on an arc and report |CTE|.

<!-- next-chapter -->
---

**Next:** [FCW / AEB / LDW](../Safety/Warnings.md)
