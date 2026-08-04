# Vehicle Model (Understeer)

## Kinematics vs reality

From the bicycle chapter, **kinematic** curvature for wheel angle $\delta$ is

$$
\kappa_{\mathrm{kin}} = \frac{\tan\delta}{L}.
$$

If the car truly followed that, yaw rate would satisfy $\dot\psi = v\,\kappa_{\mathrm{kin}}$.
Define the **measured** curvature from sensors:

$$
\kappa_{\mathrm{fact}} = \frac{\dot\psi}{v}.
$$

At parking-lot speeds $\kappa_{\mathrm{fact}} \approx \kappa_{\mathrm{kin}}$.
On the highway the tires slip: for the **same** $\delta$ you get **less** curvature,

$$
\frac{\kappa_{\mathrm{fact}}}{\kappa_{\mathrm{kin}}} < 1.
$$

![Understeer: kinematics vs fact](figures/understeer_idea.png)

If feed-forward uses $\kappa_{\mathrm{kin}}$ only, it **under-orders** SWA in arcs → outward drift; feedback fights late, after vision delay.

## Simple understeer formula

A one-parameter model used in openpilot-style stacks (and our `tire_stiffness_factor`) looks like

$$
\kappa_{\mathrm{fact}} \approx \frac{\kappa_{\mathrm{kin}}}{1 - c\, v^{2}}
\quad\text{with understeer coefficient } c < 0 \text{ in the usual slip convention,}
$$

or equivalently: to **achieve** a desired $\kappa_{\mathrm{des}}$ you must command a larger wheel angle than $\arctan(L\kappa_{\mathrm{des}})$.

Operationally in code:

$$
\delta_{\mathrm{kin}} = \arctan(L\,\kappa_{\mathrm{des}}),
\qquad
\delta_{\mathrm{cmd}} = f_{\mathrm{VM}}(\kappa_{\mathrm{des}}, v) > \delta_{\mathrm{kin}} \text{ at high } v.
$$

```{admonition} Intuition
:class: tip
Tires make the car "longer" in curvature space: same $\delta$, smaller $\kappa$.
The vehicle model **scales the command up** so the mental bicycle matches the real Golf.
```

## Measurement recipe (bag)

On frames where the **driver** steers (or any known SWA):

1. $\delta = \mathrm{SWA} / (\texttt{steer\_sign}\cdot\texttt{steer\_ratio})$.
2. $\kappa_{\mathrm{kin}} = \tan\delta / L$.
3. $\kappa_{\mathrm{fact}} = \dot\psi / v$ (need $v$ not tiny).
4. Bin by $v$, report median ratio $\kappa_{\mathrm{fact}}/\kappa_{\mathrm{kin}}$.

Reference table from our Golf:

| $v$, m/s | $\kappa_{\mathrm{fact}}/\kappa_{\mathrm{kin}}$ |
|---|---:|
| 4–8 | ~0.99 |
| 12–16 | ~0.85 |
| 20–26 | **~0.61** |

At ~80 km/h kinematics overstates curvature by ~$1/0.61 \approx 1.6\times$.

```python
import math
import numpy as np

L = 2.636
STEER_RATIO = 15.7
STEER_SIGN = -1

def delta_from_swa_deg(swa_deg: float) -> float:
    return math.radians(swa_deg / (STEER_SIGN * STEER_RATIO))

def ratio_kin_vs_fact(swa_deg, yaw_rate_rad_s, v_mps):
    """Return kappa_fact / kappa_kin (NaN if unstable)."""
    if abs(v_mps) < 1.0:
        return float("nan")
    delta = delta_from_swa_deg(swa_deg)
    k_kin = math.tan(delta) / L
    k_fact = yaw_rate_rad_s / v_mps
    if abs(k_kin) < 1e-6:
        return float("nan")
    return k_fact / k_kin

# Synthetic demo: at 22 m/s, pretend fact is 0.61 * kin
v = 22.0
swa = -60.0  # deg on CAN
delta = delta_from_swa_deg(swa)
k_kin = math.tan(delta) / L
k_fact = 0.61 * k_kin
yaw = k_fact * v
print(f"delta={math.degrees(delta):.2f} deg, k_kin={k_kin:.4f}, "
      f"ratio={ratio_kin_vs_fact(swa, yaw, v):.2f}")
```

How much extra SWA does feed-forward need? If you want the **same** $\kappa_{\mathrm{fact}}$ that kinematics promised at low speed, scale roughly by $1/0.61\approx 1.64$ at 22 m/s (order of magnitude; real VM is smoother).

```python
k_des = 0.02  # 1/m, mild arc
delta_kin = math.atan(L * k_des)
# crude high-speed correction: inflate kappa request
delta_vm = math.atan(L * k_des / 0.61)
print(f"|SWA| kin={abs(math.degrees(delta_kin)*STEER_RATIO):.1f} deg, "
      f"VM~{abs(math.degrees(delta_vm)*STEER_RATIO):.1f} deg")
```

## Project knobs

* C++ `vehicle_model.h`, Python `scripts/core/vehicle_model.py`
* `vehicle.lat_use_vehicle_model = true`
* `tire_stiffness_factor = 0.64` (shipped)

## Pipeline delay

Medians on healthy run `01_14_22`:

| link | ms |
|---|---:|
| capture → end of inference | ~62 |
| inference → publish lane_keep | ~7 |
| command → rack angle | ~40 |
| rack → yaw rate | ~120 |
| **sum** | **~230** |

`fp_steer_delay_s = 0.35` $> 0.23$: covers transport **and** dynamics.

```python
v = 22.0  # m/s
for delay in (0.23, 0.35):
    print(f"delay {delay:.2f} s → {v*delay:.1f} m of travel")
# 0.23 → 5.1 m,  0.35 → 7.7 m
```

```{admonition} Arc drift diagnostics
:class: warning
Order: (1) vehicle model on; (2) vision Hz / e2e OK; (3) only then CTE / epsi weights.
```

## Exercise

1. Run the synthetic `ratio_kin_vs_fact` snippet; change `0.61` to `0.99` and see SWA inflation vanish.
2. On a bag with driver steering, plot ratio by $v$ bins; compare to the table.
3. Argue in one sentence why cutting `fp_steer_delay_s` to $0.23$ can still hurt closed loop.

<!-- next-chapter -->
---

**Next:** [MPC and fp](./MPC_and_FP.md)
