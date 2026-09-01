# MPC as a toy — seven steps along the path

Pure Pursuit picks **one** $\delta$ that hits a look-ahead point.
**MPC** optimizes a **short future trajectory**, applies the first command, and re-solves next frame.

| `lane_keep_controller` | role |
|---|---|
| `fp` | stock-like time-domain MPC (**road default**) |
| `pp` | geometric baseline, and the fallback |

This chapter teaches MPC as a **path-domain toy you can hold in your head** — the seven steps below. This
design shipped as the `vp` / VisionPilot controller until 2026-08-21, when `fp` displaced it on the road
and it was deleted; the code lives in git history, the lesson stays here because it is the clearest way to
learn what any MPC does. The [next chapter](./MPC_and_FP.md) is the **time-domain `fp`** that actually drives.

The planners live behind one interface, `IPlanner`, and the `Planner` service holds exactly one of them:

* pure pursuit `pp`: `lateral/pp_planner.cpp`.
* flowpilot `fp`: `lateral/fp_planner.cpp` + `flowpilot_mpc.cpp` ($N{=}16$, time grid → 2.5 s), with a
  swappable numerical method — `kappa_solver_grad` or `kappa_solver_acados`.
* Path fusion (lanes↔plan): `laneLinesToPath` in `utils/lane_path.cpp` / `core/path_fusion.py`.

All of them emit **curvature**, never an angle. That is what lets them be swapped at runtime and compared on
the same bag: the conversion to steering happens once, downstream, in `Control`.

## State symbols

| symbol | meaning |
|---|---|
| CTE | lateral offset from lane center [m] |
| epsi | heading error vs path tangent [rad] |
| $\kappa$ | path curvature [1/m] |
| $\delta$ | front wheel angle [rad] |
| $L$ | wheelbase, 2.636 m (config; examples round to 2.64) |

**Straight:** CTE, epsi, $\kappa\approx 0$ → **feedback** dominates.
**Arc:** feed-forward from $\kappa$ dominates. Bad/delayed $\kappa$ → outward drift.

```{figure} figures/57_run0801_curve.png
---
width: 95%
---
When planner $\kappa$ lags yaw-derived curvature, SWA undershoots and CTE grows.
```

---

## Seven steps (a path-domain MPC, as a toy)

Shared toy constants for every snippet below:

```python
import math
import numpy as np

L = 2.64          # wheelbase [m]
N = 20            # horizon steps
DS = 0.5          # path step [m] (~1 s preview at ~10 m/s)
K_US = 0.0015     # understeer-ish term in the toy ff
FF_SCALE = 2.0
W_DELTA = 45000.0 # pin δ to δ_ff
W_DDELTA = 15000.0
```

### 1. Measure CTE, epsi, $\kappa$

Lane markings → lane-center polyline → quadratic $y = a x^2 + b x + c$ in ego frame.
At the vehicle ($x\approx 0$):

* $\mathrm{CTE} \approx c$ (lateral offset),
* $\mathrm{epsi} \approx b$ (heading vs path),
* $\kappa \approx 2a$ (curvature of the fit).

If the fit window is short or noisy, $\kappa$ is **low and late** — the whole feed-forward chain inherits that error.

```python
def fit_lane_state(x, y):
    """Quadratic fit y≈a x²+b x+c → (CTE, epsi, kappa) at x=0."""
    a, b, c = np.polyfit(x, y, 2)
    return float(c), float(b), float(2 * a)

# Toy centerline: gentle left arc in device frame (y right+)
x = np.linspace(0, 30, 31)
kappa_true = 0.02
y = 0.5 * kappa_true * x**2 + 0.05 * x + 0.30   # also 30 cm off + slight epsi
cte, epsi, kappa = fit_lane_state(x, y)
print(f"CTE={cte:.3f} m, epsi={math.degrees(epsi):.2f}°, κ={kappa:.4f} 1/m "
      f"(true κ={kappa_true})")
```

You should recover CTE ≈ 0.3 m and $\kappa$ near 0.02. On a real bag, compare `control/lane_keep_debug` κ to `yaw_rate / v`.

### 2. Predict trajectories for trial $\delta$

The toy rolls **along path length**, not clock time. For a candidate wheel angle $\delta$:

$$
\begin{aligned}
\mathrm{CTE} &\leftarrow \mathrm{CTE} - \sin(\mathrm{epsi})\, ds, \\
\mathrm{epsi} &\leftarrow \mathrm{epsi} + \big(\kappa - \tfrac{\tan\delta}{L}\big)\, ds.
\end{aligned}
$$

* Nose cocked relative to the road ($\mathrm{epsi}\neq 0$) → CTE integrates.
* $\tan\delta/L$ must match road $\kappa$, or heading error integrates.

```python
def predict(cte, epsi, kappa, delta, ds=DS, n=N, L=L):
    """Return arrays (cte[0..n], epsi[0..n]) including the start state."""
    cs, es = [cte], [epsi]
    for _ in range(n):
        cte = cte - math.sin(epsi) * ds
        epsi = epsi + (kappa - math.tan(delta) / L) * ds
        cs.append(cte)
        es.append(epsi)
    return np.array(cs), np.array(es)

cte0, epsi0, kappa = 0.40, 0.0, 0.02
delta_ff_kin = math.atan(L * kappa)

for name, delta in [("δ=0", 0.0), ("δ=atan(Lκ)", delta_ff_kin)]:
    cs, _ = predict(cte0, epsi0, kappa, delta)
    print(f"{name:12s} CTE@0={cs[0]:.3f} → CTE@end={cs[-1]:.3f} m")
```

Wrong $\delta$ lets CTE grow; geometric $\delta$ holds better. This is only the **predictor** — cost picks which $\delta$ wins.

### 3. Score with $J$

```{figure} figures/mpc_cost.png
---
width: 75%
---
The cost against a trial angle: the huge δ-pin weight pulls the minimum onto the feed-forward, so descent
barely moves the seed.
```


Lower is better. The terms, simplified (these are VisionPilot's, which is where the toy comes from):

$$
J = \sum_s \Big(
w_c\,\mathrm{CTE}_s^{2} + w_c\,s_4\,\mathrm{CTE}_s^{4} + w_e\,\mathrm{epsi}_s^{2}
+ W_\delta(\delta_s - \delta_{\mathrm{ff},s})^{2}
+ W_{\Delta}(\delta_s - \delta_{s-1})^{2}
\Big).
$$

Weights grow with $|\kappa|$ (`curve_factor = 1 + 20·κ`). The $W_\delta{=}45000$ term is enormous — so the optimum hugs $\delta_{\mathrm{ff}}$.

```python
def cost(cte0, epsi0, kappa, deltas, v=15.0,
         cte_w_base=20.0, epsi_w_base=10.0, quartic=5.0):
    """Score a sequence of wheel angles (len N)."""
    assert len(deltas) == N
    v2 = v * v
    curve = 1.0 + 20.0 * abs(kappa)
    cte_w = cte_w_base * curve
    epsi_w = epsi_w_base * curve
    cte, epsi = cte0, epsi0
    j = 0.0
    for s, delta in enumerate(deltas):
        j += cte_w * cte**2
        j += cte_w * quartic * cte**4
        j += epsi_w * epsi**2
        delta_ff = FF_SCALE * (math.atan(L * kappa) + K_US * v2 * kappa)
        j += W_DELTA * (delta - delta_ff) ** 2
        if s > 0:
            j += W_DDELTA * min(max(v2, 9.0), 25.0) * (delta - deltas[s - 1]) ** 2
        # one predict step
        cte = cte - math.sin(epsi) * DS
        epsi = epsi + (kappa - math.tan(delta) / L) * DS
    return j

kappa = 0.02
delta_ff = FF_SCALE * (math.atan(L * kappa) + K_US * 15**2 * kappa)
for scale in (0.0, 0.5, 1.0, 1.5):
    deltas = [scale * delta_ff] * N
    j = cost(0.20, 0.0, kappa, deltas)
    print(f"δ={math.degrees(scale*delta_ff):5.2f}°  J={j:.1f}")
```

Expect the minimum near `scale=1` ($\delta\approx\delta_{\mathrm{ff}}$). Set `W_DELTA=0` mentally: the ranking can flip toward pure CTE-minimizing $\delta$.

### 4. Build $\delta_{\mathrm{ff}}$

Feed-forward is the steering you need **even if already centered**:

$$
\delta_{\mathrm{ff}} = f_{\mathrm{scale}}\big(\arctan(L\kappa) + K_{\mathrm{us}} v^{2}\kappa\big).
$$

It is **linear in $\kappa$**. Underestimate $\kappa$ → understeer the arc. On a straight $\kappa\approx 0$ → $\delta_{\mathrm{ff}}\approx 0$ and feedback carries the car.

```python
def delta_ff(kappa, v, ff_scale=FF_SCALE, L=L, K_us=K_US):
    return ff_scale * (math.atan(L * kappa) + K_us * v * v * kappa)

for kappa in (0.0, 0.01, 0.02, 0.03):
    d = delta_ff(kappa, v=20.0)
    # What if perception returns only 66% of true kappa?
    d_bad = delta_ff(0.66 * kappa, v=20.0)
    print(f"κ={kappa:.3f}  δ_ff={math.degrees(d):5.2f}°  "
          f"if κ×0.66 → {math.degrees(d_bad):5.2f}°")
```

### 5. Solve (warm start + gradient)

Seed, then finite-difference / gradient steps on the $\delta$ sequence (C++: ~80 iters). In practice on road bags the descent barely moves the seed — **seed quality ≈ output quality**.

```{admonition} The toy collapses the horizon; the real solver does not
:class: warning
To keep the demo readable the block below optimises **one shared $\delta$** for all $N$ nodes — a scalar
line search, not a real MPC. It is enough to show that the descent barely leaves a good seed. The shipped
solver moves the whole *sequence* $\delta_0\ldots\delta_{N-1}$; do not read this block as how many
variables an MPC actually has.
```

$$
\delta_{\mathrm{seed}} = \delta_{\mathrm{ff}} + \underbrace{k_{\mathrm{cte}}\mathrm{CTE} + k_{\mathrm{epsi}}\mathrm{epsi}}_{\text{feedback clamp ~±0.25 rad}}.
$$

```python
def warm_start(cte, epsi, kappa, v, k_cte_base=0.6, k_epsi=0.3):
    v2 = v * v
    k_cte = max(k_cte_base / (1.0 + v2), 0.0)
    fb = max(-0.25, min(0.25, k_cte * cte + k_epsi * epsi))
    d0 = delta_ff(kappa, v) + fb
    return [d0] * N

def solve(cte, epsi, kappa, v, iters=40, step=2e-4):
    """Tiny FD gradient descent on a shared δ (toy; C++ optimizes the sequence)."""
    deltas = warm_start(cte, epsi, kappa, v)
    best = cost(cte, epsi, kappa, deltas, v=v)
    for _ in range(iters):
        # one shared angle for the demo
        d = deltas[0]
        trial_p = [d + step] * N
        trial_m = [d - step] * N
        jp = cost(cte, epsi, kappa, trial_p, v=v)
        jm = cost(cte, epsi, kappa, trial_m, v=v)
        grad = (jp - jm) / (2 * step)
        d_new = d - 5e-8 * grad          # tiny step — cost is stiff in δ
        deltas = [d_new] * N
        j = cost(cte, epsi, kappa, deltas, v=v)
        if j < best:
            best = j
        else:
            deltas = [d] * N
            break
    return deltas, best

cte, epsi, kappa, v = 0.20, 0.02, 0.02, 18.0
seed = warm_start(cte, epsi, kappa, v)
sol, j = solve(cte, epsi, kappa, v)
print(f"seed {math.degrees(seed[0]):.3f}° → solved {math.degrees(sol[0]):.3f}°  J={j:.1f}")
print(f"|Δδ|={abs(math.degrees(sol[0]-seed[0])):.4f}°  (often ~0 on road)")
```

### 6. Apply first sample → limits → SWA → PID → HCA

Take a command slightly ahead in the horizon (delay), then clamp:

1. angle cap growing with speed,
2. per-frame slew / jerk limit,
3. $\delta \times$ `steer_ratio` → steering-wheel angle,
4. angle-PID → torque $\in[\pm 300]$ cNm → `HCA_01` on CAN.

```python
STEER_RATIO = 15.7

def apply_limits(delta_cmd, delta_prev, v, max_slew_deg=8.0):
    # soft speed-growing angle cap [deg]
    cap = math.radians(max(8.0, min(25.0, 8.0 + v)))
    d = max(-cap, min(cap, delta_cmd))
    slew = math.radians(max_slew_deg)
    d = max(delta_prev - slew, min(delta_prev + slew, d))
    swa = d * STEER_RATIO
    return d, swa

delta_plan = sol[1] if len(sol) > 1 else sol[0]   # "look ahead" one sample
d_prev = 0.0
d_out, swa = apply_limits(delta_plan, d_prev, v=18.0)
print(f"wheel {math.degrees(d_out):.2f}°  SWA {math.degrees(swa):.1f}°  "
      f"(PID→torque happens in Control, CAN framing in Platform)")
```

Saturation at ±300 cNm means the **plant** cannot follow — not that the cost wanted more. Debug flag / UI: steering-limit indication. The whole δ → torque half of this step has its own chapter: [Angle control](../Control/AngleControl.md).

### 7. Repeat next frame

New `vision/lanes` (~42 ms median on the current phone) → new CTE/epsi/$\kappa$ → back to step 1.
MPC does **not** open-loop the whole second: only the first command is applied, then the plan is thrown away and rebuilt from the **measured** state.

```python
def one_phone_frame(cte, epsi, kappa, v, delta_prev):
    """Steps 4→6 for one camera frame (measure assumed already done)."""
    deltas = warm_start(cte, epsi, kappa, v)
    # real code runs gradient; seed≈solution on many bags
    d_plan = deltas[1] if len(deltas) > 1 else deltas[0]
    d_out, swa = apply_limits(d_plan, delta_prev, v)
    return d_out, swa

# Structural MPC loop — new measurement every frame, plan discarded
delta_prev = 0.0
for frame in range(5):
    # step 1 would refresh cte, epsi, kappa from vision here
    cte, epsi, kappa, v = 0.20, 0.01, 0.02, 18.0
    delta_prev, swa = one_phone_frame(cte, epsi, kappa, v, delta_prev)
    print(f"frame {frame}: δ={math.degrees(delta_prev):.2f}°  SWA={math.degrees(swa):.1f}°")

# Why κ quality matters more than "more MPC iterations"
print("\ncommand vs measured κ (CTE=epsi=0):")
for k_meas in (0.020, 0.013, 0.008):  # truth, ×0.66, worse
    d = warm_start(0.0, 0.0, k_meas, 20.0)[0]
    print(f"  κ_meas={k_meas:.3f} → δ_seed={math.degrees(d):.2f}°")
```

Only the first command is applied; the rest of the horizon is thrown away. If every frame's $\kappa$ is low, every seed undershoots — descent cannot invent curvature that step 1 never measured.

---

## Before the controller: the reference is a fusion

Both `fp` and `pp` track the polyline on `vision/path`, and that polyline is **not** raw paint or raw
model plan — it is the σ-weighted fusion of both, built in `laneLinesToPath`. That fusion, the two
failure modes it survives, and why it is where half the arc offset lives, is its own chapter:
[Lane path](./LanePath.md). This chapter assumes the reference already exists.

<!-- next-chapter -->
---

**Next:** [fp — the time-domain MPC](./MPC_and_FP.md)
