# fp — the time-domain MPC that drives

The [toy](./MpcToy.md) optimised along the path. The shipped `fp` optimises along **time**: it predicts where
the car will be in 0.01 … 2.5 s on flowpilot's grid, pays for the reference error, the heading error and the
steering rate, and — the part the toy could not do — compensates the 0.35 s the steering tract takes to
answer. This chapter builds it in seven steps, then measures where the horizon does not help, lists the
phone's knobs, and closes the loop in MetaDrive.

Code: `lateral/fp_planner.cpp`, `lateral/flowpilot_mpc.cpp` ($N{=}16$, time grid → 2.5 s), the swappable
numerical method `kappa_solver_grad` / `kappa_solver_acados`.

## `fp` model (flowpilot time-domain MPC)

Road default (`lane_keep_controller=fp`). Code: `lateral/fp_planner.cpp` and `lateral/flowpilot_mpc.cpp`,
selected by `Planner` through `makeKappaSolver`.

Opposite of the path-domain $\delta$-MPC above:

| | path-domain toy (above) | flowpilot `fp` |
|---|---|---|
| decision variable | wheel angle $\delta$ along $s$ | yaw-rate rate $u=\dot r$ in **time** |
| horizon | $N{=}20$, $ds\sim 0.5$ m | $N{=}16$, $t_i=10(i/32)^2$ → **$T_f=2.5$ s** |
| output | $\delta$ | desired **curvature** $\kappa^\star$, then VM → $\delta$ |
| delay | pick $\delta_1$ | `lagAdjustedCurvature` over `fp_steer_delay_s` |

### fp-1. Reference in time

Polyline is flipped to openpilot left+ (`y \leftarrow -y`). Sample at distances $v\cdot t_i$:

$$
t_i = 10\left(\frac{i}{32}\right)^{2},\quad i=0\ldots 16.
$$

Build $y_{\mathrm{ref}}(t)$, $\psi_{\mathrm{ref}}(t)$, $r_{\mathrm{ref}}=\dot\psi_{\mathrm{ref}}$.
If Supercombo orientation heads exist, $\psi$ / $r$ come from `plan_yaw` / `plan_yaw_rate`.

```python
import numpy as np

N_FP = 16
T_IDX_MAX = 32.0

def t_node(i):
    return 10.0 * (i / T_IDX_MAX) ** 2

print("fp time grid [s]:", [round(t_node(i), 3) for i in range(0, N_FP + 1, 4)])
# 0, 0.156, 0.625, 1.406, 2.5

def sample_y_ref(poly_xy, v):
    """poly_xy: Nx2 in left+ frame; return y_ref[0..N]."""
    x, y = poly_xy[:, 0], poly_xy[:, 1]
    s = np.zeros(len(x))
    s[1:] = np.cumsum(np.hypot(np.diff(x), np.diff(y)))
    y_ref = []
    for i in range(N_FP + 1):
        dist = min(v * t_node(i), s[-1])
        y_ref.append(float(np.interp(dist, s, y)))
    return np.array(y_ref)

# Straight offset 0.25 m left in left+ frame
poly = np.stack([np.linspace(1, 40, 40), 0.25 * np.ones(40)], axis=1)
print("y_ref@v=15:", np.round(sample_y_ref(poly, 15.0)[::4], 3))
```

### fp-2. Dynamics (state)

State $x=(X,Y,\psi,r)$, control $u=\dot r$ (yaw acceleration):

$$
\begin{aligned}
\dot X &= v\cos\psi - R_{\mathrm{rot}}\sin\psi\cdot r,\\
\dot Y &= v\sin\psi + R_{\mathrm{rot}}\cos\psi\cdot r,\\
\dot\psi &= r,\\
\dot r &= u.
\end{aligned}
$$

$R_{\mathrm{rot}}\approx L/2$ in our config. Ego starts at origin; $r_0$ seeded from path geometry on first call.

```python
def forward_fp(u, v, r0=0.0, R_rot=1.32):
    """u length N; return Y, psi, r trajectories length N+1."""
    X = Y = psi = 0.0
    r = r0
    Ys, psis, rs = [Y], [psi], [r]
    for i in range(N_FP):
        dt = max(t_node(i + 1) - t_node(i), 1e-4)
        X = X + dt * (v * np.cos(psi) - R_rot * np.sin(psi) * r)
        Y = Y + dt * (v * np.sin(psi) + R_rot * np.cos(psi) * r)
        psi = psi + dt * r
        r = r + dt * u[i]
        Ys.append(Y); psis.append(psi); rs.append(r)
    return np.array(Ys), np.array(psis), np.array(rs)
```

### fp-3. Cost

With $v_{\mathrm{off}}=v+\mathrm{speed\_offset}$ (default +10):

$$
J = \sum_i \Big(
w_y(Y_i-y_{\mathrm{ref}})^2
+ w_\psi v_{\mathrm{off}}^2(\psi_i-\psi_{\mathrm{ref}})^2
+ w_r v_{\mathrm{off}}^2(r_i-r_{\mathrm{ref}})^2
+ w_{\mathrm{jerk}}(v_{\mathrm{off}} u_i)^2
+ w_{\mathrm{srate}}(u_i/(v+0.1))^2
\Big).
$$

Shipped-ish weights: `path_weight=1`, `heading_weight=0.11`, `lat_jerk_weight=0.05`,
`fp_steering_rate_weight=400`.

```python
def cost_fp(u, y_ref, v, w_y=1.0, w_psi=0.11, w_srate=400.0, speed_offset=10.0):
    Y, psi, r = forward_fp(u, v)
    v_off = v + speed_offset
    j = 0.0
    psi_ref = np.zeros_like(y_ref)   # toy: straight
    for i in range(N_FP + 1):
        j += w_y * (Y[i] - y_ref[i]) ** 2
        j += w_psi * (v_off * (psi[i] - psi_ref[i])) ** 2
        if i < N_FP:
            j += w_srate * (u[i] / (v + 0.1)) ** 2
    return j

v = 15.0
y_ref = sample_y_ref(poly, v)
# Path-only view (w_srate=0): small +u pulls Y toward +0.25
for u0 in (0.0, 0.01):
    u = np.full(N_FP, u0)
    Y, _, _ = forward_fp(u, v)
    j_path = cost_fp(u, y_ref, v, w_srate=0.0)
    j_full = cost_fp(u, y_ref, v, w_srate=400.0)
    print(f"u={u0:+.2f}  Y_end={Y[-1]:+.3f}  J_path={j_path:.3f}  J_full={j_full:.3f}")
```

$u{=}0.01$ cuts path cost and ends near the 0.25 m ref; full $J$ still rises from the rate term — the real GD balances both (shipped `fp_steering_rate_weight=400`).
### fp-4. Solve

Warm-start previous $u$; ~50 FD-GD iterations with line search (`gd_step=0.1`).
Same spirit as the toy above: good warm start matters; horizon is short.

### fp-5. Lag-adjusted curvature (the delay trick)

After the solve, read yaw-rate trajectory $r(t)$ and form

$$
\kappa_0 = \frac{r(0)}{v},\qquad
\kappa_{\mathrm{avg}} = \frac{\psi(t_d)}{v\, t_d},\qquad
\kappa^\star = 2\kappa_{\mathrm{avg}} - \kappa_0,
$$

with $t_d=\mathrm{fp\_steer\_delay\_s}$ (shipped **0.35** s), then rate-limit $\kappa^\star$ by
`max_lateral_jerk / v²`. This is the port of openpilot `get_lag_adjusted_curvature`:
command the curvature you need **when the rack will actually move**.

```python
def lag_adjusted_kappa(psi_sol, r_sol, v, delay_s=0.35, dt_s=0.08, max_jerk=5.0):
    ts = np.array([t_node(i) for i in range(N_FP + 1)])
    kappa0 = r_sol[0] / v
    psi_d = float(np.interp(delay_s, ts, psi_sol))
    avg = psi_d / (v * delay_s)
    desired = 2.0 * avg - kappa0
    max_rate = max_jerk / (v * v)
    return float(np.clip(desired, kappa0 - max_rate * dt_s, kappa0 + max_rate * dt_s))

# Constant-r turn: psi(t)=r0*t → avg=kappa0 → κ*=kappa0 (no change)
r0 = 0.02 * 15.0
ts = np.array([t_node(i) for i in range(N_FP + 1)])
psi = r0 * ts
r = np.full_like(ts, r0)
print("steady turn κ*:", round(lag_adjusted_kappa(psi, r, v=15.0), 5))
```

### fp-6. $\kappa^\star \to$ wheel angle → PID → HCA

```text
κ*  →  vehicle_model (if lat_use_vehicle_model)  →  δ
    →  angle slew / caps  →  × steer_ratio  →  angle-PID  →  HCA torque
```

Sign: device $y$ right+; service publishes `steer_rad = -steerFromCurvature(...)`.
Details of understeer scaling: [Vehicle model](../Control/VehicleModel.md).

### fp-7. Next vision frame

Re-solve with measured `frame_dt` (not fixed 0.05). Angle-PID on the phone runs off
`vehicle/state` (~100 Hz after CAN RX 10 ms); planner rate stays vision-limited (~24 Hz).

---

## Where the horizon does not help

A horizon fixes the information limit of Pure Pursuit — it reads the whole path instead of one point. It does
not fix everything, and the three things it cannot fix were each measured rather than argued.

### 1. A steady offset is nearly free in the cost function

`fp` resets its state every frame: $x_0 = [0, 0, 0, \dot\psi]$. The car is, by definition, exactly on its own
reference at node 0. So the only way a persistent offset can be penalised is through the *later* nodes — and
that is where the time grid works against you.

```python
V = 15.0
print(f"{'node':>5} {'t, s':>7} {'distance, m':>12} {'lateral reach, m':>17}")
for i in (0, 1, 2, 3, 4, 6, 8, 12, 16):
    t = t_node(i)
    d = V * t
    # How far sideways can the car physically move by then, at a generous 3 m/s^2 of lateral accel?
    reach = 0.5 * 3.0 * t * t
    print(f"{i:>5} {t:>7.3f} {d:>12.2f} {reach:>17.3f}")
```

```{figure} figures/mpc_timegrid.png
---
width: 80%
---
The quadratic time grid samples the near field densely, but in the first few nodes the car physically
cannot move sideways — so a steady offset there is nearly free in the cost.
```

Read the reach column. Over the first four nodes the car cannot move sideways by more than a few
centimetres, whatever the steering does — so a 0.35 m offset at those nodes is not a cost the optimiser can
act on, it is a constant. The nodes where lateral motion is possible are far enough ahead that a curvature
error dominates them.

Measured consequence: the car sat **0.35 m inside its own reference** on arcs. And this is not a weight to
tune — closed-loop sweeps confirmed that neither `fp_steering_rate_weight` (150 against 800) nor
`fp_steer_delay_s` (0.23 against 0.35) changes it. What removes a steady offset is an integral term, which
this formulation does not have, or a reference that is already shifted, which is what
lane blending, a few sections above does.

### 2. Lengthening the horizon does not help either

The obvious response is "use flowpilot's N = 32 instead of our 16". Checked, and it does not follow, because
the grid is quadratic:

```python
first_second = sum(1 for i in range(N_FP + 1) if t_node(i) <= 1.0)
print(f"our N=16: {first_second} of {N_FP + 1} nodes inside the first second, horizon {t_node(N_FP):.2f} s")
first_second_32 = sum(1 for i in range(33) if 10.0 * (i / 32.0) ** 2 <= 1.0)
print(f"N=32    : {first_second_32} of 33 nodes inside the first second, horizon "
      f"{10.0 * (32 / 32.0) ** 2:.2f} s")
print("\nThe near zone is already as densely sampled as theirs. Doubling N adds far nodes, which at equal")
print("weights dilutes the near zone that actually matters, and costs about 4x more to solve under the")
print("gradient solver.")
```

### 3. The plan it optimises has its own bias

MPC minimises distance to a reference. If the reference is wrong, it tracks the wrong thing perfectly. On
arcs the model plan sits **+0.32 / −0.35 m** from the lane centre, and an attempt to remove that with a
single curvature coefficient failed in an instructive way:

* parameterised as a constant distance ($50.2\kappa$) it was unstable within one run — 20.2 below 12 m/s
  against 77.2 above;
* the correct parameterisation is a *time* lookahead, $\tfrac{1}{2}\kappa(vT)^2$, and $T$ is indeed stable
  against speed: 0.71–1.03 s;
* but not reproducible **between** runs: 0.84 s on one, 0.56 s on another, because the two runs had
  different learned camera yaw (+1.12° against +0.24°), which changes the input warp, so the network sees a
  different image and outputs a different plan.

So the coefficient cannot be baked in: on another calibration it adds its own offset. That is why lane
blending — which needs no tunable number — became the main lever instead.

### 4. And when torque saturates, none of this matters

On the measured right arcs, torque sat at the ±300 cNm HCA ceiling in **65 % of frames** (76 % on an earlier
run, 80 % within one 12.3-second episode at R = 130 m). While saturated there is no feedback at all: the
optimiser's output changes and the rack does not. The remedy is to request less curvature, not to solve
harder.

```{admonition} What to take from this section
:class: tip
Each limit above is a different kind. One is structural (no integral term), one is a non-result (the horizon
is long enough), one is upstream (the plan's own bias, coupled to calibration), and one is physical (the
actuator). Diagnosing which one you are looking at is most of the work, and none of them is found by
sweeping weights.
```

## Phone knobs

**Shared / path**

* `path_lane_blend_scale`, `lane_std_good_m`, `lane_std_bad_m`, `path_camera_offset_m`
* `min_control_speed_mps`, `lat_use_vehicle_model`, `tire_stiffness_factor`

**`fp`**

1. `fp_steer_delay_s` — lag-adjusted $\kappa$ horizon.
2. `fp_steering_rate_weight` — smoothness of $u$.
3. Measured `frame_dt` from stamps.

## Fair comparison

One bag window, vision $\gtrsim 9$ Hz: |CTE| med/p95, |$\Delta$SWA|, straight vs arc separately.
Tool: `bag/bag_config_sweep.py --vision-latency ...` (also sweeps `--blend`).

## Close the loop in a simulator

Everything in this part can be driven closed-loop in MetaDrive before any car is involved (host build +
install: [Setup](../Appendix/Setup.md)):

```bash
cd scripts
python3 -m sim.eval --track highway --controllers fp,pure_pursuit --seeds 7
python3 -m sim.eval --list-tracks
```

`highway` is built from the working region of `docs/CONTROLLER_LIMITS.md` (radii 250–700 m);
`serpentine` is deliberately outside it. The harness accepts anything that implements one call — your
own controller included:

```python
# not-runnable — the sketch of the adapter; MetaDrive and the harness live outside the book build
class MyPurePursuit:
    """Duck-typed like core/lane_keep.py results: steer_norm in [-1, 1]."""
    def __init__(self, wheelbase=2.636, ld=8.0):
        self.L, self.ld = wheelbase, ld

    def step(self, path_xy, v_mps, max_steer_rad):
        delta, _ = pure_pursuit(path_xy, 0.0, 0.0, 0.0, self.ld, 0)   # ego frame: x,y,psi = 0
        return max(-1.0, min(1.0, delta / max_steer_rad))
```

Two things change silently: the path arrives in the **ego frame** every tick, and the output is
**normalised steering** — the harness owns the actuator limits, exactly as `Control` owns them on the
phone. And remember what the simulator flatters: perception is perfect — no σ, no dropouts, no 42 ms of
latency. A controller that wins here and loses on a bag is the expected order of events
(`docs/SIM_CONTROLLER_TEST.md`).

## Exercise

1. Toy MPC steps 2–3: which $\delta$ wins on the arc? Zero the $\delta$-pin weight — does the winner move?
2. Blending snippet: at `y(40)`, how much of the −0.4 m plan cut remains at `blend=0.6`?
3. Drop lane $\sigma$ to 1.6 m in the blend toy — what is $d_{\mathrm{prob}}$?
4. fp: change `delay_s` from 0.1 to 0.35 on a rising-$r$ trajectory; how does $\kappa^\star$ move?
5. Bag: `pp` vs `fp`; disable VM once on an arc; sweep blend 0.3 / 0.6 / 1.0.

<!-- next-chapter -->
---

**Next:** [Longitudinal planner](./Longitudinal.md)
