# How lane-keep MPC works — in plain terms

> Analysis of lateral controller **VisionPilot MPC** (`app/src/main/cpp/src/visionpilot/lateral_planning.cpp`)
> on real data from run `adas_logs/2026_07_30_10_52_16` (controller = `mpc`, ~636 s clean
> autopilot @ 11 Hz). Here we explain **why it drives well on straights and poorly in arcs.**

## One sentence

MPC = predictive control. Every **~90 ms** the controller mentally plays out
**~1 second of driving ahead** at different steering angles and picks the angle where the car
ends up closest to lane center and aligned with the road. Then it sends the first step of that plan to the rack — and 90 ms later starts over.

---

## Seven steps per frame

### Step 1. Measure position in lane
From lane markings (`vision/lanes`) lane center is built, and quadratic fit
`y = a·x² + b·x + c` yields **three state numbers**:

| number | what it is | in plain words |
|-------|---------|------------------|
| **CTE** | lateral offset, m | how far left/right of center |
| **epsi** | heading error, rad | how much the nose points away from road direction |
| **κ** (kappa) | road curvature, 1/m | how sharply the road bends |

> ⚠️ **Root problem lives here.** Poly fit **underestimates κ by ~1.5×** and outputs
> it **with ~0.4 s lag**. On straights all three ≈ 0 — so there it works well.

### Step 2. Motion model — "play movie forward"
Computed by path length `s`, not time, **20 steps `ds` ≈ 1 s**. For trial
steering angle δ a simple "bicycle" model predicts how CTE and epsi evolve:

```
CTE  ← CTE  − sin(epsi)·ds          // drive crooked → drift sideways
epsi ← epsi + (κ − tan(δ)/Lf)·ds    // steering δ turns nose, road κ pulls
```

`Lf = 2.67 m` — wheelbase. Thus for any δ you get full trajectory one second ahead.

### Step 3. Cost function — "how bad is it"
For a trial trajectory a penalty is computed. Lower is better for lane keeping and smooth steering:

```
penalty = CTE²  +  CTE⁴  +  epsi²                 ← don't stand off-center, look along road
      + 45000·(δ − δ_ff)²                        ← stay near feedforward δ_ff (step 4)
      + (δ_next − δ_now)²                         ← don't jerk wheel
```

Weights in turns **grow** (`×(1 + 20·κ)`) — in arcs controller is stricter on deviations.
Term `(δ − δ_ff)²` with weight **45000** is huge, so final angle is almost pinned to δ_ff.

### Step 4. Feedforward δ_ff — how much steering for arc geometry
Separately computed: how much to turn **purely to fit the bend**, even standing perfectly
on center. This is **main steering portion in arcs**:

```
δ_ff = ff_scale · ( atan(Lf·κ) + K_us·v²·κ )      ff_scale = 2.0
```

> ⚠️ δ_ff is **directly proportional to κ**, and κ is low (step 1) → in arcs **not enough steering**,
> car drifts outward. On straights κ≈0 → δ_ff≈0, so straights run straight.

### Step 5. Solver — find best angle
Smart initial guess (warm-start seed), then gradient descent (80 iterations)
tries to improve it:

```
δ_seed = δ_ff  +  ( cte_gain·CTE + epsi_gain·epsi )      epsi_gain = 0.3
          └ feedforward ┘   └──── feedback ────┘
```

> ✅ **In practice descent almost changes nothing** — on run `corr(seed, output) = −0.996`.
> So **MPC output ≈ starting guess**. Key conclusion: on straights feedback part steers
> (well tuned — `epsi_gain` recently lowered 0.5→0.3, weaving gone); **in arcs feedforward δ_ff steers**,
> and it depends on bad κ.

### Step 6. First step → limits → rack
From plan `[δ₀, δ₁, … δ₁₉]` take **δ₁** (slightly ahead for delay) and pass three
limiters: angle cap `clamp(8+v, 8..25)°`, rate limit `Δδ` by jerk, overall slew
`≤ 8°/frame`. Then `δ × 15.7` → PID → rack torque (`torque_cnm`, ±300) → CAN.

> ✅ Torque **does not hit ceiling**: saturation ±300 = **0 %** frames (old weaving
> MPC had 87 %).

### Step 7. Again
New camera frame (~90 ms) → new state → step 1 again. That is MPC essence: **constantly
replan** from real position, so errors do not accumulate — *if state is measured
correctly*. In arcs it is not (κ), so CTE accumulates.

---

## Why "good on straight, bad in arc"

Different steps carry steering in two modes:

- **Straight (green).** CTE, epsi, κ ≈ 0. Steering held by small **feedback** — and it is
  tuned well. δ_ff silent. Command median **0.6°**, torque 0 % saturation → smooth.
- **Arc (red).** Steering held by **feedforward δ_ff = 17°**, feedback adds only 2.7°.
  Feedforward input — κ — is **low ×0.66** (right plot: cloud below ideal
  line). So not enough steering + lag → car drifts outward.

### Visible on one real turn

- **Middle panel — most important:** red `κ_used` (what MPC sees) always **below** black
  `yaw/v` (true curvature) and **time-shifted**. Perception property — independent
  of who steered.
- **Top:** green δ_ff (feedforward) due to bad κ **lags the road** — keeps commanding
  15–20° when turn already ends.
- **Bottom:** through arc `|epsi|` grows first, then **CTE drifts** from center — classic
  outward slip with late correction.

> Steering shake in arc also from κ, not feedback: `corr(|Δsteer|, |Δκ|) = +0.89`
> vs `|Δepsi| +0.51`.

---

## What to tune (and what NOT to touch)

Correct lever — **κ estimate quality**, not feedback gains:

| priority | action | how | risk |
|-----------|-------------|-----|------|
| 1 | enable κ blend with yaw-rate: `mpc_kappa_yaw_blend` = 0.3–0.5 | `config.json` only, no rebuild | low — raises κ toward truth and removes lag (yaw is low-latency) |
| 2 | fix fit window/degree | C++ `path_lateral_state.cpp` (short window 1–12 m underestimates gentle arcs) | medium — discuss with team |
| 3 | small feedforward on curvature rate | C++ | medium |

**Do NOT** raise `ff_scale` globally — will over-steer on straights where κ is sometimes phantom.
**Do NOT** touch `epsi_gain` — tuned well now (straight weaving and epsi spike on arc entry already fixed).

> **Data caveat.** This run has few turns — real arcs (`|yaw|>0.05`) only ~1 %
> of clean frames (~6 s). Arc conclusion rests on few events — before rolling κ fix,
> confirm on a curvier run. On part of that "real turn" driver steered
> (gray bands on plot), so strict closed-loop MPC in arc barely captured on this run.

---

### Files
- MPC core: `app/src/main/cpp/src/visionpilot/lateral_planning.cpp`
- orchestration + debug publish: `app/src/main/cpp/src/services/lane_keep_service.cpp`
- CTE/epsi/κ estimate (poly fit): `app/src/main/cpp/src/utils/path_lateral_state.cpp`
- tuning config: `app/src/main/assets/config.json` (`vehicle` block)
- debug topic with controller internals: `control/lane_keep_debug` (`LaneKeepDebug`, per-frame)
- bug analysis in Python: `app/src/main/scripts/vis/bag_io.py` (`load_topic_messages`)
