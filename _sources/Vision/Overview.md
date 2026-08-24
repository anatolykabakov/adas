# Vision — Overview

In this module we want a **metric lane geometry** that a lateral controller can follow — not a pretty HUD overlay.
If the polyline is wrong, delayed, or expressed in the wrong $y$-sign, Pure Pursuit and MPC will look "broken" even when their math is fine.

```{figure} figures/iso8850_crop.png
---
width: 85%
---
Bird's-eye path $y(x)$ in meters. Controllers think in this plane: longitudinal $x$ ahead, lateral $y$ across the lane.
```

## What the controller needs

At each vision frame the stack should publish something equivalent to:

* a **reference path** $\{(x_i, y_i)\}$ in meters ahead of the vehicle;
* enough metadata to know whether paint / plan is trustworthy (probabilities, age);
* timestamps (`capture_ts`, `infer_ts`) so latency can be measured later.

In Phone ADAS that contract lives on topic `vision/lanes` (after `proto_convert` the C++ services see a polyline). Point source — live Supercombo, map, or bag replay — is **indistinguishable** to `pp` / `fp`. That is intentional: you can debug geometry offline before touching gains.

## The four things that have to be true, in order

A student who has trained a lane segmenter tends to assume perception is the hard part and the contract is
paperwork. In a phone stack it is the other way round. Four conditions have to hold before a controller
can follow anything, and they fail in this order of frequency:

| # | condition | how it fails | how you notice |
|---|---|---|---|
| 1 | the geometry is in **metres**, in a known frame | a sign flip in $y$ | the car steers confidently into the barrier |
| 2 | the geometry is **fresh** | vision stalls while the loop keeps commanding | logs look healthy, the reference is seconds old |
| 3 | the geometry is **trustworthy** where it is used | $\sigma$ summarised over a range the model never observed | a good line is discarded and the reference jumps to the model plan |
| 4 | the geometry is **metrically true** | the model reads distances 11 % short | every derived quantity inherits the scale |

Each of the four has cost this project a run, and each has a chapter or a section behind it. Condition 1 is
[Coordinates](./Coordinates.md). Conditions 3 and 4 are [Supercombo](./Supercombo.md). Condition 2 is
[Latency](../Latency/Overview.md) and the staleness gate below.

## Step 1: metres in a known frame

```python
# Both frames are used in this stack, and mixing them is condition 1 failing.
def iso_to_device_y(y_iso):
    """ISO vehicle frame has y left-positive; the model and our path have y right-positive."""
    return -y_iso

# A lane line 1.75 m to the driver's left.
print(f"ISO  y = {+1.75:+.2f} m (left)  ->  device y = {iso_to_device_y(+1.75):+.2f} m")
print("Get this backwards and the cross-track error changes sign, so the controller")
print("corrects away from the lane centre with full confidence.")
```

That is the whole of condition 1, and it is worth the three lines because it is the single most common way
a working controller looks broken.

## Step 2: freshness, and why a timeout is not enough

The obvious guard is "if the command is old, stop". It does not work, and the reason is instructive.

Our actuator already had a 250 ms HCA command timeout. It did not help, because the fast angle PID
publishes at 100 Hz regardless of how old the *reference* is — so the command was always fresh while the
path it was based on could be a minute old. In one measured stall `controls/steer` ran at a mean interval
of 9.9 ms while the plan was up to **75 seconds** stale.

```python
def reference_age_ms(capture_ts_ms, now_ms):
    """Age of the geometry the command is based on — not the age of the command."""
    return now_ms - capture_ts_ms

LANE_MAX_AGE_MS = 300           # vehicle.lane_max_age_s

now = 100_000
for capture in (now - 60, now - 250, now - 900, now - 75_000):
    age = reference_age_ms(capture, now)
    verdict = "ok" if age <= LANE_MAX_AGE_MS else "stale -> command cleared, PID reset"
    print(f"reference age {age:>6} ms  {verdict}")
```

The fix has to be keyed on the capture timestamp of the frame, which is why every message in this pipeline
carries one. On the run that exposed this, the gate withdraws control 16 % of the time — and that is the
correct behaviour, not a regression.

## Step 3: trustworthy where it is used

The model reports a per-sample $\sigma$ for each lane line, and blending weights the paint against the
model plan by it. The subtlety is *which samples you summarise*: $\sigma$ roughly doubles from the near
half of a 40 m window to the far half, because on a bend the inner line leaves the frame and its far
samples are extrapolation. Summarise too far and "I have not seen that far" vetoes a line whose near half
is fine — see [Supercombo](./Supercombo.md) for the measured table.

## Step 4: metrically true, and what to do when it is not

The model reads short against wheel speed, and *two different heads read short by two different amounts* —
which is the first thing to keep straight. The camera-odometry **distance** scale, after the focal-length
fix, is **0.888** (about 11 % short — [IntrinsicsAndWarp](../Calibration/IntrinsicsAndWarp.md)); the raw
**velocity/pose** head reads lower, **0.686** ([Supercombo](./Supercombo.md)). You cannot fix either from
the outside, so the discipline is to know which of your numbers inherit which. Lane *offsets* are ratios
within one frame and largely survive; anything comparing a model distance or speed against a wheel or GNSS
value does not.

## Build: a reference path from two noisy lines

The four conditions above are what a reference must satisfy. Now build one — from what a perception
model actually gives: two lane lines, each noisy, each with its own claimed σ, each free to disappear.
Synthesise the input honestly (σ grows with distance, because pixels farther ahead subtend more road):

```python
import numpy as np

rng = np.random.default_rng(0)
HALF = 1.75                       # half of a 3.5 m lane
X = np.arange(0.0, 60.0, 2.0)     # forward distance [m]

def centre_true(x):
    return 0.5 * 0.004 * x**2     # R = 250 m left arc

def sigma_of_x(x):
    return 0.05 + 0.01 * x        # 5 cm near the bumper, 65 cm at 60 m

def measure_lines():
    """One vision frame: (y_left, y_right, sigma) in metres, y left-positive."""
    sig = sigma_of_x(X)
    y_l = centre_true(X) + HALF + rng.normal(0, sig)
    y_r = centre_true(X) - HALF + rng.normal(0, sig)
    return y_l, y_r, sig
```

The naive planner picks one line and offsets by half a lane. The honest one weights both by how much
each claims to know — inverse variance, the same rule every estimator in this book runs on:

```python
def centre_from_one(y_line, side):
    return y_line - side * HALF          # side=+1 for the left line

def centre_fused(y_l, y_r, sig_l, sig_r):
    w_l, w_r = 1.0 / sig_l**2, 1.0 / sig_r**2
    return (w_l * (y_l - HALF) + w_r * (y_r + HALF)) / (w_l + w_r)

def rmse(y_est):
    return float(np.sqrt(np.mean((y_est - centre_true(X)) ** 2)))

frames = [measure_lines() for _ in range(200)]
e_one = np.mean([rmse(centre_from_one(l, +1)) for l, r, s in frames])
e_two = np.mean([rmse(centre_fused(l, r, s, s)) for l, r, s in frames])
print(f"one line: RMSE {e_one:.3f} m   fused: {e_two:.3f} m   ratio {e_two/e_one:.3f}")
assert e_two < 0.75 * e_one, "acceptance: fusion must beat a single line by the sqrt(2) class"
```

With equal σ, fusion halves the variance: the RMSE ratio sits near $1/\sqrt{2} \approx 0.707$. That
factor is the entire reason sensor fusion exists. Real lines also vanish — exits, worn paint, rain —
and the fallback is *the other line plus the width*, remembered from when both were visible:

```python
width_memory = np.median([np.median(l - r) for l, r, s in frames])  # learned while both were visible

def centre_one_line_fallback(y_l):
    return y_l - width_memory / 2.0

e_fallback = np.mean([rmse(centre_one_line_fallback(l)) for l, r, s in frames])
print(f"width memory: {width_memory:.2f} m (true 3.50)")
print(f"fallback on left line only: RMSE {e_fallback:.3f} m")
assert abs(width_memory - 2 * HALF) < 0.05, "acceptance: width learned to within 5 cm"
assert e_fallback < 1.2 * e_one, "acceptance: fallback must not be worse than trusting one line ever was"
```

The subtlest failure is a line that is *present and confident* but wrong — a tar seam, a truck's wet
track. Weighting by claimed σ pulls the fused path toward the liar:

```python
y_l, y_r, sig = frames[0]
y_l_liar = y_l + 0.8                              # left "line" found 0.8 m off, confidently
fused_honest = centre_fused(y_l, y_r, sig, sig)
fused_lied = centre_fused(y_l_liar, y_r, sig, sig)
shift = float(np.mean(fused_lied - fused_honest))
print(f"a confident 0.8 m liar shifts the fused path by {shift:+.3f} m")
assert 0.3 < shift < 0.5, "half the lie, because the weights were equal"
```

Half of the lie goes straight into the path: σ-weighting only defends against noise a sensor *admits
to*. The shipped fusion (`laneLinesToPath` in `utils/lane_path.cpp`, mirror `core/path_fusion.py`) adds
two defences you can now read with understanding — the **width filter** (2.6–4.6 m) and a **second
opinion** with different failure modes, the model's own plan
([MPC and fp](../Planner/MPC_and_FP.md), *lane / plan blending*). And your σ here was honest by
construction; the real one comes out of a network — on this project's drives the medians are 0.37 m
(left) and 0.51 m (right), and both lines are confident at once in only ~22 % of frames. The fallback
you just built is the common case, not the exception.

## Two pipelines (AAD vs this course)

| | AAD | Phone ADAS |
|---|---|---|
| Lane pixels | you train a segmenter | Supercombo heads (fixed ONNX) |
| Meters | IPM + camera height / RPY | model output in **device** frame + calib **input warp** |
| Contract | $y_l(x)$, $y_r(x)$ polynomials | `vision/lanes`: lanes / edges / plan / probs |
| Exercise focus | implement detector + IPM | read contract, stamps, failure modes |

```{note}
Training Supercombo is **out of scope**. You treat the network as a black box with a published layout and learn how our Java/C++ pipeline feeds and parses it.
```

## Chapters in this module

1. [Coordinate systems](./Coordinates.md) — device $y$ right+ vs ISO $y$ left+; the #1 source of inverted steering.
2. [Supercombo on device](./Supercombo.md) — warp → GPU or ORT → parse → publish; Hz and thermal.

Then you move to [Control](../Planner/Overview.md).

## Code map (read, do not memorize)

| piece | role |
|---|---|
| `ModelCalibWarp` | extrinsics-aware warp to 512×256 |
| `SupercomboThneedRunner` | supercombo 0.9.7 in fp16 on the GPU — the default |
| `SupercomboOnnxRunner` | the same network in fp32 through ONNX Runtime — the fallback |
| `LaneLines` / parse helpers | heads → polylines |
| `VisionPipeline` | drop policy when busy |
| `LaneOverlayView` | HUD only — not the control contract |

## Sanity checklist before blaming the controller

1. Vision rate $\gtrsim 9$ Hz on the window you evaluate.
2. Bird's-eye plot: left paint has the expected $y$ sign for the chosen frame.
3. Plan / lane blend looks continuous (no jumps of meters between frames).
4. Only then sweep `pp_*` or MPC weights.

<!-- next-chapter -->
---

**Next:** [Coordinate systems](./Coordinates.md)
