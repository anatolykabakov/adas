# Supercombo on Device

We want the phone to turn a windshield camera frame into the `vision/lanes` contract from the [overview](./Overview.md).
The network is a single ONNX model from the openpilot / flowpilot family (typically **v0.8.x**, output width 6472 in our build). Inference runs in **Java ONNX Runtime** — not thrneed / SNPE.

## Step 1: put the frame into the geometry the model expects

The network was trained on one camera in one mounting position. Ours is neither, so before inference the
frame is warped into openpilot's canonical "medmodel" geometry — 512×256, focal 910 px — with the learned
mounting angles taken out. That homography, why it depends on two separate calibrations, and what a 1.5°
remount does to it, are the subject of
[Three calibrations, not one](../Calibration/IntrinsicsAndWarp.md). Read that first if you have not.

Two consequences to carry into this chapter:

* the model sees a *derived* image, so its output inherits every calibration error upstream;
* the warp is not free. It costs 7 ms at the median and 35.7 ms at p95, and that tail is exactly where the
  reference ages past 150 ms.

### What happens to one frame


For each accepted camera frame:

1. Bitmap + `capture_ts` (time of exposure / delivery).
2. `ModelCalibWarp` → model input **$512\times 256$** (medmodel-compatible path).
3. Temporal stack + RNN state (two-frame context).
4. `OrtSession.run` → raw vector; record `infer_duration_ms`, `infer_ts`.
5. Parse heads → lane lines / edges, plan (MHP → best hypothesis), camera odometry, optional long.
6. Publish `vision/lanes` (optionally stash full `model_out` in the bag).

```{admonition} Drop policy
:class: warning
When the pipeline is busy, **new frames are dropped**. Measured Hz falls.
That is a vision / scheduling problem, not an invitation to raise MPC CTE weights.
```

## Step 2: run it, and decide how precisely

Inference is `OrtSession.run` on Android ONNX Runtime — not thneed or SNPE, which is why the `.thneed`
artifacts shipped by flowpilot and dragonpilot are of no use to us: they are precompiled programs for a
Snapdragon 845 GPU tied to their runtime.

The execution provider is chosen with a fallback chain: NNAPI in half precision if asked, then plain NNAPI,
then CPU. Each attempt builds fresh session options, because options cannot be reused after a failed
`createSession`.

Half precision is worth about 15–20 ms of the 45.6 ms, which is the single largest lever left in the
pipeline. It is **off by default**, and the reason is a measurement rather than caution: converting the whole
model to fp16 — a stricter test than NNAPI's per-node relaxation — moved the lane centre by 0.027 m and the
plan offset by 0.037 m, while line probabilities went *up* and the σ tail shrank. Not a degradation. But
per-frame disagreement was 0.05 m median and 0.20 m p95 on the lane centre, and there is another lateral
change queued, so it waits its turn as a single-variable experiment.

```bash
# The offline check, on frames where both lane lines are visible. No drive needed.
cd scripts
python3 bag_fp16_ab.py ../adas_logs/<session> --n 200 --t0 900 --t1 1100
```

### What healthy looks like

Medians from a 28-minute night run at 30 fps, no traffic YOLO (`2026_08_06_00_36_42`):

| quantity | median | p95 |
|---|---:|---:|
| frame prep (warp) | 7.0 ms | 35.7 |
| ORT `infer` | **45.6 ms** | 53.8 |
| e2e capture → model output | 54 ms | 81 |
| capture → steering command | 79 ms | 111 |
| publish rate | **13.24 Hz** | — |

Two ways to read those. The medians say the pipeline is healthy. The p95 of the warp — five times its
median — says the tail is where trouble lives, and it is the same frames on which the reference ages past
150 ms.

If you see inference at 300–500 ms and 2–3 Hz, that is **thermal or CPU contention**, not "the network is
usually that slow". Enabling the traffic YOLO detector does the same thing on purpose: it competes for the
same SoC, so turn it off before comparing controllers.

```{admonition} Vision stalls are still an open item
:class: warning
On a daytime run the reference was older than 300 ms for 15.9 % of the time, with one **75-second** hole,
while CAN held its 10 ms and the timers held theirs — so neither CPU nor scheduling explains it. The same
route at night ran clean (`frame_dt` max 119 ms, zero stale frames). Heat is the leading explanation and not
a proven one; the frame-arrival stamps added in 2026-08 exist to settle it.
```

## Step 3: read the output vector


The model returns one flat float array. Nothing in it is labelled, and the layout is the single most
misread thing in this project — so this section is a runnable map of it.

For our build (`sc_v0.8.12`, width 6472; the 6409 build has the same prefix):

| slice | contents |
|---|---|
| `[0 : 4955)` | **PLAN** — 5 trajectory hypotheses (MHP) |
| `[4955 : 5483)` | **LANES** — 4 lines: 264 floats of means, then 264 of log-sigmas |
| `[5483 : 5491)` | lane probability logits, 8 of them |
| `[5491 : 5755)` | **ROAD EDGES** — 2 edges: 132 means, then 132 log-sigmas |
| `[5755 : …)` | lead, desire, meta, pose, GRU state |

Two traps live in there.

**Trap one: `(y, z)` interleaving, not `(y, std)`.** Inside a line's 66 floats the values alternate
lateral position and height: index `i*2` is $y$, `i*2+1` is $z$. A widely copied demo reads them as
`(y, std)` and appears to work — because it happens to pick the even indices for $y$ — while its
"sigma" is actually the height of the lane line above the road. If your uncertainty looks suspiciously
smooth and small, this is why.

**Trap two: the sigmas are a second half, and they are logs.** The real $\sigma$ is $\exp(\cdot)$ of the
corresponding entry in the block's second half. Ours went unread for road edges until 2026-08-06, which
is why no bag could answer whether the edges were usable — and when they were finally read, they turned
out to be 7.5× noisier than the lane lines.

```python
import numpy as np

PLAN_END = 4955
LANE_STDS_START = PLAN_END + 264
LANES_END = PLAN_END + 528
LANE_PROB_END = LANES_END + 8
EDGE_STDS_START = LANE_PROB_END + 132
ROAD_END = LANE_PROB_END + 264

def sigmoid(v):
    return 1.0 / (1.0 + np.exp(-v))

def parse_lines(out):
    """4 lane lines and 2 road edges from the flat vector: y, z, sigma, probability."""
    out = np.asarray(out, dtype=np.float64).ravel()
    lanes = []
    for i in range(4):
        block = out[PLAN_END + i * 66 : PLAN_END + (i + 1) * 66]
        stds = out[LANE_STDS_START + i * 66 : LANE_STDS_START + (i + 1) * 66]
        lanes.append({
            "y": block[0::2].copy(),                              # index i*2   — lateral
            "z": block[1::2].copy(),                              # index i*2+1 — height, NOT sigma
            "sigma": np.exp(stds[0::2]),                          # second half of the block, and a log
            "prob": float(sigmoid(out[LANES_END + i * 2 + 1])),   # odd logit, per the official parser
        })
    edges = []
    for i in range(2):
        block = out[LANE_PROB_END + i * 66 : LANE_PROB_END + (i + 1) * 66]
        stds = out[EDGE_STDS_START + i * 66 : EDGE_STDS_START + (i + 1) * 66]
        edges.append({"y": block[0::2].copy(), "z": block[1::2].copy(), "sigma": np.exp(stds[0::2])})
    return lanes, edges

# The 33 sample distances the model always uses — quadratic, so 11 of them are inside the first second
# at 10 m/s. Worth knowing before you wonder why the far end is so sparse.
X_IDXS = np.array([0.0, 0.1875, 0.75, 1.6875, 3.0, 4.6875, 6.75, 9.1875, 12.0, 15.1875, 18.75,
                   22.6875, 27.0, 31.6875, 36.75, 42.1875, 48.0, 54.1875, 60.75, 67.6875, 75.0,
                   82.6875, 90.75, 99.1875, 108.0, 117.1875, 126.75, 136.6875, 147.0, 157.6875,
                   168.75, 180.1875, 192.0])

# Synthetic output: a 3.5 m lane, sigma growing with range, so the indexing can be checked round-trip.
out = np.zeros(6472)
for i, y_off in enumerate((-5.25, -1.75, 1.75, 5.25)):        # farLeft, left, right, farRight
    block = np.empty(66)
    block[0::2] = y_off                                        # y
    block[1::2] = 0.02                                         # z: 2 cm of road crown
    out[PLAN_END + i * 66 : PLAN_END + (i + 1) * 66] = block
    stds = np.empty(66)
    stds[0::2] = np.log(0.1 + 0.02 * X_IDXS)                   # sigma grows with range
    stds[1::2] = np.log(0.05)
    out[LANE_STDS_START + i * 66 : LANE_STDS_START + (i + 1) * 66] = stds
    out[LANES_END + i * 2 + 1] = 3.0 if i in (1, 2) else -3.0  # host lines confident

lanes, edges = parse_lines(out)
near = (X_IDXS >= 5) & (X_IDXS <= 20)
far = (X_IDXS >= 20) & (X_IDXS <= 40)
print(f"host lane width      : {lanes[2]['y'][0] - lanes[1]['y'][0]:.2f} m")
print(f"host line probability: left {lanes[1]['prob']:.2f}, right {lanes[2]['prob']:.2f}, "
      f"far {lanes[0]['prob']:.2f}")
print(f"left line z          : {lanes[1]['z'][0]:.3f} m  <- height, not sigma")
print(f"left sigma 5-20 m    : {np.median(lanes[1]['sigma'][near]):.2f} m")
print(f"left sigma 20-40 m   : {np.median(lanes[1]['sigma'][far]):.2f} m")
```

That last pair is not a toy detail. The blending confidence in `laneLinesToPath` summarises $\sigma$ over
a range, and on real data $\sigma$ roughly doubles between those two bands, because on a bend the inner
line leaves the frame and its far samples are extrapolation. Summarising over too long a range lets
"I have not seen that far" veto a line whose near half is fine.

### Picking the plan hypothesis


PLAN holds five hypotheses of 991 floats each: 33 timesteps × 15 columns of means, then 33 × 15 of
log-sigmas, then one selection logit. Take the hypothesis with the largest logit — **not** the mean of
them, which would average two different futures into a path through the kerb.

Per point the 15 columns are position (0–2), velocity (3–5), acceleration-ish (6–8), orientation (9–11)
and orientation rate (12–14). We read $y$ from column 1, $z$ from 2, and yaw / yaw rate from 11 and 14.

```python
PLAN_MHP_N, PLAN_COLS, PLAN_T = 5, 15, 33
HYP_SIZE = PLAN_T * PLAN_COLS * 2 + 1        # means, log-sigmas, one logit

def parse_plan(out):
    out = np.asarray(out, dtype=np.float64).ravel()
    logits = [out[i * HYP_SIZE + HYP_SIZE - 1] for i in range(PLAN_MHP_N)]
    best = int(np.argmax(logits))
    base = best * HYP_SIZE
    xs, ys, yaws = [], [], []
    for t in range(PLAN_T):
        row = base + t * PLAN_COLS
        xs.append(out[row + 0])
        ys.append(out[row + 1])
        yaws.append(out[row + 11])
    return best, np.array(xs), np.array(ys), np.array(yaws)

# Give hypothesis 2 the winning logit and a gentle left arc; leave the others straight.
for h in range(PLAN_MHP_N):
    base = h * HYP_SIZE
    out[base + HYP_SIZE - 1] = 2.5 if h == 2 else -1.0
    kappa = 0.004 if h == 2 else 0.0
    for t in range(PLAN_T):
        row = base + t * PLAN_COLS
        x = X_IDXS[t]
        out[row + 0] = x
        out[row + 1] = 0.5 * kappa * x * x
        out[row + 11] = kappa * x

best, px, py, pyaw = parse_plan(out)
print(f"chosen hypothesis {best}, y at 30 m = {np.interp(30.0, px, py):+.2f} m, "
      f"yaw there = {np.degrees(np.interp(30.0, px, pyaw)):+.1f}°")
```

```{admonition} Model output is shape, not metres
:class: warning
Camera odometry from this same network, compared against wheel speed, gives a scale of 0.888 — it reads
distances about 11 % short. Its *planned speed* is worse: `plan_v0 / v_ego` measured 0.678, and the ratio
moves with speed, so no single constant repairs it. That is why the longitudinal planner does not use the
model's speed as a target at all.
```

## Step 4: the rate you actually get

The parsed heads become `vision/lanes`, and the number that matters next is how often. It is not
$1000 / \text{work}$.

The pipeline holds a one-slot buffer of the newest frame and picks it up the instant it is free, so it never
idles by choice — but a cycle can only start when a frame has arrived. The rate is therefore quantised to
whole camera periods, $f = 1 / (T \lceil W/T \rceil)$, and [Latency](../Latency/Overview.md) works through
what that does to optimisation decisions. Measured: 44 ms period with 59 ms of work gives 11.29 Hz, exactly
two camera frames per processed one.

```{admonition} Drop policy
:class: warning
When the pipeline is busy, the pending frame is **overwritten**, not queued. Measured Hz falls. That is a
vision and scheduling problem, not an invitation to raise MPC CTE weights. Since 2026-08-06 the bag records
`frames_dropped`, `submit_ts_ms` and `pickup_ts_ms`, so a late frame and a slow inference are finally
distinguishable.
```

## Step 5: what control actually consumes


After `TopicConvert`, `LaneKeepService` sees a polyline (+ chassis). It does **not** know whether the path came from Supercombo, a map, or a replay script. Therefore:

1. Validate lines and latency **first**.
2. Only then tune `pp_*` / MPC / `fp` parameters.

Optional traffic YOLO shares the phone with ORT — enable it for demos, **disable** it when measuring lane-keep quality (`TRAFFIC_VISION.md`).

## Teaching limits


* Single phone camera (no dual-wide fusion).
* Desire / maneuver inputs are often zeros in our path.
* We do not retrain Supercombo in this course.

## Code to skim


`SupercomboOnnxRunner`, `ModelCalibWarp`, parse helpers under `scripts/core/supercombo_*.py`, overlay in `LaneOverlayView`.

## Exercise


1. On a bag, plot vision rate and `infer_ms` vs time (`tools/latency.py` / PlotJuggler).
2. Mark windows with Hz $< 9$. Do **not** use them for controller Pareto sweeps.
3. Optional: re-run offline parse on one JPEG with the shipped warp and compare lane $y$ at $x=10$ m to the logged polyline.

```{admonition} Discussion — vs AAD lane detection
:class: note
AAD has you train segmentation and run IPM. Here industry practice is closer to **one multitask network** plus calibration-aware warp. The pedagogical trade-off: less implementation of perception, more discipline about **contracts, stamps, and failure modes** that dominate real phone ADAS.
```

<!-- next-chapter -->
---

**Next:** [Localization](../Localization/Overview.md)
