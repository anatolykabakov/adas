# Supercombo on Device

We want the phone to turn a windshield camera frame into the `vision/lanes` contract from the [overview](./Overview.md).
The network is a single ONNX model from the openpilot / flowpilot family (typically **v0.8.x**, output width 6472 in our build). Inference runs in **Java ONNX Runtime** — not thrneed / SNPE.

## Frame pipeline

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

## Nominal metrics (healthy session)

Example without traffic YOLO (`01_14_22`):

| quantity | typical |
|---|---:|
| ORT `infer` | ~47 ms |
| e2e capture → infer | ~62 ms |
| publish rate | ~11.4 Hz |

If you see infer $300$–$500$ ms at $2$–$3$ Hz, treat it as **thermal / CPU contention**, not "the network is usually that slow".

## What control actually consumes

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

1. On a bag, plot vision rate and `infer_ms` vs time (`latency.py` / PlotJuggler).
2. Mark windows with Hz $< 9$. Do **not** use them for controller Pareto sweeps.
3. Optional: re-run offline parse on one JPEG with the shipped warp and compare lane $y$ at $x=10$ m to the logged polyline.

```{admonition} Discussion — vs AAD lane detection
:class: note
AAD has you train segmentation and run IPM. Here industry practice is closer to **one multitask network** plus calibration-aware warp. The pedagogical trade-off: less implementation of perception, more discipline about **contracts, stamps, and failure modes** that dominate real phone ADAS.
```

<!-- next-chapter -->
---

**Next:** [Control — overview](../Control/Overview.md)
