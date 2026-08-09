# Bag and Offline Analysis

Session — topic directory with protobuf fragments:

```text
adas_logs/<session>/
  vision__lanes/
  control__lane_keep/
  phone__stats/
  ...
```

`/` in topic name → `__` on disk.

## Commands

```bash
cd <repository root>
./scripts/run_bag_vis.sh /path/to/session

cd app/src/main/scripts
./generate_proto_python.sh
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 latency.py /path/to/session
python3 vis/export_to_plotjuggler.py /path/to/session -o /tmp/out
```

## Key topics

| topic | question |
|---|---|
| `vision/lanes` | geometry, Hz, e2e |
| `control/lane_keep_debug` | CTE, $\kappa$, `frame_dt_ms` |
| `controls/steer` | actuation command |
| `vehicle/state` | $v$, actual SWA |
| `phone/stats` | CPU / thermal |
| `middleware/stats` | native timer lag |

## Reading a bag yourself

`load_topic_messages(session, topic)` returns a list of `(timestamp_ms, payload, envelope)`. That is the
whole API. Everything in `app/src/main/scripts` is built on it, and so is every analysis in this course.

```bash
cd app/src/main/scripts
export PYTHONPATH=.
```

```python
# not-runnable — needs a real session on disk
from pathlib import Path
import numpy as np
from vis.bag_io import load_topic_messages, list_topics

session = Path("adas_logs/2026_08_06_00_36_42")
print(list_topics(session))

lanes = load_topic_messages(session, "vision/lanes")
state = load_topic_messages(session, "vehicle/state")
print(f"{len(lanes)} vision frames, {len(state)} chassis samples")

ts, msg, _ = lanes[100]
print("fields:", [f.name for f in msg.DESCRIPTOR.fields])
```

Three things bite everybody once, so they are worth knowing before rather than after.

**Pass a `Path`, not a string.** `load_topic_messages` joins with `/`, so a `str` raises
`TypeError: unsupported operand type(s) for /`. Obvious in hindsight, opaque in the moment.

**Do not align by scanning.** The helper `nearest(ts, series)` is a linear search, which is fine for a
handful of lookups and quadratic for a whole run — aligning 22 000 vision frames against 140 000 chassis
samples that way takes hours. Sort once and binary-search.

**Deduplicate timestamps.** The recorder writes several topics on the same millisecond, so a topic can
contain repeated stamps. Left in, a repeat makes `dt` zero and any derivative `inf` or `nan` — and then a
downstream gate like `abs(accel) < 0.5` silently rejects every sample. That exact mistake cut 1081 GPS
samples to 8 in one afternoon.

```python
import numpy as np

def align(t_ref, t_src, v_src, max_dt_ms=120.0):
    """Nearest-sample lookup by binary search, with a staleness limit and a validity mask."""
    idx = np.clip(np.searchsorted(t_src, t_ref), 0, len(t_src) - 1)
    prev = np.clip(idx - 1, 0, len(t_src) - 1)
    take_prev = np.abs(t_src[prev] - t_ref) < np.abs(t_src[idx] - t_ref)
    idx = np.where(take_prev, prev, idx)
    return v_src[idx], np.abs(t_src[idx] - t_ref) <= max_dt_ms

def drop_repeated_stamps(t, *arrays):
    """Keep only strictly increasing timestamps, so derivatives stay finite."""
    keep = np.concatenate(([True], np.diff(t) > 0))
    return (t[keep],) + tuple(a[keep] for a in arrays)

# A 1 Hz source (GPS) against a 100 Hz one (chassis), with a duplicate stamp planted at index 3.
t_gps = np.array([0, 1000, 2000, 2000, 3000, 4000], dtype=float)
v_gps = np.array([10.0, 11.0, 12.0, 12.0, 12.5, 13.0])
t_gps, v_gps = drop_repeated_stamps(t_gps, v_gps)
print("after dedup:", t_gps)

t_can = np.arange(0, 4001, 10, dtype=float)
v_can = 10.0 + 0.75e-3 * t_can
can_at_gps, ok = align(t_gps, t_can, v_can)
print("aligned:", np.round(can_at_gps[ok], 2), "valid", ok.sum(), "of", len(ok))

# Now the derivative that used to blow up is finite.
accel = np.gradient(v_gps) / np.maximum(np.gradient(t_gps) * 1e-3, 1e-3)
print("gps accel:", np.round(accel, 3), "all finite:", bool(np.isfinite(accel).all()))
```

## Measuring something real: agreement between two sensors

The pattern that produces most of the findings in this project: take two independent measurements of the
same quantity, look at their **ratio** rather than their difference, and check whether the ratio depends
on anything. A constant ratio is a calibration error. A ratio that varies with speed or load is physics.

```python
rng = np.random.default_rng(3)
speeds = rng.uniform(5.0, 25.0, 500)
truth = speeds
wheel = truth * 1.012 + rng.normal(0, 0.05, 500)      # 1.2 % scale plus noise
doppler = truth + rng.normal(0, 0.03, 500)

ratio = wheel / doppler
print(f"scale: median {np.median(ratio):.4f}  p10 {np.percentile(ratio, 10):.4f}  "
      f"p90 {np.percentile(ratio, 90):.4f}")
print(f"{'band':>12} {'n':>5} {'scale':>8}")
for lo, hi in ((5, 10), (10, 15), (15, 20), (20, 25)):
    m = (doppler >= lo) & (doppler < hi)
    print(f"{f'{lo}-{hi} m/s':>12} {m.sum():>5} {np.median(ratio[m]):>8.4f}")
print("flat across bands -> a constant to calibrate; sloping -> something physical to model")
```

Run on the real bags this reports 1.0117 and 1.0120 on two separate runs, flat across every band. That is
how the wheel-radius error described in [Localization](../Localization/Overview.md) was found, and it is
the same method used for the coast-deceleration envelope and the model's metric scale.

```{admonition} Report the gates, always
:class: warning
Every measurement above throws samples away — slow, transient, bad fix, stale alignment. State how many
survived. "Scale is 1.012" over eight samples and over 798 are different claims, and the first one has
been wrong here before.
```

## Report template

1. Session id and time window (or HCA on/off).
2. Vision rate, e2e med/p95; if available — thermal.
3. Quality metrics: |CTE| med/p95, |$\Delta$SWA|.
4. Conclusion: vision / control / thermal (explicitly separate).

<!-- next-chapter -->
---

**Next:** [Projects](../Exercises/StudentProjects.md)
