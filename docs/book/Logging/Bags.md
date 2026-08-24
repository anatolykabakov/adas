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

## Build: the format in forty lines

A bag is topic directories of length-prefixed records. That is the whole idea; everything else is
protobuf. Write the toy version so the real one holds no mystery:

```python
import json
import struct
import tempfile
from pathlib import Path

def topic_dir(root, topic):
    return Path(root) / topic.replace("/", "__")     # '/' would nest; '__' keeps one level

def write_bag(root, topic, records):
    d = topic_dir(root, topic)
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "000.bin", "wb") as f:
        for r in records:
            blob = json.dumps(r).encode()
            f.write(struct.pack("<I", len(blob)))    # length prefix: records survive a truncated tail
            f.write(blob)

def read_bag(root, topic):
    out = []
    for part in sorted(topic_dir(root, topic).glob("*.bin")):
        data = part.read_bytes()
        i = 0
        while i + 4 <= len(data):
            (n,) = struct.unpack_from("<I", data, i)
            if i + 4 + n > len(data):
                break                                # the app was killed mid-record; keep what is whole
            out.append(json.loads(data[i + 4 : i + 4 + n]))
            i += 4 + n
    return out

root = tempfile.mkdtemp()
recs = [{"ts": 1000 + 42 * k, "y_l": -1.7, "y_r": 1.8} for k in range(100)]
write_bag(root, "vision/lanes", recs)
back = read_bag(root, "vision/lanes")
print(f"wrote 100, read {len(back)}, first ts {back[0]['ts']}, dir {topic_dir(root, 'vision/lanes').name}")
assert back == recs, "acceptance: a bag must survive the round trip byte-exactly"
```

The real format differs in two ways only: records are protobuf `ZMQMessage` (the exact bytes that
crossed the bridge — logging is a *tap*, not a re-encoding), and files rotate by size. `Logger.java`
writes it; `scripts/vis/bag_io.py` reads it; the `'/'→'__'` rule above is lifted from there verbatim.

## Commands

```bash
cd <repository root>
./scripts/run_bag_vis.sh /path/to/session

cd scripts
./generate_proto_python.sh
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 tools/latency.py /path/to/session
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
whole API. Everything in `scripts` is built on it, and so is every analysis in this course.

```bash
cd scripts
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

## Record your own bag, and run your own code on it

No car is needed for a first bag: a phone on the windshield as a passenger is best, a walk along a road
with painted edges works too. Build (`./scripts/build_project.sh`), install, start logging from the UI,
move for 3–5 minutes, pull the session (`./scripts/pull_bags.sh`), open it in the visualizer. What your
bag cannot contain — `vehicle/state`, `can/rx`, torque — is a feature: everything a controller outputs
on it is unverifiable, so you can be as wrong as you like, for free.

Then feed it to the code you built in [Vision](../Vision/Overview.md) and
[Pure Pursuit](../Planner/PurePursuit.md):

```python
# not-runnable — needs your session directory and generated protobufs
from pathlib import Path
import numpy as np
from vis.bag_io import load_topic_messages

session = Path("../adas_logs/<your-session>")
lanes = load_topic_messages(session, "vision/lanes")
print(len(lanes), "lane frames")

for ts, msg, _raw in lanes[:3]:
    left, right = msg.lanes[1], msg.lanes[2]      # near-left and near-right host lines
    print(ts, len(left.points), "pts  prob", round(left.prob, 2), round(right.prob, 2))
```

```python
# not-runnable — sketch of the analysis loop
deltas, gaps, sigmas = [], [], []
for ts, msg, _ in lanes:
    left, right = msg.lanes[1], msg.lanes[2]
    if left.prob < 0.5 or right.prob < 0.5:
        gaps.append(ts)                             # the frames your toy never had
        continue
    path = fuse_from_proto(left, right)             # your step-3 code, adapted to proto points
    delta, _ = pure_pursuit(path, 0.0, 0.0, 0.0, 8.0, 0)
    deltas.append(delta)
    sigmas.append((median_std(left), median_std(right)))

print("frames:", len(lanes), " usable:", len(deltas), " dropped:", len(gaps))
print("|delta| p95 [deg]:", np.degrees(np.percentile(np.abs(deltas), 95)))
```

Expect the shape, if not the numbers: σ nothing like a synthetic 5 cm (medians 0.37/0.51 m on this
project's drives, both lines confident at once in ~22 % of frames); whole unflagged stretches with
neither line; a 42 ms frame step with jitter. Write down the three worst moments — a gap, a σ blow-up,
a frame-step spike, one timestamp and one sentence each. Those three timestamps are your personal test
set from now on.

## Report template

1. Session id and time window (or HCA on/off).
2. Vision rate, e2e med/p95; if available — thermal.
3. Quality metrics: |CTE| med/p95, |$\Delta$SWA|.
4. Conclusion: vision / control / thermal (explicitly separate).

<!-- next-chapter -->
---

**Next:** [Projects](../Exercises/StudentProjects.md)
