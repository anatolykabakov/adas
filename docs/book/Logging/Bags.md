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

## Report template

1. Session id and time window (or HCA on/off).
2. Vision rate, e2e med/p95; if available — thermal.
3. Quality metrics: |CTE| med/p95, |$\Delta$SWA|.
4. Conclusion: vision / control / thermal (explicitly separate).

<!-- next-chapter -->
---

**Next:** [Projects](../Exercises/StudentProjects.md)
