# Setup

## Repository

Android ADAS root:

```bash
cd <repository root>
```

See also root [`README.md`](../../../README.md): APK build, `run_bag_vis.sh`, `run_sim.sh`, Java/C++/scripts layout.

## Build book (HTML)

```bash
cd docs/book
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
jupyter-book build .
# HTML → _build/html/index.html
# Live: https://anatolykabakov.github.io/adas/ (CI deploy-book on main)
```

## Python for bag

```bash
cd app/src/main/scripts
# protobuf stubs:
./generate_proto_python.sh
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 latency.py /path/to/adas_logs/SESSION
```

## AAD (optional)

For camera chapters "from scratch" and interactive PP exercises in CARLA:

[Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving)

Recommended order: AAD Lane Detection overview + Control/PP → this book (Architecture → Control → Latency) → Exercises A1/B1.

## Phone

* Android arm64, USB OTG + Panda;
* model `/sdcard/adas_models/supercombo.onnx` or assets;
* logging → `/sdcard/adas_logs/…`;
* for teaching measurements: `vision_traffic: false`, `phone_stats: true`.

<!-- next-chapter -->
---

**Next:** [Reading list and internal docs](./ReadingList.md)
