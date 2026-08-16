# Setup

## Repository

Android ADAS root:

```bash
cd <repository root>
git lfs install && git lfs pull   # the models and the map are LFS objects
```

Without that second line the network assets are 133-byte pointer files and the app starts without a model.

See also `README.md` at the repository root: APK build, `run_bag_vis.sh`, `run_sim.sh`, Java/C++/scripts layout.

## Build book (HTML)

```bash
cd docs
python3 -m venv .venv && source .venv/bin/activate
pip install -r book/requirements.txt
./build_book.sh          # both languages: EN into _site, RU into _site/ru
./build_book.sh --en     # English only, faster while writing
./build_book.sh --check  # checks without building
# Live: https://anatolykabakov.github.io/adas/ (CI deploy-book on main)
```

The **ENG/RU** button at the top takes you to the same page in the other language. The `_site` and
`_site/ru` layout is the contract the button relies on rather than an implementation detail, and the build
script separately verifies that every page has a counterpart — otherwise the switch would land on a 404.

## Python for bag

```bash
cd scripts
# protobuf stubs:
./generate_proto_python.sh
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 tools/latency.py /path/to/adas_logs/SESSION
```

## AAD (optional)

For camera chapters "from scratch" and interactive PP exercises in CARLA:

[Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving)

Recommended order: AAD Lane Detection overview + Control/PP → this book (Architecture → Control → Latency) → Exercises A1/B1.

## Phone

* Android arm64, USB OTG + Panda;
* models come from the APK assets (`supercombo.onnx`, `supercombo.thneed`); `/sdcard/adas_models/` overrides
  them when you want to test a different file without rebuilding;
* new phone: `python3 tools/model_device_probe.py --iters 50` before anything else — see `docs/NEW_PHONE.md`;
* logging → `/sdcard/adas_logs/…`;
* for teaching measurements: `vision_traffic: false`, `phone_stats: true`.

<!-- next-chapter -->
---

**Next:** [Reading list and internal docs](./ReadingList.md)
