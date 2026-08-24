# Setup

## Repository

Clone it, then build — one command does the rest:

```bash
git clone https://github.com/anatolykabakov/adas.git
cd adas
./scripts/build_project.sh   # writes local.properties, fetches models, builds the APK
```

`<repository root>` in the rest of the book means this `adas/` directory.

**Prerequisites** the build assumes (a fresh Linux/macOS box needs these first —
`scripts/install_dependencies.sh` installs them on Debian/Ubuntu): a JDK (17+), the Android SDK and NDK
(`ANDROID_HOME` must point at an SDK that already has the `compileSdk` platform), Conan 2 and CMake for
the C++, and Python 3.10+ with `numpy`. Everything offline — bag analysis, the simulator, `pyadas` — needs
only the Python side and runs **without a phone or a car** (see [Bags](../Logging/Bags.md), *Record your
own bag*, for the smallest hardware path).

The repository carries no model. `scripts/fetch_models.sh` (which the build runs for you) downloads the
fp16 supercombo from comma's own release and derives the fp32 and `.thneed` variants, verifying every
sha256 against `scripts/models.manifest`. Run it alone when you only need the files.

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
