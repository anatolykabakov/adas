# License

## Code: Apache-2.0. Book and documentation: CC BY 4.0

The code in this repository is licensed under the **Apache License 2.0** — the full text is in
[`LICENSE`](LICENSE), and the notices that license requires travel in [`NOTICE`](NOTICE). Apache was
chosen over MIT for two things it adds: an explicit patent grant, and the `NOTICE` mechanism — both
matter for code that may end up in an ECU.

The course book under [`docs/book/`](docs/book/) and [`docs/book_ru/`](docs/book_ru/), together with the
documentation in `docs/`, is licensed under **CC BY 4.0**. That is not a free choice: some figures are
derived from [Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving),
which is CC BY 4.0 itself, so the derived work carries the same terms. Attribution per chapter is in
[`docs/book/ATTRIBUTION.md`](docs/book/ATTRIBUTION.md).

Contributions are accepted under the same terms as the file being changed — Apache-2.0 for code, CC BY 4.0
for the book — which is what Apache-2.0 §5 already says by default. No separate CLA.

## What the project license does not cover

**The assets under `app/src/main/assets/` are not ours.** Whatever license the code receives, it does not
extend to these files:

| file | copyright holder and license |
|---|---|
| `supercombo.onnx`, `supercombo.thneed` | Comma.ai, Inc. — MIT. Source: [openpilot v0.9.7](https://github.com/commaai/openpilot/raw/v0.9.7/selfdrive/modeld/models/supercombo.onnx), fp16, widened to fp32 by us; `supercombo.thneed` is generated from that ONNX and remains derived from their weights. Provenance and the hashes that establish it — `THIRD_PARTY.md` |
| `traffic_yolo.onnx` — **no longer shipped** | derived from YOLOv8n, Ultralytics — **AGPL-3.0** or a commercial license. Removed from `assets/` on 2026-08-18; the detector code stays and loads it from `/sdcard/adas_models/` if you supply one |
| `Moscow.osm.admap` | derived from OpenStreetMap data — **ODbL**, © OpenStreetMap contributors. The 84 MB `.pbf` extract it is built from is no longer shipped — the app never read it |
| `vw_mqb_2010.dbc`, `toyota_nodsu_pt_generated.dbc` | opendbc, Comma.ai, Inc. — MIT |

None of these has to live in the repository: `app/build.gradle` takes them from `models/` and `maps/`, both
gitignored, and `./scripts/fetch_models.sh` brings them in. Then whoever is entitled to distribute them does
so, and we do not.

Their copyright notices, reproduced as those licenses require, are in [`NOTICE`](NOTICE). Gradle copies that
file into the APK as `assets/NOTICE.txt`, because an APK is a copy of its own and the repository's notice
does not travel with it.

**Vendored third-party code** under `app/src/main/cpp` keeps its own licenses: the thneed headers from
Comma.ai (MIT), json11 from Dropbox (MIT), and a kernel UAPI header (GPL-2.0 WITH Linux-syscall-note).

**Dependencies** built by conan and gradle keep theirs: BSD-2/BSD-3, MIT, MPL-2.0, Apache-2.0, and LGPL-2.1
for libusb — which we link **statically**, with the obligation that follows from its §6. The full list, and
what each one requires, is in [`THIRD_PARTY.md`](THIRD_PARTY.md).

## Warning

This is research code that steers a car. It is not a product, it is not certified, and it comes with no
warranty of any kind. Complying with local law, and whatever happens in the vehicle, is the responsibility
of whoever runs it. The driver must supervise the controls at all times.
