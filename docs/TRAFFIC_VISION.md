# Traffic vision — signs / lights / overspeed HUD

Low-frequency YOLO (~3 Hz) on a dedicated Java thread → `vision/traffic_dets` → C++ `TrafficSignService` → `traffic/state` → HUD (speed disc, light stack, overspeed border).

Does **not** share SupercomboInfer. Supercombo has no TFL/sign heads.

## Model download (you fetch)

### Quick start — COCO (traffic lights only)

Weights (already may exist as `models/yolov8n.pt`):

- **YOLOv8n COCO**: https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt

Export ONNX 320:

```bash
cd <repository root>
pip install ultralytics onnx onnxsim
./scripts/export_traffic_yolo.sh models/yolov8n.pt
# → models/yolov8n_traffic_320.onnx  (Gradle → assets/traffic_yolo.onnx)
```

Or push without rebuilding APK:

```bash
adb push models/yolov8n_traffic_320.onnx /sdcard/adas_models/traffic_yolo.onnx
```

COCO class `traffic light` → HSV color (R/Y/G). COCO also has `stop sign` and `person` (now drawn on HUD).

**Why no RU speed signs yet:** stock YOLOv8n is COCO-80 — there is no `speed_60` / `3_24_*` class. Those need a separately trained RTSD (or similar) checkpoint + matching `traffic_labels.txt`. Ready weights are usually on Kaggle / after you train; public GitHub repos often ship code without `.pt` in-tree.

HUD keep-list now: traffic light, stop/sign*, person, bicycle, motorcycle, any label that parses as speed limit.


### Ready weights (RU signs / lights)

| What | Link | Notes |
|-----|--------|---------|
| **RU signs `.pt` (direct download)** | https://github.com/nhassl3/Detect-russian-road-signs/releases/download/weights-first-train-v0.1.0/best.pt | YOLOv8, Roboflow RU signs; ~22 MB |
| **RU signs (doronin, 29 cls, GDrive)** | https://drive.google.com/file/d/1Kz4Iwc8lURpjwq1Om_z2NGfODRNX7PsC/view | Best public RTSD checkpoint from README; demo: [29 cls.](https://drive.google.com/file/d/12SndJXBaDCoJYB-sJqZxPP2ucQKplaSJ/view) |
| **Roboflow "Russian signs" + hosted model** | https://universe.roboflow.com/cchegeu/russian-signs/model/14 | 50 classes, can download dataset/YOLOv8 and/or API |
| **Skoltech RU signs dataset** | https://universe.roboflow.com/skoltech-zlr4k/yolo-v8-russian-road-signs | dataset for YOLOv8 |
| **RTSD (training dataset)** | https://www.kaggle.com/datasets/watchman/rtsd-dataset | source for most RU pipelines; codes `3_24`, `5_19_1`, … |
| **RTSD→YOLOv8 pipeline (classes `5_19_1`, `3_24`, …)** | https://github.com/medphisiker/drivers_helper | no weights in repo; `include_classes.txt` = codes we need (crosswalk, speed) |
| **RTSD training (notebooks)** | https://github.com/doronin99/RoadSignsDetection · https://github.com/trafficsurfer/YOLOv8 · https://github.com/kth-vyu/traffic_sign_detection_yolov8s | code + metrics; weights often GDrive only |
| **Pedestrian traffic light R/G (RU HF)** | https://huggingface.co/Aleton/trafficlight | YOLOv8n, 2 classes red/green — not vehicle TFL |
| **Vehicle TFL + color (not RU)** | https://drive.google.com/drive/folders/11nL-1PpbyIKa89_QKcn6R-Exl9IRWxpu | Satish-Vennapu YOLOv8; or our COCO `traffic light` + HSV |

Connecting to ADAS:

```bash
# example: nhassl3
curl -L -o models/ru_signs.pt \
  https://github.com/nhassl3/Detect-russian-road-signs/releases/download/weights-first-train-v0.1.0/best.pt
./scripts/export_traffic_yolo.sh models/ru_signs.pt
# put model class names in app/src/main/assets/traffic_labels.txt
adb push models/yolov8n_traffic_256.onnx /sdcard/adas_models/traffic_yolo.onnx
```

Runner already parses `3_24_*` / `speed_60` from the label. With an RU model, speed and `5_19_1` (pedestrian crossing) appear automatically if the class is named that way.

## Config

`config.json` (current default: traffic YOLO **off** while hunting lighter weights):

```json
"nodes": {
  "vision_traffic": false,
  "vision_traffic_signs": true,
  "vision_traffic_lights": true,
  "traffic_sign": true
},
"traffic_yolo_asset": "traffic_yolo.onnx"
```

| Flag | Effect |
|------|--------|
| `vision_traffic` | Master: load / run YOLO pipeline |
| `vision_traffic_signs` | Keep sign / speed / VRU dets + OCR |
| `vision_traffic_lights` | Keep TFL dets + HSV color |
| `traffic_sign` | C++ fusion → `traffic/state` HUD |

Pipeline starts only if master is on **and** at least one of signs/lights is on.
Both sub-flags off (or master false) → no YOLO thread.

### Lightweight candidates (next)

Current packaged RU `best.pt` → ONNX @256 is **~43 MB / ~11M params** → ORT med **~170 ms** under Supercombo (isolated ~84 ms). Too heavy.

| Candidate | Size / params | What it gives | Notes |
|-----------|---------------|---------------|--------|
| **COCO YOLOv8n @192/256** (`yolov8n.pt` already in repo, ONNX 13 MB) | ~3.2M | TFL + person + stop | Bench on 7T: **~22–29 ms** total @256 XNNPACK. No RU speeds. |
| **doronin RTSD yolov8n** (29 cls) | nano (claimed) | RU signs + generic speed | **Public Drive weights are 8-cls yolov8s (~11M), not 29-cls nano.** See `models/doronin/README.md`. Labels ready in `labels_29.txt`. |
| **HF Aleton/trafficlight** | YOLOv8n | ped TFL R/G only | Not vehicle TFL |
| Split: COCO nano (lights) + tiny sign head later | 2× nano | Fast path now | Prefer over single heavy RU mid |

Re-enable: set `"vision_traffic": true`, push lighter ONNX to `/sdcard/adas_models/traffic_yolo.onnx`.

OCR: template digits on speed-limit sign crops when label has no km/h (~1–5 ms).

Bag `vision/traffic_dets`: `prep_ms`, `ort_ms`, `decode_ms`, `ocr_ms`, `infer_ms`,
`capture_ts_ms`, `infer_ts_ms`, `ep`. PlotJuggler: `latency/traffic/*`.
HUD (optional): `yolo p/o/d/ocr` + `tot/e2e` under supercombo latency.


Overspeed if `v_ego > speed_limit + 5` km/h (margin in `TrafficSignService::Config`). Last speed limit held 60 s; TFL held 1.5 s.

## On-device bench (OnePlus 7T / HD1901)

```bash
./scripts/bench_traffic_yolo_phone.sh [iters] [warmup] [ep=auto|cpu|nnapi|xnnpack] [size=256|192|320]
```

**Finding:** for YOLOv8n, **CPU is much faster than NNAPI** on this phone (opposite of Supercombo).

| config | total med | ort med |
|--------|-----------|---------|
| 320 + NNAPI (old) | ~127 ms | ~100 ms |
| 256 + NNAPI | 117 ms | 90 ms |
| **256 + XNNPACK/CPU (new default)** | **29 ms** | **21 ms** |
| 192 + XNNPACK | 22 ms | 17 ms |

Default package: `models/yolov8n_traffic_256.onnx`. `auto` = XNNPACK → CPU (not NNAPI — YOLO regresses on NNAPI here).
