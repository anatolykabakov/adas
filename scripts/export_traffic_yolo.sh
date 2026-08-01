#!/usr/bin/env bash
# Export Ultralytics YOLOv8 → ONNX 320 for ADAS traffic pipeline.
# Usage:
#   ./scripts/export_traffic_yolo.sh              # uses models/yolov8n.pt (COCO)
#   ./scripts/export_traffic_yolo.sh path/to.pt  # custom RTSD / signs weights
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/models/yolov8n_traffic_320.onnx"
IMGSZ=320

if [[ -n "${1:-}" ]]; then
  SRC="$1"
elif [[ -f "$ROOT/yolov8n.pt" ]]; then
  SRC="$ROOT/yolov8n.pt"
elif [[ -f "$ROOT/models/yolov8n.pt" ]]; then
  SRC="$ROOT/models/yolov8n.pt"
else
  SRC="$ROOT/yolov8n.pt"
fi

if [[ ! -f "$SRC" ]]; then
  echo "Missing weights: $SRC"
  echo "Download COCO nano:"
  echo "  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt"
  echo "Place as $ROOT/yolov8n.pt then re-run this script."
  exit 1
fi
echo "Using weights: $SRC"

python3 - <<PY
from ultralytics import YOLO
m = YOLO("$SRC")
path = m.export(format="onnx", imgsz=$IMGSZ, simplify=True, opset=12)
print("exported:", path)
PY

# Ultralytics writes next to .pt; rename/copy to expected Gradle path
shopt -s nullglob
cands=("${SRC%.pt}.onnx" "${SRC%.*}.onnx")
# also search dir for newest onnx from this export
dir="$(dirname "$SRC")"
base="$(basename "${SRC%.pt}")"
if [[ -f "${dir}/${base}.onnx" ]]; then
  cp -f "${dir}/${base}.onnx" "$OUT"
elif [[ -f "${SRC%.pt}.onnx" ]]; then
  cp -f "${SRC%.pt}.onnx" "$OUT"
else
  echo "Could not find exported .onnx next to $SRC — check ultralytics output path"
  exit 1
fi

echo "OK → $OUT"
echo "Gradle packs it as assets/traffic_yolo.onnx on next build."
echo "Or: adb push $OUT /sdcard/adas_models/traffic_yolo.onnx"
