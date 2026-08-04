#!/usr/bin/env bash
# Build/install ADAS debug APK and run Traffic YOLO on-device bench.
# Usage: ./scripts/bench_traffic_yolo_phone.sh [iters=30] [warmup=5] [ep=auto] [model=256|192|320]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ITERS="${1:-30}"
WARMUP="${2:-5}"
EP="${3:-auto}"
SIZE="${4:-256}"
PKG=ai.flow.adas
ACT=.TrafficYoloBenchActivity
APK="$ROOT/app/build/outputs/apk/debug/app-debug.apk"
REPORT_REMOTE=/sdcard/adas_models/traffic_yolo_bench.txt
REPORT_LOCAL="/tmp/traffic_yolo_bench_phone.txt"
MODEL_SRC="$ROOT/models/yolov8n_traffic_${SIZE}.onnx"

cd "$ROOT"
adb wait-for-device
adb shell mkdir -p /sdcard/adas_models
if [[ ! -f "$MODEL_SRC" ]]; then
  echo "missing $MODEL_SRC"
  exit 1
fi
adb push "$MODEL_SRC" /sdcard/adas_models/traffic_yolo.onnx >/dev/null
echo "pushed $MODEL_SRC → /sdcard/adas_models/traffic_yolo.onnx"

# Skip full rebuild if APK already has TrafficYoloBench; rebuild when FORCE_BUILD=1
if [[ "${FORCE_BUILD:-1}" == "1" ]] || [[ ! -f "$APK" ]]; then
  echo "Building debug APK…"
  ./build_project.sh -t debug
  echo "Installing…"
  adb install -r "$APK"
fi

adb logcat -c
adb shell am force-stop "$PKG" || true
adb shell rm -f "$REPORT_REMOTE" || true
adb shell am start -n "$PKG/$ACT" --ei iters "$ITERS" --ei warmup "$WARMUP" --es ep "$EP"

echo "Waiting for bench DONE (ep=$EP size=$SIZE, up to 180s)…"
deadline=$((SECONDS + 180))
while (( SECONDS < deadline )); do
  if adb shell "test -f $REPORT_REMOTE && grep -q DONE $REPORT_REMOTE" 2>/dev/null; then
    break
  fi
  sleep 1
done

mkdir -p "$(dirname "$REPORT_LOCAL")"
adb pull "$REPORT_REMOTE" "$REPORT_LOCAL" 2>/dev/null || true
echo "==== report (ep=$EP size=$SIZE) ===="
cat "$REPORT_LOCAL" 2>/dev/null || adb logcat -d -s TrafficYoloBench:I | tail -40
