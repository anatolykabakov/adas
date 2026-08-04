#!/usr/bin/env bash
# Pull ADAS bags from phone → ./adas_logs/, then delete on device.
set -euo pipefail
REMOTE=/sdcard/adas_logs
# script lives in scripts/, bags go in project root
LOCAL="$(cd "$(dirname "$0")/.." && pwd)/adas_logs"
mkdir -p "$LOCAL"
adb pull "$REMOTE/." "$LOCAL/"
adb shell "rm -rf $REMOTE/*"
echo "pulled → $LOCAL"
