#!/bin/bash
# Push updated config.json to the device without losing calibration.
#
# Why this script. The APK copies assets/config.json to filesDir only if the file is not there yet
# (AdasAppHandler.ensureAssetCopied, force=false), and filesDir survives APK reinstall.
# So after `adb install -r` the device keeps the OLD config: new keys and new default values
# never arrive. Simply deleting the file is also wrong — the parameter panel writes camera
# calibration there (rpy, position, intrinsics), and it would be lost.
#
# What it does: starts from assets/config.json, copies from the device exactly the keys
# the parameter panel edits, shows the diff, and with --apply writes the result.
#
#   ./scripts/push_config.sh              # show what would change only
#   ./scripts/push_config.sh --apply      # write to device
#   ./scripts/push_config.sh --apply --serial <id>
#   ./scripts/push_config.sh --apply --set calibration.camera.intrinsics_prior='{"fx":993.4}'
#
# --set path=value overrides a key ON TOP of protection: normally device calibration is preserved
# (written by the parameter panel), and without --set you cannot push new calibration to the device.
# The value is parsed as JSON, so you can pass a whole object.

set -euo pipefail

PKG="ai.flow.adas"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET="${PROJECT_DIR}/app/src/main/assets/config.json"
# Absolute path: `run-as pkg sh -c` does not start in the app directory, and a relative
# `files/...` does not resolve — write failed with "No such file or directory".
REMOTE="/data/data/${PKG}/files/config.json"

APPLY=false
SERIAL=()
OVERRIDES=()
while [ $# -gt 0 ]; do
  case "$1" in
    --apply)  APPLY=true; shift ;;
    --serial) SERIAL=(-s "$2"); shift 2 ;;
    --set)    OVERRIDES+=("$2"); shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

adb "${SERIAL[@]}" get-state >/dev/null 2>&1 || { echo "device not connected (adb devices)" >&2; exit 1; }
[ -f "$ASSET" ] || { echo "missing $ASSET" >&2; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# Config from device. Empty/garbage means it does not exist yet — assets goes as-is.
adb "${SERIAL[@]}" shell run-as "$PKG" cat "$REMOTE" 2>/dev/null > "$TMP/device.json" || true

python3 - "$ASSET" "$TMP/device.json" "$TMP/merged.json" ${OVERRIDES[@]+"${OVERRIDES[@]}"} <<'PY'
import json, sys

asset_path, device_path, out_path = sys.argv[1:4]
overrides = sys.argv[4:]

# Keys edited by the parameter panel that must be preserved from the device.
KEEP = [
    ("calibration", "camera", "position_m"),
    ("calibration", "camera", "rpy_deg"),
    ("calibration", "camera", "intrinsics_prior"),
    ("logging", "record_camera_images"),
    ("vehicle", "wheelbase_m"),
    ("vehicle", "steer_ratio"),
    ("vehicle", "pp_k_dd"),
    ("vehicle", "pp_ld_min"),
    ("vehicle", "pp_ld_max"),
    ("vehicle", "pp_shift"),
    ("vehicle", "lane_keep_controller"),
    ("supercombo_asset",),
]

with open(asset_path, encoding="utf-8") as f:
    new = json.load(f)

try:
    with open(device_path, encoding="utf-8") as f:
        dev = json.load(f)
    if not isinstance(dev, dict):
        raise ValueError("not an object")
except Exception as e:
    dev = None
    print(f"no readable config on device ({e}) — will use assets/config.json as-is")


def get(root, path):
    cur = root
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return None, False
        cur = cur[k]
    return cur, True


def put(root, path, value):
    cur = root
    for k in path[:-1]:
        cur = cur.setdefault(k, {})
    cur[path[-1]] = value


def flat(d, prefix=""):
    out = {}
    for k, v in d.items():
        if k.startswith("comment"):
            continue
        if isinstance(v, dict):
            out.update(flat(v, prefix + k + "."))
        else:
            out[prefix + k] = v
    return out


kept = []
if dev is not None:
    for path in KEEP:
        val, ok = get(dev, path)
        if ok:
            put(new, path, val)
            kept.append(".".join(path))

# --set is applied AFTER protected keys are copied, so it always wins.
for ov in overrides:
    if "=" not in ov:
        raise SystemExit(f"--set expects path=value, got: {ov}")
    key, raw = ov.split("=", 1)
    try:
        val = json.loads(raw)
    except Exception:
        val = raw
    put(new, tuple(key.split(".")), val)
    print(f"--set: {key} = {val}")

before = flat(dev) if dev is not None else {}
after = flat(new)

changed = [(k, before.get(k, "—"), v) for k, v in after.items() if before.get(k, "—") != v]
gone = [k for k in before if k not in after]

if kept:
    print("copied from device (calibration and parameter panel):")
    for k in kept:
        print(f"  {k}")
print(f"\nkeys to change: {len(changed)}")
for k, old, val in sorted(changed):
    print(f"  {k}: {old} → {val}")
if gone:
    print(f"\nwill disappear on device (not in assets): {', '.join(sorted(gone))}")
if not changed and not gone:
    print("device config already matches assets — nothing to write")

with open(out_path, "w", encoding="utf-8") as f:
    json.dump(new, f, ensure_ascii=False, indent=2)
    f.write("\n")
PY

if ! $APPLY; then
  echo
  echo "dry run. to write: ./scripts/push_config.sh --apply"
  exit 0
fi

# Write via /data/local/tmp and `run-as cp`: adb push cannot go straight into /data/data, and
# redirection inside `run-as pkg sh -c "cat > ..."` is blocked by SELinux (Permission denied),
# while read and `cp` with the same run-as are allowed — verified on OnePlus 7T / Android 12.
# Write to a temp file first, then mv — a mid-write failure will not leave a broken config.
adb "${SERIAL[@]}" shell run-as "$PKG" mkdir -p "$(dirname "$REMOTE")"

STAGE="/data/local/tmp/adas_config_push.json"
adb "${SERIAL[@]}" push "$TMP/merged.json" "$STAGE" >/dev/null
adb "${SERIAL[@]}" shell run-as "$PKG" cp "$STAGE" "${REMOTE}.tmp"
adb "${SERIAL[@]}" shell run-as "$PKG" mv "${REMOTE}.tmp" "${REMOTE}"
adb "${SERIAL[@]}" shell rm -f "$STAGE"
# No redirection: `sh -c "wc -c < ..."` under run-as also hits SELinux.
SIZE="$(adb "${SERIAL[@]}" shell run-as "$PKG" wc -c "${REMOTE}" | awk '{print $1}' | tr -d '\r')"
echo
echo "written: ${REMOTE} (${SIZE} bytes). Restart the app — config is read at startup."
