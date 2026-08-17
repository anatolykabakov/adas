#!/usr/bin/env python3
"""Which vision runners this phone can use, how fast, and whether the numbers match.

On "compiling the model for a phone". It cannot be compiled ahead of time, and that is a property of
the approach rather than an omission. There are two runners:

* **ONNX** — portable. Runs on any Android with onnxruntime, nothing has to be built for the device.
  Slower: 44.7 ms against 15.9 on our OnePlus 7T.
* **thneed** — fast. Not a model but a **recorded run of the network on the GPU**: a list of kernels
  with their sources, the launch order and the buffer contents. Upstream openpilot replays such a
  recording at the Adreno driver's ioctl layer, but **our loader does not** — it creates ordinary
  OpenCL buffers, builds kernels from sources and launches them (the ioctl block is not compiled here,
  and `libthneedrunner.so` contains not a single `kgsl` string). So Adreno is not required; what is
  required is an OpenCL the app can reach, with `cl_khr_fp16` and images over buffers.

The script therefore does not compile but **finds out**: what the hardware is, which runners come up
at all, at what latency, and whether the output matches a reference phone. The last matters more than
speed: a runner that ran but produced different numbers is worse than one that failed — a failure
announces itself, different numbers do not.

Usage::

    python3 tools/model_device_probe.py                       # measure the attached phone
    python3 tools/model_device_probe.py --save ref.json       # store as a reference
    python3 tools/model_device_probe.py --compare ref.json    # compare against a reference
    python3 tools/model_device_probe.py --serial <id>         # when several phones are attached

The phone must be attached over adb with the app installed.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time

BENCH_ACTIVITY = "adas.app/adas.app.ui.SupercomboBenchActivity"
BENCH_OUT = "/sdcard/adas_models/supercombo_bench.json"

# Below this, a file on the device is definitely unusable.
MIN_MODEL_BYTES = 1_000_000

# Below this vision rate the setpoint step grows enough to visibly wander on curves (docs/VISION_RATE.md).
MIN_USABLE_HZ = 25.0


def adb(args: list[str], serial: str | None = None, timeout: int = 120) -> str:
    cmd = ["adb"] + (["-s", serial] if serial else []) + args
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if out.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}: {out.stderr.strip() or out.stdout.strip()}")
    return out.stdout


def getprop(name: str, serial: str | None) -> str:
    try:
        return adb(["shell", "getprop", name], serial).strip()
    except Exception:
        return ""


def probe_device(serial: str | None) -> dict:
    """What this phone is and what it can do, before running any model."""
    props = {
        "model": getprop("ro.product.model", serial),
        "brand": getprop("ro.product.brand", serial),
        "board": getprop("ro.board.platform", serial),
        "soc_manufacturer": getprop("ro.soc.manufacturer", serial),
        "soc_model": getprop("ro.soc.model", serial),
        "android": getprop("ro.build.version.release", serial),
        "abi": getprop("ro.product.cpu.abi", serial),
    }

    # The GPU vendor comes from SurfaceFlinger, which prints it at startup.
    gpu = ""
    try:
        dump = adb(["shell", "dumpsys", "SurfaceFlinger"], serial)
        m = re.search(r"GLES:\s*([^\n]+)", dump)
        if m:
            gpu = m.group(1).strip()
    except Exception:
        pass
    props["gpu"] = gpu

    # The OpenCL library: without it thneed will not come up even on Adreno.
    opencl = ""
    for path in (
        "/vendor/lib64/libOpenCL.so",
        "/system/vendor/lib64/libOpenCL.so",
        "/vendor/lib64/egl/libGLES_mali.so",
    ):
        try:
            adb(["shell", "ls", path], serial)
            opencl = path
            break
        except Exception:
            continue
    props["opencl_lib"] = opencl

    return props


def run_bench(serial: str | None, iters: int, warmup: int) -> dict:
    """Run the measurement on the device and fetch the report."""
    try:
        adb(["shell", "rm", "-f", BENCH_OUT], serial)
    except Exception:
        pass
    adb(["shell", "am", "force-stop", "adas.app"], serial)
    adb(
        [
            "shell",
            "am",
            "start",
            "-n",
            BENCH_ACTIVITY,
            "--ei",
            "iters",
            str(iters),
            "--ei",
            "warmup",
            str(warmup),
        ],
        serial,
    )

    # The model takes a while to load and the thneed is 51 MB; wait for the file rather than a fixed pause.
    deadline = time.time() + 300
    while time.time() < deadline:
        time.sleep(5)
        try:
            raw = adb(["shell", "cat", BENCH_OUT], serial)
        except Exception:
            continue
        if raw.strip().endswith("}"):
            return json.loads(raw)
    raise TimeoutError(
        "the measurement did not finish in 5 minutes; see logcat -s SupercomboBench"
    )


def signatures_match(a: dict, b: dict) -> tuple[bool, str]:
    """The same runner on two phones must give the same numbers."""
    if a.get("len", 0) == 0 or b.get("len", 0) == 0:
        return False, "empty output"
    if a["len"] != b["len"]:
        return False, f'output length {a["len"]} against {b["len"]}'
    # The tolerance is not zero: different arithmetic on different hardware changes the last digits.
    # But a mean differing by whole percent means different predictions, not rounding.
    if abs(a["mean"] - b["mean"]) > max(1e-3, abs(b["mean"]) * 0.01):
        return False, f'mean {a["mean"]:.5f} against {b["mean"]:.5f}'
    if abs(a["std"] - b["std"]) > max(1e-3, abs(b["std"]) * 0.01):
        return False, f'spread {a["std"]:.5f} against {b["std"]:.5f}'
    return True, "matches"


def report(device: dict, bench: dict, reference: dict | None) -> int:
    print(
        f'\n{device["brand"]} {device["model"]} — {device["soc_manufacturer"]} {device["soc_model"]}'
        f' ({device["board"]}), Android {device["android"]}, {device["abi"]}'
    )
    print(f'  GPU: {device["gpu"] or "not detected"}')
    print(f'  OpenCL: {device["opencl_lib"] or "not found"}')

    # The app reports the OpenCL capabilities itself: guessing them from the SoC name is pointless,
    # and whether libOpenCL is reachable by an app is decided by the vendor via public.libraries.
    cl = bench.get("opencl") or {}
    if cl.get("opencl"):
        print(f'  OpenCL device: {cl.get("device", "?")} ({cl.get("version", "?")})')
        print(
            f'    fp16: {"yes" if cl.get("fp16") else "NO — the fast path will not build"}'
            f'   images over buffers: {"yes" if cl.get("image2d_from_buffer") else "NO"}'
        )
        print(
            f'    row pitch alignment: {cl.get("pitch_align_px", "?")} px'
            f'   work group up to {cl.get("max_work_group", "?")}'
        )
    else:
        print(
            f'  OpenCL from the app: UNAVAILABLE ({cl.get("reason", "no reason reported")})'
        )
        print("    the fast path is ruled out, ONNX is what remains")

    usable = []
    unverifiable = []
    print(f'\n{"runner":10s} {"median":>9s} {"p95":>8s} {"Hz":>6s}  output')
    for r in bench.get("runners", []):
        if not r.get("ok"):
            print(
                f'{r["name"]:10s} {"—":>9s} {"—":>8s} {"—":>6s}  did not start: {r.get("reason", "")[:60]}'
            )
            continue
        hz = 1000.0 / r["median_ms"] if r["median_ms"] > 0 else 0.0
        sig = r.get("signature", {})
        print(
            f'{r["name"]:10s} {r["median_ms"]:8.1f}m {r["p95_ms"]:7.1f}m {hz:6.1f}  '
            f'len={sig.get("len", 0)} mean={sig.get("mean", 0):.4f}'
        )
        if sig.get("len", 0) == 0:
            unverifiable.append(r["name"])
        if hz >= MIN_USABLE_HZ:
            usable.append((r["median_ms"], r["name"]))

    problems = 0
    if reference:
        print("\ncomparison with the reference:")
        ref_by_name = {r["name"]: r for r in reference.get("runners", []) if r.get("ok")}
        for r in bench.get("runners", []):
            if not r.get("ok"):
                continue
            ref = ref_by_name.get(r["name"])
            if not ref:
                print(
                    f'  {r["name"]}: absent from the reference, nothing to compare against'
                )
                continue
            ok, why = signatures_match(r.get("signature", {}), ref.get("signature", {}))
            print(
                f'  {r["name"]}: {"output matched" if ok else "OUTPUT DIVERGED — " + why}'
            )
            if not ok:
                problems += 1

    print()
    if not usable:
        print(
            f"VERDICT: the phone cannot keep up. No runner reaches {MIN_USABLE_HZ:.0f} Hz — "
            f"on a curve that shows as visible wandering."
        )
        return 1
    usable.sort()
    best = usable[0][1]
    if problems:
        print(
            f"VERDICT: {best} is fastest, BUT its output diverged from the reference — do not drive on it."
        )
        print(
            "Investigate before installing: the runner ran, but it predicts something else."
        )
        return 1
    if best in unverifiable:
        print(
            f"VERDICT: {best} is fastest ({usable[0][0]:.1f} ms), but it has no output signature —"
        )
        print(
            "  nothing to compare against a reference phone. The runner ran; whether the numbers are"
        )
        print(
            "  the right ones is unknown. It can be installed, but look at the lane lines on the road"
        )
        print("  with your own eyes before trusting them.")
        return 1
    print(f'VERDICT: set vision.model_runner = "{best}" ({usable[0][0]:.1f} ms).')
    if best == "onnx":
        print(
            "  The fast path either did not come up here or lost. The reason is in the OpenCL lines"
        )
        print(
            "  above and in logcat: both runners check themselves with a zero-input run and refuse to"
        )
        print(
            "  work when they compute the wrong thing — the refusal is visible, not silent."
        )
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--serial", help="serial number when several phones are attached")
    ap.add_argument("--iters", type=int, default=50, help="runs per runner")
    ap.add_argument("--warmup", type=int, default=5, help="warm-up runs")
    ap.add_argument("--save", metavar="FILE", help="store the result as a reference")
    ap.add_argument(
        "--compare", metavar="FILE", help="compare the output against a reference"
    )
    args = ap.parse_args()

    try:
        device = probe_device(args.serial)
    except Exception as e:
        print(f"the phone does not respond: {e}", file=sys.stderr)
        return 2

    print(
        f'measuring {device["brand"]} {device["model"]}, {args.iters} runs per runner...'
    )
    try:
        bench = run_bench(args.serial, args.iters, args.warmup)
    except Exception as e:
        print(f"the measurement failed: {e}", file=sys.stderr)
        return 2

    reference = None
    if args.compare:
        with open(args.compare, encoding="utf-8") as f:
            reference = json.load(f)

    code = report(device, bench, reference)

    if args.save:
        bench["probe"] = device
        with open(args.save, "w", encoding="utf-8") as f:
            json.dump(bench, f, indent=2, ensure_ascii=False)
        print(f"\nreference written: {args.save}")
    return code


if __name__ == "__main__":
    sys.exit(main())
