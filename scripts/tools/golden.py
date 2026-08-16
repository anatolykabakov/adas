#!/usr/bin/env python3
"""Золотой прогон: снять эталон публикаций сервиса и сравнить с ним после правок.

Запись подаётся в `Simulated AdasApp` теми же сообщениями, что публикует телефон, и снимается то,
что сервис отдаёт наружу. Весь его эффект на мир — эти публикации, поэтому если после перекладки
кода они совпадают, машина поедет так же, что бы внутри ни переехало.

    ./golden.py record <бег> --out golden.npz
    ./golden.py check  <бег> --golden golden.npz
    ./golden.py record <бег> --types CameraCalibrationState --out calib.npz

Сравнение точное. Расхождение в последнем бите от перестановки операций — не повод ослаблять
допуск, а повод посмотреть, какое поле и на сколько разошлось: настоящая ошибка на одном кадре
тоже мала, а растёт со временем.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from pyadas import require_core
from vis.bag_io import load_topic_messages

pyadas = require_core()

CONFIG = (
    Path(__file__).resolve().parents[2]
    / "app"
    / "src"
    / "main"
    / "assets"
    / "config.json"
)

# Время публикации и метки приёма недетерминированы между прогонами: это часы, а не поведение.
SKIP = {
    "timestamp",
    "timestamp_us",
    "capture_ts_us",
    "vision_ts_us",
    "chassis_ts_us",
    "publish_ts_us",
    "capture_ts_ms",
    "vision_ts_ms",
    "chassis_ts_ms",
    "publish_ts_ms",
}


def snapshot(msg) -> dict:
    """Все скалярные поля сообщения. Список полей не зашит: иначе эталон сторожит только то,
    о чём вспомнил автор, а новые поля проходят мимо него незамеченными."""
    out = {}
    for name in dir(msg):
        if name.startswith("_") or name in SKIP:
            continue
        try:
            v = getattr(msg, name)
        except Exception:
            continue
        if isinstance(v, bool):
            out[name] = ("b", float(v))
        elif isinstance(v, (int, float)):
            out[name] = ("f", float(v))
        elif isinstance(v, str):
            out[name] = ("s", v)
    return out


def replay(bag: Path, types: set[str] | None = None) -> dict[str, np.ndarray]:
    veh = json.loads(CONFIG.read_text(encoding="utf-8")).get("vehicle", {})
    app = pyadas.AdasApp(float(veh.get("wheelbase_m", 2.636)), -1.8, 0.5, 1.10)
    app.set_lane_keep_controller(str(veh.get("lane_keep_controller", "fp")))
    app.set_lane_keep_max_steer_deg(float(veh.get("max_steer_deg", 20.0)))
    app.set_lane_keep_steer_slew_limit_deg(float(veh.get("steer_slew_limit_deg", 8.0)))
    app.set_lane_keep_vehicle_model(
        bool(veh.get("lat_use_vehicle_model", True)),
        float(veh.get("tire_stiffness_factor", 1.0)),
    )
    app.set_lane_keep_fp_steer_delay_s(float(veh.get("fp_steer_delay_s", 0.35)))
    app.set_lane_keep_pid_gains(
        float(veh.get("lat_pid_kp", 0.6)),
        float(veh.get("lat_pid_ki", 0.2)),
        float(veh.get("lat_pid_kf", 6e-5)),
    )
    app.set_param("steer_ratio", float(veh.get("steer_ratio", 15.7)))

    events: list[tuple[int, str, object]] = []
    for t, m, _ in load_topic_messages(bag, "vehicle/state"):
        events.append((t, "car", m))
    for t, m, _ in load_topic_messages(bag, "vision/lanes"):
        events.append((t, "lanes", m))
    events.sort(key=lambda e: e[0])
    if not events:
        raise SystemExit(f"в беге нет vehicle/state или vision/lanes: {bag}")

    t0 = events[0][0]
    rows: list[dict] = []
    for t, kind, m in events:
        ts_us = int((t - t0) * 1000)
        if kind == "car":
            app.publish_chassis(
                ts_us,
                float(m.v_ego),
                0.0,
                float(m.yaw_rate),
                float(m.steering_angle_deg),
                bool(m.steering_pressed),
            )
        else:
            xs = list(m.x)
            n = min(len(xs), len(m.plan_y) if m.plan_y else 0)
            if n >= 2:
                poly = [(float(a), float(b)) for a, b in zip(m.plan_x[:n], m.plan_y[:n])]
                app.publish_lanes(ts_us, poly, int(m.frame_id), poly, [], [], True)
        app.step(ts_us)
        for msg in app.pop_messages():
            if types and type(msg).__name__ not in types:
                continue
            r = snapshot(msg)
            r["__type"] = ("s", type(msg).__name__)
            rows.append(r)

    out: dict[str, np.ndarray] = {"n": np.array([len(rows)])}
    if not rows:
        return out
    keys = sorted({k for r in rows for k in r})
    for k in keys:
        kind = next(r[k][0] for r in rows if k in r)
        if kind == "s":
            out[k] = np.array([r.get(k, ("s", ""))[1] for r in rows], dtype="S48")
        else:
            out[k] = np.array([r[k][1] if k in r else np.nan for r in rows], dtype=float)
    return out


def compare(ref: dict, cur: dict, limit: int) -> int:
    bad = 0
    for k in sorted(set(ref) | set(cur)):
        if k not in ref or k not in cur:
            print(f"  {k}: есть только в {'эталоне' if k in ref else 'прогоне'}")
            bad += 1
            continue
        x, y = ref[k], cur[k]
        if x.shape != y.shape:
            print(f"  {k}: разная длина {x.shape} против {y.shape}")
            bad += 1
            continue
        if x.dtype.kind in "SU":
            diff = np.flatnonzero(x != y)
        else:
            diff = np.flatnonzero(~((x == y) | (np.isnan(x) & np.isnan(y))))
        if diff.size:
            i = int(diff[0])
            print(
                f"  {k}: расхождений {diff.size} из {x.size}, первое на кадре {i}: {x[i]!r} -> {y[i]!r}"
            )
            bad += 1
            if bad >= limit:
                print(f"  … остановлено после {limit} полей")
                break
    return bad


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("mode", choices=("record", "check"))
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, default=Path("lat_golden.npz"))
    ap.add_argument("--golden", type=Path, default=Path("lat_golden.npz"))
    ap.add_argument("--max-fields", type=int, default=12)
    ap.add_argument(
        "--types",
        default="",
        help="через запятую: какие публикации снимать (LaneKeepOutput, SteerCommand, ...). "
        "пусто — все, что отдаёт приложение",
    )
    args = ap.parse_args()

    want = {t.strip() for t in args.types.split(",") if t.strip()}
    cur = replay(args.bag, want or None)
    n = int(cur["n"][0])
    if args.mode == "record":
        np.savez_compressed(args.out, **cur)
        print(f"эталон записан: {args.out} ({n} публикаций, {len(cur)} полей)")
        return 0

    z = np.load(args.golden, allow_pickle=False)
    bad = compare({k: z[k] for k in z.files}, cur, args.max_fields)
    if bad:
        print(f"РАСХОЖДЕНИЕ: полей с отличиями {bad}")
        return 1
    print(f"совпадает поле в поле: {n} публикаций, {len(cur)} полей")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
