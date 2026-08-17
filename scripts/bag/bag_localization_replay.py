#!/usr/bin/env python3
"""Replay a bag's GNSS / IMU / wheel speeds through the real C++ localizer and plot it over OSM.

Three tracks, because two are not enough to tell a fix from a change:

* **запись** — the pose the phone produced during the drive. What actually happened;
* **гейт выкл** — this build with ``gps_max_accuracy_m = 0``, i.e. every fix admitted. This arm exists
  to validate the harness: if it cannot land near the recorded track, nothing measured here means
  anything. It is not bit-for-bit the old code — the innovation window was ``clamp(4·acc, 10, 60)``
  then and is ``clamp(2·acc, 10, 30)`` now, and that part is not switchable;
* **гейт вкл** — the same build with the accuracy gate and accuracy-weighted R.

Two yardsticks, since either alone can be gamed. Distance to the nearest OSM road rewards a track that
follows *a* road, which a straightened track along a corridor does by accident. Distance to the
receiver's own good fixes (accuracy ≤ 10 m) says whether the track is in the right place.

  python3 bag/bag_localization_replay.py ../adas_logs/2026_08_11_09_49_43 \
      --out loc.png --out-osm loc_osm.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import _path  # noqa: F401
from pyadas import core as pyadas
from core.gps_utils import gps_to_local_coords
from vis import bag_io
from vis.osm_layer import OsmLayer

# assets/config.json vehicle.steer_ratio
STEER_RATIO = 15.6

# The phone's yaw axis on this mount. `sensors/imu_yaw` — what the on-device ImuCalibrator publishes and
# what the filter subscribes to — is not in the bag, so the axis has to be picked here. Measured on
# 2026_08_11_09_49_43 against the ESP yaw rate: gyro_x r=+0.999 slope +1.022, gyro_y -0.43, gyro_z -0.52.
IMU_YAW_AXIS = "gyro_x"


def recorded_pose(bag: Path):
    msgs = bag_io.load_topic_messages(bag, "localization/pose")
    t = np.array([m[0] for m in msgs], dtype=np.int64)
    return t, np.array([m[1].x for m in msgs]), np.array([m[1].y for m in msgs])


def load_inputs(bag: Path):
    gps = bag_io.load_topic_messages(bag, "sensors/gps/location")
    imu = bag_io.load_topic_messages(bag, "sensors/imu")
    ch = bag_io.load_topic_messages(bag, "vehicle/state")
    if not gps or not ch:
        raise SystemExit("bag has no GNSS or no vehicle state")

    lat = np.array([m[1].latitude for m in gps])
    lon = np.array([m[1].longitude for m in gps])
    tcol = np.array([m[0] for m in gps], dtype=float)
    gx, gy = gps_to_local_coords(np.column_stack([tcol, lat, lon]), origin_idx=0)

    stream = (
        [(m[0], "gps", i) for i, m in enumerate(gps)]
        + [(m[0], "imu", m[1]) for m in imu]
        + [(m[0], "chassis", m[1]) for m in ch]
    )
    stream.sort(key=lambda e: e[0])
    return gps, gx, gy, stream


def replay(gps, gx, gy, stream, gate_m: float, scale_noise: bool = True):
    app = pyadas.PyAdasApp()
    app.set_param("gps_max_accuracy_m", float(gate_m))
    # Взвешивание шума позиции по точности фикса больше не переключается — оно безусловно.
    app.reset_localization()

    t_out, x_out, y_out = [], [], []
    for ts, kind, payload in stream:
        if kind == "gps":
            # Фикс уходит как есть, в градусах: сервис локализации сам проецирует его в метры своей
            # `gps_proj_`. Подавать сюда уже спроецированные метры значило кормить фильтр не тем
            # сообщением — подписчика на такое нет, и ГНСС до фильтра просто не доходил.
            app.publish_gps_location(gps[payload][1].SerializeToString())
        elif kind == "imu":
            # Сырой IMU: `publish_imu` кладёт готовую скорость рыска в топик, который никто не слушает.
            app.publish_imu_data(payload.SerializeToString())
        else:
            # steer_rad must be real: the filter gates the gyro on agreement with the bicycle-model yaw
            # rate (|yr - yr_bicycle| < 0.35 rad/s), so a zero steering angle pins that estimate to zero
            # and rejects every turn sharper than 20 deg/s — the replay then drives straight.
            app.publish_chassis(
                int(ts) * 1000,
                float(payload.v_ego),
                np.radians(float(payload.steering_angle_deg)) / STEER_RATIO,
                float(payload.yaw_rate),
                float(payload.steering_angle_deg),
                bool(payload.steering_pressed),
            )
        app.step(int(ts) * 1000)
        for out in app.pop_messages():
            if isinstance(out, pyadas.LocalizationPose):
                t_out.append(ts)
                x_out.append(float(out.x))
                y_out.append(float(out.y))

    return np.array(t_out, dtype=np.int64), np.array(x_out), np.array(y_out)


def offroad(osm, t, x, y, cap=120.0):
    """Distance to the nearest OSM road — a shape yardstick, blind to being on the wrong road."""
    idx = np.linspace(0, len(x) - 1, min(1200, len(x))).astype(int)
    mx, my = osm.to_map(x[idx], y[idx])
    d = []
    for i in range(len(mx)):
        r = osm.nearest_edge(float(mx[i]), float(my[i]), cap)
        d.append(r[1] if r else cap)
    return np.array(d)


def to_good_fixes(gps, gx, gy, t, x, y, max_acc=10.0):
    """Distance from the track to the receiver's own trustworthy fixes, matched in time."""
    tg = np.array([m[0] for m in gps], dtype=float)
    acc = np.array([m[1].horizontal_accuracy for m in gps])
    k = (acc > 0) & (acc <= max_acc)
    if not k.any():
        return np.array([])
    xi = np.interp(tg[k], t.astype(float), x)
    yi = np.interp(tg[k], t.astype(float), y)
    return np.hypot(xi - gx[k], yi - gy[k])


def align_to_fix_frame(gps, gx, gy, t, x, y, max_acc=10.0):
    """Put a recorded track into the replay's frame.

    The app anchors its ENU plane at its own origin, not at this bag's first logged fix, so the recorded
    pose is a rigid translation away from `gps_to_local_coords(origin_idx=0)`. On 2026_08_11_09_49_43 the
    offset is (82.7, 435.9) m and constant to 0.3 m over 1236 s. Comparing the two frames directly
    measures that translation and nothing else — it read as a 444 m error.
    """
    tg = np.array([m[0] for m in gps], dtype=float)
    acc = np.array([m[1].horizontal_accuracy for m in gps])
    k = (acc > 0) & (acc <= max_acc)
    if not k.any():
        return x, y, (0.0, 0.0)
    dx = float(np.median(np.interp(tg[k], t.astype(float), x) - gx[k]))
    dy = float(np.median(np.interp(tg[k], t.astype(float), y) - gy[k]))
    return x - dx, y - dy, (dx, dy)


def _stat(d):
    if len(d) == 0:
        return "нет данных"
    return f"med={np.median(d):5.1f} p90={np.percentile(d, 90):6.1f} max={d.max():6.1f}"


ARMS = (
    ("запись на телефоне", "#c62828", 1.7),
    ("реплей, гейт выкл", "#ef9a3c", 1.2),
    ("реплей, гейт вкл", "#1565c0", 1.2),
)


def _plot(ax, tracks, osm=None):
    allx = np.concatenate([t[0] for t in tracks])
    ally = np.concatenate([t[1] for t in tracks])
    if osm is not None and osm.ready:
        pad = 400.0
        xs, ys = osm.polylines(
            allx.min() - pad, ally.min() - pad, allx.max() + pad, ally.max() + pad
        )
        ax.plot(xs, ys, color="#c9ced6", linewidth=1.4, solid_capstyle="round", zorder=0)

    for (x, y), (label, color, lw) in zip(tracks, ARMS):
        ax.plot(x, y, color=color, linewidth=lw, zorder=2, label=label, alpha=0.9)
        ax.plot(x[-1], y[-1], marker="s", color=color, markersize=6, zorder=3)
    ax.plot(
        tracks[0][0][0],
        tracks[0][1][0],
        marker="o",
        color="#2e7d32",
        markersize=9,
        zorder=4,
        label="старт",
    )

    pad = 100.0
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_aspect("equal")
    ax.grid(alpha=0.2, linewidth=0.5)
    ax.legend(loc="upper left", fontsize=9)
    ax.set_xlabel("восток, м")
    ax.set_ylabel("север, м")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", type=Path)
    ap.add_argument("--map", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=Path("localization.png"))
    ap.add_argument("--out-osm", type=Path, default=None)
    ap.add_argument("--tail-s", type=float, default=0.0)
    args = ap.parse_args()

    gps, gx, gy, stream = load_inputs(args.bag)
    t_rec, x_rec, y_rec = recorded_pose(args.bag)
    x_rec, y_rec, off = align_to_fix_frame(gps, gx, gy, t_rec, x_rec, y_rec)
    print(f"кадр записи сдвинут на ({off[0]:.1f}, {off[1]:.1f}) м — вычтено")
    runs = [
        ("запись на телефоне", t_rec, x_rec, y_rec),
        ("реплей, гейт выкл", *replay(gps, gx, gy, stream, 0.0, False)),
        ("реплей, гейт вкл", *replay(gps, gx, gy, stream, 25.0, True)),
    ]
    for name, t, x, y in runs:
        L = np.hypot(np.diff(x), np.diff(y)).sum()
        net = np.hypot(x[-1] - x[0], y[-1] - y[0])
        print(f"{name:20s} {len(x):7d} поз, путь {L:7.0f} м, смещение {net:7.0f} м")

    osm = OsmLayer(args.map)
    have_map = osm.set_origin(float(gps[0][1].latitude), float(gps[0][1].longitude))
    if not have_map:
        print(f"нет карты: {osm.error}")

    print("\nдо хороших фиксов ГНСС (точность ≤ 10 м):")
    for name, t, x, y in runs:
        print(f"  {name:20s} {_stat(to_good_fixes(gps, gx, gy, t, x, y))}")
    if have_map:
        print("отклонение от дороги OSM:")
        for name, t, x, y in runs:
            print(f"  {name:20s} {_stat(offroad(osm, t, x, y))}")

    if args.tail_s > 0:
        cut = [
            (
                x[(t[-1] - t) / 1000.0 <= args.tail_s],
                y[(t[-1] - t) / 1000.0 <= args.tail_s],
            )
            for _, t, x, y in runs
        ]
    else:
        cut = [(x, y) for _, _, x, y in runs]

    fig, ax = plt.subplots(figsize=(8, 13), dpi=110)
    ax.set_facecolor("#fafafa")
    _plot(ax, cut)
    ax.set_title(f"{args.bag.name}: гейт по точности ГНСС")
    fig.savefig(args.out, bbox_inches="tight")
    plt.close(fig)
    print(args.out)

    if args.out_osm and have_map:
        # Everything moves into the map frame the way the visualizer does it: the panel frame and the map
        # frame are different projections, and linearising between them drifts metres over a long run.
        mcut = [osm.to_map(x, y) for x, y in cut]
        fig, ax = plt.subplots(figsize=(8, 13), dpi=110)
        ax.set_facecolor("#fafafa")
        _plot(ax, mcut, osm)
        ax.set_title(f"{args.bag.name}: то же на подложке OSM")
        fig.savefig(args.out_osm, bbox_inches="tight")
        plt.close(fig)
        print(args.out_osm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
