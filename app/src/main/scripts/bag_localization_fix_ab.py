#!/usr/bin/env python3
"""Траектория локализации до и после правок курса и пересева, поверх OSM.

Две ветки на одних и тех же данных заезда:

* **до правок** — поза, которую телефон посчитал во время заезда. Там курс обновлялся только внутри
  ветки принятой позиции, а защёлка курса и пересев позиции требовали стоянки;
* **после правок** — тот же поток ГНСС, IMU и шасси, прогнанный через нынешнюю сборку локализатора.

Реплей не побитово равен прогону на телефоне: порядок и тайминги сообщений на стенде свои. Поэтому
рядом печатаются две линейки — расстояние до собственных хороших фиксов приёмника и отклонение от
дороги OSM: если ветка «после» лучше по обеим, дело не в порядке сообщений.

  python3 bag_localization_fix_ab.py ../../../../adas_logs/2026_08_13_23_01_56 \\
      --out loc_ab.png --out-osm loc_ab_osm.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import _path  # noqa: F401
from bag_localization_replay import (
    align_to_fix_frame,
    load_inputs,
    offroad,
    recorded_pose,
    replay,
    to_good_fixes,
    _stat,
)
from vis.osm_layer import OsmLayer

ARMS = (
    ("до правок (запись телефона)", "#c62828", 1.8),
    ("после правок (реплей)", "#1565c0", 1.4),
)


def plot(ax, tracks, gps_xy=None, osm=None):
    allx = np.concatenate([t[0] for t in tracks])
    ally = np.concatenate([t[1] for t in tracks])
    if osm is not None and osm.ready:
        pad = 400.0
        xs, ys = osm.polylines(
            allx.min() - pad, ally.min() - pad, allx.max() + pad, ally.max() + pad
        )
        ax.plot(xs, ys, color="#c9ced6", linewidth=1.4, solid_capstyle="round", zorder=0)

    if gps_xy is not None:
        ax.plot(
            gps_xy[0],
            gps_xy[1],
            linestyle="none",
            marker=".",
            markersize=3,
            color="#6d4c41",
            alpha=0.55,
            zorder=1,
            label="фиксы ГНСС (точность ≤ 10 м)",
        )

    arms = ARMS if len(tracks) == len(ARMS) else ARMS[-len(tracks) :]
    for (x, y), (label, color, lw) in zip(tracks, arms):
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
    ap.add_argument("--out", type=Path, default=Path("localization_ab.png"))
    ap.add_argument("--out-osm", type=Path, default=None)
    ap.add_argument(
        "--only-after",
        action="store_true",
        help="одна ветка: реплей на нынешней сборке, без записи телефона",
    )
    args = ap.parse_args()

    gps, gx, gy, stream = load_inputs(args.bag)
    t_rec, x_rec, y_rec = recorded_pose(args.bag)
    x_rec, y_rec, off = align_to_fix_frame(gps, gx, gy, t_rec, x_rec, y_rec)
    print(f"кадр записи сдвинут на ({off[0]:.1f}, {off[1]:.1f}) м — вычтено")

    runs = [(ARMS[1][0], *replay(gps, gx, gy, stream, 25.0, True))]
    if not args.only_after:
        runs.insert(0, (ARMS[0][0], t_rec, x_rec, y_rec))
    for name, t, x, y in runs:
        L = np.hypot(np.diff(x), np.diff(y)).sum()
        net = np.hypot(x[-1] - x[0], y[-1] - y[0])
        print(f"{name:30s} {len(x):7d} поз, путь {L:8.0f} м, смещение {net:8.0f} м")

    osm = OsmLayer(args.map)
    have_map = osm.set_origin(float(gps[0][1].latitude), float(gps[0][1].longitude))
    if not have_map:
        print(f"нет карты: {osm.error}")

    print("\nдо хороших фиксов ГНСС (точность ≤ 10 м):")
    for name, t, x, y in runs:
        print(f"  {name:30s} {_stat(to_good_fixes(gps, gx, gy, t, x, y))}")
    if have_map:
        print("отклонение от дороги OSM:")
        for name, t, x, y in runs:
            print(f"  {name:30s} {_stat(offroad(osm, t, x, y))}")

    acc = np.array([m[1].horizontal_accuracy for m in gps])
    keep = (acc > 0) & (acc <= 10.0)
    cut = [(x, y) for _, _, x, y in runs]

    fig, ax = plt.subplots(figsize=(9, 12), dpi=110)
    ax.set_facecolor("#fafafa")
    plot(ax, cut, (gx[keep], gy[keep]))
    ax.set_title(f"{args.bag.name}: локализация до и после правок курса")
    fig.savefig(args.out, bbox_inches="tight")
    plt.close(fig)
    print(args.out)

    if args.out_osm and have_map:
        # В кадр карты переводится всё сразу, как это делает визуализатор: панель и карта — разные
        # проекции, и линеаризация между ними уводит на метры за длинный заезд.
        mcut = [osm.to_map(x, y) for x, y in cut]
        mg = osm.to_map(gx[keep], gy[keep])
        fig, ax = plt.subplots(figsize=(9, 12), dpi=110)
        ax.set_facecolor("#fafafa")
        plot(ax, mcut, mg, osm)
        ax.set_title(f"{args.bag.name}: локализация до и после правок, поверх OSM")
        fig.savefig(args.out_osm, bbox_inches="tight")
        plt.close(fig)
        print(args.out_osm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
