#!/usr/bin/env python3
"""Эксперимент: ранжировать кандидатов по цепочке манёвров, а не по профилю курса.

Проверяет гипотезу с разбора бегов 2026-08-08: смещение нуля датчика поворота (0.15 °/с)
разрушает глобальный курс — за 16 км накапливается 420° при истинных 173°, — но локальные
углы поворотов остаются точными (медиана расхождения с ГНСС 4.8°), а расстояния по одометрии
точны до 1.2 %. Значит подпись «угол поворота + расстояние до следующего» устойчива там, где
профиль курса уже бесполезен.

  python3 -m mapmatch.rerank_by_chain <bag>
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from mapmatch.locate import DEFAULT_MAP, route_streets
from mapmatch.run_cache import load as load_run

mm = pyadas.mapmatch


def yaw_bias_rps(t: np.ndarray, v: np.ndarray, w: np.ndarray) -> float:
    """Смещение нуля: медиана скорости поворота на стоянках, иначе на прямых."""
    at_rest = v[:-1] < 0.3
    if at_rest.sum() > 50:
        return float(np.median(w[:-1][at_rest]))
    straight = (v[:-1] > 8.0) & (np.abs(w[:-1]) < np.radians(1.5))
    return float(np.median(w[:-1][straight])) if straight.sum() > 50 else 0.0


def gnss_in_map_frame(bag: Path, road_map) -> Optional[np.ndarray]:
    """Точки ГНСС в системе координат карты — иначе сравнивать с маршрутами нельзя."""
    if "sensors/gps/location" not in list_topics(bag):
        return None
    rows = load_topic_messages(bag, "sensors/gps/location")
    pts = [
        (float(m.latitude), float(m.longitude))
        for _, m in [(r[0], r[1]) for r in rows]
        if abs(float(m.latitude)) > 1e-6
    ]
    if len(pts) < 5:
        return None
    frame = road_map.frame
    return np.array([frame.to_local(a, b) for a, b in pts])


def route_polyline(road_map, dir_edges) -> Tuple[np.ndarray, np.ndarray]:
    """Геометрия маршрута в порядке движения."""
    xs: List[float] = []
    ys: List[float] = []
    for de in dir_edges:
        px, py = road_map.edge_polyline(de >> 1)
        px, py = list(px), list(py)
        if de & 1:
            px, py = px[::-1], py[::-1]
        if xs and abs(px[0] - xs[-1]) < 1e-6 and abs(py[0] - ys[-1]) < 1e-6:
            px, py = px[1:], py[1:]
        xs.extend(px)
        ys.extend(py)
    return np.asarray(xs), np.asarray(ys)


def chain_from_polyline(
    x: np.ndarray, y: np.ndarray, min_turn_deg: float, merge_gap_m: float
) -> List[Tuple[float, float]]:
    """Цепочка (угол поворота, путь от предыдущего поворота) по геометрии маршрута."""
    if len(x) < 3:
        return []
    seg = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    head = np.unwrap(np.arctan2(np.diff(y), np.diff(x)))
    dtheta = np.degrees(np.diff(head))
    # Узлы полилинии несут мелкие изломы; поворотом считается накопленный угол на коротком пути.
    turns: List[Tuple[float, float]] = []  # (позиция, угол)
    acc = 0.0
    acc_from = 0.0
    for i, d in enumerate(dtheta):
        pos = s[i + 1]
        if acc != 0.0 and pos - acc_from > merge_gap_m:
            if abs(acc) >= min_turn_deg:
                turns.append((acc_from, acc))
            acc = 0.0
        if acc == 0.0:
            acc_from = pos
        acc += d
    if abs(acc) >= min_turn_deg:
        turns.append((acc_from, acc))

    chain: List[Tuple[float, float]] = []
    prev = 0.0
    for pos, ang in turns:
        chain.append((ang, pos - prev))
        prev = pos
    return chain


def track_chain(track) -> List[Tuple[float, float]]:
    """То же для трека: угол манёвра и путь от конца предыдущего поворота."""
    chain: List[Tuple[float, float]] = []
    prev_end = 0.0
    for man in track.maneuvers:
        if not man.is_turn:
            continue
        chain.append((float(man.angle_deg), float(man.s_start_m) - prev_end))
        prev_end = float(man.s_end_m)
    return chain


def align_cost(
    track: List[Tuple[float, float]],
    route: List[Tuple[float, float]],
    turn_tol_deg: float,
    dist_tol_rel: float,
    skip_penalty: float,
) -> float:
    """ДП-выравнивание двух цепочек; повороты маршрута можно сливать и пропускать."""
    if not track or not route:
        return float("inf")

    n, m = len(track), len(route)
    big = 1e9
    dp = np.full((n + 1, m + 1), big)
    dp[0, :] = 0.0  # маршрут может начинаться раньше трека

    for i in range(1, n + 1):
        ang_t, gap_t = track[i - 1]
        for j in range(1, m + 1):
            # пропуск поворота маршрута (лишний узел карты)
            best = dp[i, j - 1] + skip_penalty
            # пропуск манёвра трека: во дворах и на парковках поворотов нет на карте
            best = min(best, dp[i - 1, j] + skip_penalty)
            # сопоставление, возможно со слиянием нескольких поворотов маршрута
            ang_r = 0.0
            gap_r = route[j - 1][1]
            for k in range(j, max(j - 3, 0), -1):
                ang_r += route[k - 1][0]
                if k < j:
                    gap_r += route[k - 1][1]
                d_ang = abs(ang_t - ang_r)
                if d_ang > turn_tol_deg:
                    continue
                tol = max(dist_tol_rel * max(gap_t, 1.0), 25.0)
                d_gap = abs(gap_t - gap_r)
                if d_gap > 4.0 * tol:
                    continue
                cost = (d_ang / turn_tol_deg) ** 2 + (d_gap / tol) ** 2
                best = min(best, dp[i - 1, k - 1] + cost)
            dp[i, j] = best

    return float(dp[n, 1:].min())


def gnss_route_error(road_map, dir_edges, xy: np.ndarray) -> float:
    """Медианное расстояние от точек ГНСС до маршрута — честная проверка правильности."""
    rx, ry = route_polyline(road_map, dir_edges)
    if len(rx) < 2:
        return float("inf")
    pts = np.stack([rx, ry], axis=1)
    d = []
    step = max(1, len(xy) // 400)
    for p in xy[::step]:
        d.append(np.min(np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])))
    return float(np.median(d))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--beam", type=int, default=400)
    ap.add_argument("--candidates", type=int, default=60)
    ap.add_argument("--turn-tol-deg", type=float, default=20.0)
    ap.add_argument("--search-turn-tol-deg", type=float, default=25.0)
    ap.add_argument("--search-dist-tol-rel", type=float, default=0.15)
    ap.add_argument("--dist-tol-rel", type=float, default=0.05)
    ap.add_argument("--skip-penalty", type=float, default=1.5)
    ap.add_argument("--no-bias", action="store_true", help="не вычитать смещение нуля")
    ap.add_argument(
        "--skip-start-m",
        type=float,
        default=0.0,
        help="отбросить начало трека: во дворах и на парковках поворотов нет на карте",
    )
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"нет карты {args.map}")

    run = load_run(args.bag)
    t, v, w = run["t_s"], run["speed_mps"], run["yaw_rate_rps"]
    bias = 0.0 if args.no_bias else yaw_bias_rps(t, v, w)
    print(f"{args.bag.name}: смещение нуля {np.degrees(bias):.3f} °/с")

    if args.skip_start_m > 0.0:
        s_path = np.concatenate([[0.0], np.cumsum(np.diff(t) * v[:-1])])
        keep = s_path >= args.skip_start_m
        t, v, w = t[keep], v[keep], w[keep]
        print(f"начало трека отброшено: первые {args.skip_start_m:.0f} м")

    track = mm.build_track(list(t), list(v), list(w - bias))
    chain_t = track_chain(track)
    print(f"путь {track.length_m / 1000:.2f} км, поворотов в цепочке {len(chain_t)}")
    print("  " + " → ".join(f"{a:+.0f}°/{g:.0f}м" for a, g in chain_t))

    scfg = mm.SearchConfig()
    scfg.turn_tol_deg = args.search_turn_tol_deg
    scfg.dist_tol_rel = args.search_dist_tol_rel
    scfg.beam_width = args.beam
    scfg.max_candidates = args.candidates
    t0 = time.time()
    cands = mm.search_routes(road_map, track, scfg)
    print(f"кандидатов от поиска: {len(cands)} за {time.time() - t0:.1f} с")
    if not cands:
        return 1

    xy = gnss_in_map_frame(args.bag, road_map)

    rows = []
    for c in cands:
        chain_r = chain_from_polyline(*route_polyline(road_map, c.dir_edges), 20.0, 30.0)
        cost = align_cost(chain_t, chain_r, args.turn_tol_deg, args.dist_tol_rel, args.skip_penalty)
        err = gnss_route_error(road_map, c.dir_edges, xy) if xy is not None else float("nan")
        rows.append((cost, c.cost, err, c))

    print(f"\n{'#':>2} {'цепочка':>9} {'профиль':>8} {'ошибка ГНСС':>12}  маршрут")
    by_chain = sorted(rows, key=lambda r: r[0])
    for i, (cost, hcost, err, c) in enumerate(by_chain[:8], 1):
        mark = " ✓" if err < 60.0 else "  "
        print(f"{i:>2} {cost:>9.2f} {hcost:>8.2f} {err:>10.0f} м{mark} {route_streets(road_map, c.dir_edges)}")

    correct = [i for i, r in enumerate(by_chain, 1) if r[2] < 60.0]
    by_head = sorted(rows, key=lambda r: r[1])
    correct_head = [i for i, r in enumerate(by_head, 1) if r[2] < 60.0]
    print(f"\nправильный маршрут по цепочке: места {correct[:5] or 'нет среди кандидатов'}")
    print(f"по профилю курса (как сейчас):  места {correct_head[:5] or 'нет среди кандидатов'}")
    if len(by_chain) >= 2 and np.isfinite(by_chain[0][0]):
        m = by_chain[1][0] - by_chain[0][0]
        print(f"отрыв первого места по цепочке: {m:.2f} ({100 * m / max(by_chain[0][0], 1e-6):.0f} %)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
