#!/usr/bin/env python3
"""Сравнение трека с маршрутом приращениями курса на окнах пути.

Замер на беге 2026_08_08_23_00_28 показал, почему не работают оба прежних способа:

* глобальный профиль курса разрушен смещением нуля датчика (0.150 °/с → +421° за 16 км при
  истинных +173°);
* цепочка «поворот + перегон» несопоставима структурно: одометрия видит перегон 10 706 м, а на
  карте вдоль того же пути самый длинный перегон 4 875 м, потому что плавный съезд машина
  проходит дугой, а карта рисует его одним стыком под 123°.

Приращения курса на окне фиксированной длины пути свободны от обеих бед: дрейф на 300 м даёт
меньше 3°, а плавный съезд накапливает те же градусы, что и излом на карте.

  python3 -m mapmatch.window_score <bag>
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
from mapmatch.rerank_by_chain import yaw_bias_rps
from mapmatch.run_cache import load as load_run
from mapmatch.truth_route import Graph, gnss_in_map_frame, viterbi_match

mm = pyadas.mapmatch


def track_heading(bag: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Курс как функция пути по одометрии, со снятым смещением нуля."""
    run = load_run(bag)
    t, v, w = run["t_s"], run["speed_mps"], run["yaw_rate_rps"]
    bias = yaw_bias_rps(t, v, w)
    s = np.concatenate([[0.0], np.cumsum(np.diff(t) * v[:-1])])
    head = np.concatenate([[0.0], np.cumsum(np.diff(t) * (w[:-1] - bias))])
    return s, np.degrees(head)


def route_heading(road_map, dir_edges: List[int]) -> Tuple[np.ndarray, np.ndarray]:
    """То же для маршрута: курс по геометрии осевых линий."""
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
    x, y = np.asarray(xs), np.asarray(ys)
    if len(x) < 3:
        return np.array([0.0]), np.array([0.0])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    head = np.degrees(np.unwrap(np.arctan2(np.diff(y), np.diff(x))))
    return s[:-1], head - head[0]


def window_profile(
    s: np.ndarray, head: np.ndarray, win_m: float, total_m: float
) -> np.ndarray:
    """Приращение курса на каждом окне длиной win_m."""
    grid = np.arange(0.0, total_m + win_m, win_m)
    h = np.interp(grid, s, head)
    return np.diff(h)


def compare(
    track: Tuple[np.ndarray, np.ndarray],
    route: Tuple[np.ndarray, np.ndarray],
    win_m: float,
    scale_tol: float = 0.06,
) -> float:
    """Медиана расхождения приращений курса; масштаб пути подбирается в допуске."""
    ts, th = track
    rs, rh = route
    total = min(ts[-1], rs[-1])
    if total < 5 * win_m:
        return float("inf")

    best = float("inf")
    for scale in np.linspace(1.0 - scale_tol, 1.0 + scale_tol, 7):
        a = window_profile(ts, th, win_m, total)
        b = window_profile(rs * scale, rh, win_m, total)
        n = min(len(a), len(b))
        if n < 5:
            continue
        d = a[:n] - b[:n]
        # медленный дрейф съедаем медианой, а не подгонкой: она устойчива к настоящим поворотам
        best = min(best, float(np.median(np.abs(d - np.median(d)))))
    return best


def blinker_turns(bag: Path, s_of_t) -> List[Tuple[float, str]]:
    """Позиции включённых поворотников по пути — подсказка, где манёвр был намеренным."""
    run = load_run(bag)
    tt = run["blink_t_s"]
    if not len(tt):
        return []
    out: List[Tuple[float, str]] = []
    for name, flag in (("левый", run["blink_left"]), ("правый", run["blink_right"])):
        d = np.diff(flag.astype(int))
        starts, ends = np.where(d == 1)[0], np.where(d == -1)[0]
        for i in range(min(len(starts), len(ends))):
            if tt[ends[i]] - tt[starts[i]] < 1.0:
                continue
            out.append((float(s_of_t(tt[ends[i]])), name))
    return sorted(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--window-m", type=float, default=250.0)
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"нет карты {args.map}")
    graph = Graph(road_map)

    ts, th = track_heading(args.bag)
    print(f"{args.bag.name}: путь {ts[-1] / 1000:.2f} км")

    xy = gnss_in_map_frame(args.bag, road_map)
    truth = viterbi_match(road_map, graph, xy) if xy is not None else []
    if not truth:
        raise SystemExit("не удалось построить эталон")
    print(
        f"эталон: {len(truth)} рёбер, {sum(road_map.edge(d >> 1).length_m for d in truth) / 1000:.2f} км"
    )

    t0 = time.time()
    run = load_run(args.bag)
    track = mm.build_track(
        list(run["t_s"]), list(run["speed_mps"]), list(run["yaw_rate_rps"])
    )
    scfg = mm.SearchConfig()
    scfg.beam_width = 400
    scfg.max_candidates = 60
    cands = mm.search_routes(road_map, track, scfg)
    print(f"кандидатов поиска: {len(cands)} за {time.time() - t0:.1f} с")

    truth_score = compare((ts, th), route_heading(road_map, truth), args.window_m)
    rows = [
        (compare((ts, th), route_heading(road_map, c.dir_edges), args.window_m), c)
        for c in cands
    ]
    rows.sort(key=lambda r: r[0])

    print(
        f"\nокно {args.window_m:.0f} м, метрика — медианное расхождение приращений курса"
    )
    print(f"{'эталон':>10}: {truth_score:6.2f}°")
    for i, (score, c) in enumerate(rows[:8], 1):
        print(f"{i:>10}: {score:6.2f}°  {route_streets(road_map, c.dir_edges)}")
    better = sum(1 for score, _ in rows if score < truth_score)
    print(f"\nкандидатов лучше эталона: {better} из {len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
