#!/usr/bin/env python3
"""Поиск места по приращениям курса на окнах пути.

Метрика из window_score ставит истинный маршрут первым (3.25° против 4.69° у лучшего чужого),
но ранжировать можно только то, что дал поиск, а он истинный маршрут не находит. Здесь поиск
построен на той же метрике: маршрут растится по графу, и на каждом пройденном окне его приращение
курса сравнивается с приращением трека.

  python3 -m mapmatch.window_search <bag>
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from mapmatch.locate import DEFAULT_MAP, route_streets
from mapmatch.rerank_by_chain import gnss_route_error, yaw_bias_rps
from mapmatch.track_from_bag import motion_profile
from mapmatch.truth_route import Graph, gnss_in_map_frame, viterbi_match
from mapmatch.window_score import track_heading, window_profile
from mapmatch.human_search import Geometry

mm = pyadas.mapmatch


def search(
    graph: Graph,
    geo: Geometry,
    windows: np.ndarray,
    win_m: float,
    beam: int,
    tol_deg: float,
) -> List[Tuple[float, List[int]]]:
    """Послойный лучевой рост: слой — одно окно пути.

    Отбор внутри слоя обязателен. Если отбирать «глобально», на первом шаге все состояния имеют
    нулевую стоимость и луч выбрасывает правильный старт случайным образом.
    """
    parent: List[int] = []
    edge_of: List[int] = []

    def add(de: int, par: int) -> int:
        parent.append(par)
        edge_of.append(de)
        return len(parent) - 1

    # состояние: (стоимость, узел пути, путь, курс, курс на начало окна)
    layer: List[Tuple[float, int, float, float, float]] = []
    for de in range(2 * graph.road_map.edge_count):
        if geo.length[de] <= 0:
            continue
        layer.append((0.0, add(de, -1), geo.length[de], geo.bend[de], 0.0))

    for k, want in enumerate(windows):
        border = (k + 1) * win_m
        nxt: List[Tuple[float, int, float, float, float]] = []
        for cost, node, dist, head, head0 in layer:
            # доращиваем, пока не закроем окно
            stack = [(node, dist, head)]
            for _ in range(40):
                if not stack:
                    break
                new_stack = []
                for nd, d, h in stack:
                    if d >= border:
                        diff = abs((h - head0) - want)
                        c2 = cost + (min(diff, 45.0) if diff > tol_deg else 0.0)
                        nxt.append((c2, nd, d, h, h))
                        continue
                    de = edge_of[nd]
                    tail = graph.ends(de)[1]
                    for _, de2, _ in graph.out.get(tail, []):
                        if (de2 >> 1) == (de >> 1):
                            continue
                        new_stack.append(
                            (
                                add(de2, nd),
                                d + geo.length[de2],
                                h + geo.turn_deg(de, de2) + geo.bend[de2],
                            )
                        )
                stack = new_stack[: 8 * beam]
        nxt.sort(key=lambda s: s[0])
        # оставляем только лучшие, но не больше одного состояния на ребро
        seen = set()
        layer = []
        for st in nxt:
            de = edge_of[st[1]]
            if de in seen:
                continue
            seen.add(de)
            layer.append(st)
            if len(layer) >= beam:
                break
        if not layer:
            break

    out: List[Tuple[float, List[int]]] = []
    for cost, node, *_ in layer[:20]:
        path: List[int] = []
        cur = node
        while cur >= 0:
            path.append(edge_of[cur])
            cur = parent[cur]
        out.append((cost, path[::-1]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--window-m", type=float, default=250.0)
    ap.add_argument("--beam", type=int, default=3000)
    ap.add_argument("--tol-deg", type=float, default=4.0)
    ap.add_argument("--skip-start-m", type=float, default=300.0)
    ap.add_argument(
        "--length-m", type=float, default=6000.0, help="сколько пути сопоставлять"
    )
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"нет карты {args.map}")
    graph = Graph(road_map)
    geo = Geometry(road_map)

    ts, th = track_heading(args.bag)
    keep = ts >= args.skip_start_m
    ts, th = ts[keep] - args.skip_start_m, th[keep] - th[keep][0]
    total = min(args.length_m, ts[-1])
    windows = window_profile(ts, th, args.window_m, total)
    print(f"{args.bag.name}: сопоставляем {total / 1000:.1f} км, окон {len(windows)}")
    print("  приращения курса: " + " ".join(f"{w:+.0f}" for w in windows[:14]) + " …")

    t0 = time.time()
    found = search(graph, geo, windows, args.window_m, args.beam, args.tol_deg)
    print(f"поиск за {time.time() - t0:.1f} с, доведено до конца {len(found)} маршрутов")

    xy = gnss_in_map_frame(args.bag, road_map)
    for i, (cost, path) in enumerate(found[:8], 1):
        err = gnss_route_error(road_map, path, xy) if xy is not None else float("nan")
        mark = " ✓" if err < 80 else "  "
        print(
            f"{i:>2} стоимость {cost:7.1f}  ошибка ГНСС {err:7.0f} м{mark} {route_streets(road_map, path)}"
        )

    if xy is not None and found:
        best_err = min(gnss_route_error(road_map, p, xy) for _, p in found)
        print(f"\nлучшая ошибка ГНСС среди найденных: {best_err:.0f} м")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
