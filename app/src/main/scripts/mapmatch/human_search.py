#!/usr/bin/env python3
"""Поиск места «как человек»: зацепиться за самую редкую деталь маршрута, потом проверить остальное.

Штатный поиск стартует с первого поворота трека и растит луч вперёд. На беге
2026_08_08_23_00_28 это не работает: первый манёвр — выезд со двора, которого в OSM нет, и
правильного маршрута нет даже среди кандидатов при любых допусках.

Человек с картой поступает иначе: находит в своей поездке самое редкое — «прямая почти одиннадцать
километров» — ищет на карте такие места (их единицы), и уже там проверяет, сходятся ли повороты до
и после. Здесь ровно это.

  python3 -m mapmatch.human_search <bag>
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from mapmatch.locate import DEFAULT_MAP, route_streets
from mapmatch.rerank_by_chain import track_chain, yaw_bias_rps
from mapmatch.track_from_bag import motion_profile
from mapmatch.truth_route import Graph, gnss_in_map_frame

mm = pyadas.mapmatch


class Geometry:
    """Курс в начале и в конце каждого направленного ребра."""

    def __init__(self, road_map):
        self.road_map = road_map
        n = road_map.edge_count
        self.start = np.zeros(2 * n)
        self.end = np.zeros(2 * n)
        self.length = np.zeros(2 * n)
        # изгиб внутри ребра: без него кольцевая дорога проходит как «прямая», потому что на
        # стыках рёбер поворотов нет, а кривизна спрятана в полилинии
        self.bend = np.zeros(2 * n)
        for eid in range(n):
            px, py = road_map.edge_polyline(eid)
            px, py = np.asarray(px), np.asarray(py)
            if len(px) < 2:
                continue
            h0 = math.atan2(py[1] - py[0], px[1] - px[0])
            h1 = math.atan2(py[-1] - py[-2], px[-1] - px[-2])
            ln = road_map.edge(eid).length_m
            self.start[2 * eid], self.end[2 * eid], self.length[2 * eid] = h0, h1, ln
            self.start[2 * eid + 1] = h1 + math.pi
            self.end[2 * eid + 1] = h0 + math.pi
            self.length[2 * eid + 1] = ln
            hx = np.unwrap(np.arctan2(np.diff(py), np.diff(px)))
            bend = math.degrees(hx[-1] - hx[0])
            self.bend[2 * eid] = bend
            self.bend[2 * eid + 1] = -bend

    def turn_deg(self, de_in: int, de_out: int) -> float:
        d = self.start[de_out] - self.end[de_in]
        return math.degrees(math.atan2(math.sin(d), math.cos(d)))


def straight_runs(
    graph: Graph,
    geo: Geometry,
    straight_deg: float,
    min_len_m: float,
    total_deg: float = 70.0,
) -> List[List[int]]:
    """Места, где можно проехать min_len_m без поворота круче straight_deg и без общего разворота.

    Ограничение на суммарный курс обязательно: без него коридор уползает змейкой через весь
    город — на карте Москвы таких «прямых» длиннее 9.8 км набирается 1348 штук.
    """
    best: Dict[int, float] = {}
    nxt: Dict[int, int] = {}
    order: List[int] = []

    def run_length(de: int, depth: int = 0) -> float:
        if de in best:
            return best[de]
        if depth > 400:
            return geo.length[de]
        best[de] = geo.length[de]  # защита от циклов на время рекурсии
        tail = graph.ends(de)[1]
        top = 0.0
        top_de = -1
        for _, de2, _ in graph.out[tail]:
            if (de2 >> 1) == (de >> 1):
                continue
            if abs(geo.turn_deg(de, de2)) > straight_deg:
                continue
            val = run_length(de2, depth + 1)
            if val > top:
                top, top_de = val, de2
        best[de] = geo.length[de] + top
        if top_de >= 0:
            nxt[de] = top_de
        return best[de]

    sys.setrecursionlimit(10000)
    for de in range(2 * graph.road_map.edge_count):
        if geo.length[de] <= 0:
            continue
        if run_length(de) >= min_len_m:
            order.append(de)

    # оставить только начала коридоров: если предшественник тоже длинный, это его продолжение
    starts = []
    incoming = set(nxt.values())
    for de in order:
        if de not in incoming:
            starts.append(de)

    runs: List[List[int]] = []
    for de in starts:
        path = [de]
        cur = de
        heading = 0.0
        length = geo.length[de]
        while cur in nxt and len(path) < 2000:
            nx = nxt[cur]
            heading += geo.turn_deg(cur, nx)
            if abs(heading) > total_deg:
                break
            cur = nx
            path.append(cur)
            length += geo.length[cur]
        if length >= min_len_m:
            runs.append(path)
    return runs



def corridor_beam(
    graph: Graph,
    geo: Geometry,
    start: int,
    target_m: float,
    tol_m: float,
    straight_deg: float,
    total_deg: float,
    beam: int,
    max_steps: int = 400,
) -> List[Tuple[List[int], float, float]]:
    """Все способы проехать примерно target_m от start без крутых поворотов.

    Жадный обход здесь не работает: на развязке самое прямое ребро — съезд, и коридор уходит с
    магистрали. На Юго-Восточной хорде это обрывает прямую через 5.09 км вместо 10.7. Поэтому
    вперёд ведётся несколько вариантов сразу, отбор по накопленному отклонению курса.
    """
    live: List[Tuple[List[int], float, float]] = [([start], geo.length[start], geo.bend[start])]
    done: List[Tuple[List[int], float, float]] = []
    for _ in range(max_steps):
        if not live:
            break
        nxt_live: List[Tuple[List[int], float, float]] = []
        for path, dist, head in live:
            if dist >= target_m - tol_m:
                done.append((path, dist, head))
                if dist >= target_m + tol_m:
                    continue
            tail = graph.ends(path[-1])[1]
            for _, de2, _ in graph.out.get(tail, []):
                if (de2 >> 1) == (path[-1] >> 1):
                    continue
                turn = geo.turn_deg(path[-1], de2)
                if abs(turn) > straight_deg:
                    continue
                head2 = head + turn + geo.bend[de2]
                if abs(head2) > total_deg:
                    continue
                dist2 = dist + geo.length[de2]
                if dist2 > target_m + tol_m:
                    continue
                nxt_live.append((path + [de2], dist2, head2))
        # отбор: прямее и длиннее — лучше
        nxt_live.sort(key=lambda p: (abs(p[2]), -p[1]))
        live = nxt_live[:beam]
    return done


def follow(graph: Graph, geo: Geometry, de: int, want_turn: float, tol_deg: float) -> List[int]:
    """Продолжения после ребра de с поворотом, попадающим в допуск."""
    tail = graph.ends(de)[1]
    out = []
    for _, de2, _ in graph.out[tail]:
        if (de2 >> 1) == (de >> 1):
            continue
        if abs(geo.turn_deg(de, de2) - want_turn) <= tol_deg:
            out.append(de2)
    return out


def walk_gap(
    graph: Graph,
    geo: Geometry,
    de: int,
    gap_m: float,
    tol_m: float,
    straight_deg: float,
    beam: int = 40,
) -> List[Tuple[List[int], float]]:
    """Проехать примерно gap_m без крутых поворотов; вернуть варианты и невязку по длине."""
    out: List[Tuple[List[int], float]] = []
    stack: List[Tuple[List[int], float]] = [([de], geo.length[de])]
    while stack:
        path, dist = stack.pop()
        if dist >= gap_m - tol_m:
            out.append((path, abs(dist - gap_m)))
            if len(out) > beam:
                break
            continue
        if dist > gap_m + tol_m or len(path) > 400:
            continue
        tail = graph.ends(path[-1])[1]
        for _, de2, _ in graph.out[tail]:
            if (de2 >> 1) == (path[-1] >> 1):
                continue
            if abs(geo.turn_deg(path[-1], de2)) > straight_deg:
                continue
            stack.append((path + [de2], dist + geo.length[de2]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--straight-deg", type=float, default=25.0)
    ap.add_argument("--turn-tol-deg", type=float, default=25.0)
    ap.add_argument("--dist-tol-rel", type=float, default=0.08)
    ap.add_argument("--total-deg", type=float, default=90.0)
    ap.add_argument("--beam", type=int, default=6)
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"нет карты {args.map}")

    t0 = time.time()
    graph = Graph(road_map)
    geo = Geometry(road_map)
    print(f"граф и геометрия готовы за {time.time() - t0:.1f} с")

    t, v, w = [np.asarray(a) for a in motion_profile(args.bag)]
    bias = yaw_bias_rps(t, v, w)
    track = mm.build_track(list(t), list(v), list(w - bias))
    chain = track_chain(track)
    print(f"{args.bag.name}: {track.length_m / 1000:.2f} км, смещение нуля {math.degrees(bias):.3f} °/с")
    print("цепочка: " + " → ".join(f"{a:+.0f}°/{g:.0f}м" for a, g in chain))

    # самая редкая деталь — самый длинный перегон
    anchor_i = int(np.argmax([g for _, g in chain]))
    anchor_len = chain[anchor_i][1]
    print(f"\nзацепка: перегон {anchor_len:.0f} м перед поворотом {chain[anchor_i][0]:+.0f}°")

    tol = max(args.dist_tol_rel * anchor_len, 100.0)

    xy = gnss_in_map_frame(args.bag, road_map)

    turn_after = chain[anchor_i][0]
    turn_before = chain[anchor_i - 1][0] if anchor_i > 0 else None
    after = chain[anchor_i + 1 :]

    # начала зацепки: рёбра, в которые можно въехать нужным поворотом
    in_edges: Dict[int, List[int]] = {}
    for node, outs in graph.out.items():
        for _, de, _ in outs:
            in_edges.setdefault(graph.ends(de)[1], []).append(de)

    starts: List[int] = []
    for de in range(2 * road_map.edge_count):
        if geo.length[de] <= 0:
            continue
        if turn_before is None:
            starts.append(de)
            continue
        head_node = graph.ends(de)[0]
        for de_in in in_edges.get(head_node, []):
            if (de_in >> 1) == (de >> 1):
                continue
            if abs(geo.turn_deg(de_in, de) - turn_before) <= args.turn_tol_deg:
                starts.append(de)
                break
    print(f"рёбер, куда можно въехать поворотом {turn_before:+.0f}°: {len(starts)}")

    t0 = time.time()
    results = []
    for de in starts:
        for path, dist, _ in corridor_beam(
            graph, geo, de, anchor_len, tol, args.straight_deg, args.total_deg, args.beam
        ):
            outs = follow(graph, geo, path[-1], turn_after, args.turn_tol_deg)
            if not outs:
                continue
            full = path + [outs[0]]
            cost = abs(dist - anchor_len) / max(tol, 1.0)
            ok = True
            for ang, gap in after:
                opts = walk_gap(
                    graph, geo, full[-1], gap, max(args.dist_tol_rel * gap, 40.0), args.straight_deg
                )
                if not opts:
                    ok = False
                    break
                opts.sort(key=lambda o: o[1])
                seg, gap_err = opts[0]
                nxt_opts = follow(graph, geo, seg[-1], ang, args.turn_tol_deg)
                if not nxt_opts:
                    ok = False
                    break
                full = full + seg[1:] + [nxt_opts[0]]
                cost += gap_err / max(args.dist_tol_rel * gap, 40.0)
            if ok:
                results.append((cost, full))
    print(f"перебор зацепки за {time.time() - t0:.1f} с")

    print(f"мест, где сошлись и повороты после зацепки: {len(results)}")
    results.sort(key=lambda r: r[0])
    for i, (cost, path) in enumerate(results[:10], 1):
        err = ""
        if xy is not None:
            from mapmatch.rerank_by_chain import gnss_route_error

            err = f"{gnss_route_error(road_map, path, xy):7.0f} м"
        print(f"{i:>2} невязка {cost:6.2f}  ошибка ГНСС {err}  {route_streets(road_map, path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
