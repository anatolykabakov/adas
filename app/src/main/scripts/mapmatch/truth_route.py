#!/usr/bin/env python3
"""Эталонный маршрут по ГНСС: привязка к графу карты и сшивание кратчайшими путями.

Нужен, чтобы честно проверять локализацию. Штатная проверка в locate.py сравнивает названия
улиц по словам и засчитывает совпадение по слову «Большая»; привязка «каждая точка к ближайшему
ребру» без сшивания даёт скачки между дублёрами: на беге 2026_08_08_23_00_28 это 141 ребро,
37.9 км вместо 16.1 и 127 поворотов с разворотами на 180°.

  python3 -m mapmatch.truth_route <bag>
"""

from __future__ import annotations

import argparse
import heapq
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from mapmatch.locate import DEFAULT_MAP, route_streets
from mapmatch.run_cache import load as load_run

mm = pyadas.mapmatch


def gnss_in_map_frame(bag: Path, road_map) -> Optional[np.ndarray]:
    """Точки ГНСС в системе координат карты."""
    run = load_run(bag)
    lat, lon = run["gnss_lat"], run["gnss_lon"]
    if len(lat) < 5:
        return None
    frame = road_map.frame
    return np.array([frame.to_local(float(a), float(b)) for a, b in zip(lat, lon)])


class Graph:
    """Списки смежности по узлам карты."""

    def __init__(self, road_map):
        self.road_map = road_map
        self.out: Dict[int, List[Tuple[int, int, float]]] = defaultdict(list)
        for eid in range(road_map.edge_count):
            e = road_map.edge(eid)
            self.out[e.node_a].append((e.node_b, 2 * eid, e.length_m))
            if not e.oneway:
                self.out[e.node_b].append((e.node_a, 2 * eid + 1, e.length_m))

    def ends(self, dir_edge: int) -> Tuple[int, int]:
        e = self.road_map.edge(dir_edge >> 1)
        return (e.node_b, e.node_a) if dir_edge & 1 else (e.node_a, e.node_b)

    def shortest(self, src: int, dst: int, limit_m: float) -> Optional[List[int]]:
        """Кратчайший путь по рёбрам, None если дальше limit_m."""
        if src == dst:
            return []
        seen = {src: 0.0}
        prev: Dict[int, Tuple[int, int]] = {}
        pq = [(0.0, src)]
        while pq:
            d, node = heapq.heappop(pq)
            if node == dst:
                path = []
                while node != src:
                    node, de = prev[node]
                    path.append(de)
                return path[::-1]
            if d > limit_m or d > seen.get(node, 1e18):
                continue
            for nxt, de, length in self.out[node]:
                nd = d + length
                if nd < seen.get(nxt, 1e18):
                    seen[nxt] = nd
                    prev[nxt] = (node, de)
                    heapq.heappush(pq, (nd, nxt))
        return None


def snap_sequence(
    road_map, xy: np.ndarray, max_dist_m: float, min_hits: int
) -> List[int]:
    """Последовательность рёбер, к которым устойчиво липнут точки ГНСС."""
    raw: List[int] = []
    for p in xy:
        eid, dist, _ = road_map.nearest_edge(float(p[0]), float(p[1]), max_dist_m * 2)
        if dist > max_dist_m:
            continue
        raw.append(eid)

    kept: List[int] = []
    i = 0
    while i < len(raw):
        j = i
        while j < len(raw) and raw[j] == raw[i]:
            j += 1
        if j - i >= min_hits and (not kept or kept[-1] != raw[i]):
            kept.append(raw[i])
        i = j
    return kept


def stitch(graph: Graph, edges: List[int], gap_limit_m: float) -> List[int]:
    """Сшивает опорные рёбра в связный маршрут кратчайшими путями."""
    if not edges:
        return []

    route: List[int] = [2 * edges[0]]
    for eid in edges[1:]:
        tail = graph.ends(route[-1])[1]
        best: Optional[Tuple[float, List[int]]] = None
        for direction in (0, 1):
            de = 2 * eid + direction
            head, _ = graph.ends(de)
            path = graph.shortest(tail, head, gap_limit_m)
            if path is None:
                continue
            cost = sum(graph.road_map.edge(p >> 1).length_m for p in path)
            if best is None or cost < best[0]:
                best = (cost, path + [de])
        if best is None:
            continue
        route.extend(best[1])
    return route


def route_length(road_map, route: List[int]) -> float:
    return float(sum(road_map.edge(de >> 1).length_m for de in route))


def build(
    bag: Path, road_map, graph: Graph, max_dist_m: float = 25.0
) -> Optional[List[int]]:
    xy = gnss_in_map_frame(bag, road_map)
    if xy is None:
        return None
    anchors = snap_sequence(road_map, xy, max_dist_m, min_hits=2)
    return stitch(graph, anchors, gap_limit_m=1500.0)


def viterbi_match(
    road_map,
    graph: Graph,
    xy: np.ndarray,
    step_m: float = 50.0,
    radius_m: float = 30.0,
    sigma_m: float = 12.0,
    limit_m: float = 400.0,
) -> List[int]:
    """Привязка трека ГНСС к графу по Витерби.

    Привязка «каждая точка к ближайшему ребру» скачет между дублёрами: на беге
    2026_08_08_23_00_28 это даёт 23 % несвязанных переходов и развороты на 172°. Здесь переход
    между соседними точками штрафуется разницей между расстоянием по прямой и по дорогам.
    """
    pts = [xy[0]]
    for p in xy[1:]:
        if np.hypot(*(p - pts[-1])) >= step_m:
            pts.append(p)
    if len(pts) < 3:
        return []

    def candidates(p):
        out = []
        for eid in road_map.edges_in_bbox(
            p[0] - radius_m, p[1] - radius_m, p[0] + radius_m, p[1] + radius_m
        ):
            px, py = road_map.edge_polyline(eid)
            d = float(np.min(np.hypot(np.asarray(px) - p[0], np.asarray(py) - p[1])))
            if d <= radius_m:
                for direction in (0, 1):
                    de = 2 * eid + direction
                    if direction == 1 and road_map.edge(eid).oneway:
                        continue
                    out.append((de, d))
        return out[:12]

    prev_states = candidates(pts[0])
    if not prev_states:
        return []
    prev_cost = {de: (d / sigma_m) ** 2 for de, d in prev_states}
    back: List[Dict[int, int]] = []

    for i in range(1, len(pts)):
        cur = candidates(pts[i])
        if not cur:
            back.append({})
            continue
        straight = float(np.hypot(*(pts[i] - pts[i - 1])))
        cost: Dict[int, float] = {}
        ptr: Dict[int, int] = {}
        for de, d in cur:
            emit = (d / sigma_m) ** 2
            best = None
            for pde, pc in prev_cost.items():
                # разворот на том же ребре — артефакт привязки, а не манёвр
                if (pde >> 1) == (de >> 1) and pde != de:
                    continue
                if pde == de:
                    trans = 0.0
                else:
                    tail = graph.ends(pde)[1]
                    head = graph.ends(de)[0]
                    path = graph.shortest(tail, head, limit_m)
                    if path is None:
                        continue
                    if path and (path[0] ^ 1) == pde:
                        continue
                    route_len = sum(road_map.edge(x >> 1).length_m for x in path)
                    trans = (abs(route_len - straight) / max(straight, 10.0)) ** 2
                val = pc + trans
                if best is None or val < best[0]:
                    best = (val, pde)
            if best is not None:
                cost[de] = best[0] + emit
                ptr[de] = best[1]
        if cost:
            prev_cost = cost
            back.append(ptr)
        else:
            back.append({})

    state = min(prev_cost, key=prev_cost.get)
    states = [state]
    for ptr in reversed(back):
        if state in ptr:
            state = ptr[state]
            states.append(state)
    states.reverse()

    route: List[int] = []
    for de in states:
        if route and route[-1] == de:
            continue
        if route:
            tail = graph.ends(route[-1])[1]
            head = graph.ends(de)[0]
            if tail != head:
                path = graph.shortest(tail, head, limit_m)
                if path:
                    route.extend(path)
        route.append(de)
    return route


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"нет карты {args.map}")

    graph = Graph(road_map)
    xy = gnss_in_map_frame(args.bag, road_map)
    if xy is None:
        raise SystemExit("в беге нет ГНСС")

    route = viterbi_match(road_map, graph, xy)
    print(f"рёбер в маршруте {len(route)}")
    print(f"длина маршрута {route_length(road_map, route) / 1000:.2f} км")
    print(f"маршрут: {route_streets(road_map, route, max_items=14)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
