#!/usr/bin/env python3
"""Lane counts per map edge: sidecar for the compact map.

The compact map (``ADASMAP1``) stores only geometry, ``oneway`` and names — enough for
localization, but the visualizer also wants to draw how many lanes a road has. Rebuilding
the map format for that would break the phone; a sidecar next to the ``.admap`` costs
nothing and is loaded only when present.

Lane counts come from the OSM tags: ``lanes`` for the whole road, ``lanes:forward`` /
``lanes:backward`` for the split, ``width`` when tagged. Each way is matched to map edges
by snapping the midpoints of its segments, and an edge takes the count of the way that
covers most of it.

  python3 -m mapmatch.osm_lanes maps/Moscow.osm.pbf --map maps/Moscow.osm.admap
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from mapmatch.locate import DEFAULT_MAP
from mapmatch.osm_graph import DRIVABLE

mm = pyadas.mapmatch


def _int_tag(tags, key: str) -> int:
    value = tags.get(key)
    if not value:
        return 0
    try:
        return max(0, int(float(str(value).split(";")[0])))
    except ValueError:
        return 0


def _float_tag(tags, key: str) -> float:
    value = tags.get(key)
    if not value:
        return 0.0
    try:
        return max(0.0, float(str(value).split(";")[0].replace("m", "").strip()))
    except ValueError:
        return 0.0


class LaneCollector:
    """Ways with a lane count, and the node ids needed for their geometry."""

    def __init__(self):
        self.ways: List[Tuple[List[int], int, float]] = []
        self.wanted: set = set()

    def way(self, w) -> None:
        if w.tags.get("highway") not in DRIVABLE:
            return
        total = _int_tag(w.tags, "lanes")
        fwd = _int_tag(w.tags, "lanes:forward")
        bwd = _int_tag(w.tags, "lanes:backward")
        if not total:
            total = fwd + bwd
        if not total:
            return
        ids = [n.ref for n in w.nodes]
        if len(ids) < 2:
            return
        self.ways.append((ids, total, _float_tag(w.tags, "width")))
        self.wanted.update(ids)


def collect(pbf: Path) -> LaneCollector:
    import osmium

    class H(osmium.SimpleHandler):
        def __init__(self, sink):
            super().__init__()
            self.sink = sink

        def way(self, w):
            self.sink.way(w)

    sink = LaneCollector()
    H(sink).apply_file(str(pbf), locations=False)
    return sink


def node_coords(pbf: Path, wanted: set) -> Dict[int, Tuple[float, float]]:
    import osmium

    class H(osmium.SimpleHandler):
        def __init__(self):
            super().__init__()
            self.coords: Dict[int, Tuple[float, float]] = {}

        def node(self, n):
            if n.id in wanted:
                self.coords[n.id] = (n.location.lat, n.location.lon)

    handler = H()
    handler.apply_file(str(pbf))
    return handler.coords


def assign(
    road_map,
    ways: List[Tuple[List[int], int, float]],
    coords: Dict[int, Tuple[float, float]],
    snap_m: float = 12.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vote lane counts onto map edges by snapping way geometry."""
    frame = road_map.frame
    votes: Dict[int, Counter] = defaultdict(Counter)
    widths: Dict[int, Counter] = defaultdict(Counter)
    for ids, lanes, width in ways:
        pts = [coords[i] for i in ids if i in coords]
        if len(pts) < 2:
            continue
        lat = [p[0] for p in pts]
        lon = [p[1] for p in pts]
        ex, ny = frame.to_local_many(lat, lon)
        ex, ny = np.asarray(ex), np.asarray(ny)
        # Snap midpoints: way nodes sit on intersections, where the nearest edge is ambiguous.
        mx = 0.5 * (ex[:-1] + ex[1:])
        my = 0.5 * (ny[:-1] + ny[1:])
        seg = np.hypot(np.diff(ex), np.diff(ny))
        for x, y, length in zip(mx, my, seg):
            eid, dist, _ = road_map.nearest_edge(float(x), float(y), snap_m)
            if eid < 0 or dist > snap_m:
                continue
            votes[eid][lanes] += max(1, int(length))
            if width > 0.0:
                widths[eid][round(width, 1)] += max(1, int(length))

    edges = np.array(sorted(votes), dtype=np.int32)
    lanes = np.array([votes[e].most_common(1)[0][0] for e in edges], dtype=np.int16)
    width = np.array(
        [widths[e].most_common(1)[0][0] if e in widths else 0.0 for e in edges],
        dtype=np.float32,
    )
    return edges, lanes, width


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("pbf", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"no map {args.map}")

    sink = collect(args.pbf)
    print(f"ways with lanes: {len(sink.ways)}, nodes needed: {len(sink.wanted)}")
    coords = node_coords(args.pbf, sink.wanted)
    print(f"node coordinates: {len(coords)}")

    edges, lanes, width = assign(road_map, sink.ways, coords)
    out = args.out or args.map.with_suffix(args.map.suffix + ".lanes.npz")
    np.savez_compressed(out, edge=edges, lanes=lanes, width=width)
    covered = 100.0 * len(edges) / max(road_map.edge_count, 1)
    print(
        f"edges with a lane count: {len(edges)} of {road_map.edge_count} ({covered:.1f} %)"
    )
    print(f"distribution: {dict(sorted(Counter(lanes.tolist()).items()))}")
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
