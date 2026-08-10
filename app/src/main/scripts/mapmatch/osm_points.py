#!/usr/bin/env python3
"""Point features from OSM — traffic lights, stop and give-way, crossings, calming — as a sidecar.

Speed limits belong to a road and ride on the edge (``osm_maxspeed.py``). These do not: a traffic
light is a point on the way, and pinning it to an edge would lose where along that edge it stands.
So this sidecar stores coordinates in the map's own local frame, and the consumer decides what is
near enough to matter.

What is collected and why it is worth collecting:

* ``highway=traffic_signals`` — the light itself. The map cannot say what colour it is showing, so
  this is a hint about where to look, not a rule to obey;
* ``highway=stop`` / ``give_way`` — signs that OSM does record reliably, unlike ``traffic_sign=*``;
* ``highway=crossing`` — pedestrian crossings, split by whether they are signalised;
* ``traffic_calming=*`` — speed bumps, which are the one point feature the suspension confirms;
* ``traffic_sign=*`` — the literal sign, kept when present even though it rarely is.

``direction`` is carried where tagged: a stop line on a two-way street applies to one approach, and
without it the feature would appear for oncoming traffic as well.

  python3 -m mapmatch.osm_points assets/Moscow.osm.pbf --map maps/Moscow.osm.admap
"""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
from typing import List, Tuple

import numpy as np

import _path  # noqa: F401
from pyadas import core as pyadas
from mapmatch.locate import DEFAULT_MAP

mm = pyadas.mapmatch

HIGHWAY_KINDS = {
    "traffic_signals": "signal",
    "stop": "stop",
    "give_way": "give_way",
    "crossing": "crossing",
    "speed_camera": "camera",
}


class PointCollector:
    def __init__(self):
        self.lat: List[float] = []
        self.lon: List[float] = []
        self.kind: List[str] = []
        self.detail: List[str] = []

    def node(self, n) -> None:
        tags = n.tags
        kind = HIGHWAY_KINDS.get(tags.get("highway", ""))
        detail = ""

        if kind == "crossing":
            if tags.get("crossing") == "traffic_signals":
                kind = "signal"
                detail = "crossing"
            else:
                detail = tags.get("crossing", "")
        elif kind == "signal":
            if tags.get("traffic_signals:vehicle") == "no":
                return
            detail = tags.get("traffic_signals:direction", "") or tags.get(
                "direction", ""
            )
        elif kind in ("stop", "give_way"):
            detail = tags.get("direction", "")

        if kind is None:
            calming = tags.get("traffic_calming")
            if calming:
                kind, detail = "calming", calming
            elif tags.get("traffic_sign"):
                kind, detail = "sign", tags.get("traffic_sign", "")
            else:
                return

        loc = n.location
        if not loc.valid():
            return
        self.lat.append(loc.lat)
        self.lon.append(loc.lon)
        self.kind.append(kind)
        self.detail.append(detail)


def collect(pbf: Path) -> PointCollector:
    import osmium

    sink = PointCollector()

    class H(osmium.SimpleHandler):
        def __init__(self, sink):
            super().__init__()
            self.sink = sink

        def node(self, n):
            self.sink.node(n)

    H(sink).apply_file(str(pbf))
    return sink


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("pbf", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument(
        "--max-dist-m",
        type=float,
        default=25.0,
        help="drop points further than this from any road: they are not on our network",
    )
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"no map {args.map}")

    sink = collect(args.pbf)
    if not sink.lat:
        raise SystemExit("no point features found")

    ex, ny = road_map.frame.to_local_many(sink.lat, sink.lon)
    ex, ny = np.asarray(ex, dtype=np.float32), np.asarray(ny, dtype=np.float32)

    keep, edges = [], []
    for i in range(len(ex)):
        eid, dist, _ = road_map.nearest_edge(float(ex[i]), float(ny[i]), args.max_dist_m)
        if eid == 0xFFFFFFFF or dist > args.max_dist_m:
            continue
        keep.append(i)
        edges.append(eid)

    idx = np.array(keep, dtype=np.int64)
    out = args.out or args.map.with_suffix(args.map.suffix + ".points.npz")
    np.savez_compressed(
        out,
        x=ex[idx],
        y=ny[idx],
        edge=np.array(edges, dtype=np.int32),
        kind=np.array([sink.kind[i] for i in keep], dtype=object),
        detail=np.array([sink.detail[i] for i in keep], dtype=object),
    )

    counts = Counter(sink.kind[i] for i in keep)
    print(f"{out}: {len(keep)} точек на дороге из {len(sink.lat)} найденных")
    for k, c in counts.most_common():
        print(f"  {k:10} {c}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
