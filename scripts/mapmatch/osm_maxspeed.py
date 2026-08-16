#!/usr/bin/env python3
"""Speed limits from OSM as a sidecar next to the ``.admap``.

The compact map (``ADASMAP1``) stores geometry, ``oneway`` and names only. Rebuilding the format for
a speed limit would break the phone, so this follows what ``osm_lanes.py`` already does: a file next
to the map, loaded only when present.

Why a speed limit is worth having at all. The sign detector reads a limit off the image; the map says
what limit that road carries. Agreement raises confidence, disagreement is a signal — the sign was
read wrong, or it belongs to a side road, or to trucks. Today there is nothing to check a recognised
sign against.

What is taken, and what is deliberately not:

* ``maxspeed`` — the value in force. ``maxspeed:forward`` / ``:backward`` are read too, because a
  two-way road can carry different limits per direction;
* ``maxspeed:type`` (e.g. ``RU:urban``) is kept separately: it is not a sign but the default that
  applies where no sign stands. Useful for telling "no sign here" from "no data";
* ``maxspeed:conditional`` is **skipped**. "70 @ (22:00-06:00)" needs a clock and a parser, and a
  conditional limit applied at the wrong hour is worse than no limit;
* ``walk``, ``none``, ``signals`` and other non-numeric values are skipped for the same reason.

  python3 -m mapmatch.osm_maxspeed maps/Moscow.osm.pbf --map maps/Moscow.osm.admap
"""

from __future__ import annotations

import argparse
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

import _path  # noqa: F401
from pyadas import core as pyadas
from mapmatch.locate import DEFAULT_MAP
from mapmatch.osm_graph import DRIVABLE
from mapmatch.osm_lanes import node_coords

mm = pyadas.mapmatch

_SPEED_RE = re.compile(r"^\s*(\d+(?:\.\d+)?)\s*(km/h|kph|mph)?\s*$", re.I)


def parse_speed_kmh(value: str) -> float:
    """Numeric limit in km/h, or 0 when the value is not one.

    ``mph`` is converted rather than dropped: it costs one multiplication and makes the script work
    on maps outside Europe without a second code path.
    """
    if not value:
        return 0.0
    m = _SPEED_RE.match(value)
    if not m:
        return 0.0
    speed = float(m.group(1))
    if (m.group(2) or "").lower() == "mph":
        speed *= 1.609344
    return speed if 5.0 <= speed <= 200.0 else 0.0


class SpeedCollector:
    """Ways carrying a numeric speed limit, plus the node ids needed for their geometry."""

    def __init__(self):
        self.ways: List[Tuple[List[int], float, float, str]] = []
        self.wanted: set = set()

    def way(self, w) -> None:
        if w.tags.get("highway") not in DRIVABLE:
            return
        both = parse_speed_kmh(w.tags.get("maxspeed", ""))
        fwd = parse_speed_kmh(w.tags.get("maxspeed:forward", "")) or both
        bwd = parse_speed_kmh(w.tags.get("maxspeed:backward", "")) or both
        kind = w.tags.get("maxspeed:type", "") or w.tags.get("source:maxspeed", "")
        if not fwd and not bwd and not kind:
            return
        ids = [n.ref for n in w.nodes]
        if len(ids) < 2:
            return
        self.ways.append((ids, fwd, bwd, kind))
        self.wanted.update(ids)


def collect(pbf: Path) -> SpeedCollector:
    import osmium

    sink = SpeedCollector()

    class H(osmium.SimpleHandler):
        def __init__(self, sink):
            super().__init__()
            self.sink = sink

        def way(self, w):
            self.sink.way(w)

    H(sink).apply_file(str(pbf))
    return sink


def assign(
    road_map,
    ways: List[Tuple[List[int], float, float, str]],
    coords: Dict[int, Tuple[float, float]],
    snap_m: float = 12.0,
):
    """Vote speed limits onto map edges by snapping way geometry.

    Same snapping as the lane sidecar, and for the same reason: way nodes sit on intersections where
    the nearest edge is ambiguous, so segment midpoints are matched instead. Votes are weighted by
    segment length, so a long way wins over a short stub that happens to overlap.

    Forward and backward limits are kept apart because the map edge has a direction and the two can
    differ; where only ``maxspeed`` is tagged both carry the same number.
    """
    frame = road_map.frame
    fwd_votes: Dict[int, Counter] = defaultdict(Counter)
    bwd_votes: Dict[int, Counter] = defaultdict(Counter)
    kinds: Dict[int, Counter] = defaultdict(Counter)

    for ids, fwd, bwd, kind in ways:
        pts = [coords[i] for i in ids if i in coords]
        if len(pts) < 2:
            continue
        ex, ny = frame.to_local_many([p[0] for p in pts], [p[1] for p in pts])
        ex, ny = np.asarray(ex), np.asarray(ny)
        mx = 0.5 * (ex[:-1] + ex[1:])
        my = 0.5 * (ny[:-1] + ny[1:])
        seg = np.hypot(np.diff(ex), np.diff(ny))
        for x, y, length in zip(mx, my, seg):
            eid, dist, _ = road_map.nearest_edge(float(x), float(y), snap_m)
            if eid < 0 or dist > snap_m:
                continue
            weight = max(1, int(length))
            if fwd:
                fwd_votes[eid][round(fwd)] += weight
            if bwd:
                bwd_votes[eid][round(bwd)] += weight
            if kind:
                kinds[eid][kind] += weight

    edges = np.array(sorted(set(fwd_votes) | set(bwd_votes) | set(kinds)), dtype=np.int32)

    def pick(votes: Dict[int, Counter]) -> np.ndarray:
        return np.array(
            [votes[e].most_common(1)[0][0] if e in votes else 0 for e in edges],
            dtype=np.int16,
        )

    kind_names = [kinds[e].most_common(1)[0][0] if e in kinds else "" for e in edges]
    return edges, pick(fwd_votes), pick(bwd_votes), np.array(kind_names, dtype=object)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("pbf", type=Path)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--snap-m", type=float, default=12.0)
    args = ap.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(f"no map {args.map}")

    sink = collect(args.pbf)
    print(f"ways with a speed limit: {len(sink.ways)}, nodes needed: {len(sink.wanted)}")
    coords = node_coords(args.pbf, sink.wanted)
    edges, fwd, bwd, kind = assign(road_map, sink.ways, coords, args.snap_m)

    out = args.out or args.map.with_suffix(args.map.suffix + ".maxspeed.npz")
    np.savez_compressed(out, edge=edges, forward_kmh=fwd, backward_kmh=bwd, kind=kind)

    tagged = int(np.count_nonzero(fwd))
    total = road_map.edge_count
    print(
        f"{out}: {len(edges)} рёбер со скоростью из {total} ({100.0 * tagged / max(total, 1):.1f} %)"
    )
    if len(edges):
        vals, cnt = np.unique(fwd[fwd > 0], return_counts=True)
        top = sorted(zip(vals.tolist(), cnt.tolist()), key=lambda x: -x[1])[:8]
        print("  чаще всего:", ", ".join(f"{v} км/ч × {c}" for v, c in top))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
