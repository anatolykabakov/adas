#!/usr/bin/env python3
"""OSM extract → compact drivable-road graph for localization.

Convert a .osm.pbf to a binary once; the C++ algorithm and the phone read it afterward.
Python here is only a preprocessor: no pyosmium or XML parsing on device.

What we do:
  * keep only drivable highway=* (no footways, steps, paths);
  * project to meters (same LocalFrame as C++ — call it via the binding);
  * split ways into edges between intersections (a node where ≥3 drivable way ends meet,
    or where a way ends);
  * store edge polylines, lengths, oneway, names, and a spatial grid for area lookup.

  python3 -m mapmatch.osm_graph maps/Moscow.osm.pbf
  python3 -m mapmatch.osm_graph maps/Moscow.osm.pbf --out maps/moscow.admap --stats
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas

mm = pyadas.mapmatch

MAGIC = b"ADASMAP1"

# What counts as drivable. service/track are deliberately excluded: in courtyards and
# industrial zones OSM geometry is messy, and the track cannot be recognized on them anyway.
DRIVABLE = {
    "motorway",
    "trunk",
    "primary",
    "secondary",
    "tertiary",
    "unclassified",
    "residential",
    "living_street",
    "motorway_link",
    "trunk_link",
    "primary_link",
    "secondary_link",
    "tertiary_link",
}

GRID_CELL_M = 500.0


class WayCollector:
    """First pass: which nodes are intersections, and geometry of drivable ways."""

    def __init__(self):
        self.ways: List[Tuple[List[int], str, bool]] = []  # (node_ids, name, oneway)
        self.node_use = Counter()

    def way(self, w):
        hw = w.tags.get("highway")
        if hw not in DRIVABLE:
            return
        if w.tags.get("area") == "yes":
            return
        ids = [n.ref for n in w.nodes]
        if len(ids) < 2:
            return
        oneway = w.tags.get("oneway") in ("yes", "1", "true", "-1") or hw in (
            "motorway",
            "motorway_link",
        )
        reverse = w.tags.get("oneway") == "-1"
        if reverse:
            ids = ids[::-1]
        self.ways.append((ids, w.tags.get("name") or "", oneway))
        # Way endpoints are always graph nodes; interior nodes only if split by another way.
        for i in ids:
            self.node_use[i] += 1
        self.node_use[ids[0]] += 2
        self.node_use[ids[-1]] += 2


def collect(pbf: Path) -> WayCollector:
    import osmium

    class H(osmium.SimpleHandler):
        def __init__(self, sink):
            super().__init__()
            self.sink = sink

        def way(self, w):
            self.sink.way(w)

    sink = WayCollector()
    H(sink).apply_file(str(pbf), locations=False)
    return sink


def node_coords(pbf: Path, wanted: set) -> Dict[int, Tuple[float, float]]:
    """Coordinates for only the needed nodes — far fewer than all nodes in the extract."""
    import osmium

    class H(osmium.SimpleHandler):
        def __init__(self):
            super().__init__()
            self.coords: Dict[int, Tuple[float, float]] = {}

        def node(self, n):
            if n.id in wanted:
                self.coords[n.id] = (n.location.lat, n.location.lon)

    h = H()
    h.apply_file(str(pbf), locations=False)
    return h.coords


def build(pbf: Path, verbose: bool = True) -> dict:
    t0 = time.time()
    ways = collect(pbf)
    if verbose:
        print(f"drivable ways {len(ways.ways):,} in {time.time() - t0:.0f} s")

    wanted = set()
    for ids, _, _ in ways.ways:
        wanted.update(ids)
    t1 = time.time()
    coords = node_coords(pbf, wanted)
    if verbose:
        print(f"node coordinates {len(coords):,} in {time.time() - t1:.0f} s")

    # Intersections: nodes used by more than one way (or endpoints).
    junction = {i for i, c in ways.node_use.items() if c >= 2 and i in coords}

    frame = mm.LocalFrame()
    lats, lons, order = [], [], []
    for nid in sorted(wanted & coords.keys()):
        lat, lon = coords[nid]
        lats.append(lat)
        lons.append(lon)
        order.append(nid)
    east, north = frame.to_local_many(lats, lons)
    xy = {nid: (e, n) for nid, e, n in zip(order, east, north)}

    # Split ways into edges between intersections.
    node_index: Dict[int, int] = {}
    node_xy: List[Tuple[float, float]] = []
    names: List[str] = []
    name_index: Dict[str, int] = {}
    edges: List[dict] = []

    def node_id(osm_id: int) -> int:
        idx = node_index.get(osm_id)
        if idx is None:
            idx = len(node_xy)
            node_index[osm_id] = idx
            node_xy.append(xy[osm_id])
        return idx

    def name_id(name: str) -> int:
        if not name:
            return 0xFFFFFFFF
        idx = name_index.get(name)
        if idx is None:
            idx = len(names)
            name_index[name] = idx
            names.append(name)
        return idx

    for ids, name, oneway in ways.ways:
        ids = [i for i in ids if i in xy]
        if len(ids) < 2:
            continue
        nid = name_id(name)
        start = 0
        for k in range(1, len(ids)):
            is_last = k == len(ids) - 1
            if ids[k] not in junction and not is_last:
                continue
            seg = ids[start : k + 1]
            if len(seg) < 2:
                continue
            pts = [xy[i] for i in seg]
            length = float(
                np.sum(
                    np.hypot(np.diff([p[0] for p in pts]), np.diff([p[1] for p in pts]))
                )
            )
            if length >= 1.0:
                edges.append(
                    {
                        "a": node_id(seg[0]),
                        "b": node_id(seg[-1]),
                        "pts": pts,
                        "length": length,
                        "name": nid,
                        "oneway": bool(oneway),
                    }
                )
            start = k

    if verbose:
        print(
            f"graph nodes {len(node_xy):,}, edges {len(edges):,}, names {len(names):,}, "
            f"geometry points {sum(len(e['pts']) for e in edges):,} in {time.time() - t0:.0f} s"
        )

    return {"frame": frame, "nodes": node_xy, "edges": edges, "names": names}


def write_binary(graph: dict, dst: Path) -> None:
    """Binary for single-chunk loading: the phone loads it as-is, without parsing."""
    nodes = graph["nodes"]
    edges = graph["edges"]
    names = graph["names"]
    frame = graph["frame"]

    # Outgoing-edge index per node — for graph search.
    out_edges: Dict[int, List[int]] = defaultdict(list)
    for ei, e in enumerate(edges):
        out_edges[e["a"]].append(ei)
        if not e["oneway"]:
            out_edges[e["b"]].append(ei)

    # Grid for "which edges are nearby" — seed candidates and geometry.
    xs = np.array([p[0] for p in nodes], dtype=np.float64)
    ys = np.array([p[1] for p in nodes], dtype=np.float64)
    x0, y0 = float(xs.min()), float(ys.min())
    nx = int((xs.max() - x0) / GRID_CELL_M) + 1
    ny = int((ys.max() - y0) / GRID_CELL_M) + 1
    cells: Dict[int, List[int]] = defaultdict(list)
    for ei, e in enumerate(edges):
        seen = set()
        for px, py in e["pts"]:
            cx = int((px - x0) / GRID_CELL_M)
            cy = int((py - y0) / GRID_CELL_M)
            if 0 <= cx < nx and 0 <= cy < ny:
                seen.add(cy * nx + cx)
        for c in seen:
            cells[c].append(ei)

    buf = bytearray()
    buf += MAGIC
    buf += struct.pack("<dd", frame.lat0_deg, frame.lon0_deg)
    n_points = sum(len(e["pts"]) for e in edges)
    buf += struct.pack("<IIII", len(nodes), len(edges), n_points, len(names))
    buf += struct.pack("<ddII", x0, y0, nx, ny)
    buf += struct.pack("<d", GRID_CELL_M)

    for px, py in nodes:
        buf += struct.pack("<ii", int(round(px * 100.0)), int(round(py * 100.0)))

    pt_offset = 0
    edge_rec = bytearray()
    pts_rec = bytearray()
    for e in edges:
        edge_rec += struct.pack(
            "<IIIHIfB",
            e["a"],
            e["b"],
            pt_offset,
            len(e["pts"]),
            e["name"],
            float(e["length"]),
            1 if e["oneway"] else 0,
        )
        for px, py in e["pts"]:
            pts_rec += struct.pack("<ii", int(round(px * 100.0)), int(round(py * 100.0)))
        pt_offset += len(e["pts"])
    buf += edge_rec
    buf += pts_rec

    # node → outgoing edges (CSR)
    csr_off = bytearray()
    csr_val = bytearray()
    total = 0
    for i in range(len(nodes)):
        csr_off += struct.pack("<I", total)
        for ei in out_edges.get(i, ()):
            csr_val += struct.pack("<I", ei)
            total += 1
    csr_off += struct.pack("<I", total)
    buf += csr_off
    buf += csr_val

    # grid (CSR by cell)
    grid_off = bytearray()
    grid_val = bytearray()
    total = 0
    for c in range(nx * ny):
        grid_off += struct.pack("<I", total)
        for ei in cells.get(c, ()):
            grid_val += struct.pack("<I", ei)
            total += 1
    grid_off += struct.pack("<I", total)
    buf += grid_off
    buf += grid_val

    name_blob = bytearray()
    for s in names:
        b = s.encode("utf-8")[:255]
        name_blob += struct.pack("<H", len(b)) + b
    buf += struct.pack("<I", len(name_blob))
    buf += name_blob

    dst.write_bytes(bytes(buf))


def stats(graph: dict, dst: Path) -> None:
    edges = graph["edges"]
    lens = np.array([e["length"] for e in edges])
    oneway = sum(1 for e in edges if e["oneway"])
    print()
    print(f"file: {dst} — {dst.stat().st_size / 1e6:.1f} MB")
    print(
        f"  nodes {len(graph['nodes']):,}, edges {len(edges):,}, names {len(graph['names']):,}"
    )
    print(
        f"  edge length: median {np.median(lens):.0f} m, p95 {np.percentile(lens, 95):.0f} m, "
        f"total {lens.sum() / 1000:.0f} km"
    )
    print(f"  one-way edges {100.0 * oneway / max(1, len(edges)):.0f} %")


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("pbf", type=Path)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--stats", action="store_true")
    args = p.parse_args()

    if not args.pbf.exists():
        print(
            f"file not found {args.pbf} — run python3 -m mapmatch.fetch_map --chunked first",
            file=sys.stderr,
        )
        return 2

    graph = build(args.pbf)
    dst = args.out or args.pbf.with_suffix(".admap")
    write_binary(graph, dst)
    if args.stats:
        stats(graph, dst)
    else:
        print(f"{dst} — {dst.stat().st_size / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
