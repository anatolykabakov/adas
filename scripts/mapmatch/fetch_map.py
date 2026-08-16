#!/usr/bin/env python3
"""Download OSM extract for map-based localization.

Saves the file under `maps/` (directory is gitignored). Then `osm_graph.py` builds a compact
drivable-road graph that the C++ algorithm and phone read.

  python3 -m mapmatch.fetch_map                 # Moscow, ~84 MB
  python3 -m mapmatch.fetch_map --area moscow-region
  python3 -m mapmatch.fetch_map --bbox 55.74,37.60,55.78,37.66   # small patch via Overpass

If long downloads fail, fetch manually — URL is printed with --print-url; curl supports resume:
`curl -L -C - -o maps/Moscow.osm.pbf <url>`.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

MAPS_DIR = Path(__file__).resolve().parents[2] / "maps"

AREAS = {
    # BBBike cuts by city — most compact option for Moscow.
    "moscow": (
        "https://download.bbbike.org/osm/bbbike/Moscow/Moscow.osm.pbf",
        "Moscow.osm.pbf",
        "~84 MB",
    ),
    # Geofabrik: Moscow region — needed if bags go beyond MKAD.
    "moscow-region": (
        "https://download.geofabrik.de/russia/central-fed-district-latest.osm.pbf",
        "central-fed-district.osm.pbf",
        "~870 MB",
    ),
}

OVERPASS = "https://overpass-api.de/api/interpreter"
DRIVABLE = (
    "motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street"
    "|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link"
)


def fetch_pbf(
    area: str,
    out_dir: Path,
    print_url: bool,
    chunked: bool = False,
    chunk_bytes: int = 15000,
    workers: int = 8,
) -> int:
    url, name, size = AREAS[area]
    if print_url:
        print(url)
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)
    dst = out_dir / name
    print(f"{area} ({size}) → {dst}")
    if chunked:
        return fetch_chunked(url, dst, chunk_bytes, workers)
    # -C -: resume. Long downloads fail; rerunning continues from the break.
    rc = subprocess.call(
        [
            "curl",
            "-fL",
            "-C",
            "-",
            "--retry",
            "5",
            "--retry-delay",
            "2",
            "-o",
            str(dst),
            url,
        ]
    )
    if rc != 0:
        print(
            f"curl returned {rc}. Partial file saved — run again to resume.",
            file=sys.stderr,
        )
        return rc
    print(f"done: {dst.stat().st_size / 1e6:.0f} MB")
    return 0


def fetch_chunked(url: str, dst: Path, chunk_bytes: int, workers: int) -> int:
    """Download file in chunk_bytes ranges.

    For environments where the connection dies after the first tens of KB (some proxies):
    one 15 KB request completes in 0.2 s, while 256 KB hangs and drops. Total size is the
    same as a normal download.
    """
    req = urllib.request.Request(url, method="HEAD")
    with urllib.request.urlopen(req, timeout=30) as r:
        total = int(r.headers["Content-Length"])
        if r.headers.get("Accept-Ranges", "").lower() != "bytes":
            print(
                "server does not support ranges — chunked mode will not work",
                file=sys.stderr,
            )
            return 2
    print(
        f"{total / 1e6:.0f} MB in {chunk_bytes // 1024} KB ranges "
        f"({(total + chunk_bytes - 1) // chunk_bytes} requests, {workers} threads)"
    )

    part = dst.with_suffix(dst.suffix + ".part")
    with part.open("wb") as f:
        f.truncate(total)

    ranges = [(o, min(o + chunk_bytes, total) - 1) for o in range(0, total, chunk_bytes)]
    done = [0]

    def grab(rng):
        a, b = rng
        for attempt in range(6):
            try:
                r = urllib.request.Request(url, headers={"Range": f"bytes={a}-{b}"})
                with urllib.request.urlopen(r, timeout=30) as resp:
                    data = resp.read()
                if len(data) != b - a + 1:
                    continue
                with open(part, "r+b") as f:
                    f.seek(a)
                    f.write(data)
                done[0] += 1
                if done[0] % 500 == 0:
                    print(
                        f"  {done[0]}/{len(ranges)} ({done[0] * chunk_bytes / 1e6:.0f} MB)",
                        flush=True,
                    )
                return True
            except Exception:
                continue
        return False

    with ThreadPoolExecutor(max_workers=workers) as pool:
        ok = list(pool.map(grab, ranges))
    if not all(ok):
        print(f"missing {ok.count(False)} chunks — run again", file=sys.stderr)
        return 1

    part.rename(dst)
    print(f"done: {dst} ({dst.stat().st_size / 1e6:.0f} MB)")
    return 0


def fetch_bbox(bbox: str, out_dir: Path) -> int:
    """Small patch via Overpass — for debugging, not city coverage."""
    lat0, lon0, lat1, lon1 = (float(x) for x in bbox.split(","))
    query = (
        f"[out:json][timeout:180];\n"
        f'way["highway"~"^({DRIVABLE})$"]({lat0},{lon0},{lat1},{lon1});\n'
        f"out geom;\n"
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    dst = out_dir / f"bbox_{lat0}_{lon0}_{lat1}_{lon1}.json"
    print(f"Overpass bbox {bbox} → {dst}")
    rc = subprocess.call(
        ["curl", "-f", "--data-urlencode", f"data={query}", "-o", str(dst), OVERPASS]
    )
    if rc != 0:
        print("Overpass did not respond (often busy) — retry later.", file=sys.stderr)
        return rc
    print(f"done: {dst.stat().st_size / 1e6:.1f} MB")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--area", choices=sorted(AREAS), default="moscow")
    p.add_argument(
        "--bbox", default=None, help="lat0,lon0,lat1,lon1 — fetch patch via Overpass"
    )
    p.add_argument("--out", type=Path, default=MAPS_DIR)
    p.add_argument("--print-url", action="store_true", help="print URL only and exit")
    p.add_argument(
        "--chunked",
        action="store_true",
        help="range download: workaround for environments that drop after first KB",
    )
    p.add_argument("--chunk-bytes", type=int, default=15000)
    p.add_argument("--workers", type=int, default=8)
    args = p.parse_args()

    if args.bbox:
        return fetch_bbox(args.bbox, args.out)
    return fetch_pbf(
        args.area, args.out, args.print_url, args.chunked, args.chunk_bytes, args.workers
    )


if __name__ == "__main__":
    raise SystemExit(main())
