#!/usr/bin/env python3
"""Where we drove — from odometry and an OSM map, without GNSS.

Two steps, both in C++ (`pyadas.core.mapmatch`):
  1. `search_routes` — candidates by maneuver chain: "straight 2.3 km → right 47° → straight 130 m →
     left 113° → …". Routes are enumerated on the graph, not by geometry, so this is cheap.
  2. `fit_track` — elastic fitting checks each candidate: the track is snapped to its roads, and
     we measure how much deformation was required. The correct location snaps with almost no edits.

Output is top-K streets with a score and **margin to second place**: without margin you cannot say "we are here".

  python3 -m mapmatch.locate <bag>
  python3 -m mapmatch.locate <bag> --top 5 --plot out.png
  python3 -m mapmatch.locate <bag> --verify-gnss     # compare answer to recorded GNSS
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from mapmatch.plot_on_map import street_sequence
from mapmatch.track_from_bag import motion_profile

mm = pyadas.mapmatch

# Repo-root maps/ (same as fetch_map.py); build with osm_graph after fetch_map.
DEFAULT_MAP = Path(__file__).resolve().parents[2] / "maps" / "Moscow.osm.admap"


def route_streets(road_map, dir_edges, max_items: int = 8) -> str:
    """Route street names in order, without repeats."""
    names: List[str] = []
    for de in dir_edges:
        name = road_map.edge_name(de >> 1) or "—"
        if not names or names[-1] != name:
            names.append(name)
    named = [n for n in names if n != "—"]
    if len(named) > max_items:
        named = named[: max_items - 1] + ["…"]
    return " → ".join(named) if named else "(unnamed roads)"


def truth_words(truth: Optional[str]) -> List[str]:
    """Reference keywords — to check whether the correct route appears in the list."""
    if not truth:
        return []
    # OSM street names in this map are Russian; keep the local type word when stripping.
    return [
        w
        for w in truth.replace("улица", "").replace("→", " ").split()
        if len(w) > 5 and w[0].isupper()
    ]


def gnss_truth(bag: Path, road_map) -> Optional[str]:
    """Reference: streets from recorded GNSS. For verifying the answer only."""
    from vis.bag_io import list_topics, load_topic_messages

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
    east, north = road_map.frame.to_local_many([p[0] for p in pts], [p[1] for p in pts])
    seq = street_sequence(road_map, np.array(east), np.array(north))
    return " → ".join(o["name"] for o in seq)


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("bag", type=Path)
    p.add_argument("--map", type=Path, default=DEFAULT_MAP)
    p.add_argument("--top", type=int, default=5, help="how many locations to show")
    p.add_argument(
        "--fit-top",
        type=int,
        default=15,
        help="how many candidates to check with fitting",
    )
    p.add_argument("--turn-tol-deg", type=float, default=25.0)
    p.add_argument("--dist-tol-rel", type=float, default=0.15)
    p.add_argument("--beam", type=int, default=400)
    p.add_argument("--verbose", action="store_true", help="search statistics")
    p.add_argument(
        "--heading-weight",
        type=float,
        default=6.0,
        help="weight of heading-profile match in the final score",
    )
    p.add_argument("--verify-gnss", action="store_true", help="compare to recorded GNSS")
    p.add_argument("--plot", type=Path, default=None)
    args = p.parse_args()

    road_map = mm.RoadMap()
    if not road_map.load(str(args.map)):
        raise SystemExit(
            f"could not open map {args.map} — run python3 -m mapmatch.osm_graph <pbf> first"
        )

    t, v, w = motion_profile(args.bag)
    track = mm.build_track(list(t), list(v), list(w))
    if track.length_m <= 0:
        raise SystemExit("track not built")
    turns = [m for m in track.maneuvers if m.is_turn]
    print(f"{args.bag.name}: {track.length_m / 1000:.2f} km, turns {len(turns)}")
    print(f"  {track.describe()}")
    if len(turns) < 2:
        print("  too few turns — such a track cannot be localized by shape")

    scfg = mm.SearchConfig()
    scfg.turn_tol_deg = args.turn_tol_deg
    scfg.dist_tol_rel = args.dist_tol_rel
    scfg.beam_width = args.beam
    scfg.verbose = args.verbose
    scfg.max_candidates = max(args.fit_top * 3, 30)

    t0 = time.time()
    cands = mm.search_routes(road_map, track, scfg)
    t_search = time.time() - t0
    print(
        f"\ngraph search: {len(cands)} candidates in {t_search:.1f} s "
        f"(map {road_map.edge_count:,} edges)"
    )
    if not cands:
        print("no candidates — relax tolerances (--turn-tol-deg, --dist-tol-rel)")
        return 1

    fcfg = mm.FitConfig()
    fcfg.sample_m = 20.0  # subsample: this is selection, not final snapping
    fcfg.iterations = 12
    fcfg.anneal_steps = 3

    t0 = time.time()
    scored = []
    for c in cands[: args.fit_top]:
        fit = mm.fit_track_to_route(road_map, track, c.dir_edges, fcfg)
        if fit.ok:
            # Final score combines geometry (fitting) and heading-profile match (search).
            # Fitting alone lets a long avenue with a duplicate beat the correct place:
            # the track snaps to it just as well, but heading matches worse.
            total = fit.score + args.heading_weight * c.cost
            scored.append((total, fit, c))
    t_fit = time.time() - t0
    scored.sort(key=lambda x: x[0])
    print(f"selection: {len(scored)} candidates coarsely fitted in {t_fit:.1f} s")

    # Refit leaders at full precision: on a coarse grid the gap between similar places
    # drowns in noise, and the correct place may not rank first.
    fine = mm.FitConfig()
    fine.sample_m = 10.0
    fine.iterations = 30
    fine.anneal_steps = 4
    t0 = time.time()
    refined = []
    for _, _, c in scored[: max(args.top * 2, 10)]:
        fit = mm.fit_track_to_route(road_map, track, c.dir_edges, fine)
        if fit.ok:
            refined.append((fit.score + args.heading_weight * c.cost, fit, c))
    refined.sort(key=lambda x: x[0])
    if refined:
        scored = refined + scored[len(refined) :]
    print(f"refinement of {len(refined)} leaders in {time.time() - t0:.1f} s\n")

    print(
        f"{'#':>2} {'total':>7} {'geom.':>7} {'head.':>6} {'median':>8} {'p95':>7} {'deform.':>7}  route"
    )
    for i, (total, fit, c) in enumerate(scored[: args.top], 1):
        print(
            f"{i:>2} {total:>7.1f} {fit.score:>7.1f} {c.cost:>6.2f} {fit.median_m:>7.1f}m {fit.p95_m:>6.1f}m "
            f"{fit.deform_cost:>6.2f}σ  {route_streets(road_map, c.dir_edges)}"
        )

    if len(scored) >= 2:
        margin = scored[1][0] - scored[0][0]
        rel = margin / max(scored[0][0], 1e-6)
        verdict = (
            "confident" if rel > 0.35 else ("ambiguous" if rel > 0.1 else "cannot choose")
        )
        print(f"\nmargin to second place: {margin:.1f} ({rel * 100:.0f} %) — {verdict}")

    if args.verify_gnss:
        truth = gnss_truth(args.bag, road_map)
        best = route_streets(road_map, scored[0][2].dir_edges) if scored else "—"
        hit = [
            i
            for i, (_, _, c) in enumerate(scored, 1)
            if any(w in route_streets(road_map, c.dir_edges) for w in truth_words(truth))
        ]
        if hit:
            print(f"correct route in list at ranks: {hit[:5]}")
        else:
            print("correct route not found even among checked candidates")
        print(f"\nanswer:     {best}")
        print(f"reference:  {truth or 'no GNSS in bag'}")

    if args.plot and scored:
        fit, c = scored[0][1], scored[0][2]
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fx, fy = np.array(fit.x_m), np.array(fit.y_m)
        m = 500.0
        bx0, bx1 = fx.min() - m, fx.max() + m
        by0, by1 = fy.min() - m, fy.max() + m
        rx, ry = road_map.polylines_in_bbox(bx0, by0, bx1, by1)

        fig, ax = plt.subplots(figsize=(12, 12))
        ax.plot(rx, ry, lw=1.2, color="0.72", zorder=1, label="OSM roads")
        for de in c.dir_edges:
            ex, ey = road_map.edge_polyline(de >> 1)
            ax.plot(ex, ey, lw=3.0, color="tab:red", alpha=0.35, zorder=2)
        ax.plot(fx, fy, lw=2.0, color="tab:blue", zorder=3, label="track after fitting")
        ax.plot([fx[0]], [fy[0]], "k*", ms=13, zorder=4, label="start")
        ax.set_aspect("equal")
        ax.set_xlim(bx0, bx1)
        ax.set_ylim(by0, by1)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=9)
        ax.set_title(
            f"{args.bag.name} — found without GNSS\n{route_streets(road_map, c.dir_edges)}\n"
            f"to roads median {fit.median_m:.1f} m, deformation {fit.deform_cost:.2f} σ",
            fontsize=11,
        )
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"map → {args.plot}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
