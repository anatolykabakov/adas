#!/usr/bin/env python3
"""Plot a bag on an OSM map: GNSS, raw odometry, and odometry after fitting.

What each line means:
  * **GNSS** — recorded fixes. Reference only; not fed to the algorithm.
  * **odometry** — shape from speed and yaw rate, placed at the GNSS start point with its
    heading. No fitting: shows how the shape drifts from roads due to odometry error.
  * **after fitting** — the same odometry snapped to the road network by elastic fitting
    (`mapmatch::fitTrack`): position, heading, speed scale, yaw-rate scale, and small
    corrections to turn angles and straight lengths are adjusted.

Initial pose here comes from GNSS — this illustrates what fitting does. Location search
without GNSS (beam search over the maneuver chain) is the next step; then the start will
be found automatically.

  python3 -m mapmatch.plot_on_map <bag> --map maps/Moscow.osm.admap -o out.png
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

import _path  # noqa: F401

from pyadas import core as pyadas
from mapmatch.track_from_bag import gnss_reference, motion_profile

mm = pyadas.mapmatch


def gnss_in_map_frame(bag: Path, road_map) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """GNSS in the same coordinate frame as the map (origin taken from the map)."""
    from vis.bag_io import load_topic_messages

    rows = load_topic_messages(bag, "sensors/gps/location")
    pts = [
        (float(m.latitude), float(m.longitude), float(getattr(m, "speed", 0.0) or 0.0))
        for _, m in [(r[0], r[1]) for r in rows]
        if abs(float(m.latitude)) > 1e-6
    ]
    if len(pts) < 5:
        return None
    frame = road_map.frame
    lat = [p[0] for p in pts]
    lon = [p[1] for p in pts]
    east, north = frame.to_local_many(lat, lon)
    spd = np.array([p[2] for p in pts])
    return np.column_stack([east, north]), spd


def initial_pose(track, xy: np.ndarray, spd: np.ndarray) -> Tuple[float, float, float]:
    """Initial alignment: rigidly (rotate+translate, no scale) align track shape to GNSS.

    A single start point is not enough: GNSS does not begin at the same moment as odometry,
    and a heading error at the start pushes the far end of the track hundreds of meters away.
    This is not a trick — it substitutes the reference instead of searching for location, to
    see what fitting itself does. Search without GNSS is a separate step.
    """
    step = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    good = np.flatnonzero((step > 1.0) & (step < 200.0) & (spd[:-1] > 2.0))
    if len(good) < 5:
        return float(xy[0, 0]), float(xy[0, 1]), 0.0
    ref = xy[good]
    # Parameterize GNSS and track by path length and match by fraction of path.
    s_ref = np.concatenate(
        [[0.0], np.cumsum(np.hypot(np.diff(ref[:, 0]), np.diff(ref[:, 1])))]
    )
    s_trk = np.array(track.s_m)
    u = np.linspace(0.0, 1.0, 200)
    px = np.interp(u * s_ref[-1], s_ref, ref[:, 0])
    py = np.interp(u * s_ref[-1], s_ref, ref[:, 1])
    qx = np.interp(u * s_trk[-1], s_trk, np.array(track.x_m))
    qy = np.interp(u * s_trk[-1], s_trk, np.array(track.y_m))

    # Procrustes without scale: R and t minimizing |R·q + t − p|.
    pc = np.column_stack([px - px.mean(), py - py.mean()])
    qc = np.column_stack([qx - qx.mean(), qy - qy.mean()])
    H = qc.T @ pc
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    heading = float(np.arctan2(R[1, 0], R[0, 0]))
    t = np.array([px.mean(), py.mean()]) - R @ np.array([qx.mean(), qy.mean()])
    # Where the track start will land (it has x=y=0 in its own frame).
    return float(t[0]), float(t[1]), heading


def street_sequence(road_map, xs, ys, step_m: float = 25.0, min_len_m: float = 100.0):
    """Which streets the track follows: sequence of segments with distance to the road."""
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
    out = []
    last = None
    for target in np.arange(0.0, s[-1], step_m):
        i = min(int(np.searchsorted(s, target)), len(xs) - 1)
        _, d, name = road_map.nearest_edge(float(xs[i]), float(ys[i]), 250.0)
        name = name or "(unnamed)"
        if name != last:
            out.append(
                {"name": name, "d_min": d, "d_max": d, "n": 1, "x": xs[i], "y": ys[i]}
            )
            last = name
        else:
            out[-1]["d_min"] = min(out[-1]["d_min"], d)
            out[-1]["d_max"] = max(out[-1]["d_max"], d)
            out[-1]["n"] += 1
    return [o for o in out if o["n"] * step_m >= min_len_m]


def place_raw(
    track, x0: float, y0: float, heading: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Rotate and translate raw odometry to the start — no deformation."""
    x = np.array(track.x_m)
    y = np.array(track.y_m)
    c, s = np.cos(heading), np.sin(heading)
    return x0 + c * x - s * y, y0 + s * x + c * y


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("bag", type=Path)
    p.add_argument(
        "--map",
        type=Path,
        default=Path(__file__).resolve().parents[5] / "maps" / "Moscow.osm.admap",
    )
    p.add_argument("-o", "--out", type=Path, default=Path("track_on_map.png"))
    p.add_argument("--margin-m", type=float, default=400.0)
    p.add_argument("--sample-m", type=float, default=10.0)
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

    ref = gnss_in_map_frame(args.bag, road_map)
    if ref is None:
        raise SystemExit(
            "no GNSS in bag — without it there is nothing to use as initial position"
        )
    gnss_xy, gnss_spd = ref
    x0, y0, heading = initial_pose(track, gnss_xy, gnss_spd)

    raw_x, raw_y = place_raw(track, x0, y0, heading)

    cfg = mm.FitConfig()
    cfg.sample_m = args.sample_m
    fit = mm.fit_track(road_map, track, x0, y0, heading, cfg)
    fit_x = np.array(fit.x_m)
    fit_y = np.array(fit.y_m)

    # --- numbers
    print(
        f"{args.bag.name}: track {track.length_m:.0f} m, turns "
        f"{sum(1 for m in track.maneuvers if m.is_turn)}"
    )
    print(
        f"fit in {fit.iterations} iterations: to roads median {fit.median_m:.1f} m, "
        f"p95 {fit.p95_m:.1f} m, rms {fit.rms_m:.1f} m"
    )
    print(f"  speed scale {fit.speed_scale:.3f}, yaw-rate scale {fit.yaw_rate_scale:.3f}")
    print(f"  turn corrections, °: {[round(a, 1) for a in fit.turn_corr_deg]}")
    print(f"  straight length corrections: {[round(a, 3) for a in fit.straight_corr]}")
    print(
        f"  heading drift by block, °/100 m: {[round(a, 2) for a in fit.drift_deg_per_100m]}"
    )
    print(f"  deformation {fit.deform_cost:.2f} σ, score {fit.score:.2f}")

    def dist_to_road(xs, ys):
        d = [
            road_map.nearest_edge(float(a), float(b), 200.0)[1]
            for a, b in zip(xs[::5], ys[::5])
        ]
        d = np.array([x for x in d if x >= 0])
        return np.median(d), np.percentile(d, 95)

    for label, (xs, ys) in (
        ("GNSS", (gnss_xy[:, 0], gnss_xy[:, 1])),
        ("raw odometry", (raw_x, raw_y)),
        ("after fitting", (fit_x, fit_y)),
    ):
        med, p95 = dist_to_road(np.asarray(xs), np.asarray(ys))
        print(f"  {label:20s} to roads: median {med:5.1f} m, p95 {p95:5.1f} m")

    seq_ref = street_sequence(road_map, gnss_xy[:, 0], gnss_xy[:, 1])
    seq_fit = street_sequence(road_map, fit_x, fit_y)
    print()
    print("  streets from GNSS (reference): " + " → ".join(o["name"] for o in seq_ref))
    print("  streets after fitting:         " + " → ".join(o["name"] for o in seq_fit))

    # --- map
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    all_x = np.concatenate([gnss_xy[:, 0], raw_x, fit_x])
    all_y = np.concatenate([gnss_xy[:, 1], raw_y, fit_y])
    bx0, bx1 = all_x.min() - args.margin_m, all_x.max() + args.margin_m
    by0, by1 = all_y.min() - args.margin_m, all_y.max() + args.margin_m
    road_x, road_y = road_map.polylines_in_bbox(bx0, by0, bx1, by1)

    fig, ax = plt.subplots(figsize=(13, 13))
    ax.plot(road_x, road_y, lw=1.4, color="0.62", zorder=1, label="OSM roads")
    ax.plot(raw_x, raw_y, lw=1.6, color="tab:orange", zorder=2, label="raw odometry")
    ax.plot(
        fit_x, fit_y, lw=2.0, color="tab:blue", zorder=4, label="odometry after fitting"
    )
    ax.plot(
        gnss_xy[:, 0],
        gnss_xy[:, 1],
        lw=1.4,
        color="tab:green",
        zorder=3,
        label="GNSS (reference)",
    )
    ax.plot([x0], [y0], "k*", ms=12, zorder=5, label="start")

    # Street labels along the matched route — the answer to "where we drove".
    for o in seq_fit:
        ax.annotate(
            o["name"],
            (o["x"], o["y"]),
            fontsize=9,
            color="tab:blue",
            zorder=6,
            textcoords="offset points",
            xytext=(8, 8),
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.75),
        )
    route = " → ".join(o["name"] for o in seq_fit)
    ax.set_title(
        f"{args.bag.name}: {track.length_m / 1000:.1f} km\n"
        f"after fitting: {route}\n"
        f"to roads median {fit.median_m:.1f} m, deformation {fit.deform_cost:.2f} σ",
        fontsize=11,
    )
    ax.set_xlim(bx0, bx1)
    ax.set_ylim(by0, by1)
    ax.set_aspect("equal")
    ax.set_xlabel("east, m")
    ax.set_ylabel("north, m")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"map → {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
