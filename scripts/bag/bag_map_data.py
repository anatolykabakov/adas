#!/usr/bin/env python3
"""What the OSM map said the road ahead looked like, over a recorded run.

This is the analysis side of the `map_data` service (`cpp/services/map_data_service.h`), which is the
dragonpilot `selfdrive/mapd` chain ported to C++: snap the car onto the road graph, walk the route ahead,
differentiate the heading to get curvature, cut it into turn sections with `v = sqrt(a_lat / kappa)`.

Two modes, chosen automatically:

  * **logged** — the bag contains `map/local`, i.e. it was driven with the service enabled. Read as recorded.
  * **replay** — the bag has no `map/local` (every run before this service existed). The bag's GPS and pose
    are pushed through the same C++ code offline, so old runs can be analysed too. The numbers are then the
    numbers the service *would* have produced, which is not quite the same claim: on the phone it runs at
    2 Hz off a live pose, here at whatever rate the fixes arrived.

What to look at first, in order:

  1. **matched fraction and match distance.** Everything else is conditional on the car being snapped to the
     right road. On this map, a dual carriageway is one centreline, so 10-20 m is normal and 35 m is
     suspicious. An unmatched stretch is not a bug to hide — it is where the map has no road.
  2. **node spacing.** The curvature is only as good as the geometry it came from. On the motorway sections
     of these runs the map has a node every 70-170 m, so `kappa` there is an average over hundreds of
     metres, and a real 500 m-radius bend can read as almost straight.
  3. **map vs the road actually driven.** `--vs-driven` compares the heading change the map predicts over the
     next `--horizon-m` against the one the car then made. This needs no second estimator to be correct, so
     it is the test that can actually convict the map — and it separates "the route walked onto the wrong
     road" from "the drawing has a kink that looks like a corner".
  4. **map vs vision.** `--compare-vision` puts the map's curvature next to the model's
     (`vision/model_long` / `control/long_plan`) at the same instant. That comparison is the reason this
     service exists: before any map speed reaches the controller, we need to know where the two disagree
     and which one is right.

  python3 bag/bag_map_data.py adas_logs/2026_08_07_19_04_05
  python3 bag/bag_map_data.py adas_logs/2026_08_07_19_04_05 --plot out.png --compare-vision
  python3 bag/bag_map_data.py adas_logs/* --map maps/Moscow.osm.admap
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

import _path  # noqa: F401
from pyadas import core as pyadas
from vis.bag_io import list_topics, load_topic_messages

mm = pyadas.mapmatch

MAP_TOPIC = "map/local"
DEFAULT_MAP = "maps/Moscow.osm.admap"


@dataclass
class Sample:
    """One evaluation of the route ahead, from either source."""

    t_ms: int
    x: float
    y: float
    yaw: float
    speed_mps: float
    matched: bool
    match_dist_m: float = float("nan")
    road_name: str = ""
    node_spacing_m: float = float("nan")
    route_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    route_x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    route_y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    kappa: np.ndarray = field(default_factory=lambda: np.zeros(0))
    turns: List[Tuple[float, float, float, float, int]] = field(default_factory=list)
    build_ms: float = float("nan")

    def kappa_ahead(self, horizon_m: float) -> float:
        """Peak |curvature| within `horizon_m` — the number a speed planner would act on."""
        if self.route_s.size == 0:
            return float("nan")
        sel = self.route_s <= horizon_m
        return float(np.max(np.abs(self.kappa[sel]))) if np.any(sel) else float("nan")

    def turn_speed_mps(self, horizon_m: float) -> float:
        """Lowest turn-section speed starting within `horizon_m`, or nan if the road is straight."""
        v = [t[3] for t in self.turns if t[0] <= horizon_m]
        return float(min(v)) if v else float("nan")


def load_map(path: Path):
    road_map = mm.RoadMap()
    if not road_map.load(str(path)):
        raise SystemExit(f"cannot load map: {path}")
    return road_map


def samples_from_log(bag: Path) -> List[Sample]:
    out: List[Sample] = []
    for t_ms, p, _ in load_topic_messages(bag, MAP_TOPIC):
        s = Sample(
            t_ms=t_ms,
            x=p.map_x,
            y=p.map_y,
            yaw=p.yaw,
            speed_mps=p.speed_mps,
            matched=p.matched,
            match_dist_m=p.match_dist_m,
            road_name=p.road_name,
            node_spacing_m=p.route_node_spacing_m,
            build_ms=p.build_ms,
        )
        if p.matched and len(p.route_x):
            step = p.route_step_m or 5.0
            s.route_x = np.asarray(p.route_x, dtype=float)
            s.route_y = np.asarray(p.route_y, dtype=float)
            s.kappa = np.asarray(p.route_kappa, dtype=float)
            s.route_s = np.arange(len(s.route_x), dtype=float) * step
            s.turns = [
                (t.start_m, t.end_m, t.kappa, t.speed_mps, t.sign) for t in p.turns
            ]
        out.append(s)
    return out


def _pose_track(
    bag: Path,
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    rows = load_topic_messages(bag, "localization/pose")
    if len(rows) < 10:
        return None
    t = np.asarray([r[0] for r in rows], dtype=float)
    x = np.asarray([r[1].x for r in rows], dtype=float)
    y = np.asarray([r[1].y for r in rows], dtype=float)
    yaw = np.unwrap(np.asarray([r[1].yaw for r in rows], dtype=float))
    v = np.asarray([r[1].v for r in rows], dtype=float)
    return t, x, y, yaw, v


def samples_from_replay(
    bag: Path, road_map, cfg, min_speed: float, stride: int
) -> List[Sample]:
    """Re-run the service's logic on a bag recorded without it.

    Where this differs from the device, and it matters when reading the numbers: the service runs on a timer
    at `update_hz` off a continuously dead-reckoned position, while this evaluates *at each GPS fix* and
    takes heading and speed from the pose nearest that fix. So replay never exercises the dead-reckoning
    between fixes, which on these runs can be tens of seconds — it is the optimistic case for position.
    Heading falls back to GPS course when there is no pose; a run with neither cannot be placed at all.
    """
    fixes = [
        (
            t,
            m.latitude,
            m.longitude,
            float(getattr(m, "speed", 0.0) or 0.0),
            float(getattr(m, "bearing", 0.0) or 0.0),
        )
        for t, m, _ in load_topic_messages(bag, "sensors/gps/data")
        if abs(m.latitude) > 1e-9 or abs(m.longitude) > 1e-9
    ]
    if not fixes:
        return []

    frame = road_map.frame
    pose = _pose_track(bag)
    out: List[Sample] = []

    for t_ms, lat, lon, gps_speed, bearing in fixes[::stride]:
        ax, ay = frame.to_local(lat, lon)
        if pose is not None:
            pt, px, py, pyaw, pv = pose
            i = int(np.searchsorted(pt, t_ms))
            if i >= len(pt):
                i = len(pt) - 1
            if abs(pt[i] - t_ms) > 2000:  # no pose near this fix
                continue
            # The anchor is the pose at the fix, so at the fix itself the position is the fix.
            x, y, yaw, speed = ax, ay, float(pyaw[i]), float(pv[i])
        else:
            if gps_speed <= 2.0:
                continue
            x, y, yaw, speed = ax, ay, math.radians(90.0 - bearing), gps_speed

        if speed < min_speed:
            continue

        r = mm.build_route_ahead(road_map, x, y, yaw, cfg)
        s = Sample(t_ms=t_ms, x=x, y=y, yaw=yaw, speed_mps=speed, matched=r.matched)
        if r.matched:
            s.match_dist_m = r.match_dist_m
            s.road_name = r.road_name
            s.node_spacing_m = r.node_spacing_m
            s.route_s = np.asarray(r.s_m, dtype=float)
            s.route_x = np.asarray(r.x, dtype=float)
            s.route_y = np.asarray(r.y, dtype=float)
            s.kappa = np.asarray(r.kappa, dtype=float)
            s.turns = [
                (t.start_m, t.end_m, t.kappa, t.speed_mps, t.sign) for t in r.turns
            ]
        out.append(s)
    return out


def driven_heading(bag: Path) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Where the car actually went: (timestamp_ms, arc length, unwrapped heading) from the pose."""
    pose = _pose_track(bag)
    if pose is None:
        return None
    t, x, y, yaw, _v = pose
    s_m = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    return t, s_m, yaw


def compare_against_driven(
    bag: Path, samples: Sequence[Sample], ahead_m: float
) -> Optional[Dict[str, Any]]:
    """Did the road actually do what the map said it would?

    The honest test of this service, and the only one that does not need another estimator to be right. For
    each sample it takes the heading change the map predicts over the next `ahead_m` — the integral of the
    curvature, which is what the geometry actually claims — and the heading change the car then made over
    its next `ahead_m` of travel. Two failures show up separately:

      * **the walk went to the wrong road** — the map claims a turn the car never made;
      * **the drawing has a kink** — a peak curvature sharp enough to imply a tight radius, at a place where
        the heading barely changes. That peak is what a naive speed planner would brake for, and it is not a
        corner. It is one node placed a few metres off the centreline.

    The second is why the peak is reported next to the integral rather than on its own.
    """
    driven = driven_heading(bag)
    if driven is None:
        return None
    t_pose, s_pose, yaw_pose = driven

    rows = []
    for s in samples:
        if not s.matched or s.route_s.size < 3:
            continue
        i = min(int(np.searchsorted(t_pose, s.t_ms)), len(t_pose) - 1)
        j = int(np.searchsorted(s_pose, s_pose[i] + ahead_m))
        if j >= len(t_pose):
            continue
        sel = s.route_s <= ahead_m
        if int(np.count_nonzero(sel)) < 3:
            continue
        map_turn = float(np.degrees(np.trapezoid(s.kappa[sel], s.route_s[sel])))
        car_turn = float(np.degrees(yaw_pose[j] - yaw_pose[i]))
        peak = float(np.max(np.abs(s.kappa[sel])))
        rows.append((map_turn, car_turn, peak))

    if len(rows) < 20:
        return None
    a = np.asarray(rows)
    err = np.abs(a[:, 0] - a[:, 1])
    sharp = a[:, 2] > 1.0 / 100.0  # peak implies a radius under 100 m
    return {
        "n": len(rows),
        "err_med": float(np.median(err)),
        "err_p90": float(np.percentile(err, 90)),
        "phantom_turn": int(
            np.count_nonzero((np.abs(a[:, 0]) > 20.0) & (np.abs(a[:, 1]) < 10.0))
        ),
        "sharp": int(np.count_nonzero(sharp)),
        "sharp_but_straight": int(np.count_nonzero(sharp & (np.abs(a[:, 1]) < 20.0))),
    }


def vision_curvature(bag: Path) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Curvature the vision path predicted: `LongPlanState.kappa_ahead`, a high percentile over the
    controller's preview window (`utils/curvature_preview.h`).

    Note the two are not measuring the same window — the map summary here uses `--horizon-m`, the model uses
    `curv_preview_s` seconds of travel — so a correlation below 1 is expected even if both are right.
    Returns (timestamp_ms, |kappa|), or None on runs recorded before the field existed.
    """
    rows = load_topic_messages(bag, "control/long_plan")
    if not rows or not hasattr(rows[0][1], "kappa_ahead"):
        return None
    t = np.asarray([r[0] for r in rows], dtype=float)
    k = np.asarray([abs(r[1].kappa_ahead) for r in rows], dtype=float)
    return t, k


def summarise(
    bag: Path,
    samples: Sequence[Sample],
    mode: str,
    horizon_m: float,
    vision: Optional[Tuple[np.ndarray, np.ndarray]],
    driven: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    n = len(samples)
    matched = [s for s in samples if s.matched]
    row: Dict[str, Any] = {"bag": bag.name, "mode": mode, "n": n, "matched": len(matched)}
    if not matched:
        return row

    d = np.asarray([s.match_dist_m for s in matched])
    sp = np.asarray([s.node_spacing_m for s in matched])
    kap = np.asarray([s.kappa_ahead(horizon_m) for s in matched])
    kap = kap[np.isfinite(kap)]
    tv = np.asarray([s.turn_speed_mps(horizon_m) for s in matched])
    tv = tv[np.isfinite(tv)]
    build = np.asarray([s.build_ms for s in matched])
    build = build[np.isfinite(build)]

    row.update(
        match_med=float(np.median(d)),
        match_p95=float(np.percentile(d, 95)),
        spacing_med=float(np.median(sp)),
        spacing_p90=float(np.percentile(sp, 90)),
        kappa_med=float(np.median(kap)) if kap.size else float("nan"),
        kappa_p95=float(np.percentile(kap, 95)) if kap.size else float("nan"),
        turn_frac=float(len(tv)) / len(matched),
        turn_v_min=float(np.min(tv)) if tv.size else float("nan"),
        build_med=float(np.median(build)) if build.size else float("nan"),
    )

    names: Dict[str, int] = {}
    for s in matched:
        if s.road_name:
            names[s.road_name] = names.get(s.road_name, 0) + 1
    row["roads"] = sorted(names.items(), key=lambda kv: -kv[1])[:5]

    if driven is not None:
        row["driven"] = driven

    if vision is not None:
        vt, vk = vision
        pairs = []
        for s in matched:
            j = int(np.argmin(np.abs(vt - s.t_ms)))
            if abs(vt[j] - s.t_ms) > 500:
                continue
            k = s.kappa_ahead(horizon_m)
            if np.isfinite(k):
                pairs.append((k, vk[j]))
        if len(pairs) > 20:
            a = np.asarray(pairs)
            row["vision_n"] = len(pairs)
            row["vision_map_med"] = float(np.median(a[:, 0]))
            row["vision_mdl_med"] = float(np.median(a[:, 1]))
            row["vision_corr"] = float(np.corrcoef(a[:, 0], a[:, 1])[0, 1])
    return row


def print_summary(row: Dict[str, Any]) -> None:
    print(f"\n{row['bag']}  [{row['mode']}]")
    if not row.get("matched"):
        print(f"  {row['n']} samples, none matched to the map")
        return
    print(
        f"  samples          {row['matched']} of {row['n']} matched ({100.0 * row['matched'] / max(row['n'], 1):.0f}%)"
    )
    print(
        f"  match distance   median {row['match_med']:.1f} m, p95 {row['match_p95']:.1f} m"
    )
    print(
        f"  map node spacing median {row['spacing_med']:.0f} m, p90 {row['spacing_p90']:.0f} m"
        f"   <- the resolution the curvature came from"
    )
    if np.isfinite(row["kappa_med"]):
        r_med = 1.0 / row["kappa_med"] if row["kappa_med"] > 1e-9 else float("inf")
        r_p95 = 1.0 / row["kappa_p95"] if row["kappa_p95"] > 1e-9 else float("inf")
        print(
            f"  |kappa| ahead    median {row['kappa_med']:.5f} 1/m (R {r_med:.0f} m), "
            f"p95 {row['kappa_p95']:.5f} (R {r_p95:.0f} m)"
        )
    print(
        f"  turn sections    on {100.0 * row['turn_frac']:.0f}% of samples"
        + (
            f", slowest {row['turn_v_min'] * 3.6:.0f} km/h"
            if np.isfinite(row["turn_v_min"])
            else ""
        )
    )
    if np.isfinite(row.get("build_med", float("nan"))):
        print(f"  build time       median {row['build_med']:.2f} ms")
    if row.get("roads"):
        print("  roads            " + ", ".join(f"{n} ({c})" for n, c in row["roads"]))
    d = row.get("driven")
    if d:
        print(
            f"  vs driven road   n={d['n']}, heading change over the next {row.get('horizon', 200):.0f} m "
            f"off by median {d['err_med']:.0f}°, p90 {d['err_p90']:.0f}°"
        )
        print(f"                   route on the wrong road: {d['phantom_turn']} samples")
        print(
            f"                   peak implies R<100 m on {d['sharp']}, of which {d['sharp_but_straight']} "
            f"barely turn -> drawing kinks, not corners"
        )
    if "vision_corr" in row:
        print(
            f"  vs vision        n={row['vision_n']} corr {row['vision_corr']:+.2f}, "
            f"map median {row['vision_map_med']:.5f} vs model {row['vision_mdl_med']:.5f} 1/m"
        )


def plot(
    bag: Path, samples: Sequence[Sample], road_map, out: Path, horizon_m: float
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    matched = [s for s in samples if s.matched]
    if not matched:
        print("nothing matched — no plot")
        return

    tx = np.asarray([s.x for s in samples])
    ty = np.asarray([s.y for s in samples])
    pad = 400.0
    x0, x1 = float(tx.min()) - pad, float(tx.max()) + pad
    y0, y1 = float(ty.min()) - pad, float(ty.max()) + pad

    fig, (ax_map, ax_k) = plt.subplots(2, 1, figsize=(13, 14), height_ratios=[3, 1])

    mx, my = road_map.polylines_in_bbox(x0, y0, x1, y1)
    ax_map.plot(mx, my, lw=0.6, color="0.8", zorder=1, label="OSM road graph")

    # Every route the service built, faint — where they fan out is where the "keep going straight" guess
    # was ambiguous, which is exactly where a map speed would have been wrong.
    for s in matched[:: max(1, len(matched) // 150)]:
        if s.route_x.size:
            ax_map.plot(
                s.route_x, s.route_y, lw=0.7, color="tab:orange", alpha=0.35, zorder=2
            )

    ax_map.plot(tx, ty, lw=1.6, color="tab:blue", zorder=3, label="driven (map frame)")
    unm = [s for s in samples if not s.matched]
    if unm:
        ax_map.scatter(
            [s.x for s in unm],
            [s.y for s in unm],
            s=9,
            color="tab:red",
            zorder=4,
            label=f"unmatched ({len(unm)})",
        )

    # Turn sections as points on the route, coloured by the speed they imply.
    px, py, pv = [], [], []
    for s in matched:
        for st, en, _k, v, _sg in s.turns:
            i = int(np.searchsorted(s.route_s, 0.5 * (st + en)))
            if i < s.route_x.size:
                px.append(s.route_x[i])
                py.append(s.route_y[i])
                pv.append(v * 3.6)
    if px:
        sc = ax_map.scatter(
            px, py, c=pv, s=14, cmap="viridis_r", vmin=30, vmax=130, zorder=5
        )
        fig.colorbar(sc, ax=ax_map, label="turn speed, km/h", shrink=0.6)

    ax_map.set_aspect("equal")
    ax_map.set_xlim(x0, x1)
    ax_map.set_ylim(y0, y1)
    ax_map.set_xlabel("east, m")
    ax_map.set_ylabel("north, m")
    ax_map.set_title(
        f"{bag.name} — route ahead from OSM, orange = what the service believed"
    )
    ax_map.legend(loc="upper right", fontsize=8)

    t0 = matched[0].t_ms
    tt = np.asarray([(s.t_ms - t0) / 1000.0 for s in matched])
    kk = np.asarray([s.kappa_ahead(horizon_m) for s in matched])
    dd = np.asarray([s.match_dist_m for s in matched])
    ax_k.plot(
        tt, kk, lw=1.0, color="tab:orange", label=f"peak |kappa| within {horizon_m:.0f} m"
    )
    ax_k.axhline(0.002, color="0.5", ls="--", lw=0.8, label="turn threshold 0.002 1/m")
    ax_k.set_xlabel("time, s")
    ax_k.set_ylabel("|kappa|, 1/m")
    ax_k.legend(loc="upper left", fontsize=8)

    ax_d = ax_k.twinx()
    ax_d.plot(tt, dd, lw=0.8, color="tab:blue", alpha=0.5)
    ax_d.set_ylabel("match distance, m", color="tab:blue")

    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"\nplot: {out}")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument(
        "--map",
        type=Path,
        default=None,
        help=f"ADASMAP1 file (default: {DEFAULT_MAP} in the repo)",
    )
    ap.add_argument(
        "--plot", type=Path, default=None, help="write a PNG (single bag only)"
    )
    ap.add_argument(
        "--horizon-m", type=float, default=200.0, help="how far ahead the summary looks"
    )
    ap.add_argument(
        "--route-m", type=float, default=2000.0, help="route length built in replay mode"
    )
    ap.add_argument(
        "--window-m", type=float, default=25.0, help="curvature window in replay mode"
    )
    ap.add_argument(
        "--min-speed",
        type=float,
        default=3.0,
        help="skip samples slower than this (replay mode)",
    )
    ap.add_argument(
        "--stride", type=int, default=1, help="use every Nth fix in replay mode"
    )
    ap.add_argument(
        "--compare-vision",
        action="store_true",
        help="correlate against the model's curvature",
    )
    ap.add_argument(
        "--vs-driven",
        action="store_true",
        help="check the map against the road the car actually drove (needs localization/pose)",
    )
    ap.add_argument(
        "--force-replay",
        action="store_true",
        help="ignore logged map/local and recompute",
    )
    args = ap.parse_args()

    map_path = args.map
    if map_path is None:
        repo = Path(__file__).resolve().parents[2]
        map_path = repo / DEFAULT_MAP
    road_map = load_map(map_path)

    cfg = mm.RouteConfig()
    cfg.horizon_m = args.route_m
    cfg.window_m = args.window_m

    rows = []
    for bag in args.bags:
        if not bag.is_dir():
            continue
        topics = list_topics(bag)
        if MAP_TOPIC in topics and not args.force_replay:
            samples, mode = samples_from_log(bag), "logged"
        else:
            samples, mode = (
                samples_from_replay(
                    bag, road_map, cfg, args.min_speed, max(1, args.stride)
                ),
                "replay",
            )
        if not samples:
            print(f"\n{bag.name}: no GPS and no map/local — nothing to do")
            continue

        vision = vision_curvature(bag) if args.compare_vision else None
        driven = (
            compare_against_driven(bag, samples, args.horizon_m)
            if args.vs_driven
            else None
        )
        row = summarise(bag, samples, mode, args.horizon_m, vision, driven)
        row["horizon"] = args.horizon_m
        print_summary(row)
        rows.append(row)

        if args.plot is not None and len(args.bags) == 1:
            plot(bag, samples, road_map, args.plot, args.horizon_m)

    if len(rows) > 1:
        ok = [r for r in rows if r.get("matched")]
        if ok:
            print(
                f"\n{len(rows)} bags: matched "
                f"{100.0 * sum(r['matched'] for r in ok) / max(sum(r['n'] for r in ok), 1):.0f}% overall, "
                f"match distance median {np.median([r['match_med'] for r in ok]):.1f} m, "
                f"node spacing median {np.median([r['spacing_med'] for r in ok]):.0f} m"
            )


if __name__ == "__main__":
    main()
