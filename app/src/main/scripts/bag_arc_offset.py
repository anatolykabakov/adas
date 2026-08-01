#!/usr/bin/env python3
"""Lane offset in a bag run: how much is the car's own, how much from reference — and arc plots.

Answers the question "why is the car not in the center of the lane on a curve". Offset decomposes
into two parts that add exactly:

    offset from lane center = (−CTE) + (reference offset from lane center)
                                 ^tracking error    ^reference offset

The first part is how far the car deviated from the line it follows. The second is how far that
line deviated from the lane center; it in turn sums from the supercombo plan, lane center estimate
from two lane lines, and the constant shift `path_camera_offset_m`.

Key metric subtlety: offset is taken **at the car itself**. Lines are approximated with a quadratic
fit over x ∈ [0, FIT_X_MAX] and the value is taken at x = 0. You cannot average the lane midpoint
"ahead": on a curve it is itself shifted toward the turn by ½κd² (up to 1.9 m at 25 m with
κ=0.006), and the metric shows the wrong sign.

The annotation frame is right-positive (`topic_convert.cpp`: "Device Y right-positive"), the camera
is shifted left by `cam_y_left` (`lane_keep_service.cpp`: `p.y() -= cam_y_left`), therefore

    left_offset(0) = y_right_pos(0) − cam_y_left

Parsing correctness is self-checked: computed −CTE is compared with recorded `mpc_cte_m`. If the
discrepancy exceeds a few centimeters, reference replay does not match the run (e.g. wrong
`blend`/shift or a different version of `laneLinesToPath`), and the numbers cannot be trusted.

  python3 bag_arc_offset.py adas_logs/<run>
  python3 bag_arc_offset.py adas_logs/<run> --plots docs/mpc_img --prefix 0802
  python3 bag_arc_offset.py adas_logs/<run> --cache /tmp/run.npz   # second run is instant
  python3 bag_arc_offset.py adas_logs/<run> --weight-by-std        # how σ weight behaves

For runs recorded with a build from 2026-08-03 and later, config must be set explicitly — otherwise
the wrong reference is restored, and the CTE self-check will show it:

  python3 bag_arc_offset.py adas_logs/<run> --blend 0.6 --weight-by-std \
      --std-good 0.3 --std-bad 1.5 --center-force <value from config.json>

Report for run 2026_08_02_22_02_38 — `docs/RUN_0802_ARC_OFFSET.md`.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import _path  # noqa: F401

FIT_X_MAX = 30.0  # longitudinal fit window for lane lines
CAR_HALF_W = 0.90  # Golf 7 half-width without mirrors (1.80 m)
STRAIGHT_K = 0.002  # |κ| below — straight
ARC_K = 0.004  # |κ| above — arc
SAT_CNM = 295  # HCA torque considered saturated at limit (±300)

COLS = [
    "t",
    "mid0",
    "l0",
    "r0",
    "k_mid",
    "plan0",
    "prob_l",
    "prob_r",
    "std_l",
    "std_r",
    "cam_y_left",
    "cte",
    "k_used",
    "swa_des",
    "swa_act",
    "torque",
    "enabled",
    "v",
    "yaw",
    "swa",
    "pressed",
    # Recorded values from the run itself (builds from 2026-08-03). Zero/NaN — old run.
    "log_anchored",
    "log_width",
    "log_offset",
    "log_center",
    "log_blend",
    "log_shift",
    "log_gain",
]


# ─────────────────────────────────────────────────────────── bag extraction
def fit_at_zero(x: np.ndarray, y: np.ndarray) -> Tuple[float, float]:
    """Quadratic fit over x∈[0,FIT_X_MAX]; returns (y(0), κ≈2a)."""
    m = (x >= 0.0) & (x <= FIT_X_MAX) & np.isfinite(y)
    if m.sum() < 5:
        return float("nan"), float("nan")
    c = np.polyfit(x[m], y[m], 2)
    return float(c[2]), float(2.0 * c[0])


def extract(bag: Path) -> np.ndarray:
    from vis.bag_io import iter_aligned, load_topic_messages

    dbg = load_topic_messages(bag, "control/lane_keep_debug")
    lanes = load_topic_messages(bag, "vision/lanes")
    state = load_topic_messages(bag, "vehicle/state")
    if not dbg or not lanes or not state:
        raise SystemExit(
            f"bag missing required topics: debug {len(dbg)}, lanes {len(lanes)}, "
            f"state {len(state)}"
        )
    rows: List[tuple] = []
    for r in iter_aligned(lanes, {"dbg": dbg, "st": state}, max_dt_ms=80):
        ll, db, st = r["primary"], r["dbg"], r["st"]
        if db is None or st is None:
            continue
        x = np.asarray(ll.x, dtype=np.float64)
        if x.size < 6 or len(ll.lanes) < 3:
            continue
        yl = np.asarray(ll.lanes[1].y, dtype=np.float64)
        yr = np.asarray(ll.lanes[2].y, dtype=np.float64)
        if yl.size != x.size or yr.size != x.size:
            continue
        mid0, k_mid = fit_at_zero(x, 0.5 * (yl + yr))
        l0, _ = fit_at_zero(x, yl)
        r0, _ = fit_at_zero(x, yr)
        plan_x = np.asarray(ll.plan_x, dtype=np.float64)
        plan_y = np.asarray(ll.plan_y, dtype=np.float64)
        plan0 = (
            fit_at_zero(plan_x, plan_y)[0] if plan_x.size == plan_y.size >= 6 else np.nan
        )
        # y_std appeared in runs from 2026-08-02; on older runs — empty, read as "no data"
        sl = np.median(ll.lanes[1].y_std) if len(ll.lanes[1].y_std) else np.nan
        sr = np.median(ll.lanes[2].y_std) if len(ll.lanes[2].y_std) else np.nan
        rows.append(
            (
                float(r["t"]) / 1000.0,
                mid0,
                l0,
                r0,
                k_mid,
                plan0,
                float(ll.lanes[1].prob),
                float(ll.lanes[2].prob),
                float(sl),
                float(sr),
                float(db.cam_y_left_m),
                float(db.mpc_cte_m),
                float(db.mpc_kappa_used),
                float(db.desired_swa_deg),
                float(db.actual_swa_deg),
                float(db.torque_cnm),
                1.0 if db.steer_output_enabled else 0.0,
                float(st.v_ego),
                float(st.yaw_rate),
                float(st.steering_angle_deg),
                1.0 if getattr(st, "steering_pressed", False) else 0.0,
                1.0 if getattr(db, "lane_anchored", False) else 0.0,
                float(getattr(db, "lane_width_m", 0.0) or 0.0),
                float(getattr(db, "lane_offset_m", 0.0) or 0.0),
                float(getattr(db, "center_force_m", 0.0) or 0.0),
                float(getattr(db, "p_lane_blend_scale", 0.0) or 0.0),
                float(getattr(db, "p_camera_offset_m", 0.0) or 0.0),
                float(getattr(db, "p_center_force_gain", 0.0) or 0.0),
            )
        )
    if not rows:
        raise SystemExit("no usable frames — no lane polylines?")
    return np.asarray(rows, dtype=np.float64)


# ─────────────────────────────────────────────────────────── analysis
def center_force_shift(offset_m, width_m, kappa_rp, gain, max_m=0.8, turn_scale=0.7):
    """Mirror of `center_force` from C++ `laneLinesToPath` — keep in sync with it.

    `offset_m` — lane center offset relative to the car (right+), `kappa_rp` — reference curvature
    in the same right-positive frame. Returns — shift of the entire reference to the right, m.
    """
    if gain <= 0.0:
        return np.zeros_like(offset_m)
    w = np.maximum(width_m, 1e-3)
    cf = gain * (3.4 / w) * offset_m
    cf *= np.clip((w - 2.6) / 0.2, 0.0, 1.0)  # don't jerk in a narrow lane
    cf *= np.clip((6.0 - w) / 2.0, 0.0, 1.0)  # and in a very wide one too
    damp = (np.abs(kappa_rp) > 5e-4) & (cf * kappa_rp > 0.0)
    cf = np.where(damp, cf * turn_scale, cf)
    return np.clip(cf, -max_m, max_m)


class Analysis:
    def __init__(
        self,
        data: np.ndarray,
        blend: float,
        shift: float,
        std_good: float,
        std_bad: float,
        weight_by_std: bool,
        center_force: float = 0.0,
        width_min: float = 2.6,
        width_max: float = 4.2,
    ):
        c = {k: data[:, i] for i, k in enumerate(COLS)}
        self.c = c
        self.t = c["t"] - c["t"][0]
        self.blend, self.shift = blend, shift

        def soft(p):
            return np.where(p >= 0.3, p, 0.0)

        def std_conf(s):
            """σ≤good — line counted fully, σ≥bad — ignored. NaN = no data → 1."""
            out = np.clip((std_bad - s) / max(std_bad - std_good, 1e-9), 0.0, 1.0)
            return np.where(np.isfinite(s), out, 1.0)

        pl, pr = soft(c["prob_l"]), soft(c["prob_r"])
        if weight_by_std:
            pl, pr = pl * std_conf(c["std_l"]), pr * std_conf(c["std_r"])
        self.width = np.abs(c["l0"] - c["r0"])
        w = np.clip(self.width, width_min, width_max)
        # exactly as laneLinesToPath: path = left+w/2 and right−w/2, weighted by confidence
        self.lane_y0 = (pl * (c["l0"] + 0.5 * w) + pr * (c["r0"] - 0.5 * w)) / (
            pl + pr + 1e-6
        )
        anchored = (
            (pl > 0) & (pr > 0) & (self.width > width_min) & (self.width < width_max)
        )
        self.d = np.where(anchored, (pl + pr - pl * pr) * np.clip(blend, 0.0, 1.0), 0.0)
        # Centering term: constant shift of the entire reference, computed from the same offset at the car.
        self.center = np.where(
            anchored,
            center_force_shift(
                c["mid0"] - c["cam_y_left"], self.width, c["k_mid"], center_force
            ),
            0.0,
        )
        self.path0 = (
            self.d * self.lane_y0 + (1.0 - self.d) * c["plan0"] + shift + self.center
        )

        self.off_lane = c["mid0"] - c["cam_y_left"]  # car left of lane center
        self.off_path = self.path0 - c["cam_y_left"]  # car left of reference == −CTE
        self.ref_bias = c["mid0"] - self.path0  # reference left of lane center

        # If the run was recorded by a build that logs these values itself, use the logged ones:
        # then the decomposition reflects what actually ran, not a second reimplementation of
        # laneLinesToPath. Recomputation remains as a cross-check.
        self.logged = bool(np.any(c["log_anchored"] > 0.5)) or bool(
            np.any(c["log_gain"] > 0)
        )
        if self.logged:
            self.off_recomputed = self.off_lane
            self.center_recomputed = self.center
            self.off_lane = c["log_offset"]
            self.center = c["log_center"]
            # Reference = car offset minus how far it lagged the reference (= −CTE).
            self.ref_bias = self.off_lane + c["cte"]
            self.off_path = -c["cte"]
            self.width = np.where(c["log_width"] > 0, c["log_width"], self.width)
            anchored = c["log_anchored"] > 0.5
        self.plan_bias = c["mid0"] - c["plan0"]
        self.lane_bias = c["mid0"] - self.lane_y0
        self.k_road = -c["k_mid"]  # lane curvature, left+
        self.sat = np.abs(c["torque"]) >= SAT_CNM

        self.good = (
            (c["enabled"] > 0.5)
            & (c["pressed"] < 0.5)
            & (c["v"] > 5.0)
            & (c["prob_l"] > 0.3)
            & (c["prob_r"] > 0.3)
            & (self.width > 2.6)
            & (self.width < 4.2)
            & np.isfinite(self.off_lane)
            & np.isfinite(self.path0)
        )

    def bins(self):
        k = self.k_road
        return [
            ("straight |κ|<0.002", np.abs(k) < STRAIGHT_K),
            ("left weak 0.002–0.004", (k >= STRAIGHT_K) & (k < ARC_K)),
            ("left arc κ>0.004", k >= ARC_K),
            ("right weak", (k <= -STRAIGHT_K) & (k > -ARC_K)),
            ("right arc κ<−0.004", k <= -ARC_K),
        ]

    def episodes(self, mask, min_s=2.5, merge_gap_s=1.5):
        """Continuous arcs; short gaps are merged, otherwise one arc splits into pieces."""
        raw, start = [], None
        for i, val in enumerate(mask):
            if val and start is None:
                start = i
            elif not val and start is not None:
                raw.append((start, i))
                start = None
        if start is not None:
            raw.append((start, len(mask)))
        merged: List[Tuple[int, int]] = []
        for a, b in raw:
            if merged and self.t[a] - self.t[merged[-1][1] - 1] <= merge_gap_s:
                merged[-1] = (merged[-1][0], b)
            else:
                merged.append((a, b))
        return [(a, b) for a, b in merged if self.t[b - 1] - self.t[a] >= min_s]


def report(A: Analysis) -> None:
    c = A.c
    print(
        f"frames {len(A.t)}, usable {A.good.sum()} ({A.good.mean() * 100:.0f} %), "
        f"duration {A.t[-1] / 60:.1f} min"
    )
    print(
        f"cam_y_left_m in bag: {np.unique(np.round(c['cam_y_left'], 4))}   "
        f"reference replay: blend {A.blend}, shift {A.shift}"
    )

    gg = A.good & np.isfinite(c["cte"])
    if A.logged:
        print(
            f"\nbag logs reference itself: blend {np.median(c['log_blend'][gg]):.2f}, "
            f"shift {np.median(c['log_shift'][gg]):.2f}, "
            f"centering {np.median(c['log_gain'][gg]):.2f} — numbers below from bag, not recomputed"
        )
        e = A.off_recomputed[gg] - A.off_lane[gg]
        print(
            f"cross-check — my offset measurement vs logged: "
            f"median {np.median(e):+.3f} m, p90 |·| {np.percentile(np.abs(e), 90):.3f} m"
        )
        e2 = A.center_recomputed[gg] - A.center[gg]
        print(
            f"cross-check — my centering term vs logged: "
            f"median {np.median(e2):+.3f} m, p90 |·| {np.percentile(np.abs(e2), 90):.3f} m"
        )
    else:
        err = A.off_path[gg] + c["cte"][gg]
        ok = abs(float(np.median(err))) < 0.06
        print(
            f"\nself-check — computed −CTE vs recorded mpc_cte_m: "
            f"median {np.median(err):+.3f} m, p90 |·| {np.percentile(np.abs(err), 90):.3f} m, "
            f"corr {np.corrcoef(A.off_path[gg], -c['cte'][gg])[0, 1]:+.3f}  "
            f"{'— reference replayed' if ok else '— MISMATCH, do not trust numbers below'}"
        )
    m = A.good & (np.abs(A.k_road) > 0.003)
    print(
        f"self-check — arc sign: corr(κ lane lines, chassis yaw) = "
        f"{np.corrcoef(A.k_road[m], c['yaw'][m])[0, 1]:+.3f} (expect noticeably > 0)"
    )

    print(
        f"\n{'segment':<26} {'n':>6} {'total':>7} {'tracking':>9} {'ref off':>8} {'sat':>6}"
    )
    for name, sel in A.bins():
        s = A.good & sel
        if s.sum() < 30:
            continue
        print(
            f"{name:<26} {s.sum():>6} {np.median(A.off_lane[s]):>+7.2f} "
            f"{np.median(A.off_path[s]):>+9.2f} {np.median(A.ref_bias[s]):>+8.2f} "
            f"{np.mean(A.sat[s]) * 100:>5.0f}%"
        )

    print(f"\nreference offset by component:")
    print(
        f"{'segment':<26} {'n':>6} {'ref off':>8} {'plan·(1−d)':>12} {'lane·d':>10} "
        f"{'shift':>7} {'d':>5} {'width':>7} {'σ worst':>9}"
    )
    s_worst = np.fmax(c["std_l"], c["std_r"])
    for name, sel in A.bins():
        s = A.good & sel
        if s.sum() < 30:
            continue
        d = float(np.median(A.d[s]))
        print(
            f"{name:<26} {s.sum():>6} {np.median(A.ref_bias[s]):>+8.2f} "
            f"{np.median(A.plan_bias[s]) * (1 - d):>+12.2f} "
            f"{np.median(A.lane_bias[s]) * d:>+10.2f} {-A.shift:>+7.2f} {d:>5.2f} "
            f"{np.median(A.width[s]):>7.2f} {np.nanmedian(s_worst[s]):>9.2f}"
        )
    print(
        "  model plan relative to lane center, unweighted: "
        + ", ".join(
            f"{n} {np.median(A.plan_bias[A.good & sel]):+.2f}"
            for n, sel in A.bins()
            if (A.good & sel).sum() >= 30
        )
    )

    print(f"\narc episodes (≥2.5 s):")
    print(
        f"{'side':<9} {'t, s':>7} {'dur.':>6} {'v':>5} {'R, m':>6} {'total':>7} "
        f"{'tracking':>9} {'ref off':>8} {'sat':>6}"
    )
    for side, sel, key in (
        ("left", A.k_road >= 0.0035, -1.0),
        ("right", A.k_road <= -0.0035, 1.0),
    ):
        eps = sorted(
            A.episodes(A.good & sel),
            key=lambda e: key * np.median(A.off_lane[e[0] : e[1]]),
        )
        for a, b in eps[:4]:
            print(
                f"{side:<9} {A.t[a]:>7.0f} {A.t[b - 1] - A.t[a]:>5.1f}s "
                f"{np.median(c['v'][a:b]):>5.1f} {1.0 / abs(np.median(A.k_road[a:b])):>6.0f} "
                f"{np.median(A.off_lane[a:b]):>+7.2f} {np.median(A.off_path[a:b]):>+9.2f} "
                f"{np.median(A.ref_bias[a:b]):>+8.2f} {np.mean(A.sat[a:b]) * 100:>5.0f}%"
            )

    arc = A.good & (np.abs(A.k_road) >= ARC_K)
    inside = arc & (np.sign(A.off_lane) == np.sign(A.k_road)) & (np.abs(A.off_lane) > 0.3)
    if inside.sum():
        into = np.sign(c["torque"][inside & A.sat]) == np.sign(A.k_road[inside & A.sat])
        print(
            f"\non arcs where the car is already >0.3 m inside ({inside.sum()} frames): torque saturated "
            f"{np.mean(A.sat[inside]) * 100:.0f}% of frames, of which directed into turn "
            f"{np.mean(into) * 100 if into.size else float('nan'):.0f}%"
        )
    print(
        f"desired → actual SWA: "
        + ", ".join(
            f"{n} {np.median(c['swa_des'][A.good & sel]):+.1f}° → "
            f"{np.median(c['swa_act'][A.good & sel]):+.1f}°"
            for n, sel in A.bins()
            if (A.good & sel).sum() >= 30
        )
    )


# ─────────────────────────────────────────────────────────── plots
def plots(A: Analysis, outdir: Path, prefix: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    SURFACE, INK, INK2, INK3 = "#fcfcfb", "#0b0b0b", "#52514e", "#8b8a83"
    C_TOTAL, C_TRACK, C_REF, C_SAT = "#2a78d6", "#eb6834", "#1baf7a", "#e34948"
    plt.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "text.color": INK,
            "axes.labelcolor": INK2,
            "xtick.color": INK2,
            "ytick.color": INK2,
            "axes.edgecolor": "#d9d8d2",
            "grid.color": "#e8e7e1",
            "grid.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.size": 9,
            "axes.titlesize": 9.5,
            "figure.dpi": 130,
        }
    )
    outdir.mkdir(parents=True, exist_ok=True)
    t, off, rb = A.t, A.off_lane, A.ref_bias

    left = sorted(
        A.episodes(A.good & (A.k_road >= 0.0035)),
        key=lambda e: -np.median(off[e[0] : e[1]]),
    )[:2]
    right = sorted(
        A.episodes(A.good & (A.k_road <= -0.0035)),
        key=lambda e: np.median(off[e[0] : e[1]]),
    )[:2]

    fig, axes = plt.subplots(2, 2, figsize=(11.0, 7.0), sharey=True, squeeze=False)
    for row, (eps, label) in enumerate(((left, "left arc"), (right, "right arc"))):
        for col in range(2):
            ax = axes[row, col]
            if col >= len(eps):
                ax.axis("off")
                continue
            a, b = eps[col]
            tt, y, r = t[a:b] - t[a], off[a:b], rb[a:b]
            half = float(np.nanmedian(A.width[a:b])) / 2.0 - CAR_HALF_W
            ax.axhspan(-half, half, color="#eeede7", zorder=0)
            ax.axhline(0.0, color=INK3, lw=1.0, zorder=1)
            ax.fill_between(tt, r, y, color=C_TRACK, alpha=0.16, lw=0, zorder=2)
            ax.plot(tt, r, lw=2.0, color=C_REF, zorder=3)
            ax.plot(tt, y, lw=2.0, color=C_TOTAL, zorder=4)
            if A.sat[a:b].any():
                ax.plot(
                    tt[A.sat[a:b]],
                    y[A.sat[a:b]],
                    "o",
                    ms=3.6,
                    color=C_SAT,
                    mec=SURFACE,
                    mew=0.5,
                    zorder=5,
                )
            ax.grid(axis="y", alpha=0.9)
            ax.set_title(
                f"{label} · t={t[a]:.0f} s · {np.nanmedian(A.c['v'][a:b]):.0f} m/s · "
                f"R≈{1.0 / abs(np.nanmedian(A.k_road[a:b])):.0f} m\n"
                f"total {np.nanmedian(y):+.2f} m = tracking {np.nanmedian(y - r):+.2f}"
                f" + ref off {np.nanmedian(r):+.2f}",
                color=INK,
            )
            ax.annotate(
                f"{np.nanmedian(y):+.2f}",
                (tt[-1], y[-1]),
                color=C_TOTAL,
                fontsize=9,
                fontweight="bold",
                va="center",
                textcoords="offset points",
                xytext=(4, 0),
                annotation_clip=False,
            )
            if row == 1:
                ax.set_xlabel("time since arc entry, s")
            if col == 0:
                ax.set_ylabel("left of lane center, m")
    axes[0, 0].legend(
        handles=[
            Line2D([], [], color=C_TOTAL, lw=2, label="car: from lane center"),
            Line2D(
                [],
                [],
                color=C_REF,
                lw=2,
                label="reference: from lane center (reference offset)",
            ),
            Patch(facecolor=C_TRACK, alpha=0.16, label="gap = tracking error (−CTE)"),
            Line2D(
                [],
                [],
                color=C_SAT,
                marker="o",
                ms=3.6,
                ls="none",
                label=f"steer torque saturated ±{SAT_CNM + 5} cNm",
            ),
            Patch(facecolor="#eeede7", label="wheels inside lane markings"),
        ],
        loc="lower left",
        frameon=False,
        fontsize=7.6,
    )
    fig.suptitle(
        "On arcs the car cuts inside: left on left arcs, right on right arcs",
        fontsize=11.5,
        y=0.985,
        color=INK,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    p1 = outdir / f"{prefix}_offset_arcs.png"
    fig.savefig(p1, bbox_inches="tight")
    print(f"\n→ {p1}")

    edges = np.array([-0.010, -0.006, -0.004, -0.002, 0.002, 0.004, 0.006, 0.010])
    ctr, tot, trk, ref, p10, p90, ns = [], [], [], [], [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (
            A.good
            & (A.k_road >= lo)
            & (A.k_road < hi)
            & np.isfinite(off)
            & np.isfinite(rb)
        )
        if m.sum() < 40:
            continue
        ctr.append(0.5 * (lo + hi))
        tot.append(np.median(off[m]))
        trk.append(np.median(off[m] - rb[m]))
        ref.append(np.median(rb[m]))
        p10.append(np.percentile(off[m], 10))
        p90.append(np.percentile(off[m], 90))
        ns.append(m.sum())
    fig, ax = plt.subplots(figsize=(10.0, 5.8))
    ax.axhline(0.0, color=INK3, lw=1.0)
    ax.fill_between(ctr, p10, p90, color=C_TOTAL, alpha=0.10, lw=0, zorder=1)
    for y, col, lbl, short in (
        (tot, C_TOTAL, "total: car from lane center", "total"),
        (trk, C_TRACK, "tracking error (−CTE)", "tracking"),
        (ref, C_REF, "reference offset (ref vs lane)", "ref off"),
    ):
        ax.plot(
            ctr,
            y,
            lw=2.0,
            color=col,
            marker="o",
            ms=5,
            mec=SURFACE,
            mew=0.8,
            zorder=3,
            label=lbl,
        )
        ax.annotate(
            short,
            (ctr[-1], y[-1]),
            color=col,
            fontsize=9,
            fontweight="bold",
            va="center",
            textcoords="offset points",
            xytext=(6, 0),
            annotation_clip=False,
        )
    for x, y, n in zip(ctr, p90, ns):
        ax.annotate(
            f"n={n}",
            (x, y),
            color=INK3,
            fontsize=7,
            ha="center",
            textcoords="offset points",
            xytext=(0, 5),
        )
    ax.set_xlabel("lane curvature from markings, 1/m     (κ<0 — right arc, κ>0 — left)")
    ax.set_ylabel("left of lane center, m")
    ax.set_title(
        "Offset = tracking error + reference offset\n"
        "on straights both parts cancel, on arcs they add toward the inside of the turn "
        "(pale band — p10–p90 of total offset)",
        color=INK,
    )
    ax.grid(axis="y", alpha=0.9)
    ax.legend(frameon=False, loc="upper left", fontsize=8.5)
    ax.set_xlim(edges[0] - 0.0005, edges[-1] - 0.0008)
    fig.tight_layout()
    p2 = outdir / f"{prefix}_offset_decomp.png"
    fig.savefig(p2, bbox_inches="tight")
    print(f"→ {p2}")


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("bag", type=Path)
    p.add_argument(
        "--cache",
        type=Path,
        default=None,
        help="npz with extraction: read if present, else written (bag parsing — minutes)",
    )
    p.add_argument(
        "--blend", type=float, default=0.3, help="path_lane_blend_scale in bag"
    )
    p.add_argument(
        "--shift", type=float, default=0.08, help="path_camera_offset_m in bag"
    )
    p.add_argument("--std-good", type=float, default=0.2, help="lane_std_good_m in bag")
    p.add_argument("--std-bad", type=float, default=0.8, help="lane_std_bad_m in bag")
    p.add_argument(
        "--center-force",
        type=float,
        default=0.0,
        help="center_force_gain in bag (0 = term disabled)",
    )
    p.add_argument("--width-min", type=float, default=2.6, help="lane_width_min_m in bag")
    p.add_argument(
        "--width-max",
        type=float,
        default=4.2,
        help="lane_width_max_m in bag (builds before 2026-08-03 — 4.2, after — 4.6)",
    )
    p.add_argument(
        "--weight-by-std",
        action="store_true",
        help="weight blending by model σ (not in APK before 2026-08-02)",
    )
    p.add_argument("--plots", type=Path, default=None, help="where to save plots")
    p.add_argument("--prefix", default="offset", help="plot filename prefix")
    args = p.parse_args()

    if args.cache and args.cache.exists():
        data = np.load(args.cache)["data"]
        print(f"loaded from cache {args.cache}")
    else:
        data = extract(args.bag)
        if args.cache:
            args.cache.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(args.cache, data=data)
            print(f"extraction saved to {args.cache}")

    print(f"\nbag {args.bag.name}")
    A = Analysis(
        data,
        args.blend,
        args.shift,
        args.std_good,
        args.std_bad,
        args.weight_by_std,
        args.center_force,
        args.width_min,
        args.width_max,
    )
    report(A)
    if args.plots:
        plots(A, args.plots, args.prefix)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
