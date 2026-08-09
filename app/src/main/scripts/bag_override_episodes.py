#!/usr/bin/env python3
"""Driver overrides while the assist was engaged: what the controller wanted, what the rack did.

The question this answers is "why did the driver have to help", and it can only be answered on frames where
the assist was actually engaged. That is not the same as frames where the lane-keep service thought it was
steering: lateral output is gated by panda's `controls_allowed`, which on VW MQB follows the *stock cruise
engagement*, and on run 2026_08_06_18_27_12 half the "steering" frames had no torque on the bus at all. So
every episode here is filtered on `HCA_01.HCA_Active` decoded from `can/rx`, not on `steer_output_enabled`.

An override is defined as the EPS reporting driver torque above `STEER_DRIVER_ALLOWANCE` (80, the same
threshold the torque limiter uses to start clawing back assist) for at least `--min-ms`, while the assist is
engaged. Episodes are ranked by peak |CTE| so the worst come first.

Each plot has four panels sharing a time axis, which is the whole point — the four signals only mean
something together:

1. **CTE** (`mpc_cte_m`), the lateral error the controller is trying to remove;
2. **torque**: the assist torque actually on the bus (from HCA_01) against the driver's own torque from
   `LH_EPS_03.EPS_Lenkmoment`. Both in cNm, so the tug-of-war is visible directly;
3. **steering angle**: what the controller asked for against what the column did;
4. **angle error**, signed, with the assist-engaged span shaded.

  python3 bag_override_episodes.py adas_logs/<run> --out /tmp/plots
  python3 bag_override_episodes.py adas_logs/<run> --top 8 --pad 4.0
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis.bag_io import load_topic_messages

HCA_01 = 0x126
DRIVER_ALLOWANCE = 80.0  # volkswagen::CarControllerParams::STEER_DRIVER_ALLOWANCE


def bit_field(data: bytes, start: int, length: int) -> int:
    """Little-endian bit extraction, matching `set_bits` in volkswagen/mqbcan.cpp."""
    v = 0
    for i in range(length):
        b = start + i
        if data[b // 8] >> (b % 8) & 1:
            v |= 1 << i
    return v


def load_hca(bag: Path):
    """Assist torque and the HCA_Active bit as they went out on the wire."""
    t, tq, on = [], [], []
    for rec in load_topic_messages(bag, "can/rx"):
        ts, msg = rec[0], rec[1]
        for fr in msg.frames:
            # src 128 is our own TX echoed back by the panda; the camera's copy would double-count.
            if fr.address != HCA_01 or fr.src != 128 or len(fr.data) < 8:
                continue
            d = fr.data
            t.append(ts)
            tq.append((-1 if bit_field(d, 31, 1) else 1) * bit_field(d, 16, 14))
            on.append(bool(bit_field(d, 34, 1)))
    o = np.argsort(np.asarray(t, dtype=np.float64))
    return (
        np.asarray(t, dtype=np.float64)[o],
        np.asarray(tq, dtype=np.float64)[o],
        np.asarray(on, dtype=bool)[o],
    )


def nearest(t_ref, t_src, v_src, max_dt_ms=60.0):
    if len(t_src) == 0:
        return np.zeros_like(t_ref), np.zeros_like(t_ref, dtype=bool)
    i = np.clip(np.searchsorted(t_src, t_ref), 0, len(t_src) - 1)
    j = np.clip(i - 1, 0, len(t_src) - 1)
    take = np.abs(t_src[j] - t_ref) < np.abs(t_src[i] - t_ref)
    i = np.where(take, j, i)
    return v_src[i], np.abs(t_src[i] - t_ref) <= max_dt_ms


def episodes(mask, t, min_ms):
    """Contiguous runs of `mask`, as (start_index, end_index) pairs, at least `min_ms` long."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.r_[idx[0], idx[breaks + 1]]
    ends = np.r_[idx[breaks], idx[-1]]
    return [(s, e) for s, e in zip(starts, ends) if t[e] - t[s] >= min_ms]


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, default=Path("/tmp/override_plots"))
    ap.add_argument(
        "--top", type=int, default=6, help="how many episodes to plot, worst |CTE| first"
    )
    ap.add_argument(
        "--min-ms", type=float, default=300.0, help="shortest override worth plotting"
    )
    ap.add_argument(
        "--pad", type=float, default=3.0, help="seconds of context on each side"
    )
    args = ap.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dbg = load_topic_messages(args.bag, "control/lane_keep_debug")
    if not dbg:
        print("нет control/lane_keep_debug")
        return 1
    t = np.asarray([r[0] for r in dbg], dtype=np.float64)
    f = lambda n: np.asarray(
        [float(getattr(r[1], n, 0.0)) for r in dbg], dtype=np.float64
    )
    des, act, err = f("desired_swa_deg"), f("actual_swa_deg"), f("angle_error_deg")
    cte, req_tq, v = f("mpc_cte_m"), f("torque_cnm"), f("speed_mps")

    st = load_topic_messages(args.bag, "vehicle/state")
    t_st = np.asarray([r[0] for r in st], dtype=np.float64)
    drv = np.asarray(
        [float(getattr(r[1], "steering_torque", 0.0)) for r in st], dtype=np.float64
    )
    o = np.argsort(t_st)
    t_st, drv = t_st[o], drv[o]

    t_h, applied, hca_on = load_hca(args.bag)
    ap_at, ap_fresh = nearest(t, t_h, applied, 40.0)
    on_at, on_fresh = nearest(t, t_h, hca_on.astype(float), 40.0)
    assist = (on_at > 0.5) & on_fresh
    drv_at, _ = nearest(t, t_st, drv, 60.0)

    over = assist & (np.abs(drv_at) > DRIVER_ALLOWANCE) & (v > 5)
    eps = episodes(over, t, args.min_ms)
    print(f"кадров с включённым ассистом: {assist.sum()} из {len(t)}")
    print(
        f"перехватов водителем при включённом ассисте (>{DRIVER_ALLOWANCE:.0f} и >{args.min_ms:.0f} мс): {len(eps)}"
    )
    if not eps:
        return 0

    total_ms = sum(t[e] - t[s] for s, e in eps)
    print(
        f"суммарно {total_ms / 1000:.1f} с, это {100 * total_ms / max(t[assist][-1] - t[assist][0], 1):.1f}% "
        f"времени с включённым ассистом"
    )

    def peak_abs_cte(se):
        seg = np.abs(cte[se[0] : se[1] + 1])
        seg = seg[np.isfinite(seg)]
        return float(seg.max()) if seg.size else 0.0

    ranked = sorted(eps, key=peak_abs_cte, reverse=True)

    args.out.mkdir(parents=True, exist_ok=True)
    print(f"\nхудшие {min(args.top, len(ranked))} по пиковому |CTE|:")
    for k, (s, e) in enumerate(ranked[: args.top]):
        lo, hi = t[s] - args.pad * 1000, t[e] + args.pad * 1000
        w = (t >= lo) & (t <= hi)
        tt = (t[w] - t[s]) / 1000.0
        peak_cte = float(np.nanmax(np.abs(cte[s : e + 1])))
        peak_err = float(np.nanmax(np.abs(err[s : e + 1])))
        print(
            f"  #{k + 1}: t={t[s] / 1000:.1f} с, длительность {(t[e] - t[s]) / 1000:.1f} с, "
            f"v={np.median(v[s:e + 1]):.1f} м/с, пик |CTE| {peak_cte:.2f} м, пик |ошибка угла| {peak_err:.1f}°, "
            f"момент водителя до {np.max(np.abs(drv_at[s:e + 1])):.0f}, ассист до {np.max(np.abs(ap_at[s:e + 1])):.0f} cNm"
        )

        fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
        span = lambda ax: ax.axvspan(
            0.0,
            (t[e] - t[s]) / 1000.0,
            color="0.85",
            zorder=0,
            label="водитель держит руль",
        )

        ax = axes[0]
        span(ax)
        ax.plot(tt, cte[w], color="tab:red", lw=1.6, label="CTE, м")
        ax.axhline(0, color="0.6", lw=0.8)
        ax.set_ylabel("CTE, м")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(
            f"{args.bag.name}: перехват #{k + 1}, ассист включён, v≈{np.median(v[s:e + 1]):.0f} м/с"
        )

        ax = axes[1]
        span(ax)
        ax.plot(tt, ap_at[w], color="tab:blue", lw=1.6, label="ассист на шине, cNm")
        ax.plot(
            tt,
            req_tq[w],
            color="tab:blue",
            lw=1.0,
            ls=":",
            label="запрошено контроллером, cNm",
        )
        ax.plot(tt, drv_at[w], color="tab:orange", lw=1.6, label="момент водителя, cNm")
        for lim in (300, -300):
            ax.axhline(lim, color="0.7", lw=0.8, ls="--")
        ax.axhspan(-57, 57, color="tab:green", alpha=0.12, label="полоса трения ±57 cNm")
        ax.set_ylabel("момент, cNm")
        ax.legend(loc="upper right", fontsize=8, ncol=2)

        ax = axes[2]
        span(ax)
        ax.plot(tt, des[w], color="tab:green", lw=1.6, label="угол руля: контроллер")
        ax.plot(tt, act[w], color="k", lw=1.4, label="угол руля: фактический")
        ax.axhline(0, color="0.6", lw=0.8)
        ax.set_ylabel("угол руля, °")
        ax.legend(loc="upper right", fontsize=8)

        ax = axes[3]
        span(ax)
        ax.plot(
            tt, err[w], color="tab:purple", lw=1.6, label="ошибка угла (задан − факт)"
        )
        ax.axhline(0, color="0.6", lw=0.8)
        ax.set_ylabel("ошибка, °")
        ax.set_xlabel("время от начала перехвата, с")
        ax.legend(loc="upper right", fontsize=8)

        for a in axes:
            a.grid(alpha=0.25)
        fig.tight_layout()
        out = args.out / f"override_{k + 1:02d}.png"
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(f"       -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
