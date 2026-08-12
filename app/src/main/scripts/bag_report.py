#!/usr/bin/env python3
"""Сводный разбор заезда за один проход: темп зрения, шаг уставки, положение в полосе по дугам,
упор по моменту, задержка цепочки, уверенность разметки, поза против колёс, средний слой,
голосовые пометки водителя.

Выводы обычно берутся из сопоставления этих величин, а не из каждой по отдельности — поэтому они
печатаются рядом. Считается по кэшу из `vis/bag_cache.py`: первый прогон около 95 секунд, дальше
0.3 секунды.

    python3 bag_report.py adas_logs/2026_08_08_23_00_28
    python3 bag_report.py adas_logs/A adas_logs/B --only step   # сравнение заездов
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict

import numpy as np

import _path  # noqa: F401
from vis import bag_cache

# Ошибка, на которой угловой ПИД (kp = 0.6, предел ±1) уже в упоре — без интегратора и без трения.
PID_RAIL_DEG = 1.0 / 0.6
# Запасной потолок для багов, записанных до появления p_max_torque_cnm в LaneKeepDebug.
FALLBACK_MAX_TORQUE_CNM = 300.0
# Порог аналитика, а не контроллера: управление включается на 1.5 м/с, а смотреть ниже 23 км/ч
# просили не смотреть — там водитель ведёт машину сам.
MIN_SPEED_MPS = 23.0 / 3.6

ARCS = [
    ("прямая R>500", 500.0, np.inf),
    ("пологая 167-500", 167.0, 500.0),
    ("средняя 83-167", 83.0, 167.0),
    ("крутая R<83", 0.0, 83.0),
]


def _pct(x, q):
    return float(np.percentile(x, q)) if x.size else float("nan")


def _clean(d: Dict[str, np.ndarray]) -> np.ndarray:
    """Тики, по которым имеет смысл судить о поперечном управлении."""
    t = d["ctrl_t"]
    return (
        (d["ctrl_v"] >= MIN_SPEED_MPS)
        & d["ctrl_has_target"]
        & ~bag_cache.blinker_mask(d, t)
    )


def sec_rate(d):
    ct = d["lanes_capture_t"]
    if ct.size < 10:
        return
    src = ct if (ct > 0).all() else d["lanes_t"]
    dt = np.diff(src)
    dt = dt[(dt > 0) & (dt < 500)]
    print("\n== темп зрения ==")
    print(
        f"  интервал кадров: медиана {np.median(dt):.1f} мс, p95 {_pct(dt, 95):.1f}  -> {1000/np.median(dt):.2f} Гц"
    )
    inf = d["lanes_infer_ms"]
    inf = inf[inf > 0]
    if inf.size:
        print(f"  инференс: медиана {np.median(inf):.1f} мс, p95 {_pct(inf, 95):.1f}")


def sec_step(d):
    m = _clean(d)
    if m.sum() < 100:
        return
    t, des, R = d["ctrl_t"], d["ctrl_des_swa"], d["ctrl_R"]
    err = np.abs(d["ctrl_err"])
    print("\n== шаг уставки и ошибка слежения ==")
    print(
        f"  {'участок':<18} {'n':>7} {'|ош| мед':>9} {'|ош| p90':>9} {'шаг мед':>9} {'шаг>упора':>11}"
    )
    # Только между соседними тиками: разрыв дал бы сумму нескольких шагов.
    ok_pair = m[1:] & m[:-1] & (np.diff(t) <= 150)
    step = np.abs(np.diff(des))
    for label, lo, hi in ARCS:
        em = m & (R >= lo) & (R < hi)
        sm = ok_pair & (R[1:] >= lo) & (R[1:] < hi)
        if em.sum() < 30:
            continue
        s = step[sm]
        print(
            f"  {label:<18} {int(em.sum()):>7} {np.median(err[em]):>8.2f}° {_pct(err[em], 90):>8.2f}° "
            f"{(np.median(s) if s.size else float('nan')):>8.2f}° "
            f"{(100*np.mean(s > PID_RAIL_DEG) if s.size else float('nan')):>10.1f} %"
        )


def sec_torque(d):
    m = _clean(d)
    if m.sum() < 100:
        return
    tq, R, la = np.abs(d["ctrl_torque"]), d["ctrl_R"], d["ctrl_lat_acc"]
    cap = d.get("ctrl_max_torque")
    if cap is None or cap.size != tq.size or not (cap > 0).any():
        cap = np.full(tq.shape, FALLBACK_MAX_TORQUE_CNM)
        cap_note = (
            f"{FALLBACK_MAX_TORQUE_CNM:.0f} (константа: баг не несёт p_max_torque_cnm)"
        )
    else:
        cap = np.where(cap > 0, cap, FALLBACK_MAX_TORQUE_CNM)
        cap_note = f"{np.median(cap):.0f} (из бага)"
    rail = tq >= 0.995 * cap
    print("\n== момент: где упирается в потолок панды ==")
    print(f"  потолок: {cap_note}")
    print(
        f"  всего на упоре: {100*np.mean(rail[m]):.1f} % тиков "
        f"(маска: v > 23 км/ч, есть цель, без поворотника)"
    )
    print(
        f"  {'участок':<18} {'n':>7} {'на упоре':>10} {'|Nm| мед':>9} {'попер.уск мед':>14} {'|ош| мед':>9}"
    )
    err = np.abs(d["ctrl_err"])
    for label, lo, hi in ARCS:
        sub = m & (R >= lo) & (R < hi)
        if sub.sum() < 30:
            continue
        print(
            f"  {label:<18} {int(sub.sum()):>7} {100*np.mean(rail[sub]):>9.1f}% {np.median(tq[sub]):>9.0f} "
            f"{np.median(la[sub]):>13.2f} {np.median(err[sub]):>8.2f}°"
        )
    if rail[m].any():
        r = m & rail
        print(
            f"  на упоре: попер. ускорение мед {np.median(la[r]):.2f} м/с² (p95 {_pct(la[r],95):.2f}) — "
            f"если это мало, момент уходит не на удержание в дуге, а на проворот руля"
        )


def sec_lanes(d):
    if d["lanes_prob"].size == 0:
        return
    p, s = d["lanes_prob"], d["lanes_ystd"]
    print("\n== уверенность разметки (свои линии) ==")
    print(
        f"  вероятность: левая мед {np.median(p[:,1]):.2f}, правая мед {np.median(p[:,2]):.2f}"
    )
    print(
        f"  σ: левая мед {np.nanmedian(s[:,1]):.2f}, правая мед {np.nanmedian(s[:,2]):.2f}"
    )
    both_low = (p[:, 1] < 0.3) & (p[:, 2] < 0.3)
    print(f"  обе линии ниже 0.3: {100*np.mean(both_low):.1f} % кадров")
    st = d["ctrl_status"]
    if st.size:
        vals, cnt = np.unique(st, return_counts=True)
        shown = ", ".join(
            f"{v.decode() or '(пусто)'}={c}"
            for v, c in sorted(zip(vals, cnt), key=lambda x: -x[1])[:5]
        )
        print(f"  статусы управления: {shown}")
        lost = int(np.count_nonzero(both_low))
        if lost and b"lines_unsure" not in vals:
            print(
                f"  ВНИМАНИЕ: {lost} кадров без обеих линий, а состояния для этого нет (задача #40)"
            )


def sec_pose(d):
    if d["odom_trans"].size == 0 or d["chassis_t"].size == 0:
        return
    idx = bag_cache.nearest_index(d["odom_t"], d["chassis_t"], 50)
    ok = idx >= 0
    if ok.sum() < 200:
        return
    mv = d["odom_trans"][ok, 0]
    wv = d["chassis_v"][idx[ok]]
    fast = wv >= 3.0
    mv, wv = mv[fast], wv[fast]
    if mv.size < 100:
        return
    slope = float(np.sum(mv * wv) / np.sum(wv * wv))
    corr = float(np.corrcoef(mv, wv)[0, 1])
    print("\n== поза модели против колёс ==")
    print(
        f"  продольная: наклон {slope:.3f} (1.000 = масштаб верный), корреляция {corr:.3f}"
    )
    mw = d["odom_rot"][ok, 2][fast]
    cw = d["chassis_yaw_rate"][idx[ok]][fast]
    if np.std(cw) > 1e-6:
        s2 = float(np.sum(mw * cw) / np.sum(cw * cw))
        print(
            f"  рыскание:   наклон {s2:.3f}, корреляция {float(np.corrcoef(mw, cw)[0,1]):.3f}"
        )


def sec_notes(d, bag: Path, model: str):
    """Голосовые пометки с числами на тех же секундах, если транскрипт уже посчитан."""
    import bag_voice_notes as vn

    try:
        audio, t0 = vn.find_audio(bag)
    except SystemExit:
        return
    cache = vn._cache_path(audio, model, False)
    if not cache.exists():
        print(
            f"\n== голосовые пометки ==\n  транскрипта нет; посчитать: "
            f"python3 bag_voice_notes.py {bag} --model {model}"
        )
        return
    segs, _ = vn.transcribe(audio, model, "ru", 8, False, 0.25)
    notes = vn.group_notes(segs, vn.DEFAULT_KEYWORDS, 1.5)
    if not notes:
        return
    t, R, tq = d["ctrl_t"], d["ctrl_R"], np.abs(d["ctrl_torque"])
    err, v = np.abs(d["ctrl_err"]), d["ctrl_v"]
    print(f"\n== голосовые пометки: {len(notes)} ==")
    print(f"  {'время':>6} {'v км/ч':>7} {'R м':>7} {'|ош| max':>9} {'Nm p95':>7}  текст")
    for start, _end, text in notes:
        ts = t0 + int(start * 1000)
        m = (t >= ts - 3000) & (t <= ts + 3000)
        if m.sum() < 3:
            continue
        rm = np.median(R[m])
        mm, ss = divmod(int(start), 60)
        print(
            f"  {mm:02d}:{ss:02d}  {np.median(v[m])*3.6:>7.1f} "
            f"{('%7.0f' % rm) if np.isfinite(rm) else '    ---'} {err[m].max():>8.1f}° "
            f"{_pct(tq[m], 95):>7.0f}  {text[:60]}"
        )


def sec_offset(d):
    """Положение в полосе по прямым и дугам — то, ради чего существует bag_arc_offset.

    Считается только там, где полоса реально опознана: при выключенном подмешивании смещение
    описывает план модели, а не полосу, и сравнивать его с нулём бессмысленно.
    """
    m = _clean(d) & d["ctrl_lane_anchored"] & d["ctrl_lanelines_active"]
    off, R, w = d["ctrl_lane_offset"], d["ctrl_R"], d["ctrl_lane_width"]
    print("\n== положение в полосе (только где полоса опознана) ==")
    share = 100.0 * np.mean(
        _clean(d) & d["ctrl_lane_anchored"] & d["ctrl_lanelines_active"]
    )
    print(f"  годных тиков: {int(m.sum())} ({share:.1f} % от управляемых)")
    if m.sum() < 100:
        print("  мало данных: полоса почти не опознавалась, смещение не о полосе")
        return
    print(
        f"  {'участок':<18} {'n':>7} {'смещ мед':>9} {'смещ p90':>9} {'|смещ| мед':>11} {'ширина мед':>11}"
    )
    for label, lo, hi in ARCS:
        sub = m & (R >= lo) & (R < hi)
        if sub.sum() < 30:
            continue
        o = off[sub]
        print(
            f"  {label:<18} {int(sub.sum()):>7} {np.median(o):>+8.3f}м {_pct(o, 90):>+8.3f}м "
            f"{np.median(np.abs(o)):>10.3f}м {np.median(w[sub]):>10.2f}м"
        )
    left = off[m] > 0
    print(
        f"  знак: правее центра {100*np.mean(~left):.0f} %, левее {100*np.mean(left):.0f} % — "
        f"устойчивый перекос означает смещение камеры или угловой ноль, а не управление"
    )


def sec_latency(d):
    """Цепочка кадр камеры -> инференс -> публикация команды. Разности, а не абсолютные метки."""
    cap, vis, pub = d["ctrl_capture_t"], d["ctrl_vision_t"], d["ctrl_publish_t"]
    ok = (cap > 0) & (pub > 0)
    if ok.sum() < 100:
        return
    print("\n== задержка ==")
    full = (pub - cap)[ok].astype(float)
    print(
        f"  кадр -> команда:   медиана {np.median(full):5.1f} мс  p95 {_pct(full, 95):5.1f}  max {full.max():5.0f}"
    )
    okv = ok & (vis > 0)
    if okv.sum() > 100:
        post = (pub - vis)[okv].astype(float)
        infer = (vis - cap)[okv].astype(float)
        print(
            f"  кадр -> инференс:  медиана {np.median(infer):5.1f} мс  p95 {_pct(infer, 95):5.1f}"
        )
        print(
            f"  инференс -> команда: медиана {np.median(post):5.1f} мс  p95 {_pct(post, 95):5.1f}"
        )
    fdt = d["ctrl_frame_dt"]
    fdt = fdt[fdt > 0]
    if fdt.size:
        print(f"  шаг зрения, как его видел контроллер: медиана {np.median(fdt):.1f} мс")


def sec_mw(d, bag: Path):
    """Средний слой: потери в очередях и отставание сервисов. Считается прямо по топику."""
    from vis.bag_io import load_topic_messages

    try:
        st = [m for _, m, _ in load_topic_messages(bag, "middleware/stats")]
    except Exception:
        return
    if not st:
        return
    print("\n== middleware ==")
    last = st[-1]
    drop = np.array([int(getattr(m, "dropped_total", 0)) for m in st])
    lag = np.array([bool(getattr(m, "any_lagging", False)) for m in st])
    print(
        f"  снимков {len(st)}, сервисов {int(getattr(last, 'services', 0))}, "
        f"потеряно всего {drop.max()}, отставание хоть у кого-то: {100*lag.mean():.1f} % снимков"
    )
    rows = {}
    for m in st:
        for s in getattr(m, "services_timing", []):
            r = rows.setdefault(
                s.name, {"drop": 0, "lag": 0, "n": 0, "max_cb": 0.0, "max_dt": 0.0}
            )
            r["drop"] = max(r["drop"], int(s.dropped))
            r["lag"] += int(bool(s.lagging))
            r["n"] += 1
            r["max_cb"] = max(r["max_cb"], float(s.max_cb_ms))
            r["max_dt"] = max(r["max_dt"], float(s.max_dt_ms))
    if not rows:
        return
    print(f"  {'сервис':<18} {'потерь':>7} {'отстав':>7} {'cb max':>8} {'dt max':>8}")
    for name, r in sorted(rows.items(), key=lambda kv: -kv[1]["drop"]):
        print(
            f"  {name:<18} {r['drop']:>7} {100*r['lag']/max(r['n'],1):>6.0f}% "
            f"{r['max_cb']:>7.1f}м {r['max_dt']:>7.1f}м"
        )


SECTIONS = {
    "rate": sec_rate,
    "step": sec_step,
    "offset": sec_offset,
    "torque": sec_torque,
    "latency": sec_latency,
    "lanes": sec_lanes,
    "pose": sec_pose,
}


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument(
        "--only", default=None, help="через запятую: " + ",".join(SECTIONS) + ",mw,notes"
    )
    ap.add_argument("--refresh", action="store_true", help="пересобрать кэш")
    ap.add_argument("--notes-model", default="medium")
    args = ap.parse_args()

    want = (
        [s.strip() for s in args.only.split(",")]
        if args.only
        else list(SECTIONS) + ["mw", "notes"]
    )

    for bag in args.bags:
        d = bag_cache.load(bag, refresh=args.refresh)
        dur = (d["ctrl_t"][-1] - d["ctrl_t"][0]) / 60000.0 if d["ctrl_t"].size else 0.0
        print(
            f"\n########  {bag.name}  ({dur:.1f} мин, тиков управления {d['ctrl_t'].size})"
        )
        for key in want:
            if key == "mw":
                sec_mw(d, bag)
            elif key in SECTIONS:
                SECTIONS[key](d)
            elif key == "notes":
                sec_notes(d, bag, args.notes_model)


if __name__ == "__main__":
    main()
