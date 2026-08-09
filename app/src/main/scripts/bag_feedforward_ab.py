#!/usr/bin/env python3
"""Что изменил бы новый вид упреждения на записанном заезде.

ВОСПРОИЗВЕДЕНИЕ РАЗОМКНУТОЕ. Фактический угол руля в баге — это то, что машина сделала под старым
моментом; с большим моментом она повернула бы сильнее и ошибка была бы меньше. Значит расчёт
систематически ЗАВЫШАЕТ новый момент, и читать его надо как верхнюю границу, а не как предсказание.
Замкнуть контур офлайн нечем: нужна проверенная модель реакции рейки, а её нет.

Проверяются две вещи: не станет ли команда систематически бить в потолок, и приходит ли момент
раньше там, где водитель отметил недокрут.

Арифметика PID повторяет `utils/lat_control_pid.h`, который покрыт модульными тестами.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import _path  # noqa: F401
from vis import bag_cache

MAX_TORQUE_CNM = 300.0


def run_pid(err_deg, swa_deg, v_mps, k_p, k_i, k_f, v_floor, dt_s):
    """Тот же порядок действий, что в PidController::update, включая условие анти-виндапа.

    Шаг интегратора берётся из меток времени: регулятор на машине крутится по тикам шины, а
    `control/lane_keep_debug` пишется на темпе зрения.
    """
    i_acc = 0.0
    out = np.empty_like(err_deg)
    f_out = np.empty_like(err_deg)
    for n in range(err_deg.size):
        e = err_deg[n]
        p = e * k_p
        f = k_f * swa_deg[n] * (v_mps[n] ** 2 + v_floor**2)
        i_new = i_acc + e * k_i * dt_s[n]
        control_try = p + i_new + f
        if (e >= 0.0 and (control_try <= 1.0 or i_new < 0.0)) or (e <= 0.0 and (control_try >= -1.0 or i_new > 0.0)):
            i_acc = i_new
        out[n] = np.clip(p + i_acc + f, -1.0, 1.0)
        f_out[n] = f
    return out, f_out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--kp", type=float, default=0.6)
    ap.add_argument("--ki", type=float, default=0.2)
    ap.add_argument("--kf-old", type=float, default=0.00006)
    ap.add_argument("--kf-new", type=float, default=0.00015)
    ap.add_argument("--floor-new", type=float, default=9.8)
    args = ap.parse_args()

    d = bag_cache.load(args.bag)
    if d["ctrl_t"].size == 0:
        raise SystemExit("нет control/lane_keep_debug")

    t = d["ctrl_t"]
    v = d["ctrl_v"]
    des = d["ctrl_des_swa"]
    tq = d["ctrl_torque"]
    err = d["ctrl_err"]
    R = d["ctrl_R"]

    dt_s = np.diff(t, prepend=t[0]).astype(float) / 1000.0
    # Разрывы записи не должны разово доливать интегратор: ограничиваем шаг разумным периодом.
    dt_s = np.clip(dt_s, 0.0, 0.1)

    old, f_old = run_pid(err, des, v, args.kp, args.ki, args.kf_old, 0.0, dt_s)
    new, f_new = run_pid(err, des, v, args.kp, args.ki, args.kf_new, args.floor_new, dt_s)

    keep = v > 6.4
    print(f"=== {args.bag.name} ===  тиков выше 23 км/ч: {keep.sum()}")
    print(f"сверка: воспроизведённый старый момент против записанного — медиана |Δ| "
          f"{np.median(np.abs(old[keep] * MAX_TORQUE_CNM - tq[keep])):.1f} cNm\n")

    print(f"{'участок':<12} {'n':>6} {'на упоре было':>14} {'станет':>8} {'|момент| мед было':>18} {'станет':>8}")
    for name, lo, hi in [("R<83", 0, 83), ("83-167", 83, 167), ("167-500", 167, 500), ("R>500", 500, np.inf)]:
        m = keep & (R >= lo) & (R < hi)
        if m.sum() < 50:
            continue
        print(
            f"{name:<12} {m.sum():>6} "
            f"{100 * np.mean(np.abs(old[m]) >= 0.995):>13.1f}% "
            f"{100 * np.mean(np.abs(new[m]) >= 0.995):>7.1f}% "
            f"{np.median(np.abs(old[m])) * MAX_TORQUE_CNM:>17.0f} "
            f"{np.median(np.abs(new[m])) * MAX_TORQUE_CNM:>7.0f}"
        )

    print("\nчем держится момент в установившемся режиме (|ошибка| < 1°):")
    st = keep & (np.abs(err) < 1.0) & (np.abs(des) > 2.0)
    for label, f_arr, ctl in (("было", f_old, old), ("станет", f_new, new)):
        share = np.median(np.abs(f_arr[st]) / np.maximum(np.abs(ctl[st]), 1e-6))
        print(f"  {label:>6}: доля упреждения в команде {100 * share:5.1f} %  "
              f"(остальное набирает интегратор)")

    print("\nна сколько раньше приходит момент — вход в дугу (кривизна растёт, |ошибка| ещё < 2°):")
    entry = keep & (np.abs(err) < 2.0) & (R < 500)
    if entry.sum() > 100:
        print(f"  медиана |момента| на входе: было {np.median(np.abs(old[entry])) * MAX_TORQUE_CNM:.0f} cNm, "
              f"станет {np.median(np.abs(new[entry])) * MAX_TORQUE_CNM:.0f} cNm")


if __name__ == "__main__":
    main()
