"""Кэш заезда в виде плоских numpy-таблиц: 4 с на разбор часового заезда, 0.1 с из кэша.

Кэшируются не сообщения, а величины, которыми пользуются разборы: время, скорость, углы, момент,
кривизна, вероятности и σ линий, поза. Сырой выход модели (`model_out`, 6504 float на кадр) НЕ
кэшируется — он нужен только при перепроверке самой модели, а разворачивание его в python-список
стоит дороже всего остального разбора вместе взятого.

Инвалидация по содержимому каталога: список .bin с размерами и временем правки плюс версия схемы.
Молчаливо устаревший кэш хуже отсутствующего, потому что выглядит как свежий результат.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict

import numpy as np

from . import bag_io

# Поднимать при ЛЮБОМ изменении набора или смысла столбцов.
# v5: заготовки protobuf пересобраны, в них появилось поле p_max_torque_cnm — в кэшах v4
# столбец ctrl_max_torque заполнен нулями из getattr-заглушки.
SCHEMA_VERSION = 5

CACHE_NAME = f".bagcache_v{SCHEMA_VERSION}.npz"


def _fingerprint(bag: Path) -> str:
    parts = []
    for p in sorted(bag.rglob("*.bin")):
        st = p.stat()
        parts.append(f"{p.relative_to(bag)}:{st.st_size}:{int(st.st_mtime)}")
    h = hashlib.sha1("|".join(parts).encode()).hexdigest()
    return f"{SCHEMA_VERSION}:{len(parts)}:{h}"


def _col(msgs, fn, dtype=float):
    if not msgs:
        return np.zeros(0, dtype=dtype)
    return np.array([fn(m[1]) for m in msgs], dtype=dtype)


def _times(msgs):
    return (
        np.array([m[0] for m in msgs], dtype=np.int64)
        if msgs
        else np.zeros(0, dtype=np.int64)
    )


def _radius(kappa):
    """Радиус дуги из кривизны; прямая — бесконечность, а не гигантское число."""
    a = np.abs(kappa)
    return np.where(a < 1e-6, np.inf, 1.0 / np.maximum(a, 1e-9))


def _build(bag: Path) -> Dict[str, np.ndarray]:
    out: Dict[str, np.ndarray] = {}

    dbg = bag_io.load_topic_messages(bag, "control/lane_keep_debug")
    out["ctrl_t"] = _times(dbg)
    out["ctrl_v"] = _col(dbg, lambda d: d.speed_mps)
    out["ctrl_des_swa"] = _col(dbg, lambda d: d.desired_swa_deg)
    out["ctrl_act_swa"] = _col(dbg, lambda d: d.actual_swa_deg)
    out["ctrl_torque"] = _col(dbg, lambda d: int(d.torque_cnm))
    # Поле выбирается по контроллеру, а не по истинности числа: `a or b` подставил бы b при a == 0.0,
    # а ноль кривизны — законное «идеально прямо», а не «данных нет».
    out["ctrl_kappa"] = _col(
        dbg, lambda d: d.mpc_kappa_used if d.controller == "mpc" else d.pp_curvature
    )
    out["ctrl_has_target"] = _col(dbg, lambda d: bool(d.has_target), dtype=bool)
    out["ctrl_slew_clipped"] = _col(dbg, lambda d: bool(d.slew_clipped), dtype=bool)
    # Действующий потолок момента из самого бага: зашитая константа врала бы после правки конфига.
    out["ctrl_max_torque"] = _col(dbg, lambda d: int(getattr(d, "p_max_torque_cnm", 0)))
    # Строки в npz живут только как массив объектов; храним как массив байтовых строк.
    out["ctrl_status"] = (
        np.array([d[1].status for d in dbg], dtype="S24") if dbg else np.zeros(0, "S24")
    )
    out["ctrl_err"] = out["ctrl_des_swa"] - out["ctrl_act_swa"]
    out["ctrl_R"] = _radius(out["ctrl_kappa"])
    out["ctrl_lat_acc"] = out["ctrl_v"] ** 2 * np.abs(out["ctrl_kappa"])

    ch = bag_io.load_topic_messages(bag, "vehicle/state")
    out["chassis_t"] = _times(ch)
    out["chassis_v"] = _col(ch, lambda c: c.v_ego)
    out["chassis_yaw_rate"] = _col(ch, lambda c: c.yaw_rate)
    out["chassis_swa"] = _col(ch, lambda c: c.steering_angle_deg)
    out["chassis_blinker"] = _col(
        ch, lambda c: bool(c.left_blinker or c.right_blinker), dtype=bool
    )

    ln = bag_io.load_topic_messages(bag, "vision/lanes")
    out["lanes_t"] = _times(ln)
    out["lanes_capture_t"] = _col(ln, lambda l: l.capture_ts_ms, dtype=np.int64)
    out["lanes_infer_ms"] = _col(ln, lambda l: l.infer_duration_ms)
    out["lanes_frame_id"] = _col(ln, lambda l: l.frame_id, dtype=np.int64)
    # Четыре линии: 0 и 3 — соседние полосы, 1 и 2 — свои. Порядок сохраняется как есть.
    out["lanes_prob"] = (
        np.array([[p.prob for p in l[1].lanes] for l in ln], dtype=float)
        if ln
        else np.zeros((0, 4))
    )
    out["lanes_ystd"] = (
        np.array(
            [
                [
                    (np.median(list(p.y_std)) if len(p.y_std) else np.nan)
                    for p in l[1].lanes
                ]
                for l in ln
            ],
            dtype=float,
        )
        if ln
        else np.zeros((0, 4))
    )

    od = bag_io.load_topic_messages(bag, "model/camera_odometry")
    out["odom_t"] = _times(od)
    out["odom_trans"] = (
        np.array([list(o[1].trans) for o in od], dtype=float) if od else np.zeros((0, 3))
    )
    out["odom_rot"] = (
        np.array([list(o[1].rot) for o in od], dtype=float) if od else np.zeros((0, 3))
    )
    out["odom_rot_std"] = (
        np.array([list(o[1].rot_std) for o in od], dtype=float)
        if od
        else np.zeros((0, 3))
    )

    stm = bag_io.load_topic_messages(bag, "controls/steer")
    out["steer_t"] = _times(stm)
    out["steer_torque"] = _col(stm, lambda s: int(s.torque_cnm), dtype=float)
    out["steer_enabled"] = _col(stm, lambda s: bool(s.enabled), dtype=bool)

    return out


def load(bag: Path, refresh: bool = False, quiet: bool = False) -> Dict[str, np.ndarray]:
    """Таблицы заезда, из кэша если он свеж."""
    bag = Path(bag)
    cache = bag / CACHE_NAME
    fp = _fingerprint(bag)
    if cache.exists() and not refresh:
        try:
            z = np.load(cache, allow_pickle=False)
            if str(z["_fingerprint"]) == fp:
                data = {k: z[k] for k in z.files if not k.startswith("_")}
                if not quiet:
                    print(f"# кэш {cache.name}")
                return data
        except Exception:
            pass  # битый кэш — просто пересобираем, это дешевле разбирательства
    data = _build(bag)
    tmp = cache.with_suffix(".tmp.npz")
    np.savez_compressed(tmp, _fingerprint=np.array(fp), **data)
    tmp.replace(cache)
    if not quiet:
        print(f"# кэш собран: {cache.name}")
    return data


def nearest_index(
    src_t: np.ndarray, dst_t: np.ndarray, max_dt_ms: int = 50
) -> np.ndarray:
    """Для каждого src_t индекс ближайшего dst_t, либо −1 если ближе max_dt_ms ничего нет."""
    if dst_t.size == 0 or src_t.size == 0:
        return np.full(src_t.shape, -1, dtype=np.int64)
    idx = np.clip(np.searchsorted(dst_t, src_t), 0, dst_t.size - 1)
    left = np.clip(idx - 1, 0, dst_t.size - 1)
    pick = np.where(np.abs(dst_t[left] - src_t) < np.abs(dst_t[idx] - src_t), left, idx)
    return np.where(np.abs(dst_t[pick] - src_t) <= max_dt_ms, pick, -1)


def blinker_mask(
    d: Dict[str, np.ndarray], t: np.ndarray, pad_ms: int = 2000
) -> np.ndarray:
    """Тики рядом с включённым поворотником, расширенные на 2 с: водитель начинает уводить машину
    раньше, чем щёлкает рычаг, и продолжает после того, как тот выключился сам."""
    ct, on = d["chassis_t"], d["chassis_blinker"]
    if ct.size == 0 or not on.any():
        return np.zeros(t.shape, dtype=bool)
    edges = np.diff(on.astype(np.int8), prepend=0)
    starts = ct[edges == 1]
    ends = ct[edges == -1] if (edges == -1).any() else np.array([ct[-1]])
    if ends.size < starts.size:
        ends = np.append(ends, ct[-1])
    mask = np.zeros(t.shape, dtype=bool)
    for a, b in zip(starts, ends[: starts.size]):
        mask |= (t >= a - pad_ms) & (t <= b + pad_ms)
    return mask
