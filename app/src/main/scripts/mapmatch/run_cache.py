"""Кэш величин заезда, которыми пользуется локализация.

Разбор одного бега стоит десятки секунд: `vehicle/state` на этом заезде — 131 179 сообщений, и
каждый прогон поиска начинался с их повторного чтения. Отладка алгоритма, где нужно два десятка
прогонов подряд, от этого становится не медленной, а бессмысленной.

Кэшируются не сообщения, а ровно то, что нужно: время, скорость и скорость поворота для трека,
сырые широта и долгота для эталона (в системе координат карты они станут потом — карта может быть
другой), и поворотники.

Инвалидация по содержимому каталога плюс версия схемы, как в vis/bag_cache.py: молчаливо устаревший
кэш хуже отсутствующего.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from mapmatch.track_from_bag import list_topics, load_topic_messages, motion_profile

SCHEMA_VERSION = 1
CACHE_NAME = f".mapmatchcache_v{SCHEMA_VERSION}.npz"


def _fingerprint(bag: Path) -> str:
    parts = []
    for p in sorted(bag.rglob("*.bin")):
        st = p.stat()
        parts.append(f"{p.relative_to(bag)}:{st.st_size}:{int(st.st_mtime)}")
    h = hashlib.sha1("|".join(parts).encode()).hexdigest()
    return f"{SCHEMA_VERSION}:{len(parts)}:{h}"


def _build(bag: Path) -> Dict[str, np.ndarray]:
    out: Dict[str, np.ndarray] = {}

    t, v, w = motion_profile(bag)
    out["t_s"] = np.asarray(t, dtype=float)
    out["speed_mps"] = np.asarray(v, dtype=float)
    out["yaw_rate_rps"] = np.asarray(w, dtype=float)

    lat = lon = np.zeros(0)
    if "sensors/gps/location" in list_topics(bag):
        rows = load_topic_messages(bag, "sensors/gps/location")
        pts = [
            (float(m.latitude), float(m.longitude))
            for _, m in [(r[0], r[1]) for r in rows]
            if abs(float(m.latitude)) > 1e-6
        ]
        if pts:
            lat = np.array([p[0] for p in pts])
            lon = np.array([p[1] for p in pts])
    out["gnss_lat"] = lat
    out["gnss_lon"] = lon

    bt = bl = br = np.zeros(0)
    rows = (
        load_topic_messages(bag, "vehicle/state")
        if "vehicle/state" in list_topics(bag)
        else []
    )
    if rows:
        bt = np.array([r[0] for r in rows], dtype=float) / 1000.0
        bl = np.array([bool(r[1].left_blinker) for r in rows])
        br = np.array([bool(r[1].right_blinker) for r in rows])
    out["blink_t_s"] = bt
    out["blink_left"] = bl
    out["blink_right"] = br

    return out


def load(bag: Path, refresh: bool = False) -> Dict[str, np.ndarray]:
    """Величины заезда из кэша, при необходимости пересобирая его."""
    path = bag / CACHE_NAME
    want = _fingerprint(bag)
    if not refresh and path.exists():
        try:
            data = np.load(path, allow_pickle=False)
            if str(data["fingerprint"]) == want:
                return {k: data[k] for k in data.files if k != "fingerprint"}
        except Exception:
            pass

    data = _build(bag)
    try:
        np.savez_compressed(path, fingerprint=want, **data)
    except OSError:
        pass  # каталог бега может быть только для чтения — кэш не обязателен
    return data
