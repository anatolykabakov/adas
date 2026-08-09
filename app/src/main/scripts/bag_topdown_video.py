#!/usr/bin/env python3
"""Вид сверху из бага в MP4: что модель видела, что контроллер решил, куда машина поехала.

Кадр видео = кадр камеры. Машина смотрит **вправо**, поперечное смещение — по вертикали. Поперечный размер
окна подбирается по разметке за весь клип плюс запас 3 м с каждой стороны (один раз, чтобы масштаб не
дёргался); вперёд — `--ahead`, потому что разметка публикуется до 192 м и от неё поперечник сжался бы в
ничто.
Цвет кузова — факт актюации (`HCA_01.HCA_Active` с шины), а не наше разрешение: у серой машины
момент на шину не шёл, и качество слежения по ней оценивать нельзя.

  python3 bag_topdown_video.py <bag> --list-assist            # где рулить было разрешено
  python3 bag_topdown_video.py <bag> --start 2860 --duration 14
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator, Optional

import numpy as np

import _path  # noqa: F401
from core import path_fusion as pf
from proto import bag_pb2
from vis.bag_io import fix_topic, lateral_actuation_on, parse_payload

HCA_01 = 0x126
CAR_L, CAR_W, REAR_OVERHANG = 4.26, 1.80, 0.90  # Golf 7, задняя ось в начале координат
ARC_LEN_M = 25.0  # длина дуги команды
PAST_S, FUTURE_S = 8.0, 6.0
MARGIN_M = 3.0  # запас вокруг линий разметки


def bits(data: bytes, start: int, length: int) -> int:
    """Little-endian, как `set_bits` в volkswagen/mqbcan.cpp."""
    return sum(
        1 << i for i in range(length) if data[(start + i) // 8] >> ((start + i) % 8) & 1
    )


def col(rows: list, name: str, default: float = 0.0) -> np.ndarray:
    return np.asarray(
        [float(getattr(r[1], name, default) or default) for r in rows], dtype=np.float64
    )


def seq(obj: Any, name: str) -> np.ndarray:
    v = getattr(obj, name, None)
    return np.asarray(list(v), dtype=np.float64) if v else np.empty(0)


class Bag:
    """Ленивое чтение топиков: шард за шардом, с остановкой за правым краем окна.

    Полный `vision/lanes` из полуторагигабайтного бага читается минуты; окну на сорок секунд нужен
    десяток шардов, а они лежат по времени.
    """

    def __init__(self, path: Path):
        self.path = path

    def read(
        self, topic: str, t0: Optional[float] = None, t1: Optional[float] = None
    ) -> list:
        d = self.path / fix_topic(topic)
        if not d.is_dir():
            return []
        out = []
        for p in sorted(d.glob("*.bin")):
            shard = bag_pb2.Bag()
            shard.ParseFromString(p.read_bytes())
            if not shard.messages:
                continue
            if t1 is not None and int(shard.messages[0].timestamp) > t1:
                break
            if t0 is not None and int(shard.messages[-1].timestamp) < t0:
                continue
            for m in shard.messages:
                ts = int(m.timestamp)
                if (t0 is None or ts >= t0) and (t1 is None or ts <= t1):
                    out.append((ts, parse_payload(m)))
        return sorted(out, key=lambda r: r[0])

    def start_ms(self) -> Optional[int]:
        """Метка первого кадра модели — начало отсчёта для --start."""
        d = self.path / fix_topic("vision/lanes")
        for p in sorted(d.glob("*.bin")) if d.is_dir() else []:
            shard = bag_pb2.Bag()
            shard.ParseFromString(p.read_bytes())
            if shard.messages:
                return int(shard.messages[0].timestamp)
        return None

    def hca(self, t0: float, t1: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(t, HCA_Active, момент на шине) из наших же кадров HCA_01."""
        ts, on, tq = [], [], []
        for t, msg in self.read("can/rx", t0, t1):
            for fr in getattr(msg, "frames", []):
                if fr.address == HCA_01 and fr.src == 128 and len(fr.data) >= 8:
                    ts.append(t)
                    on.append(bool(bits(fr.data, 34, 1)))
                    tq.append((-1 if bits(fr.data, 31, 1) else 1) * bits(fr.data, 16, 14))
        return np.asarray(ts, float), np.asarray(on, bool), np.asarray(tq, float)


class Align:
    """Ближайший по времени отсчёт, или None если слишком далеко."""

    def __init__(self, t: np.ndarray, tol_ms: float):
        self.t, self.tol = t, tol_ms

    def __call__(self, t_ref: float) -> Optional[int]:
        if not len(self.t):
            return None
        i = int(np.clip(np.searchsorted(self.t, t_ref), 0, len(self.t) - 1))
        j = max(i - 1, 0)
        i = j if abs(self.t[j] - t_ref) < abs(self.t[i] - t_ref) else i
        return i if abs(self.t[i] - t_ref) <= self.tol else None


@dataclass
class Scene:
    """Один момент: геометрия в системе машины (x вперёд, y вправо) плюс числа для панели."""

    t_s: float = 0.0
    v: float = 0.0
    assist: bool = False
    tq_req: float = 0.0
    tq_bus: float = 0.0
    swa_des: float = 0.0
    swa_act: float = 0.0
    kappa: float = 0.0
    cte: float = 0.0
    blend: float = 0.0
    status: str = ""
    conf: tuple[float, float] = (1.0, 1.0)
    left: Optional[np.ndarray] = None
    right: Optional[np.ndarray] = None
    plan: Optional[np.ndarray] = None
    center: Optional[np.ndarray] = None
    ref: Optional[np.ndarray] = None
    past: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    future: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))

    @property
    def arc(self) -> np.ndarray:
        """Дуга постоянной кривизны — команда, а не план.

        `mpc_kappa_used` живёт во внутреннем фрейме MPC, где y влево (`stepFlowpilot` подаёт туда
        `poly_left` с отрицанием y и потом берёт `-steerFromCurvature`), а вся геометрия бага — y вправо.
        Поэтому знак переворачивается здесь. Проверка на кадре 2861.6 с: κ +0.0062, задан руль +18.05°, а
        положительный угол на CAN у этой машины — поворот влево; опора там тоже уходит влево.
        """
        k = -self.kappa
        s = np.linspace(0.0, ARC_LEN_M, 40)
        if abs(k) < 1e-6:
            return np.column_stack([s, np.zeros_like(s)])
        r, th = 1.0 / k, s * k
        return np.column_stack([r * np.sin(th), r * (1.0 - np.cos(th))])

    @property
    def panel(self) -> str:
        return "\n".join(
            [
                f"t={self.t_s:7.2f} с   v={self.v:5.1f} м/с ({self.v * 3.6:5.1f} км/ч)",
                f"ассист: {'ВКЛ' if self.assist else 'выкл'}   момент: запрос {self.tq_req:+5.0f} → "
                f"шина {self.tq_bus:+5.0f} cNm",
                f"руль: задан {self.swa_des:+6.2f}°  факт {self.swa_act:+6.2f}°  "
                f"ошибка {self.swa_des - self.swa_act:+6.2f}°",
                f"κ {self.kappa:+.5f} 1/м   CTE {self.cte:+5.2f} м   {self.status}",
                f"смешивание {self.blend:.2f}   доверие σ: Л {self.conf[0]:.2f} / П {self.conf[1]:.2f}",
            ]
        )


class Scenes:
    """Сборка сцен из бага: чтение, выравнивание по времени, пересчёт опоры."""

    def __init__(self, bag: Bag, start_s: float, duration_s: float, std_range_m: float):
        self.bag, self.std_range = bag, std_range_m
        t0 = bag.start_ms()
        if t0 is None:
            raise SystemExit("в баге нет vision/lanes")
        self.t0 = t0
        a, b = t0 + start_s * 1000.0, t0 + (start_s + duration_s) * 1000.0
        pad = max(PAST_S, FUTURE_S) * 1000.0

        self.lanes = bag.read("vision/lanes", a, b)
        self.pose = bag.read("localization/pose", a - pad, b + pad)
        self.state = bag.read("vehicle/state", a - pad, b + pad)
        self.dbg = bag.read("control/lane_keep_debug", a - pad, b + pad)
        self.h_t, self.h_on, self.h_tq = bag.hca(a - pad, b + pad)

        self.p_t = np.asarray([r[0] for r in self.pose], float)
        self.p_xy = np.column_stack([col(self.pose, "x"), col(self.pose, "y")])
        self.p_yaw = col(self.pose, "yaw")
        self.at_pose = Align(self.p_t, 200.0)
        self.at_state = Align(np.asarray([r[0] for r in self.state], float), 120.0)
        self.at_dbg = Align(np.asarray([r[0] for r in self.dbg], float), 120.0)
        self.at_hca = Align(self.h_t, 60.0)

    def __len__(self) -> int:
        return len(self.lanes)

    NEAR_M = 20.0  # ближняя зона, по которой измеряется нормальная ширина коридора
    BAND_FACTOR = 3.0  # во сколько раз шире неё считать разметку ещё разметкой

    def extent(
        self, margin_m: float = MARGIN_M, ahead_m: float = 40.0
    ) -> tuple[float, float, float, float]:
        """Границы кадра по линиям разметки за весь клип плюс запас.

        Считается один раз по всему клипу, а не на каждом кадре: иначе масштаб дёргался бы и глазом
        сравнить соседние моменты стало бы нельзя.

        **Адаптивен поперечник, вперёд смотрим на `ahead_m`.** Так вышло не из лени: разметка публикуется
        до 192 м и на прямой честно параллельна на всей длине, поэтому «границы по линиям» дают окно
        198 × 13 м, где поперечник сжимается в сотню пикселей и смотреть нечего. Вперёд поэтому — параметр,
        а поперёк — ровно то, о чём речь: размах линий плюс запас с каждой стороны.

        Дальний хвост при этом всё равно отбрасывается, потому что он экстраполяция: замер по окну
        2599…2609 с даёт медиану |y| 1.6 м на 0–20 м, 7.1 м на 40–60 м и 70 м на 120–200 м при максимуме
        155 м. Берутся только точки внутри коридора шириной `BAND_FACTOR` × медианы |y| ближней зоны.

        Машина влезает сама: разметка начинается с x = 0, а запас назад больше её заднего свеса.
        """
        xs, ys = [], []
        for _, ll in self.lanes:
            x = seq(ll, "x")
            lanes = list(getattr(ll, "lanes", []))
            for k in (1, 2):
                y = seq(lanes[k], "y") if k < len(lanes) else np.empty(0)
                if y.size == x.size and x.size:
                    good = np.isfinite(x) & np.isfinite(y)
                    xs.append(x[good])
                    ys.append(y[good])
        if not xs:
            return -margin_m, ahead_m, -margin_m, margin_m

        x_all, y_all = np.concatenate(xs), np.concatenate(ys)
        near = x_all <= self.NEAR_M
        band = (
            self.BAND_FACTOR * float(np.median(np.abs(y_all[near])))
            if near.any()
            else 5.0
        )
        keep = (np.abs(y_all) <= max(band, 2.0)) & (x_all <= ahead_m)
        if not keep.any():
            keep = near
        y_k = y_all[keep]
        return (
            -margin_m,
            ahead_m,
            float(y_k.min()) - margin_m,
            float(y_k.max()) + margin_m,
        )

    def __iter__(self) -> Iterator[Scene]:
        for t, ll in self.lanes:
            yield self._scene(t, ll)

    def _scene(self, t: float, ll: Any) -> Scene:
        s = Scene(t_s=(t - self.t0) / 1000.0)

        i = self.at_dbg(t)
        if i is not None:
            m = self.dbg[i][1]
            s.blend = float(getattr(m, "p_lane_blend_scale", 0.6) or 0.6)
            cam_off = float(getattr(m, "p_camera_offset_m", 0.05))
            s.kappa = float(getattr(m, "mpc_kappa_used", 0.0) or 0.0)
            s.swa_des = float(getattr(m, "desired_swa_deg", 0.0))
            s.swa_act = float(getattr(m, "actual_swa_deg", 0.0))
            s.tq_req = float(getattr(m, "torque_cnm", 0.0))
            s.cte = float(getattr(m, "mpc_cte_m", 0.0))
            s.status = str(getattr(m, "status", ""))
        else:
            s.blend, cam_off = 0.6, 0.05

        i = self.at_hca(t)
        if i is not None:
            s.assist, s.tq_bus = bool(self.h_on[i]), float(self.h_tq[i])
        i = self.at_state(t)
        if i is not None:
            s.v = float(getattr(self.state[i][1], "v_ego", 0.0))

        self._geometry(s, ll, cam_off)
        self._trajectory(s, t)
        return s

    def _geometry(self, s: Scene, ll: Any, cam_off: float) -> None:
        """Линии, центр полосы и опора — зеркалом `laneLinesToPath`, а не второй реализацией."""
        xs = seq(ll, "x")
        lanes = list(getattr(ll, "lanes", []))
        ys = [seq(l, "y") for l in lanes]
        stds = [seq(l, "y_std") for l in lanes]
        probs = [float(getattr(l, "prob", 0.0)) for l in lanes]
        if len(ys) < 3 or not xs.size:
            return

        conf, w = [], []
        for k in (1, 2):
            c = 1.0
            if stds[k].size:
                med = pf._median_std(stds[k], xs, self.std_range)  # noqa: SLF001
                c = pf._std_confidence(
                    med, pf.DEFAULT_LANE_STD_GOOD_M, pf.DEFAULT_LANE_STD_BAD_M
                )  # noqa: SLF001
            conf.append(c)
            w.append(
                pf._soft_lane_prob(probs[k], pf.DEFAULT_MIN_LANE_PROB) * c
            )  # noqa: SLF001
        s.conf = (conf[0], conf[1])

        if ys[1].size == xs.size:
            s.left = np.column_stack([xs, ys[1]])
        if ys[2].size == xs.size:
            s.right = np.column_stack([xs, ys[2]])

        plan_x, plan_y = seq(ll, "plan_x"), seq(ll, "plan_y")
        if plan_x.size and plan_x.size == plan_y.size:
            s.plan = np.column_stack([plan_x, plan_y])

        if s.left is not None and s.right is not None and (w[0] > 0 or w[1] > 0):
            # Центр восстанавливается от каждой линии независимо и взвешивается доверием — поэтому
            # потеря одной линии его не рвёт.
            width = np.clip(
                np.abs(ys[2] - ys[1]),
                pf.DEFAULT_LANE_WIDTH_MIN_M,
                pf.DEFAULT_LANE_WIDTH_MAX_M,
            )
            c = (w[0] * (ys[1] + 0.5 * width) + w[1] * (ys[2] - 0.5 * width)) / (
                w[0] + w[1] + 1e-6
            )
            s.center = np.column_stack([xs, c])

        ref = pf.lane_lines_to_path(
            plan_x if plan_x.size else None,
            plan_y if plan_y.size else None,
            xs,
            [y if y.size else None for y in ys],
            probs,
            lane_blend_scale=s.blend,
            lane_stds=[d if d.size else None for d in stds],
            lane_std_range_m=self.std_range,
        )
        if ref is not None:
            s.ref = ref + np.array([0.0, cam_off])

    def _trajectory(self, s: Scene, t: float) -> None:
        """Локализация в систему текущего кадра — через относительное смещение.

        Абсолютная точность позы не нужна, только относительная: дрейф ENU за минуту картинку не портит.
        """
        i = self.at_pose(t)
        if i is None or len(self.p_t) < 3:
            return
        sel = (self.p_t >= t - PAST_S * 1000.0) & (self.p_t <= t + FUTURE_S * 1000.0)
        if sel.sum() < 2:
            return
        d = self.p_xy[sel] - self.p_xy[i]
        c, sn = np.cos(-self.p_yaw[i]), np.sin(-self.p_yaw[i])
        rel = np.column_stack([c * d[:, 0] - sn * d[:, 1], -(sn * d[:, 0] + c * d[:, 1])])
        past = self.p_t[sel] <= t
        s.past, s.future = rel[past], rel[~past]


class View:
    """Отрисовка сцены. Машина смотрит вправо: изображение повёрнуто на 90° по часовой."""

    BG, GRID, FG = "#111318", "#2a2f3a", "#e8eaed"

    # Слои: атрибут сцены, цвет, толщина, штрих, порядок, подпись. Таблицей, потому что все они рисуются
    # одинаково и различаются только оформлением — а различия должны быть видны рядом друг с другом.
    LAYERS = (
        ("past", "#7f8c9b", 1.4, "-", 2, "локализация: прошлое"),
        ("center", "#8bc34a", 1.6, (0, (6, 4)), 4, "центр полосы"),
        ("plan", "#ba68c8", 1.6, ":", 4, "план модели"),
        ("ref", "#ffb300", 2.6, "-", 5, "опора контроллера"),
        ("arc", "#ef5350", 1.4, "-.", 5, "дуга команды"),
    )

    def __init__(self, extent: tuple[float, float, float, float], max_px: int = 1600):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        self.plt = plt
        self.x0, self.x1, self.y0, self.y1 = extent
        span_x, span_y = self.x1 - self.x0, self.y1 - self.y0
        # Масштаб один на обе оси, иначе дуга перестанет быть дугой. Ограничен сверху, чтобы дальняя
        # разметка не раздувала файл до неприличия.
        self.px_per_m = min(max_px / span_x, max_px / span_y, 30.0)
        self.w = 2 * int(
            round(span_x * self.px_per_m / 2)
        )  # чётные размеры: их любит кодек
        self.h = 2 * int(round(span_y * self.px_per_m / 2))
        self.fig = plt.figure(figsize=(self.w / 100.0, self.h / 100.0), dpi=100)
        self.ax = self.fig.add_axes([0, 0, 1, 1])

    @staticmethod
    def xy(pts: Optional[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
        """(x вперёд, y вправо) → экран после поворота на 90° по часовой: вперёд направо, y вверх."""
        if pts is None or len(pts) == 0:
            return np.empty(0), np.empty(0)
        return pts[:, 0], pts[:, 1]

    def draw(self, s: Scene) -> np.ndarray:
        from matplotlib.patches import Rectangle

        ax = self.ax
        ax.clear()
        ax.set_facecolor(self.BG)
        ax.set_xlim(self.x0, self.x1)
        ax.set_ylim(self.y0, self.y1)
        ax.set_xticks([])
        ax.set_yticks([])
        for g in range(-60, 61, 10):
            ax.axhline(g, color=self.GRID, lw=0.6, zorder=0)
            ax.axvline(g, color=self.GRID, lw=0.6, zorder=0)

        # Линии разметки: прозрачность несёт доверие к σ — обнулённую линию почти не видно.
        for pts, conf, name in (
            (s.left, s.conf[0], "линии модели"),
            (s.right, s.conf[1], None),
        ):
            ax.plot(
                *self.xy(pts),
                color="#4fc3f7",
                lw=2.0,
                alpha=0.20 + 0.80 * conf,
                zorder=3,
                label=name,
            )
        for attr, colour, lw, ls, z, name in self.LAYERS:
            ax.plot(
                *self.xy(getattr(s, attr)),
                color=colour,
                lw=lw,
                ls=ls,
                zorder=z,
                label=name,
            )
        # Маркерами: на прямой будущая траектория ложится ровно на опору, а расхождение этих двух и есть
        # предмет осмотра.
        ax.plot(
            *self.xy(s.future),
            color="#ffffff",
            lw=1.2,
            ls="--",
            marker="o",
            markersize=2.2,
            markevery=6,
            markeredgecolor="none",
            zorder=6,
            label="локализация: будущее",
        )

        ax.add_patch(
            Rectangle(
                (-REAR_OVERHANG, -CAR_W / 2.0),
                CAR_L,
                CAR_W,
                facecolor="#43a047" if s.assist else "#616161",
                edgecolor=self.FG,
                lw=1.2,
                alpha=0.85,
                zorder=7,
            )
        )
        ax.text(
            0.012,
            0.988,
            s.panel,
            transform=ax.transAxes,
            va="top",
            ha="left",
            family="monospace",
            fontsize=7.2,
            color=self.FG,
            bbox=dict(facecolor="#0b0d11", edgecolor=self.GRID, alpha=0.85, pad=4),
        )
        ax.legend(
            loc="lower right",
            fontsize=7,
            framealpha=0.85,
            facecolor="#0b0d11",
            edgecolor=self.GRID,
            labelcolor=self.FG,
        )

        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3]

    def close(self) -> None:
        self.plt.close(self.fig)


class Writer:
    """Видео через OpenCV. Кодек выбирается проверкой, а не надеждой.

    В сборке OpenCV на этом хосте H.264 нет вовсе — `avc1`/`H264`/`X264` даже не открываются, — а `mp4v` и
    `XVID` пишутся с тегом `FMP4`, и плееры на такой файл отвечают «file contains no playable streams».
    Корректный тег даёт только MJPG, он же играется везде. Порядок попыток начинается с `avc1`, чтобы на
    хосте с полным ffmpeg вышел компактный H.264, а заканчивается MJPG как заведомо рабочим.

    После закрытия файл перечитывается: ноль кадров — это ошибка, а не повод отдать битое видео.
    """

    CODECS = (("avc1", ".mp4"), ("MJPG", ".avi"))

    def __init__(self, path: Path, fps: float, size: tuple[int, int]):
        import cv2

        self.cv2, self.fps, self.size = cv2, fps, size
        for codec, ext in self.CODECS:
            self.path = path.with_suffix(ext)
            self.w = cv2.VideoWriter(
                str(self.path), cv2.VideoWriter_fourcc(*codec), fps, size
            )
            if self.w.isOpened():
                self.codec = codec
                return
        raise SystemExit("ни один кодек не открылся на запись")

    def write(self, rgb: np.ndarray) -> None:
        self.w.write(self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR))

    def close(self) -> int:
        self.w.release()
        cap = self.cv2.VideoCapture(str(self.path))
        n = int(cap.get(self.cv2.CAP_PROP_FRAME_COUNT))
        tag = int(cap.get(self.cv2.CAP_PROP_FOURCC))
        cap.release()
        name = "".join(chr((tag >> (8 * i)) & 0xFF) for i in range(4))
        if n == 0:
            raise SystemExit(f"{self.path}: записалось 0 кадров — файл непригоден")
        if name == "FMP4":
            raise SystemExit(
                f"{self.path}: кодек записался тегом FMP4, плееры его не примут"
            )
        return n


def list_assist(bag: Bag) -> None:
    """Интервалы, где рулить было разрешено, по кривизне команды.

    По `panda/health` (10 Гц, крошечный топик), а не по кадрам на шине: просмотр `can/rx` за час занял бы
    минуты, а гейт всё равно решает он. Флаг берётся через `lateral_actuation_on`, а не напрямую из
    `controls_allowed`: при включённом `lat_always_on` панда пропускает момент, когда `controls_allowed`
    ложен, и фильтр по нему выбросил бы ровно те кадры, где ассист работал.
    """
    t0 = bag.start_ms()
    health = bag.read("panda/health")
    dbg = bag.read("control/lane_keep_debug")
    if t0 is None or not health:
        raise SystemExit("нужны vision/lanes и panda/health")
    d_t, d_k = np.asarray([r[0] for r in dbg], float), np.abs(col(dbg, "mpc_kappa_used"))
    d_v = col(dbg, "speed_mps")

    h_t = np.asarray([r[0] for r in health], float)
    on = lateral_actuation_on(health)
    edges = np.flatnonzero(np.diff(on.astype(int)))
    bounds = np.r_[0, edges + 1, len(on)]
    rows = []
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        if not on[lo] or h_t[hi - 1] - h_t[lo] < 4000:
            continue
        sel = (d_t >= h_t[lo]) & (d_t <= h_t[hi - 1])
        rows.append(
            (
                (h_t[lo] - t0) / 1000.0,
                (h_t[hi - 1] - h_t[lo]) / 1000.0,
                float(np.median(d_v[sel])) if sel.any() else 0.0,
                float(d_k[sel].max()) if sel.any() else 0.0,
            )
        )

    print(
        f"интервалов длиннее 4 с: {len(rows)}\n  --start   длит.   скорость   макс |κ|   радиус"
    )
    for st, dur, v, k in sorted(rows, key=lambda r: -r[3])[:15]:
        print(
            f"  {st:8.1f}  {dur:5.1f} с  {v:5.1f} м/с   {k:.5f}   "
            f"{f'{1.0 / k:.0f} м' if k > 1e-6 else 'прямая'}"
        )


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--start", type=float, default=0.0, help="секунд от начала бага")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument(
        "--margin",
        type=float,
        default=MARGIN_M,
        help="запас вокруг линий разметки с каждой стороны, м",
    )
    ap.add_argument(
        "--ahead", type=float, default=40.0, help="насколько смотреть вперёд, м"
    )
    ap.add_argument("--fps", type=float, default=12.0)
    ap.add_argument("--std-range", type=float, default=pf.DEFAULT_LANE_STD_RANGE_M)
    ap.add_argument("--list-assist", action="store_true")
    args = ap.parse_args()

    bag = Bag(args.bag)
    if args.list_assist:
        list_assist(bag)
        return 0

    scenes = Scenes(bag, args.start, args.duration, args.std_range)
    print(
        f"{args.bag.name}: {args.start:.0f}…{args.start + args.duration:.0f} с, кадров {len(scenes)}"
    )
    if not len(scenes):
        raise SystemExit("в окне нет кадров модели — проверь --start")

    extent = scenes.extent(args.margin, args.ahead)
    view = View(extent)
    print(
        f"  окно {extent[1] - extent[0]:.0f}×{extent[3] - extent[2]:.0f} м "
        f"({view.w}×{view.h} px, {view.px_per_m:.1f} px/м)"
    )
    out = args.out or (args.bag / f"topdown_{int(args.start)}s_{int(args.duration)}s.mp4")
    writer = Writer(out, args.fps, (view.w, view.h))

    n_assist = 0
    for s in scenes:
        writer.write(view.draw(s))
        n_assist += s.assist
    n = writer.close()
    view.close()
    print(
        f"готово: {writer.path} ({writer.codec}, {n} кадров, "
        f"{writer.path.stat().st_size / 1e6:.1f} МБ)"
    )
    print(f"  ассист был включён в {100 * n_assist / max(len(scenes), 1):.0f}% кадров")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
