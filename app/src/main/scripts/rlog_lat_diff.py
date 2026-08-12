#!/usr/bin/env python3
"""Прогон записи openpilot/dragonpilot через *наш* поперечный стек и разница команд.

Самая жёсткая проверка, доступная порту: та же машина, та же дорога, та же опорная линия, тот же
измеренный угол руля — отличается только код, который превращает это в команду. Синтетика говорит,
что код делает написанное; это говорит, делает ли он то же, что апстрим.

Реплей идёт через middleware, а не через внутренние классы: `AdasApp` — это всё приложение,
публикуем в те же топики, что телефон, зовём `step()`, читаем что вышло. Намеренно: middleware
существует, чтобы скрывать реализацию, и сравнение, залезающее внутрь, мерило бы придуманную для
теста схему, а не ту, по которой едет машина.

## Два режима эталона

`--reference plan` (по умолчанию) подаёт `lateralPlan.dPathPoints`: их готовую опору, УЖЕ после их
смешивания разметки, смещения камеры и центрирующего члена. Их `y_pts` сняты на `v_ego · t_idxs`,
поэтому x восстанавливаются так же. Перцепция и фьюжн исключены нарочно, и наши смешивание,
смещение и центрирование выключены по той же причине — иначе сравнивались бы две разные уставки.
Под тестом остаётся: путь → кривизна → угол → момент.

`--reference model` подаёт `modelV2`: их линии разметки, края дороги и план как `LaneLines`, так что
**наш** `laneLinesToPath` работает на их выходе модели. Это расширяет сравнение на стадию вверх —
наши смешивание, гейт по ширине, гейт по σ и смещение камеры. Две оговорки к его числам:

* их `laneLineStds` — **один скаляр на линию**, наш — поточечный `y_std`, чью медиану на 5–20 м
  читает гейт по σ. Скаляр размножается на все точки, то есть режим гоняет наш гейт на ИХ
  определении σ. Это половина задачи #23, но это не та величина, которую даёт наша модель;
* их знак поперечной оси совпадает с нашим (измерено: на уверенном кадре ближние линии стоят на
  −1.64 и +1.82 м, то есть y положителен вправо, как и написано в `lanes.proto`), поэтому полилинии
  переносятся без переворота.

## Что сравнивается и почему не угол руля в первую очередь

Четыре величины, в порядке движения сигнала. Держать их раздельно — весь смысл: расхождение в
планировании, в модели машины и в контроллере лечатся по-разному.

1. **кривизна** — их `controlsState.desiredCurvature` против нашей `LaneKeepOutput.curvature`:
   собственный выход планировщика с обеих сторон, до того как передаточное или снос тронут его;
2. **уставной угол** — их `actuators.steeringAngleDeg` против нашего. Кривизна → угол идёт через
   модель машины каждой стороны, поэтому здесь проявляется расхождение по `steer_ratio` или
   `tire_stiffness_factor`, а не по планированию;
3. **момент до ограничителя** — их `actuators.steer · STEER_MAX` против `SteerCommand.torque_cnm`;
4. **момент после ограничителя** — их `actuatorsOutput.steerOutputCan` (уже в cNm) против нашей
   команды через `applyDriverSteerTorqueLimits` на сетке 20 мс, как делает `CarController` при
   `STEER_STEP = 2`. Сравнение только (3) льстит обеим сторонам: темповый предел 200 cNm/с
   превращает запрошенные 300 в медианные 187 приложенных, и ограничитель — то место, где команда
   обнуляется при каждой потере ассиста.

**Их уставной угол годен не везде, и поэтому существует (1).** Измерено на этом заезде:
`actuators.steeringAngleDeg` доходит до **649.7°** и держит p90 **130.8°** даже на кадрах с
`latActive`, потому что на малой скорости план длиной несколько метров даёт большую кривизну, а их
модель машины превращает её в угол, которого нет ни у одной рейки. `desiredCurvature` на тех же
кадрах остаётся физическим — p90 0.049 1/м, то есть радиус 20 м. Поэтому сравнение угла закрыто
гейтом `|их угол| ≤ max_steer_deg · steer_ratio`: это наибольшая уставка, которую вообще может выдать
наш планировщик, за ней мы зажаты по построению и сравнение мерило бы наш зажим. Сколько кадров это
убирает — печатается, а не скрывается.

Ограничитель берётся из биндинга, а не из копии на python: асимметричная логика вверх/вниз
существует в одном экземпляре. Дрожание для живости EPS (±1 cNm каждые 1.9 с) **не** моделируется —
это трюк живости, а не управление.

## Единственная несправедливость, которую нельзя убрать, только учесть

**Реплей разомкнут, и это смещает сравнение момента, но не остальное.** Измеренный угол руля взят из
их лога, то есть это угол, который создал ИХ момент. Наша команда руль не двигает, поэтому остаток
ошибки наш интегратор копит, а их — копил, активно закрывая ту же ошибку. Измерено на 102 000 кадрах:
наша уставка угла совпадает с их до медианных **0.08°** при корреляции **0.965**, а момент до
ограничителя стоит на медиане **229 cNm против их 69** и упирается в ±300 на 35 % кадров против их
10 %. Две стадии, совпадающие так близко, не могут дать трёхкратное расхождение момента —
интегратор может, и даёт.

Поэтому `--no-integrator` гоняет наш PID с `ki = 0`: это сравнение мгновенного отклика на ту же
ошибку и единственная версия сравнения момента, которую стоит читать. По умолчанию интеграл включён,
чтобы смещение было видно, а не тихо исправлено.

Итого для чтения: **кривизна и уставной угол достоверны, момент достоверен только с
`--no-integrator`**, а установившееся смещение здесь не отвечается вообще — для него нужен замкнутый
контур, чем и является симулятор, о котором `docs/BACKLOG.md` §3 говорит, что ранжировать
контроллеры он тоже не умеет.

Знаковые соглашения измеряются, а не предполагаются: их угол — в соглашении openpilot, наш — в
соглашении CAN с уже применённым `vehicle.steer_sign`, поэтому скрипт мерит связь, а не зашивает её.

  OPENPILOT_ROOT=/path/to/openpilot python3 rlog_lat_diff.py <route dir> [<route dir> ...]
  python3 rlog_lat_diff.py <parent of routes> --reference model
  python3 rlog_lat_diff.py <route> --segments 6 --controller fp --steer-ratio 16.12
"""

from __future__ import annotations

import argparse
import bz2
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import _path  # noqa: F401

# Сетка времени модели openpilot. Первые LAT_MPC_N+1 — то, на чём снят `dPathPoints`; зашита, а не
# импортируется: стенд должен работать и когда их дерева нет на пути.
T_IDXS = np.array(
    [
        0.0,
        0.00976562,
        0.0390625,
        0.08789062,
        0.15625,
        0.24414062,
        0.3515625,
        0.47851562,
        0.625,
        0.79101562,
        0.9765625,
        1.18164062,
        1.40625,
        1.65039062,
        1.9140625,
        2.19726562,
        2.5,
        2.82226562,
        3.1640625,
        3.52539062,
        3.90625,
        4.30664062,
        4.7265625,
        5.16601562,
        5.625,
        6.10351562,
        6.6015625,
        7.11914062,
        7.65625,
        8.21289062,
        8.7890625,
        9.38476562,
        10.0,
    ]
)
LAT_MPC_N = 16
LIMITER_DT_S = 0.020  # STEER_STEP = 2 на таймере 10 мс
ALIGN_TOL_MS = 30.0
SPEED_BINS = ((0, 5), (5, 10), (10, 15), (15, 20), (20, 30))
TORQUE_BINS = ((0, 50), (50, 150), (150, 250), (250, 299), (299, 1000))
MIN_ROUTE, MIN_POOL, MIN_BIN = 50, 200, 30


def med(x):
    return float(np.median(np.abs(x)))


def p90(x):
    return float(np.percentile(np.abs(x), 90))


def sat(x):
    """Доля кадров на потолке момента, %."""
    return 100.0 * float(np.mean(np.abs(x) >= 299))


def arr(seq):
    return np.asarray(list(seq), dtype=np.float64)


class Series:
    """Одна сравниваемая величина: наша против их, со снятым знаковым соглашением.

    Знак снимается корреляцией, а не зашивается: их угол в соглашении openpilot, наш — в соглашении
    CAN с уже применённым `steer_sign`.
    """

    def __init__(self, ours, theirs):
        self.corr = float(np.corrcoef(ours, theirs)[0, 1]) if len(ours) > 2 else 0.0
        self.ours = ours * (-1.0 if self.corr < 0 else 1.0)
        self.theirs = theirs
        self.diff = self.ours - theirs

    def __len__(self):
        return len(self.ours)

    @property
    def slope(self):
        return float(np.polyfit(self.theirs, self.ours, 1)[0])

    def bin(self, values, lo, hi):
        """Подвыборка по внешней величине — скорости или их же моменту."""
        return (np.abs(values) >= lo) & (np.abs(values) < hi)


@dataclass
class Frames:
    """Что вышло из одного маршрута. Столбцы названы там, где читаются."""

    ours: np.ndarray  # t, swa, torque, on, v, driver_tq, curv
    theirs: np.ndarray  # t, swa, steer_norm, applied, on
    curv: np.ndarray  # t, curvature
    ratio: float
    refs: int

    def __bool__(self):
        return len(self.ours) > 0 and len(self.theirs) > 0


class RouteLog:
    """Чтение одного маршрута в события, терпя битый или обрезанный хвост.

    Оба отказа нормальны для записей и оба раньше валили весь прогон: оборванная запись оставляет
    `rlog.bz2`, распаковывающийся частично, и поток capnp, кончающийся посреди сообщения. Тихо
    укороченный маршрут — это как раз то, из-за чего свип заявляет покрытие, которого у него нет,
    поэтому потери попадают в `damaged`, а не в тишину.
    """

    def __init__(self, route: Path, segments: int | None, want_model: bool):
        self.route = route
        self.want_model = want_model
        self.car: dict | None = None
        self.damaged: list[str] = []
        self.segs = sorted(
            (p for p in route.iterdir() if p.is_dir()), key=lambda p: int(p.name)
        )[: segments or None]
        self.events = sorted(self._read_all(), key=lambda e: e[0])

    @staticmethod
    def _cereal():
        # cereal живёт в чужом дереве (openpilot/dragonpilot), путь к нему у каждого свой.
        root = os.environ.get("OPENPILOT_ROOT")
        if not root:
            raise SystemExit(
                "нужен OPENPILOT_ROOT — путь к дереву openpilot/dragonpilot, откуда cereal:\n"
                "  OPENPILOT_ROOT=/path/to/openpilot python3 rlog_lat_diff.py <route dir>"
            )
        sys.path.insert(0, root) if root not in sys.path else None
        from cereal import log

        return log

    @staticmethod
    def _unpack(path: Path):
        """Распаковка по кускам: `bz2.decompress` целиком поднимает OSError на битом файле."""
        raw, d, out, lost = path.read_bytes(), bz2.BZ2Decompressor(), b"", False
        for k in range(0, len(raw), 1 << 18):
            try:
                out += d.decompress(raw[k : k + (1 << 18)])
            except (OSError, EOFError):
                lost = True
                break
            if d.eof:
                break
        return out, lost

    def _segment(self, path: Path, capnp_log):
        data, lost = self._unpack(path)
        if not data:
            return [], "не распаковался"
        evts, note = [], ""
        try:
            # По одному, а не `list(...)`: на обрезанном потоке исключение придёт посреди чтения, и
            # прочитанное до него — годные кадры, которые нельзя терять вместе с хвостом.
            for evt in capnp_log.Event.read_multiple_bytes(data):
                evts.append(evt)
        except Exception:
            note = "capnp обрезан"
        if lost:
            note = (
                note + ", " if note else ""
            ) + f"bz2 битый, взято {len(data)/1e6:.1f} МБ"
        return evts, note

    def _read_all(self):
        capnp_log = self._cereal()
        for seg in self.segs:
            path = seg / "rlog.bz2"
            if not path.exists():
                continue
            evts, note = self._segment(path, capnp_log)
            if note:
                self.damaged.append(f"{seg.name}: {note}")
            for evt in evts:
                got = self._event(evt)
                if got:
                    yield got

    def _event(self, evt):
        w, t = evt.which(), evt.logMonoTime * 1e-6
        if w == "carParams":
            cp = evt.carParams
            self.car = self.car or {
                "fingerprint": str(cp.carFingerprint),
                "wheelbase": float(cp.wheelbase),
                "steer_ratio": float(cp.steerRatio),
            }
        elif w == "carState":
            cs = evt.carState
            # `steeringTorque` — момент водителя, и он не украшение: он расширяет зажим ограничителя
            # на STEER_DRIVER_ALLOWANCE, поэтому ноль здесь сделал бы наш ограниченный поток строже
            # того, который выдала машина.
            return (
                t,
                "car",
                (
                    float(cs.vEgo),
                    float(cs.steeringAngleDeg),
                    bool(cs.steeringPressed),
                    float(getattr(cs, "yawRate", 0.0)),
                    float(cs.steeringTorque),
                ),
            )
        elif w == "lateralPlan" and not self.want_model:
            lp = evt.lateralPlan
            return t, "plan", (arr(lp.dPathPoints), arr(lp.psis), arr(lp.curvatures))
        elif w == "modelV2" and self.want_model:
            mv = evt.modelV2
            return (
                t,
                "model",
                {
                    "x": arr(mv.laneLines[1].x),
                    "lanes": [arr(ln.y) for ln in mv.laneLines],
                    "probs": arr(mv.laneLineProbs),
                    "stds": arr(mv.laneLineStds),
                    "edges": [arr(e.y) for e in mv.roadEdges],
                    "plan_x": arr(mv.position.x),
                    "plan_y": arr(mv.position.y),
                    "plan_z": arr(mv.position.z),
                    "plan_yaw": arr(mv.orientation.z),
                    "plan_yaw_rate": arr(mv.orientationRate.z),
                },
            )
        elif w == "controlsState":
            # Держится отдельно от `carControl`: это единственная их поперечная величина,
            # остающаяся физической на малой скорости.
            return t, "curv", float(evt.controlsState.desiredCurvature)
        elif w == "carControl":
            cc = evt.carControl
            out = cc.actuatorsOutput
            # `steerOutputCan` — ограниченное значение в cNm; откат нужен для логов без поля.
            applied = float(getattr(out, "steerOutputCan", out.steer * 300.0))
            return (
                t,
                "cmd",
                (
                    float(cc.actuators.steeringAngleDeg),
                    float(cc.actuators.steer),
                    applied,
                    bool(cc.latActive),
                ),
            )
        return None


class Harness:
    """Приложение, настроенное как ОТГРУЖАЕТСЯ, и прогон маршрута через него.

    Дефолты конструктора — не та машина, по которой едет телефон: `max_steer_deg` там 8, а в
    `assets/config.json` 20, контроллер там `pp`, а в файле `fp`. Реплей на дефолтах мерит машину,
    которой не существует — первый прогон этого скрипта держал наш момент в упоре на 85 % кадров
    только потому, что 8 градусов колеса нормируются в 1.0 в два с половиной раза раньше.
    """

    # Знобы оценщика параметров машины, ровно как они названы в `localization` конфига. Без них
    # оценщик в реплее — не тот, что на машине: при дефолтном `params_stiffness_p0_std = 0` жёсткость
    # стоит там, где стартовала (P = Q), и «включённый оценщик» ничего не учит.
    LEARNER_KEYS = (
        "learn_vehicle_params",
        "params_angle_offset_init_deg",
        "params_stiffness_p0_std",
        "params_stiffness_process_std",
        "params_steer_ratio_process_std",
        "params_min_speed_ms",
        "params_max_lateral_jerk",
        "params_max_roll_std_deg",
        "params_use_roll",
    )

    def __init__(self, core, veh: dict, loc: dict, args):
        self.core = core
        self.veh = veh
        self.loc = loc
        self.args = args
        self.model_mode = args.reference == "model"

    def recompute(self):
        """Пересчёт уставки между кадрами: флаг ИЛИ конфиг.

        Читать только флаг значило бы мерить не то, что поедет: отгружаемый конфиг везёт
        `lat_recompute_setpoint: true`, а пересчёт правит уставку на каждом тике шасси, то есть на
        100 Гц вместо 16.
        """
        return bool(
            self.args.recompute_setpoint or self.veh.get("lat_recompute_setpoint", False)
        )

    def shipped_controller(self):
        """Контроллер вместе с численным методом, как задано в конфиге.

        `fp_solver` — не отдельный параметр реестра, он выбирается через имя контроллера
        `fp_acados`. Без этого стенд мерил бы `grad` (дефолт `forSimulated`), тогда как отгружаемый
        конфиг везёт `acados` — то есть сравнивал бы решатель, которым машина не едет.
        """
        ctrl = self.veh.get("lane_keep_controller", "fp")
        if ctrl == "fp" and self.veh.get("fp_solver") == "acados":
            return "fp_acados"
        return ctrl

    def _build(self, car):
        a, veh, args = self.core, self.veh, self.args
        app = a.AdasApp(
            car["wheelbase"] if car else 2.62,
            -1.8,
            0.5,
            1.10,
            topic_convert=self.model_mode,
        )
        app.set_lane_keep_controller(args.controller or self.shipped_controller())
        app.set_lane_keep_max_steer_deg(float(veh.get("max_steer_deg", 20.0)))
        app.set_lane_keep_steer_slew_limit_deg(
            float(veh.get("steer_slew_limit_deg", 8.0))
        )
        stiff = args.stiffness or float(veh.get("tire_stiffness_factor", 0.64))
        app.set_lane_keep_vehicle_model(
            bool(veh.get("lat_use_vehicle_model", True)), stiff
        )
        app.set_lane_keep_fp_steer_delay_s(float(veh.get("fp_steer_delay_s", 0.35)))
        app.set_lane_keep_recompute_setpoint(self.recompute())
        if args.no_integrator:
            # См. «единственную несправедливость» в шапке: без обратной связи наш интегратор
            # наматывает ошибку, на которую не влияет.
            app.set_lane_keep_pid_gains(
                float(veh.get("pid_kp", 0.6)), 0.0, float(veh.get("pid_kf", 6e-5))
            )
        ratio = args.steer_ratio or (
            car["steer_ratio"] if car else float(veh["steer_ratio"])
        )
        app.set_param("steer_ratio", ratio)
        self._apply_fusion(app)
        self._apply_learner(app)
        return app, ratio

    def _apply_learner(self, app):
        """Оценщик параметров машины и разрешение контроллеру его читать — как в конфиге.

        `forSimulated` держит оценщик выключенным, поэтому по дефолтам стенд мерил бы контроллер на
        константах, тогда как отгружаемый конфиг везёт `use_learned_params: true`.
        """

        def put(name, value):
            text = str(value).lower() if isinstance(value, bool) else repr(float(value))
            app.set_param_str(name, text)

        put("use_learned_params", bool(self.veh.get("use_learned_params", False)))
        for key in self.LEARNER_KEYS:
            if key in self.loc:
                put(key, self.loc[key])

    def _apply_fusion(self, app):
        if not self.model_mode:
            # Их путь уже готовая опора; наш не должен добавлять поверх второе смещение камеры.
            app.set_lane_keep_cam_y_left_m(0.0)
            return
        # Здесь работает наш фьюжн, значит он обязан работать с отгружаемыми числами.
        for k in (
            "path_lane_blend_scale",
            "path_camera_offset_m",
            "center_force_gain",
            "lane_std_good_m",
            "lane_std_bad_m",
            "lane_std_range_m",
        ):
            if k in self.veh:
                app.set_param(k, float(self.veh[k]))
        app.set_param("cam_y_left_m", float(self.veh.get("cam_y_left_m", 0.0)))

    def _lane_lines(self, m, ts_ms, frame_id):
        """Их `modelV2` как наш `LaneLines`. Поточечный `y_std` — размноженный их скаляр."""
        import lanes_pb2

        ll = lanes_pb2.LaneLines()
        ll.timestamp = ll.capture_ts_ms = ts_ms
        ll.frame_id = frame_id
        ll.x.extend(m["x"].tolist())
        for y, prob, std in zip(m["lanes"], m["probs"], m["stds"]):
            p = ll.lanes.add()
            p.y.extend(y.tolist())
            p.prob = float(prob)
            p.y_std.extend([float(std)] * len(y))
        for y in m["edges"]:
            ll.edges.add().y.extend(y.tolist())
        for name in ("plan_x", "plan_y", "plan_z", "plan_yaw", "plan_yaw_rate"):
            getattr(ll, name).extend(m[name].tolist())
        ll.plan_hyp = 0
        return ll.SerializeToString()

    def run(self, log: RouteLog) -> Frames:
        app, ratio = self._build(log.car)
        t0 = log.events[0][0]
        v_ego = driver_tq = 0.0
        swa = curv = float("nan")
        ours, theirs, their_curv, refs = [], [], [], 0

        for t, kind, payload in log.events:
            ts_us = int((t - t0) * 1000)
            if kind == "car":
                v_ego, ang, pressed, yaw, driver_tq = payload
                app.publish_chassis(ts_us, v_ego, 0.0, yaw, ang, pressed)
            elif kind == "plan":
                refs += self._publish_plan(app, ts_us, payload, v_ego, refs)
            elif kind == "model":
                app.publish_lane_lines(self._lane_lines(payload, ts_us // 1000, refs))
                refs += 1
            elif kind == "cmd":
                theirs.append((t - t0, *payload))
            elif kind == "curv":
                their_curv.append((t - t0, payload))

            app.step(ts_us)
            for msg in app.pop_messages():
                if isinstance(msg, self.core.LaneKeepOutput):
                    # `desired_swa_deg` заполняется только на пути шасси, поэтому уставка
                    # восстанавливается из `steer_rad`, который заполняет путь зрения:
                    # SWA = угол колеса x передаточное. Арифметика над опубликованным полем,
                    # а не подглядывание внутрь сервиса.
                    deg = msg.steer_rad * 180.0 / np.pi
                    swa = deg * ratio if msg.has_target else np.nan
                    curv = msg.curvature if msg.has_target else np.nan
                elif isinstance(msg, self.core.SteerCommand):
                    ours.append(
                        (
                            ts_us / 1000.0,
                            swa,
                            float(msg.torque_cnm),
                            float(msg.enabled),
                            v_ego,
                            driver_tq,
                            curv,
                        )
                    )
        app.stop()
        return Frames(
            self._table(ours, 7),
            self._table(theirs, 5),
            self._table(their_curv, 2),
            ratio,
            refs,
        )

    @staticmethod
    def _publish_plan(app, ts_us, payload, v_ego, refs):
        dpath, psis, curvs = payload
        n = min(len(dpath), LAT_MPC_N + 1, len(T_IDXS))
        if n < 6:
            return 0
        v = max(v_ego, 0.1)
        poly = [(float(x), float(y)) for x, y in zip(T_IDXS[:n] * v, dpath[:n])]
        yaws = [float(p) for p in psis[:n]] if len(psis) >= n else []
        rates = [float(c) * v for c in curvs[:n]] if len(curvs) >= n else []
        app.publish_lanes(ts_us, poly, refs, poly, yaws, rates, True)
        return 1

    @staticmethod
    def _table(rows, width):
        return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, width))


class Match:
    """Сопоставление одного маршрута по времени: маски годных кадров и счётчики отброшенного."""

    def __init__(self, core, f: Frames, min_speed: float, angle_ceiling: float):
        self.f = f
        self.i, dt_ok = self.align(f.ours, f.theirs)
        both_on = (f.ours[:, 3] > 0.5) & (f.theirs[self.i, 4] > 0.5)
        self.fast = f.ours[:, 4] > min_speed
        self.pre = dt_ok & both_on & self.fast

        # После ограничителя — на собственной сетке актюатора 20 мс.
        self.limited = self.limit(core, f.ours)
        self.li, ldt_ok = self.align(self.limited, f.theirs)
        self.post = (
            ldt_ok
            & (self.limited[:, 2] > 0.5)
            & (f.theirs[self.li, 4] > 0.5)
            & (self.limited[:, 3] > min_speed)
        )

        # Кривизна: их планировщик против нашего, на их собственном потоке событий, а не нашем.
        self.ci, cdt_ok = self.align(f.ours, f.curv)
        self.curv = self.pre & cdt_ok & np.isfinite(f.ours[:, 6])

        # Угол: только там, где их уставка вообще достижима нашим планировщиком.
        finite = self.pre & np.isfinite(f.ours[:, 1])
        wild = np.abs(f.theirs[self.i, 1]) > angle_ceiling
        self.ang = finite & ~wild
        self.drops = {
            "slow": int((dt_ok & both_on & ~self.fast).sum()),
            "off": int((dt_ok & ~both_on).sum()),
            "align": int((~dt_ok).sum()),
            "angle_wild": int((finite & wild).sum()),
        }

    @staticmethod
    def align(a, t, tol_ms=ALIGN_TOL_MS):
        """Индекс ближайшей строки `их` для каждой строки `a` и маска годности."""
        if len(a) == 0 or len(t) == 0:
            return np.zeros(len(a), dtype=int), np.zeros(len(a), dtype=bool)
        i = np.clip(np.searchsorted(t[:, 0], a[:, 0]), 0, len(t) - 1)
        j = np.clip(i - 1, 0, len(t) - 1)
        i = np.where(np.abs(t[j, 0] - a[:, 0]) < np.abs(t[i, 0] - a[:, 0]), j, i)
        return i, np.abs(t[i, 0] - a[:, 0]) <= tol_ms

    @staticmethod
    def limit(core, o):
        """Наш поток таким, каким его получила бы рейка: удержан на сетке 20 мс и прогнан через
        ограничитель MQB, как делает `CarController` при `frame_ % STEER_STEP == 0`.

        Удержание — не выбор сглаживания, а то, что делает таймер актюатора: он подхватывает
        последнюю опубликованную команду. Не lat_active означает apply_steer = 0 *и* перезапуск
        разгона с нуля, потому что `apply_steer_last_` присваивается безусловно.
        """
        if len(o) == 0:
            return np.zeros((0, 4))
        grid = np.arange(o[0, 0], o[-1, 0], LIMITER_DT_S * 1000.0)
        idx = np.clip(np.searchsorted(o[:, 0], grid, side="right") - 1, 0, len(o) - 1)
        out, last = np.empty((len(grid), 4)), 0
        for k, i in enumerate(idx):
            on = o[i, 3] > 0.5
            last = (
                core.apply_driver_steer_torque_limits(int(round(o[i, 2])), o[i, 5], last)
                if on
                else 0
            )
            out[k] = (grid[k], last, o[i, 3], o[i, 4])
        return out

    def stats(self, name, segs, car):
        return {
            "name": name,
            "segs": segs,
            "refs": self.f.refs,
            "ours": len(self.f.ours),
            "matched": int(self.pre.sum()),
            "matched_post": int(self.post.sum()),
            "matched_curv": int(self.curv.sum()),
            "matched_ang": int(self.ang.sum()),
            "fast_share": 100.0 * float(np.mean(self.fast)),
            "car": car["fingerprint"] if car else "?",
            "ratio": self.f.ratio,
            **self.drops,
        }

    def pairs(self, steer_max):
        """Сопоставленные пары (наше, их) по каждой сравниваемой величине."""
        f, i = self.f, self.i
        return {
            "pre": (self.pre, f.ours[:, 2], f.theirs[i, 2] * steer_max),
            "ang": (self.ang, f.ours[:, 1], f.theirs[i, 1]),
            "curv": (self.curv, f.ours[:, 6], f.curv[self.ci, 1]),
            "post": (self.post, self.limited[:, 1], f.theirs[self.li, 3]),
        }


@dataclass
class Pool:
    """Копилка сопоставленных пар по всем маршрутам; отдаёт `Series` на каждую величину."""

    parts: dict = field(default_factory=dict)
    speeds: list = field(default_factory=list)

    def add(self, match: Match, steer_max: float):
        for key, (mask, ours, theirs) in match.pairs(steer_max).items():
            if mask.sum() < MIN_ROUTE:
                continue
            self.parts.setdefault(key, []).append((ours[mask], theirs[mask]))
        if match.ang.sum() >= MIN_ROUTE:
            self.speeds.append(match.f.ours[match.ang, 4])

    def series(self, key) -> Series | None:
        rows = self.parts.get(key)
        if not rows:
            return None
        ours = np.concatenate([o for o, _ in rows])
        theirs = np.concatenate([t for _, t in rows])
        return Series(ours, theirs) if len(ours) >= MIN_POOL else None

    def speed(self):
        return np.concatenate(self.speeds) if self.speeds else np.zeros(0)


class Report:
    """Печать. Каждый метод — один блок вывода."""

    def __init__(self, args, veh, angle_ceiling):
        self.args = args
        self.veh = veh
        self.ceiling = angle_ceiling

    def header(self, n_routes, loc):
        if self.args.set:
            print("переопределено на этот прогон: " + ", ".join(self.args.set))
        integ = (
            "ВЫКЛ (честный момент)"
            if self.args.no_integrator
            else "вкл (момент смещён открытым контуром)"
        )
        stiff = self.args.stiffness or self.veh.get("tire_stiffness_factor")
        print(
            f"маршрутов: {n_routes}, эталон: {self.args.reference}, "
            f"контроллер {self.args.controller or self.veh.get('lane_keep_controller')}, "
            f"жёсткость {stiff}, интеграл {integ}, пересчёт уставки между кадрами "
            f"{'ВКЛ' if (self.args.recompute_setpoint or self.veh.get('lat_recompute_setpoint')) else 'выкл'}"
        )
        learn = loc.get("learn_vehicle_params", False)
        used = self.veh.get("use_learned_params", False)
        print(
            f"оценщик параметров: {'вкл' if learn else 'выкл'}, контроллер читает его: "
            f"{'ДА' if used else 'нет'}, крен в оценщике: "
            f"{'вкл' if loc.get('params_use_roll') else 'выкл'}"
        )

    def routes(self, rows):
        print("\nпо маршрутам:")
        print(
            "  маршрут                                сегм  наших   v>гейта   сопост.  кривизна"
            "  угол  после огр.    отброшено: медл / выкл / дикий угол"
        )
        for r in rows:
            print(
                f"  {r['name']:38s} {r['segs']:4d} {r['ours']:6d} {r['fast_share']:8.0f}% "
                f"{r['matched']:9d} {r['matched_curv']:9d} {r['matched_ang']:5d} "
                f"{r['matched_post']:11d}    {r['slow']:6d} / {r['off']:5d} / "
                f"{r['angle_wild']:5d}"
            )
        cars = ", ".join(sorted({r["car"] for r in rows}))
        print(
            f"  машина: {cars}, передаточное {rows[0]['ratio']:.2f}, "
            f"потолок уставного угла для сравнения {self.ceiling:.0f}°"
        )

    def curvature(self, s: Series):
        print(
            f"\nкривизна, 1/м — выход планировщика до любой модели машины, n={len(s)} "
            f"(знак: корр {s.corr:+.3f})"
        )
        print(f"  |их|  медиана {med(s.theirs):.5f}  p90 {p90(s.theirs):.5f}")
        print(f"  |наш| медиана {med(s.ours):.5f}  p90 {p90(s.ours):.5f}")
        print(
            f"  наш = {s.slope:.3f} x их, корр {abs(s.corr):.3f}, "
            f"расхождение ск.кв {np.std(s.diff):.5f}"
        )

    def torque(self, title, s: Series):
        print(f"\n{title} (знак: корр {s.corr:+.3f}), n={len(s)}")
        for who, x in (("|их| ", s.theirs), ("|наш|", s.ours)):
            print(
                f"  {who} медиана {med(x):6.0f}   p90 {p90(x):6.0f}   "
                f"в упоре 300: {sat(x):5.1f}%"
            )
        print(
            f"  расхождение: медиана {np.median(s.diff):+.0f}, ск.кв {np.std(s.diff):.0f}, "
            f"p90 |·| {p90(s.diff):.0f} cNm, наклон наш/их {s.slope:.3f}"
        )

    def limiter_effect(self, pre: Series, post: Series):
        print(
            f"\n  ограничитель меняет вывод: ск.кв расхождения {np.std(pre.diff):.0f} → "
            f"{np.std(post.diff):.0f} cNm, p90 |·| {p90(pre.diff):.0f} → {p90(post.diff):.0f}"
        )
        print(
            "  (сравнение только до ограничителя льстит обеим сторонам — темповый предел "
            "срезает и запрос, и расхождение)"
        )

    def torque_bins(self, s: Series):
        print("\nпо величине их момента (до ограничителя):")
        print("  |их| cNm       n    |наш| медиана   расхождение ск.кв   упор их / наш")
        for lo, hi in TORQUE_BINS:
            b = s.bin(s.theirs, lo, hi)
            if b.sum() < MIN_BIN:
                continue
            print(
                f"  {lo:3d}-{hi:4d} {b.sum():8d}      {med(s.ours[b]):6.0f}         "
                f"{np.std(s.diff[b]):7.0f}          {sat(s.theirs[b]):4.0f}% / "
                f"{sat(s.ours[b]):4.0f}%"
            )

    def angle(self, s: Series, speeds):
        print(
            f"\nуставной угол руля на одном и том же пути, n={len(s)} "
            f"(знак: корр {s.corr:+.3f}):"
        )
        print(f"  |их|  медиана {med(s.theirs):6.2f}°  p90 {p90(s.theirs):6.2f}°")
        print(f"  |наш| медиана {med(s.ours):6.2f}°  p90 {p90(s.ours):6.2f}°")
        print(f"  наш = {s.slope:.3f} x их, корр {abs(s.corr):.3f}")
        print(
            f"  расхождение: медиана {np.median(s.diff):+.2f}°, ск.кв {np.std(s.diff):.2f}°, "
            f"p90 |·| {p90(s.diff):.2f}°"
        )
        print(
            f"  доля кадров с расхождением больше 1.67° (порога, за которым наш PID сразу "
            f"в упоре): {100 * np.mean(np.abs(s.diff) > 1.667):.0f}%"
        )
        print(
            "\n  по скорости:      n   |их| мед   |наш| мед   наклон наш/их   расхожд. ск.кв"
        )
        for lo, hi in SPEED_BINS:
            b = (speeds >= lo) & (speeds < hi)
            if b.sum() < MIN_ROUTE:
                continue
            note = "  ← сетка x = t·v вырождена, эталон короче машины" if hi <= 5 else ""
            sub = Series(s.ours[b], s.theirs[b])
            print(
                f"    {lo:2d}-{hi:2d} м/с {b.sum():7d}   {med(s.theirs[b]):6.2f}°   "
                f"{med(s.ours[b]):6.2f}°        {sub.slope:5.2f}         "
                f"{np.std(s.diff[b]):6.2f}°{note}"
            )

    def integrator_warning(self):
        print(
            "\nвнимание: интеграл включён, а контур разомкнут — момент ниже завышен нашим "
            "виндапом."
        )
        print(
            "           честное сравнение момента: --no-integrator (угол и кривизна выше "
            "от этого не зависят)"
        )


def route_dirs(paths: list[Path]) -> list[Path]:
    """Каталоги маршрутов или один родительский. Маршрут — каталог с нумерованными сегментами."""

    def is_route(p):
        return any(c.is_dir() and c.name.isdigit() for c in p.iterdir())

    out = []
    for p in (q for q in paths if q.is_dir()):
        out.extend(
            [p]
            if is_route(p)
            else sorted(c for c in p.iterdir() if c.is_dir() and is_route(c))
        )
    return out


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    add = ap.add_argument
    add("routes", type=Path, nargs="+", help="каталоги маршрутов или один родительский")
    add("--segments", type=int, default=6, help="сегментов на маршрут (0 = все)")
    add(
        "--reference",
        choices=("plan", "model"),
        default="plan",
        help="plan: их готовый dPathPoints. model: их modelV2 через наш фьюжн",
    )
    add("--controller", default=None, help="переопределить контроллер (fp | pp)")
    add("--steer-ratio", type=float, default=None, help="переопределить, напр. их 16.12")
    add(
        "--stiffness",
        type=float,
        default=None,
        help="переопределить tire_stiffness_factor",
    )
    add(
        "--min-speed",
        type=float,
        default=5.0,
        help="гейт по скорости; отброшенное печатается",
    )
    add(
        "--recompute-setpoint",
        action="store_true",
        help="пересчитывать уставку между кадрами, как апстрим на 100 Гц",
    )
    add(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="переопределить vehicle.* только на этот прогон, не трогая config.json",
    )
    add(
        "--no-integrator",
        action="store_true",
        help="наш PID с ki=0 — единственный честный способ сравнить момент в разомкнутом реплее",
    )
    return ap.parse_args()


def shipped_config(overrides):
    """Блоки `vehicle` и `localization` отгружаемого конфига плюс правки на время прогона.

    Правки живут только в этом процессе и печатаются в шапке: сравнивать варианты, переписывая
    боевой файл, — верный способ уехать на дорогу с настройкой от последнего эксперимента.
    """
    path = Path(__file__).resolve().parents[1] / "assets" / "config.json"
    cfg = json.loads(path.read_text())
    veh = cfg["vehicle"]
    for item in overrides:
        key, _, value = item.partition("=")
        veh[key.strip()] = float(value)
    return veh, cfg.get("localization", {})


def main() -> int:
    args = parse_args()
    from pyadas import core

    routes = route_dirs(args.routes)
    if not routes:
        print("не нашёл ни одного маршрута (каталог с нумерованными сегментами)")
        return 1

    veh, loc = shipped_config(args.set)
    ceiling = float(veh.get("max_steer_deg", 20.0)) * (
        args.steer_ratio or float(veh["steer_ratio"])
    )
    report = Report(args, veh, ceiling)
    report.header(len(routes), loc)

    harness = Harness(core, veh, loc, args)
    pool, rows = Pool(), []
    for route in routes:
        log = RouteLog(route, args.segments, harness.model_mode)
        for d in log.damaged:
            print(f"  {route.name} / {d}")
        if not log.events:
            print(f"  {route.name}: событий нет, пропускаю")
            continue
        frames = harness.run(log)
        if not frames:
            print(
                f"  {route.name}: наш стек не выдал команд (эталонов подано {frames.refs})"
            )
            continue
        match = Match(core, frames, args.min_speed, ceiling)
        rows.append(match.stats(route.name, len(log.segs), log.car))
        pool.add(match, core.STEER_MAX)

    if not rows:
        print("ни один маршрут не дал данных")
        return 1
    report.routes(rows)

    pre = pool.series("pre")
    if pre is None:
        print("\nмало сопоставленных кадров для выводов")
        return 0

    curv = pool.series("curv")
    if curv:
        report.curvature(curv)

    if not args.no_integrator:
        report.integrator_warning()
    report.torque("момент ДО ограничителя, cNm", pre)
    post = pool.series("post")
    if post:
        report.torque("момент ПОСЛЕ ограничителя, cNm — то, что доходит до рейки", post)
        report.limiter_effect(pre, post)
    else:
        print("\nпосле ограничителя сопоставить не удалось — мало кадров")
    report.torque_bins(pre)

    # Половина про планировщик решает, значит ли что-нибудь сравнение момента выше: контроллер,
    # которому дали другую уставку, разойдётся как угодно верно его ни портировали.
    ang = pool.series("ang")
    if ang:
        report.angle(ang, pool.speed())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
