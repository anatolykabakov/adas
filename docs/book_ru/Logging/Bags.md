# Беги и офлайн-анализ

Сессия — это каталог топиков с фрагментами protobuf:

```text
adas_logs/<session>/
  vision__lanes/
  control__lane_keep/
  phone__stats/
  ...
```

`/` в имени топика → `__` на диске.

## Стройка: формат в сорок строк

Бег — это каталоги топиков с записями, у каждой впереди длина. Это вся идея; остальное — protobuf.
Напишите игрушечную версию, чтобы в настоящей не осталось загадок:

```python
import json
import struct
import tempfile
from pathlib import Path

def topic_dir(root, topic):
    return Path(root) / topic.replace("/", "__")     # '/' would nest; '__' keeps one level

def write_bag(root, topic, records):
    d = topic_dir(root, topic)
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "000.bin", "wb") as f:
        for r in records:
            blob = json.dumps(r).encode()
            f.write(struct.pack("<I", len(blob)))    # length prefix: records survive a truncated tail
            f.write(blob)

def read_bag(root, topic):
    out = []
    for part in sorted(topic_dir(root, topic).glob("*.bin")):
        data = part.read_bytes()
        i = 0
        while i + 4 <= len(data):
            (n,) = struct.unpack_from("<I", data, i)
            if i + 4 + n > len(data):
                break                                # the app was killed mid-record; keep what is whole
            out.append(json.loads(data[i + 4 : i + 4 + n]))
            i += 4 + n
    return out

root = tempfile.mkdtemp()
recs = [{"ts": 1000 + 42 * k, "y_l": -1.7, "y_r": 1.8} for k in range(100)]
write_bag(root, "vision/lanes", recs)
back = read_bag(root, "vision/lanes")
print(f"wrote 100, read {len(back)}, first ts {back[0]['ts']}, dir {topic_dir(root, 'vision/lanes').name}")
assert back == recs, "acceptance: a bag must survive the round trip byte-exactly"
```

Настоящий формат отличается двумя вещами: записи — protobuf `ZMQMessage` (ровно те байты, что пересекли
мост, — логгирование это *отвод*, а не перекодировка), и файлы ротируются по размеру. Пишет
`Logger.java`, читает `scripts/vis/bag_io.py`; правило `'/'→'__'` выше взято оттуда дословно.

## Команды

```bash
cd <корень репозитория>
./scripts/run_bag_vis.sh /path/to/session

cd scripts
./generate_proto_python.sh
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 tools/latency.py /path/to/session
python3 vis/export_to_plotjuggler.py /path/to/session -o /tmp/out
```

## Ключевые топики

| топик | на какой вопрос отвечает |
|---|---|
| `vision/lanes` | геометрия, темп, сквозная задержка |
| `control/lane_keep_debug` | CTE, $\kappa$, `frame_dt_ms` |
| `controls/steer` | команда актуации |
| `vehicle/state` | $v$, фактический SWA |
| `phone/stats` | процессор, нагрев |
| `middleware/stats` | отставание нативных таймеров |

## Как читать бег самому

`load_topic_messages(session, topic)` возвращает список `(timestamp_ms, payload, envelope)`. Это весь API.
Всё в `scripts` построено на нём, и каждый анализ в этом курсе тоже.

```bash
cd scripts
export PYTHONPATH=.
```

```python
# not-runnable — needs a real session on disk
from pathlib import Path
import numpy as np
from vis.bag_io import load_topic_messages, list_topics

session = Path("adas_logs/2026_08_06_00_36_42")
print(list_topics(session))

lanes = load_topic_messages(session, "vision/lanes")
state = load_topic_messages(session, "vehicle/state")
print(f"{len(lanes)} vision frames, {len(state)} chassis samples")

ts, msg, _ = lanes[100]
print("fields:", [f.name for f in msg.DESCRIPTOR.fields])
```

Три вещи кусают каждого по одному разу, поэтому их стоит узнать заранее, а не потом.

**Передавайте `Path`, а не строку.** `load_topic_messages` склеивает через `/`, поэтому `str` даёт
`TypeError: unsupported operand type(s) for /`. Задним числом очевидно, в момент — непрозрачно.

**Не выравнивайте сканированием.** Вспомогательная `nearest(ts, series)` — линейный поиск: это нормально для
пары обращений и квадратично для всего бега. Выровнять 22 000 кадров зрения по 140 000 отсчётов шасси таким
способом — это часы. Отсортируйте раз и ищите двоичным поиском.

**Убирайте повторяющиеся метки времени.** Рекордер пишет несколько топиков в одну миллисекунду, поэтому в
топике могут встретиться повторы. Если их оставить, `dt` станет нулём, любая производная — `inf` или `nan`, а
дальше какой-нибудь гейт вида `abs(accel) < 0.5` молча отбросит все отсчёты. Именно эта ошибка однажды срезала
1081 отсчёт GPS до 8.

```python
import numpy as np

def align(t_ref, t_src, v_src, max_dt_ms=120.0):
    """Nearest-sample lookup by binary search, with a staleness limit and a validity mask."""
    idx = np.clip(np.searchsorted(t_src, t_ref), 0, len(t_src) - 1)
    prev = np.clip(idx - 1, 0, len(t_src) - 1)
    take_prev = np.abs(t_src[prev] - t_ref) < np.abs(t_src[idx] - t_ref)
    idx = np.where(take_prev, prev, idx)
    return v_src[idx], np.abs(t_src[idx] - t_ref) <= max_dt_ms

def drop_repeated_stamps(t, *arrays):
    """Keep only strictly increasing timestamps, so derivatives stay finite."""
    keep = np.concatenate(([True], np.diff(t) > 0))
    return (t[keep],) + tuple(a[keep] for a in arrays)

# A 1 Hz source (GPS) against a 100 Hz one (chassis), with a duplicate stamp planted at index 3.
t_gps = np.array([0, 1000, 2000, 2000, 3000, 4000], dtype=float)
v_gps = np.array([10.0, 11.0, 12.0, 12.0, 12.5, 13.0])
t_gps, v_gps = drop_repeated_stamps(t_gps, v_gps)
print("after dedup:", t_gps)

t_can = np.arange(0, 4001, 10, dtype=float)
v_can = 10.0 + 0.75e-3 * t_can
can_at_gps, ok = align(t_gps, t_can, v_can)
print("aligned:", np.round(can_at_gps[ok], 2), "valid", ok.sum(), "of", len(ok))

# Now the derivative that used to blow up is finite.
accel = np.gradient(v_gps) / np.maximum(np.gradient(t_gps) * 1e-3, 1e-3)
print("gps accel:", np.round(accel, 3), "all finite:", bool(np.isfinite(accel).all()))
```

## Измерить что-то настоящее: согласие двух датчиков

Приём, который дал большинство находок в этом проекте: возьмите два независимых измерения одной величины,
смотрите на их **отношение**, а не на разность, и проверьте, зависит ли отношение от чего-нибудь. Постоянное
отношение — это ошибка калибровки. Отношение, которое плывёт со скоростью или нагрузкой, — это физика.

```python
rng = np.random.default_rng(3)
speeds = rng.uniform(5.0, 25.0, 500)
truth = speeds
wheel = truth * 1.012 + rng.normal(0, 0.05, 500)      # 1.2 % scale plus noise
doppler = truth + rng.normal(0, 0.03, 500)

ratio = wheel / doppler
print(f"scale: median {np.median(ratio):.4f}  p10 {np.percentile(ratio, 10):.4f}  "
      f"p90 {np.percentile(ratio, 90):.4f}")
print(f"{'band':>12} {'n':>5} {'scale':>8}")
for lo, hi in ((5, 10), (10, 15), (15, 20), (20, 25)):
    m = (doppler >= lo) & (doppler < hi)
    print(f"{f'{lo}-{hi} m/s':>12} {m.sum():>5} {np.median(ratio[m]):>8.4f}")
print("flat across bands -> a constant to calibrate; sloping -> something physical to model")
```

На настоящих бегах это даёт 1.0117 и 1.0120 на двух разных заездах, плоско по всем диапазонам. Так и была
найдена ошибка радиуса колеса, описанная в главе [про локализацию](../Localization/Overview.md), и тем же
методом получены конверт замедления выбегом и метрический масштаб модели.

```{admonition} Всегда сообщайте гейты
:class: warning
Каждое измерение выше выбрасывает часть отсчётов — медленные, переходные, с плохим фиксом, с устаревшим
выравниванием. Говорите, сколько выжило. «Масштаб 1.012» по восьми отсчётам и по 798 — это разные
утверждения, и первое здесь уже однажды оказывалось неверным.
```

## Запишите собственный бег и прогоните по нему собственный код

Для первого бега машина не нужна: лучше всего телефон на лобовом пассажиром, годится и прогулка вдоль
дороги с окрашенными краями. Соберите (`./scripts/build_project.sh`), поставьте, включите логгирование
из UI, подвигайтесь 3–5 минут, заберите сессию (`./scripts/pull_bags.sh`), откройте в визуализаторе.
Чего в вашем беге не может быть — `vehicle/state`, `can/rx`, момента — это достоинство: всё, что
контроллер на нём выдаёт, непроверяемо, так что ошибаться можно сколько угодно, бесплатно.

Затем скормите его коду, который вы построили в [Зрении](../Vision/Overview.md) и
[Pure Pursuit](../Planner/PurePursuit.md):

```python
# not-runnable — needs your session directory and generated protobufs
from pathlib import Path
import numpy as np
from vis.bag_io import load_topic_messages

session = Path("../adas_logs/<your-session>")
lanes = load_topic_messages(session, "vision/lanes")
print(len(lanes), "lane frames")

for ts, msg, _raw in lanes[:3]:
    left, right = msg.lanes[1], msg.lanes[2]      # near-left and near-right host lines
    print(ts, len(left.points), "pts  prob", round(left.prob, 2), round(right.prob, 2))
```

```python
# not-runnable — sketch of the analysis loop
deltas, gaps, sigmas = [], [], []
for ts, msg, _ in lanes:
    left, right = msg.lanes[1], msg.lanes[2]
    if left.prob < 0.5 or right.prob < 0.5:
        gaps.append(ts)                             # the frames your toy never had
        continue
    path = fuse_from_proto(left, right)             # your step-3 code, adapted to proto points
    delta, _ = pure_pursuit(path, 0.0, 0.0, 0.0, 8.0, 0)
    deltas.append(delta)
    sigmas.append((median_std(left), median_std(right)))

print("frames:", len(lanes), " usable:", len(deltas), " dropped:", len(gaps))
print("|delta| p95 [deg]:", np.degrees(np.percentile(np.abs(deltas), 95)))
```

Ждите форму, если не числа: σ ничем не похожа на синтетические 5 см (медианы 0.37/0.51 м на заездах
проекта, обе линии уверены одновременно в ~22 % кадров); целые нефлагуемые отрезки без обеих линий;
шаг кадров 42 мс с дрожанием. Запишите три худших момента — пропуск, вспышку σ, скачок шага кадров, по
метке времени и предложению на каждый. Эти три метки — ваш личный тестовый набор с этого дня.

## Шаблон отчёта

1. Идентификатор сессии и временное окно (или HCA включён / выключен).
2. Темп зрения, сквозная задержка медиана/p95; если есть — нагрев.
3. Метрики качества: |CTE| медиана/p95, |$\Delta$SWA|.
4. Вывод: зрение / управление / нагрев (разделять явно).

<!-- next-chapter -->
---

**Дальше:** [Задания](../Exercises/StudentProjects.md)
