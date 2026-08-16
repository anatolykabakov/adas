# Предупреждения: FCW, AEB, LDW

Сервис **`safety_warn`** поднимает **только иконки и текст на экране** — он **не** тормозит и не рулит.
Это сделано намеренно: студенты изучают логику угроз без риска актуации.

Основной инженерный документ: `docs/SAFETY_WARN.md`.
Код: `safety_planner.hpp`, `safety_warn_service.cpp`, тесты `test_safety_warn.cpp`.

## Что здесь означают сокращения

| флаг | смысл в этом проекте |
|---|---|
| **FCW** | предупреждение о фронтальном столкновении — «вы сближаетесь слишком быстро» |
| **AEB** | более сильная фронтальная угроза (на экране красное `BRAKE!`) — **автоторможения в нашем стеке всё равно нет** |
| **LLDW / RLDW** | предупреждение о выходе из полосы влево / вправо |

AEB перебивает FCW, когда сработали бы оба.

## Путь данных

```text
vision/lanes (Java) → TopicConvert → vision/path   ─┐
vision/model_long (lead0, …)                       ─┼→ SafetyWarnService @ 50 ms
vehicle/chassis (v, blinker, steering_pressed)     ─┘
        → safety/warn → ZMQ OUT → HUD / bag
```

```{note}
`LongPlanService` тоже может читать `model_long` для желаемого ускорения в духе ACC.
Предупреждения этим ускорением IDM **больше не пользуются** — оно ложно срабатывало на пустом шоссе.
```

## Как эти правила получились: три круга ошибок

Ничто в этой главе не спроектировано на бумаге. Каждое правило заменило более простое, которое срабатывало
там, где опасности не было, и последовательность стоит проследить: поломки поучительнее формул.

### Круг 1: ускорение как признак опасности

Первая версия поднимала FCW, когда желаемое ускорение IDM падало ниже −3 м/с², а AEB — ниже −5. Звучало
разумно: хотим сильно тормозить — значит впереди опасность.

Это неверно, потому что в ускорении IDM есть слагаемые, не имеющие отношения к машине впереди.

```python
def idm_accel(v_ego, v_lim, gap=None, v_lead=None, a_max=1.5, b=3.0, T=1.5, s0=2.0, expn=4.0):
    """Treiber IDM. The free-road term alone can demand hard braking with nothing in front."""
    free = (v_ego / max(v_lim, 0.1)) ** expn
    inter = 0.0
    if gap is not None and v_lead is not None:
        dv = v_ego - v_lead
        s_star = s0 + max(0.0, v_ego * T + v_ego * dv / (2.0 * (a_max * b) ** 0.5))
        inter = (s_star / max(gap, 0.5)) ** 2
    return a_max * (1.0 - free - inter)

MU_G = 0.5 * 9.81
print(f"{'situation':>38} {'a_idm':>8}  would fire?")
# Empty road, over the assumed limit
print(f"{'empty road at 37 m/s, limit 27.8':>38} {idm_accel(37.0, 27.8):>8.2f}  FCW")
# Empty arc: the curvature speed limit becomes v_lim
v_lim_arc = (MU_G / (1.0 / 50.0)) ** 0.5
print(f"{'empty R=50 m arc at 22 m/s':>38} {idm_accel(22.0, v_lim_arc):>8.2f}  FCW")
# An actual lead car, closing
print(f"{'lead 15 m ahead at 13 m/s, we do 20':>38} {idm_accel(20.0, 27.8, 15.0, 13.0):>8.2f}  FCW")
```

В первых двух строках нет никакой машины. И пустая дорога выше предполагаемого ограничения, и пустой крутой
поворот требуют сильного торможения, и оба раньше зажигали предупреждение о столкновении. Урок обобщается за
пределы этого проекта: **желаемый выход контроллера не является сигналом опасности.** В нём лежит всё, что
контроллеру важно, а предупреждение должно строиться из самой угрозы.

Поэтому правило стало основанным на угрозе — время до столкновения и требуемое замедление, вычисленные по
цели, которая существует, — а ускорение IDM осталось в сообщении только как отладочное поле.

### Круг 2: цель, которой ещё нет

Вторая версия брала самый вероятный из `lead0`, `lead1`, `lead2`. Это не три кандидата в машины: `lead1` и
`lead2` — предсказания модели о лидере **на +2 с и +4 с**. Предупреждать о машине, которая по мнению модели
окажется там через четыре секунды, — это другой продукт.

Исправлено переходом только на `lead0`. Отмечаю это отдельно, потому что тот же дефект прожил в продольном
планировщике ещё два месяца и нашёлся только когда планировщику дали действовать.

### Круг 3: предупреждение о самих себе

С правилами по угрозе и с верной целью 23-минутный ночной заезд всё равно дал 5 фронтальных и 7 полосных
предупреждений, все ложные. Две разные причины, и обе видны только в измерении:

| класс | что сказали числа | какой гейт появился |
|---|---|---|
| фронтальные | все 5 эпизодов — движение в пробке: медиана 4.7 м/с, максимум 8.5. Худший случай 4.3 м/с при почти стоящем лидере в 9.5 м — арифметически TTC 1.9 с, на практике тривиально | `warn_min_speed_ms` 3 → 8 м/с |
| полосные | из 144 кадров LDW **82 % были при включённом своём поперечном управлении**, а водитель рулил лишь в 6 %. \|CTE\| 0.54 м при сносе 0.15 м/с | подавлять LDW, пока рулим сами (`ldw_suppress_on_lat_active`) |

Вторая причина — интереснее. LDW существует, чтобы предупреждать *сносящего водителя*. С включённым
ассистентом смещение, которое он измеряет, — это собственная ошибка слежения ассистента на дуге; система
предупреждала о себе, и делала это в 82 % случаев, когда открывала рот.

```{admonition} Дефолт в заголовке — ещё не решение
:class: warning
Скоростной гейт выше был поднят в коде и **не** в `assets/config.json`, а тот держит своё значение и
перебивает. Поэтому правка оказалась мёртвой, и следующий заезд воспроизвёл те же три ложных срабатывания на
4.8–5.5 м/с. Теперь три теста проверяют, что поставляемый конфиг соответствует решениям, принятым на дороге, а
сборка отказывается компилировать конфиг, который не разбирается.
```

## Математика продольной угрозы (FCW / AEB)

Нужен **присутствующий лидер** (только `lead0`, не будущие гипотезы lead1/2):

* вероятность ≥ `lead_prob_thresh` (0.5);
* дистанция $d$ в диапазоне $(1,\ 150)$ м;
* своя скорость $v \ge$ `warn_min_speed_ms` (**8 м/с** — поднято с 3 после ложных срабатываний в пробке из круга 3);
* лидер примерно **на нашем пути**: $|y_{\mathrm{lead}} - y_{\mathrm{path}}(d)| \le$ `lead_max_offset_m`;
* сближение: $\Delta v = v - v_{\mathrm{lead}} \ge$ `min_closing_speed_ms`.

Тогда

$$
\mathrm{ttc} = \frac{\mathrm{gap}}{\Delta v},
\qquad
a_{\mathrm{req}} = \frac{(\Delta v)^{2}}{2\cdot\mathrm{gap}}.
$$

С поставляемыми порогами:

| | TTC | требуемое замедление |
|---|---:|---:|
| FCW | $\le 2.5$ с | или $a_{\mathrm{req}} \ge 3.5$ м/с² |
| AEB | $\le 1.4$ с | или $a_{\mathrm{req}} \ge 5.5$ м/с² |

```python
def forward_warning(gap_m, v_ego, v_lead, *,
                    min_close=0.5, fcw_ttc=2.5, aeb_ttc=1.4,
                    fcw_a=3.5, aeb_a=5.5):
    dv = v_ego - v_lead
    if gap_m <= 1.0 or dv < min_close:
        return None, float("nan"), float("nan")
    ttc = gap_m / dv
    a_req = (dv * dv) / (2.0 * gap_m)
    if ttc <= aeb_ttc or a_req >= aeb_a:
        return "AEB", ttc, a_req
    if ttc <= fcw_ttc or a_req >= fcw_a:
        return "FCW", ttc, a_req
    return None, ttc, a_req

# Slow close from 30 m at 5 m/s relative
print(forward_warning(30.0, 20.0, 15.0))
# Fast close — expect AEB
print(forward_warning(12.0, 25.0, 10.0))
# Not closing
print(forward_warning(30.0, 15.0, 16.0))
```

### Почему важен $y$ относительно пути

На дуге машина впереди **в вашей же** полосе имеет ненулевой $y$ в системе машины.
Сравнение с **осью** ложно даёт «соседняя полоса». Сравнение с $y_{\mathrm{path}}(d)\approx \mathrm{CTE}+\tfrac12\kappa d^2$ удерживает вашего собственного лидера в полосе.

```python
def path_y(d, cte=0.1, kappa=0.01):
    return cte + 0.5 * kappa * d * d

d, y_lead = 25.0, 0.8
print("in path?", abs(y_lead - path_y(d)) <= 2.0)
```

## Математика поперечного LDW

Гейты (все обязательны, если не сказано иное):

* $v \ge$ `ldw_min_speed_ms` (12.5 м/с ≈ 45 км/ч);
* `lane_anchored` — путь построен по **двум** правдоподобным линиям разметки;
* поворотник с этой стороны выключен (`ldw_suppress_on_blinker`);
* водитель не держит руль, если `ldw_suppress_on_driver_steer`;
* **мы не рулим сами** (`ldw_suppress_on_lat_active`) — 82 % ложных полосных предупреждений были измерением собственной ошибки слежения ассистента на дуге.

Тогда срабатывает, если

$$
|\mathrm{CTE}| > 0.5~\mathrm{м}
\ \textbf{и}\
\text{скорость наружу} > 0.05~\mathrm{м/с}
\quad\textbf{или}\quad
|\mathrm{CTE}| > 0.8~\mathrm{м}.
$$

```python
def ldw(cte, cte_rate, side_right, *,
        v=15.0, anchored=True, blinker=False, hands_on=False):
    if v < 12.5 or not anchored or hands_on:
        return False
    if blinker:
        return False
    outward = (cte_rate > 0.05) if side_right else (cte_rate < -0.05)
    # simplify: |cte| with outward OR hard threshold
    return (abs(cte) > 0.5 and outward) or abs(cte) > 0.8

print("hold offset in arc:", ldw(0.55, 0.0, True))   # False (no outward rate)
print("drifting out:", ldw(0.55, 0.1, True))         # True
print("hard offset:", ldw(0.85, 0.0, True))           # True
```

Старое правило «просто $|cte|>0.5$» давало **сотни** ложных эпизодов на городских бегах; LDW с гейтами их убрал — см. `SAFETY_WARN.md`.

## Антидребезг (`WarningLatch`)

Тик — 50 мс. Поднимаем после **3** истинных тиков (150 мс); снимаем после **10** ложных (500 мс).

```python
class Latch:
    def __init__(self, set_n=3, hold_n=10):
        self.set_n, self.hold_n = set_n, hold_n
        self.on = False
        self.cnt = 0

    def update(self, raw: bool) -> bool:
        if raw:
            self.cnt = self.cnt + 1 if not self.on else self.hold_n
            if self.cnt >= self.set_n:
                self.on = True
        else:
            if self.on:
                self.cnt -= 1
                if self.cnt <= 0:
                    self.on = False
                    self.cnt = 0
            else:
                self.cnt = 0
        return self.on

L = Latch()
seq = [0, 1, 1, 1, 1, 0, 0, 0]  # spikes then quiet start
print([int(L.update(bool(x))) for x in seq])
```

(Настоящая семантика защёлки — в `WarningLatch`, истина в тестах; этот набросок показывает, *зачем* антидребезг нужен.)

## Блок конфига

`config.json` → `safety_warn`:

```json
"fcw_ttc_s": 2.5,
"aeb_ttc_s": 1.4,
"fcw_decel_ms2": 3.5,
"aeb_decel_ms2": 5.5,
"warn_min_speed_ms": 8.0,
"cte_ldw_threshold_m": 0.5,
"cte_ldw_hard_m": 0.8,
"ldw_min_speed_ms": 12.5,
"ldw_suppress_on_lat_active": true,
"warn_set_frames": 3,
"warn_hold_frames": 10
```

Два из этих значений — результат круга 3 выше, и оба лежат в поставляемом конфиге, а не только в дефолтах
заголовка: дефолт, который конфиг перебивает, решением не является, — именно так скоростной гейт двое суток
оставался мёртвым.

Переключатель узла: `nodes.safety_warn: true`.

## Как воспроизвести и проверить

**Юнит-тесты** (без телефона):

```bash
./scripts/docker.sh tests
# или отфильтровать: test_safety_warn
```

**Эпизоды по бегу** (настоящая цепочка Java→конвертация→сервис через `pyadas`):

```bash
PYTHONPATH=scripts \
  python3 scripts/bag/bag_safety_warn.py adas_logs/<session>
```

Сообщайте **эпизоды**, а не сырые количества кадров.

```{warning}
Во многих дорожных бегах нет пригодного `vision/model_long` → FCW и AEB молчат. LDW всё равно нужны `lane_anchored` и скорость.
```

## Задания

1. Прогоните примеры `forward_warning`; найдите пару `(gap, Δv)`, которая даёт FCW, но не AEB.
2. Объясните одним предложением, почему ускорение IDM было плохим триггером для FCW.
3. По бегу прогоните `bag/bag_safety_warn.py` и запишите количества эпизодов LDW / FCW / AEB плюс темп зрения.

<!-- next-chapter -->
---

**Дальше:** [Калибровка камеры](../Calibration/Overview.md)
