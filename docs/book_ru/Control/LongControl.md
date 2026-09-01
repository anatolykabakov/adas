# Продольное управление — читать план там, где будет актуатор

[Продольный планировщик](../Planner/Longitudinal.md) отдаёт траекторию скорости; этот контур превращает её в
запрос ускорения — так же, как [угловой контур](./AngleControl.md) превращает кривизну в момент. Это
`LongControl` upstream: пропорциональный закон с ускорением плана в качестве feedforward, чтение на 0.15 с
вперёд, чтобы покрыть актуатор, и автомат из четырёх состояний, который делает остановку остановкой.

Код: `longitudinal/long_control.cpp`; тик живёт в `services/control.cpp` рядом с поперечным законом.

Общие константы для фрагмента, как в главе планировщика:

```python
import numpy as np

ACCEL_MIN, ACCEL_MAX = -3.5, 2.0   # the panda's envelope for a VW MQB [m/s^2]
```

## Закон

План — траектория скорости на 33-точечной сетке модели, 17 её точек (2.5 с) отдаются в `Control`. Моторный
и тормозной контроллеры машины отвечают на запрос ускорения с опозданием около 0.15 с, поэтому закон читает
план **на 0.15 с вперёд** и превращает наклон скорости на этой задержке в ускорение, которое просит:

```python
T_MODEL = np.array([10.0 * (i / 32) ** 2 for i in range(33)])[:17]
DELAY = 0.15

def long_control(speeds, accels, v_ego, t_since_plan=0.0, kp=0.1):
    v_now = np.interp(t_since_plan, T_MODEL, speeds)
    a_now = np.interp(t_since_plan, T_MODEL, accels)
    v_target = np.interp(DELAY + t_since_plan, T_MODEL, speeds)
    a_target = 2.0 * (v_target - v_now) / DELAY - a_now       # the slope over the delay, minus what we have
    accel = kp * (v_target - v_ego) + a_target                 # VW: kp 0.1, ki 0, kf 1
    return float(np.clip(accel, ACCEL_MIN, ACCEL_MAX)), v_target, a_target

speeds = np.maximum(0.0, 20.0 - 1.5 * T_MODEL)
accels = np.full(17, -1.5)
cmd, v_t, a_t = long_control(speeds, accels, v_ego=20.0)
print(f"braking plan at -1.5: v_target {v_t:.2f} m/s, feedforward {a_t:.2f}, command {cmd:.2f} m/s^2")
assert abs(a_t + 1.5) < 1e-9, "on a plan of constant acceleration the slope gives back that acceleration"
```

Откуда $2(\cdot)/\Delta$: за задержку $\Delta = 0.15$ с скорость должна пройти от $v_{\mathrm{now}}$ до
$v_{\mathrm{target}}$, пока ускорение линейно растёт от того, что у машины сейчас, $a_{\mathrm{now}}$, до того, что мы
просим, $a_{\mathrm{target}}$. Среднее ускорение на рампе — $\tfrac12(a_{\mathrm{now}} + a_{\mathrm{target}})$, откуда

$$
v_{\mathrm{target}} - v_{\mathrm{now}} = \tfrac{1}{2}\,(a_{\mathrm{now}} + a_{\mathrm{target}})\,\Delta
\quad\Longrightarrow\quad
a_{\mathrm{target}} = \frac{2\,(v_{\mathrm{target}} - v_{\mathrm{now}})}{\Delta} - a_{\mathrm{now}} .
$$

На плане с постоянным ускорением формула возвращает это ускорение ровно (фрагмент это утверждает). На плане,
который *изгибается* — машина в $a = 0$, а план хочет $-1.5$ через 0.15 с, — она просит $-3.0$ на один тик и
отпускает: запрос опережает план на задержку, и машина не отстаёт от него. Пропорциональный член
$0.1\,(v_{\mathrm{target}} - v_{\mathrm{ego}})$ мал намеренно: при kp 0.1 ошибка скорости 2 м/с добавляет лишь
$0.2$ м/с² — ведёт feedforward, P-член только правит дрейф.

```{figure} figures/long_control_law.png
---
width: 100%
---
Слева: план, прочитанный на задержке актуатора. Справа: четыре состояния — `pid` в движении, `stopping`,
когда план стоит ниже 1 м/с (запрос уходит к −2 м/с² и держится), `starting`, когда план трогается (+1 м/с²
до 1 м/с), затем снова `pid`.
```

**Автомат состояний** — то, что делает остановку остановкой: ниже 1 м/с при стоящем плане закон покидает
пропорциональный мир и уходит к удержанию −2 м/с² со скоростью 0.8 м/с³ — `stopAccel` и
`stoppingDecelRate` upstream, — потому что P-закон на скорости 0.3 м/с ползёт. Тонкость, стоившая вечера:
`stay_stopped` upstream читает флаг стоянки *штатного ACC*, которого нет, как только осью владеем мы;
подача туда простой скорости заставляла `starting` и `stopping` меняться местами каждый тик.

## Приёмка

* фрагмент возвращает постоянное ускорение плана ровно и добавляет $0.1$ м/с² на каждый м/с ошибки скорости;
* автомат проходит pid → stopping → starting → pid на плане «стоп и трогание», удерживая −2 м/с² на месте;
* одно предложение о том, почему `stay_stopped` не должен читать простую скорость.

## Куда дальше

* [Продольный планировщик](../Planner/Longitudinal.md) — откуда берётся траектория скорости.
* [Угловой контур](./AngleControl.md) — поперечный внутренний контур, близнец этого.
* [Платформа](../Platform/Overview.md) — как ускорение становится `ACC_06`/`ACC_07` на шине.

<!-- next-chapter -->
---

**Дальше:** [Платформа](../Platform/Overview.md)
