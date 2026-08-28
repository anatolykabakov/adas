# Продольный планировщик — скорость и дистанция

Поперечные главы заканчивались кривизной. Эта заканчивается **ускорением**: планировщик решает, как быстро
ехать и на каком расстоянии сидеть за машиной впереди, закон управления превращает это в запрос ускорения,
а платформа кладёт его в собственные ACC-кадры автомобиля — те же три сервиса, что и у рулевого пути,
намеренно. Это замена кнопочному пути через штатный круиз, которым стек управлял скоростью раньше: нажимать
подрулевой переключатель за водителя — обход отсутствия актуатора; ускорение — и есть актуатор.

Код: `longitudinal/long_mpc.cpp` (оптимизатор), `longitudinal/long_planner.cpp` (то, что вокруг него),
`longitudinal/long_control.cpp` (закон), `platform/volkswagen/mqbcan.cpp` (`ACC_06`/`ACC_07`/`ACC_02`).
Формулировка, веса и сетка — из `long_mpc.py` / `longitudinal_planner.py` openpilot; солвер — наш, а глава
заканчивается прогонами в симуляторе, которые говорят, что порт ведёт себя как надо.

```{admonition} Если вы не встречали ACC
:class: note
Адаптивный круиз держит **уставку скорости**, когда дорога пуста, и **временной зазор** за машиной впереди,
когда нет, — здесь 1.45 с плюс запас 6 м на стоянке. Всё ниже — один вопрос, заданный двенадцать раз в
будущее: *где мне можно быть в момент t, чтобы остановка всё ещё была комфортной?*
```

## Две дистанции, на которых всё держится

Общие константы игрушки для всех фрагментов ниже — upstream'овские, дословно:

```python
import math
import numpy as np

T_FOLLOW = 1.45      # desired time gap [s]
COMFORT_BRAKE = 2.5  # deceleration the safe distance assumes [m/s^2]
STOP_DISTANCE = 6.0  # gap kept at standstill [m]
A_CRUISE_MIN = -1.2  # comfort deceleration when cruising or following [m/s^2]
ACCEL_MIN, ACCEL_MAX = -3.5, 2.0   # the panda's envelope for a VW MQB [m/s^2]

def safe_distance(v_ego, t_follow=T_FOLLOW):
    """Room to stop at COMFORT_BRAKE from v_ego, plus the time gap, plus the standstill margin."""
    return v_ego**2 / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE

def stopped_equivalence(v_lead):
    """How much further a moving lead's stopping point is than its bumper."""
    return v_lead**2 / (2 * COMFORT_BRAKE)

def desired_gap(v_ego, v_lead):
    return safe_distance(v_ego) - stopped_equivalence(v_lead)

for v in (10, 20, 30):
    print(f"v={v:>2} m/s: safe to a wall {safe_distance(v):6.1f} m, gap behind an equal-speed lead {desired_gap(v, v):5.1f} m")
assert abs(desired_gap(20, 20) - (T_FOLLOW * 20 + STOP_DISTANCE)) < 1e-9
```

Две вещи, которые надо удержать. Во-первых, **безопасная дистанция** — это тормозной путь: она растёт как
$v^2$, и при 30 м/с это 230 м. Во-вторых, лид, который *движется*, возвращает вам свой тормозной путь,
поэтому за машиной на вашей же скорости зазор схлопывается до линейной части: $1.45\,v + 6$. Весь
планировщик — способ жить между этими двумя кривыми.

```{figure} figures/long_safe_distance.png
---
width: 85%
---
Безопасная дистанция до неподвижного препятствия (квадратичная) против желаемого зазора за лидом на той же
скорости (линейный). План никогда не даёт ego пересечь первую; он пытается сидеть на второй.
```

## Сборка 1: пропорциональный закон по зазору — и где он ломается

Очевидный регулятор: ускоряться пропорционально тому, насколько зазор отличается от желаемого.

```python
def follow_p(kp, kv, seconds=40.0, dt=0.05):
    """Ego behind a lead that brakes from 20 to 5 m/s at -2 m/s^2 between t=5 and t=10 s."""
    v_lead, v_ego, gap = 20.0, 20.0, 30.0
    log = []
    for k in range(int(seconds / dt)):
        t = k * dt
        if 5.0 <= t < 10.0:
            v_lead = max(5.0, v_lead - 2.0 * dt)
        want = T_FOLLOW * v_ego + STOP_DISTANCE
        a = kp * (gap - want) + kv * (v_lead - v_ego)
        a = max(ACCEL_MIN, min(ACCEL_MAX, a))
        v_ego = max(0.0, v_ego + a * dt)
        gap += (v_lead - v_ego) * dt
        log.append((t, gap, want, a))
    return np.array(log)

gap_only = follow_p(kp=0.35, kv=0.0)
with_speed = follow_p(kp=0.15, kv=0.5)
print(f"gap-only law:   min gap {gap_only[:, 1].min():5.1f} m, hardest brake {gap_only[:, 3].min():5.2f} m/s^2")
print(f"with speed term: min gap {with_speed[:, 1].min():5.1f} m, hardest brake {with_speed[:, 3].min():5.2f} m/s^2")
assert gap_only[:, 1].min() < with_speed[:, 1].min(), "a gap-only law overshoots into the lead"
```

```{figure} figures/long_gap_p_toy.png
---
width: 95%
---
Игрушка 1: P-закон только по зазору звенит после торможения лида; скоростной член его гасит. Ни один не
знает, что лид сделает в следующую секунду, и ни один не умеет обменять комфорт сейчас на дистанцию потом.
```

У P-закона три отказа, которые не лечатся коэффициентом. Он реагирует на зазор *после* того, как тот
закрылся; лида, тормозящего с −2, и лида, который уже перестал тормозить, он трактует одинаково; и у него
нет понятия **рывка**, поэтому он просит любое ускорение, какое скажет ошибка, мгновенно. Каждый из трёх —
утверждение о будущем, а для этого и существует оптимизатор на горизонте.

## Сборка 2: горизонт — тройной интегратор, управляемый рывком

Продольный MPC upstream мал настолько, что помещается в одной руке. Объект — положение, скорость и
ускорение ego, управление — рывок:

$$
\dot x = v,\qquad \dot v = a,\qquad \dot a = j .
$$

Он интегрируется на **двенадцати интервалах квадратичной сетки** $t_i = 10\,(i/12)^2$: 0.07 с у руля,
1.6 с в конце 10-секундного горизонта. Ближние решения получают разрешение, дальний конец — охват.

```python
N = 12
T = np.array([10.0 * (i / N) ** 2 for i in range(N + 1)])
DT = np.diff(T)

def rollout(u, v0, a0):
    """Exact integration of constant jerk per interval — the plant is linear."""
    x, v, a = np.zeros(N + 1), np.zeros(N + 1), np.zeros(N + 1)
    v[0], a[0] = v0, a0
    for k in range(N):
        d = DT[k]
        x[k + 1] = x[k] + v[k] * d + 0.5 * a[k] * d * d + u[k] * d**3 / 6
        v[k + 1] = v[k] + a[k] * d + 0.5 * u[k] * d * d
        a[k + 1] = a[k] + u[k] * d
    return x, v, a

print("grid [s]:", np.round(T, 2))
print("first interval %.3f s, last %.2f s" % (DT[0], DT[-1]))
assert abs(T[6] - 2.5) < 1e-9 and abs(T[-1] - 10.0) < 1e-9
```

```{figure} figures/long_mpc_grid.png
---
width: 90%
---
Сетка горизонта. Красные числа — длина каждого интервала и, поскольку acados масштабирует стоимость каждой
стадии её шагом по времени, её **вес**. Без этого масштабирования та же стоимость даёт план куда мягче
upstream'овского.
```

Стоимость — один член по препятствию плюс рывок, с числами upstream: `X_EGO_OBSTACLE_COST = 3`,
`J_EGO_COST = 5`, `A_CHANGE_COST = 200` (гаснет на 1–2 с) и мягкие ограничения с `LIMIT_COST = 1e6` на
границы скорости и ускорения и `DANGER_ZONE_COST = 100` за вход в 0.75 безопасной дистанции:

$$
r_{\mathrm{obs},i} = \frac{(x_{\mathrm{obs},i} - x_i) - d_{\mathrm{safe}}(v_i)}{v_i + 10},\qquad
J = \sum_i \Delta t_i \Big[\,3\,r_{\mathrm{obs},i}^2 + 5\,j_i^2 + 200\,w_i\,(a_i - a^{\mathrm{prev}}_i)^2 + \text{штрафы}\Big].
$$

Деление на $v+10$ — нормировка upstream: ошибка в 10 м значит больше на 5 м/с, чем на 30. «Препятствие»
$x_{\mathrm{obs}}$ — это где будет *точка остановки* лида: его предсказанное положение плюс
`stopped_equivalence`, — так что план целится в $g = 0$: ровно безопасная дистанция от места, где лид мог бы
остановиться.

```python
def residuals(u, v0, a0, x_obs, a_min=ACCEL_MIN, a_max=1.2):
    """upstream's cost as a residual vector, each stage scaled by sqrt(dt) like acados does."""
    x, v, a = rollout(u, v0, a0)
    sc = np.sqrt(np.append(DT, 1.0))
    r = []
    for i in range(N + 1):
        g = (x_obs[i] - x[i]) - safe_distance(v[i])
        r.append(sc[i] * math.sqrt(3.0) * g / (v[i] + 10.0))
        if i < N:
            r.append(sc[i] * math.sqrt(5.0) * u[i])
    for i in range(N):
        z = sc[i] * 1000.0                          # sqrt(LIMIT_COST)
        r += [z * max(0.0, -v[i]), z * max(0.0, a_min - a[i]), z * max(0.0, a[i] - a_max)]
        danger = (x_obs[i] - x[i]) - 0.75 * safe_distance(v[i])
        r.append(sc[i] * 10.0 * max(0.0, -danger / (v[i] + 10.0)))   # sqrt(DANGER_ZONE_COST)
    return np.array(r)

def solve(v0, a0, x_obs, iters=60):
    """Gauss-Newton with a backtracking step: twelve unknowns, a linear plant, a mildly nonlinear cost."""
    u = np.zeros(N)
    for _ in range(iters):
        r = residuals(u, v0, a0, x_obs)
        J = np.zeros((len(r), N))
        for k in range(N):
            du = np.zeros(N); du[k] = 1e-6
            J[:, k] = (residuals(u + du, v0, a0, x_obs) - r) / 1e-6
        step = np.linalg.solve(J.T @ J + 1e-3 * np.eye(N), -J.T @ r)
        for alpha in (1.0, 0.5, 0.25, 0.125):
            r_try = residuals(u + alpha * step, v0, a0, x_obs)
            if r_try @ r_try < r @ r:
                u = u + alpha * step
                break
        else:
            break
    return rollout(u, v0, a0)

def lead_trajectory(x_lead, v_lead, a_lead, tau=1.5):
    """upstream's extrapolate_lead: acceleration decays as exp(-tau t^2/2), speed never reverses."""
    xs, vs, x, v = [], [], x_lead, v_lead
    for i in range(N + 1):
        d = 0.0 if i == 0 else DT[i - 1]
        v = max(0.0, v + d * a_lead * math.exp(-tau * T[i] ** 2 / 2))
        x += d * v
        xs.append(x); vs.append(v)
    return np.array(xs), np.array(vs)

x_lead, v_lead = lead_trajectory(35.0, 15.0, -3.0)
x_obs = x_lead + stopped_equivalence(v_lead)
x, v, a = solve(20.0, 0.0, x_obs)
print(f"lead 35 m ahead at 15 m/s braking, ego 20 m/s: a within 1.2 s = {a[T <= 1.2].min():.2f} m/s^2, "
      f"speed at 10 s = {v[-1]:.1f} m/s, closest approach {np.min(x_lead - x):.1f} m")
assert np.all(x < x_lead), "the plan must never cross the lead"
assert a.min() >= ACCEL_MIN - 0.3
```

```{figure} figures/long_mpc_plan.png
---
width: 100%
---
То же решение, что во фрагменте: тормозить сейчас, сильнее комфортных −1.2, потому что лид тормозит, и
осесть там, где зазор равен безопасной дистанции от точки остановки лида.
```

Выпадают три свойства, которых у P-закона быть не могло. План тормозит **до** того, как зазор закрылся,
потому что предсказанная траектория лида — внутри стоимости. Он тормозит **сильнее круизного лимита**, когда
должен, — `A_CRUISE_MIN` формирует круизное препятствие ниже, а жёсткая граница — панда с её −3.5. И за
каждый рывок платится, так что запрос гладок по построению, а не благодаря фильтру после.

```{admonition} Солвер не upstream'овский — задача да
:class: warning
upstream генерирует эту задачу в C через `acados_template` и решает SQP-RTI. Этого генератора в сборке
репозитория нет, поэтому `long_mpc.cpp` решает идентичную формулировку плотным Гауссом–Ньютоном
(аналитический якобиан, демпфирование Левенберга, бэктрекинг, тёплый старт). Две вещи, на которые ушёл день
и которые стоит знать: **масштабирование стадий по Δt** выше, и то, что полный гаусс-ньютоновский шаг из
покоя перепрыгивает в штраф границы с весом 1e6, так что шагу нужен линейный поиск — одно раздувание
демпфирования заставляет его ползти. Сходимость — 4–9 итераций на тёплом старте, меньше миллисекунды.
```

## Сборка 3: уставка скорости как призрачная машина

Без лида та же стоимость гонит к **круизному препятствию**: где была бы машина, уже едущая на уставке, —
удержанная в пределах того, чего ego физически достигнет на этом горизонте, чтобы солвер никогда не
стартовал вне собственных границ.

```python
def cruise_obstacle(v_ego, v_cruise, a_min=A_CRUISE_MIN, a_max=1.2):
    v_lower = v_ego + T * a_min * 1.05
    v_upper = v_ego + T * a_max * 1.05
    v_c = np.clip(v_cruise, v_lower, v_upper)
    x_c = np.cumsum(np.append(0.0, DT) * v_c)
    return x_c + safe_distance(v_c)

x_c = cruise_obstacle(10.0, 25.0)
x, v, a = solve(10.0, 0.0, x_c)
print(f"from 10 m/s toward a 25 m/s set speed: a(0.6 s) = {a[3]:.2f}, a(2.5 s) = {a[6]:.2f} m/s^2, v(10 s) = {v[-1]:.1f} m/s")
assert 0.7 < a[6] <= 1.25, "cruising accelerates at the ceiling, not above it"
```

```{figure} figures/long_cruise_obstacle.png
---
width: 95%
---
Без лида: уставка — призрачная машина; план гонится за ней с зависящим от скорости потолком (1.6 м/с² с
места, 0.6 на 40 м/с) и оседает на её скорости.
```

Планировщик вокруг MPC (`long_planner.cpp`) делает ещё четыре вещи, которые делает upstream: фильтрует
желаемую скорость с постоянной времени 2 с и интегрирует её вперёд по плану, ограничивает ускорение по
скорости и по **повороту** (дуга на 20 м/с при 60° руля почти ничего не оставляет продольному), верит лиду
модели только выше `prob 0.5` и только на нашем пути и поднимает **FCW**, когда план три тика подряд
предсказывает столкновение внутри 5 с. Этот стек добавляет одну свою вещь: превью кривизны по слитому пути
ограничивает *уставку* — никогда не текущую скорость, — чтобы к повороту впереди подъезжали, а не
обнаруживали его.

## Сборка 4: закон управления — читать план там, где будет актуатор

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

## Платформа: три кадра и один флаг

`Control` публикует ускорение в том же сообщении `SteerCommand`, что и момент. Платформа гейтит его по
`controls_allowed` панды — нет никакого «всегда включённого» продольного, как есть всегда включённое
поперечное, — и VW-порт кладёт его в `ACC_06` (0x122, для мотора), `ACC_07` (0x12E, для ESP) на 50 Гц и
`ACC_02` (0x30C, приборка) на 16.7 Гц. Поле `ACC_Sollbeschleunigung_02` несёт запрос шагами 0.005 м/с² от
−7.22; **3.01** — слово шины для «ничего не просим», а панда проверяет, что `ACC_Folgebeschl` в каждом
`ACC_07` читается ровно 3.02. Счётчик и CRC следуют рецепту `HCA_01`, с секретными таблицами на каждое
сообщение (`ACC_06` и `ACC_07` — те два кадра MQB, у которых секрет меняется со счётчиком).

Всё связывает один флаг: `FLAG_VOLKSWAGEN_LONG_CONTROL` в параметре безопасности панды. С ним панда
принимает наши три кадра **и перестаёт форвардить радарные** — перехват одним переключателем, поэтому
`vehicle.long_control: true` выставляет и флаг, и кадры, а выключенный оставляет штатный ACC нетронутым. Что
флагу нужно на машине, честно сказать: панда на разъёме гейтвея (на камерном разъёме кадры радара через неё
не проходят), машина с ACC-радаром, чтобы мотор и ESP были закодированы под запрос, и прошивка панды, собранная
с `ALLOW_DEBUG`. У нашего Golf радара нет — поэтому дорожные данные этой главы взяты из симулятора.

## Замкнуть контур в MetaDrive

Стенд играет машину: скриптованный лид, движущийся по полосе с заданной скоростью, и актуатор, превращающий
запрос ускорения в педальную ось MetaDrive (измеренный feedforward — газ 0.3 даёт 2.84 м/с² на этой машине
— плюс небольшой внутренний контур, как ECU замыкает свой):

```bash
cd scripts
python3 -m sim.eval --track long_straight --scenario lead_const --speed 25    # steady 15 m/s lead
python3 -m sim.eval --track long_straight --scenario lead_brake --speed 22    # lead 20 → 5 m/s at −2
python3 -m sim.eval --track long_straight --scenario lead_stop  --speed 20    # lead brakes to a stop
python3 -m sim.eval --track long_straight --scenario stationary --speed 20    # stopped car 120 m ahead
python3 -m sim.eval --track long_straight --scenario none --long --speed 25   # cruise alone
```

| сценарий | мин. зазор | зазор в конце | желаемый | сильнейшее торможение | итог |
|---|---|---|---|---|---|
| круиз без лида, уставка 25 м/с | — | — | — | −0.02 | 25.0 м/с удержаны, достигнуты за 22.6 с, рывок p95 0.16 м/с³ |
| лид 15 м/с, ego с 25 | 27.7 м | 27.7 м | 27.7 м | −2.42 | оседает ровно на желаемом зазоре |
| лид 20 → 5 м/с при −2 | 10.3 м | 13.3 м | 13.3 м | −2.52 | TTC не ниже 4.2 с |
| лид 15 → 0 при −2.5 | 4.6 м | 4.6 м | 6.0 м | −2.90 | остановка; удержание `stopping` |
| стоящая машина в 120 м | 5.4 м | 5.4 м | 6.0 м | −2.44 | остановка; TTC мин. 3.1 с |

```{figure} figures/long_sim_scenarios.png
---
width: 95%
---
Пять встреч в MetaDrive: без контакта, конверт панды не тронут, зазоры на стоянке внутри стоп-дистанции 6 м.
```

## Где ломается

* **Превью кривизны принимает шум за поворот.** С измеренными 0.15 м разброса пути на 20 м квадратичная
  аппроксимация 25 м пути говорит $R \approx 200$ м на прямой, и первая версия тихо режала уставку до 19 м/с.
  Одно длинное окно, медиана, НЧ-фильтр 2 с и мёртвая зона при $R < 250$ м лечат это в симуляторе — на дороге
  смотрите `v_curv` в `control/long_plan`, прежде чем доверять.
* **Лид — зрение, не радар.** `long_plan.lead_source` называет это явно — `vision` сегодня, `none`, чтобы
  планировать только по уставке, `radar` зарезервирован (ни одна платформа не декодирует объекты радара;
  он падает в vision и говорит об этом). У лида модели нет доплера; его скорость и ускорение выведены. `prob 0.5` —
  гейт, `prob 0.9` — гейт FCW, а лид дальше 2 м от нашего пути игнорируется. Безрадарный фолбэк upstream —
  тот же, но у upstream на большинстве машин ещё и радар.
* **Модель актуатора.** В симуляторе ускорение становится педалью через калиброванную карту; в машине это
  делают мотор и ESP со своей динамикой. Задержка 0.15 с — число upstream для VW, не измеренное на этой
  машине.
* **Солвер.** Гаусс–Ньютон — не SQP-RTI; тесты закрепляют формулировку (дистанции, сетка, границы) и
  поведение (осесть на желаемом зазоре, остановиться на стоп-дистанции), а не точные числа upstream.

## Приёмка

* Игрушка 1 воспроизводится: закон только по зазору подходит к лиду ближе, чем демпфированный;
* фрагмент MPC тормозит в первые 1.2 с за тормозящим лидом и никогда не планирует сквозь него;
* круизный фрагмент ускоряется на потолке, не выше;
* фрагмент закона управления возвращает постоянное ускорение плана ровно;
* по одному предложению: почему уставка — препятствие и почему `stay_stopped` не должен читать скорость.

## Упражнение

1. Поменяйте `T_FOLLOW` на 1.0 и 1.8 во фрагменте MPC — как сдвигается точка наибольшего сближения?
2. Обнулите `A_CHANGE_COST` (уберите член) и перерешайте с плана $a=1$: что происходит с $j$?
3. Дайте круизному препятствию уставку *ниже* текущей скорости — какой узел связывается первым?
4. В `sim.eval` запустите `lead_brake` с `--vision-latency-ms 300`: где проявится лишняя задержка?

## Куда дальше

* [MPC и fp](./MPC_and_FP.md) — поперечный оптимизатор; идея горизонта та же, объект — нет.
* [Платформа](../Platform/Overview.md) — кадры, счётчики и супервизор панды, общие с ACC-путём.
* [Предупреждения](../Safety/Warnings.md) — FCW, который поднимает этот планировщик, и тот, что независимо
  поднимает `safety_warn`.

<!-- next-chapter -->
---

**Дальше:** [Модель машины (недоворот)](../Control/VehicleModel.md)
