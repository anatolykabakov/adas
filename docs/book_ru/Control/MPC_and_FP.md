# MPC и fp

Pure Pursuit выбирает **один** $\delta$, попадающий в точку взгляда.
**MPC** оптимизирует **короткую будущую траекторию**, применяет первую команду и решает заново на следующем кадре.

| `lane_keep_controller` | роль |
|---|---|
| `mpc` | MPC пути из VisionPilot (лучше всего для изучения стоимости и $\kappa$) |
| `fp` | MPC в области времени, как у штатных систем (**по умолчанию на дороге**) |

Приложение с разбором настоящих дуг: `docs/MPC_EXPLAINED.md`.

* VisionPilot `mpc`: `visionpilot/lateral_planning.cpp` ($N{=}20$, шаг по пути $ds$).
* flowpilot `fp`: `flowpilot/lateral_mpc.cpp` ($N{=}16$, сетка по времени → 2.5 с).
* Слияние пути (разметка ↔ план): `laneLinesToPath` / `core/path_fusion.py`.

## Обозначения состояния

| обозначение | смысл |
|---|---|
| CTE | поперечное смещение от центра полосы, м |
| epsi | ошибка курса относительно касательной к пути, рад |
| $\kappa$ | кривизна пути, 1/м |
| $\delta$ | угол переднего колеса, рад |
| $L$ | база (~2.64–2.67 м) |

**Прямая:** CTE, epsi, $\kappa\approx 0$ → доминирует **обратная связь**.
**Дуга:** доминирует упреждение от $\kappa$. Плохая или запоздавшая $\kappa$ → снос наружу.

```{figure} figures/57_run0801_curve.png
---
width: 95%
---
Когда $\kappa$ планировщика отстаёт от кривизны по рыску, SWA недобирает и CTE растёт.
```

---

## Семь шагов (мысленная модель VisionPilot)

Общие игрушечные константы для всех примеров ниже:

```python
import math
import numpy as np

L = 2.64          # wheelbase [m]
N = 20            # horizon steps
DS = 0.5          # path step [m] (~1 s preview at ~10 m/s)
K_US = 0.0015     # understeer-ish term in VisionPilot ff
FF_SCALE = 2.0
W_DELTA = 45000.0 # pin δ to δ_ff
W_DDELTA = 15000.0
```

### 1. Измерить CTE, epsi, $\kappa$

Разметка → полилиния центра полосы → квадратичная $y = a x^2 + b x + c$ в системе машины.
У машины ($x\approx 0$):

* $\mathrm{CTE} \approx c$ (поперечное смещение),
* $\mathrm{epsi} \approx b$ (курс относительно пути),
* $\kappa \approx 2a$ (кривизна подгонки).

Если окно подгонки короткое или шумное, $\kappa$ выходит **малой и запоздавшей**, и всю эту ошибку наследует вся упреждающая цепочка.

```python
def fit_lane_state(x, y):
    """Quadratic fit y≈a x²+b x+c → (CTE, epsi, kappa) at x=0."""
    a, b, c = np.polyfit(x, y, 2)
    return float(c), float(b), float(2 * a)

# Toy centerline: gentle left arc in device frame (y right+)
x = np.linspace(0, 30, 31)
kappa_true = 0.02
y = 0.5 * kappa_true * x**2 + 0.05 * x + 0.30   # also 30 cm off + slight epsi
cte, epsi, kappa = fit_lane_state(x, y)
print(f"CTE={cte:.3f} m, epsi={math.degrees(epsi):.2f}°, κ={kappa:.4f} 1/m "
      f"(true κ={kappa_true})")
```

Вы должны получить CTE ≈ 0.3 м и $\kappa$ около 0.02. На настоящем беге сравните κ из `control/lane_keep_debug` с `yaw_rate / v`.

### 2. Предсказать траектории для пробного $\delta$

VisionPilot катит **по длине пути**, а не по часам. Для кандидата $\delta$:

$$
\begin{aligned}
\mathrm{CTE} &\leftarrow \mathrm{CTE} - \sin(\mathrm{epsi})\, ds, \\
\mathrm{epsi} &\leftarrow \mathrm{epsi} + \big(\kappa - \tfrac{\tan\delta}{L}\big)\, ds.
\end{aligned}
$$

* Нос повёрнут относительно дороги ($\mathrm{epsi}\neq 0$) → CTE интегрируется.
* $\tan\delta/L$ должен совпасть с $\kappa$ дороги, иначе интегрируется ошибка курса.

```python
def predict(cte, epsi, kappa, delta, ds=DS, n=N, L=L):
    """Return arrays (cte[0..n], epsi[0..n]) including the start state."""
    cs, es = [cte], [epsi]
    for _ in range(n):
        cte = cte - math.sin(epsi) * ds
        epsi = epsi + (kappa - math.tan(delta) / L) * ds
        cs.append(cte)
        es.append(epsi)
    return np.array(cs), np.array(es)

cte0, epsi0, kappa = 0.40, 0.0, 0.02
delta_ff_kin = math.atan(L * kappa)

for name, delta in [("δ=0", 0.0), ("δ=atan(Lκ)", delta_ff_kin)]:
    cs, _ = predict(cte0, epsi0, kappa, delta)
    print(f"{name:12s} CTE@0={cs[0]:.3f} → CTE@end={cs[-1]:.3f} m")
```

При неверном $\delta$ CTE растёт; геометрический $\delta$ держит лучше. Это только **предсказатель** — какой $\delta$ победит, решает стоимость.

### 3. Оценить через $J$

Меньше — лучше. Слагаемые в духе VisionPilot (упрощённо):

$$
J = \sum_s \Big(
w_c\,\mathrm{CTE}_s^{2} + w_c\,s_4\,\mathrm{CTE}_s^{4} + w_e\,\mathrm{epsi}_s^{2}
+ W_\delta(\delta_s - \delta_{\mathrm{ff},s})^{2}
+ W_{\Delta}(\delta_s - \delta_{s-1})^{2}
\Big).
$$

Веса растут с $|\kappa|$ (`curve_factor = 1 + 20·κ`). Слагаемое $W_\delta{=}45000$ огромно, поэтому оптимум прижимается к $\delta_{\mathrm{ff}}$.

```python
def cost(cte0, epsi0, kappa, deltas, v=15.0,
         cte_w_base=20.0, epsi_w_base=10.0, quartic=5.0):
    """Score a sequence of wheel angles (len N)."""
    assert len(deltas) == N
    v2 = v * v
    curve = 1.0 + 20.0 * abs(kappa)
    cte_w = cte_w_base * curve
    epsi_w = epsi_w_base * curve
    cte, epsi = cte0, epsi0
    j = 0.0
    for s, delta in enumerate(deltas):
        j += cte_w * cte**2
        j += cte_w * quartic * cte**4
        j += epsi_w * epsi**2
        delta_ff = FF_SCALE * (math.atan(L * kappa) + K_US * v2 * kappa)
        j += W_DELTA * (delta - delta_ff) ** 2
        if s > 0:
            j += W_DDELTA * min(max(v2, 9.0), 25.0) * (delta - deltas[s - 1]) ** 2
        # one predict step
        cte = cte - math.sin(epsi) * DS
        epsi = epsi + (kappa - math.tan(delta) / L) * DS
    return j

kappa = 0.02
delta_ff = FF_SCALE * (math.atan(L * kappa) + K_US * 15**2 * kappa)
for scale in (0.0, 0.5, 1.0, 1.5):
    deltas = [scale * delta_ff] * N
    j = cost(0.20, 0.0, kappa, deltas)
    print(f"δ={math.degrees(scale*delta_ff):5.2f}°  J={j:.1f}")
```

Минимум ожидается около `scale=1` ($\delta\approx\delta_{\mathrm{ff}}$). Мысленно поставьте `W_DELTA=0`: порядок может перевернуться в сторону $\delta$, минимизирующего чистую CTE.

### 4. Построить $\delta_{\mathrm{ff}}$

Упреждение — это руление, которое нужно, **даже если вы уже по центру**:

$$
\delta_{\mathrm{ff}} = f_{\mathrm{scale}}\big(\arctan(L\kappa) + K_{\mathrm{us}} v^{2}\kappa\big).
$$

Оно **линейно по $\kappa$**. Недооценили $\kappa$ → недокрутили дугу. На прямой $\kappa\approx 0$ → $\delta_{\mathrm{ff}}\approx 0$, и машину везёт обратная связь.

```python
def delta_ff(kappa, v, ff_scale=FF_SCALE, L=L, K_us=K_US):
    return ff_scale * (math.atan(L * kappa) + K_us * v * v * kappa)

for kappa in (0.0, 0.01, 0.02, 0.03):
    d = delta_ff(kappa, v=20.0)
    # What if perception returns only 66% of true kappa?
    d_bad = delta_ff(0.66 * kappa, v=20.0)
    print(f"κ={kappa:.3f}  δ_ff={math.degrees(d):5.2f}°  "
          f"if κ×0.66 → {math.degrees(d_bad):5.2f}°")
```

### 5. Решить (тёплый старт и градиент)

Затравка, затем шаги по конечным разностям или градиенту по последовательности $\delta$ (в C++ около 80 итераций). На практике на дорожных бегах спуск почти не двигает затравку — **качество затравки ≈ качество выхода**.

$$
\delta_{\mathrm{seed}} = \delta_{\mathrm{ff}} + \underbrace{k_{\mathrm{cte}}\mathrm{CTE} + k_{\mathrm{epsi}}\mathrm{epsi}}_{\text{обратная связь, зажим ~±0.25 рад}}.
$$

```python
def warm_start(cte, epsi, kappa, v, k_cte_base=0.6, k_epsi=0.3):
    v2 = v * v
    k_cte = max(k_cte_base / (1.0 + v2), 0.0)
    fb = max(-0.25, min(0.25, k_cte * cte + k_epsi * epsi))
    d0 = delta_ff(kappa, v) + fb
    return [d0] * N

def solve(cte, epsi, kappa, v, iters=40, step=2e-4):
    """Tiny FD gradient descent on a shared δ (toy; C++ optimizes the sequence)."""
    deltas = warm_start(cte, epsi, kappa, v)
    best = cost(cte, epsi, kappa, deltas, v=v)
    for _ in range(iters):
        # one shared angle for the demo
        d = deltas[0]
        trial_p = [d + step] * N
        trial_m = [d - step] * N
        jp = cost(cte, epsi, kappa, trial_p, v=v)
        jm = cost(cte, epsi, kappa, trial_m, v=v)
        grad = (jp - jm) / (2 * step)
        d_new = d - 5e-8 * grad          # tiny step — cost is stiff in δ
        deltas = [d_new] * N
        j = cost(cte, epsi, kappa, deltas, v=v)
        if j < best:
            best = j
        else:
            deltas = [d] * N
            break
    return deltas, best

cte, epsi, kappa, v = 0.20, 0.02, 0.02, 18.0
seed = warm_start(cte, epsi, kappa, v)
sol, j = solve(cte, epsi, kappa, v)
print(f"seed {math.degrees(seed[0]):.3f}° → solved {math.degrees(sol[0]):.3f}°  J={j:.1f}")
print(f"|Δδ|={abs(math.degrees(sol[0]-seed[0])):.4f}°  (often ~0 on road)")
```

### 6. Применить первый отсчёт → ограничения → SWA → PID → HCA

Взять команду чуть впереди по горизонту (задержка), затем зажать:

1. предел угла, растущий со скоростью,
2. ограничение скорости изменения на кадр,
3. $\delta \times$ `steer_ratio` → угол рулевого колеса,
4. угловой PID → момент $\in[\pm 300]$ cNm → `HCA_01` на CAN.

```python
STEER_RATIO = 15.7

def apply_limits(delta_cmd, delta_prev, v, max_slew_deg=8.0):
    # soft VisionPilot-ish angle cap [deg]
    cap = math.radians(max(8.0, min(25.0, 8.0 + v)))
    d = max(-cap, min(cap, delta_cmd))
    slew = math.radians(max_slew_deg)
    d = max(delta_prev - slew, min(delta_prev + slew, d))
    swa = d * STEER_RATIO
    return d, swa

delta_plan = sol[1] if len(sol) > 1 else sol[0]   # "look ahead" one sample
d_prev = 0.0
d_out, swa = apply_limits(delta_plan, d_prev, v=18.0)
print(f"wheel {math.degrees(d_out):.2f}°  SWA {math.degrees(swa):.1f}°  "
      f"(PID→torque happens in LaneKeepService / PandaService)")
```

Насыщение на ±300 cNm означает, что **машина** не может исполнить, а не что стоимость хотела больше. Отладочный флаг и индикация в интерфейсе: предел руления.

### 7. Повторить на следующем кадре

Новые `vision/lanes` (медиана ~80–90 мс на телефоне) → новые CTE/epsi/$\kappa$ → снова шаг 1.
MPC **не** гонит всю секунду в открытом контуре: применяется только первая команда, затем план выбрасывается и строится заново от **измеренного** состояния.

```python
def one_phone_frame(cte, epsi, kappa, v, delta_prev):
    """Steps 4→6 for one camera frame (measure assumed already done)."""
    deltas = warm_start(cte, epsi, kappa, v)
    # real code runs gradient; seed≈solution on many bags
    d_plan = deltas[1] if len(deltas) > 1 else deltas[0]
    d_out, swa = apply_limits(d_plan, delta_prev, v)
    return d_out, swa

# Structural MPC loop — new measurement every frame, plan discarded
delta_prev = 0.0
for frame in range(5):
    # step 1 would refresh cte, epsi, kappa from vision here
    cte, epsi, kappa, v = 0.20, 0.01, 0.02, 18.0
    delta_prev, swa = one_phone_frame(cte, epsi, kappa, v, delta_prev)
    print(f"frame {frame}: δ={math.degrees(delta_prev):.2f}°  SWA={math.degrees(swa):.1f}°")

# Why κ quality matters more than "more MPC iterations"
print("\ncommand vs measured κ (CTE=epsi=0):")
for k_meas in (0.020, 0.013, 0.008):  # truth, ×0.66, worse
    d = warm_start(0.0, 0.0, k_meas, 20.0)[0]
    print(f"  κ_meas={k_meas:.3f} → δ_seed={math.degrees(d):.2f}°")
```

Применяется только первая команда; остальной горизонт выбрасывается. Если $\kappa$ занижена на каждом кадре, то каждая затравка недобирает — спуск не может выдумать кривизну, которую шаг 1 никогда не измерил.

---

## До контроллера: слияние разметки и плана

И `fp`, и `mpc` / `pp` следят за **эталонной полилинией** на топике `vision/path`.
Эта полилиния — **не** сырая разметка и **не** сырой план Supercombo, а слияние в
`laneLinesToPath` (`topic_convert`, зеркало на Python `core/path_fusion.py`).

### Зачем сливать?

| источник | сильная сторона | режим отказа |
|---|---|---|
| **Линии разметки** (ближние Л/П) | геометрический центр той полосы, в которой вы едете | исчезают, σ взлетает на дугах и в дождь |
| **План модели** (`plan_x/y`) | всегда есть, гладкий | систематически **срезает внутрь** дуги (смещение уставки) |

Upstream (comma-two) доверяет разметке, когда уверен (`d_prob≈1`). Мы выносим пропорцию как
`path_lane_blend_scale` (поставляется **0.6**): даже при идеальных линиях 40 % смещения плана остаётся.

### Формула (система device, $y$ вправо)

1. Мягкие вероятности от ближних левой и правой (`lanes[1]`, `lanes[2]`), затем × уверенность по σ
   (медиана `y_std` на 5–40 м: полный вес при $\sigma\le$ `lane_std_good_m` 0.3,
   ноль при $\sigma\ge$ `lane_std_bad_m` 1.5).
2. Нужны обе линии и медианная ширина $\in[2.6, 4.6]$ м → **anchored**.
3. Середина полосы на каждом $x$:

$$
y_{\mathrm{lane}} = \frac{p_L(y_L + w/2) + p_R(y_R - w/2)}{p_L + p_R},\quad
w=\mathrm{clip}(|y_L-y_R|).
$$

4. Вес подмешивания (ИЛИ вероятностей в духе flowpilot, затем масштаб):

$$
d_{\mathrm{prob}} = (p_L + p_R - p_L p_R)\cdot\texttt{path\_lane\_blend\_scale}.
$$

5. Слияние с точками плана:

$$
y = d_{\mathrm{prob}}\, y_{\mathrm{lane}} + (1 - d_{\mathrm{prob}})\, y_{\mathrm{plan}}.
$$

Если не anchored или `blend_scale=0` → **только план**.

```python
import numpy as np

def soft_prob(p, min_p=0.3):
    if p < min_p:
        return 0.0
    return min(1.0, (p - min_p) / max(1e-3, 1.0 - min_p))

def std_conf(median_std, good=0.3, bad=1.5):
    if median_std < 0:
        return 1.0
    if median_std <= good:
        return 1.0
    if median_std >= bad:
        return 0.0
    return (bad - median_std) / (bad - good)

def blend_reference(x, y_plan, y_L, y_R, p_L, p_R,
                    std_L=0.2, std_R=0.2, blend_scale=0.6,
                    w_min=2.6, w_max=4.6):
    """Return fused y(x). All arrays same length; device y right+."""
    p_L = soft_prob(p_L) * std_conf(std_L)
    p_R = soft_prob(p_R) * std_conf(std_R)
    sel = (x >= 5.0) & (x <= 40.0)
    w_med = float(np.mean(np.abs(y_R[sel] - y_L[sel]))) if sel.any() else 0.0
    anchored = (w_min < w_med < w_max) and p_L > 0 and p_R > 0
    d_prob = (p_L + p_R - p_L * p_R) * blend_scale if anchored else 0.0

    w = np.clip(np.abs(y_L - y_R), w_min, w_max)
    y_lane = (p_L * (y_L + 0.5 * w) + p_R * (y_R - 0.5 * w)) / (p_L + p_R + 1e-6)
    y = d_prob * y_lane + (1.0 - d_prob) * y_plan
    return y, d_prob, anchored

# Toy: plan cuts 40 cm inside; paint is centered
x = np.linspace(0, 40, 21)
y_plan = -0.40 * (x / 40.0)          # drifts to −0.4 m at 40 m
y_L = -1.75 * np.ones_like(x)
y_R = +1.75 * np.ones_like(x)

for scale in (0.0, 0.6, 1.0):
    y, d, ok = blend_reference(x, y_plan, y_L, y_R, 0.95, 0.95, blend_scale=scale)
    print(f"blend={scale:.1f} anchored={ok} d_prob={d:.2f}  "
          f"y(0)={y[0]:+.3f}  y(40)={y[-1]:+.3f}")
```

Ожидается: `blend=0` следует срезающему плану; `blend=1` сидит на середине полосы ($≈0$);
`blend=0.6` посередине — поставляемое значение для телефона.

```{admonition} Ручки
:class: tip
`path_lane_blend_scale`, `lane_std_good_m` / `lane_std_bad_m`, `path_camera_offset_m`
(постоянный сдвиг после слияния). Поднимать подмешивание к 1.0 помогает дугам **только если** σ остаётся в
«хорошей» полосе — иначе $d_{\mathrm{prob}}\to 0$ и вы снова молча едете по плану.
```

---

## Модель `fp` (MPC flowpilot в области времени)

По умолчанию на дороге (`lane_keep_controller=fp`). Код: `flowpilot/lateral_mpc.cpp`,
подключение в `LaneKeepService::stepFlowpilot`.

Противоположность $\delta$-MPC VisionPilot в **области пути**:

| | VisionPilot `mpc` | flowpilot `fp` |
|---|---|---|
| переменная решения | угол колеса $\delta$ по $s$ | производная скорости рыска $u=\dot r$ во **времени** |
| горизонт | $N{=}20$, $ds\sim 0.5$ м | $N{=}16$, $t_i=10(i/32)^2$ → **$T_f=2.5$ с** |
| выход | $\delta$ | желаемая **кривизна** $\kappa^\star$, затем модель машины → $\delta$ |
| задержка | взять $\delta_1$ | `lagAdjustedCurvature` на `fp_steer_delay_s` |

### fp-1. Эталон во времени

Полилиния переворачивается в систему openpilot с $y$ влево (`y \leftarrow -y`). Отсчёты берутся на расстояниях $v\cdot t_i$:

$$
t_i = 10\left(\frac{i}{32}\right)^{2},\quad i=0\ldots 16.
$$

Строятся $y_{\mathrm{ref}}(t)$, $\psi_{\mathrm{ref}}(t)$, $r_{\mathrm{ref}}=\dot\psi_{\mathrm{ref}}$.
Если головы ориентации Supercombo есть, $\psi$ и $r$ берутся из `plan_yaw` и `plan_yaw_rate`.

```python
import numpy as np

N_FP = 16
T_IDX_MAX = 32.0

def t_node(i):
    return 10.0 * (i / T_IDX_MAX) ** 2

print("fp time grid [s]:", [round(t_node(i), 3) for i in range(0, N_FP + 1, 4)])
# 0, 0.156, 0.625, 1.406, 2.5

def sample_y_ref(poly_xy, v):
    """poly_xy: Nx2 in left+ frame; return y_ref[0..N]."""
    x, y = poly_xy[:, 0], poly_xy[:, 1]
    s = np.zeros(len(x))
    s[1:] = np.cumsum(np.hypot(np.diff(x), np.diff(y)))
    y_ref = []
    for i in range(N_FP + 1):
        dist = min(v * t_node(i), s[-1])
        y_ref.append(float(np.interp(dist, s, y)))
    return np.array(y_ref)

# Straight offset 0.25 m left in left+ frame
poly = np.stack([np.linspace(1, 40, 40), 0.25 * np.ones(40)], axis=1)
print("y_ref@v=15:", np.round(sample_y_ref(poly, 15.0)[::4], 3))
```

### fp-2. Динамика (состояние)

Состояние $x=(X,Y,\psi,r)$, управление $u=\dot r$ (угловое ускорение рыска):

$$
\begin{aligned}
\dot X &= v\cos\psi - R_{\mathrm{rot}}\sin\psi\cdot r,\\
\dot Y &= v\sin\psi + R_{\mathrm{rot}}\cos\psi\cdot r,\\
\dot\psi &= r,\\
\dot r &= u.
\end{aligned}
$$

$R_{\mathrm{rot}}\approx L/2$ в нашем конфиге. Машина начинается в начале координат; $r_0$ засевается из геометрии пути при первом вызове.

```python
def forward_fp(u, v, r0=0.0, R_rot=1.32):
    """u length N; return Y, psi, r trajectories length N+1."""
    X = Y = psi = 0.0
    r = r0
    Ys, psis, rs = [Y], [psi], [r]
    for i in range(N_FP):
        dt = max(t_node(i + 1) - t_node(i), 1e-4)
        X = X + dt * (v * np.cos(psi) - R_rot * np.sin(psi) * r)
        Y = Y + dt * (v * np.sin(psi) + R_rot * np.cos(psi) * r)
        psi = psi + dt * r
        r = r + dt * u[i]
        Ys.append(Y); psis.append(psi); rs.append(r)
    return np.array(Ys), np.array(psis), np.array(rs)
```

### fp-3. Стоимость

С $v_{\mathrm{off}}=v+\texttt{speed\_offset}$ (по умолчанию +10):

$$
J = \sum_i \Big(
w_y(Y_i-y_{\mathrm{ref}})^2
+ w_\psi v_{\mathrm{off}}^2(\psi_i-\psi_{\mathrm{ref}})^2
+ w_r v_{\mathrm{off}}^2(r_i-r_{\mathrm{ref}})^2
+ w_{\mathrm{jerk}}(v_{\mathrm{off}} u_i)^2
+ w_{\mathrm{srate}}(u_i/(v+0.1))^2
\Big).
$$

Примерно поставляемые веса: `path_weight=1`, `heading_weight=0.11`, `lat_jerk_weight=0.05`,
`fp_steering_rate_weight=400`.

```python
def cost_fp(u, y_ref, v, w_y=1.0, w_psi=0.11, w_srate=400.0, speed_offset=10.0):
    Y, psi, r = forward_fp(u, v)
    v_off = v + speed_offset
    j = 0.0
    psi_ref = np.zeros_like(y_ref)   # toy: straight
    for i in range(N_FP + 1):
        j += w_y * (Y[i] - y_ref[i]) ** 2
        j += w_psi * (v_off * (psi[i] - psi_ref[i])) ** 2
        if i < N_FP:
            j += w_srate * (u[i] / (v + 0.1)) ** 2
    return j

v = 15.0
y_ref = sample_y_ref(poly, v)
# Path-only view (w_srate=0): small +u pulls Y toward +0.25
for u0 in (0.0, 0.01):
    u = np.full(N_FP, u0)
    Y, _, _ = forward_fp(u, v)
    j_path = cost_fp(u, y_ref, v, w_srate=0.0)
    j_full = cost_fp(u, y_ref, v, w_srate=400.0)
    print(f"u={u0:+.2f}  Y_end={Y[-1]:+.3f}  J_path={j_path:.3f}  J_full={j_full:.3f}")
```

$u{=}0.01$ снижает стоимость пути и заканчивает около эталонных 0.25 м; полное $J$ всё равно растёт из-за слагаемого по скорости — настоящий градиентный спуск балансирует оба (поставляется `fp_steering_rate_weight=400`).
### fp-4. Решение

Тёплый старт по предыдущему $u$; около 50 итераций конечно-разностного градиентного спуска с линейным поиском (`gd_step=0.1`).
Дух тот же, что у VisionPilot: хорошая затравка важна, горизонт короткий.

### fp-5. Кривизна с поправкой на запаздывание (трюк с задержкой)

После решения читается траектория скорости рыска $r(t)$ и строится

$$
\kappa_0 = \frac{r(0)}{v},\qquad
\kappa_{\mathrm{avg}} = \frac{\psi(t_d)}{v\, t_d},\qquad
\kappa^\star = 2\kappa_{\mathrm{avg}} - \kappa_0,
$$

где $t_d=\texttt{fp\_steer\_delay\_s}$ (поставляется **0.35** с), затем $\kappa^\star$ ограничивается по скорости через
`max_lateral_jerk / v²`. Это порт `get_lag_adjusted_curvature` из openpilot:
командовать ту кривизну, которая нужна **в момент, когда рейка действительно двинется**.

```python
def lag_adjusted_kappa(psi_sol, r_sol, v, delay_s=0.35, dt_s=0.08, max_jerk=5.0):
    ts = np.array([t_node(i) for i in range(N_FP + 1)])
    kappa0 = r_sol[0] / v
    psi_d = float(np.interp(delay_s, ts, psi_sol))
    avg = psi_d / (v * delay_s)
    desired = 2.0 * avg - kappa0
    max_rate = max_jerk / (v * v)
    return float(np.clip(desired, kappa0 - max_rate * dt_s, kappa0 + max_rate * dt_s))

# Constant-r turn: psi(t)=r0*t → avg=kappa0 → κ*=kappa0 (no change)
r0 = 0.02 * 15.0
ts = np.array([t_node(i) for i in range(N_FP + 1)])
psi = r0 * ts
r = np.full_like(ts, r0)
print("steady turn κ*:", round(lag_adjusted_kappa(psi, r, v=15.0), 5))
```

### fp-6. $\kappa^\star \to$ угол колеса → PID → HCA

```text
κ*  →  vehicle_model (if lat_use_vehicle_model)  →  δ
    →  angle slew / caps  →  × steer_ratio  →  angle-PID  →  HCA torque
```

Знак: в системе device $y$ вправо; сервис публикует `steer_rad = -steerFromCurvature(...)`.
Детали масштабирования недокрута — в главе [про модель машины](./VehicleModel.md).

### fp-7. Следующий кадр зрения

Решаем заново с измеренным `frame_dt` (а не с фиксированными 0.05). Угловой PID на телефоне идёт от
`vehicle/state` (около 100 Гц после приёма CAN за 10 мс); темп планировщика остаётся ограничен зрением (около 12.5 Гц).

---

## Где горизонт не помогает

Горизонт снимает информационный предел Pure Pursuit — он читает весь путь, а не одну точку. Но не всё, и три
вещи, которые он не снимает, каждая была измерена, а не выведена из рассуждений.

### 1. Устойчивое смещение почти бесплатно в функции стоимости

`fp` сбрасывает состояние каждый кадр: $x_0 = [0, 0, 0, \dot\psi]$. Машина по определению находится точно на
своём эталоне в нулевом узле. Значит наказать устойчивое смещение можно только через *дальние* узлы — и вот там
сетка времени работает против вас.

```python
V = 15.0
print(f"{'node':>5} {'t, s':>7} {'distance, m':>12} {'lateral reach, m':>17}")
for i in (0, 1, 2, 3, 4, 6, 8, 12, 16):
    t = t_node(i)
    d = V * t
    # How far sideways can the car physically move by then, at a generous 3 m/s^2 of lateral accel?
    reach = 0.5 * 3.0 * t * t
    print(f"{i:>5} {t:>7.3f} {d:>12.2f} {reach:>17.3f}")
```

Прочитайте колонку доступного бокового смещения. За первые четыре узла машина не может сдвинуться боком больше
чем на пару сантиметров, что бы руль ни делал, — поэтому 0.35 м смещения в этих узлах не стоимость, на которую
оптимизатор может подействовать, а константа. Узлы, где боковое движение возможно, лежат достаточно далеко, и
там доминирует ошибка кривизны.

Измеренное следствие: машина сидела **0.35 м внутри своего же эталона** на дугах. И это не вес, который надо
подстроить: прогоны в замкнутом контуре подтвердили, что ни `fp_steering_rate_weight` (150 против 800), ни
`fp_steer_delay_s` (0.23 против 0.35) этого не меняют. Устойчивое смещение убирает интегральное слагаемое,
которого в этой постановке нет, либо уже сдвинутый эталон — что и делает
подмешивание разметки, несколькими разделами выше.

### 2. Удлинять горизонт тоже не помогает

Очевидный ответ — «взять N = 32, как у flowpilot, вместо наших 16». Проверено, и из этого не следует, потому
что сетка квадратичная:

```python
first_second = sum(1 for i in range(N_FP + 1) if t_node(i) <= 1.0)
print(f"our N=16: {first_second} of {N_FP + 1} nodes inside the first second, horizon {t_node(N_FP):.2f} s")
first_second_32 = sum(1 for i in range(33) if 10.0 * (i / 32.0) ** 2 <= 1.0)
print(f"N=32    : {first_second_32} of 33 nodes inside the first second, horizon "
      f"{10.0 * (32 / 32.0) ** 2:.2f} s")
print("\nThe near zone is already as densely sampled as theirs. Doubling N adds far nodes, which at equal")
print("weights dilutes the near zone that actually matters, and costs about 4x more to solve because we")
print("use gradient descent rather than acados.")
```

### 3. У плана, который оптимизируется, своё смещение

MPC минимизирует расстояние до эталона. Если эталон неверен, он идеально следит за неверным. На дугах план
модели сидит в **+0.32 / −0.35 м** от центра полосы, и попытка убрать это одним коэффициентом кривизны
провалилась поучительно:

* параметризованный как постоянное расстояние ($50.2\kappa$), он был нестабилен даже внутри одного заезда —
  20.2 ниже 12 м/с против 77.2 выше;
* верная параметризация — упреждение по *времени*, $\tfrac{1}{2}\kappa(vT)^2$, и $T$ действительно устойчиво
  по скорости: 0.71–1.03 с;
* но не воспроизводимо **между** заездами: 0.84 с на одном, 0.56 с на другом, потому что у двух заездов был
  разный выученный рыск камеры (+1.12° против +0.24°), а он меняет warp входа, поэтому сеть видит другое
  изображение и выдаёт другой план.

Значит коэффициент нельзя запечь: на другой калибровке он добавит своё собственное смещение. Поэтому главным
рычагом стало подмешивание разметки, которому не нужно ни одного настраиваемого числа.

### 4. А когда момент насыщается, всё это неважно

На измеренных правых дугах момент сидел на потолке HCA ±300 cNm в **65 % кадров** (76 % на более раннем заезде,
80 % внутри одного эпизода длиной 12.3 с при R = 130 м). Пока он насыщен, обратной связи нет вовсе: выход
оптимизатора меняется, а рейка нет. Лечится это запросом меньшей кривизны, а не более усердным решением.

```{admonition} Что вынести из этого раздела
:class: tip
Каждый предел выше — своего рода. Один структурный (нет интегрального слагаемого), один — не-результат
(горизонт и так достаточно длинный), один вышестоящий (собственное смещение плана, связанное с калибровкой) и
один физический (актуатор). Понять, на какой из них вы смотрите, — это и есть основная работа, и ни один из них
не находится перебором весов.
```

## Ручки на телефоне

**Общие и путь**

* `path_lane_blend_scale`, `lane_std_good_m`, `lane_std_bad_m`, `path_camera_offset_m`
* `min_control_speed_mps`, `lat_use_vehicle_model`, `tire_stiffness_factor`

**`fp`**

1. `fp_steer_delay_s` — горизонт $\kappa$ с поправкой на запаздывание.
2. `fp_steering_rate_weight` — плавность $u$.
3. Измеренный `frame_dt` из метк времени.

**`mpc` (VisionPilot)**

`ff_scale`, `epsi_gain`, `cte_gain_*`, `mpc_kappa_yaw_blend`.

## Честное сравнение

Одно окно бега, темп зрения $\gtrsim 9$ Гц: |CTE| медиана/p95, |$\Delta$SWA|, прямая и дуга по отдельности.
Инструмент: `bag_config_sweep.py --vision-latency ...` (умеет перебирать и `--blend`).

## Задания

1. Шаги 2–3 VisionPilot: какой $\delta$ побеждает на дуге? Обнулите вес прижатия к $\delta$ — победитель сдвинулся?
2. Пример со слиянием: сколько от срезки плана в −0.4 м остаётся при `blend=0.6` на `y(40)`?
3. Уроните σ разметки до 1.6 м в игрушечном примере слияния — чему равно $d_{\mathrm{prob}}$?
4. `fp`: измените `delay_s` с 0.1 до 0.35 на траектории с растущим $r$; как двигается $\kappa^\star$?
5. По бегу: `pp` против `fp` против `mpc`; отключите модель машины один раз на дуге; переберите подмешивание 0.3 / 0.6 / 1.0.

<!-- next-chapter -->
---

**Дальше:** [FCW / AEB / LDW](../Safety/Warnings.md)
