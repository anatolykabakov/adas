# Системы координат

Неверный знак $y$ или неверный `steer_sign` **инвертируют руление**. Наведите порядок в договорённостях раньше, чем возьмётесь за математику контроллера.

## Три представления

1. **Пиксели** $(u,v)$ — плоскость изображения.
2. **Device / model** — Supercombo. $y$ **вправо-положительная**.
3. **ISO 8855, машина** — `calibration.camera.*`. $y$ **влево-положительная**.

```{figure} figures/coordinate_systems.png
---
width: 90%
---
Всегда уточняйте, в каком $y$ построен график.
```

```{admonition} Запомнить
:class: tip
Supercombo: $y$ вправо. Машина по ISO: $y$ влево. Смешать их без явного переворота — всё равно что поменять местами лево и право.
```

## Математика переворота системы

Если $y_{\mathrm{ISO}}$ влево-положительная, а $y_{\mathrm{dev}}$ вправо-положительная, то

$$
y_{\mathrm{dev}} = -\, y_{\mathrm{ISO}},
\qquad
y_{\mathrm{ISO}} = -\, y_{\mathrm{dev}}.
$$

Центр полосы в device-системе по левой и правой разметке (обе в device $y$):

$$
y_{\mathrm{center}} = \tfrac{1}{2}\big(y_{\mathrm{left}} + y_{\mathrm{right}}\big).
$$

Поперечную ошибку для машины в $y=0$ в этой системе часто берут как

$$
\mathrm{CTE} \approx - y_{\mathrm{center}}
$$

(знак зависит от того, означает ли положительная CTE «машина левее пути» или наоборот — **выберите что-то одно и держитесь этого**).

```python
# Device frame: y right-positive
y_left, y_right = -1.85, 1.90   # meters at some x ahead
y_center = 0.5 * (y_left + y_right)
cte = -y_center   # car at 0; positive CTE ⇒ path is left of car ⇒ steer left in right+ frame?
print(f"y_center={y_center:.3f} m, CTE={cte:.3f} m")

# Bug demo: someone treated ISO left+ numbers as device
y_left_iso = +1.85
y_center_wrong = 0.5 * (y_left_iso + (-1.90))
print(f"wrong center if signs mixed: {y_center_wrong:.3f} m")
```

## Знак SWA на MQB

$$
\mathrm{SWA} = \mathrm{steer\_sign}\cdot \delta \cdot \mathrm{steer\_ratio},
\qquad \mathrm{steer\_sign}=-1.
$$

```python
STEER_SIGN = -1
STEER_RATIO = 15.7

def swa(delta_deg: float) -> float:
    return STEER_SIGN * delta_deg * STEER_RATIO

# Model says "steer +δ toward +y (right in device frame)"
print("delta=+3° → SWA", swa(3.0), "deg on CAN")
print("If you forget steer_sign:", +3.0 * STEER_RATIO, "(WRONG sign on car)")
```

Перевернуть **только** $y$ или **только** `steer_sign` — и вы получите инвертированный HCA. Согласованными должны быть оба.

## Пиксель против метра

Пиксель — это **луч**, а не дальность. В AAD метры восстанавливают через IPM.
Здесь Supercombo отдаёт метры сразу; но экстринсики всё равно важны, ведь они задают **warp входа**. Неверный тангаж → смещённые поперечные величины → постоянная CTE.

## Готовый чеклист

| наблюдение (device, $y$ вправо) | что значит |
|---|---|
| левая разметка $y\approx -1.8$ м | так и должно быть |
| левая разметка устойчиво $y>0$ | ошибка в знаке или в имени |
| путь всегда не с той стороны | проверьте `$y$` **и** `steer_sign` |

```python
def left_line_ok(y_left_samples, tol=0.5):
    """Most samples of left paint should be negative in device frame."""
    import numpy as np
    y = np.asarray(y_left_samples, float)
    return float(np.nanmedian(y)) < -tol

print(left_line_ok([-1.7, -1.9, -1.8, -2.0]))  # True
print(left_line_ok([+1.7, +1.9, +1.8]))         # False → investigate
```

## Задания

1. Запустите примеры и меняйте знаки, пока `left_line_ok` не начнёт возвращать ложь, — именно на эту ошибку студенты и натыкаются в бегах.
2. В визуализаторе одним предложением назовите систему координат для вида сверху.
3. Сделайте офлайн-прогноз: переверните в конфиге только `steer_sign`, прогоните реплей PP — в какую сторону пойдёт руль?

<!-- next-chapter -->
---

**Дальше:** [Supercombo на устройстве](./Supercombo.md)
