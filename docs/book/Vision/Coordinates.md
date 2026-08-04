# Coordinate Systems

Wrong $y$ sign or wrong `steer_sign` **inverts steering**. Fix conventions before any controller math.

## Three representations

1. **Pixels** $(u,v)$ — image plane.
2. **Device / model** — Supercombo. $y$ **right-positive**.
3. **ISO 8855 vehicle** — `calibration.camera.*`. $y$ **left-positive**.

```{figure} figures/coordinate_systems.png
---
width: 90%
---
Always ask which $y$ a plot uses.
```

```{admonition} Remember
:class: tip
Supercombo: $y$ right+. ISO vehicle: $y$ left+. Mixing without an explicit flip inverts left/right.
```

## Math of a frame flip

If $y_{\mathrm{ISO}}$ is left+ and $y_{\mathrm{dev}}$ is right+,

$$
y_{\mathrm{dev}} = -\, y_{\mathrm{ISO}},
\qquad
y_{\mathrm{ISO}} = -\, y_{\mathrm{dev}}.
$$

Lane center in device frame from left/right paint (both in device $y$):

$$
y_{\mathrm{center}} = \tfrac{1}{2}\big(y_{\mathrm{left}} + y_{\mathrm{right}}\big).
$$

Cross-track error for a vehicle at $y=0$ in that frame is often taken as

$$
\mathrm{CTE} \approx - y_{\mathrm{center}}
$$

(sign depends on whether positive CTE means "car left of path" or the opposite — **pick one and keep it**).

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

## SWA sign on MQB

$$
\mathrm{SWA} = \texttt{steer\_sign}\cdot \delta \cdot \texttt{steer\_ratio},
\qquad \texttt{steer\_sign}=-1.
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

Flipping **only** $y$ or **only** `steer_sign` still yields inverted HCA. Both must be consistent.

## Pixel vs meter

A pixel is a **ray**, not a range. AAD recovers meters with IPM.
Here Supercombo already outputs meters; extrinsics still matter because they drive **input warp**. Wrong pitch → biased laterals → constant CTE.

## Worked checklist

| observation (device $y$ right+) | meaning |
|---|---|
| Left paint $y\approx -1.8$ m | expected |
| Left paint stably $y>0$ | sign/name bug |
| Path always wrong side | check `$y$` **and** `steer_sign` |

```python
def left_line_ok(y_left_samples, tol=0.5):
    """Most samples of left paint should be negative in device frame."""
    import numpy as np
    y = np.asarray(y_left_samples, float)
    return float(np.nanmedian(y)) < -tol

print(left_line_ok([-1.7, -1.9, -1.8, -2.0]))  # True
print(left_line_ok([+1.7, +1.9, +1.8]))         # False → investigate
```

## Exercise

1. Run the snippets; change signs until `left_line_ok` fails — that is the bug students hit in bags.
2. In a visualizer, state the frame of the bird's-eye plot in one sentence.
3. Predict offline: flip only `steer_sign` in config and replay PP — which way does the wheel go?

<!-- next-chapter -->
---

**Next:** [Supercombo on device](./Supercombo.md)
