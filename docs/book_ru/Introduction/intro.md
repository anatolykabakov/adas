# Алгоритмы ADAS на телефоне

Эта (мини-)книга ведёт вас по стеку **Android ADAS**, который используется в нашем проекте: телефон на лобовом стекле, Panda по USB, Volkswagen MQB (Golf 7) и поперечное удержание полосы через HCA.

Учебная линия следует курсу [Algorithms for Automated Driving (AAD)](https://github.com/thomasfermi/Algorithms-for-Automated-Driving): **геометрия → модель → алгоритм → измерение на данных**.
Главы про велосипедную модель и Pure Pursuit используют рисунки и выводы AAD (CC BY 4.0). Архитектура, Supercombo, `fp` / MPC, задержки и работа с бегами — специфика именно этого проекта.

## Цели курса

Проработав книгу, вы должны уметь:

* проследить кадр камеры $1280\times 720$ до CAN-кадра `HCA_01`;
* объяснить publish/subscribe в Middleware, таймеры и живые параметры;
* понимать, где кончается Java-обвязка (камера, ORT, ZMQ, беги) и начинаются алгоритмы на C++;
* отличать угол поворота колеса $\delta$ от угла рулевого колеса (SWA) на шине;
* реализовать и настроить Pure Pursuit и объяснить, чем `mpc` / `fp` отличаются от одношаговой геометрии;
* учитывать недокрут (`tire_stiffness_factor`) и транспортное запаздывание (`fp_steer_delay_s`);
* проецировать GPS в локальный ENU, объяснить захват ориентации IMU телефона и читать `localization/pose`;
* считать TTC и $a_{\mathrm{req}}$ для FCW/AEB и условия LDW со всеми гейтами;
* отделять деградацию зрения (нагрев, конкурирующий YOLO) от ошибки контроллера при чтении бега.

## Конвейер целиком

```{figure} ../Architecture/figures/pipeline_simple.png
---
width: 95%
---
Камера → Supercombo → метрический путь → удержание полосы → Panda / HCA.
```

1. Камера телефона снимает кадр.
2. Supercombo (ONNX Runtime) публикует полилинии разметки и план в метрах.
3. `Planner` (`pp` | `mpc` | `fp`) строит план в кривизне, `Control` превращает его в SWA или момент.
4. Panda отправляет `HCA_01`; EPS прикладывает момент (только под присмотром).

## Чем отличается от AAD

| | AAD | этот курс |
|---|---|---|
| Среда | CARLA и задания на Python | Android, нативный C++ и офлайн-Python |
| Детекция разметки | вы делаете сегментацию и IPM | Supercombo (семейство openpilot) как готовый датчик |
| Актуация | симулятор | HCA на MQB (на дороге — под присмотром преподавателя) |
| Данные | симулятор | дорожные беги и MetaDrive на хосте |

Берите AAD, если хотите реализовать детектор и Pure Pursuit **с нуля**.
Берите эту книгу, если хотите **близкий к промышленному телефонный конвейер** и дисциплину работы с настоящими логами.

## Рекомендуемый порядок

1. [Архитектура](../Architecture/Overview.md) → [Middleware](../Architecture/Middleware.md) → [Java](../Architecture/JavaLayer.md) → [Конвейер](../Architecture/Pipeline.md)
2. [Зрение](../Vision/Overview.md) → [системы координат](../Vision/Coordinates.md) → [Supercombo](../Vision/Supercombo.md)
3. [Локализация](../Localization/Overview.md) (ENU из GPS, IMU телефона, EKF)
4. [Велосипедная модель](../Control/BicycleModel.md) → [Pure Pursuit](../Control/PurePursuit.md) → [Модель машины](../Control/VehicleModel.md) → [MPC / fp](../Control/MPC_and_FP.md)
5. [FCW / AEB / LDW](../Safety/Warnings.md)
6. [Калибровка](../Calibration/Overview.md) → [Задержки](../Latency/Overview.md) → [Беги](../Logging/Bags.md)
7. [Задания](../Exercises/StudentProjects.md)

Дорожные эксперименты с HCA требуют присутствия преподавателя. Режим курса по умолчанию — **бег и скрипты**.

## Что нужно знать заранее

* Линейная алгебра и кинематика плоского твёрдого тела; тригонометрия.
* Python (`numpy`), чтение protobuf и CSV; умение читать C++ желательно.
* CNN и ONNX на уровне пользователя (обучать Supercombo вы не будете).

Инженерные заметки команды лежат в `docs/*.md`. Эта книга — **учебная дорожка** к тому материалу.

<!-- next-chapter -->
---

**Дальше:** [Обзор системы](../Architecture/Overview.md)
