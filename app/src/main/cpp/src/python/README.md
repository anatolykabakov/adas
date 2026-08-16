# pyadas — тот же C++ из Python

Модуль pybind11 поверх `AdasApp` в режиме `Simulated`: потоков нет, время двигает хост. Сервисы
остаются внутри приложения и наружу не выставляются — снаружи видно только то, что они опубликовали.
Именно поэтому офлайн-разбор бега, симулятор и телефон считают одним и тем же кодом.

## Как пользоваться

```python
import pyadas
from pyadas import AdasApp, LaneKeepOutput, LocalizationPose, CameraCalibrationState

pyadas.require_core()          # понятная ошибка, если модуль не собран

app = AdasApp(wheelbase=2.636, pitch0_deg=0.0, yaw0_deg=0.0, camera_height=1.22)
app.set_camera_intrinsics(fx=930, fy=930, cx=640, cy=360)

app.publish_chassis(t_us, speed_mps=10.0, steer_rad=0.01)
app.publish_lanes(t_us, [(1, 0.0), (10, 0.1), (30, 0.2)])
app.publish_lane_uv(t_us, left_uv, right_uv)
app.publish_gps_location(gps_proto.SerializeToString())
app.publish_imu(t_us, yaw_rate=0.0)
app.step(t_us)

for msg in app.pop_messages():
    if isinstance(msg, LaneKeepOutput):
        ...
    elif isinstance(msg, LocalizationPose):
        ...
    elif isinstance(msg, CameraCalibrationState):
        ...
```

Тип сообщения и есть указание на источник — отдельного поля с именем топика нет.

Публикации бывают двух видов. Типизированные (`publish_chassis`, `publish_lanes`, `publish_imu`)
собирают структуру прямо из аргументов. Протобуфные (`publish_gps_location`, `publish_imu_data`,
`publish_lane_lines`) принимают сериализованное сообщение — ровно то, что лежит в беге, поэтому
воспроизведение заезда не требует ничего разбирать вручную.

**`publish_gps` брать не нужно.** Он кладёт уже спроецированные метры на `sensors/gps/location`, а
сервис локализации подписан там на `GPSLocation` и проецирует широту с долготой сам. Подписчика у
типизированного варианта нет: публикация проходит впустую, в логе появляется `publish type mismatch`,
и прогон едет на счислении пути, вовсе не получая ГНСС. Рабочий вызов — `publish_gps_location`.

## Сборка

```bash
./app/src/main/cpp/build_cpp.sh -t linux        # биндинги под Linux собираются всегда
```

Готовый модуль сборка кладёт в `scripts/pyadas/`, откуда его берут все офлайн-инструменты. Отдельно
настраивать cmake не нужно: `BUILD_PYTHON_BINDINGS` включается для хоста сам.

Если модуль не собран, `pyadas.core` равен `None`, а `require_core()` бросает исключение с
подсказкой, что запустить, — вместо `AttributeError` в середине разбора.
