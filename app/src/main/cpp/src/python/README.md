# pyadas — C++ ADAS for Python (sim / bag visualizer)

Host path uses **`AdasApp` in Simulated mode**: publish inputs → `step` → `pop_messages`.
Services stay inside the app (not exposed to Python). Output type identifies the source.

## Host API

```python
from pyadas import AdasApp, LaneKeepOutput, LocalizationPose, CameraCalibrationState

app = AdasApp(wheelbase=2.636, pitch0_deg=0.0, camera_height=1.40)
app.set_camera_intrinsics(930, 930, 640, 360)

app.publish_chassis(t_us, speed_mps=10, steer_rad=0.01)
app.publish_lanes(t_us, [(1, 0), (10, 0.1), (30, 0.2)])
app.publish_lane_uv(t_us, left_uv, right_uv)
app.publish_gps(t_us, x, y)
app.publish_imu(t_us, yaw_rate)
app.step(t_us)

for msg in app.pop_messages():
    if isinstance(msg, LaneKeepOutput):
        ...
    elif isinstance(msg, LocalizationPose):
        ...
    elif isinstance(msg, CameraCalibrationState):
        ...
```

## Build (desktop)

```bash
cd app/src/main/cpp
cmake -B build-linux -DBUILD_FOR_ANDROID=OFF -DBUILD_PYTHON_BINDINGS=ON ...
cmake --build build-linux -j --target core
```
