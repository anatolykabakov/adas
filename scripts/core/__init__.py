"""Shared ADAS host helpers. Algorithms live in ``pyadas.AdasApp`` (C++).

Python here is viz / bag glue only:

  - ``core.lane_keep.LaneKeepController`` — publish chassis/lanes → AdasApp
  - ``core.vanishing_point_calib`` — Hough → ``publish_lane_uv``
  - ``core.pure_pursuit`` — draw / plan→polyline HUD
  - ``core.online_localizer`` — IMU warm-up + MetaDrive EKF helper around C++
"""
