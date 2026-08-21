"""Canonical frames for Android / bag / sim parity.

Control path (TopicConvert → PurePursuit on phone) uses **device** frame:
  X forward, Y **right+**, Z up  (flowpilot Parser / Android LaneLines).

Host ISO overlay helpers (AAD CameraGeometry) use Y **left+**.
When projecting device-Y points with ``project_iso_xyz``, pass ``DRAW_Y_SIGN``.

MetaDrive steering is ISO-ish (left+). Device-frame ``steer_rad`` from PP must be
negated for ``env.step`` — see ``METADRIVE_STEER_FROM_DEVICE``.
"""

from __future__ import annotations

# Keep model / bag plan_y as-is for AdasApp.publish_lanes (Android parity).
PP_Y_SIGN = 1.0

# project_iso_xyz: device Y-right → ISO Y-left for drawing.
DRAW_Y_SIGN = -1.0

# Angles have a sign convention too, and it caught us once. `calibration/camera` publishes RPY in the
# device convention `ModelCalibWarp` / `warp_matrix_deg` use; AAD's `CameraGeometry` runs **pitch and
# yaw the other way** (roll agrees). `make_overlay_geometry` converts, so pass it the calibration as
# published — do not pre-negate. Building a `CameraGeometry` by hand from calibration angles tilts the
# overlay by twice the angle and lifts it off the road; `PhoneRtGeometry` needs no conversion, being
# the inverse of the warp itself. With the conversion the two projectors agree to 0 px.

# MetaDrive action steer from device-frame PP δ (right+ → left+ actuator).
METADRIVE_STEER_FROM_DEVICE = -1.0

# Match C++ LaneKeepService::Config default (config.json may omit).
DEFAULT_MAX_STEER_DEG = 8.0
DEFAULT_PP_K_DD = 0.4
DEFAULT_PP_LD_MIN = 3.0
DEFAULT_PP_LD_MAX = 20.0
DEFAULT_PP_SHIFT = 1.40
DEFAULT_WHEELBASE_M = 2.636
DEFAULT_MIN_LANE_PROB = 0.3
