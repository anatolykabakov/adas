"""pyadas — C++ ADAS algorithms (Pure Pursuit, EKF) via pybind11.

Build (desktop)::

  cd app/src/main/cpp
  cmake -B build-linux -DBUILD_FOR_ANDROID=OFF -DBUILD_PYTHON_BINDINGS=ON ...
  cmake --build build-linux

Module lands in ``scripts/pyadas/`` (copied on build) or build ``pyadas/core*.so``.
Falls back to pure Python ``core.*`` if the extension is missing.
"""

from __future__ import annotations

try:
    from . import core as core
except ModuleNotFoundError:
    core = None
else:
    from .core import (  # noqa: F401
        AdasApp,
        AdasPipeline,
        CameraCalibService,
        CameraCalibrationState,
        ChassisSample,
        GpsSample,
        ImuSample,
        LaneKeepService,
        LanePathMsg,
        LaneUvMsg,
        LocalizationPose,
        LocalizationService,
        OnlineLocalizer,
        PurePursuit,
        PurePursuitResult,
        PyAdasApp,
        PyAdasPipeline,
        VehicleEKF,
        Vec2,
    )
