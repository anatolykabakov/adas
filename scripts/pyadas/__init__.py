"""pyadas — C++ ADAS algorithms via pybind11.

Build (desktop)::

  cd app/src/main/cpp
  cmake -B build-linux -DBUILD_FOR_ANDROID=OFF -DBUILD_PYTHON_BINDINGS=ON ...
  cmake --build build-linux --target core

Module lands in ``scripts/pyadas/`` (copied on build).

Host API: ``AdasApp`` — publish inputs → ``step`` → read ``*_state``.
"""

from __future__ import annotations

try:
    from . import core as core
except ImportError as exc:
    core = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None
    from .core import (  # noqa: F401
        AdasApp,
        CameraCalibrationState,
        ChassisSample,
        GpsSample,
        ImuSample,
        LaneKeepOutput,
        LanePathMsg,
        LocalizationPose,
        PyAdasApp,
        Vec2,
    )


def require_core():
    """Raise a clear error if the pybind module was not built."""
    if core is None:
        raise ImportError(
            "pyadas.core is not available — build the host C++ target first "
            "(./scripts/docker.sh host or ./app/src/main/cpp/build_cpp.sh -t linux). "
            f"Import failed with: {_IMPORT_ERROR}"
        ) from _IMPORT_ERROR
    return core
