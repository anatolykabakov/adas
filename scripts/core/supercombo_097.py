#!/usr/bin/env python3
"""supercombo 0.9.7 through onnxruntime — one offline runtime, matching the phone.

The phone runs this network two ways (`SupercomboOnnxRunner.java`, `thneed_runner.cpp`) and both are
held to the same output. Offline there were two more attempts at it, and they had drifted: the
simulator's runner still fed the 0.8.x input set (four inputs, no wide picture) and died on the 0.9.7
asset, while the auto-labeller fed the *narrow* warp to the wide input. A network that gets the wrong
inputs does not fail — it answers about a road that is not there. So there is one runtime here now,
and it mirrors the Java one input for input:

* ``input_imgs`` / ``big_input_imgs`` — the same camera frame twice, warped by medmodel and
  sbigmodel intrinsics; each is a pair of consecutive frames, six channels apiece;
* ``desire`` — zeros: there is no lane-change planner on either side;
* ``traffic_convention`` — zeros, exactly as both phone paths leave it (backlog #48);
* ``lateral_control_params`` — ``[v_ego, 0.1]``, the actuator delay flowpilot uses;
* ``prev_desired_curv`` / ``features_buffer`` — the recurrence, shifted by one every frame.

The recurrence is the part that punishes carelessness: skip the shift and the model keeps answering
about a past that never happened, with no error anywhere. `zero_input_signature` is the cheap guard —
the same all-zeros probe the phone runs at startup, comparable against the number in the shipped
config.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

from .model_calib_warp import warp_matrix_deg, warp_to_model

# Dimensions of supercombo 0.9.7. The same numbers live in SupercomboOnnxRunner.java and
# thneed_runner.cpp; if these three drift apart, so do the predictions.
HISTORY_LEN = 99
FEATURE_LEN = 512
PARSED_OUTPUT = 5992
NET_OUTPUT_SIZE = PARSED_OUTPUT + FEATURE_LEN  # 6504
DESIRED_CURV_IDX = 5990
DESIRE_FRAMES = HISTORY_LEN + 1  # 100 frames of 8 features
PREV_CURV_LEN = HISTORY_LEN + 1
#: Actuator delay handed to the model, as in flowpilot.
ACTUATOR_DELAY_S = 0.1

REQUIRED_INPUTS = (
    "input_imgs",
    "big_input_imgs",
    "desire",
    "traffic_convention",
    "lateral_control_params",
    "prev_desired_curv",
    "features_buffer",
)

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SHIPPED_CONFIG = _REPO_ROOT / "app" / "src" / "main" / "assets" / "config.json"


def parse_image_yuv(frame_yuv_i420: np.ndarray) -> np.ndarray:
    """I420 → the model's six planes (Y in four half-res phases, then U and V)."""
    H = (frame_yuv_i420.shape[0] * 2) // 3
    W = frame_yuv_i420.shape[1]
    parsed = np.zeros((6, H // 2, W // 2), dtype=np.uint8)
    parsed[0] = frame_yuv_i420[0:H:2, 0::2]
    parsed[1] = frame_yuv_i420[1:H:2, 0::2]
    parsed[2] = frame_yuv_i420[0:H:2, 1::2]
    parsed[3] = frame_yuv_i420[1:H:2, 1::2]
    parsed[4] = frame_yuv_i420[H : H + H // 4].reshape((-1, H // 2, W // 2))
    parsed[5] = frame_yuv_i420[H + H // 4 : H + H // 2].reshape((-1, H // 2, W // 2))
    return parsed


def bgr_to_model_yuv6(bgr: np.ndarray, m_model_to_cam: np.ndarray) -> np.ndarray:
    """One camera frame → the six planes the model eats, through the given warp."""
    img = warp_to_model(bgr, m_model_to_cam)
    return parse_image_yuv(cv2.cvtColor(img, cv2.COLOR_BGR2YUV_I420))


def shipped_zero_input_signature(runner: str = "onnx") -> Optional[Tuple[float, float]]:
    """The (mean, std) the phone checks itself against, from the shipped config.

    Returns ``None`` when the config has no such entry, so a caller can say "not checked" instead of
    inventing a threshold.
    """
    try:
        cfg = json.loads(_SHIPPED_CONFIG.read_text())
    except (OSError, ValueError):
        return None
    entry = (cfg.get("vision") or cfg).get("zero_input")
    if not isinstance(entry, dict):
        # The key sits at whatever level the config happens to use; search one level down.
        for value in cfg.values():
            if isinstance(value, dict) and isinstance(value.get("zero_input"), dict):
                entry = value["zero_input"]
                break
    if not isinstance(entry, dict):
        return None
    got = entry.get(runner)
    if not isinstance(got, dict) or "mean" not in got or "std" not in got:
        return None
    return float(got["mean"]), float(got["std"])


class Supercombo097:
    """The 0.9.7 network with its recurrence, fed the way the phone feeds it."""

    def __init__(
        self,
        model_path: str | Path,
        threads: int = 0,
        providers: Optional[Sequence[str]] = None,
    ) -> None:
        import onnxruntime as ort

        self.model_path = Path(model_path)
        opts = ort.SessionOptions()
        if threads > 0:
            opts.intra_op_num_threads = threads
        self.session = ort.InferenceSession(
            str(self.model_path),
            opts,
            providers=list(providers) if providers else ["CPUExecutionProvider"],
        )
        inputs = {i.name: i for i in self.session.get_inputs()}
        missing = [name for name in REQUIRED_INPUTS if name not in inputs]
        if missing:
            raise ValueError(
                f"{self.model_path.name}: not a supercombo 0.9.x graph — missing {missing}; "
                f"its inputs are {sorted(inputs)}"
            )
        # fp16 graphs want fp16 feeds; the widened fp32 asset wants fp32 (see onnx_widen_fp32.py).
        self.dtype = (
            np.float16 if "float16" in (inputs["input_imgs"].type or "") else np.float32
        )
        self.out_name = self.session.get_outputs()[0].name
        self.image_shape = (1, 12, 128, 256)

        self.fx = 930.0
        self.fy = 930.0
        self.cx = 640.0
        self.cy = 360.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self._rebuild_warps()
        self.reset()

    # ---- state ---------------------------------------------------------------------------

    def reset(self) -> None:
        """Forget the past. Anything but consecutive frames must go through here."""
        self.features = np.zeros((1, HISTORY_LEN, FEATURE_LEN), dtype=np.float32)
        self.prev_curv = np.zeros((1, PREV_CURV_LEN, 1), dtype=np.float32)
        self._prev_narrow: Optional[np.ndarray] = None
        self._prev_wide: Optional[np.ndarray] = None

    def set_calib(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> None:
        self.roll_deg = float(roll_deg)
        self.pitch_deg = float(pitch_deg)
        self.yaw_deg = float(yaw_deg)
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self._rebuild_warps()

    def _rebuild_warps(self) -> None:
        args = (
            self.roll_deg,
            self.pitch_deg,
            self.yaw_deg,
            self.fx,
            self.fy,
            self.cx,
            self.cy,
        )
        self.warp_narrow = warp_matrix_deg(*args)
        self.warp_wide = warp_matrix_deg(*args, big_model=True)

    # ---- inference -----------------------------------------------------------------------

    def _feeds(
        self,
        imgs: np.ndarray,
        wide: np.ndarray,
        v_ego: float,
        features: np.ndarray,
        prev_curv: np.ndarray,
    ) -> Dict[str, Any]:
        lat = np.array([[float(v_ego), ACTUATOR_DELAY_S]], dtype=np.float32)
        return {
            "input_imgs": imgs.astype(self.dtype, copy=False),
            "big_input_imgs": wide.astype(self.dtype, copy=False),
            "desire": np.zeros((1, DESIRE_FRAMES, 8), dtype=self.dtype),
            "traffic_convention": np.zeros((1, 2), dtype=self.dtype),
            "lateral_control_params": lat.astype(self.dtype, copy=False),
            "prev_desired_curv": prev_curv.astype(self.dtype, copy=False),
            "features_buffer": features.astype(self.dtype, copy=False),
        }

    def run_frame6(
        self, narrow6: np.ndarray, wide6: np.ndarray, v_ego: float = 0.0
    ) -> Optional[np.ndarray]:
        """One prepared frame in, the flat 6504 output out.

        ``None`` on the very first frame: the input is a *pair* of frames, and there is no honest way
        to invent the one before the first.
        """
        if self._prev_narrow is None or self._prev_wide is None:
            self._prev_narrow, self._prev_wide = narrow6, wide6
            return None

        imgs = np.concatenate([self._prev_narrow, narrow6], axis=0)[None]
        wide = np.concatenate([self._prev_wide, wide6], axis=0)[None]
        self._prev_narrow, self._prev_wide = narrow6, wide6

        out = self.session.run(
            [self.out_name],
            self._feeds(imgs, wide, v_ego, self.features, self.prev_curv),
        )[0]
        flat = np.asarray(out, dtype=np.float64).reshape(-1)
        self._advance(flat)
        return flat

    def run_bgr(self, bgr: np.ndarray, v_ego: float = 0.0) -> Optional[np.ndarray]:
        """One camera frame in — warped twice, then as above."""
        narrow = bgr_to_model_yuv6(bgr, self.warp_narrow)
        wide = bgr_to_model_yuv6(bgr, self.warp_wide)
        return self.run_frame6(narrow, wide, v_ego)

    def _advance(self, flat: np.ndarray) -> None:
        """Shift both feedback paths by one, exactly as `advanceRecurrence` does on the phone."""
        if flat.size < NET_OUTPUT_SIZE:
            return
        self.features[0, :-1] = self.features[0, 1:]
        self.features[0, -1] = flat[PARSED_OUTPUT:NET_OUTPUT_SIZE].astype(np.float32)
        self.prev_curv[0, :-1] = self.prev_curv[0, 1:]
        self.prev_curv[0, -1, 0] = float(flat[DESIRED_CURV_IDX])

    # ---- self-check ----------------------------------------------------------------------

    def zero_input_signature(self) -> Tuple[float, float]:
        """(mean, std) of the output on all-zero inputs — the phone's own startup probe.

        Runs on local zero state, so it can be called at any point without disturbing the recurrence.
        """
        zeros_img = np.zeros(self.image_shape, dtype=np.float32)
        out = self.session.run(
            [self.out_name],
            self._feeds(
                zeros_img,
                zeros_img,
                0.0,
                np.zeros((1, HISTORY_LEN, FEATURE_LEN), dtype=np.float32),
                np.zeros((1, PREV_CURV_LEN, 1), dtype=np.float32),
            ),
        )[0]
        flat = np.asarray(out, dtype=np.float64).reshape(-1)
        return float(flat.mean()), float(flat.std())

    def check_zero_input(
        self, tol_mean: float = 0.05, tol_std: float = 0.05
    ) -> Tuple[bool, Tuple[float, float], Optional[Tuple[float, float]]]:
        """Compare the probe against the shipped reference.

        Returns ``(ok, measured, expected)``. With no reference in the config, ``ok`` is True and
        ``expected`` is None — "not checked" is a different answer from "checked and fine", and the
        caller should say which one it means.
        """
        measured = self.zero_input_signature()
        expected = shipped_zero_input_signature("onnx")
        if expected is None:
            return True, measured, None
        ok = (
            abs(measured[0] - expected[0]) <= tol_mean
            and abs(measured[1] - expected[1]) <= tol_std
        )
        return ok, measured, expected
