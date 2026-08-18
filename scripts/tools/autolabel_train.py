#!/usr/bin/env python3
"""Auto-label a drive with a labeller network, then train a segmentation student on simple labels.

The shape follows the course (`Algorithms-for-Automated-Driving`, LaneDetection): **an image and a mask
of class indices**, a segmentation net over it, a polynomial fitted in road coordinates afterwards.
Nothing of openpilot's parametrisation reaches the student — no 33-point y on their `X_IDXS` grid, no
medmodel warp, no YUV6 packing, no recurrent feature buffer. The student sees an ordinary camera frame
and predicts per-pixel classes:

    0 back      1 laneLeft      2 laneRight      3 edgeLeft      4 edgeRight

supercombo appears in exactly one place — as the labeller, offline — and its output becomes pixels
immediately. Replace it with a human annotator or another network and nothing downstream changes: the
dataset on disk is the interface.

Layout is the course's, so their notebook reads it unchanged::

    <dataset>/train/<name>.png        <dataset>/train_label/<name>_label.png
    <dataset>/val/<name>.png          <dataset>/val_label/<name>_label.png

Two things are deliberately *not* taken from the labeller:

* the **path** — the trajectory the driver drove is recorded in the pose, so it needs no teacher;
* the **pose** head — measured against the wheels it reads 0.679 of the truth with the yaw sign
  inverted (task #37).

The value of an auto-labeller is its gate, not its network. Four gates here, each from a measurement on
our own drives: input sharpness, lane-line probability, σ, and — the one that is easy to miss —
**calibration convergence**, because a mask drawn with mounting angles that have not settled is a mask
of the wrong road. On one drive the learned yaw moved 3.1–3.5° over the session, which is up to 1.5 m of
lateral error at 50 m ahead.

Usage::

    cd scripts
    # label a drive and train from scratch
    python3 -m tools.autolabel_train ../app/src/main/assets/supercombo.onnx ../adas_logs/<session>

    # label several drives, then fine-tune an existing student on the union
    python3 -m tools.autolabel_train labeller.onnx ../adas_logs/a ../adas_logs/b --stage label
    python3 -m tools.autolabel_train --stage train --dataset build/autolabel --init student.pt
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np

import _path  # noqa: F401  (scripts/ on sys.path)

import cv2

from core.model_calib_warp import warp_matrix_deg, warp_to_model
from core.phone_rt import project_overlay_xyz
from core.supercombo_compare import make_overlay_geometry, parse_image_yuv
from core.supercombo_parse import X_IDXS, parse_supercombo
from vis.bag_io import load_topic_messages, nearest

# --- what the student predicts ---------------------------------------------------------------------
# Index order is the mask's pixel value and the ONNX channel order; changing it invalidates a dataset.
CLASS_NAMES = ("back", "laneLeft", "laneRight", "edgeLeft", "edgeRight")
N_CLASSES = len(CLASS_NAMES)

# Nearest range drawn. Closer than this the lateral projection fx·y/x runs away and the pixels say
# nothing about where the lane is on the road — the same 4 m the overlay uses.
LANE_X_MIN_M = 4.0
LANE_X_MAX_M = 30.0  # where σ is summarised for the gate

# Labeller (supercombo 0.9.x) input shapes.
TEACHER_DESIRE = (1, 100, 8)
TEACHER_TRAFFIC = (1, 2)
TEACHER_LAT_PARAMS = (1, 2)
TEACHER_PREV_CURV = (1, 100, 1)
TEACHER_FEATURES = (1, 99, 512)
FEATURE_LEN = 512
PARSED_OUTPUT = 5992
DESIRED_CURV_IDX = 5990

DEFAULT_OUT = Path(__file__).resolve().parents[2] / "build" / "autolabel"


# ======================================================================================
# stage 1: labelling
# ======================================================================================


@dataclass
class GateConfig:
    """Thresholds deciding whether a frame becomes a training sample.

    Measured, not guessed — and measured on this offline path, which matters for sharpness: it is read on
    the warped frame here, so the scale is not the runtime's (the runtime reads 9.9–14.9 defocused
    against 369–942 healthy on the full-resolution input). Over the same two drives this path reads
    8–10 defocused against 72–189 healthy, so the default sits at 40, in the middle of the gap.

    `min_prob` is the 0.3 the controller uses to decide a line exists; `max_std` keeps the σ tail out;
    `require_calibrated` drops frames whose online calibration had not converged yet.
    """

    min_focus: float = 40.0
    min_prob: float = 0.3
    max_std: float = 1.5
    require_both_host_lines: bool = True
    require_calibrated: bool = True


@dataclass
class GateStats:
    total: int = 0
    no_calib: int = 0
    uncalibrated: int = 0
    no_intrinsics: int = 0
    decode_failed: int = 0
    defocused: int = 0
    low_prob: int = 0
    empty_mask: int = 0
    kept: int = 0
    focus_values: List[float] = field(default_factory=list)
    class_pixels: np.ndarray = field(
        default_factory=lambda: np.zeros(N_CLASSES, dtype=np.int64)
    )

    def report(self) -> str:
        pct = (100.0 * self.kept / self.total) if self.total else 0.0
        focus = np.median(self.focus_values) if self.focus_values else float("nan")
        total_px = max(int(self.class_pixels.sum()), 1)
        shares = ", ".join(
            f"{name} {100.0 * self.class_pixels[i] / total_px:.2f} %"
            for i, name in enumerate(CLASS_NAMES)
        )
        return (
            f"  frames {self.total}, kept {self.kept} ({pct:.1f} %)\n"
            f"  dropped: no calib {self.no_calib}, uncalibrated {self.uncalibrated}, "
            f"no intrinsics {self.no_intrinsics}, decode {self.decode_failed}, "
            f"defocus {self.defocused}, low prob {self.low_prob}, empty mask {self.empty_mask}\n"
            f"  input sharpness, median over kept: {focus:.0f}\n"
            f"  label pixels: {shares}"
        )


class LastKnown:
    """The most recent accepted message at or before a timestamp — what the runtime had in hand.

    Mounting angles are a slowly-learned constant published far less often than frames arrive.
    Interpolating them would invent values; requiring one inside a window throws most of the drive away
    (512 of 790 frames on the reference drive at a 2 s window).
    """

    def __init__(self, series, accept=None):
        self.series = (
            [row for row in series if accept is None or accept(row[1])] if series else []
        )
        self.i = 0

    def at(self, ts: int):
        if not self.series:
            return None
        while self.i + 1 < len(self.series) and self.series[self.i + 1][0] <= ts:
            self.i += 1
        if self.series[self.i][0] > ts:
            return None  # nothing had arrived yet when this frame was taken
        return self.series[self.i][1]


def focus_score(img_bgr: np.ndarray) -> float:
    """Mean squared gradient of the luminance plane — the runtime's own sharpness metric.

    A defocused lens does not fail: it returns a valid frame with no lines in it, and the labeller then
    invents lines confidently. This is the number that separated the lost drive from the healthy ones,
    so labelling refuses the same frames.
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    return float(np.mean(gx * gx + gy * gy) / 16.0)


def frame_intrinsics(cam_msg) -> Optional[Tuple[float, float, float, float]]:
    """fx, fy, cx, cy **in the units of the stored image**.

    The units trap that cost a drive: `calibration/camera` carries full-frame numbers (fx 993.4, cx 640)
    while the bag stores a 640×360 preview whose own fx is 475.5. Masks are drawn in the stored frame,
    so only the frame's own values may be used here.
    """
    w = int(getattr(cam_msg, "width", 0) or 0)
    h = int(getattr(cam_msg, "height", 0) or 0)
    fx = float(getattr(cam_msg, "focal_length_x", 0.0) or 0.0)
    fy = float(getattr(cam_msg, "focal_length_y", 0.0) or 0.0)
    if w <= 0 or h <= 0 or fx <= 1.0 or fy <= 1.0:
        return None
    cx = float(getattr(cam_msg, "principal_point_x", 0.0) or 0.0)
    cy = float(getattr(cam_msg, "principal_point_y", 0.0) or 0.0)
    if not (0.25 * w < cx < 0.75 * w and 0.25 * h < cy < 0.75 * h):
        cx, cy = 0.5 * w, 0.5 * h
    return fx, fy, cx, cy


class Labeller:
    """supercombo through onnxruntime, with the recurrence the model needs.

    Offline there is no 33 ms budget, so this runs fp32 on the CPU and keeps the history the way the
    runtime does: features shift by one frame, `prev_desired_curv` by one value. Getting that wrong does
    not fail — the model simply remembers a past that never happened.
    """

    def __init__(self, path: Path, threads: int = 0):
        import onnxruntime as ort

        opts = ort.SessionOptions()
        if threads > 0:
            opts.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(
            str(path), opts, providers=["CPUExecutionProvider"]
        )
        self.names = {i.name for i in self.sess.get_inputs()}
        for required in ("input_imgs", "big_input_imgs", "features_buffer"):
            if required not in self.names:
                raise SystemExit(
                    f"{path}: not a supercombo 0.9.x graph (no '{required}' input); "
                    f"inputs are {sorted(self.names)}"
                )
        self.out_name = self.sess.get_outputs()[0].name
        self.reset()

    def reset(self) -> None:
        self.features = np.zeros(TEACHER_FEATURES, dtype=np.float32)
        self.prev_curv = np.zeros(TEACHER_PREV_CURV, dtype=np.float32)
        self.prev_yuv: Optional[np.ndarray] = None

    def run(self, yuv6: np.ndarray, v_ego: float) -> Optional[np.ndarray]:
        """One frame in, the flat output vector out. None until the two-frame stack is filled."""
        if self.prev_yuv is None:
            self.prev_yuv = yuv6
            return None
        imgs = np.concatenate([self.prev_yuv, yuv6], axis=0)[None].astype(np.float32)
        self.prev_yuv = yuv6

        feeds: Dict[str, np.ndarray] = {
            "input_imgs": imgs,
            "big_input_imgs": imgs,
            "features_buffer": self.features,
            "prev_desired_curv": self.prev_curv,
        }
        if "desire" in self.names:
            feeds["desire"] = np.zeros(TEACHER_DESIRE, dtype=np.float32)
        if "traffic_convention" in self.names:
            feeds["traffic_convention"] = np.zeros(TEACHER_TRAFFIC, dtype=np.float32)
        if "lateral_control_params" in self.names:
            lat = np.zeros(TEACHER_LAT_PARAMS, dtype=np.float32)
            lat[0, 0] = v_ego
            feeds["lateral_control_params"] = lat

        out = self.sess.run([self.out_name], feeds)[0].reshape(-1).astype(np.float64)

        if out.size >= PARSED_OUTPUT + FEATURE_LEN:
            self.features[0, :-1] = self.features[0, 1:]
            self.features[0, -1] = out[
                PARSED_OUTPUT : PARSED_OUTPUT + FEATURE_LEN
            ].astype(np.float32)
        if out.size > DESIRED_CURV_IDX:
            self.prev_curv[0, :-1] = self.prev_curv[0, 1:]
            self.prev_curv[0, -1, 0] = float(out[DESIRED_CURV_IDX])
        return out


def lines_from_labeller(
    out: np.ndarray, gate: GateConfig
) -> Optional[List[Tuple[int, np.ndarray]]]:
    """(class index, lateral y) per line that passes the gate; None if the frame is rejected outright.

    The model's third component is deliberately dropped. It is not "height above the road": measured on a
    real frame it reads **1.22–1.24 m** with the camera at 1.10 m, i.e. it is the distance *down* from the
    camera to the line. The projection wants ISO height above the road, so feeding the model's value put
    every line on the horizon — the masks came out as horizontal streaks. Lane lines and kerbs live on
    the road, so the label is drawn at z = 0.
    """
    parsed = parse_supercombo(out)
    lane_left, lane_right = parsed.lanes[1], parsed.lanes[2]
    edge_left, edge_right = parsed.edges[0], parsed.edges[1]

    if gate.require_both_host_lines and (
        lane_left.prob < gate.min_prob or lane_right.prob < gate.min_prob
    ):
        return None

    near = (X_IDXS >= LANE_X_MIN_M) & (X_IDXS <= LANE_X_MAX_M)
    kept: List[Tuple[int, np.ndarray]] = []
    for cls, line, prob in (
        (1, lane_left, lane_left.prob),
        (2, lane_right, lane_right.prob),
        (3, edge_left, 1.0),
        (4, edge_right, 1.0),
    ):
        if prob < gate.min_prob:
            continue
        if line.y_std is not None:
            if float(np.median(np.asarray(line.y_std)[near])) > gate.max_std:
                continue
        kept.append((cls, np.asarray(line.y, dtype=np.float64)))
    return kept or None


def draw_mask(
    shape: Tuple[int, int],
    lines: Sequence[Tuple[int, np.ndarray]],
    geom,
    line_px: int,
) -> np.ndarray:
    """Class-index mask in the stored frame's own pixels.

    Road edges are drawn after the lane lines, so where they overlap the edge wins. That is a rule, not
    an accident: overlaps are rare and a deterministic outcome beats whichever order the loop happened
    to use.
    """
    h, w = shape
    mask = np.zeros((h, w), dtype=np.uint8)
    on_road = np.zeros_like(X_IDXS)
    for cls, y in lines:
        pts = project_overlay_xyz(
            X_IDXS, y, on_road, geom, w, h, x_min=LANE_X_MIN_M, y_sign=-1.0
        )
        if len(pts) < 2:
            continue
        cv2.polylines(
            mask,
            [np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)],
            isClosed=False,
            color=int(cls),
            thickness=line_px,
            lineType=cv2.LINE_8,
        )
    return mask


def label_bag(
    bag: Path,
    labeller: Labeller,
    dataset: Path,
    gate: GateConfig,
    stride: int,
    limit: int,
    line_px: int,
    val_frac: float,
) -> GateStats:
    """Runs the labeller over one drive and writes image/mask pairs in the course's layout."""
    stats = GateStats()
    frames = load_topic_messages(bag, "sensors/camera/image")
    if not frames:
        print(f"  {bag.name}: no sensors/camera/image — nothing to label")
        return stats
    calib_all = load_topic_messages(bag, "calibration/camera")
    chassis = load_topic_messages(bag, "vehicle/state")

    def converged(msg) -> bool:
        if not gate.require_calibrated:
            return True
        return bool(getattr(msg, "calibration_success", False))

    hold_calib = LastKnown(calib_all, accept=converged)
    if calib_all and not hold_calib.series:
        print(
            f"  {bag.name}: online calibration never converged here ({len(calib_all)} messages) — "
            f"masks would be drawn with unsettled mounting angles. Pass --allow-uncalibrated to label "
            f"anyway, knowing the geometry is approximate."
        )
        return stats

    # Which frames to consider is decided up front, and `--limit` thins that list instead of stopping
    # early. Stopping early would take the first N frames of the drive, and since the split below is by
    # time, every sample would land on the train side and validation would come out empty.
    candidates = list(range(0, len(frames), max(stride, 1)))
    if limit:
        # Oversample ×2: roughly half the frames are rejected by the gate, so this lands near `limit`.
        keep_every = max(1, len(candidates) // max(2 * limit, 1))
        if keep_every > 1:
            candidates = candidates[::keep_every]
            print(
                f"  --limit {limit}: considering {len(candidates)} frames spread over the drive"
            )

    # The split is by time: the tail of the considered range is validation, because neighbouring frames
    # at 30 Hz are near-duplicates and a random split would put the same road metre on both sides.
    val_from = (
        candidates[int(round((1.0 - val_frac) * (len(candidates) - 1)))]
        if candidates
        else 0
    )
    for sub in ("train", "train_label", "val", "val_label"):
        (dataset / sub).mkdir(parents=True, exist_ok=True)

    labeller.reset()
    for n in candidates:
        ts, cam, _raw = frames[n]
        stats.total += 1

        cal = hold_calib.at(ts)
        if cal is None:
            if calib_all:
                stats.uncalibrated += 1
            else:
                stats.no_calib += 1
            continue
        k = frame_intrinsics(cam)
        if k is None:
            stats.no_intrinsics += 1
            continue
        fx, fy, cx, cy = k

        data = bytes(getattr(cam, "image_data", b"") or b"")
        bgr = (
            cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if data
            else None
        )
        if bgr is None:
            stats.decode_failed += 1
            continue
        h, w = bgr.shape[:2]

        roll = float(getattr(cal, "roll_deg", 0.0))
        pitch = float(getattr(cal, "pitch_deg", 0.0))
        yaw = float(getattr(cal, "yaw_deg", 0.0))

        # The labeller wants its own geometry — that is its business, and none of it reaches the student.
        warped = warp_to_model(bgr, warp_matrix_deg(roll, pitch, yaw, fx, fy, cx, cy))
        sharp = focus_score(warped)
        if sharp < gate.min_focus:
            stats.defocused += 1
            labeller.prev_yuv = (
                None  # a blind frame must not become the stack's history either
            )
            continue

        v_ego = 0.0
        ch_hit = nearest(ts, chassis, max_dt_ms=200) if chassis else None
        if ch_hit is not None:
            v_ego = float(getattr(ch_hit[1], "speed_mps", 0.0) or 0.0)

        out = labeller.run(
            parse_image_yuv(cv2.cvtColor(warped, cv2.COLOR_BGR2YUV_I420)), v_ego
        )
        if out is None:
            continue

        lines = lines_from_labeller(out, gate)
        if lines is None:
            stats.low_prob += 1
            continue

        geom = make_overlay_geometry(
            fx,
            fy,
            cx,
            cy,
            w,
            h,
            camera_height=float(getattr(cal, "camera_height_m", 1.1) or 1.1),
            pitch_deg=pitch,
            yaw_deg=yaw,
        )
        mask = draw_mask((h, w), lines, geom, line_px)
        if not mask.any():
            stats.empty_mask += 1
            continue

        split = "val" if n >= val_from else "train"
        name = f"{bag.name}_{ts}"
        cv2.imwrite(str(dataset / split / f"{name}.png"), bgr)
        cv2.imwrite(str(dataset / f"{split}_label" / f"{name}_label.png"), mask)

        stats.kept += 1
        stats.focus_values.append(sharp)
        stats.class_pixels += np.bincount(mask.reshape(-1), minlength=N_CLASSES)[
            :N_CLASSES
        ]

    return stats


# ======================================================================================
# stage 2: training
# ======================================================================================


class MaskDataset:
    """Image/mask pairs in the course's layout, resized to the training size."""

    def __init__(self, dataset: Path, split: str, size: Tuple[int, int]):
        self.images = sorted((dataset / split).glob("*.png"))
        self.labels = dataset / f"{split}_label"
        self.size = size

    def __len__(self) -> int:
        return len(self.images)

    def sample(self, i: int) -> Tuple[np.ndarray, np.ndarray]:
        img_path = self.images[i]
        lbl_path = self.labels / f"{img_path.stem}_label.png"
        bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(lbl_path), cv2.IMREAD_GRAYSCALE)
        if bgr is None or mask is None:
            raise SystemExit(f"unreadable pair: {img_path} / {lbl_path}")
        w, h = self.size
        bgr = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
        # NEAREST for the mask: class indices must never be blended into classes that do not exist.
        mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
        x = (
            cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).transpose(2, 0, 1).astype(np.float32)
            / 255.0
        )
        return x, mask.astype(np.int64)


def build_student(width: int = 16):
    """A compact U-Net, written out rather than pulled from a package.

    The course uses `fastseg`'s MobileV3Small; nothing of that kind is installed here and adding a
    dependency for a harness is not worth it. What matters is the shape: RGB in, one logit per class per
    pixel out — the same contract, so the course's post-processing (probability → polynomial in road
    coordinates) applies unchanged.
    """
    import torch
    import torch.nn as nn

    def conv(cin: int, cout: int) -> nn.Sequential:
        return nn.Sequential(
            nn.Conv2d(cin, cout, 3, padding=1, bias=False),
            nn.BatchNorm2d(cout),
            nn.ReLU(inplace=True),
            nn.Conv2d(cout, cout, 3, padding=1, bias=False),
            nn.BatchNorm2d(cout),
            nn.ReLU(inplace=True),
        )

    class UNet(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            w = width
            self.enc1 = conv(3, w)
            self.enc2 = conv(w, 2 * w)
            self.enc3 = conv(2 * w, 4 * w)
            self.enc4 = conv(4 * w, 8 * w)
            self.pool = nn.MaxPool2d(2)
            self.up3 = nn.ConvTranspose2d(8 * w, 4 * w, 2, stride=2)
            self.dec3 = conv(8 * w, 4 * w)
            self.up2 = nn.ConvTranspose2d(4 * w, 2 * w, 2, stride=2)
            self.dec2 = conv(4 * w, 2 * w)
            self.up1 = nn.ConvTranspose2d(2 * w, w, 2, stride=2)
            self.dec1 = conv(2 * w, w)
            self.head = nn.Conv2d(w, N_CLASSES, 1)

        def forward(self, x):  # type: ignore[no-untyped-def]
            import torch

            e1 = self.enc1(x)
            e2 = self.enc2(self.pool(e1))
            e3 = self.enc3(self.pool(e2))
            e4 = self.enc4(self.pool(e3))
            d3 = self.dec3(torch.cat([self.up3(e4), e3], dim=1))
            d2 = self.dec2(torch.cat([self.up2(d3), e2], dim=1))
            d1 = self.dec1(torch.cat([self.up1(d2), e1], dim=1))
            return self.head(d1)

    return UNet()


def dice_per_class(pred: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Dice for every class, NaN where the class is absent from both — the course's metric."""
    out = np.full(N_CLASSES, np.nan)
    for c in range(N_CLASSES):
        p = pred == c
        t = target == c
        denom = int(p.sum() + t.sum())
        if denom:
            out[c] = 2.0 * float(np.logical_and(p, t).sum()) / denom
    return out


def train(
    dataset: Path,
    size: Tuple[int, int],
    epochs: int,
    batch: int,
    lr: float,
    init: Optional[Path],
    out_ckpt: Path,
    export_onnx: Optional[Path],
    line_weight: float,
    seed: int = 0,
) -> None:
    import torch
    import torch.nn.functional as F

    torch.manual_seed(seed)
    train_ds = MaskDataset(dataset, "train", size)
    val_ds = MaskDataset(dataset, "val", size)
    if len(train_ds) == 0:
        raise SystemExit(f"no images in {dataset / 'train'} — run the label stage first")
    print(f"train {len(train_ds)} images, val {len(val_ds)}, size {size[0]}×{size[1]}")

    model = build_student()
    if init is not None:
        state = torch.load(init, map_location="cpu", weights_only=True)
        model.load_state_dict(state["model"] if "model" in state else state)
        print(f"fine-tuning from {init}")
    opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)

    # Lines are a fraction of a per cent of the pixels, so unweighted cross-entropy is minimised by
    # predicting background everywhere — the classic way to score 99 % accuracy and detect nothing.
    weights = torch.full((N_CLASSES,), float(line_weight))
    weights[0] = 1.0

    def batches(ds: MaskDataset, shuffle: bool) -> Iterator[Tuple]:
        idx = np.arange(len(ds))
        if shuffle:
            np.random.default_rng(seed).shuffle(idx)
        for s in range(0, len(idx), batch):
            chunk = idx[s : s + batch]
            xs, ms = zip(*(ds.sample(int(i)) for i in chunk))
            yield torch.from_numpy(np.stack(xs)), torch.from_numpy(np.stack(ms))

    for epoch in range(1, epochs + 1):
        model.train()
        t0 = time.time()
        run_loss, seen = 0.0, 0
        for x, m in batches(train_ds, shuffle=True):
            loss = F.cross_entropy(model(x), m, weight=weights)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            run_loss += float(loss) * x.shape[0]
            seen += x.shape[0]

        line = f"epoch {epoch}/{epochs}  train loss {run_loss / max(seen, 1):.4f}"
        if len(val_ds):
            model.eval()
            dices: List[np.ndarray] = []
            fg_hit = fg_total = 0
            with torch.no_grad():
                for x, m in batches(val_ds, shuffle=False):
                    pred = model(x).argmax(dim=1).numpy()
                    tgt = m.numpy()
                    dices.append(dice_per_class(pred, tgt))
                    fg = tgt > 0
                    fg_total += int(fg.sum())
                    fg_hit += int((pred[fg] == tgt[fg]).sum())
            d = np.nanmean(np.stack(dices), axis=0)
            per_class = ", ".join(
                f"{name} {d[i]:.3f}"
                for i, name in enumerate(CLASS_NAMES)
                if not np.isnan(d[i])
            )
            fg_acc = (fg_hit / fg_total) if fg_total else float("nan")
            line += f"  val dice: {per_class}  foreground acc {fg_acc:.3f}"
        print(f"{line}  ({time.time() - t0:.0f} s)")

    out_ckpt.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model": model.state_dict(),
            "classes": list(CLASS_NAMES),
            "size": [size[0], size[1]],
        },
        out_ckpt,
    )
    print(f"checkpoint → {out_ckpt}")

    if export_onnx is not None:
        dummy = torch.zeros(1, 3, size[1], size[0])
        kwargs = dict(
            input_names=["rgb"],
            output_names=["class_logits"],
            dynamic_axes={"rgb": {0: "batch"}},
            opset_version=17,
        )
        # torch >= 2.5 defaults to the dynamo exporter, which needs `onnxscript` installed. The
        # TorchScript path needs nothing extra and is enough for a net this simple; fall back for older
        # torch that has no such argument.
        try:
            try:
                torch.onnx.export(model, dummy, str(export_onnx), dynamo=False, **kwargs)
            except TypeError:
                torch.onnx.export(model, dummy, str(export_onnx), **kwargs)
            print(f"onnx → {export_onnx}")
        except Exception as exc:  # the checkpoint above is the result; export is a convenience
            print(f"onnx export failed ({type(exc).__name__}: {exc})")
            print(
                "  checkpoint is written; `pip install onnxscript` if the ONNX is needed too"
            )


# ======================================================================================


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description="Auto-label host lane lines and road edges with a labeller network, "
        "then train a segmentation student on the masks.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "labeller", nargs="?", type=Path, help="labeller network (supercombo 0.9.x ONNX)"
    )
    p.add_argument("bags", nargs="*", type=Path, help="drive directories to label")
    p.add_argument("--stage", choices=("label", "train", "all"), default="all")
    p.add_argument("--dataset", type=Path, default=DEFAULT_OUT, help="dataset root")
    p.add_argument("--stride", type=int, default=1, help="take every Nth frame")
    p.add_argument(
        "--limit",
        type=int,
        default=0,
        help="stop after N kept frames per drive (0 = all)",
    )
    p.add_argument(
        "--line-px", type=int, default=6, help="mask line thickness in the stored frame"
    )
    p.add_argument(
        "--val-frac",
        type=float,
        default=0.2,
        help="tail of each drive kept for validation",
    )
    p.add_argument(
        "--threads",
        type=int,
        default=0,
        help="onnxruntime intra-op threads (0 = default)",
    )

    g = p.add_argument_group("gate")
    g.add_argument("--min-focus", type=float, default=40.0)
    g.add_argument("--min-prob", type=float, default=0.3)
    g.add_argument("--max-std", type=float, default=1.5)
    g.add_argument(
        "--any-host-line",
        action="store_true",
        help="keep frames where only one host line is confident",
    )
    g.add_argument(
        "--allow-uncalibrated",
        action="store_true",
        help="label even where the online calibration had not converged",
    )

    t = p.add_argument_group("training")
    t.add_argument("--size", type=int, nargs=2, default=(320, 192), metavar=("W", "H"))
    t.add_argument("--epochs", type=int, default=8)
    t.add_argument("--batch", type=int, default=4)
    t.add_argument("--lr", type=float, default=1e-3)
    t.add_argument(
        "--line-weight",
        type=float,
        default=20.0,
        help="class weight of the four line classes against background",
    )
    t.add_argument("--init", type=Path, help="student checkpoint to fine-tune")
    t.add_argument("--ckpt", type=Path, help="where to write the checkpoint")
    t.add_argument("--export-onnx", type=Path)

    a = p.parse_args(argv)
    ckpt = a.ckpt or (a.dataset / "student.pt")
    size = (int(a.size[0]), int(a.size[1]))

    if a.stage in ("label", "all"):
        if a.labeller is None or not a.bags:
            p.error("labelling needs a labeller network and at least one drive")
        if not a.labeller.is_file():
            p.error(f"labeller not found: {a.labeller}")
        gate = GateConfig(
            min_focus=a.min_focus,
            min_prob=a.min_prob,
            max_std=a.max_std,
            require_both_host_lines=not a.any_host_line,
            require_calibrated=not a.allow_uncalibrated,
        )
        net = Labeller(a.labeller, threads=a.threads)
        print(f"labeller: {a.labeller}")
        totals = GateStats()
        for bag in a.bags:
            if not bag.is_dir():
                print(f"  {bag}: not a directory, skipped")
                continue
            print(f"labelling {bag.name}")
            st = label_bag(
                bag, net, a.dataset, gate, a.stride, a.limit, a.line_px, a.val_frac
            )
            print(st.report())
            for f in (
                "total",
                "no_calib",
                "uncalibrated",
                "no_intrinsics",
                "decode_failed",
                "defocused",
                "low_prob",
                "empty_mask",
                "kept",
            ):
                setattr(totals, f, getattr(totals, f) + getattr(st, f))
            totals.focus_values.extend(st.focus_values)
            totals.class_pixels += st.class_pixels
        print("\nlabelling totals")
        print(totals.report())

        a.dataset.mkdir(parents=True, exist_ok=True)
        (a.dataset / "dataset.json").write_text(
            json.dumps(
                {
                    "labeller": str(a.labeller),
                    "bags": [str(b) for b in a.bags],
                    "classes": list(CLASS_NAMES),
                    "line_px": a.line_px,
                    "gate": {
                        "min_focus": a.min_focus,
                        "min_prob": a.min_prob,
                        "max_std": a.max_std,
                        "require_both_host_lines": not a.any_host_line,
                        "require_calibrated": not a.allow_uncalibrated,
                    },
                    "kept": totals.kept,
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        if totals.kept == 0:
            print("nothing passed the gate — not training")
            return 1

    if a.stage in ("train", "all"):
        train(
            a.dataset,
            size=size,
            epochs=a.epochs,
            batch=a.batch,
            lr=a.lr,
            init=a.init,
            out_ckpt=ckpt,
            export_onnx=a.export_onnx,
            line_weight=a.line_weight,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
