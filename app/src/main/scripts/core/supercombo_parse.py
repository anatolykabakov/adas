#!/usr/bin/env python3
"""Parse supercombo.onnx output (shape 1×6409).

Layout matches driving-model ``driving.cc`` + the demo's slice starts
(plan ends at 4955). Important: values are **not** plain image polylines.

Coordinates are in the **ego / calibrated frame**:
  X forward (m), Y **right** (m) for this ONNX (matches Android overlay
  ``u = cx + fx·Y/X``), Z up (m), sampled at ``X_IDXS`` (0…192 m).

  Note: classic ISO docs say Y-left; this build’s lane means have
  leftNear with **negative** Y — treat as Y-right when projecting.

================================================================================
Index map (this ONNX, out=6409)
================================================================================
  [0    : 4955)  PLAN     5 trajectory hypotheses (MHP)
  [4955 : 5483)  LANES    4 lane lines × 33 × (y, z)   then × (y_std, z_std)
  [5483 : 5491)  LANE_PROB  8 logits (use odd indices → sigmoid)
  [5491 : 5755)  ROAD_EDGES 2 edges × 33 × (y, z) then stds
  [5755 : 6409)  LEAD + desire/meta/pose + GRU state (rest; sizes vary by build)

================================================================================
PLAN (the main driving output)
================================================================================
5 hypotheses × 991 floats:
  33 timesteps × 15 dims  (means)
  33 timesteps × 15 dims  (log-stds)
  1  selection logit

Per point, 15 columns (see fill_model / fill_xyzt):
  0..2   position  x, y, z
  3..5   velocity
  6..8   (related / acceleration-ish in some builds)
  9..11  orientation
  12..14 orientation rate

Pick hypothesis with max selection logit. That curve is the **planned path**,
not a painted lane.

================================================================================
LANES / ROAD EDGES
================================================================================
4 lines: leftFar, leftNear, rightNear, rightFar.
2 edges: left, right road edge.

Storage per geometry block (528 for lanes, 264 for edges):
  first half  = means: for each of N lines, 33 pairs (y, z) interleaved
  second half = stds:  same layout (exp(.) for σ)

So index ``i*2`` within a line's 66 floats is **lateral Y**, ``i*2+1`` is **Z**
(height), NOT (y, std) as the GitHub demo's ``seperate_points_and_std_values``
suggests. That demo only works for Y by accident (even indices).

Lane probs: sigmoid(lane_lines_prob[i*2+1]) for line i.

================================================================================
What the toy demo plots
================================================================================
Yellow “lanes” / red “edges” / green “mid” are a **partial, buggy** read of
the lane/edge heads. It never draws the PLAN head. Green is
``0.5*(ll_t2 + l_t)`` (blend of two lane tensors), not the planner path.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

X_IDXS = np.array(
    [
        0.0,
        0.1875,
        0.75,
        1.6875,
        3.0,
        4.6875,
        6.75,
        9.1875,
        12.0,
        15.1875,
        18.75,
        22.6875,
        27.0,
        31.6875,
        36.75,
        42.1875,
        48.0,
        54.1875,
        60.75,
        67.6875,
        75.0,
        82.6875,
        90.75,
        99.1875,
        108.0,
        117.1875,
        126.75,
        136.6875,
        147.0,
        157.6875,
        168.75,
        180.1875,
        192.0,
    ],
    dtype=np.float64,
)

PLAN_END = 4955
LANES_END = PLAN_END + 528  # 5483
LANE_PROB_END = LANES_END + 8  # 5491
ROAD_END = LANE_PROB_END + 264  # 5755

PLAN_MHP_N = 5
PLAN_COLS = 15
PLAN_GROUP = 2 * PLAN_COLS * 33 + 1  # 991
TRAJ_N = 33

LANE_NAMES = ("leftFar", "leftNear", "rightNear", "rightFar")


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.asarray(x, dtype=np.float64)))


@dataclass
class LaneXYZ:
    name: str
    y: np.ndarray  # (33,) left-positive
    z: np.ndarray  # (33,) up
    prob: float
    y_std: Optional[np.ndarray] = None


@dataclass
class PlanXYZ:
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    hyp_index: int
    logit: float
    yaw: Optional[np.ndarray] = None  # orientation.z (rad)
    yaw_rate: Optional[np.ndarray] = None  # orientationRate.z (rad/s)


@dataclass
class SupercomboOut:
    plan: PlanXYZ
    lanes: List[LaneXYZ]
    edges: List[LaneXYZ]  # left, right; prob unused


def _yz_from_block(block66: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """33×(y,z) interleaved → y, z arrays."""
    flat = np.asarray(block66, dtype=np.float64).reshape(-1)
    return flat[0::2].copy(), flat[1::2].copy()


def parse_supercombo(out: np.ndarray) -> SupercomboOut:
    """Parse flat 6409 (or longer) model vector."""
    res = np.asarray(out, dtype=np.float64).reshape(-1)
    if res.size < ROAD_END:
        raise ValueError(f"output too short: {res.size}")

    # --- plan: best of 5 MHP ---
    best_i, best_logit = 0, -1e9
    for i in range(PLAN_MHP_N):
        logit = float(res[(i + 1) * PLAN_GROUP - 1])
        if logit > best_logit:
            best_logit, best_i = logit, i
    base = best_i * PLAN_GROUP
    means = res[base : base + PLAN_COLS * TRAJ_N].reshape(TRAJ_N, PLAN_COLS)
    plan = PlanXYZ(
        x=means[:, 0].copy(),
        y=means[:, 1].copy(),
        z=means[:, 2].copy(),
        hyp_index=best_i,
        logit=best_logit,
        yaw=means[:, 11].copy(),
        yaw_rate=means[:, 14].copy(),
    )

    # --- lanes: means in first 264, stds in second 264 ---
    lane_raw = res[PLAN_END:LANES_END]
    probs_raw = res[LANES_END:LANE_PROB_END]
    n_mean = 4 * 66  # 264 means; same count of log-sigmas follows
    lanes: List[LaneXYZ] = []
    for i, name in enumerate(LANE_NAMES):
        y, z = _yz_from_block(lane_raw[i * 66 : (i + 1) * 66])
        # official: sigmoid(prob[i*2+1])
        prob = float(_sigmoid(probs_raw[i * 2 + 1]))
        y_std = None
        if lane_raw.size >= 2 * n_mean:
            ystd, _ = _yz_from_block(lane_raw[n_mean + i * 66 : n_mean + (i + 1) * 66])
            y_std = np.exp(ystd)
        lanes.append(LaneXYZ(name=name, y=y, z=z, prob=prob, y_std=y_std))

    # --- road edges ---
    edge_raw = res[LANE_PROB_END:ROAD_END]
    edges: List[LaneXYZ] = []
    for i, name in enumerate(("edgeLeft", "edgeRight")):
        y, z = _yz_from_block(edge_raw[i * 66 : (i + 1) * 66])
        edges.append(LaneXYZ(name=name, y=y, z=z, prob=1.0))

    return SupercomboOut(plan=plan, lanes=lanes, edges=edges)


@dataclass
class LeadTrack:
    prob: float
    x: np.ndarray  # (6,)
    y: np.ndarray
    v: np.ndarray
    a: np.ndarray
    prob_time: float = 0.0

    @property
    def d_rel(self) -> float:
        return float(self.x[0]) if self.x.size else 0.0


@dataclass
class ModelLongOut:
    plan_vx: np.ndarray
    plan_vy: np.ndarray
    plan_vz: np.ndarray
    lead0: LeadTrack
    lead1: LeadTrack
    lead2: LeadTrack
    pose_vx: float = 0.0
    pose_vy: float = 0.0
    pose_vz: float = 0.0
    pose_valid: bool = False

    def best_lead(self) -> LeadTrack:
        return max((self.lead0, self.lead1, self.lead2), key=lambda L: L.prob)


LEAD_IDX = ROAD_END
LEAD_MHP_N = 2
LEAD_TRAJ_LEN = 6
LEAD_PRED_DIM = 4
LEAD_MHP_VALS = LEAD_PRED_DIM * LEAD_TRAJ_LEN
LEAD_MHP_SELECTION = 3
LEAD_MHP_GROUP = 2 * LEAD_MHP_VALS + LEAD_MHP_SELECTION
LEAD_PROB_IDX = LEAD_IDX + LEAD_MHP_N * LEAD_MHP_GROUP
TEMPORAL_SIZE = 512
POSE_SIZE = 12


def pose_idx_for_output(n: int) -> int:
    """Pose slice start: last 12 floats before temporal GRU state."""
    if n < POSE_SIZE + TEMPORAL_SIZE:
        return max(0, n - POSE_SIZE)
    return n - TEMPORAL_SIZE - POSE_SIZE


# Default for assets 6409; prefer pose_idx_for_output(len(out)) at runtime.
POSE_IDX = pose_idx_for_output(6409)


def _fill_lead(out: np.ndarray, t_offset: int, prob_time: float) -> LeadTrack:
    prob = float(_sigmoid(np.array([out[LEAD_PROB_IDX + t_offset]]))[0])
    sel_off = t_offset - LEAD_MHP_SELECTION
    best, best_logit = 0, -1e9
    for i in range(LEAD_MHP_N):
        logit = float(out[LEAD_IDX + (i + 1) * LEAD_MHP_GROUP + sel_off])
        if logit > best_logit:
            best, best_logit = i, logit
    base = LEAD_IDX + best * LEAD_MHP_GROUP
    xs = np.empty(LEAD_TRAJ_LEN)
    ys = np.empty(LEAD_TRAJ_LEN)
    vs = np.empty(LEAD_TRAJ_LEN)
    aas = np.empty(LEAD_TRAJ_LEN)
    for i in range(LEAD_TRAJ_LEN):
        row = base + i * LEAD_PRED_DIM
        xs[i] = out[row]
        ys[i] = out[row + 1]
        vs[i] = out[row + 2]
        aas[i] = out[row + 3]
    return LeadTrack(prob=prob, prob_time=prob_time, x=xs, y=ys, v=vs, a=aas)


def parse_model_long(out: np.ndarray) -> Optional[ModelLongOut]:
    """Parse lead + PLAN velocity (+ pose) from flat supercombo output."""
    res = np.asarray(out, dtype=np.float64).reshape(-1)
    if res.size < LEAD_PROB_IDX + 2:
        return None
    best_i, best_logit = 0, -1e9
    for i in range(PLAN_MHP_N):
        logit = float(res[(i + 1) * PLAN_GROUP - 1])
        if logit > best_logit:
            best_logit, best_i = logit, i
    base = best_i * PLAN_GROUP
    means = res[base : base + PLAN_COLS * TRAJ_N].reshape(TRAJ_N, PLAN_COLS)
    pose_valid = res.size >= pose_idx_for_output(res.size) + 3
    pidx = pose_idx_for_output(int(res.size))
    return ModelLongOut(
        plan_vx=means[:, 3].copy(),
        plan_vy=means[:, 4].copy(),
        plan_vz=means[:, 5].copy(),
        lead0=_fill_lead(res, 0, 0.0),
        lead1=_fill_lead(res, 1, 2.0),
        lead2=_fill_lead(res, 2, 4.0),
        pose_vx=float(res[pidx]) if pose_valid else 0.0,
        pose_vy=float(res[pidx + 1]) if pose_valid else 0.0,
        pose_vz=float(res[pidx + 2]) if pose_valid else 0.0,
        pose_valid=pose_valid,
    )


def explain_output(out: SupercomboOut) -> str:
    lines = [f"plan hyp#{out.plan.hyp_index} logit={out.plan.logit:.3f}"]
    for lane in out.lanes:
        lines.append(f"  {lane.name}: y0={lane.y[0]:+.3f} prob={lane.prob:.3f}")
    return "\n".join(lines)
