"""Test tracks for the MetaDrive controller runs.

MetaDrive generates PG maps from a block sequence (``S`` straight, ``C`` curve) and picks each
block's geometry at random from a parameter space. Stock ranges give 25–60 m arcs — city corners,
not the highway arcs the assistant is built for. A track pins that space, so a run is
reproducible from (track, seed) and the arc radii are known up front instead of measured after.

Each ``C`` block is an arc followed by a straight, so a sequence alternates loaded and unloaded
sections without extra bookkeeping. Arcs start at full curvature (no clothoid entry) — harsher
than a real road, which is the point: curvature steps is where a controller shows its lag.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Tuple

from metadrive.component.pg_space import (
    BlockParameterSpace,
    BoxSpace,
    ConstantSpace,
    DiscreteSpace,
    Parameter,
    ParameterSpace,
    VehicleParameterSpace,
)
from metadrive.component.pgblock.curve import Curve
from metadrive.component.pgblock.straight import Straight
from metadrive.component.vehicle.vehicle_type import DefaultVehicle

LANE_WIDTH_M = 3.5

# Stock ego is capped at 80 km/h with 800 N of engine force — it never reaches highway speed,
# let alone holds it through an arc. These are vehicle *parameters*, not vehicle_config keys.
MAX_SPEED_KMH = 130
MAX_ENGINE_FORCE = 2600


@dataclass(frozen=True)
class Track:
    name: str
    sequence: str
    radius_m: Tuple[float, float]
    angle_deg: Tuple[float, float]
    curve_exit_m: Tuple[float, float]
    straight_m: Tuple[float, float]
    speed_mps: float
    lane_num: int = 1
    description: str = ""

    def lateral_accel_ms2(self, speed_mps: float | None = None) -> Tuple[float, float]:
        v = self.speed_mps if speed_mps is None else speed_mps
        return (v * v / self.radius_m[1], v * v / self.radius_m[0])


TRACKS: Dict[str, Track] = {
    "highway": Track(
        name="highway",
        sequence="CSCSCSC",
        radius_m=(250.0, 700.0),
        angle_deg=(25.0, 60.0),
        curve_exit_m=(70.0, 110.0),
        straight_m=(120.0, 200.0),
        speed_mps=25.0,
        description="track arcs R 250–700 m at 25 m/s (a_lat 0.9–2.5 m/s²) — operating region",
    ),
    "curvy": Track(
        name="curvy",
        sequence="CCSCCSCC",
        radius_m=(120.0, 260.0),
        angle_deg=(35.0, 80.0),
        curve_exit_m=(40.0, 70.0),
        straight_m=(80.0, 120.0),
        speed_mps=22.0,
        description="winding R 120–260 m at 22 m/s (a_lat 1.9–4.0) — torque limit",
    ),
    "tight": Track(
        name="tight",
        sequence="CSCCSC",
        radius_m=(45.0, 110.0),
        angle_deg=(45.0, 110.0),
        curve_exit_m=(30.0, 50.0),
        straight_m=(60.0, 90.0),
        speed_mps=13.0,
        description="tight arcs R 45–110 m at 13 m/s — beyond limit, expect off-road",
    ),
    "straight": Track(
        name="straight",
        sequence="SSSS",
        radius_m=(250.0, 700.0),
        angle_deg=(25.0, 60.0),
        curve_exit_m=(70.0, 110.0),
        straight_m=(150.0, 250.0),
        speed_mps=25.0,
        description="straights only — baseline for yaw and command jitter",
    ),
}


def apply_track_geometry(track: Track) -> None:
    """Pin the PG generator to this track's geometry.

    Class-level parameter spaces are read at map generation, so this has to run before
    ``MetaDriveEnv`` is constructed.
    """
    Curve.PARAMETER_SPACE = ParameterSpace(
        {
            Parameter.length: BoxSpace(
                min=track.curve_exit_m[0], max=track.curve_exit_m[1]
            ),
            Parameter.radius: BoxSpace(min=track.radius_m[0], max=track.radius_m[1]),
            Parameter.angle: BoxSpace(min=track.angle_deg[0], max=track.angle_deg[1]),
            Parameter.dir: DiscreteSpace(min=0, max=1),
        }
    )
    Straight.PARAMETER_SPACE = ParameterSpace(
        {Parameter.length: BoxSpace(min=track.straight_m[0], max=track.straight_m[1])}
    )
    BlockParameterSpace.CURVE = {
        Parameter.length: BoxSpace(min=track.curve_exit_m[0], max=track.curve_exit_m[1]),
        Parameter.radius: BoxSpace(min=track.radius_m[0], max=track.radius_m[1]),
        Parameter.angle: BoxSpace(min=track.angle_deg[0], max=track.angle_deg[1]),
        Parameter.dir: DiscreteSpace(min=0, max=1),
    }
    ego = dict(VehicleParameterSpace.DEFAULT_VEHICLE)
    ego["max_speed_km_h"] = ConstantSpace(MAX_SPEED_KMH)
    ego["max_engine_force"] = ConstantSpace(MAX_ENGINE_FORCE)
    DefaultVehicle.PARAMETER_SPACE = ParameterSpace(ego)


def env_config(track: Track, seed: int, **overrides: Any) -> Dict[str, Any]:
    """MetaDrive config for a headless run of this track."""
    apply_track_geometry(track)
    cfg: Dict[str, Any] = {
        "use_render": False,
        "manual_control": False,
        "traffic_density": 0.0,
        "num_scenarios": 1,
        "start_seed": int(seed),
        "horizon": 1_000_000,
        "log_level": 50,
        # 9 physics ticks of 10 ms = 90 ms per control step: the phone's median vision period,
        # so the planner sees the same solve rate it sees on the road.
        "physics_world_step_size": 0.01,
        "decision_repeat": 9,
        "map_config": {
            "type": "block_sequence",
            "config": track.sequence,
            "lane_num": track.lane_num,
            "lane_width": LANE_WIDTH_M,
            "exit_length": 80,
        },
    }
    cfg.update(overrides)
    return cfg


def resolve(name: str) -> Track:
    if name not in TRACKS:
        raise SystemExit(f"unknown track {name!r}; have: {', '.join(TRACKS)}")
    return TRACKS[name]
