#!/usr/bin/env python3
"""OSM road graph as a background layer for the bag trajectory plot.

Everything is drawn in the map's own frame (``RoadMap.frame``), and the trajectories are
re-projected into it once, at load. The obvious alternative — moving the map into the
panel's frame — needs a transform between two different projections: the panel uses a
spherical equirectangular grid anchored at the first GNSS fix
(``core.gps_utils.gps_to_local_coords``), the map an ellipsoidal one anchored at the map
origin. Linearising that transform at the run origin drifts to 16 m at the far end of a
16 km run, which at a 20 m zoom puts the car a lane and a half off the road.

Re-projection is exact and vectorised: the panel frame inverts to lat/lon in closed form,
and ``LocalFrame.to_local_many`` takes it from there.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

EARTH_R_M = 6371000.0

_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP = _ROOT / "maps" / "Moscow.osm.admap"
# The APK no longer carries the map; the underlay reads maps/ only.


def default_map() -> Path:
    return DEFAULT_MAP


class OsmLayer:
    """Loads the compact road map once and serves its geometry in the map frame."""

    def __init__(self, map_path: Optional[Path] = None):
        self.map_path = Path(map_path) if map_path else default_map()
        self.road_map = None
        self.error: Optional[str] = None
        self._origin: Optional[Tuple[float, float]] = None

    @property
    def ready(self) -> bool:
        return self.road_map is not None and self._origin is not None

    def load(self) -> bool:
        if self.road_map is not None:
            return True
        if not self.map_path.is_file():
            self.error = f"no map {self.map_path}"
            return False
        try:
            from pyadas import core as pyadas

            road_map = pyadas.mapmatch.RoadMap()
            if not road_map.load(str(self.map_path)):
                self.error = f"failed to load {self.map_path}"
                return False
            self.road_map = road_map
            return True
        except Exception as exc:  # pyadas missing or map unreadable
            self.error = str(exc)
            return False

    def set_origin(self, lat0: float, lon0: float) -> bool:
        """Remember the panel frame's anchor — the run's first GNSS fix."""
        if not self.load():
            return False
        self._origin = (float(lat0), float(lon0))
        return True

    def to_map(self, x, y) -> Tuple[np.ndarray, np.ndarray]:
        """Panel frame (equirectangular metres about the first fix) → map frame."""
        lat0, lon0 = self._origin
        deg = math.pi / 180.0
        lat = lat0 + np.asarray(y, dtype=np.float64) / (EARTH_R_M * deg)
        lon = lon0 + np.asarray(x, dtype=np.float64) / (
            math.cos(math.radians(lat0)) * EARTH_R_M * deg
        )
        mx, my = self.road_map.frame.to_local_many(lat.tolist(), lon.tolist())
        return np.asarray(mx), np.asarray(my)

    def polylines(
        self, x0: float, y0: float, x1: float, y1: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Road centrelines inside a map-frame bbox, NaN-separated for one Line2D."""
        if not self.ready:
            return np.array([]), np.array([])
        xs, ys = self.road_map.polylines_in_bbox(
            float(min(x0, x1)), float(min(y0, y1)), float(max(x0, x1)), float(max(y0, y1))
        )
        if not xs:
            return np.array([]), np.array([])
        return np.asarray(xs), np.asarray(ys)

    def edges(self, x0: float, y0: float, x1: float, y1: float) -> List[int]:
        if not self.ready:
            return []
        return list(
            self.road_map.edges_in_bbox(
                float(min(x0, x1)),
                float(min(y0, y1)),
                float(max(x0, x1)),
                float(max(y0, y1)),
            )
        )

    def edge_polyline(self, eid: int) -> Tuple[np.ndarray, np.ndarray]:
        px, py = self.road_map.edge_polyline(eid)
        return np.asarray(px), np.asarray(py)

    def edge_name(self, eid: int) -> str:
        return self.road_map.edge_name(eid) or ""

    def nearest_edge(self, x: float, y: float, radius_m: float = 40.0):
        """(edge id, distance, name) for the road under a map-frame point."""
        if not self.ready:
            return None
        eid, dist, _ = self.road_map.nearest_edge(float(x), float(y), float(radius_m))
        if eid < 0 or dist > radius_m:
            return None
        return eid, float(dist), self.edge_name(eid)

    def street_labels(
        self, x0: float, y0: float, x1: float, y1: float, limit: int = 12
    ) -> List[Tuple[float, float, float, str]]:
        """(x, y, angle_deg, name) label anchors, one per named street in view."""
        if not self.ready:
            return []
        seen = set()
        out: List[Tuple[float, float, float, str]] = []
        for eid in self.edges(x0, y0, x1, y1):
            name = self.edge_name(eid)
            if not name or name in seen:
                continue
            px, py = self.edge_polyline(eid)
            if len(px) < 2:
                continue
            i = max(1, min(len(px) // 2, len(px) - 1))
            ang = math.degrees(math.atan2(py[i] - py[i - 1], px[i] - px[i - 1]))
            if ang > 90.0:
                ang -= 180.0
            elif ang < -90.0:
                ang += 180.0
            seen.add(name)
            out.append((float(px[i]), float(py[i]), ang, name))
            if len(out) >= limit:
                break
        return out


def lane_offsets(px: np.ndarray, py: np.ndarray, n_lanes: int, width_m: float):
    """Lane boundaries of a road: its centreline shifted by whole lane widths."""
    dx, dy = np.gradient(px), np.gradient(py)
    norm = np.hypot(dx, dy)
    norm[norm < 1e-6] = 1.0
    nx, ny = -dy / norm, dx / norm
    for k in range(n_lanes + 1):
        off = (k - n_lanes / 2.0) * width_m
        yield px + nx * off, py + ny * off


def lane_counts(admap: Path) -> dict:
    """Lane counts per edge from the sidecar produced by ``mapmatch.osm_lanes`` (if any)."""
    side = admap.with_suffix(admap.suffix + ".lanes.npz")
    if not side.is_file():
        return {}
    data = np.load(side)
    return {int(k): int(v) for k, v in zip(data["edge"], data["lanes"])}


def speed_limits(admap: Path) -> dict:
    """Speed limits per edge from the sidecar produced by ``mapmatch.osm_maxspeed`` (if any).

    Forward and backward are stored apart because a two-way road can carry different limits; the
    visualizer draws the forward one, which is what the car on that edge is subject to.
    """
    side = admap.with_suffix(admap.suffix + ".maxspeed.npz")
    if not side.is_file():
        return {}
    data = np.load(side, allow_pickle=True)
    return {
        int(e): int(v) for e, v in zip(data["edge"], data["forward_kmh"]) if int(v) > 0
    }


def point_features(admap: Path):
    """Point features from the sidecar produced by ``mapmatch.osm_points`` (if any).

    Returned as arrays rather than a dict: the visualizer filters them by the visible window on every
    redraw, and 45k points is where a per-point Python loop starts to show.
    """
    side = admap.with_suffix(admap.suffix + ".points.npz")
    if not side.is_file():
        return None
    d = np.load(side, allow_pickle=True)
    return d["x"], d["y"], d["kind"], d["detail"]


def transform_to_world(
    poly: Sequence[Sequence[float]],
    x: float,
    y: float,
    heading_rad: float,
    y_sign: float = -1.0,
) -> Optional[np.ndarray]:
    """Device-frame polyline (X forward, Y right+) → world points around a pose."""
    pts = np.asarray(poly, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 2:
        return None
    fx, fy = pts[:, 0], pts[:, 1] * y_sign
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    return np.stack([x + c * fx - s * fy, y + s * fx + c * fy], axis=1)
