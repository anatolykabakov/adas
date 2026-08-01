#!/usr/bin/env python3
"""
Utilities for working with GPS coordinates.
Convert GPS (lat, lon) to a local coordinate system (x, y).
"""

import numpy as np
from typing import Tuple, List

# Earth radius in meters
EARTH_RADIUS = 6371000.0


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Compute bearing (azimuth) from point 1 to point 2 on a sphere (Earth).

    Uses spherical trigonometry to compute the shortest path
    between two points on the Earth's surface (great-circle arc).

    Args:
        lat1, lon1: Latitude and longitude of the first point (degrees)
        lat2, lon2: Latitude and longitude of the second point (degrees)

    Returns:
        Bearing in degrees (0° = north, 90° = east, 180° = south, 270° = west)

    Example:
        # Moscow → Saint Petersburg
        bearing = calculate_bearing(55.7558, 37.6173, 59.9343, 30.3351)
        # bearing ≈ 318° (northwest)

    Formula (spherical trigonometry):
        x = sin(Δlon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(Δlon)
        bearing = atan2(x, y)

    Where:
        - x: eastward direction component
        - y: northward direction component
        - atan2(x, y): angle from north (bearing)
    """
    # STEP 1: Convert degrees to radians
    # ===================================
    # All trigonometric functions work with radians
    lat1_rad = np.radians(lat1)
    lat2_rad = np.radians(lat2)
    dlon_rad = np.radians(lon2 - lon1)  # Δlon - longitude difference

    # STEP 2: Compute direction components (spherical trigonometry formula)
    # ===================================================================
    # This formula accounts for Earth's curvature and gives the shortest-path direction

    # X component (eastward direction):
    # --------------------------------
    # sin(Δlon) - how far we moved in longitude
    #   > 0: movement to the east
    #   < 0: movement to the west
    # cos(lat2) - latitude correction (meridians converge at the poles!)
    #   At equator (lat=0°): cos(0°) = 1 (full contribution)
    #   At pole (lat=90°): cos(90°) = 0 (meridians converge to a point)
    x = np.sin(dlon_rad) * np.cos(lat2_rad)

    # Y component (northward direction):
    # ----------------------------------
    # More complex formula accounting for spherical geometry:
    #   cos(lat1) * sin(lat2) - movement in latitude
    #   sin(lat1) * cos(lat2) * cos(Δlon) - Earth curvature correction
    # Result:
    #   > 0: movement to the north
    #   < 0: movement to the south
    y = np.cos(lat1_rad) * np.sin(lat2_rad) - np.sin(lat1_rad) * np.cos(
        lat2_rad
    ) * np.cos(dlon_rad)

    # STEP 3: Compute bearing via atan2
    # ==================================
    # atan2(x, y) determines the angle in polar coordinates:
    #   - Uses signs of x and y to determine the quadrant
    #   - Returns angle from the Y axis (north) in radians
    #
    # Argument order MATTERS: atan2(x, y), NOT atan2(y, x)!
    # This gives the angle from north (navigation coordinate system)
    bearing_rad = np.arctan2(x, y)
    bearing_deg = np.degrees(bearing_rad)  # Convert back to degrees

    # STEP 4: Normalize to [0°, 360°)
    # ================================
    # atan2 returns angle in [-180°, 180°]
    # Convert to standard navigation format [0°, 360°)
    bearing_deg = (bearing_deg + 360) % 360

    return bearing_deg


def calculate_initial_heading_from_gps(gps_data: np.ndarray, n_points: int = 5) -> float:
    """
    Compute initial vehicle orientation (heading/yaw) from the first GPS points.

    Calculation steps:
    1. Take the first N pairs of GPS points
    2. Compute bearing (azimuth) between each pair
    3. Average bearings (accounting for circular angle wrap-around)
    4. Convert bearing (navigation) to yaw (mathematical)

    Args:
        gps_data: numpy array shape (N, 5) [timestamp, lat, lon, alt, speed]
        n_points: Number of points to average (reduces GPS noise)

    Returns:
        Initial yaw in radians (0 = east, π/2 = north, -π/2 = south)

    Example:
        gps_data = [[t1, 55.62, 37.65, ...], [t2, 55.63, 37.66, ...], ...]
        yaw = calculate_initial_heading_from_gps(gps_data, n_points=5)
        # yaw ≈ 1.075 rad (61.6°) = movement to the northeast
    """
    if gps_data is None or len(gps_data) < 2:
        return 0.0

    # STEP 1: Limit point count to available data
    # ============================================
    n_points = min(n_points, len(gps_data) - 1)

    # STEP 2: Compute bearing between each pair of consecutive points
    # ==============================================================
    # Bearing = azimuth from north (0°=north, 90°=east, 180°=south, 270°=west)
    bearings = []
    for i in range(n_points):
        lat1 = gps_data[i, 1]  # Latitude of first point
        lon1 = gps_data[i, 2]  # Longitude of first point
        lat2 = gps_data[i + 1, 1]  # Latitude of second point
        lon2 = gps_data[i + 1, 2]  # Longitude of second point

        # Compute bearing between points (spherical trigonometry formula)
        bearing = calculate_bearing(lat1, lon1, lat2, lon2)
        bearings.append(bearing)

    # STEP 3: Average bearing (correct angle averaging!)
    # ===================================================
    # PROBLEM: Simple averaging does not work for angles!
    #   Example: mean([350°, 10°]) = 180° ❌ (wrong, should be 0°)
    #
    # SOLUTION: Average via sin/cos (correct method)
    #   1. Represent each angle as a vector on the unit circle
    #   2. Average the x and y components of the vectors
    #   3. Compute the angle of the resulting vector

    bearings_rad = np.radians(bearings)  # Convert to radians

    # Angle averaging via vector representation:
    #   angle → (cos(angle), sin(angle))  - vector on the unit circle
    mean_sin = np.mean(np.sin(bearings_rad))  # Mean y component
    mean_cos = np.mean(np.cos(bearings_rad))  # Mean x component
    mean_bearing_rad = np.arctan2(mean_sin, mean_cos)  # Angle of resulting vector

    # STEP 4: Convert bearing → yaw (coordinate system change)
    # ===========================================================
    # Bearing (navigation system):
    #   0° = North, 90° = East, measured CLOCKWISE from north
    #        N (0°)
    #        ↑
    #   W ←--+--→ E (90°)
    #        ↓
    #        S (180°)
    #
    # Yaw (mathematical system):
    #   0° = East, 90° = North, measured COUNTER-CLOCKWISE from east
    #        N (π/2)
    #        ↑
    #   W ←--+--→ E (0°)
    #        ↓
    #        S (-π/2)
    #
    # Conversion formula: yaw = π/2 - bearing
    yaw_rad = np.pi / 2 - mean_bearing_rad

    # STEP 5: Normalize to [-π, π]
    # =============================
    # Bring angle to standard range
    while yaw_rad > np.pi:
        yaw_rad -= 2 * np.pi
    while yaw_rad < -np.pi:
        yaw_rad += 2 * np.pi

    return yaw_rad


def gps_to_local_coords(
    gps_data: np.ndarray, origin_idx: int = 0
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert GPS coordinates to a local coordinate system (X, Y in meters).
    Uses equirectangular projection (flat projection).

    Accuracy: Excellent for distances <100 km, acceptable up to 1000 km.
              For larger distances, use Haversine or UTM.

    Conversion steps:
    1. Select origin point as coordinate origin (0, 0)
    2. Compute coordinate differences (Δlat, Δlon) relative to origin
    3. Convert degrees to meters with latitude correction

    Formula:
        y = Δlat × R × π/180  (north-south)
        x = Δlon × cos(lat₀) × R × π/180  (west-east)

    Where:
        R = 6371000 m (Earth radius)
        lat₀ = origin latitude
        Δlat = lat - lat₀
        Δlon = lon - lon₀

    Args:
        gps_data: numpy array shape (N, 5) [timestamp, lat, lon, alt, speed]
        origin_idx: Origin point index (usually 0 - first point)

    Returns:
        Tuple (x_array, y_array) - coordinates in meters from the origin point

    Example:
        # GPS data: [(t1, 55.62, 37.65, ...), (t2, 55.63, 37.66, ...)]
        x, y = gps_to_local_coords(gps_data, origin_idx=0)
        # x[0], y[0] = 0, 0 (origin point)
        # x[1], y[1] = offset in meters from the first point
    """
    if gps_data is None or len(gps_data) == 0:
        return np.array([]), np.array([])

    # STEP 1: Select origin point (coordinate origin)
    # ================================================
    lat0 = gps_data[origin_idx, 1]  # origin latitude (degrees)
    lon0 = gps_data[origin_idx, 2]  # origin longitude (degrees)

    # Convert origin latitude to radians (for cos)
    lat0_rad = np.radians(lat0)

    # STEP 2: Extract all coordinates
    # ================================
    lats = gps_data[:, 1]  # All latitudes
    lons = gps_data[:, 2]  # All longitudes

    # STEP 3: Compute coordinate differences
    # =======================================
    # Latitude difference (positive = north, negative = south)
    dlat = lats - lat0
    # Longitude difference (positive = east, negative = west)
    dlon = lons - lon0

    # STEP 4: Convert degrees to meters
    # ==================================
    # Equirectangular projection formula:
    #
    # Y (north-south):
    #   1° latitude ≈ 111,320 m everywhere (meridians are parallel)
    #   Formula: y = Δlat × R × π/180
    #   Where R×π/180 = 6371000×π/180 ≈ 111,320 m/degree
    #
    #   Why constant?
    #   All meridians have the same length (20,004 km)
    #   → 1° latitude = 20,004,000 m / 180° ≈ 111,133 m
    y = dlat * EARTH_RADIUS * np.pi / 180.0

    # X (west-east):
    #   1° longitude = cos(latitude) × 111,320 m
    #   Formula: x = Δlon × cos(lat₀) × R × π/180
    #
    #   Why cos(latitude)?
    #   Parallels (latitude lines) converge toward the poles:
    #     - At equator (lat=0°):  cos(0°)=1  → 1° longitude ≈ 111,320 m
    #     - At latitude 45°:     cos(45°)≈0.707 → 1° longitude ≈ 78,710 m
    #     - At latitude 60°:     cos(60°)=0.5   → 1° longitude ≈ 55,660 m
    #     - At pole (lat=90°):   cos(90°)=0     → 1° longitude = 0 m (a point!)
    #
    #   Example for Moscow (lat ≈ 55.75°):
    #     cos(55.75°) ≈ 0.564
    #     1° longitude ≈ 111,320 × 0.564 ≈ 62,800 m
    x = dlon * np.cos(lat0_rad) * EARTH_RADIUS * np.pi / 180.0

    return x, y
