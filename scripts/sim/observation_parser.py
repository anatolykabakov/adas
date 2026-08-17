"""
Observation parser for MetaDrive environment
"""

import numpy as np
from metadrive.component.navigation_module.trajectory_navigation import (
    TrajectoryNavigation,
)


class ObservationParser:
    """Parse observations from MetaDrive environment into structured data"""

    def __init__(self, env):
        self.env = env

    def make_odometry(self, agent):
        """Get vehicle odometry (position, heading, velocity, speed, acceleration, yaw_rate, roll, pitch)"""
        return {
            "position": np.array(agent.position),
            "heading": agent.heading_theta,
            "velocity": np.array(agent.velocity),
            "speed": agent.speed,
            "acceleration": np.array(agent.acc)
            if hasattr(agent, "acc")
            else np.array([0, 0, 0]),
            "yaw_rate": agent.yaw_rate if hasattr(agent, "yaw_rate") else 0.0,
            "roll": agent.roll if hasattr(agent, "roll") else 0.0,
            "pitch": agent.pitch if hasattr(agent, "pitch") else 0.0,
        }

    def make_lane_observations(self, agent, max_distance=30.0):
        """
        Get lane polygons (left and right boundaries of current lane and next lane if available)
        Returns polygon points in local vehicle coordinates
        """
        current_lane = agent.lane

        # Try to get next lane if available (for intersections/road changes)
        lane_polygon = None
        if (
            hasattr(agent, "navigation")
            and type(agent.navigation) != TrajectoryNavigation
        ):
            next_road = agent.navigation.next_road
        else:
            next_road = None

        if next_road:
            road_network = self.env.engine.current_map.road_network
            next_road_lanes = next_road.get_lanes(road_network)

            if next_road_lanes:
                # Get polygons
                next_road_polygons = [np.array(x.polygon) for x in next_road_lanes]
                current_road_polygons = [np.array(current_lane.polygon)]

                # Find closest matching next lane
                next_road_starts = np.array([x[0] for x in next_road_polygons])
                current_road_ends = np.array(
                    [
                        x[x.shape[0] // 2 - 1 : x.shape[0] // 2]
                        for x in current_road_polygons
                    ]
                )

                pairwise_diff = np.linalg.norm(
                    (current_road_ends[:, None] - next_road_starts[None])[0, 0], axis=(-1)
                )
                next_lane_idx = pairwise_diff.argmin()
                next_lane = next_road_lanes[next_lane_idx]

                # Concatenate current and next lane polygons
                current_lane_polygon = current_lane.polygon
                next_lane_polygon = next_lane.polygon

                lane_polygon = np.concatenate(
                    [
                        current_lane_polygon[
                            : len(current_lane_polygon) // 2
                        ],  # First half of current
                        next_lane_polygon[1:-1],  # Next lane without endpoints
                        current_lane_polygon[
                            len(current_lane_polygon) // 2 :
                        ],  # Second half of current
                    ]
                )

        # Use current lane if next lane concatenation failed
        if lane_polygon is None:
            lane_polygon = np.array(current_lane.polygon)

        # Split into right and left sides
        right_side_global = np.array(lane_polygon[: len(lane_polygon) // 2])
        left_side_global = np.array(lane_polygon[len(lane_polygon) // 2 :][::-1])

        # Find closest point to agent (more efficient than filtering first)
        start_index = np.argmin(
            np.linalg.norm(right_side_global - agent.position, axis=1)
        )
        right_side_global = right_side_global[start_index:]
        right_side_global = right_side_global[
            np.linalg.norm(right_side_global - agent.position, axis=1) < max_distance
        ]

        start_index = np.argmin(np.linalg.norm(left_side_global - agent.position, axis=1))
        left_side_global = left_side_global[start_index:]
        left_side_global = left_side_global[
            np.linalg.norm(left_side_global - agent.position, axis=1) < max_distance
        ]

        # Convert from WORLD frame to ROAD frame
        # Road frame origin is at vehicle's position (bottom of front wheel per ISO8855 standard)
        # convert_to_local_coordinates does: road_coords = world_coords - vehicle_position
        right_road = np.array(
            [
                agent.convert_to_local_coordinates(p, agent.position)
                for p in right_side_global
            ]
        )
        left_road = np.array(
            [
                agent.convert_to_local_coordinates(p, agent.position)
                for p in left_side_global
            ]
        )

        # Filter out points behind the vehicle (x < 0 in road frame)
        # Camera only sees forward, so points with negative x should be removed
        # right_local = right_local[right_local[:, 0] >= 0]
        # left_local = left_local[left_local[:, 0] >= 0]

        # Fit polynomials only if we have enough points
        if len(right_road) >= 2 and len(left_road) >= 2:
            p = min(3, right_road.shape[0] - 1)
            right_a = np.poly1d(np.polyfit(right_road[:, 0], right_road[:, 1], p))
            left_a = np.poly1d(np.polyfit(left_road[:, 0], left_road[:, 1], p))
        else:
            # Not enough points for polynomial fitting
            right_a = np.poly1d([0, 0, 0, 0])  # Zero polynomial
            left_a = np.poly1d([0, 0, 0, 0])  # Zero polynomial

        return {
            "right_polyline": right_a,
            "left_polyline": left_a,
            "right_road": right_road,  # Already in ROAD frame (vehicle local coordinates)
            "left_road": left_road,  # Already in ROAD frame (vehicle local coordinates)
            "right_world": right_side_global,
            "left_world": left_side_global,
        }

    def make_camera_image(self):
        """Extract camera image from ego-mounted RGB sensor."""
        try:
            from metadrive.constants import DEFAULT_SENSOR_OFFSET

            rgb_sensor = self.env.engine.get_sensor("rgb")
            if rgb_sensor is None:
                return None
            # Explicit mount: same pose CameraParams / overlay use
            hpr = getattr(self, "_cam_hpr", None) or (0.0, 0.59681, 0.0)
            offset = getattr(self, "_cam_offset", None) or tuple(DEFAULT_SENSOR_OFFSET)
            img = rgb_sensor.perceive(
                to_float=True,
                new_parent_node=self.env.agent.origin,
                position=list(offset),
                hpr=list(hpr),
            )
            return img
        except Exception:
            return None

    def set_camera_mount(self, offset, hpr) -> None:
        self._cam_offset = tuple(float(x) for x in offset)
        self._cam_hpr = tuple(float(x) for x in hpr)

    def make_perception_data(self):
        """Get all observation data: odometry, lane polygons, and camera image"""
        agent = self.env.agent

        return {
            "odometry": self.make_odometry(agent),
            "lanes": self.make_lane_observations(agent),
            "camera_image": self.make_camera_image(),
        }
