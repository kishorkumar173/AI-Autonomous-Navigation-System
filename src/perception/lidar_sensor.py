"""
=============================================================================
Module: perception.lidar_sensor
Description: High-fidelity 2D LIDAR sensor simulation using 360-degree raycasting,
             obstacle boundary intersection, and simulated Gaussian noise.
=============================================================================
"""

import math
import numpy as np
from config import Config

class LidarSensor:
    def __init__(self, num_rays=Config.LIDAR_NUM_RAYS, max_range=Config.LIDAR_MAX_RANGE):
        self.num_rays = num_rays
        self.max_range = max_range
        # Calculate ray angular offsets evenly distributed over 360 degrees
        self.angles = [i * (2 * math.pi / num_rays) for i in range(num_rays)]
        self.scan_results = []  # List of tuples: (angle, distance, hit_point)
        self.sector_distances = {"front": max_range, "front_left": max_range, 
                                 "front_right": max_range, "left": max_range, "right": max_range}

    def scan(self, vehicle_x, vehicle_y, vehicle_heading, obstacles, canvas_width, canvas_height):
        """
        Executes a 360-degree range scan from the vehicle center.
        
        Args:
            vehicle_x (float): Current X coordinate of vehicle.
            vehicle_y (float): Current Y coordinate of vehicle.
            vehicle_heading (float): Current heading angle of vehicle in radians.
            obstacles (list): List of obstacle dicts with shape properties.
            canvas_width (int): Boundary width of the simulation canvas.
            canvas_height (int): Boundary height of the simulation canvas.
            
        Returns:
            list: List of dicts with keys 'angle', 'distance', 'hit_x', 'hit_y'.
        """
        self.scan_results = []
        sectors = {"front": [], "front_left": [], "front_right": [], "left": [], "right": []}

        for rel_angle in self.angles:
            global_angle = (vehicle_heading + rel_angle) % (2 * math.pi)
            cos_a = math.cos(global_angle)
            sin_a = math.sin(global_angle)

            # Default hit point is max range boundary
            min_dist = self.max_range
            hit_x = vehicle_x + cos_a * self.max_range
            hit_y = vehicle_y + sin_a * self.max_range

            # 1. Check canvas boundary limits
            # Left boundary (x = 0)
            if cos_a < -1e-5:
                d = (0 - vehicle_x) / cos_a
                if 0 <= d < min_dist:
                    min_dist = d
            # Right boundary (x = canvas_width)
            if cos_a > 1e-5:
                d = (canvas_width - vehicle_x) / cos_a
                if 0 <= d < min_dist:
                    min_dist = d
            # Top boundary (y = 0)
            if sin_a < -1e-5:
                d = (0 - vehicle_y) / sin_a
                if 0 <= d < min_dist:
                    min_dist = d
            # Bottom boundary (y = canvas_height)
            if sin_a > 1e-5:
                d = (canvas_height - vehicle_y) / sin_a
                if 0 <= d < min_dist:
                    min_dist = d

            # 2. Check collision with obstacles (rectangles and circles)
            for obs in obstacles:
                obs_type = obs.get("type", "rect")
                if obs_type == "rect":
                    d = self._ray_rect_intersect(vehicle_x, vehicle_y, cos_a, sin_a, obs)
                    if d is not None and 0 <= d < min_dist:
                        min_dist = d
                elif obs_type == "circle":
                    d = self._ray_circle_intersect(vehicle_x, vehicle_y, cos_a, sin_a, obs)
                    if d is not None and 0 <= d < min_dist:
                        min_dist = d

            # Add simulated sensor measurement noise (Gaussian)
            noisy_dist = max(0.0, min_dist + np.random.normal(0, Config.LIDAR_NOISE_STD))
            noisy_dist = min(self.max_range, noisy_dist)

            hit_x = vehicle_x + cos_a * noisy_dist
            hit_y = vehicle_y + sin_a * noisy_dist

            self.scan_results.append({
                "angle": global_angle,
                "relative_angle": rel_angle,
                "distance": noisy_dist,
                "hit_x": hit_x,
                "hit_y": hit_y
            })

            # Sector classification relative to forward direction
            deg = math.degrees(rel_angle)
            if deg > 180:
                deg -= 360  # Normalize to [-180, 180]

            if abs(deg) <= 20:
                sectors["front"].append(noisy_dist)
            elif 20 < deg <= 65:
                sectors["front_right"].append(noisy_dist)
            elif -65 <= deg < -20:
                sectors["front_left"].append(noisy_dist)
            elif 65 < deg <= 120:
                sectors["right"].append(noisy_dist)
            elif -120 <= deg < -65:
                sectors["left"].append(noisy_dist)

        # Compute minimum distance in each sector
        for sector, dists in sectors.items():
            self.sector_distances[sector] = min(dists) if dists else self.max_range

        return self.scan_results

    def _ray_circle_intersect(self, ox, oy, dx, dy, circle):
        cx, cy, r = circle["x"], circle["y"], circle["radius"]
        # Vector from ray origin to circle center
        fx = ox - cx
        fy = oy - cy

        a = dx*dx + dy*dy
        b = 2 * (fx*dx + fy*dy)
        c = (fx*fx + fy*fy) - r*r

        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return None
        
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)

        if t1 >= 0:
            return t1
        if t2 >= 0:
            return t2
        return None

    def _ray_rect_intersect(self, ox, oy, dx, dy, rect):
        rx, ry, rw, rh = rect["x"], rect["y"], rect["width"], rect["height"]
        tmin = -float('inf')
        tmax = float('inf')

        # X-slab
        if abs(dx) > 1e-6:
            tx1 = (rx - ox) / dx
            tx2 = (rx + rw - ox) / dx
            tmin = max(tmin, min(tx1, tx2))
            tmax = min(tmax, max(tx1, tx2))
        else:
            if ox < rx or ox > rx + rw:
                return None

        # Y-slab
        if abs(dy) > 1e-6:
            ty1 = (ry - oy) / dy
            ty2 = (ry + rh - oy) / dy
            tmin = max(tmin, min(ty1, ty2))
            tmax = min(tmax, max(ty1, ty2))
        else:
            if oy < ry or oy > ry + rh:
                return None

        if tmax >= tmin and tmax >= 0:
            return tmin if tmin >= 0 else tmax
        return None
