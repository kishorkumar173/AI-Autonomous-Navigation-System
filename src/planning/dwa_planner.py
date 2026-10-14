"""
=============================================================================
Module: planning.dwa_planner
Description: Dynamic Window Approach (DWA) and Reactive Potential Field
             local obstacle avoidance for dynamic and unmapped objects.
=============================================================================
"""

import math
import numpy as np
from config import Config

class DWAPlanner:
    def __init__(self):
        # Kinematic limits for local window
        self.max_speed = Config.MAX_SPEED
        self.min_speed = 0.0
        self.max_yaw_rate = math.radians(45) # 45 deg/sec
        self.max_accel = Config.MAX_ACCELERATION * 60 # per second
        self.max_dyaw_rate = math.radians(60)
        self.v_resolution = 0.5
        self.yaw_rate_resolution = math.radians(3)
        self.dt = 0.1
        self.predict_time = 1.2 # Lookahead horizon in seconds

        # Cost function weights
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.5

    def plan_reactive_avoidance(self, vehicle_x, vehicle_y, vehicle_heading, current_speed, 
                                target_point, lidar_points):
        """
        Calculates reactive steering nudge and speed adjustment when obstacles
        encroach on the safety zone using virtual repulsive forces.
        
        Args:
            vehicle_x, vehicle_y: Current vehicle position.
            vehicle_heading: Current vehicle heading in radians.
            current_speed: Current vehicle speed.
            target_point: Current lookahead waypoint (gx, gy).
            lidar_points: List of LIDAR hit dicts {'distance', 'relative_angle', 'angle'}.
            
        Returns:
            tuple: (steering_nudge, speed_scale, is_avoiding)
        """
        repulsive_x = 0.0
        repulsive_y = 0.0
        critical_count = 0
        min_front_dist = float('inf')

        for pt in lidar_points:
            d = pt["distance"]
            rel_deg = math.degrees(pt["relative_angle"])
            if rel_deg > 180:
                rel_deg -= 360

            # Only consider forward-facing arc (-75 to +75 degrees)
            if abs(rel_deg) < 75 and d < (Config.LIDAR_SAFE_DISTANCE * 2.2):
                if abs(rel_deg) < 25:
                    min_front_dist = min(min_front_dist, d)

                # Repulsive strength inversely proportional to distance squared
                force = 1.0 / max(5.0, d) ** 1.8
                ray_angle = pt["angle"]
                # Push away from obstacle
                repulsive_x -= math.cos(ray_angle) * force
                repulsive_y -= math.sin(ray_angle) * force
                critical_count += 1

        is_avoiding = critical_count > 0 and min_front_dist < (Config.LIDAR_SAFE_DISTANCE * 1.8)

        if not is_avoiding:
            return 0.0, 1.0, False

        # Compute repulsive angle relative to current heading
        repulsive_angle = math.atan2(repulsive_y, repulsive_x)
        steering_nudge = (repulsive_angle - vehicle_heading + math.pi) % (2 * math.pi) - math.pi
        steering_nudge = float(np.clip(steering_nudge, -Config.MAX_STEER_ANGLE, Config.MAX_STEER_ANGLE))

        # Moderate speed when avoiding
        speed_scale = max(0.2, min(1.0, min_front_dist / (Config.LIDAR_SAFE_DISTANCE * 1.8)))

        # Emergency brake if extremely close
        if min_front_dist < Config.LIDAR_SAFE_DISTANCE * 0.7:
            speed_scale = 0.0

        return steering_nudge, speed_scale, True
