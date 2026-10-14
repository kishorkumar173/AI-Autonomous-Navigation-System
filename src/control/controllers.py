"""
=============================================================================
Module: control.controllers
Description: Industrial path tracking and speed regulation controllers:
             Pure Pursuit Geometric Tracking and PID Speed Regulation.
=============================================================================
"""

import math
import numpy as np
from config import Config

class PurePursuitController:
    def __init__(self, lookahead_dist=Config.LOOKAHEAD_DISTANCE, wheelbase=Config.WHEELBASE):
        self.lookahead_dist = lookahead_dist
        self.wheelbase = wheelbase
        self.current_target_idx = 0

    def compute_steering(self, vehicle_x, vehicle_y, vehicle_heading, path):
        """
        Calculates steering angle required to track the target waypoint.
        
        Args:
            vehicle_x, vehicle_y: Vehicle current position.
            vehicle_heading: Vehicle heading angle in radians.
            path: List of (x, y) waypoints.
            
        Returns:
            tuple: (steering_angle, target_point, is_at_end)
        """
        if not path or len(path) == 0:
            return 0.0, (vehicle_x, vehicle_y), True

        # Find closest point on path to update index
        min_dist = float('inf')
        closest_idx = self.current_target_idx
        for i in range(self.current_target_idx, min(len(path), self.current_target_idx + 10)):
            d = math.hypot(path[i][0] - vehicle_x, path[i][1] - vehicle_y)
            if d < min_dist:
                min_dist = d
                closest_idx = i
        self.current_target_idx = closest_idx

        # Search forward for lookahead point that exceeds lookahead distance
        target_point = path[-1]
        for i in range(self.current_target_idx, len(path)):
            d = math.hypot(path[i][0] - vehicle_x, path[i][1] - vehicle_y)
            if d >= self.lookahead_dist:
                target_point = path[i]
                break

        # Check if reaching final waypoint
        dist_to_final = math.hypot(path[-1][0] - vehicle_x, path[-1][1] - vehicle_y)
        is_at_end = dist_to_final < Config.GOAL_TOLERANCE

        # Pure Pursuit geometry:
        # alpha is angle between vehicle heading and vector to lookahead target
        dx = target_point[0] - vehicle_x
        dy = target_point[1] - vehicle_y
        angle_to_target = math.atan2(dy, dx)
        alpha = (angle_to_target - vehicle_heading + math.pi) % (2 * math.pi) - math.pi

        # Steering angle formula: delta = arctan(2 * L * sin(alpha) / Ld)
        Ld = max(10.0, math.hypot(dx, dy))
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)
        steer = float(np.clip(steer, -Config.MAX_STEER_ANGLE, Config.MAX_STEER_ANGLE))

        return steer, target_point, is_at_end

    def reset(self):
        self.current_target_idx = 0


class PIDController:
    """Proportional-Integral-Derivative controller for velocity regulation."""
    def __init__(self, kp=0.4, ki=0.02, kd=0.08):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target_value, current_value, dt=1.0):
        error = target_value - current_value
        self.integral += error * dt
        # Anti-windup clamping
        self.integral = float(np.clip(self.integral, -5.0, 5.0))
        derivative = (error - self.prev_error) / max(0.001, dt)
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
