"""
=============================================================================
Module: control.vehicle_model
Description: Non-holonomic Kinematic Bicycle Model implementing vehicle dynamics,
             wheelbase constraints, steering rate saturation, and collision bounds.
=============================================================================
"""

import math
import numpy as np
from config import Config

class VehicleModel:
    def __init__(self, x=100.0, y=100.0, heading=0.0):
        # State variables
        self.x = float(x)
        self.y = float(y)
        self.heading = float(heading) # Heading angle in radians (yaw)
        self.speed = 0.0              # Linear forward velocity
        self.steer_angle = 0.0        # Current steering angle of front wheels

        # Physical constants
        self.wheelbase = Config.WHEELBASE
        self.width = Config.VEHICLE_WIDTH
        self.length = Config.VEHICLE_LENGTH
        self.max_speed = Config.MAX_SPEED
        self.max_steer = Config.MAX_STEER_ANGLE

        # Trail history for visualization
        self.trail = []

    def update(self, target_steer, target_accel, dt=1.0):
        """
        Advances the kinematic state by dt using the Bicycle Model equations.
        
        Args:
            target_steer (float): Desired steering angle in radians.
            target_accel (float): Desired acceleration (positive or braking).
            dt (float): Simulation timestep factor.
        """
        # 1. Apply steering rate limit
        steer_diff = target_steer - self.steer_angle
        steer_step = np.clip(steer_diff, -Config.STEER_RATE, Config.STEER_RATE)
        self.steer_angle += steer_step
        self.steer_angle = float(np.clip(self.steer_angle, -self.max_steer, self.max_steer))

        # 2. Update speed with acceleration and gentle rolling resistance
        self.speed += target_accel * dt
        self.speed -= 0.01 * self.speed # Rolling resistance
        self.speed = float(np.clip(self.speed, Config.MIN_SPEED, self.max_speed))

        # 3. Bicycle kinematics
        # dx/dt = v * cos(theta)
        # dy/dt = v * sin(theta)
        # dtheta/dt = (v / L) * tan(delta)
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt
        angular_velocity = (self.speed / self.wheelbase) * math.tan(self.steer_angle)
        self.heading = (self.heading + angular_velocity * dt) % (2 * math.pi)

        # Update position history (sample every few frames)
        if len(self.trail) == 0 or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 5.0:
            self.trail.append((self.x, self.y))
            if len(self.trail) > 400:
                self.trail.pop(0)

    def get_bounding_polygon(self):
        """Returns 4 corner coordinates of the vehicle box in world space."""
        half_l = self.length / 2.0
        half_w = self.width / 2.0
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)

        corners = [
            (-half_l, -half_w),
            (half_l, -half_w),
            (half_l, half_w),
            (-half_l, half_w)
        ]

        world_corners = []
        for lx, ly in corners:
            wx = self.x + (lx * cos_h - ly * sin_h)
            wy = self.y + (lx * sin_h + ly * cos_h)
            world_corners.append((wx, wy))

        return world_corners

    def reset(self, x, y, heading=0.0):
        self.x = float(x)
        self.y = float(y)
        self.heading = float(heading)
        self.speed = 0.0
        self.steer_angle = 0.0
        self.trail = []
