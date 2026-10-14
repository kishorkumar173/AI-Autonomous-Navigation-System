"""
=============================================================================
Project: AI-Based Autonomous Navigation System
File: config.py
Description: Centralized configuration parameters for vehicle kinematics,
             sensors, path planning algorithms, and simulation visualization.
=============================================================================
"""

import math

class Config:
    # -------------------------------------------------------------------------
    # Window & Display Settings
    # -------------------------------------------------------------------------
    WINDOW_WIDTH = 1200
    WINDOW_HEIGHT = 700
    CANVAS_WIDTH = 850
    CANVAS_HEIGHT = 700
    HUD_WIDTH = WINDOW_WIDTH - CANVAS_WIDTH
    FPS = 60

    # -------------------------------------------------------------------------
    # Color Palette (RGB)
    # -------------------------------------------------------------------------
    COLOR_BG = (22, 27, 34)           # Dark slate background
    COLOR_GRID = (35, 42, 54)         # Faint coordinate grid
    COLOR_ROAD = (40, 44, 52)         # Road surface
    COLOR_LANE = (230, 200, 50)       # Yellow lane markings
    COLOR_OBSTACLE = (220, 60, 60)    # Red for static obstacles
    COLOR_DYNAMIC_OBS = (255, 140, 0) # Orange for moving obstacles
    COLOR_PEDESTRIAN = (255, 105, 180)# Pink for simulated pedestrians
    COLOR_VEHICLE = (46, 204, 113)    # Emerald green for autonomous vehicle
    COLOR_PATH_GLOBAL = (52, 152, 219)# Blue for A* planned path
    COLOR_PATH_TRAIL = (46, 204, 113) # Green faint trail for driven path
    COLOR_GOAL = (241, 196, 15)       # Yellow-gold target goal
    COLOR_LIDAR_RAY = (0, 255, 200)   # Cyan for LIDAR rays
    COLOR_LIDAR_HIT = (255, 70, 70)   # Red dot for LIDAR collision contact
    COLOR_HUD_BG = (15, 18, 23)       # Deep carbon HUD panel
    COLOR_TEXT = (240, 246, 252)      # Primary text color
    COLOR_TEXT_MUTED = (139, 148, 158)# Subdued label text

    # -------------------------------------------------------------------------
    # Vehicle Physical Kinematics (Bicycle Model)
    # -------------------------------------------------------------------------
    VEHICLE_WIDTH = 24.0              # Pixels (~1.8 meters)
    VEHICLE_LENGTH = 44.0             # Pixels (~4.2 meters)
    WHEELBASE = 32.0                  # Distance between front and rear axles
    MAX_SPEED = 5.5                   # Max forward velocity (pixels/frame)
    MIN_SPEED = 0.0                   # Reverse prohibited for forward pipeline
    MAX_ACCELERATION = 0.15           # Maximum acceleration rate
    MAX_BRAKE = 0.35                  # Emergency brake deceleration rate
    MAX_STEER_ANGLE = math.radians(35)# Maximum steering angle (+/- 35 degrees)
    STEER_RATE = math.radians(6)      # Max steering change per frame

    # -------------------------------------------------------------------------
    # LIDAR Sensor Specifications
    # -------------------------------------------------------------------------
    LIDAR_NUM_RAYS = 36               # 360-degree scan (10-degree resolution)
    LIDAR_MAX_RANGE = 180.0           # Maximum detection range in pixels
    LIDAR_NOISE_STD = 0.8             # Simulated Gaussian measurement noise
    LIDAR_SAFE_DISTANCE = 40.0        # Critical distance trigger for braking

    # -------------------------------------------------------------------------
    # Forward Camera / Vision Sensor
    # -------------------------------------------------------------------------
    CAMERA_FOV = math.radians(70)     # 70-degree forward field of view
    CAMERA_RANGE = 200.0              # Camera forward viewing depth
    TTC_WARNING_THRESHOLD = 1.8       # Time To Collision (seconds) warning

    # -------------------------------------------------------------------------
    # Path Planning Settings (A* & DWA)
    # -------------------------------------------------------------------------
    GRID_RESOLUTION = 20              # Grid cell size in pixels for A*
    OBSTACLE_SAFETY_MARGIN = 24       # Inflation radius around obstacles
    LOOKAHEAD_DISTANCE = 35.0         # Pure pursuit target search distance
    GOAL_TOLERANCE = 25.0             # Distance to declare goal reached

    # -------------------------------------------------------------------------
    # Telemetry and Logging
    # -------------------------------------------------------------------------
    LOG_TELEMETRY = True
    OUTPUTS_DIR = "outputs"
