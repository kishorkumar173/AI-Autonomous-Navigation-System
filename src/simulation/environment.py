"""
=============================================================================
Module: simulation.environment
Description: Multi-scenario simulation world supporting static obstacles,
             dynamic moving agents (pedestrians, cars), lanes, and goal points.
=============================================================================
"""

import math
from config import Config

class SimulationEnvironment:
    def __init__(self, canvas_width=Config.CANVAS_WIDTH, canvas_height=Config.CANVAS_HEIGHT):
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.current_preset = "urban_streets"
        self.obstacles = []
        self.dynamic_obstacles = []
        self.lane_lines = []
        self.start_pos = (80, 80)
        self.start_heading = 0.0
        self.goal_pos = (750, 600)

        # Load default scenario
        self.load_preset("urban_streets")

    def load_preset(self, preset_name):
        self.current_preset = preset_name
        self.obstacles = []
        self.dynamic_obstacles = []
        self.lane_lines = []

        if preset_name == "urban_streets":
            self.start_pos = (80, 150)
            self.start_heading = 0.0
            self.goal_pos = (780, 560)

            # Road borders and static structures
            self.obstacles.extend([
                {"id": "bldg_1", "type": "rect", "x": 0, "y": 0, "width": 850, "height": 60, "category": "barrier"},
                {"id": "bldg_2", "type": "rect", "x": 0, "y": 240, "width": 320, "height": 180, "category": "building"},
                {"id": "bldg_3", "type": "rect", "x": 450, "y": 140, "width": 180, "height": 260, "category": "building"},
                {"id": "parked_car_1", "type": "rect", "x": 240, "y": 110, "width": 60, "height": 30, "category": "vehicle"},
                {"id": "cone_1", "type": "circle", "x": 390, "y": 100, "radius": 14, "category": "traffic_cone"},
                {"id": "cone_2", "type": "circle", "x": 420, "y": 110, "radius": 14, "category": "traffic_cone"},
                {"id": "bldg_4", "type": "rect", "x": 100, "y": 500, "width": 300, "height": 140, "category": "building"},
                {"id": "bldg_5", "type": "rect", "x": 520, "y": 460, "width": 140, "height": 180, "category": "building"},
            ])

            # Dynamic moving pedestrian across intersection
            self.dynamic_obstacles.append({
                "id": "ped_1", "type": "circle", "x": 360.0, "y": 220.0, "radius": 12,
                "vx": 0.0, "vy": 1.2, "min_y": 80.0, "max_y": 380.0, "category": "pedestrian"
            })
            # Dynamic cross-traffic vehicle
            self.dynamic_obstacles.append({
                "id": "veh_cross", "type": "rect", "x": 680.0, "y": 320.0, "width": 50, "height": 26,
                "vx": -1.5, "vy": 0.0, "min_x": 400.0, "max_x": 750.0, "category": "vehicle"
            })

            # Lane markings
            self.lane_lines = [
                ((0, 180), (850, 180)),
                ((380, 0), (380, 700)),
                ((0, 460), (850, 460))
            ]

        elif preset_name == "warehouse":
            self.start_pos = (80, 80)
            self.start_heading = 0.0
            self.goal_pos = (760, 620)

            # Warehouse shelving racks
            rack_w, rack_h = 160, 45
            for row in [150, 270, 390, 510]:
                self.obstacles.append({"id": f"rack_l_{row}", "type": "rect", "x": 100, "y": row, "width": rack_w, "height": rack_h, "category": "obstacle"})
                self.obstacles.append({"id": f"rack_r_{row}", "type": "rect", "x": 450, "y": row, "width": rack_w, "height": rack_h, "category": "obstacle"})

            # AGV Forklift patrolling
            self.dynamic_obstacles.append({
                "id": "forklift_1", "type": "rect", "x": 340.0, "y": 180.0, "width": 40, "height": 30,
                "vx": 0.0, "vy": 1.8, "min_y": 100.0, "max_y": 580.0, "category": "vehicle"
            })

        elif preset_name == "obstacle_slalom":
            self.start_pos = (60, 350)
            self.start_heading = 0.0
            self.goal_pos = (780, 350)

            # Alternating slalom barriers
            for i, x in enumerate([180, 300, 420, 540, 660]):
                y = 120 if i % 2 == 0 else 380
                h = 220
                self.obstacles.append({
                    "id": f"gate_{i}", "type": "rect", "x": x, "y": y, "width": 30, "height": h, "category": "barrier"
                })

    def update_dynamic_obstacles(self):
        """Advances positions of dynamic pedestrians and vehicles."""
        for dyn in self.dynamic_obstacles:
            # Update coordinate
            dyn["x"] += dyn.get("vx", 0.0)
            dyn["y"] += dyn.get("vy", 0.0)

            # Bounce reversal on boundary
            if "min_y" in dyn and "max_y" in dyn:
                if dyn["y"] <= dyn["min_y"] or dyn["y"] >= dyn["max_y"]:
                    dyn["vy"] *= -1
            if "min_x" in dyn and "max_x" in dyn:
                if dyn["x"] <= dyn["min_x"] or dyn["x"] >= dyn["max_x"]:
                    dyn["vx"] *= -1

    def get_all_obstacles(self):
        """Returns consolidated list of static and current dynamic obstacles."""
        return self.obstacles + self.dynamic_obstacles

    def add_user_obstacle(self, x, y, radius=18):
        """Allows interactive user to drop an obstacle on the canvas."""
        obs_id = f"user_obs_{len(self.obstacles)}"
        self.obstacles.append({
            "id": obs_id,
            "type": "circle",
            "x": float(x),
            "y": float(y),
            "radius": float(radius),
            "category": "traffic_cone"
        })
