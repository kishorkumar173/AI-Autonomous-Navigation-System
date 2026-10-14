"""
=============================================================================
Module: simulation.simulator_gui
Description: Real-time interactive Pygame HUD simulator rendering 2D vehicle
             kinematics, LIDAR point cloud, OpenCV camera feed, and navigation telemetry.
=============================================================================
"""

import math
import pygame
import cv2
import numpy as np
from config import Config

class SimulatorGUI:
    def __init__(self, vehicle, environment, lidar, camera, planner, controller, dwa, logger):
        self.vehicle = vehicle
        self.env = environment
        self.lidar = lidar
        self.camera = camera
        self.planner = planner
        self.controller = controller
        self.dwa = dwa
        self.logger = logger

        pygame.init()
        pygame.display.set_caption("AI-Based Autonomous Navigation System | Proof-of-Work Portfolio")
        self.screen = pygame.display.set_mode((Config.WINDOW_WIDTH, Config.WINDOW_HEIGHT))
        self.clock = pygame.time.Clock()

        # Fonts
        self.font_title = pygame.font.SysFont("Segoe UI", 18, bold=True)
        self.font_main = pygame.font.SysFont("Segoe UI", 14)
        self.font_bold = pygame.font.SysFont("Segoe UI", 14, bold=True)
        self.font_small = pygame.font.SysFont("Consolas", 12)

        # Simulation states
        self.paused = False
        self.current_state = "GLOBAL_PLANNING"
        self.global_path = []
        self.target_lookahead_pt = (0, 0)
        self.plan_global_path()

    def plan_global_path(self):
        """Recomputes global A* path from current vehicle position to goal."""
        self.current_state = "GLOBAL_PLANNING"
        self.global_path = self.planner.plan(
            (self.vehicle.x, self.vehicle.y),
            self.env.goal_pos,
            self.env.get_all_obstacles(),
            Config.CANVAS_WIDTH,
            Config.CANVAS_HEIGHT
        )
        self.controller.reset()
        self.current_state = "PURSUIT"

    def handle_events(self):
        """Processes interactive user clicks and keyboard commands."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                if mx < Config.CANVAS_WIDTH:
                    # User dropped an obstacle on the map!
                    self.env.add_user_obstacle(mx, my)
                    self.plan_global_path() # Auto-replanning upon environmental change

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.vehicle.reset(self.env.start_pos[0], self.env.start_pos[1], self.env.start_heading)
                    self.plan_global_path()
                    self.logger.reset()
                elif event.key == pygame.K_p:
                    self.plan_global_path()
                elif event.key == pygame.K_1:
                    self.env.load_preset("urban_streets")
                    self.vehicle.reset(self.env.start_pos[0], self.env.start_pos[1], self.env.start_heading)
                    self.plan_global_path()
                elif event.key == pygame.K_2:
                    self.env.load_preset("warehouse")
                    self.vehicle.reset(self.env.start_pos[0], self.env.start_pos[1], self.env.start_heading)
                    self.plan_global_path()
                elif event.key == pygame.K_3:
                    self.env.load_preset("obstacle_slalom")
                    self.vehicle.reset(self.env.start_pos[0], self.env.start_pos[1], self.env.start_heading)
                    self.plan_global_path()
                elif event.key == pygame.K_s:
                    self.logger.export_summary(self.global_path, self.env.get_all_obstacles())
                    pygame.image.save(self.screen, f"{Config.OUTPUTS_DIR}/simulation_screenshot.png")
                    print("[INFO] Screenshot & Telemetry Exported to /outputs folder!")

        return True

    def step(self):
        """Executes one simulation tick through perception, planning, and control."""
        if self.paused:
            return

        # 1. Update dynamic moving obstacles in the environment
        self.env.update_dynamic_obstacles()

        # 2. Perception: Run LIDAR 360-degree raycasting
        lidar_data = self.lidar.scan(
            self.vehicle.x, self.vehicle.y, self.vehicle.heading,
            self.env.get_all_obstacles(), Config.CANVAS_WIDTH, Config.CANVAS_HEIGHT
        )

        # 3. Perception: Run Forward Camera Computer Vision detection
        cv_detections = self.camera.detect(
            self.vehicle.x, self.vehicle.y, self.vehicle.heading,
            self.vehicle.speed, self.env.get_all_obstacles()
        )

        # 4. Control: Global path tracking with Pure Pursuit
        steer_target, target_pt, is_goal_reached = self.controller.compute_steering(
            self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.global_path
        )
        self.target_lookahead_pt = target_pt

        # 5. Planning: Local reactive obstacle avoidance (DWA/Vector field)
        steer_nudge, speed_scale, is_avoiding = self.dwa.plan_reactive_avoidance(
            self.vehicle.x, self.vehicle.y, self.vehicle.heading,
            self.vehicle.speed, target_pt, lidar_data
        )

        # 6. Decision Making State Machine
        dist_to_goal = math.hypot(self.env.goal_pos[0] - self.vehicle.x, self.env.goal_pos[1] - self.vehicle.y)
        min_front_lidar = self.lidar.sector_distances.get("front", Config.LIDAR_MAX_RANGE)

        if is_goal_reached or dist_to_goal < Config.GOAL_TOLERANCE:
            self.current_state = "GOAL_REACHED"
            target_accel = -Config.MAX_BRAKE
            final_steer = 0.0
        elif min_front_lidar < Config.LIDAR_SAFE_DISTANCE * 0.7:
            self.current_state = "EMERGENCY_BRAKE"
            target_accel = -Config.MAX_BRAKE
            final_steer = steer_target + steer_nudge * 1.5
        elif is_avoiding:
            self.current_state = "LOCAL_AVOIDANCE"
            final_steer = steer_target * 0.3 + steer_nudge * 0.7
            target_speed = Config.MAX_SPEED * speed_scale
            target_accel = 0.08 if self.vehicle.speed < target_speed else -0.15
        else:
            self.current_state = "PURSUIT"
            final_steer = steer_target
            target_speed = Config.MAX_SPEED
            target_accel = 0.12 if self.vehicle.speed < target_speed else -0.05

        # 7. Apply actions to Vehicle Kinematic Model
        self.vehicle.update(final_steer, target_accel)

        # 8. Log telemetry
        self.logger.log_frame(
            self.vehicle.speed, self.vehicle.steer_angle,
            min(self.lidar.sector_distances.values()),
            dist_to_goal, self.vehicle.x, self.vehicle.y, self.current_state
        )

    def render(self):
        """Renders the graphical frame."""
        self.screen.fill(Config.COLOR_BG)

        # --- A. Render Simulation Canvas ---
        self._render_canvas()

        # --- B. Render Telemetry HUD ---
        self._render_hud()

        pygame.display.flip()
        self.clock.tick(Config.FPS)

    def _render_canvas(self):
        canvas_rect = pygame.Rect(0, 0, Config.CANVAS_WIDTH, Config.CANVAS_HEIGHT)
        pygame.draw.rect(self.screen, Config.COLOR_ROAD, canvas_rect)

        # Subtle road lanes
        for line in self.env.lane_lines:
            pygame.draw.line(self.screen, Config.COLOR_LANE, line[0], line[1], 2)

        # Draw Goal target
        gx, gy = self.env.goal_pos
        pygame.draw.circle(self.screen, Config.COLOR_GOAL, (int(gx), int(gy)), int(Config.GOAL_TOLERANCE), 2)
        pygame.draw.circle(self.screen, Config.COLOR_GOAL, (int(gx), int(gy)), 6)

        # Draw Global Planned Path (A*)
        if len(self.global_path) > 1:
            points = [(int(p[0]), int(p[1])) for p in self.global_path]
            pygame.draw.lines(self.screen, Config.COLOR_PATH_GLOBAL, False, points, 3)
            for pt in points:
                pygame.draw.circle(self.screen, Config.COLOR_PATH_GLOBAL, pt, 4)

        # Draw Trajectory Trail
        if len(self.vehicle.trail) > 1:
            trail_pts = [(int(p[0]), int(p[1])) for p in self.vehicle.trail]
            pygame.draw.lines(self.screen, Config.COLOR_PATH_TRAIL, False, trail_pts, 2)

        # Draw Obstacles
        for obs in self.env.get_all_obstacles():
            obs_type = obs.get("type", "rect")
            cat = obs.get("category", "obstacle")
            color = Config.COLOR_OBSTACLE
            if cat == "pedestrian":
                color = Config.COLOR_PEDESTRIAN
            elif cat == "vehicle":
                color = Config.COLOR_DYNAMIC_OBS
            elif cat == "traffic_cone":
                color = (255, 165, 0)

            if obs_type == "rect":
                pygame.draw.rect(self.screen, color, pygame.Rect(obs["x"], obs["y"], obs["width"], obs["height"]), border_radius=4)
            elif obs_type == "circle":
                pygame.draw.circle(self.screen, color, (int(obs["x"]), int(obs["y"])), int(obs["radius"]))

        # Draw LIDAR scan rays
        for ray in self.lidar.scan_results:
            hx, hy = int(ray["hit_x"]), int(ray["hit_y"])
            # Green ray for distant, red contact dot for close obstacle
            ray_color = Config.COLOR_LIDAR_HIT if ray["distance"] < Config.LIDAR_SAFE_DISTANCE else Config.COLOR_LIDAR_RAY
            pygame.draw.line(self.screen, (30, 80, 70), (int(self.vehicle.x), int(self.vehicle.y)), (hx, hy), 1)
            pygame.draw.circle(self.screen, ray_color, (hx, hy), 3)

        # Draw Pure Pursuit Target Lookahead Crosshair
        if self.target_lookahead_pt:
            tx, ty = int(self.target_lookahead_pt[0]), int(self.target_lookahead_pt[1])
            pygame.draw.circle(self.screen, (255, 0, 128), (tx, ty), 6, 2)
            pygame.draw.line(self.screen, (255, 0, 128), (tx - 8, ty), (tx + 8, ty), 1)
            pygame.draw.line(self.screen, (255, 0, 128), (tx, ty - 8), (tx, ty + 8), 1)

        # Draw Vehicle Body (Oriented Polygon)
        corners = self.vehicle.get_bounding_polygon()
        int_corners = [(int(c[0]), int(c[1])) for c in corners]
        pygame.draw.polygon(self.screen, Config.COLOR_VEHICLE, int_corners)
        pygame.draw.polygon(self.screen, (255, 255, 255), int_corners, 2)

        # Headlights / Heading Vector
        hx = int(self.vehicle.x + math.cos(self.vehicle.heading) * 28)
        hy = int(self.vehicle.y + math.sin(self.vehicle.heading) * 28)
        pygame.draw.line(self.screen, (255, 255, 200), (int(self.vehicle.x), int(self.vehicle.y)), (hx, hy), 3)

    def _render_hud(self):
        hud_rect = pygame.Rect(Config.CANVAS_WIDTH, 0, Config.HUD_WIDTH, Config.WINDOW_HEIGHT)
        pygame.draw.rect(self.screen, Config.COLOR_HUD_BG, hud_rect)
        pygame.draw.line(self.screen, (50, 60, 75), (Config.CANVAS_WIDTH, 0), (Config.CANVAS_WIDTH, Config.WINDOW_HEIGHT), 2)

        x_offset = Config.CANVAS_WIDTH + 15
        y = 15

        # Header
        title_surf = self.font_title.render("AUTONOMOUS VEHICLE HUD", True, (0, 255, 200))
        self.screen.blit(title_surf, (x_offset, y))
        y += 28

        # State banner
        state_colors = {
            "PURSUIT": (46, 204, 113),
            "LOCAL_AVOIDANCE": (243, 156, 18),
            "EMERGENCY_BRAKE": (231, 76, 60),
            "GOAL_REACHED": (241, 196, 15),
            "GLOBAL_PLANNING": (52, 152, 219)
        }
        status_c = state_colors.get(self.current_state, (200, 200, 200))
        pygame.draw.rect(self.screen, status_c, pygame.Rect(x_offset, y, Config.HUD_WIDTH - 30, 26), border_radius=4)
        status_txt = self.font_bold.render(f"STATE: {self.current_state}", True, (10, 10, 10))
        self.screen.blit(status_txt, (x_offset + 10, y + 4))
        y += 38

        # OpenCV Synthetic Camera Viewport
        cam_frame = self.camera.generate_cv_view(width=315, height=140)
        # Convert BGR (OpenCV) to RGB (Pygame)
        cam_frame_rgb = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
        cam_surf = pygame.surfarray.make_surface(np.rot90(cam_frame_rgb))
        cam_surf = pygame.transform.flip(cam_surf, True, False)
        self.screen.blit(cam_surf, (x_offset, y))
        y += 150

        # Telemetry Gauges
        speed_kmh = round(self.vehicle.speed * 18.0, 1) # Approx scale
        steer_deg = round(math.degrees(self.vehicle.steer_angle), 1)
        dist_goal = round(math.hypot(self.env.goal_pos[0] - self.vehicle.x, self.env.goal_pos[1] - self.vehicle.y), 1)

        metrics = [
            ("Speedometer", f"{self.vehicle.speed:.2f} px/f ({speed_kmh} km/h)"),
            ("Steering Angle", f"{steer_deg:+.1f} deg"),
            ("Distance to Goal", f"{dist_goal:.1f} px"),
            ("Scenario Preset", f"{self.env.current_preset}"),
            ("Front LIDAR Clear", f"{self.lidar.sector_distances.get('front', 0.0):.1f} px"),
            ("Left LIDAR Clear", f"{self.lidar.sector_distances.get('left', 0.0):.1f} px"),
            ("Right LIDAR Clear", f"{self.lidar.sector_distances.get('right', 0.0):.1f} px"),
        ]

        for label, val in metrics:
            lbl_surf = self.font_main.render(label, True, Config.COLOR_TEXT_MUTED)
            val_surf = self.font_bold.render(val, True, Config.COLOR_TEXT)
            self.screen.blit(lbl_surf, (x_offset, y))
            self.screen.blit(val_surf, (x_offset + 160, y))
            y += 22

        y += 10
        # Interactive Controls Guide Box
        pygame.draw.rect(self.screen, (25, 32, 44), pygame.Rect(x_offset, y, Config.HUD_WIDTH - 30, 150), border_radius=6)
        y += 8
        c_title = self.font_bold.render("INTERACTIVE CONTROLS", True, (0, 255, 200))
        self.screen.blit(c_title, (x_offset + 10, y))
        y += 24

        controls = [
            ("[Click Map]", "Drop custom obstacle dynamically"),
            ("[1 / 2 / 3]", "Switch map scenario preset"),
            ("[P]", "Re-plan global A* path"),
            ("[R]", "Reset vehicle to origin"),
            ("[Space]", "Pause / Resume simulation"),
            ("[S]", "Save screenshot & telemetry plots"),
        ]
        for key, desc in controls:
            k_surf = self.font_small.render(key, True, (241, 196, 15))
            d_surf = self.font_small.render(desc, True, (200, 200, 200))
            self.screen.blit(k_surf, (x_offset + 10, y))
            self.screen.blit(d_surf, (x_offset + 105, y))
            y += 18
