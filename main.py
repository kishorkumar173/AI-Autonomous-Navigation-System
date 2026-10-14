"""
=============================================================================
Project: AI-Based Autonomous Navigation System
File: main.py
Description: Main orchestration entry point. Supports:
             1. Interactive Pygame GUI simulation mode
             2. Automated headless benchmark mode (exports telemetry & plots)
             3. Computer Vision perception evaluation mode
=============================================================================
"""

import sys
import os
import argparse
import time
import math
import cv2
import numpy as np

# Ensure local imports work reliably
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import Config
from src.control.vehicle_model import VehicleModel
from src.control.controllers import PurePursuitController, PIDController
from src.perception.lidar_sensor import LidarSensor
from src.perception.camera_detector import CameraDetector
from src.planning.a_star_planner import AStarPlanner
from src.planning.dwa_planner import DWAPlanner
from src.simulation.environment import SimulationEnvironment
from src.utils.metrics_logger import MetricsLogger

def run_gui():
    """Runs interactive graphical simulation with Pygame."""
    from src.simulation.simulator_gui import SimulatorGUI

    env = SimulationEnvironment()
    vehicle = VehicleModel(x=env.start_pos[0], y=env.start_pos[1], heading=env.start_heading)
    lidar = LidarSensor()
    camera = CameraDetector()
    planner = AStarPlanner()
    controller = PurePursuitController()
    dwa = DWAPlanner()
    logger = MetricsLogger()

    gui = SimulatorGUI(vehicle, env, lidar, camera, planner, controller, dwa, logger)

    print("\n" + "="*70)
    print("[SYSTEM] AI-BASED AUTONOMOUS NAVIGATION SYSTEM: SIMULATION ACTIVE")
    print("="*70)
    print("Controls:")
    print(" - [CLICK MAP]: Place dynamic obstacles in real time")
    print(" - [1 / 2 / 3]: Switch scenario (1: Urban, 2: Warehouse, 3: Slalom)")
    print(" - [P]: Force re-plan global path")
    print(" - [R]: Reset vehicle to origin")
    print(" - [SPACE]: Pause / Resume simulation")
    print(" - [S]: Export telemetry analysis charts and screenshots")
    print("="*70 + "\n")

    running = True
    while running:
        running = gui.handle_events()
        gui.step()
        gui.render()

    gui.logger.export_summary(gui.global_path, env.get_all_obstacles())
    print("\n[SUCCESS] Simulation session ended. Summary saved to /outputs directory.\n")

def run_benchmark(steps=600):
    """Executes headless simulation benchmark and generates telemetry artifacts."""
    print(f"\n[BENCHMARK] Initializing headless evaluation for {steps} steps...")
    env = SimulationEnvironment()
    vehicle = VehicleModel(x=env.start_pos[0], y=env.start_pos[1], heading=env.start_heading)
    lidar = LidarSensor()
    camera = CameraDetector()
    planner = AStarPlanner()
    controller = PurePursuitController()
    dwa = DWAPlanner()
    logger = MetricsLogger()

    # Plan initial path
    global_path = planner.plan(
        (vehicle.x, vehicle.y), env.goal_pos,
        env.get_all_obstacles(), Config.CANVAS_WIDTH, Config.CANVAS_HEIGHT
    )

    print(f"[A* PLANNER] Generated global trajectory with {len(global_path)} waypoints.")

    for step in range(steps):
        env.update_dynamic_obstacles()

        lidar_data = lidar.scan(
            vehicle.x, vehicle.y, vehicle.heading,
            env.get_all_obstacles(), Config.CANVAS_WIDTH, Config.CANVAS_HEIGHT
        )

        cv_detections = camera.detect(
            vehicle.x, vehicle.y, vehicle.heading,
            vehicle.speed, env.get_all_obstacles()
        )

        steer_target, target_pt, is_goal_reached = controller.compute_steering(
            vehicle.x, vehicle.y, vehicle.heading, global_path
        )

        steer_nudge, speed_scale, is_avoiding = dwa.plan_reactive_avoidance(
            vehicle.x, vehicle.y, vehicle.heading,
            vehicle.speed, target_pt, lidar_data
        )

        dist_to_goal = math.hypot(env.goal_pos[0] - vehicle.x, env.goal_pos[1] - vehicle.y)
        min_front_lidar = lidar.sector_distances.get("front", Config.LIDAR_MAX_RANGE)

        if is_goal_reached or dist_to_goal < Config.GOAL_TOLERANCE:
            state = "GOAL_REACHED"
            target_accel = -Config.MAX_BRAKE
            final_steer = 0.0
        elif min_front_lidar < Config.LIDAR_SAFE_DISTANCE * 0.7:
            state = "EMERGENCY_BRAKE"
            target_accel = -Config.MAX_BRAKE
            final_steer = steer_target + steer_nudge * 1.5
        elif is_avoiding:
            state = "LOCAL_AVOIDANCE"
            final_steer = steer_target * 0.3 + steer_nudge * 0.7
            target_speed = Config.MAX_SPEED * speed_scale
            target_accel = 0.08 if vehicle.speed < target_speed else -0.15
        else:
            state = "PURSUIT"
            final_steer = steer_target
            target_speed = Config.MAX_SPEED
            target_accel = 0.12 if vehicle.speed < target_speed else -0.05

        vehicle.update(final_steer, target_accel)

        logger.log_frame(
            vehicle.speed, vehicle.steer_angle,
            min(lidar.sector_distances.values()),
            dist_to_goal, vehicle.x, vehicle.y, state
        )

        if state == "GOAL_REACHED" and vehicle.speed < 0.1:
            print(f"[BENCHMARK] Target goal reached successfully at step {step}!")
            break

    summary = logger.export_summary(global_path, env.get_all_obstacles())
    print("\n" + "="*50)
    print("[METRICS] BENCHMARK EXECUTION SUMMARY")
    print("="*50)
    for k, v in summary.items():
        print(f"  {k:30s}: {v}")
    print("="*50)
    print(f"[ARTIFACTS] Analytical plots written to: {Config.OUTPUTS_DIR}/telemetry_analysis.png")
    print(f"[ARTIFACTS] Telemetry log saved to:    {Config.OUTPUTS_DIR}/navigation_telemetry.csv")
    print(f"[ARTIFACTS] JSON metrics saved to:      {Config.OUTPUTS_DIR}/summary_metrics.json\n")

def run_vision_eval():
    """Generates synthetic computer vision test frame with detected bounding boxes."""
    print("\n[VISION EVAL] Evaluating computer vision object detection and lane perception...")
    camera = CameraDetector()
    obstacles = [
        {"id": "ped_1", "category": "pedestrian", "x": 120, "y": 80, "width": 20, "height": 40},
        {"id": "veh_1", "category": "vehicle", "x": 100, "y": 140, "width": 45, "height": 30},
        {"id": "cone_1", "category": "traffic_cone", "x": 80, "y": 110, "radius": 15}
    ]
    camera.detect(vehicle_x=50, vehicle_y=50, vehicle_heading=0.7, vehicle_speed=3.5, obstacles=obstacles)
    frame = camera.generate_cv_view(width=640, height=360)

    out_path = os.path.join(Config.OUTPUTS_DIR, "vision_detections.png")
    os.makedirs(Config.OUTPUTS_DIR, exist_ok=True)
    cv2.imwrite(out_path, frame)
    print(f"[SUCCESS] Annotated Computer Vision test frame exported to {out_path}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI-Based Autonomous Navigation System")
    parser.add_argument("--mode", choices=["gui", "benchmark", "vision_eval"], default="gui",
                        help="Execution mode: 'gui' (interactive simulation), 'benchmark' (headless telemetry), 'vision_eval' (CV test)")
    parser.add_argument("--steps", type=int, default=600, help="Number of steps for benchmark mode")
    args = parser.parse_args()

    if args.mode == "gui":
        run_gui()
    elif args.mode == "benchmark":
        run_benchmark(steps=args.steps)
    elif args.mode == "vision_eval":
        run_vision_eval()
