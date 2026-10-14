"""
=============================================================================
Module: utils.metrics_logger
Description: Performance metrics recorder and automated visual reporting engine.
             Generates analytical plots for speed, steering, clearances, and trajectory.
=============================================================================
"""

import os
import json
import csv
import matplotlib
matplotlib.use('Agg') # Headless backend for reliable plot generation
import matplotlib.pyplot as plt
import numpy as np
from config import Config

class MetricsLogger:
    def __init__(self, output_dir=Config.OUTPUTS_DIR):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.reset()

    def reset(self):
        self.time_steps = []
        self.speeds = []
        self.steering_angles = []
        self.min_obstacle_distances = []
        self.distances_to_goal = []
        self.trajectory_x = []
        self.trajectory_y = []
        self.states = []
        self.collisions = 0
        self.step_count = 0

    def log_frame(self, speed, steer, min_obs_dist, dist_to_goal, x, y, state):
        self.step_count += 1
        t = self.step_count / Config.FPS
        self.time_steps.append(t)
        self.speeds.append(speed)
        self.steering_angles.append(np.degrees(steer))
        self.min_obstacle_distances.append(min_obs_dist)
        self.distances_to_goal.append(dist_to_goal)
        self.trajectory_x.append(x)
        self.trajectory_y.append(y)
        self.states.append(state)

        if min_obs_dist < 15.0:
            self.collisions += 1

    def export_summary(self, planned_path=None, obstacles=None):
        """Exports analytical plots, CSV logs, and JSON summary statistics."""
        if not self.time_steps:
            return None

        total_time = self.time_steps[-1]
        avg_speed = float(np.mean(self.speeds)) if self.speeds else 0.0
        max_speed = float(np.max(self.speeds)) if self.speeds else 0.0
        min_clearance = float(np.min(self.min_obstacle_distances)) if self.min_obstacle_distances else 0.0

        # Compute approximate path length
        total_dist = 0.0
        for i in range(1, len(self.trajectory_x)):
            dx = self.trajectory_x[i] - self.trajectory_x[i-1]
            dy = self.trajectory_y[i] - self.trajectory_y[i-1]
            total_dist += np.hypot(dx, dy)

        summary = {
            "total_time_seconds": round(total_time, 2),
            "total_distance_px": round(total_dist, 2),
            "average_speed": round(avg_speed, 2),
            "max_speed": round(max_speed, 2),
            "minimum_obstacle_clearance": round(min_clearance, 2),
            "near_collision_events": self.collisions,
            "final_distance_to_goal": round(self.distances_to_goal[-1], 2) if self.distances_to_goal else 0.0,
            "success": bool(self.distances_to_goal and self.distances_to_goal[-1] < Config.GOAL_TOLERANCE * 1.5)
        }

        # Save JSON
        json_path = os.path.join(self.output_dir, "summary_metrics.json")
        with open(json_path, "w") as f:
            json.dump(summary, f, indent=4)

        # Save CSV
        csv_path = os.path.join(self.output_dir, "navigation_telemetry.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "speed", "steer_deg", "min_clearance", "dist_goal", "x", "y", "state"])
            for i in range(len(self.time_steps)):
                writer.writerow([
                    round(self.time_steps[i], 3),
                    round(self.speeds[i], 3),
                    round(self.steering_angles[i], 2),
                    round(self.min_obstacle_distances[i], 2),
                    round(self.distances_to_goal[i], 2),
                    round(self.trajectory_x[i], 1),
                    round(self.trajectory_y[i], 1),
                    self.states[i]
                ])

        # Generate High-Resolution Matplotlib Performance Graph
        self._generate_plots(planned_path, obstacles)
        return summary

    def _generate_plots(self, planned_path, obstacles):
        plt.style.use('dark_background')
        fig, axs = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("AI Autonomous Navigation System - Performance Analysis", fontsize=16, fontweight='bold', color="#00ffc8")

        # 1. Speed Profile
        axs[0, 0].plot(self.time_steps, self.speeds, color="#2ecc71", linewidth=2.0, label="Velocity (px/s)")
        axs[0, 0].axhline(y=Config.MAX_SPEED, color="#e74c3c", linestyle="--", alpha=0.7, label="Speed Limit")
        axs[0, 0].set_title("Longitudinal Velocity Profile", fontsize=12, color="#ffffff")
        axs[0, 0].set_xlabel("Time (seconds)")
        axs[0, 0].set_ylabel("Speed (pixels/frame)")
        axs[0, 0].grid(True, alpha=0.25)
        axs[0, 0].legend()

        # 2. Steering Angle Dynamics
        axs[0, 1].plot(self.time_steps, self.steering_angles, color="#f1c40f", linewidth=1.8, label="Steering (°)")
        axs[0, 1].axhline(y=np.degrees(Config.MAX_STEER_ANGLE), color="#e74c3c", linestyle=":", alpha=0.5)
        axs[0, 1].axhline(y=-np.degrees(Config.MAX_STEER_ANGLE), color="#e74c3c", linestyle=":", alpha=0.5)
        axs[0, 1].set_title("Steering Angle Response (°)", fontsize=12, color="#ffffff")
        axs[0, 1].set_xlabel("Time (seconds)")
        axs[0, 1].set_ylabel("Degrees")
        axs[0, 1].grid(True, alpha=0.25)
        axs[0, 1].legend()

        # 3. Obstacle Clearance & Safety Margin
        axs[1, 0].plot(self.time_steps, self.min_obstacle_distances, color="#00d2d3", linewidth=2.0, label="Min Obstacle Dist")
        axs[1, 0].axhline(y=Config.LIDAR_SAFE_DISTANCE, color="#e74c3c", linestyle="--", label="Critical Safety Threshold")
        axs[1, 0].set_title("LIDAR Obstacle Clearance Distance", fontsize=12, color="#ffffff")
        axs[1, 0].set_xlabel("Time (seconds)")
        axs[1, 0].set_ylabel("Clearance (pixels)")
        axs[1, 0].grid(True, alpha=0.25)
        axs[1, 0].legend()

        # 4. Trajectory Tracking (Planned vs Actual)
        if planned_path:
            px, py = zip(*planned_path)
            axs[1, 1].plot(px, py, 'b--', linewidth=2.0, label="Global A* Plan", alpha=0.8)
        axs[1, 1].plot(self.trajectory_x, self.trajectory_y, color="#2ecc71", linewidth=2.5, label="Driven Path")

        # Plot obstacles on the 2D map
        if obstacles:
            for obs in obstacles:
                if obs.get("type") == "rect":
                    r = plt.Rectangle((obs["x"], obs["y"]), obs["width"], obs["height"], color="#e74c3c", alpha=0.5)
                    axs[1, 1].add_patch(r)
                elif obs.get("type") == "circle":
                    c = plt.Circle((obs["x"], obs["y"]), obs["radius"], color="#e67e22", alpha=0.6)
                    axs[1, 1].add_patch(c)

        axs[1, 1].set_title("2D Trajectory & Obstacle Map", fontsize=12, color="#ffffff")
        axs[1, 1].set_xlabel("X (pixels)")
        axs[1, 1].set_ylabel("Y (pixels)")
        axs[1, 1].set_xlim(0, Config.CANVAS_WIDTH)
        axs[1, 1].set_ylim(Config.CANVAS_HEIGHT, 0) # Invert Y for screen coordinates
        axs[1, 1].grid(True, alpha=0.25)
        axs[1, 1].legend()

        plt.tight_layout()
        plot_path = os.path.join(self.output_dir, "telemetry_analysis.png")
        plt.savefig(plot_path, dpi=200)
        plt.close()
