"""
=============================================================================
Module: perception.camera_detector
Description: Computer Vision perception module simulating a forward-facing camera,
             object classification, 2D bounding-box generation, and Time-To-Collision
             (TTC) calculation with OpenCV rendering.
=============================================================================
"""

import math
import cv2
import numpy as np
from config import Config

class CameraDetector:
    def __init__(self, fov=Config.CAMERA_FOV, view_range=Config.CAMERA_RANGE):
        self.fov = fov
        self.view_range = view_range
        self.detected_objects = []

    def detect(self, vehicle_x, vehicle_y, vehicle_heading, vehicle_speed, obstacles):
        """
        Scans forward conical FOV to detect obstacles, classify them,
        and calculate spatial metrics (distance, bearing, TTC).
        """
        self.detected_objects = []

        for obs in obstacles:
            ox = obs.get("x", 0) + (obs.get("width", 0) / 2 if "width" in obs else 0)
            oy = obs.get("y", 0) + (obs.get("height", 0) / 2 if "height" in obs else 0)

            dx = ox - vehicle_x
            dy = oy - vehicle_y
            distance = math.hypot(dx, dy)

            if distance > self.view_range:
                continue

            # Compute angle relative to vehicle heading
            angle_to_obj = math.atan2(dy, dx)
            relative_angle = (angle_to_obj - vehicle_heading + math.pi) % (2 * math.pi) - math.pi

            # Check if within forward FOV
            if abs(relative_angle) <= (self.fov / 2):
                # Calculate Time To Collision (TTC)
                closing_speed = vehicle_speed * math.cos(relative_angle)
                ttc = (distance / closing_speed) if closing_speed > 0.05 else 99.9

                # Determine classification
                obs_type = obs.get("category", "obstacle")
                color = (0, 0, 255) # Default red
                if obs_type == "pedestrian":
                    color = (255, 105, 180)
                elif obs_type == "vehicle":
                    color = (255, 140, 0)
                elif obs_type == "traffic_cone":
                    color = (0, 165, 255)

                confidence = round(float(np.clip(1.0 - (distance / (self.view_range * 1.2)) + np.random.normal(0, 0.03), 0.70, 0.99)), 2)

                self.detected_objects.append({
                    "id": obs.get("id", "obs_0"),
                    "category": obs_type,
                    "distance": round(distance, 1),
                    "relative_angle": round(math.degrees(relative_angle), 1),
                    "ttc": round(ttc, 2),
                    "confidence": confidence,
                    "color": color,
                    "pos": (ox, oy)
                })

        # Sort by closest distance first
        self.detected_objects.sort(key=lambda x: x["distance"])
        return self.detected_objects

    def generate_cv_view(self, width=320, height=180):
        """
        Generates a synthetic Computer Vision camera HUD frame using OpenCV,
        complete with artificial horizon, perspective lane guides, and bounding boxes.
        """
        # Create dark road view frame
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:] = (30, 30, 35)

        # Draw simulated sky and road horizon
        cv2.rectangle(frame, (0, 0), (width, height // 2), (50, 45, 40), -1) # Sky/warehouse wall
        cv2.line(frame, (0, height // 2), (width, height // 2), (70, 70, 75), 1)

        # Draw simulated perspective lane guides
        pt_vanishing = (width // 2, height // 2)
        cv2.line(frame, (20, height), pt_vanishing, (0, 255, 255), 2)  # Left lane
        cv2.line(frame, (width - 20, height), pt_vanishing, (0, 255, 255), 2) # Right lane
        cv2.line(frame, (width // 2, height), pt_vanishing, (200, 200, 200), 1) # Center dash

        # Render detected targets as 2D bounding boxes in camera frame
        for i, obj in enumerate(self.detected_objects[:4]):  # Max top 4 closest
            dist = max(10.0, obj["distance"])
            rel_angle = obj["relative_angle"]

            # Perspective scale based on distance
            scale = max(0.2, min(1.5, 70.0 / dist))
            box_w = int(45 * scale)
            box_h = int(60 * scale)

            # Horizontal placement mapped across FOV (-35 to +35 deg)
            center_x = int((width // 2) + (rel_angle / 35.0) * (width // 2 * 0.8))
            center_y = int((height // 2) + (height // 2) * (1.0 - (dist / self.view_range)))

            x1 = max(0, center_x - box_w // 2)
            y1 = max(0, center_y - box_h // 2)
            x2 = min(width - 1, center_x + box_w // 2)
            y2 = min(height - 1, center_y + box_h // 2)

            box_color = (0, 255, 120) if obj["ttc"] > 2.5 else (0, 70, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

            # Label overlay
            label = f"{obj['category'][:4].upper()}: {obj['distance']}m"
            cv2.putText(frame, label, (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            ttc_label = f"TTC:{obj['ttc']}s"
            cv2.putText(frame, ttc_label, (x1, min(height - 4, y2 + 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, box_color, 1)

        # Status text
        cv2.putText(frame, f"CAM FOV: {int(math.degrees(self.fov))}deg | DET: {len(self.detected_objects)}", 
                    (8, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1)

        return frame
