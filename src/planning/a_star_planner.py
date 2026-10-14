"""
=============================================================================
Module: planning.a_star_planner
Description: Production-ready 8-connected Grid A* Global Path Planner with
             obstacle inflation and line-of-sight path smoothing.
=============================================================================
"""

import math
import heapq
import numpy as np
from config import Config

class Node:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

class AStarPlanner:
    def __init__(self, grid_size=Config.GRID_RESOLUTION, robot_radius=Config.OBSTACLE_SAFETY_MARGIN):
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        self.motion = [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2))
        ]

    def plan(self, start_pos, goal_pos, obstacles, canvas_width, canvas_height):
        """
        Executes A* search algorithm from start to goal position.
        
        Args:
            start_pos (tuple): (x, y) starting coordinate in pixels.
            goal_pos (tuple): (x, y) target coordinate in pixels.
            obstacles (list): List of obstacle dicts.
            canvas_width (int): Canvas boundary width.
            canvas_height (int): Canvas boundary height.
            
        Returns:
            list: List of smoothed waypoint tuples [(x1, y1), (x2, y2), ...]
        """
        start_gx = int(start_pos[0] // self.grid_size)
        start_gy = int(start_pos[1] // self.grid_size)
        goal_gx = int(goal_pos[0] // self.grid_size)
        goal_gy = int(goal_pos[1] // self.grid_size)

        max_gx = int(canvas_width // self.grid_size)
        max_gy = int(canvas_height // self.grid_size)

        # Build occupancy grid with safety inflation
        occupancy = self._build_occupancy_grid(obstacles, max_gx, max_gy)

        # Ensure goal and start are within bounds
        goal_gx = max(0, min(max_gx - 1, goal_gx))
        goal_gy = max(0, min(max_gy - 1, goal_gy))
        start_gx = max(0, min(max_gx - 1, start_gx))
        start_gy = max(0, min(max_gy - 1, start_gy))

        open_set = []
        start_node = Node(start_gx, start_gy, 0.0)
        h_start = self._heuristic(start_gx, start_gy, goal_gx, goal_gy)
        heapq.heappush(open_set, (h_start, start_node))

        closed_set = set()
        best_g = {(start_gx, start_gy): 0.0}

        goal_node = None

        while open_set:
            _, current = heapq.heappop(open_set)

            if (current.x, current.y) in closed_set:
                continue
            closed_set.add((current.x, current.y))

            # Goal check (reached cell or adjacent to goal)
            if current.x == goal_gx and current.y == goal_gy:
                goal_node = current
                break

            for dx, dy, move_cost in self.motion:
                nx, ny = current.x + dx, current.y + dy

                if not (0 <= nx < max_gx and 0 <= ny < max_gy):
                    continue
                if occupancy[ny, nx]:  # Collides with obstacle margin
                    continue

                new_g = best_g[(current.x, current.y)] + move_cost
                if (nx, ny) not in best_g or new_g < best_g[(nx, ny)]:
                    best_g[(nx, ny)] = new_g
                    h = self._heuristic(nx, ny, goal_gx, goal_gy)
                    neighbor_node = Node(nx, ny, new_g, current)
                    heapq.heappush(open_set, (new_g + h, neighbor_node))

        if goal_node is None:
            # Fallback direct path if no obstacle-free path could be found
            return [start_pos, goal_pos]

        # Reconstruct path backwards
        raw_path = []
        curr = goal_node
        while curr:
            px = (curr.x + 0.5) * self.grid_size
            py = (curr.y + 0.5) * self.grid_size
            raw_path.append((px, py))
            curr = curr.parent

        raw_path.reverse()
        # Smooth path using line-of-sight algorithm
        smoothed_path = self._smooth_path(raw_path, occupancy, max_gx, max_gy)
        return smoothed_path

    def _heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic with slight octile weighting."""
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        return math.hypot(dx, dy)

    def _build_occupancy_grid(self, obstacles, max_gx, max_gy):
        """Constructs boolean 2D grid where True represents obstacle or safety margin."""
        grid = np.zeros((max_gy, max_gx), dtype=bool)

        for obs in obstacles:
            obs_type = obs.get("type", "rect")
            margin = self.robot_radius

            if obs_type == "rect":
                rx = obs["x"] - margin
                ry = obs["y"] - margin
                rw = obs["width"] + 2 * margin
                rh = obs["height"] + 2 * margin

                min_x = max(0, int(rx // self.grid_size))
                max_x = min(max_gx, int((rx + rw) // self.grid_size) + 1)
                min_y = max(0, int(ry // self.grid_size))
                max_y = min(max_gy, int((ry + rh) // self.grid_size) + 1)

                grid[min_y:max_y, min_x:max_x] = True

            elif obs_type == "circle":
                cx, cy, r = obs["x"], obs["y"], obs["radius"] + margin
                min_x = max(0, int((cx - r) // self.grid_size))
                max_x = min(max_gx, int((cx + r) // self.grid_size) + 1)
                min_y = max(0, int((cy - r) // self.grid_size))
                max_y = min(max_gy, int((cy + r) // self.grid_size) + 1)

                for gy in range(min_y, max_y):
                    for gx in range(min_x, max_x):
                        px = (gx + 0.5) * self.grid_size
                        py = (gy + 0.5) * self.grid_size
                        if math.hypot(px - cx, py - cy) <= r:
                            grid[gy, gx] = True

        return grid

    def _smooth_path(self, path, occupancy, max_gx, max_gy):
        """Removes redundant intermediate waypoints if direct line of sight exists."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            best_next = current_idx + 1
            for candidate_idx in range(len(path) - 1, current_idx + 1, -1):
                if self._line_of_sight(path[current_idx], path[candidate_idx], occupancy, max_gx, max_gy):
                    best_next = candidate_idx
                    break
            smoothed.append(path[best_next])
            current_idx = best_next

        return smoothed

    def _line_of_sight(self, p1, p2, occupancy, max_gx, max_gy):
        """Bresenham-like ray check between two pixel positions."""
        steps = int(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) / (self.grid_size / 2)) + 1
        for i in range(steps + 1):
            t = i / max(1, steps)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])

            gx = int(x // self.grid_size)
            gy = int(y // self.grid_size)

            if 0 <= gx < max_gx and 0 <= gy < max_gy:
                if occupancy[gy, gx]:
                    return False
            else:
                return False
        return True
