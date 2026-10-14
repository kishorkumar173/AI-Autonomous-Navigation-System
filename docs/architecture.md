# System Architecture: AI-Based Autonomous Navigation System

## 1. High-Level System Architecture Diagram

```
+-----------------------------------------------------------------------------------+
|                           VIRTUAL SIMULATION WORLD                                |
|  - 2D Kinematic Grid Environment (Roads, Intersections, Aisles)                   |
|  - Static Obstacles (Buildings, Racks, Safety Cones)                              |
|  - Dynamic Obstacles (Pedestrians, Crossing Vehicles)                            |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                               PERCEPTION LAYER                                    |
|                                                                                   |
|   [ 360° LIDAR Sensor ]                     [ Forward Camera (CV) ]               |
|   - Multi-ray raycasting                    - Conical Field of View (FOV)         |
|   - 10° angular resolution                  - Perspective projection              |
|   - Range & bearing estimation              - Bounding-box detection              |
|   - Sector clearance analysis               - Time-To-Collision (TTC)             |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                        PLANNING & DECISION MAKING LAYER                           |
|                                                                                   |
|   [ Global Planner: 8-Connected A* ]       [ Local Avoidance: DWA / Repulsion ]   |
|   - Obstacle margin inflation              - Repulsive potential field vectors    |
|   - Euclidean distance heuristic           - Dynamic speed & steering throttling  |
|   - Line-of-sight path smoothing           - Emergency collision mitigation       |
|                                                                                   |
|                                [ Finite State Machine ]                           |
|      GLOBAL_PLANNING ──► PURSUIT ──► LOCAL_AVOIDANCE ──► GOAL_REACHED             |
|                                  └──► EMERGENCY_BRAKE                             |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                            CONTROL & ACTUATION LAYER                              |
|                                                                                   |
|   [ Pure Pursuit Geometric Controller ]     [ Longitudinal PID Controller ]       |
|   - Lookahead point search along path       - Speed error tracking                |
|   - Steering angle calculation              - Anti-windup clamping                |
|                                                                                   |
|                       [ Kinematic Bicycle Vehicle Model ]                         |
|   dx/dt = v * cos(θ)    |   dy/dt = v * sin(θ)   |   dθ/dt = (v / L) * tan(δ)     |
+-----------------------------------------------------------------------------------+
                                         │
                                         ▼
+-----------------------------------------------------------------------------------+
|                         TELEMETRY & VISUALIZATION LAYER                           |
|  - Pygame Real-Time HUD Display (Speedometer, Steering Angle, Status Banners)     |
|  - Mini OpenCV Camera Viewport (Horizon, Lane Lines, Bounding Boxes)              |
|  - Performance Analytics Engine (Matplotlib plots, CSV logs, JSON KPIs)           |
+-----------------------------------------------------------------------------------+
```

## 2. Mathematical Formulations

### 2.1 Kinematic Bicycle Model
The autonomous agent is modeled using the non-holonomic front-wheel steering bicycle model:
$$\dot{x} = v \cos(\theta)$$
$$\dot{y} = v \sin(\theta)$$
$$\dot{\theta} = \frac{v}{L} \tan(\delta)$$
Where:
- $(x, y)$: Vehicle coordinate center
- $\theta$: Vehicle yaw / heading angle
- $v$: Longitudinal velocity
- $L$: Wheelbase length (distance between axles)
- $\delta$: Front wheel steering angle

### 2.2 Pure Pursuit Path Tracking
The Pure Pursuit algorithm geometrically calculates the curvature required to steer towards a lookahead point $(g_x, g_y)$ located distance $L_d$ ahead on the global path:
$$\alpha = \text{atan2}(g_y - y, g_x - x) - \theta$$
$$\delta = \arctan\left(\frac{2 L \sin(\alpha)}{L_d}\right)$$

### 2.3 A* Heuristic
$$f(n) = g(n) + h(n)$$
Where $g(n)$ is the exact cost from the start node to node $n$, and $h(n)$ is the Euclidean distance heuristic:
$$h(n) = \sqrt{(n_x - \text{goal}_x)^2 + (n_y - \text{goal}_y)^2}$$
