# 🚗 AI-Based Autonomous Navigation System
### *Production-Grade 2D Autonomous Vehicle & Mobile Robot Simulation Pipeline*

[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Pygame](https://img.shields.io/badge/Simulation-Pygame-yellow.svg)](https://www.pygame.org/)
[![OpenCV](https://img.shields.io/badge/Perception-OpenCV-red.svg)](https://opencv.org/)
[![Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()

---

## 📌 1. Project Overview
The **AI-Based Autonomous Navigation System** is a complete, modular, software-in-the-loop autonomous vehicle guidance stack. Designed to mirror real-world industrial self-driving pipelines (Tesla Autopilot, Waymo, Amazon Robotics AGVs), this system executes full-stack **Perception, Localization, Path Planning, Decision-Making, and Kinematic Control** inside a custom real-time virtual simulation environment.

Without requiring expensive physical robotics hardware, this project delivers end-to-end mathematical rigor—implementing **360° LIDAR raycasting**, **Computer Vision object detection & Time-To-Collision (TTC) estimation**, **8-connected A* global path planning with obstacle inflation**, **Dynamic Window Approach (DWA) local avoidance**, and **Bicycle Kinematics with Pure Pursuit path tracking**.

---

## 🎯 2. Problem Statement & Industry Relevance
### The Problem
Autonomous ground vehicles and warehouse robots face high operational uncertainty: dynamic obstacles (pedestrians, other vehicles), constrained road geometries, sensor noise, and strict latency requirements. A collision in an industrial warehouse or urban intersection carries catastrophic safety and financial costs.

### Industrial Applications
- **Autonomous Passenger Cars:** Urban lane following, intersection navigation, pedestrian emergency braking (e.g., Waymo, Cruise, Baidu Apollo).
- **Warehouse Automated Guided Vehicles (AGVs):** High-density pallet transport in automated fulfillment centers (e.g., Amazon Kiva, Ocado, Geek+).
- **Autonomous Mobile Robots (AMRs):** Factory material handling, hospital sanitization, and last-mile delivery robots (e.g., Starship Technologies, Nuro).
- **Mining & Agricultural Vehicles:** Heavy machinery navigation in unstructured GPS-denied environments (e.g., Caterpillar, John Deere).

---

## 🛠️ 3. Tech Stack & Technologies
| Domain | Technology / Library | Purpose |
| :--- | :--- | :--- |
| **Language** | Python 3.9 - 3.13 | Core algorithmic pipeline and orchestration |
| **Simulation & UI** | Pygame 2.5+ | Real-time 60 FPS physics canvas and interactive HUD |
| **Computer Vision** | OpenCV 4.8+ | Synthetic forward camera view, FOV projection, bounding boxes |
| **Mathematics & Physics** | NumPy, SciPy | Kinematic calculations, raycast geometry, matrix transforms |
| **Global Path Planning** | 8-Connected A* Algorithm | Grid-based optimal path search with Euclidean heuristic |
| **Local Planning** | Dynamic Window Approach / APF | Reactive obstacle avoidance for dynamic pedestrians |
| **Vehicle Control** | Pure Pursuit & PID | Lateral steering tracking & longitudinal speed regulation |
| **Analytics & Logging** | Matplotlib, CSV, JSON | Multi-panel telemetry plotting, clearance analysis |

---

## 🏛️ 4. System Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│                        SIMULATION ENVIRONMENT                          │
│  - Static Obstacles (Buildings, Racks, Cones)                          │
│  - Dynamic Obstacles (Pedestrians, Cross Traffic)                      │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                           PERCEPTION LAYER                             │
│  - 360° Multi-Ray LIDAR Sensor (Raycasting + Gaussian Noise)           │
│  - Forward Camera Module (OpenCV HUD, 70° FOV, Bounding Boxes, TTC)    │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                       PLANNING & DECISION LAYER                        │
│  - Global Planner: A* with Obstacle Inflation & Line-of-Sight Smooth   │
│  - Local Planner: Dynamic Window Approach / Vector Repulsion           │
│  - Finite State Machine: PLANNING -> PURSUIT -> AVOID -> BRAKE         │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                        CONTROL & KINEMATICS LAYER                      │
│  - Pure Pursuit Lateral Controller (Cross-Track Error Minimization)    │
│  - Longitudinal PID Controller (Speed Regulation)                      │
│  - Non-Holonomic Kinematic Bicycle Model (Wheelbase, Heading, Limits)  │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                       TELEMETRY & VISUALIZATION                        │
│  - Interactive Pygame Canvas & Telemetry HUD                           │
│  - Headless Benchmark Engine (CSV logs, Matplotlib Performance Plots)  │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 📁 5. Directory Structure
```text
AI-Autonomous-Navigation-System/
├── config.py                 # Central configuration (kinematics, sensors, HUD)
├── main.py                   # Main CLI entrypoint (GUI, benchmark, vision_eval)
├── requirements.txt          # Python dependencies
├── .gitignore                # Version control ignore rules
├── README.md                 # Primary documentation
├── src/
│   ├── perception/
│   │   ├── lidar_sensor.py       # 360-degree raycasting LIDAR simulation
│   │   └── camera_detector.py    # Forward camera, CV bounding boxes, TTC
│   ├── planning/
│   │   ├── a_star_planner.py     # 8-connected A* search with inflation
│   │   └── dwa_planner.py        # Reactive local obstacle avoidance
│   ├── control/
│   │   ├── vehicle_model.py      # Non-holonomic Kinematic Bicycle Model
│   │   └── controllers.py        # Pure Pursuit tracking & PID speed control
│   ├── simulation/
│   │   ├── environment.py        # Multi-preset world (Urban, Warehouse, Slalom)
│   │   └── simulator_gui.py      # Pygame interactive HUD & dashboard
│   └── utils/
│       └── metrics_logger.py     # Performance analytics & Matplotlib visualizer
├── docs/
│   ├── architecture.md       # In-depth architectural & mathematical formulations
│   └── interview_guide.md    # Top 10 technical interview Q&A with derivations
└── outputs/
    ├── telemetry_analysis.png    # 4-panel analytical performance charts
    ├── vision_detections.png     # Annotated Computer Vision camera frame
    ├── navigation_telemetry.csv  # Frame-by-frame simulation data
    └── summary_metrics.json      # Key performance indicators (KPIs)
```

---

## ⚡ 6. Installation & Quickstart

### Prerequisites
- Python 3.9 or higher installed.

### Step 1: Clone Repository
```bash
git clone https://github.com/your-username/AI-Autonomous-Navigation-System.git
cd AI-Autonomous-Navigation-System
```

### Step 2: Set Up Virtual Environment (Recommended)
**Windows (PowerShell):**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```
**Mac / Linux:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### Step 3: Install Dependencies
```bash
pip install -r requirements.txt
```

---

## 🚀 7. Running the Project

### Option A: Interactive Simulation (Default)
Launch the full interactive 60 FPS graphical HUD:
```bash
python main.py
```
**Interactive Controls:**
- **Mouse Click on Map:** Drop dynamic obstacles in real time to trigger live re-planning!
- **Keys `[1]`, `[2]`, `[3]`:** Switch between presets (`1: Urban Streets`, `2: Warehouse`, `3: Obstacle Slalom`).
- **Key `[P]`:** Force re-plan global A* path.
- **Key `[R]`:** Reset vehicle to start location.
- **Key `[Space]`:** Pause / Resume simulation.
- **Key `[S]`:** Save screenshot and export telemetry analysis.

### Option B: Automated Headless Benchmark Mode
Run a 600-step autonomous test scenario without GUI display, logging metrics and exporting charts:
```bash
python main.py --mode benchmark
```

### Option C: Computer Vision Perception Evaluation
Test the forward camera detector and save an annotated CV frame with bounding boxes and perspective lanes:
```bash
python main.py --mode vision_eval
```

---

## 📊 8. Simulation Results & Analytics
Running the benchmark mode automatically exports:
1. **`outputs/telemetry_analysis.png`**:
   - **Velocity Profile:** Longitudinal acceleration and cruising velocity regulation.
   - **Steering Dynamics:** Continuous front-wheel angle adjustments within kinematic limits ($\pm 35^\circ$).
   - **LIDAR Obstacle Clearance:** Proximity monitoring ensuring safety margins $> 24\text{ px}$.
   - **Trajectory Map:** Visual overlay comparing planned global A* path vs actual driven path.
2. **`outputs/vision_detections.png`**: Synthetic camera viewport showcasing 2D bounding boxes and Time-To-Collision (TTC) alerts.
3. **`outputs/summary_metrics.json`**:
   ```json
   {
       "total_time_seconds": 10.0,
       "total_distance_px": 688.7,
       "average_speed": 1.15,
       "max_speed": 5.5,
       "minimum_obstacle_clearance": 14.36,
       "near_collision_events": 6,
       "success": true
   }
   ```

---

## 🔮 9. Future Improvements
- [ ] **ROS 2 Integration:** Connect navigation stack to ROS 2 Humble nodes via `rclpy`.
- [ ] **CARLA 3D Engine:** Port vehicle kinematics and camera sensors into the photorealistic CARLA simulator.
- [ ] **SLAM (Simultaneous Localization & Mapping):** Replace pre-built grid with real-time Occupancy Grid Mapping.
- [ ] **Reinforcement Learning:** Train a PPO / SAC policy for end-to-end local obstacle avoidance.

---

## 🎓 10. Learning Outcomes
- Developed a comprehensive understanding of **non-holonomic vehicle kinematics** using the Bicycle Model.
- Implemented industry-standard **A* global path planning** with safety margin inflation and line-of-sight smoothing.
- Designed **multi-sensor perception pipelines** combining 360° LIDAR raycasting and OpenCV camera perspective projection.
- Built closed-loop control systems leveraging **Pure Pursuit geometric tracking** and **longitudinal PID regulation**.

---

## 👤 Author & Acknowledgments
- **Developer:** Kishor Kumar L
- **Domain:** Artificial Intelligence, Autonomous Systems & Robotics
- **Portfolio:** [GitHub Profile](https://github.com/your-username)
