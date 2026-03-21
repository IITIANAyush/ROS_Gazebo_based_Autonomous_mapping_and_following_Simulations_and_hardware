# Dual-Planner Autonomous Navigation System (ROS2)

## A Structured Implementation of A* and Medial-Axis (GVD) Planning in Simulation

---

## 1. Overview

This project presents a structured development of a complete autonomous navigation stack in ROS2, built in progressive stages:

1. Simulation Environment & Mapping (Digital Twin Creation)
2. Configuration Space Modeling
3. Dual Planner Implementation (A* and GVD)
4. Planner-Specific Path Following

The system is designed to investigate the trade-off between:

- **Optimality (Shortest Path Planning)**
- **Safety (Maximum Clearance Planning)**

The implementation is fully modular and runs in ROS2 Humble using Gazebo and TurtleBot3 simulation.

---

## 2. Phase I — Environment & Digital Twin Construction

### 2.1 Simulation Environment Setup

A custom world ("Gauntlet") was integrated into a ROS2 workspace. The objective was to create a reproducible digital twin of the operating environment.

Key tasks completed:

- Custom world integration in Gazebo
- ROS2 workspace configuration
- Proper frame tree validation in RViz
- Verification of `/odom`, `/scan`, and `/tf` pipelines

---

### 2.2 SLAM & Kartographer-Based Map Generation

To solve the **Blind Robot Problem**, a complete occupancy grid map was generated using online SLAM & Kartographer.

Process:

1. Teleoperation of TurtleBot3 in simulation
2. Real-time mapping via SLAM toolbox
3. Loop-closure validation
4. Map export to:

gauntlet_map.pgm
gauntlet_map.yaml


The final map was validated for:
- Closed loops
- No ghost obstacles
- Consistent resolution
- Correct origin alignment

---

### 2.3 Configuration Space Modeling

The robot is not a point mass; therefore, configuration space (C-Space) modeling was applied.

Steps performed:

- Manual measurement of robot radius `r`
- Application of Minkowski sum concept
- Obstacle inflation by `(r + δ)` safety margin
- Visual overlay comparison of:
  - Raw occupancy grid
  - Inflated obstacle map

This allowed planners to treat the robot as a point in C-space while preserving collision safety.

---

## 3. Phase II — Dual Planner Implementation

### 3.1 Grid-to-Graph Conversion

The occupancy grid published on `/map` was converted into a graph representation:

- 2D matrix extraction from `OccupancyGrid`
- 8-connected node expansion
- Free-space thresholding
- Boundary checking
- Index-to-world coordinate transformations

This enabled direct application of graph search algorithms.

---

## 3.2 A* Planner (Optimal Path)

A complete A* implementation was developed from scratch.

Key technical elements:

- Priority queue via `heapq`
- Explicit `g(n)` and `f(n)` cost tracking
- Euclidean heuristic for admissibility
- Parent tracking for path reconstruction
- Diagonal movement with √2 cost
- Efficient neighbor expansion

Properties:

- Strict shortest path guarantee
- Efficient search convergence
- Aggressive corner cutting near obstacles
- Minimal travel distance

---

## 3.3 GVD Planner (Medial Axis Planning)

A safety-oriented planner was implemented using a medial-axis approach.

Concept:

- Extract skeleton of free space
- Maximize clearance from obstacles
- Follow Voronoi-like centerlines

Characteristics:

- Longer path length
- Higher minimum obstacle distance
- Reduced risk in narrow corridors
- Centered traversal through constrained regions

This planner intentionally prioritizes safety over path optimality.

---

## 4. Planner-Specific Controllers

Distinct controllers were developed to match planner characteristics.

### 4.1 A* Controller

- Lookahead-based tracking
- Continuous heading correction
- Smooth motion behavior
- Efficient trajectory tracking

### 4.2 GVD Controller

- Sequential waypoint tracking
- Rotate-then-drive strategy
- Guaranteed traversal of medial axis
- Robust against high-curvature segments

Controller-planner coupling was critical to ensure faithful path execution.

---

## 5. System Architecture

nav2_map_server → planner_server → /planned_path → path_follower → /cmd_vel → Gazebo


Core Nodes:

- `planner_server.py`
- `path_follower.py`
- `path_follower_gvd.py`
- `gauntlet_nav.launch.py`

Planner selection is handled at launch via parameter:

planner_type:=astar
planner_type:=gvd


---

## 6. Comparative Observations

### A*

- Shortest path length
- Lower time-to-goal
- Minimal clearance
- Sharp turns

### GVD

- Longer path
- Increased obstacle clearance
- Stable corridor traversal
- More conservative motion

The experiments confirm:

> Optimality and safety are competing objectives in mobile robot navigation.

---

## 7. Engineering Challenges Resolved

During development, several non-trivial system-level issues were addressed:

- ROS2 QoS mismatch between Gazebo and subscribers
- Lifecycle-based Map Server activation
- Path republishing causing controller reset loops
- Heading oscillation at high-curvature turns
- Synchronization of planner execution timing
- Frame alignment between `map`, `odom`, and `base_link`

These debugging phases significantly improved system robustness.

---

## 8. How to Run

### Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
Launch A*
```
ros2 launch gauntlet_sim gauntlet_nav.launch.py planner_type:=astar
```
Launch GVD
```
ros2 launch gauntlet_sim gauntlet_nav.launch.py planner_type:=gvd
```
Launch Trajectory Tracker 
```
ros2 run gauntlet_sim trajectory_publisher
```
Launch AMCL Based localisation
```
	 	ros2 run nav2_amcl amcl # then in other terminal you put next commands to activate amcL 

			ros2 lifecycle set /amcl configure


			ros2 lifecycle set /amcl activate


			ros2 lifecycle get /amcl

      # To give an initial reference to AMCL , we initialise amcl with a initial_pos and orientation , by the following command

  ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}" \
--once
```


9. Current Development Stage

Completed:

    * Simulation environment

    * SLAM-based map generation

    * Configuration space modeling

    * A* implementation

    * GVD implementation

    * Planner-specific controllers

    * Planner selection at runtime

10. Planned Future Extensions

The system will be extended with:

    Hardware deployment on physical TurtleBot

    AMCL-based localization for odometry drift correction

    Inflation radius tuning for real-world imperfections

    Controller parameter optimization under sensor noise

    Quantitative comparison:

        Path length

        Time to goal

        Minimum obstacle clearance

    Multiple-run statistical evaluation

    Sim-to-real gap analysis

11. Technologies Used

    * ROS2 (rclpy)

    * Gazebo

    * nav2_map_server

    * SLAM Toolbox

    * RViz2

    * Python

    * Occupancy Grid Mapping

    * Graph Search Algorithms

12. Author

Ayush Bhaskar
Autonomous Systems & Robotics
IIT Bombay
