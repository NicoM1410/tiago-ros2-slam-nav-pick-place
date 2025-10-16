# TIAGo ROS2 – SLAM, Navigation & Pick&Place

This repository presents the implementation of **autonomous and mobile robotics** functionalities on the **TIAGO robot** using **ROS2 Humble (Ubuntu 22.04)**.  
The system integrates **SLAM**, **autonomous navigation (Nav2)**, **object detection via ArUco markers**, **manipulation with MoveIt**, and a **Pick & Place pipeline**, all coordinated through a **finite state machine** written in Python.

This project was developed as part of the *Autonomous and Mobile Robotics* course at the **University of Bologna**.

---

## Abstract

The goal of this work is to design and implement a complete robotic architecture that enables a TIAGo mobile manipulator to autonomously explore, localize, and interact with its environment.

The robot:
- Builds a map of the environment using **SLAM Toolbox**
- Performs **localization** and **path planning** via **Nav2**
- Detects **ArUco markers** for target identification
- Executes **Pick & Place** operations using **MoveIt**
- Coordinates all subsystems through a **state machine** ensuring modularity and synchronization

This architecture demonstrates the integration of perception, planning, and control in a unified robotic system capable of executing complex tasks in a simulated environment.

---

## System Overview

### 1. SLAM and Localization
- Implemented with `slam_toolbox` for online asynchronous mapping and AMCL-based localization.  
- The robot generates and updates a 2D occupancy grid map (`map.pgm`, `map.yaml`) in real time.

### 2. Navigation (Nav2)
- Autonomous path planning and obstacle avoidance.  
- Configured using custom parameters for costmaps and motion controllers.  
- Launch file: `tiago_navigation/tiago_2dnav/launch/tiago_nav_bringup.launch.py`.

### 3. Manipulation (MoveIt)
- Motion planning for TIAGo’s 7-DOF arm.  
- Used to execute the Pick & Place sequence once the target marker is detected.

### 4. Object Detection (ArUco)
- Managed by `aruco_manager` package, containing:
  - `aruco_broadcaster.py` – TF broadcasting of detected markers  
  - `attacher.py` – Virtual grasping in Gazebo  
  - `gripper.py`, `arm_pose.py`, `torso.py` – Motion primitives for manipulation  
- Detection of markers with IDs **63** and **582** used for target identification.

### 5. State Machine
- Located in `state_machine/launch/state_machine.launch.py`  
- Coordinates all nodes: SLAM, navigation, perception, and manipulation.  
- Ensures proper task synchronization and transition management.

### 6. Autonomous Localization
- Handles initial pose estimation and covariance thresholds before starting navigation.

### 7. Simulation Environment
- Gazebo world configured in `tiago_simulation/tiago_gazebo/launch/tiago_gazebo.launch.py`.  
- Full robot simulation with physics, sensors, and controllers integrated.

---

## Technical Specifications

| Component              | Description                                    |
|-------------------------|------------------------------------------------|
| **Robot**              | TIAGo mobile manipulator                       |
| **Simulation**         | Gazebo 11 / ROS2 Humble                        |
| **Mapping**            | SLAM Toolbox                                   |
| **Localization**       | AMCL                                            |
| **Navigation**         | Nav2 (planner, controller, recovery server)     |
| **Manipulation**       | MoveIt2                                        |
| **Marker Detection**   | OpenCV + ArUco                                 |
| **Coordination**       | Python-based finite state machine              |
| **OS / Middleware**    | Ubuntu 22.04 + ROS2 Humble                     |
| **Programming Languages** | Python, C++                                 |

---

## Results

- Successful integration of multiple ROS2 subsystems (SLAM, Nav2, MoveIt, ArUco)
- Generated consistent and accurate maps via SLAM
- Robust navigation with real-time obstacle avoidance
- Accurate localization using AMCL
- Stable Pick & Place sequence with ArUco-based pose estimation
- Full coordination through modular state machine architecture

---

## Repository Content

```
tiago-ros2-slam-nav-pick-place/
├── tiago_ws/ # Custom ROS2 workspace (controllers, behaviors, perception)
│ ├── src/
│ │ ├── state_machine/
│ │ ├── aruco_manager/
│ │ ├── navigation/
│ │ ├── autonomous_localization/
│ │ └── ...
│ ├── map.pgm
│ └── map.yaml
│
├── ros2_ws/ # Base ROS2 workspace (dependencies, ros-controls)
│
├── AMR_presentation.pdf # Final project presentation
└── README.md # Project documentation
```

---

## How to Run the Simulation

### 1. Clone the repository
```
git clone https://github.com/NicoM1410/tiago-ros2-slam-nav-pick-place.git
cd tiago-ros2-slam-nav-pick-place/tiago_ws
```

### 2. Build the workspace
```
colcon build 
source install/setup.bash
```

### 3. Install dependencies (if not already present)
```
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
                 ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-aruco-ros
```

### 4. Launch Gazebo simulation
```
ros2 launch tiago_gazebo tiago_world.launch.py
```

### 5. Start SLAM mapping
```
ros2 launch slam_toolbox online_async_launch.py
```

### 6. Run navigation stack
```
ros2 launch nav2_bringup navigation_launch.py
```

### 7. Execute Pick & Place
```
ros2 run tiago_pick_place pick_place_node
```

---

## Presentation

The final project presentation is available here:  
[Download AMR Project Presentation (PDF)](./AMR%20presentation%20(1).pdf)

---

## Future Work

- Improve scalability of the state machine for multi-task scenarios
- Optimize computational load during simultaneous SLAM and manipulation
- Extend to real-world deployment using TIAGo hardware
- Integrate vision-based grasp planning for dynamic environments

---

## Authors

- **Niccolò Mazzocchi** – University of Bologna  
- **Silvia Cagnolati** – University of Bologna  
- **Daniele Crivellari** – University of Bologna  

**Supervised by:** Prof. *Gianluca Palli* and *Alessio Caporali*

---

## License

This project is released for academic and educational purposes.
Feel free to explore, adapt, and extend.

---
