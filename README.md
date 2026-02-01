# COVAPSy Simulation Environment — Sorbonne Université 2025

This repository contains the **simulation environment for the COVAPSy course** (Design and Validation of Cyber-Physical Systems), offered at **Sorbonne Université in 2025**.
It provides a complete platform based on **Webots** and **ROS 2**, allowing to **design, test and visualize autonomous navigation algorithms** for a car-type vehicle.

---

##  Project Objective

The main objective of this project is to set up a **realistic autonomous car simulator**, close to the conditions encountered on a real vehicle.

Initially, the work consisted of:
- Setting up a **functional Webots + ROS 2 architecture**
- Modeling a **TT02** type vehicle with **Ackermann** kinematics
- Integrating several **realistic sensors**
- Properly publishing **ROS 2 topics**, **odometry** and **TF transformations**
- Preparing the simulator for:
  - autonomous navigation
  - obstacle avoidance
  - SLAM (mapping)
  - camera-based perception

This simulator serves as an **experimental base** for the practical sessions and projects of the COVAPSy course.

---

##  Simulator Description

The simulated vehicle is a **TT02 type car**, integrated in Webots with:
- realistic dynamics
- Ackermann steering
- direct interface with ROS 2 via a **custom Webots controller**

Each vehicle is **namespaced** (e.g., `/TT02_jaune/...`), which allows future extension to multi-robot scenarios.

---

##  Integrated Sensors

###  LiDAR
- **RpLidar A2**
- Used for obstacle avoidance and SLAM
- Topics:
  - `/TT02_jaune/RpLidarA2`
  - `/TT02_jaune/RpLidarA2/point_cloud`

###  RGB-D Camera (Qualcomm RB5 type)

A camera inspired by the **Qualcomm RB5 kit** has been integrated on the vehicle's roof.

- **RGB Camera**
  - `/TT02_jaune/rb5_rgb/image_color`
  - `/TT02_jaune/rb5_rgb/camera_info`

- **Depth Camera**
  - `/TT02_jaune/rb5_depth/image`
  - `/TT02_jaune/rb5_depth/point_cloud`

This camera enables work on:
- visual perception
- line following
- vision-based navigation
- future deep learning approaches

###  Other Sensors
- IMU / Gyroscope
- Rear sonar sensor

---

##  System Prerequisites

###  Operating System
- **Ubuntu 24.04 LTS (recommended)**

###  ROS 2
- **ROS 2 Jazzy**

Official installation:  
https://docs.ros.org/en/jazzy/Installation.html

## Install necessary tools:
```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git
```

---

##  Installation

### 1. Initialize rosdep
To be done only once before compilation:
```bash
sudo rosdep init
rosdep update
```

### 2. Install additional ROS 2 dependencies
Make sure you are running ROS 2 Jazzy.
```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt-image-view \
  ros-jazzy-tf2-tools
```

### 3. Create workspace and clone
```bash
# Create src directory
mkdir -p ~/covapsy_ws/src
# Navigate to directory and clone repository
cd ~/covapsy_ws/src
git clone https://github.com/Ferras007/covapsy_simulation.git
```

### 4. Install project dependencies
```bash
cd ~/covapsy_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Compilation
```bash
cd ~/covapsy_ws
colcon build
# Source the environment
source install/setup.bash
```

⚠️ Sourcing (`source install/setup.bash`) must be redone in each new terminal.

##  Launch the Simulator

Open a terminal and execute:
```bash
source /opt/ros/jazzy/setup.bash
source ~/covapsy_ws/install/setup.bash
ros2 launch webot_simulation simulation.launch.py
```

## Visualization:

### RGB Camera (RQT):
To view the video stream:
```bash
ros2 run rqt_image_view rqt_image_view
```
