# COVAPSy Simulation Environment — Sorbonne Université 2025

This repository contains the simulation component of the **COVAPSy** course (Sorbonne Université, 2025).
It provides a **Webots + ROS 2** simulation environment to build, test and visualize autonomous vehicle control.

## Repository structure

- `simulation_ros2/webot_simulation/` : ROS 2 package (launch files, Webots world/protos/controllers, etc.)
- `README.md` : setup & run instructions

---

## Requirements

### OS
- **Ubuntu 24.04** recommended

### ROS 2
- **ROS 2 Jazzy** (recommended for Ubuntu 24.04)

### Tools / dependencies
- `colcon` (build tool)
- `git`
- **Webots** (R2023b/R2024a depending on your setup)

---

## Installation

### 1) Install ROS 2 Jazzy + colcon
Follow the official ROS 2 installation guide, then install colcon:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep git
