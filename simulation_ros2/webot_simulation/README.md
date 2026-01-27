# webot_simulation – Autonomous Navigation (Depth + LiDAR)

## Overview
Autonomous navigation for an Ackermann vehicle using:
- **Depth camera** for local perception (left/center/right distances).
- **LiDAR** for safety (emergency stop only in straight motion to avoid false positives in turns).

Main node: `vision_lane_follower.py`

## Topics
### Subscribed
- `/TT02_jaune/rb5_depth/image` (sensor_msgs/Image)
- `/TT02_jaune/RpLidarA2` (sensor_msgs/LaserScan)

### Published
- `/cmd_ackermann` (ackermann_msgs/AckermannDrive)

## Build
From your ROS 2 workspace:
```bash
cd ~/covapsy_ws
colcon build
source install/setup.bash

