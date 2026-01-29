#  Autonomous Navigation – LiDAR + Depth Camera (ROS 2)

##  Description

This project implements a **robust autonomous navigation** system based on the **fusion of a depth camera and a LiDAR**, developed and tested under **ROS 2 with Webots**.

The goal is to enable an autonomous vehicle to:

- **follow a track**,
- **anticipate turns** thanks to the depth camera,
- **avoid obstacles** with the LiDAR,
- while ensuring **smart safety** (emergency stop only on straight paths).

The behavior is optimized to be:

- smooth,
- stable,
- without oscillations,
- and able to **complete the entire circuit**.

---

##  Directory Structure (excerpt)

```text
webot_simulation/
├── obstacle_avoider.py
├── teleop.py
├── vision_lane_follower.py
├── voiture_driver.py
├── __init__.py
└── README.md

```
# Vision Lane Follower

##  Main file
**`vision_lane_follower.py`**

##  Dependencies
- **ROS 2** ( Jazzy)
- **Webots**
- **Python 3**
- **ROS packages** : 
  - `rclpy`
  - `sensor_msgs`
  - `ackermann_msgs`
  - `cv_bridge`
  - `numpy`

##  Workspace build
From the root of the ROS 2 Workspace :
```bash
cd ~/covapsy_ws
colcon build
source install/setup.bash
```
##  Launching the simulation
**Launch Webots** (according to the usual setup) after 
**Launch autonomous navigation :**
```bash
ros2 run webot_simulation vision_lane_follower.py
```
##  Operating principle

### 1️ Depth camera – Local anticipation
The depth image is divided into three zones: left, center, and right. For each zone, the median distance is computed.

**This allows to :**
- anticipate turns
- start turning before facing a wall

### 2️ Steering control
The system uses two decision levels :

**Soft anticipation :**

```python
steering = -k_side * (depth_left - depth_right)
```
#### Strong decision in the presence of an obstacle

When the distance measured in front of the vehicle becomes lower than a predefined threshold, a more aggressive decision is applied to avoid a collision :

```python
if depth_center < depth_threshold:
    if depth_left > depth_right:
        steering = -steering_gain
    else:
        steering = +steering_gain
```
An **exponential smoothing** is then applied to the steering command to avoid oscillations and abrupt changes :

```python
steering = steer_smooth * prev_steering + (1.0 - steer_smooth) * steering
```

### 3️ LiDAR – Smart safety

The LiDAR is used as a safety sensor to prevent frontal collisions.
  
It monitors a **narrow frontal zone** (±10°) in front of the vehicle.

The emergency stop is triggered **only if** :
- an obstacle is detected at a very short distance,
- **and** the vehicle is almost going straight (not turning).

```python
if lidar_front < lidar_stop_dist and abs(steering) < 0.12:
    lidar_stop_counter += 1
else:
    lidar_stop_counter = 0
```
After several consecutive detections, the speed is canceled :

```python
if lidar_stop_counter >= lidar_stop_count_req:
    speed = 0.0
```
This logic helps avoid false stops caused by side walls in turns.

### 4️ Speed management

The vehicle speed is managed adaptively :
- nominal speed on straight lines,
- automatic slowdown in turns,
- stop only in case of real danger.
```python
speed = max_speed

if abs(steering) > 0.1 and speed > 0.0:
    speed *= turn_slow_factor

```
This mechanism ensures a good compromise between speed and stability.

### 5️ Command publishing

Final speed and steering commands are sent to the vehicle using an AckermannDrive message:
```python
cmd = AckermannDrive()
cmd.speed = speed
cmd.steering_angle = steering
cmd_pub.publish(cmd)

```
###  Main parameters
```python
depth_threshold = 1.5
k_side = 0.20

steering_gain = 0.35
steer_smooth = 0.7
max_steer = 0.45

lidar_stop_dist = 0.28
lidar_stop_count_req = 3

max_speed = 0.30
turn_slow_factor = 0.6

```
These parameters were tuned to achieve a good compromise between responsiveness, stability, and safety.

###  Results

- Smooth and stable navigation
- Clean turns without oscillations
- No false emergency stops
- Circuit completed entirely





