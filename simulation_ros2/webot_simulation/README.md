# Webots Simulation for Bolide Project

This repository provides a simulation environment using Webots and ROS 2 for controlling a TT02-based car model with various sensors and controllers.

## Installation

Follow the Webots installation procedure for Linux here: [Webots Installation Guide](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Make sure to install the `webots_ros2` stack:

```bash
sudo apt install ros-${ROS_DISTRO}-webots-ros2
```

## Project Structure

```
webots_simulation/
│
├── launch/                    # ROS 2 launch file for starting Webots
│   └── webot_launch.py        # Change the world name here if needed
│
├── protos/                    # Custom robot & sensor PROTO files
│
├── resource/                  
│   ├── voiture_webots.urdf    # URDF containing <plugin> for driver binding
│   └── ros2control.yml
│
├── test/webot_simulation/     # ROS 2 nodes
│   ├── obstacle_avoider.py
│   ├── teleop.py
│   └── voiture_driver.py      # Main driver communicating with Webots
│
├── worlds/                    # Add your .wbt world files here
│   └── piste_enscopy.wbt      # Default world
│
├── setup.py                   # Register new files (e.g. .wbt, .proto, .stl) here
└── ...
```

## Adding a New World

To add a custom world:

1. Place the `.wbt` file inside `webots_simulation/worlds/`.
2. Register it in `setup.py` under `data_files`, for example:

```python
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/my_new_world.wbt',
]))
```

3. Update the default world in `webot_launch.py`:

```python
DeclareLaunchArgument(
    'world',
    default_value='my_new_world.wbt',  # Change this
    description='Choose one of the world files from `/webots_simulation/worlds`'
)
```

## Changing the Controller

If you write a new driver (controller) for interfacing ROS 2 with Webots, ensure the following:

1. Set the plugin path in the URDF like this:

```xml
<plugin type="my_package.my_robot_driver.MyRobotDriver" />
```



## Sensors Included

Currently integrated sensors on the robot:

- RpLidar A2 → publishes to `/TT02_jaune/RpLidarA2/point_cloud`
- IMU via InertialUnit and Gyro
- Sonar (rear) → Webots DistanceSensor (type sonar)

These were inspired by:

- [webots_ros2_tesla](https://github.com/cyberbotics/webots_ros2_tesla)
- [webots_ros2_turtlebot](https://github.com/cyberbotics/webots_ros2_turtlebot)

## Debugging Notes

If `/cmd_vel` or `/cmd_ackermann` does not appear, ensure your controller is correctly launched and its plugin path is registered in the URDF.

Webots `<extern>` controllers must match the `robot_name` in the `WebotsController()` block in the launch file.

Rebuild the project if necessary:

```bash
colcon build && source install/setup.bash
```

## Questions

For ROS 2–Webots integration questions, refer to the [webots_ros2 GitHub repository](https://github.com/cyberbotics/webots_ros2).