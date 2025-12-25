#!/usr/bin/env python

# Code taken inspired from 

"""Launch Webots car."""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('webot_simulation')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode') 

    # ------------------------------------------ #
    # Webots Launcher
    # Goal: Launch Webots with the specified world and mode
    # ------------------------------------------ #
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    # ------------------------------------------ #
    # Load the URDF file
    # Goal: Load the URDF file for the robot (used differently in Webots compared to Gazebo/RViz)
    # Details: Refer to the README for more information
    # ------------------------------------------ #
    robot_description_path = os.path.join(package_dir, 'resource', 'voiture_webots.urdf')

    # ------------------------------------------ #
    # Webots Controllers
    # Goal: Control the robots in the Webots world
    # Note: Ensure every robot in the world file has a corresponding WebotsController
    # ------------------------------------------ #
    TT02_violette_driver = WebotsController(
        robot_name='TT02_violette',
        parameters=[
            {'robot_description': robot_description_path}
        ]
    )

    TT02_jaune_driver = WebotsController(
        robot_name='TT02_jaune',
        parameters=[
            {'robot_description': robot_description_path}
        ]
    )

    TT02_bleue_driver = WebotsController(
        robot_name='TT02_bleue',
        parameters=[
            {'robot_description': robot_description_path}
        ]
    )

    # ------------------------------------------ #
    # Robot State Publisher (Optional)
    # Goal: Publish the robot's state (not needed for this simulation)
    # ------------------------------------------ #
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': '<robot name=""><link name=""/></robot>'
    #     }],
    # )
    

    # footprint_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    # )
    # ------------------------------------------ #


    # -----------------------------#
    #     OBSTACLE AVOIDER TEST    #
    # -----------------------------#
    # If needed to test the obstacle_avoider uncomment this part
    # obstacle_avoider = Node(
    #     package='webot_simulation',
    #     executable='obstacle_avoider',
    # )

    # -----------------------------#
    #     ROS CONTROL (OPTIONAL)   #
    # -----------------------------#
    # We do not need to use the ROS control spawners because we do not use ros_control in this simulation.
    # ros_control is used to control a robot that is defined with differential drive like a TurtleBot.
    # We are using a car defined with Ackermann dynamics, which is simpler.
    
    # However, if we need to expose the same topics as in real life (e.g. /cmd_vel and /cmd_dir),
    # we would need to implement a DifferentialDrive and use ros_control to control the back wheels
    # using the /cmd_vel topic.

    # controller_manager_timeout = ['--controller-manager-timeout', '50']
    # controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    # diffdrive_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     prefix=controller_manager_prefix,
    #     arguments=['diffdrive_controller'] + controller_manager_timeout,
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     prefix=controller_manager_prefix,
    #     arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    # )

    # ros_control_spawners = [
    #     diffdrive_controller_spawner,
    #     joint_state_broadcaster_spawner
    # ]

    # robot_description_path = os.path.join(package_dir, 'resource', 'voiture_webots.urdf')
    # ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')

    # use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    # if use_twist_stamped:
    #     mappings = [
    #         ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
    #         ('/diffdrive_controller/odom', '/odom')
    #     ]
    # else:
    #     mappings = [
    #         ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
    #         ('/diffdrive_controller/odom', '/odom')
    #     ]

    # voiture_driver = WebotsController(
    #     robot_name='bolide',
    #     parameters=[
    #         {'robot_description': robot_description_path,
    #          'use_sim_time': use_sim_time,
    #          'set_robot_state_publisher': True},
    #         # ros2_control_params
    #     ],
    #     remappings=mappings,
    #     respawn=True
    # )

    # -----------------------------#
    #        NAVIGATION & SLAM     #
    # -----------------------------#
    # Uncomment the following lines to use the Nav and SLAM
    # Make sure to add the required YAML and PGM map files to the resource folder.

    # navigation_nodes = []
    # os.environ['TURTLEBOT3_MODEL'] = 'burger'
    # nav2_map = os.path.join(package_dir, 'resource', 'turtlebot3_burger_example_map.yaml')
    # nav2_params = os.path.join(package_dir, 'resource', 'nav2_params.yaml')

    # if 'turtlebot3_navigation2' in get_packages_with_prefixes():
    #     turtlebot_navigation = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             get_package_share_directory('turtlebot3_navigation2'),
    #             'launch', 'navigation2.launch.py')),
    #         launch_arguments=[
    #             ('map', nav2_map),
    #             ('params_file', nav2_params),
    #             ('use_sim_time', use_sim_time),
    #         ],
    #         condition=launch.conditions.IfCondition(use_nav)
    #     )
    #     navigation_nodes.append(turtlebot_navigation)

    # if 'turtlebot3_cartographer' in get_packages_with_prefixes():
    #     turtlebot_slam = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             get_package_share_directory('turtlebot3_cartographer'),
    #             'launch', 'cartographer.launch.py')),
    #         launch_arguments=[
    #             ('use_sim_time', use_sim_time),
    #         ],
    #         condition=launch.conditions.IfCondition(use_slam)
    #     )
    #     navigation_nodes.append(turtlebot_slam)

    # -----------------------------#
    #   WAIT FOR SIM INITIALIZATION
    # -----------------------------#
    # Wait for the simulation to be ready to start navigation nodes
    # waiting_nodes = WaitForControllerConnection(
    #     target_driver=turtlebot_driver,
    #     nodes_to_start=navigation_nodes + ros_control_spawners
    # )


    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='piste_ens2025.wbt',
            description='Choose one of the world files from `/webot_simulation/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,
        ## If needed to test the obstacle_avoider uncomment this part
        #obstacle_avoider,

        # robot_state_publisher,
        # footprint_publisher,
        TT02_violette_driver,
        TT02_jaune_driver,
        TT02_bleue_driver,
        # voiture_driver,
        # turtlebot_driver,
        # waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit( 
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])