#!/usr/bin/env python3
"""
Launch file MC4: navegacion + vision integradas.

NO incluye el pipeline de la camara (ros_deep_learning).  Eso debe estar
corriendo en otra terminal antes de lanzar esto:

  ros2 launch ros_deep_learning video_source.ros2.launch

Este launch arranca:
  - odometry_node
  - cmd_vel_bridge
  - controller (con enable_semaphore: true)
  - path_generator (con 6 waypoints del MC4)
  - traffic_light_detector
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_mc2')
    robot_params = os.path.join(pkg, 'config', 'robot_params.yaml')
    ctrl_params = os.path.join(pkg, 'config', 'controller_params_mc4.yaml')
    wp_params = os.path.join(pkg, 'config', 'waypoints_mc4.yaml')
    vision_params = os.path.join(pkg, 'config', 'vision_params.yaml')

    return LaunchDescription([
        Node(
            package='puzzlebot_mc2',
            executable='odometry_node.py',
            name='odometry_node',
            output='screen',
            parameters=[robot_params],
        ),
        Node(
            package='puzzlebot_mc2',
            executable='cmd_vel_bridge.py',
            name='cmd_vel_bridge',
            output='screen',
            parameters=[robot_params],
        ),
        Node(
            package='puzzlebot_mc2',
            executable='controller.py',
            name='controller',
            output='screen',
            parameters=[ctrl_params],
        ),
        Node(
            package='puzzlebot_mc2',
            executable='path_generator.py',
            name='path_generator',
            output='screen',
            parameters=[wp_params],
        ),
        Node(
            package='puzzlebot_mc2',
            executable='traffic_light_detector.py',
            name='traffic_light_detector',
            output='screen',
            parameters=[vision_params],
        ),
    ])
