#!/usr/bin/env python3
"""Launch file — MC2 Parte 1: cuadrado de 2 m."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_mc2')
    robot_params = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    ctrl_params = os.path.join(pkg_share, 'config', 'controller_params.yaml')
    wp_params = os.path.join(pkg_share, 'config', 'waypoints_square.yaml')

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
    ])
