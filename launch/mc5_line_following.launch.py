#!/usr/bin/env python3
"""
Launch file MC5: line following + traffic light integration.

The camera pipeline must be running before launching this:
  ros2 launch ros_deep_learning video_source.ros2.launch

This launch starts:
  - odometry_node       (for logging the robot pose during the run)
  - cmd_vel_bridge      (translates /cmd_vel to /VelocitySetL, /VelocitySetR)
  - controller          (mode = line_following, configured by mc5 YAML)
  - line_detector       (publishes /line_error and /line_detected)
  - traffic_light_detector (publishes /semaphore_state)

For Part 1 of MC5 (line following alone), set enable_semaphore: false
in controller_params_mc5.yaml. The traffic light detector still runs
but the controller ignores its output.

For Part 2 (line + semaphore), set enable_semaphore: true.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('puzzlebot_mc2')
    robot_params = os.path.join(pkg, 'config', 'robot_params.yaml')
    ctrl_params = os.path.join(pkg, 'config', 'controller_params_mc5.yaml')
    line_params = os.path.join(pkg, 'config', 'line_detector_params.yaml')
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
            executable='line_detector.py',
            name='line_detector',
            output='screen',
            parameters=[line_params],
        ),
        Node(
            package='puzzlebot_mc2',
            executable='traffic_light_detector.py',
            name='traffic_light_detector',
            output='screen',
            parameters=[vision_params],
        ),
    ])
