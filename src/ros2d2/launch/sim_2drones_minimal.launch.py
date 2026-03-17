#!/usr/bin/env python3
"""
Alias for sim_drones.launch.py. Launches game nodes (assumes Gazebo already running).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros2d2"), "launch", "sim_drones.launch.py"
                ])
            ),
        ),
    ])
