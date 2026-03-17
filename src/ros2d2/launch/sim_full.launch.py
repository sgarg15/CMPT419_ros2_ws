#!/usr/bin/env python3
"""
Full-stack launch: Gazebo + 2 PX4 + Agent + game nodes + markers + RViz.

One command to run everything. All nodes start immediately; bridge nodes wait
internally for PX4 state/position validity before arming and commanding motion.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("ros2d2")

    control_mode_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="normal",
        description="Controller mode: 'normal' or 'test_scripted'",
    )

    gazebo_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "sim_gazebo_rviz.launch.py"])
        ),
    )

    drones_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "sim_drones.launch.py"])
        ),
        launch_arguments=[("control_mode", LaunchConfiguration("control_mode"))],
    )

    return LaunchDescription([
        control_mode_arg,
        gazebo_rviz_launch,
        drones_launch,
    ])
