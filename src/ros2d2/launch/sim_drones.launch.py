#!/usr/bin/env python3
"""
Launch game nodes (drone run): state fusion, controller, bridges, capture detection.

Assumes Gazebo + 2 PX4 + Agent already running (e.g. sim_gazebo_rviz.launch.py).
Bridge nodes use internal readiness logic; no launch-time delay required.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_mode_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="normal",
        description="Controller mode: 'normal' (pursuit) or 'test_scripted' (independent motion test)",
    )

    state_fusion = Node(
        package="ros2d2",
        executable="state_fusion_node",
        name="state_fusion_node",
        output="screen",
    )

    game_controller = Node(
        package="ros2d2",
        executable="game_controller_node",
        name="game_controller_node",
        output="screen",
        parameters=[{"control_mode": LaunchConfiguration("control_mode")}],
    )

    capture_detection = Node(
        package="ros2d2",
        executable="capture_detection_node",
        name="capture_detection_node",
        output="screen",
    )

    px4_bridge_defender = Node(
        package="ros2d2",
        executable="px4_bridge_defender_node",
        name="px4_bridge_defender_node",
        output="screen",
    )

    px4_bridge_attacker = Node(
        package="ros2d2",
        executable="px4_bridge_attacker_node",
        name="px4_bridge_attacker_node",
        output="screen",
    )

    return LaunchDescription([
        control_mode_arg,
        state_fusion,
        game_controller,
        capture_detection,
        px4_bridge_defender,
        px4_bridge_attacker,
    ])
