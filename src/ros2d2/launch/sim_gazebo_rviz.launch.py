#!/usr/bin/env python3
"""
Launch Gazebo + 2 PX4 SITL + Micro-XRCE-Agent + static TF + RViz.

Simulation and visualization only. Run sim_drones.launch.py separately for game logic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz for drone markers (blue=D, red=A)",
    )
    spawn_defender_x_arg = DeclareLaunchArgument(
        "spawn_defender_x", default_value="0.0",
        description="Defender (px4_1) spawn X (m)",
    )
    spawn_defender_y_arg = DeclareLaunchArgument(
        "spawn_defender_y", default_value="3.0",
        description="Defender (px4_1) spawn Y (m)",
    )
    spawn_attacker_x_arg = DeclareLaunchArgument(
        "spawn_attacker_x", default_value="0.0",
        description="Attacker (px4_2) spawn X (m)",
    )
    spawn_attacker_y_arg = DeclareLaunchArgument(
        "spawn_attacker_y", default_value="0.0",
        description="Attacker (px4_2) spawn Y (m)",
    )

    # Gazebo + PX4 + Agent (pass spawn args through)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros2d2"), "launch", "sim_gazebo_px4_only.launch.py"])
        ),
        launch_arguments=[
            ("spawn_defender_x", LaunchConfiguration("spawn_defender_x")),
            ("spawn_defender_y", LaunchConfiguration("spawn_defender_y")),
            ("spawn_attacker_x", LaunchConfiguration("spawn_attacker_x")),
            ("spawn_attacker_y", LaunchConfiguration("spawn_attacker_y")),
        ],
    )

    # Static TF: world frame for RViz markers
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        name="world_to_map_tf",
    )

    # Drone markers: publishes to /game/drone_markers (shows "Waiting..." until sim_drones runs)
    drone_markers = Node(
        package="ros2d2",
        executable="drone_markers_node",
        name="drone_markers_node",
        output="screen",
    )

    # RViz with drone markers config
    rviz_config = os.path.join(
        get_package_share_directory("ros2d2"), "config", "drone_markers.rviz"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        use_rviz_arg,
        spawn_defender_x_arg,
        spawn_defender_y_arg,
        spawn_attacker_x_arg,
        spawn_attacker_y_arg,
        gazebo_launch,
        static_tf,
        drone_markers,
        rviz2_node,
    ])
