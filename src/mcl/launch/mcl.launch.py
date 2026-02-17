import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

DEFAULT_PARAMS_FILES = {
    "mcl_params": os.path.join(
        get_package_share_directory("mcl"),
        "params",
        "default_mcl_params.yaml",
    ),
    "rviz_settings": os.path.join(
        get_package_share_directory("mcl"),
        "rviz",
        "default_mcl_settings.rviz",
    ),
    "map_file": os.path.join(
        get_package_share_directory("nav2_bringup"),
        "maps",
        "turtlebot3_world.yaml",
    ),
}


def launch_setup(context, *args, **kwargs):
    mcl_params_file = LaunchConfiguration("mcl_params_file")
    run_rviz = LaunchConfiguration("run_rviz")
    rviz_settings_file = LaunchConfiguration("rviz_settings_file").perform(context)

    map_file = LaunchConfiguration("map_file").perform(context)

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}],
    )
    lifecycle_nodes = ["map_server"]

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
        ],
    )

    tb3_sim_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("turtlebot3_gazebo"),
                    "launch",
                    "turtlebot3_world.launch.py",
                )
            ]
        ),
    )

    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        parameters=[
            {"use_sim_time": True},
        ],
        arguments=["-d" + rviz_settings_file],
        output={"both": "log"},
    )

    mcl_node = Node(
        package="mcl",
        executable="mcl",
        parameters=[mcl_params_file],
    )

    return [
        map_server_node,
        lifecycle_manager_node,
        tb3_sim_launch,
        rviz_node,
        mcl_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mcl_params_file",
                default_value=DEFAULT_PARAMS_FILES["mcl_params"],
                description="MCL parameters file to use",
            ),
            DeclareLaunchArgument(
                "run_rviz",
                default_value="True",
                description="Whether to use rviz",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "rviz_settings_file",
                default_value=DEFAULT_PARAMS_FILES["rviz_settings"],
                description="Rviz settings file to use",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=DEFAULT_PARAMS_FILES["map_file"],
                description="Map file to use",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
