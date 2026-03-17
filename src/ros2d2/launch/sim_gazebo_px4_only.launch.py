#!/usr/bin/env python3
"""
Launch Gazebo + 2 PX4 SITL instances + Micro-XRCE-DDS-Agent.

No game nodes. Use for M2 verification: two iris in Gazebo, PX4 running, Agent up.
Verify with: ros2 topic echo /px4_1/fmu/out/vehicle_local_position
             ros2 topic echo /px4_2/fmu/out/vehicle_local_position
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def get_px4_path():
    """Find PX4-Autopilot path. Prefer env var, else infer from workspace."""
    if os.environ.get("PX4_AUTOPILOT"):
        path = os.environ["PX4_AUTOPILOT"]
        if os.path.isdir(path):
            return path
    try:
        prefix = get_package_prefix("ros2d2")
        ws_root = os.path.abspath(os.path.join(prefix, "..", ".."))
        px4_path = os.path.join(ws_root, "src", "PX4-Autopilot")
        if os.path.isdir(px4_path):
            return px4_path
    except Exception:
        pass
    # Common dev container path
    for candidate in [
        "/workspaces/ros2_ws/src/PX4-Autopilot",
        os.path.join(os.path.expanduser("~"), "PX4-Autopilot"),
    ]:
        if os.path.isdir(candidate):
            return candidate
    return os.path.join(os.path.expanduser("~"), "PX4-Autopilot")


def generate_launch_description():
    px4_dir_arg = DeclareLaunchArgument(
        "px4_dir",
        default_value=get_px4_path(),
        description="Path to PX4-Autopilot directory",
    )

    px4_dir = LaunchConfiguration("px4_dir")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty",
        description="World name (must exist in PX4 sitl_gazebo-classic/worlds/)",
    )
    world = LaunchConfiguration("world")

    spawn_defender_x_arg = DeclareLaunchArgument(
        "spawn_defender_x", default_value="0.0",
        description="Defender (px4_1) spawn X (m, Gazebo world)",
    )
    spawn_defender_y_arg = DeclareLaunchArgument(
        "spawn_defender_y", default_value="3.0",
        description="Defender (px4_1) spawn Y (m, Gazebo world)",
    )
    spawn_attacker_x_arg = DeclareLaunchArgument(
        "spawn_attacker_x", default_value="0.0",
        description="Attacker (px4_2) spawn X (m, Gazebo world)",
    )
    spawn_attacker_y_arg = DeclareLaunchArgument(
        "spawn_attacker_y", default_value="0.0",
        description="Attacker (px4_2) spawn Y (m, Gazebo world)",
    )

    spawn_defender_x = LaunchConfiguration("spawn_defender_x")
    spawn_defender_y = LaunchConfiguration("spawn_defender_y")
    spawn_attacker_x = LaunchConfiguration("spawn_attacker_x")
    spawn_attacker_y = LaunchConfiguration("spawn_attacker_y")

    target = "px4_sitl_default"

    from launch.actions import OpaqueFunction

    def add_sim_actions(context):
        px4 = context.perform_substitution(px4_dir)
        w = context.perform_substitution(world)
        x1 = context.perform_substitution(spawn_defender_x)
        y1 = context.perform_substitution(spawn_defender_y)
        x2 = context.perform_substitution(spawn_attacker_x)
        y2 = context.perform_substitution(spawn_attacker_y)
        # Use -s for configurable spawn positions (iris:count:x:y)
        spawn_script = f"iris:1:{x1}:{y1},iris:1:{x2}:{y2}"
        cmd = (
            f"source {px4}/Tools/simulation/gazebo-classic/setup_gazebo.bash "
            f"{px4} {px4}/build/{target} && "
            f"{px4}/Tools/simulation/gazebo-classic/sitl_multiple_run.sh "
            f"-s '{spawn_script}' -m iris -w {w}"
        )
        return [
            ExecuteProcess(cmd=["bash", "-c", cmd], output="screen", shell=False),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
                        output="screen",
                    ),
                ],
            ),
        ]

    return LaunchDescription([
        px4_dir_arg,
        world_arg,
        spawn_defender_x_arg,
        spawn_defender_y_arg,
        spawn_attacker_x_arg,
        spawn_attacker_y_arg,
        OpaqueFunction(function=add_sim_actions),
    ])
