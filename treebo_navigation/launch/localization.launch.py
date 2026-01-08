#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    default_params = os.path.join(
        get_package_share_directory("treebo_navigation"),
        "config",
        "nav2_params.yaml",
    )

    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to nav2 params",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use sim time",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Autostart localization",
    )
    declare_use_respawn = DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Respawn nodes if they crash",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            params_file,
            {"yaml_filename": map_yaml},
            {"use_sim_time": use_sim_time},
        ],
        remappings=remappings,
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    return LaunchDescription(
        [
            declare_map,
            declare_params_file,
            declare_use_sim_time,
            declare_autostart,
            declare_use_respawn,
            declare_log_level,
            map_server,
            amcl,
            lifecycle_manager,
        ]
    )
