#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # --- 공통 Launch Argument ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    nav_share_dir = get_package_share_directory("treebo_navigation")
    default_slam_params = os.path.join(
        nav_share_dir, "config", "slam_toolbox_mapping.yaml"
    )

    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_slam_params,
        description="Full path to the slam_toolbox mapping parameter file",
    )

    # --- slam_toolbox에서 제공하는 공식 launch 포함 ---
    slam_share_dir = get_package_share_directory("slam_toolbox")
    online_sync_launch = os.path.join(
        slam_share_dir, "launch", "online_sync_launch.py"
    )

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(online_sync_launch),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_slam_params_file,
            slam_include,
        ]
    )
