#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _rewrite_initial_pose_params(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    set_initial_pose = (
        LaunchConfiguration("set_initial_pose").perform(context).lower() == "true"
    )
    initial_pose_x = float(LaunchConfiguration("initial_pose_x").perform(context))
    initial_pose_y = float(LaunchConfiguration("initial_pose_y").perform(context))
    initial_pose_yaw = float(LaunchConfiguration("initial_pose_yaw").perform(context))

    with open(params_file, "r", encoding="utf-8") as params_handle:
        data = yaml.safe_load(params_handle)

    amcl_params = data.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl_params["set_initial_pose"] = bool(set_initial_pose)
    amcl_params["initial_pose"] = [initial_pose_x, initial_pose_y, initial_pose_yaw]

    tmp_handle = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix="nav2_params_", suffix=".yaml"
    )
    yaml.safe_dump(data, tmp_handle)
    tmp_handle.close()

    return [SetLaunchConfiguration("params_file", tmp_handle.name)]


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    slam = LaunchConfiguration("slam")
    use_composition = LaunchConfiguration("use_composition")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    default_params = os.path.join(
        get_package_share_directory("treebo_navigation"),
        "config",
        "nav2_params.yaml",
    )
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Full path to map yaml",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Full path to nav2 params",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Autostart Nav2",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="False",
                description="Use SLAM (slam_toolbox)",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description="Use composed bringup",
            ),
            DeclareLaunchArgument(
                "set_initial_pose",
                default_value="true",
                description="Set AMCL initial pose from parameters",
            ),
            DeclareLaunchArgument(
                "initial_pose_x",
                default_value="0.0",
                description="Initial pose x (map frame)",
            ),
            DeclareLaunchArgument(
                "initial_pose_y",
                default_value="0.0",
                description="Initial pose y (map frame)",
            ),
            DeclareLaunchArgument(
                "initial_pose_yaw",
                default_value="0.0",
                description="Initial pose yaw in radians",
            ),
            OpaqueFunction(function=_rewrite_initial_pose_params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "slam": slam,
                    "use_composition": use_composition,
                }.items(),
            ),
        ]
    )
