#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
        description="Autostart Nav2",
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

    node_common = {
        "output": "screen",
        "respawn": use_respawn,
        "respawn_delay": 2.0,
        "parameters": [params_file, {"use_sim_time": use_sim_time}],
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        **node_common,
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        remappings=remappings,
        **node_common,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        remappings=remappings,
        **node_common,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        **node_common,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        remappings=remappings,
        **node_common,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        remappings=remappings,
        **node_common,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        remappings=remappings
        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
        **node_common,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ]
            },
        ],
    )

    return LaunchDescription(
        [
            declare_params_file,
            declare_use_sim_time,
            declare_autostart,
            declare_use_respawn,
            declare_log_level,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager,
        ]
    )
